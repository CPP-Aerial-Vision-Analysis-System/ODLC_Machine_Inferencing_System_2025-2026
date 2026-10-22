"""
sim_demo/test/ros_e2e/fake_mission.py -- PURPOSE: end-to-end test of all four sim_demo nodes
WITHOUT Gazebo or SITL. It plays a complete search mission (about 90 s) over a test world:
  * publishes what MAVROS would (/mavros/state, global_position/global + rel_alt + compass_hdg,
    vfr_hud, mission/reached, statustext/recv) and what the team's nodes would say on
    /mavros/statustext/send ("Camera trigger STARTED", "Targets: tent@9 -> person@10.
    Diverting.", "Servo 13 -> 1400", ...);
  * publishes a live camera picture on /sim/camera/image, rendered from a painted world
    (synthetic_world.py) at the drone's pose;
  * "takes" a photo every second along a search lane: renders it at 2560x1440, publishes the
    emulator's /sim/camera/photo_info record, and 1 s later writes it into the photo folder as
    "<lat> , <lon>.jpg" (exactly like video_cam: name chosen from the GPS at the shutter,
    tmp file + rename);
  * listens to /image_detection and runs the TEAM'S OWN main_controller.image_result_cb on it
    (team_acceptance.py), and to /sim/demo/markers.
At the end it prints a JSON summary and exits 0 only if: every photo was reported, the team's
callback chose a person AND a tent, and the RViz markers contain all expected layers.

Run (with truth_detector, demo_markers, score_run, demo_panel running):
    python3 fake_mission.py --world testfield --worlds-dir ../fixtures/worlds --photo-dir DIR
"""
from __future__ import annotations

import argparse
import json
import math
import os
import queue
import sys
import threading
import time
from datetime import datetime, timezone

import rclpy
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from sim_demo.camera_model import CameraIntrinsics, CameraPose, camera_axes  # noqa: E402
from sim_demo.photos import gps_photo_name  # noqa: E402
from sim_demo.world_info import load_world  # noqa: E402
from synthetic_world import SyntheticGround  # noqa: E402
from team_acceptance import TeamCallbackRunner  # noqa: E402

PHOTO_INTR = CameraIntrinsics(2560, 1440, 1.4137, 1.0)
VIDEO_INTR = CameraIntrinsics(1280, 720, 1.4137, 1.0)
ALT = 30.0


def quat_from_axes(x, y, z):
    """(x, y, z, w) quaternion of the rotation whose columns are the given axes."""
    m = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)


class FlightPath:
    """Piecewise-linear scripted flight: (t, east, north, up) knots, heading from motion."""

    def __init__(self, tent, hiker):
        k = [(0.0, 0, 0, 0.2), (3.5, 0, 0, 0.2), (9.5, 0, 0, ALT), (14.5, -10, 40, ALT),
             (38.5, 110, 40, ALT), (42.0, 110, 40, ALT), (47.0, tent.east, tent.north, ALT),
             (50.5, tent.east, tent.north, ALT), (55.5, hiker.east, hiker.north, ALT),
             (59.0, hiker.east, hiker.north, ALT), (65.5, 0, 0, ALT), (75.5, 0, 0, 0.2),
             (999.0, 0, 0, 0.2)]
        self.knots = k
        self.heading = 90.0

    def at(self, t):
        """(east, north, up, heading_deg, ground_speed) at time t."""
        k = self.knots
        for (t0, e0, n0, u0), (t1, e1, n1, u1) in zip(k, k[1:]):
            if t0 <= t <= t1:
                f = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                de, dn = e1 - e0, n1 - n0
                speed = math.hypot(de, dn) / max(1e-6, t1 - t0)
                if speed > 0.2:
                    self.heading = math.degrees(math.atan2(de, dn)) % 360.0
                return e0 + de * f, n0 + dn * f, u0 + (u1 - u0) * f, self.heading, speed
        return k[-1][1], k[-1][2], k[-1][3], self.heading, 0.0


EVENTS = [  # (t, kind, value)   kind: state=(armed, mode) | fcu | team | reached
    (0.0, 'state', (False, 'STABILIZE')),
    (3.0, 'state', (True, 'GUIDED')), (3.5, 'state', (True, 'AUTO')), (3.5, 'fcu', 'Mission: 1 Takeoff'),
    (9.5, 'fcu', 'Mission: 2 DigiCamCtrl'), (9.6, 'team', 'Camera trigger STARTED'), (9.6, 'reached', 1),
    (14.5, 'reached', 3), (14.5, 'fcu', 'Mission: 4 WP'),
    (38.5, 'reached', 8),
    (38.6, 'team', 'Reached last WP. Holding to finish image processing before RTL'),
    (38.7, 'state', (True, 'GUIDED')),
    (41.0, 'team', 'All 24 images processed. Selecting targets.'),
    (41.2, 'team', 'Targets: tent@9 -> person@10. Diverting.'),
    (41.8, 'team', 'Set current WP -> 9'), (42.0, 'team', 'Resumed AUTO -> WP 9'),
    (42.0, 'state', (True, 'AUTO')),
    (47.0, 'reached', 9), (47.1, 'team', 'Object waypoint reached. Holding for 3 seconds.'),
    (47.2, 'team', 'Over tent: loiter 6s then drop'), (47.3, 'state', (True, 'GUIDED')),
    (49.0, 'team', 'Servo 13 -> 1400'), (50.0, 'team', 'Servo 13 -> 500'),
    (50.5, 'state', (True, 'AUTO')),
    (55.5, 'reached', 10), (55.6, 'team', 'Over person: loiter 6s then drop'),
    (55.7, 'state', (True, 'GUIDED')),
    (57.0, 'team', 'Servo 11 -> 1400'), (58.0, 'team', 'Servo 11 -> 500'),
    (58.2, 'team', 'All targets done. Waiting 6s then RTL.'),
    (59.0, 'state', (True, 'RTL')), (59.0, 'team', 'RTL'),
    (75.6, 'state', (False, 'LAND')), (75.7, 'fcu', 'Disarming motors'),
]
PHOTO_TIMES = [15.0 + i for i in range(24)]


class FakeMission(Node):
    """Publishes the scripted mission and records what sim_demo publishes back."""

    def __init__(self, args):
        super().__init__('fake_mission')
        from mavros_msgs.msg import State, StatusText, VfrHud, WaypointReached
        from sensor_msgs.msg import Image, NavSatFix
        from std_msgs.msg import Float64, String
        from interfaces.msg import ImageResult
        from visualization_msgs.msg import MarkerArray
        self.M = dict(State=State, StatusText=StatusText, VfrHud=VfrHud, WaypointReached=WaypointReached,
                      Image=Image, NavSatFix=NavSatFix, Float64=Float64, String=String)
        self.world = load_world(args.world, args.worlds_dir)
        tg = {t.cls: t for t in self.world.targets}
        self.path = FlightPath(tg['tent'], tg['person'])
        self.ground = SyntheticGround(self.world, px_per_m=args.px_per_m)
        self.photo_dir = args.photo_dir
        os.makedirs(self.photo_dir, exist_ok=True)
        self.with_photo_info = args.photo_info
        pub = self.create_publisher
        self.p_state = pub(State, '/mavros/state', 10)
        self.p_fix = pub(NavSatFix, '/mavros/global_position/global', 10)
        self.p_rel = pub(Float64, '/mavros/global_position/rel_alt', 10)
        self.p_hdg = pub(Float64, '/mavros/global_position/compass_hdg', 10)
        self.p_vfr = pub(VfrHud, '/mavros/vfr_hud', 10)
        self.p_reached = pub(WaypointReached, '/mavros/mission/reached', 10)
        self.p_recv = pub(StatusText, '/mavros/statustext/recv', 10)
        self.p_send = pub(StatusText, '/mavros/statustext/send', 10)
        self.p_img = pub(Image, '/sim/camera/image',
                         QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.p_info = pub(String, '/sim/camera/photo_info',
                          QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.team = TeamCallbackRunner(args.team_repo)
        self.results = []
        self.marker_ns = set()
        self.create_subscription(ImageResult, '/image_detection', self._on_result,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(MarkerArray, '/sim/demo/markers', self._on_markers,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.t0 = time.time() + args.start_delay
        self.armed, self.mode = False, 'STABILIZE'
        self.events = list(EVENTS)
        self.photo_times = list(PHOTO_TIMES)
        self.photos_written = []
        self.jobs: queue.Queue = queue.Queue()
        self.worker = threading.Thread(target=self._photo_worker, daemon=True)
        self.worker.start()
        self.last_frame = 0.0
        self.frame_period = 1.0 / args.video_hz
        self.video_scale = args.video_scale
        self.create_timer(0.1, self._tick)

    # ---------------------------------------------------------------- outputs
    def _text(self, pub, text):
        m = self.M['StatusText']()
        m.severity = 6
        m.text = text
        pub.publish(m)

    def _tick(self):
        t = time.time() - self.t0
        if t < 0:
            return
        e, n, u, hdg, spd = self.path.at(t)
        lat, lon, _ = self.world.origin.to_geo(e, n)
        while self.events and self.events[0][0] <= t:
            _, kind, val = self.events.pop(0)
            if kind == 'state':
                self.armed, self.mode = val
            elif kind == 'fcu':
                self._text(self.p_recv, val)
            elif kind == 'team':
                self._text(self.p_send, val)
            elif kind == 'reached':
                r = self.M['WaypointReached']()
                r.wp_seq = int(val)
                self.p_reached.publish(r)
        st = self.M['State']()
        st.connected, st.armed, st.guided, st.mode = True, self.armed, True, self.mode
        self.p_state.publish(st)
        fix = self.M['NavSatFix']()
        fix.latitude, fix.longitude, fix.altitude = lat, lon, self.world.origin.alt + u
        self.p_fix.publish(fix)
        self.p_rel.publish(self.M['Float64'](data=float(u)))
        self.p_hdg.publish(self.M['Float64'](data=float(hdg)))
        v = self.M['VfrHud']()
        v.groundspeed, v.heading, v.altitude, v.climb = float(spd), int(hdg), float(u), 0.0
        self.p_vfr.publish(v)
        while self.photo_times and self.photo_times[0] <= t:
            self.photo_times.pop(0)
            # video_cam picks the name from the newest GPS fix BEFORE the shutter.
            self.jobs.put((time.time(), lat, lon, e, n, u, hdg))
        if time.time() - self.last_frame >= self.frame_period:
            self.last_frame = time.time()
            pose = CameraPose(e, n, max(0.5, u), yaw_deg=hdg, pitch_deg=-90.0)
            arr = self.ground.render(VIDEO_INTR, pose, scale=self.video_scale)
            img = self.M['Image']()
            img.height, img.width = arr.shape[:2]
            img.encoding, img.step = 'rgb8', arr.shape[1] * 3
            img.data = arr.tobytes()
            img.header.frame_id = 'a8mini_camera_optical_frame'
            self.p_img.publish(img)

    def _photo_worker(self):
        while True:
            shutter, lat, lon, e, n, u, hdg = self.jobs.get()
            pose = CameraPose(e, n, u, yaw_deg=hdg, pitch_deg=-90.0)
            arr = self.ground.render(PHOTO_INTR, pose)
            if self.with_photo_info:
                x, y, z = camera_axes(pose)
                info = {'emulator': 'fake_mission', 'camera_model': 'ZR10',
                        'wall_time': datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
                        'image': {'width': PHOTO_INTR.width, 'height': PHOTO_INTR.height},
                        'zoom': {'total': 1.0, 'optical': 1.0, 'digital': 1.0},
                        'hfov_rad': {'lens': 1.4137, 'effective': 1.4137},
                        'camera_world': {'position_enu_m': [e, n, u],
                                         'optical_quat_xyzw': list(quat_from_axes(x, y, z)),
                                         'heading_compass_deg': hdg, 'pitch_deg': -90.0},
                        'sd_name': f'IMG_{len(self.photos_written) + 1:04d}.jpg'}
                self.p_info.publish(self.M['String'](data=json.dumps(info)))
            time.sleep(max(0.0, shutter + 1.0 - time.time()))      # "download" takes ~1 s
            name = gps_photo_name(lat, lon)
            tmp = os.path.join(self.photo_dir, name.replace('.jpg', '.tmp.jpg'))
            PILImage.fromarray(arr).save(tmp, quality=88)
            os.replace(tmp, os.path.join(self.photo_dir, name))
            self.photos_written.append(name)

    # ---------------------------------------------------------------- inputs
    def _on_result(self, msg):
        self.results.append((msg.image_name, len(msg.detections.detections), msg.method,
                             float(msg.latitude), float(msg.longitude)))
        self.team.feed(msg)

    def _on_markers(self, msg):
        for m in msg.markers:
            if m.ns:
                self.marker_ns.add(m.ns)

    def summary(self):
        best = self.team.best()
        tg = {t.cls: t for t in self.world.targets}
        errors = {}
        for cls, (conf, lat, lon) in best.items():
            if conf > 0:
                e, n, _ = self.world.origin.to_enu(lat, lon)
                errors[cls] = round(math.hypot(e - tg[cls].east, n - tg[cls].north), 2)
        with_boxes = sum(1 for r in self.results if r[1] > 0)
        need_ns = {'search_area', 'targets', 'footprints', 'detections', 'best', 'track', 'drone',
                   'drops'}
        ok = (len(self.photos_written) == len(PHOTO_TIMES)
              and self.team.processed == len(PHOTO_TIMES)
              and best['person'][0] > 0 and best['tent'][0] > 0
              and need_ns <= self.marker_ns)
        return {'ok': ok, 'photos_written': len(self.photos_written),
                'image_results': len(self.results), 'results_with_boxes': with_boxes,
                'team_processed_photos': self.team.processed, 'team_best': best,
                'team_choice_error_m': errors, 'team_status_texts': self.team.acks[-6:],
                'methods': sorted({r[2] for r in self.results}),
                'marker_namespaces': sorted(self.marker_ns),
                'missing_namespaces': sorted(need_ns - self.marker_ns)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--world', default='testfield')
    ap.add_argument('--worlds-dir', default=os.path.join(HERE, '..', 'fixtures', 'worlds'))
    ap.add_argument('--photo-dir', required=True)
    ap.add_argument('--team-repo', default='')
    ap.add_argument('--duration', type=float, default=92.0)
    ap.add_argument('--start-delay', type=float, default=3.0)
    ap.add_argument('--video-hz', type=float, default=6.0)
    ap.add_argument('--video-scale', type=float, default=0.75)
    ap.add_argument('--px-per-m', type=float, default=20.0)
    ap.add_argument('--photo-info', type=int, default=1, help='1 = also publish emulator records')
    ap.add_argument('--summary', default='')
    args = ap.parse_args()
    rclpy.init()
    node = FakeMission(args)
    end = time.time() + args.duration
    try:
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        s = node.summary()
        text = json.dumps(s, indent=2)
        print(text, flush=True)
        if args.summary:
            with open(args.summary, 'w', encoding='utf-8') as f:
                f.write(text)
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if s['ok'] else 1)


if __name__ == '__main__':
    main()
