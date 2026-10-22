"""
sim_demo/truth_detector.py -- PURPOSE: the OPTIONAL stand-in for the team's detector
(detection/new_od). It watches the same photo folder new_od watches and, for every new photo,
publishes the SAME message on the SAME topic that new_od publishes - interfaces/ImageResult
on /image_detection (RELIABLE, depth 10) - but the boxes come from the world's answer key
(sim_worlds truth/<world>.yaml) and the camera pose at the moment the photo was taken,
instead of from the YOLO model.

WHY: so the rest of the mission (main_controller choosing targets, flying there, dropping
payloads) can be demonstrated even if the real model does not recognise simulated targets.
Every message says method = "truth_detector (stand-in: answer key, not the AI model)" and the
demo panel shows a big amber banner while it runs. Run only ONE detector at a time (the
launch argument detector:=real|truth chooses).

Behaviour copied from new_od so main_controller cannot tell the difference:
  * every photo in the folder gets exactly one ImageResult (also photos without targets and
    photos that were already there at start), because main_controller waits until every
    photo file has been reported before it chooses targets;
  * image_name = the file name; latitude/longitude = the GPS in the file name (the drone's
    position at the shutter; 0.0 if the name has none); class_id "0" = person, "1" = tent;
  * /sahi_detection_info JSON and the /sahi/get_statistics, /sahi/get_health services.
Optional (parameter geolocation:=projected): report the target's own position (projected
from the box centre, plus small noise) instead of the drone's - shows what better
geolocation would change.

Run:  ros2 run sim_demo truth_detector --ros-args -p world:=runway
"""
from __future__ import annotations

import json
import random
import time
import zlib
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .camera_model import DEFAULT_HFOV_RAD, pixel_to_ground
from .detections import DETECTION_TOPIC, TRUTH_METHOD_PREFIX, DetectionFrame, frame_to_image_result
from .photo_geometry import PhotoGeometry
from .photo_info import PhotoInfoIndex
from .photos import PhotoWatcher, resolve_photo_dir
from .ros_common import QOS_DETECTIONS, FlightData, detector_publishers
from .truth_model import TruthModelConfig, detect_targets
from .world_info import load_world

METHOD = f'{TRUTH_METHOD_PREFIX} (stand-in: answer key, not the AI model)'


class TruthDetectorNode(Node):
    """ROS node: photo folder -> answer-key detections on /image_detection."""

    def __init__(self):
        super().__init__('truth_detector')
        p = self.declare_parameter
        p('world', 'runway')                  # world name (sim_worlds/config/<world>.yaml)
        p('worlds_dir', '')                   # sim_worlds folder ('' = installed package)
        p('truth_file', '')                   # override the truth YAML
        p('photo_dir', '')                    # '' = the team's <ws>/src/video_cam/mapping_photos
        p('poll_period_s', 0.5)               # how often to look for new photos [s]
        p('processing_delay_s', 0.5)          # pretend model time before publishing [s]
        p('hfov_rad', DEFAULT_HFOV_RAD)       # camera horizontal field of view at zoom 1
        p('zoom', 0.0)                        # > 0 forces a zoom; 0 = follow the camera
        p('gimbal_pitch_deg', -90.0)          # camera tilt during the search
        p('gimbal_yaw_offset_deg', 0.0)       # camera yaw relative to the nose (+ = right)
        p('capture_latency_s', 1.0)           # shutter-to-file delay when the name has no GPS
        p('geolocation', 'filename')          # 'filename' (like new_od) or 'projected'
        p('position_noise_m', 0.5)            # noise of 'projected' positions [m]
        p('min_visible_fraction', 0.5)
        p('min_pixels', 12.0)
        p('good_pixels', 60.0)
        p('detect_probability', 0.97)
        p('confidence_noise', 0.03)
        p('seed', 0)
        p('include_distractors', False)       # also "detect" look-alikes (standing people ...)
        gp = self.get_parameter
        self.poll_period = float(gp('poll_period_s').value)
        self.delay = float(gp('processing_delay_s').value)
        self.geolocation = str(gp('geolocation').value).lower()
        self.position_noise = float(gp('position_noise_m').value)
        self.cfg = TruthModelConfig(
            min_visible_fraction=float(gp('min_visible_fraction').value),
            min_pixels=float(gp('min_pixels').value),
            good_pixels=float(gp('good_pixels').value),
            detect_probability=float(gp('detect_probability').value),
            noise_sigma=float(gp('confidence_noise').value),
            seed=int(gp('seed').value))

        log = self.get_logger()
        try:
            self.world = load_world(str(gp('world').value), str(gp('worlds_dir').value),
                                    str(gp('truth_file').value))
        except Exception as exc:  # noqa: BLE001 - keep running: publish empty results
            log.error(f'Could not load the world ({exc}). The stand-in will report NO targets '
                      'until it is restarted with a valid world.')
            self.world = None
        self.detectable = []
        if self.world is not None:
            for w in self.world.warnings:
                log.warn(w)
            names = ', '.join(f'{t.name} ({t.cls})' for t in self.world.targets) or 'none'
            log.info(f'World {self.world.name}: answer key has {names}')
            self.detectable = list(self.world.targets)
            if bool(gp('include_distractors').value) and self.world.distractors:
                # A real detector also "sees" look-alikes (a standing person is a person).
                self.detectable += self.world.distractors
                log.info(f'Also reporting {len(self.world.distractors)} look-alike distractors '
                         '(false alarms on purpose)')

        self.photo_dir, how = resolve_photo_dir(str(gp('photo_dir').value))
        log.info(f'Watching photos in {self.photo_dir} ({how})')
        self.watcher = PhotoWatcher(self.photo_dir, report_existing=True)
        origin = self.world.origin if self.world else None
        # The camera emulator's exact record of each photo (best source of the camera pose).
        self.photo_info = PhotoInfoIndex(origin) if origin else None
        self.flight = FlightData(self, zoom_fixed=float(gp('zoom').value),
                                 on_reached=self._on_reached, photo_info=self.photo_info)
        self.geometry = (PhotoGeometry(origin, self.flight, float(gp('hfov_rad').value),
                                       float(gp('gimbal_pitch_deg').value),
                                       float(gp('gimbal_yaw_offset_deg').value),
                                       float(gp('capture_latency_s').value),
                                       photo_info=self.photo_info)
                         if origin else None)
        self.last_reached = 0
        self.queue: deque = deque()
        self.stats = dict(images=0, detections=0, people=0, tents=0, start=time.time(),
                          no_pose=0, preexisting=0)
        self._warned: set[str] = set()

        from interfaces.msg import ImageResult
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        self.pub = self.create_publisher(ImageResult, DETECTION_TOPIC, QOS_DETECTIONS)
        self.create_subscription(ImageResult, DETECTION_TOPIC, self._on_any_result, QOS_DETECTIONS)
        self.info_pub = self.create_publisher(String, '/sahi_detection_info', QOS_DETECTIONS)
        self._String = String
        self.create_service(Trigger, 'sahi/get_statistics', self._stats_srv)
        self.create_service(Trigger, 'sahi/get_health', self._health_srv)
        self.create_timer(self.poll_period, self._tick)
        self.create_timer(5.0, self._check_other_detectors)
        log.warn('TRUTH DETECTOR STAND-IN ACTIVE: detections come from the answer key, '
                 'NOT from the AI model.')

    # ------------------------------------------------------------------ helpers
    def _warn_once(self, key: str, text: str) -> None:
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warn(text)

    def _on_reached(self, t: float, seq: int) -> None:
        self.last_reached = seq

    def _check_other_detectors(self) -> None:
        others = [n for n in detector_publishers(self, DETECTION_TOPIC) if n != self.get_name()]
        if others and 'others_info' not in self._warned:
            self._warned.add('others_info')
            self.get_logger().info(f'{others} also has a publisher on {DETECTION_TOPIC}. Fine while '
                                   'it stays idle (sim_bringup detector:=truth points new_od at an '
                                   'empty folder); a warning follows if it really publishes.')

    def _on_any_result(self, msg) -> None:
        """Warn once if ANOTHER detector really publishes results (two detectors confuse
        main_controller: it would mix the AI's and the answer key's detections)."""
        if not str(msg.method).startswith(TRUTH_METHOD_PREFIX):
            self._warn_once('others', f'Another detector is publishing on {DETECTION_TOPIC} '
                                      f'(method "{msg.method}"). Run only one: launch '
                                      'detector:=real or detector:=truth.')

    # ------------------------------------------------------------------ main loop
    def _tick(self) -> None:
        now = time.time()
        for ev in self.watcher.poll(now):
            self.queue.append((now + (0.0 if ev.preexisting else self.delay), ev))
        while self.queue and self.queue[0][0] <= now:
            _, ev = self.queue.popleft()
            try:
                self._process(ev)
            except Exception as exc:  # noqa: BLE001 - one bad photo must not stop the stand-in
                self.get_logger().error(f'Failed on {ev.name}: {exc}')

    def _process(self, ev) -> None:
        boxes, how, zoom_txt = [], 'none', ''
        lat = ev.lat if ev.lat is not None else 0.0
        lon = ev.lon if ev.lon is not None else 0.0
        if ev.lat is None:
            self._warn_once('nogps', f"No lat/lon in filename '{ev.name}'; detection waypoints "
                                     'will default to (0.0, 0.0)  [same as new_od]')
        sol = self.geometry.solve(ev) if (self.geometry and self.world) else None
        if ev.preexisting:
            self.stats['preexisting'] += 1
            self._warn_once('pre', 'Photos were already in the folder at start: they are reported '
                                   'with no detections (their camera pose is unknown). Empty the '
                                   'folder before a run for a clean demo.')
        elif sol is None or sol.pose is None:
            self.stats['no_pose'] += 1
            self._warn_once('nopose', f'No drone position recorded for {ev.name} (is MAVROS '
                                      'running?) - reported with no detections.')
        else:
            how = sol.how
            zoom_txt = f'zoom {sol.intr.zoom:.1f}x ({sol.zoom_source})'
            if how != 'emulator' and self.photo_info is not None and self.photo_info.received:
                self.get_logger().warn(f'{ev.name}: no camera-emulator record matched '
                                       f'({self.photo_info.last_miss}); using the MAVROS history')
            hits = detect_targets(sol.intr, sol.pose, self.detectable, ev.name, self.cfg)
            boxes = [h.box for h in hits]
            if hits and self.geolocation == 'projected':
                best = max(hits, key=lambda h: h.box.score)
                g = pixel_to_ground(sol.intr, sol.pose, best.box.cx, best.box.cy, best.target.up)
                if g is not None:
                    rng = random.Random(zlib.crc32(ev.name.encode()))   # same noise every run
                    e = g[0] + rng.gauss(0.0, self.position_noise)
                    n = g[1] + rng.gauss(0.0, self.position_noise)
                    lat, lon, _ = self.world.origin.to_geo(e, n)
        frame = DetectionFrame(image_name=ev.name, lat=lat, lon=lon, boxes=boxes, method=METHOD,
                               num_detections=len(boxes), waypoint_index=self.last_reached,
                               received=time.time(), saved_to=ev.path)
        msg = frame_to_image_result(frame, stamp=self.get_clock().now().to_msg(),
                                    slice_size='n/a', overlap='n/a')
        self.pub.publish(msg)
        n_p = sum(1 for b in boxes if b.cls == 'person')
        n_t = sum(1 for b in boxes if b.cls == 'tent')
        self.stats['images'] += 1
        self.stats['detections'] += len(boxes)
        self.stats['people'] += n_p
        self.stats['tents'] += n_t
        info = {'image': ev.name, 'timestamp': msg.timestamp, 'num_people': n_p,
                'num_tents': n_t, 'num_objects': 0, 'total_detections': len(boxes),
                'method': METHOD, 'slice_size': 'n/a', 'overlap': 'n/a'}
        self.info_pub.publish(self._String(data=json.dumps(info, separators=(',', ':'))))
        found = ', '.join(f'{b.cls} {b.score:.2f}' for b in boxes) or 'no targets'
        extra = f' [{how}, {zoom_txt}]' if zoom_txt else ''
        self.get_logger().info(f'{ev.name}: {found}{extra}')

    # ------------------------------------------------------------------ services
    def _stats_srv(self, request, response):
        s = self.stats
        response.success = True
        response.message = (f"Images: {s['images']}, Detections: {s['detections']}, "
                            f"People: {s['people']}, Tents: {s['tents']}, AvgTime: "
                            f"{self.delay:.2f}s, Errors: 0, Uptime: {time.time() - s['start']:.0f}s"
                            ' (truth_detector stand-in)')
        return response

    def _health_srv(self, request, response):
        ok = self.world is not None and bool(self.world.targets)
        response.success = ok
        response.message = (f'Healthy: {ok}, ConsecErrors: 0, ModelLoaded: {ok}, '
                            'Format: truth_detector (answer key)')
        return response


def main(args=None):
    """Entry point: ros2 run sim_demo truth_detector."""
    rclpy.init(args=args)
    node = TruthDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
