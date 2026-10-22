#!/usr/bin/env python3
"""
sim_models/test/acceptance/acceptance_driver.py -- PURPOSE: fly the simulated drone and MEASURE whether
the iris_a8mini model does what the simulation contract promises. run_acceptance.sh starts Gazebo,
the model, payload_release and ArduCopter SITL, then runs this file.

What it does (each step writes a result with numbers into results.json and summary.txt):
  parm     astra_sitl.parm really loaded: every value read back from SITL over MAVLink
  flight   GUIDED, arm (after the ~30 s settling window), take off to 15 m, hover 20 s (altitude,
           roll/pitch, drift, hover throttle), later LAND and disarm
  cruise   with both payloads on: 30 m north and back at the default waypoint speed (attitude,
           top speed, height kept)
  gimbal   step commands on /sim/gimbal/*/cmd_pos: slew rate (~90 deg/s) and steady error (< 0.5 deg)
           from /sim/gimbal/joint_states; video frames saved at pitch -90 (straight down) and with
           yaw +30, and the coloured targets of the test world must appear where the camera
           geometry (drone pose + joint angles, offsets read from the model SDF) predicts
  photo    one message on /sim/camera/photo/trigger -> exactly one photo at the profile resolution
  rtf      real-time factor without cameras and with the video subscribed + 1 photo per second
  payload  MAV_CMD_DO_SET_SERVO 11 1400 during the hover: the person payload falls to the ground,
           then servo 13: the tent payload; the drone stays stable; payload_release logged it

Talks to SITL with pymavlink (tcp:127.0.0.1:5760+10*I) and to Gazebo with gz.transport13.
Usage: see run_acceptance.sh (all options: acceptance_driver.py --help).
Exit code: 0 = all selected tests passed, 1 = something failed.
"""

import argparse
import json
import math
import os
import sys
import threading
import time
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera_geometry import GimbalKinematics, PinholeCamera, find_blob  # noqa: E402

DEG = math.pi / 180.0
TAKEOFF_ALT = 15.0  # m
HOLD_S = 20.0  # s
SLEW_EXPECTED = 90.0  # deg/s
STEADY_MAX_DEG = 0.5  # deg
PIXEL_TOL = 12.0  # px at 1280 wide (scaled for other widths)

# Coloured targets of test/worlds/sim_models_test.sdf (world ENU metres)
GROUND_TARGETS = {
    "red": (0.0, 5.0, 0.02),
    "magenta": (5.0, 0.0, 0.02),
    "blue": (0.0, -5.0, 0.02),
    "cyan": (-5.0, 0.0, 0.02),
}
POLE_TARGETS = {"red": (0.0, 60.0, 15.0), "magenta": (-30.0, 51.9615, 15.0)}


def log(msg):
    print(f"[driver {time.strftime('%H:%M:%S')}] {msg}", flush=True)


# =========================================================================== MAVLink side
class Mav:
    """pymavlink connection with a reader thread that keeps the latest messages and logs."""

    def __init__(self, url):
        from pymavlink import mavutil

        self.mavutil = mavutil
        self.m = mavutil.mavlink_connection(url, source_system=245, source_component=190)
        self.lock = threading.Lock()
        self.latest = {}
        self.statustexts = []  # (wall, text)
        self.acks = deque(maxlen=200)
        self.params = {}
        self.attitude = deque(maxlen=20000)  # (wall, roll, pitch, yaw) rad
        self.gpos = deque(maxlen=20000)  # (wall, rel_alt m, lat, lon)
        self.hud = deque(maxlen=20000)  # (wall, throttle %)
        self.servo = deque(maxlen=20000)  # (wall, s1, s2, s3, s4, s11, s13)
        self.armed = False
        self.custom_mode = None
        self._stop = False
        self._thread = threading.Thread(target=self._reader, daemon=True)

    def start(self, timeout=300):
        """Wait for the first heartbeat (SITL only boots when Gazebo answers), start the reader."""
        hb = self.m.wait_heartbeat(timeout=timeout)
        if hb is None:
            raise RuntimeError("no MAVLink heartbeat from SITL")
        self.m.target_system, self.m.target_component = 1, 1
        self._thread.start()
        # ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD, SERVO_OUTPUT_RAW, SYS_STATUS, GPS_RAW_INT,
        # DISTANCE_SENSOR, EKF_STATUS_REPORT
        for msg_id, hz in (
            (30, 20),
            (33, 10),
            (74, 5),
            (36, 10),
            (1, 2),
            (24, 2),
            (132, 2),
            (193, 2),
        ):
            self.command(511, msg_id, 1e6 / hz, wait=False)  # MAV_CMD_SET_MESSAGE_INTERVAL

    def stop(self):
        self._stop = True

    def _reader(self):
        while not self._stop:
            try:
                msg = self.m.recv_match(blocking=True, timeout=0.5)
            except Exception:  # noqa: BLE001 - connection hiccup: keep reading
                time.sleep(0.1)
                continue
            if msg is None:
                continue
            t = time.time()
            kind = msg.get_type()
            with self.lock:
                self.latest[kind] = msg
                if kind == "HEARTBEAT" and msg.get_srcSystem() == 1 and msg.type != 6:
                    self.armed = bool(msg.base_mode & 128)
                    self.custom_mode = msg.custom_mode
                elif kind == "STATUSTEXT":
                    self.statustexts.append((t, msg.text))
                    log(f"SITL says: {msg.text}")
                elif kind == "COMMAND_ACK":
                    self.acks.append((t, msg.command, msg.result))
                elif kind == "PARAM_VALUE":
                    self.params[msg.param_id] = msg.param_value
                elif kind == "ATTITUDE":
                    self.attitude.append((t, msg.roll, msg.pitch, msg.yaw))
                elif kind == "GLOBAL_POSITION_INT":
                    self.gpos.append((t, msg.relative_alt / 1000.0, msg.lat / 1e7, msg.lon / 1e7))
                elif kind == "VFR_HUD":
                    self.hud.append((t, msg.throttle))
                elif kind == "SERVO_OUTPUT_RAW":
                    self.servo.append(
                        (
                            t,
                            msg.servo1_raw,
                            msg.servo2_raw,
                            msg.servo3_raw,
                            msg.servo4_raw,
                            msg.servo11_raw,
                            msg.servo13_raw,
                        )
                    )

    def command(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, wait=True, timeout=5.0):
        """Send COMMAND_LONG; return the COMMAND_ACK result (0 = accepted) or None."""
        t0 = time.time()
        self.m.mav.command_long_send(1, 1, cmd, 0, p1, p2, p3, p4, p5, p6, p7)
        if not wait:
            return None
        while time.time() - t0 < timeout:
            with self.lock:
                for t, c, r in self.acks:
                    if c == cmd and t >= t0:
                        return r
            time.sleep(0.05)
        return None

    def set_mode(self, name, timeout=10.0):
        """Switch flight mode by name (GUIDED, LAND, ...); True once the heartbeat shows it."""
        mode_id = self.m.mode_mapping()[name]
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.m.mav.set_mode_send(
                1, self.mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id
            )
            time.sleep(0.5)
            if self.custom_mode == mode_id:
                return True
        return False

    def get_param(self, name, timeout=3.0):
        with self.lock:
            self.params.pop(name, None)
        self.m.mav.param_request_read_send(1, 1, name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                if name in self.params:
                    return self.params[name]
            time.sleep(0.05)
        return None

    def rel_alt(self):
        with self.lock:
            return self.gpos[-1][1] if self.gpos else None

    def texts_since(self, t0):
        with self.lock:
            return [s for t, s in self.statustexts if t >= t0]

    def window(self, series, t0, t1):
        with self.lock:
            return [s for s in series if t0 <= s[0] <= t1]


# =========================================================================== Gazebo side
class Gz:
    """gz-transport subscriptions and publishers used by the tests."""

    def __init__(self, world):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.model_pb2 import Model
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.msgs10.world_stats_pb2 import WorldStatistics
        from gz.transport13 import Node, SubscribeOptions

        self.T = dict(
            Boolean=Boolean,
            Double=Double,
            Image=Image,
            Model=Model,
            Odometry=Odometry,
            Pose_V=Pose_V,
            StringMsg=StringMsg,
            WorldStatistics=WorldStatistics,
        )
        self.SubscribeOptions = SubscribeOptions
        self.world = world
        self.node = Node()
        self.lock = threading.Lock()
        self.odom = deque(maxlen=50000)  # (t_sim, (x,y,z), (qx,qy,qz,qw), wall)
        self.joints = deque(maxlen=50000)  # (t_sim, yaw, roll, pitch, wall)
        self.stats = deque(maxlen=50000)  # (wall, sim_s, real_time_factor)
        self.payload_odom = {"person": deque(maxlen=20000), "tent": deque(maxlen=20000)}
        self.payload_state = {"person": [], "tent": []}
        self.servo = {"11": deque(maxlen=5000), "13": deque(maxlen=5000)}
        self.photos = []  # (wall, msg)
        self.video_latest = None
        self.video_count = 0
        self.video_stamps = deque(maxlen=5000)
        self.dyn_pose = deque(maxlen=5000)  # (wall, name, x, y, z) payload entries only
        self.camera_info = {}
        n, T = self.node, self.T
        ok = n.subscribe(T["Odometry"], "/sim/drone/odometry", self._on_odom)
        ok &= n.subscribe(T["Model"], "/sim/gimbal/joint_states", self._on_joints)
        ok &= n.subscribe(T["WorldStatistics"], f"/world/{world}/stats", self._on_stats)
        ok &= n.subscribe(T["Image"], "/sim/camera/photo/image", self._on_photo)
        for name in ("person", "tent"):
            ok &= n.subscribe(
                T["Odometry"],
                f"/sim/payload/{name}/odometry",
                lambda m, k=name: self._on_payload_odom(k, m),
            )
            ok &= n.subscribe(
                T["StringMsg"],
                f"/sim/payload/{name}/state",
                lambda m, k=name: self._on_payload_state(k, m),
            )
        opts = SubscribeOptions()
        opts.msgs_per_sec = 20
        for ch in ("11", "13"):
            ok &= n.subscribe(
                T["Double"], f"/sim/payload/servo{ch}", lambda m, k=ch: self._on_servo(k, m), opts
            )
        self.subscribed_ok = bool(ok)
        self.pub = {
            axis: n.advertise(f"/sim/gimbal/{axis}/cmd_pos", T["Double"])
            for axis in ("yaw", "roll", "pitch")
        }
        self.trigger_pub = n.advertise("/sim/camera/photo/trigger", T["Boolean"])
        # fixed inspection camera of the test world (pictures of the drone itself)
        self.inspection = []  # (wall, msg)
        n.subscribe(T["Image"], "/sim_models_test/inspection/image", self._on_inspection)
        self.inspection_pub = n.advertise("/sim_models_test/inspection/trigger", T["Boolean"])

    def _on_inspection(self, m):
        with self.lock:
            self.inspection.append((time.time(), m))

    def inspection_photo(self, tries=3, wait_s=10.0):
        """Trigger the world's inspection camera; return the new image message (None if none came).

        The trigger is sent again (up to `tries` times) if no picture arrives within `wait_s`
        seconds, because the very first trigger can be lost while gz-transport is still
        discovering the new publisher."""
        msg = self.T["Boolean"]()
        msg.data = True
        for _ in range(tries):
            with self.lock:
                before = len(self.inspection)
            self.inspection_pub.publish(msg)
            t0 = time.time()
            while time.time() - t0 < wait_s:
                with self.lock:
                    if len(self.inspection) > before:
                        return self.inspection[-1][1]
                time.sleep(0.1)
        return None

    @staticmethod
    def _stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9

    def _on_odom(self, m):
        p, q = m.pose.position, m.pose.orientation
        with self.lock:
            self.odom.append((self._stamp(m), (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), time.time()))

    def _on_joints(self, m):
        pos = {j.name: j.axis1.position for j in m.joint}
        with self.lock:
            self.joints.append(
                (
                    self._stamp(m),
                    pos.get("gimbal_yaw", math.nan),
                    pos.get("gimbal_roll", math.nan),
                    pos.get("gimbal_pitch", math.nan),
                    time.time(),
                )
            )

    def _on_stats(self, m):
        with self.lock:
            self.stats.append(
                (time.time(), m.sim_time.sec + m.sim_time.nsec * 1e-9, m.real_time_factor)
            )

    def _on_photo(self, m):
        with self.lock:
            self.photos.append((time.time(), m))

    def _on_video(self, m):
        with self.lock:
            self.video_latest = m
            self.video_count += 1
            self.video_stamps.append((time.time(), self._stamp(m)))

    def _on_payload_odom(self, name, m):
        p = m.pose.position
        with self.lock:
            self.payload_odom[name].append((self._stamp(m), p.x, p.y, p.z, time.time()))

    def _on_payload_state(self, name, m):
        with self.lock:
            self.payload_state[name].append((time.time(), m.data))

    def _on_servo(self, ch, m):
        with self.lock:
            self.servo[ch].append((time.time(), m.data))

    def _on_dyn_pose(self, m):
        t = time.time()
        with self.lock:
            for p in m.pose:
                if p.name.startswith("payload_"):
                    self.dyn_pose.append((t, p.name, p.position.x, p.position.y, p.position.z))

    def subscribe_video(self):
        return self.node.subscribe(self.T["Image"], "/sim/camera/video/image", self._on_video)

    def unsubscribe_video(self):
        self.node.unsubscribe("/sim/camera/video/image")

    def subscribe_dynamic_pose(self):
        return self.node.subscribe(
            self.T["Pose_V"], f"/world/{self.world}/dynamic_pose/info", self._on_dyn_pose
        )

    def set_gimbal(self, yaw=None, roll=None, pitch=None):
        for axis, value in (("yaw", yaw), ("roll", roll), ("pitch", pitch)):
            if value is not None:
                msg = self.T["Double"]()
                msg.data = float(value)
                self.pub[axis].publish(msg)

    def trigger_photo(self):
        msg = self.T["Boolean"]()
        msg.data = True
        self.trigger_pub.publish(msg)

    def latest_joint(self):
        with self.lock:
            return self.joints[-1] if self.joints else None

    def latest_odom(self):
        with self.lock:
            return self.odom[-1] if self.odom else None

    def nearest(self, series, t_sim):
        with self.lock:
            if not series:
                return None
            return min(series, key=lambda s: abs(s[0] - t_sim))

    def sim_now(self):
        j = self.latest_joint()
        return j[0] if j else None


def image_to_rgb(msg):
    """gz.msgs.Image (R8G8B8) -> HxWx3 uint8 numpy array."""
    h, w, step = msg.height, msg.width, msg.step or msg.width * 3
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    return buf.reshape(h, step)[:, : w * 3].reshape(h, w, 3)


def save_image(rgb, path, quality=92):
    import cv2

    bgr = np.ascontiguousarray(rgb[:, :, ::-1])
    if path.endswith(".jpg"):
        cv2.imwrite(path, bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    else:
        cv2.imwrite(path, bgr)
    return path


# =========================================================================== the tests
class Acceptance:
    def __init__(self, args):
        self.a = args
        self.results = {}
        self.images = []
        with open(args.model_sdf) as f:
            sdf_text = f.read()
        self.kin = GimbalKinematics(sdf_text)
        import xml.etree.ElementTree as ET

        model = ET.fromstring(sdf_text).find("model")
        cams = {s.get("name"): s for s in model.iter("sensor")}
        self.cam_cfg = {}
        for name in ("video_camera", "photo_camera"):
            s = cams[name]
            self.cam_cfg[name] = (
                int(s.find("camera/image/width").text),
                int(s.find("camera/image/height").text),
                float(s.find("camera/horizontal_fov").text),
            )
        self.gz = Gz(args.world)
        self.mav = None

    def out(self, name):
        return os.path.join(self.a.out, name)

    def record(self, name, ok, evidence, **data):
        res = "pass" if ok is True else ("fail" if ok is False else str(ok))
        self.results[name] = {"result": res, "evidence": evidence, **data}
        log(f"{name}: {res.upper()}  {evidence}")

    # ------------------------------------------------------------------ helpers
    def state_text(self):
        """One line with the autopilot state that matters for arming/takeoff (for the log)."""
        with self.mav.lock:
            lt = dict(self.mav.latest)
        parts = [
            f"armed={self.mav.armed}",
            f"mode={self.mav.custom_mode}",
            f"rel_alt={self.mav.rel_alt()}",
        ]
        gps = lt.get("GPS_RAW_INT")
        if gps is not None:
            parts.append(f"gps_fix={gps.fix_type} sats={gps.satellites_visible}")
        dist = lt.get("DISTANCE_SENSOR")
        if dist is not None:
            parts.append(f"rangefinder={dist.current_distance} cm")
        ekf = lt.get("EKF_STATUS_REPORT")
        if ekf is not None:
            parts.append(f"ekf_flags=0x{ekf.flags:x}")
        return ", ".join(parts)

    def wait(self, predicate, timeout, period=0.1):
        t0 = time.time()
        while time.time() - t0 < timeout:
            v = predicate()
            if v:
                return v
            time.sleep(period)
        return None

    def sim_sleep(self, seconds, wall_timeout=None):
        """Wait until SIMULATION time has advanced by `seconds` (the simulator may run slower
        than real time on a busy machine). Returns the sim time reached (None on timeout)."""
        start = self.gz.sim_now()
        if start is None:
            time.sleep(seconds)
            return None
        limit = wall_timeout if wall_timeout is not None else max(30.0, 15.0 * seconds)
        return self.wait(
            lambda: (self.gz.sim_now() or start) >= start + seconds and self.gz.sim_now(),
            limit,
            0.02,
        )

    def rtf_window(self, wall0, wall1):
        """(rtf from sim/wall deltas, mean of reported rtf, samples) over a wall-time window."""
        with self.gz.lock:
            s = [x for x in self.gz.stats if wall0 <= x[0] <= wall1]
        if len(s) < 2:
            return None, None, len(s)
        ratio = (s[-1][1] - s[0][1]) / max(1e-6, s[-1][0] - s[0][0])
        return ratio, float(np.mean([x[2] for x in s])), len(s)

    def fresh_video_frame(self, after_sim, timeout=5.0):
        def get():
            with self.gz.lock:
                m = self.gz.video_latest
            return m if m is not None and Gz._stamp(m) >= after_sim else None

        return self.wait(get, timeout, 0.02)

    # ------------------------------------------------------------------ parm
    def test_parm(self):
        names, bad = [], []
        with open(self.a.parm) as f:
            for line in f:
                s = line.split("#", 1)[0].split()
                if len(s) >= 2:
                    names.append((s[0], float(s[1])))
        for name, want in names:
            got = self.mav.get_param(name)
            if got is None or abs(got - want) > 1e-3:
                bad.append(f"{name}={got} (want {want})")
        self.record(
            "parm_loaded",
            not bad,
            f"{len(names) - len(bad)}/{len(names)} astra_sitl.parm values read back from SITL"
            + (f"; wrong: {bad}" if bad else ""),
        )

    # ------------------------------------------------------------------ flight
    def arm_and_takeoff(self):
        t_boot = time.time()
        # SITL resets the flight mode to STABILIZE when its start-up finishes ("ArduPilot Ready",
        # about 1.5 s after the first heartbeat; measured on 4.6.3): a mode asked for earlier is
        # lost. So wait for that message, then ask for GUIDED, and check it again before arming.
        self.wait(lambda: any("ArduPilot Ready" in s for s in self.mav.texts_since(0)), 30)
        if not self.mav.set_mode("GUIDED"):
            self.record("flight", False, "could not switch to GUIDED")
            return False
        guided = self.mav.m.mode_mapping()["GUIDED"]
        attempts, armed_at = 0, None
        while time.time() - t_boot < 600:
            attempts += 1
            if self.mav.custom_mode != guided:
                log(f"mode is {self.mav.custom_mode}, not GUIDED: asking again")
                self.mav.set_mode("GUIDED")
            r = self.mav.command(400, 1)  # MAV_CMD_COMPONENT_ARM_DISARM
            if r == 0 and self.wait(lambda: self.mav.armed, 3):
                armed_at = time.time()
                break
            time.sleep(4)
        prearm = sorted({t for t in self.mav.texts_since(t_boot) if "Arm" in t or "PreArm" in t})
        if armed_at is None:
            self.record("flight", False, f"did not arm in 600 s ({attempts} tries); {prearm[-5:]}")
            return False
        self.arm_info = {
            "seconds_after_first_try": round(armed_at - t_boot, 1),
            "attempts": attempts,
            "prearm_messages": prearm[:10],
        }
        log(
            f"armed after {armed_at - t_boot:.0f} s ({attempts} tries); pre-arm messages: {prearm[:4]}"
        )
        r = None
        for attempt in range(5):
            time.sleep(1.0)
            r = self.mav.command(22, p7=TAKEOFF_ALT)  # MAV_CMD_NAV_TAKEOFF
            log(f"takeoff try {attempt + 1}: ACK {r}; state: {self.state_text()}")
            if r == 0:
                break
            if not self.mav.armed:
                log("disarmed again (no takeoff within the auto-disarm delay?): re-arming")
                self.mav.command(400, 1)
        if r != 0:
            self.record("flight", False, f"takeoff refused (ACK {r}); state: {self.state_text()}")
            return False
        t0 = time.time()
        if not self.wait(lambda: (self.mav.rel_alt() or 0) > TAKEOFF_ALT - 0.7, 300):
            self.record(
                "flight", False, f"did not reach {TAKEOFF_ALT} m (alt {self.mav.rel_alt()})"
            )
            return False
        self.climb_s = time.time() - t0
        self.sim_sleep(3.0)  # let it settle
        return True

    def test_hold(self):
        o0 = self.gz.latest_odom()
        w0 = time.time()
        self.sim_sleep(HOLD_S, wall_timeout=600)
        w1 = time.time()
        o1 = self.gz.latest_odom()
        alts = [s[1] for s in self.mav.window(self.mav.gpos, w0, w1)]
        att = self.mav.window(self.mav.attitude, w0, w1)
        thr = [s[1] for s in self.mav.window(self.mav.hud, w0, w1)]
        motors = self.mav.window(self.mav.servo, w0, w1)
        roll = max(abs(s[1]) for s in att) / DEG
        pitch = max(abs(s[2]) for s in att) / DEG
        drift = math.dist(o0[1][:2], o1[1][:2]) if o0 and o1 else math.nan
        rtf, rtf_rep, _ = self.rtf_window(w0, w1)
        self.hold_rtf = rtf
        data = {
            "alt_mean_m": round(float(np.mean(alts)), 2),
            "alt_min_m": round(min(alts), 2),
            "alt_max_m": round(max(alts), 2),
            "alt_std_m": round(float(np.std(alts)), 3),
            "max_abs_roll_deg": round(roll, 2),
            "max_abs_pitch_deg": round(pitch, 2),
            "horizontal_drift_m": round(drift, 2),
            "hover_throttle_pct": round(float(np.mean(thr)), 1) if thr else None,
            "motor_pwm_mean": [round(float(np.mean([m[i] for m in motors]))) for i in range(1, 5)]
            if motors
            else None,
            "climb_time_s": round(self.climb_s, 1),
            "arming": self.arm_info,
            "gz_odometry_z_m": round(o1[1][2], 2) if o1 else None,
            "rtf_no_camera": round(rtf, 3) if rtf else None,
        }
        ok = (
            abs(data["alt_mean_m"] - TAKEOFF_ALT) < 1.0
            and data["alt_std_m"] < 0.3
            and roll < 5.0
            and pitch < 5.0
            and drift < 1.5
        )
        self.record(
            "flight_hover",
            ok,
            f"{HOLD_S:.0f} s at {data['alt_mean_m']} m (min {data['alt_min_m']}, max "
            f"{data['alt_max_m']}, std {data['alt_std_m']}), max |roll| {data['max_abs_roll_deg']} "
            f"deg, max |pitch| {data['max_abs_pitch_deg']} deg, drift {data['horizontal_drift_m']} m, "
            f"hover throttle {data['hover_throttle_pct']} %",
            **data,
        )

    def goto_local(self, north, east, timeout=120.0):
        """GUIDED position target in ArduPilot's local NED frame (metres from the EKF origin =
        take-off point), at TAKEOFF_ALT; returns the wall time it took to get within 1 m."""
        self.mav.m.mav.set_position_target_local_ned_send(
            0,
            1,
            1,
            1,
            0b0000111111111000,  # MAV_FRAME_LOCAL_NED, use position only
            north,
            east,
            -TAKEOFF_ALT,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        t0 = time.time()

        def arrived():
            o = self.gz.latest_odom()  # Gazebo ENU: x = East, y = North
            return o is not None and math.hypot(o[1][1] - north, o[1][0] - east) < 1.0

        return (time.time() - t0) if self.wait(arrived, timeout, 0.1) else None

    def test_cruise(self, distance=30.0):
        """Forward flight with the payloads on: 30 m north and back at the default waypoint
        speed (WPNAV_SPEED, 10 m/s); records attitude, speed and height."""
        w0 = time.time()
        with self.gz.lock:
            start = len(self.gz.odom)
        t_out = self.goto_local(distance, 0.0)
        self.sim_sleep(2.0)
        t_back = self.goto_local(0.0, 0.0)
        self.sim_sleep(2.0)
        w1 = time.time()
        # ArduCopter turned the nose toward each target (WP_YAW_BEHAVIOR): face north again
        # (MAV_CMD_CONDITION_YAW: 0 deg absolute, 45 deg/s, shortest way)
        self.mav.command(115, 0.0, 45.0, 0.0, 0.0)

        def facing_north():
            with self.mav.lock:
                att_now = self.mav.attitude[-1] if self.mav.attitude else None
            return att_now is not None and abs(math.degrees(att_now[3])) < 3.0

        self.wait(facing_north, 60, 0.1)
        self.sim_sleep(2.0)
        att = self.mav.window(self.mav.attitude, w0, w1)
        alts = [s[1] for s in self.mav.window(self.mav.gpos, w0, w1)]
        with self.gz.lock:
            odo = list(self.gz.odom)[start:]
        speeds = [
            math.dist(a[1][:2], b[1][:2]) / (b[0] - a[0])
            for a, b in zip(odo[::10], odo[10::10])
            if b[0] > a[0]
        ]
        data = {
            "distance_m": distance,
            "time_out_s": round(t_out, 1) if t_out else None,
            "time_back_s": round(t_back, 1) if t_back else None,
            "max_ground_speed_m_s": round(max(speeds), 2) if speeds else None,
            "max_abs_roll_deg": round(max(abs(s[1]) for s in att) / DEG, 1) if att else None,
            "max_abs_pitch_deg": round(max(abs(s[2]) for s in att) / DEG, 1) if att else None,
            "alt_min_m": round(min(alts), 2) if alts else None,
            "alt_max_m": round(max(alts), 2) if alts else None,
        }
        ok = (
            t_out is not None
            and t_back is not None
            and data["max_abs_pitch_deg"] is not None
            and data["max_abs_pitch_deg"] < 35
            and abs(data["alt_min_m"] - TAKEOFF_ALT) < 2.0
            and abs(data["alt_max_m"] - TAKEOFF_ALT) < 2.0
        )
        self.record(
            "flight_cruise",
            ok,
            f"{distance:.0f} m north and back with both payloads: {data['time_out_s']} s / "
            f"{data['time_back_s']} s, top speed {data['max_ground_speed_m_s']} m/s, max |pitch| "
            f"{data['max_abs_pitch_deg']} deg, max |roll| {data['max_abs_roll_deg']} deg, height "
            f"{data['alt_min_m']}..{data['alt_max_m']} m",
            **data,
        )

    def land(self):
        t0 = time.time()
        self.mav.set_mode("LAND")
        disarmed = self.wait(lambda: not self.mav.armed, 600, 0.5)
        o = self.gz.latest_odom()
        self.record(
            "flight_land",
            bool(disarmed),
            f"LAND -> disarmed after {time.time() - t0:.0f} s, base_link z = {o[1][2]:.3f} m"
            if o
            else "no odometry",
            landed_z_m=o[1][2] if o else None,
        )

    # ------------------------------------------------------------------ gimbal
    def step(self, axis, target_rad, settle_s):
        """Command one joint, record its motion; returns (slew deg/s, steady error deg, samples)."""
        idx = {"yaw": 1, "roll": 2, "pitch": 3}[axis]
        j0 = self.gz.latest_joint()
        start = j0[idx]
        t_cmd = j0[0]
        self.gz.set_gimbal(**{axis: target_rad})
        self.sim_sleep(settle_s)
        with self.gz.lock:
            samples = [(s[0], s[idx]) for s in self.gz.joints if s[0] >= t_cmd]
        step_rad = target_rad - start
        lo, hi = start + 0.1 * step_rad, start + 0.9 * step_rad
        mid = [(t, v) for t, v in samples if min(lo, hi) <= v <= max(lo, hi)]
        slew = math.nan
        if len(mid) >= 3:
            ts, vs = np.array([m[0] for m in mid]), np.array([m[1] for m in mid])
            slew = abs(np.polyfit(ts, vs, 1)[0]) / DEG
        t_end = samples[-1][0] if samples else t_cmd
        tail = [abs(v - target_rad) / DEG for t, v in samples if t >= t_end - 0.5]
        steady = float(np.mean(tail)) if tail else math.nan
        return slew, steady, samples

    def analyze_frame(self, rgb, stamp, targets, cam_name, label):
        """Find coloured targets and compare with the projection; returns (ok, details)."""
        w, h, hfov = self.cam_cfg[cam_name]
        cam = PinholeCamera(w, h, hfov)
        od = self.gz.nearest(self.gz.odom, stamp)
        jt = self.gz.nearest(self.gz.joints, stamp)
        pose = self.kin.camera_pose(od[1], od[2], jt[1], jt[2], jt[3])
        axis = cam.optical_axis(pose)
        tol = PIXEL_TOL * w / 1280.0
        details, ok = {}, True
        for color, point in targets.items():
            pred = cam.project(pose, point)
            blob = find_blob(rgb, color)
            visible = pred is not None and 0 <= pred[0] < w and 0 <= pred[1] < h
            entry = {
                "predicted_px": None if pred is None else [round(pred[0], 1), round(pred[1], 1)],
                "found_px": None if blob is None else [round(blob[0], 1), round(blob[1], 1)],
                "pixels": None if blob is None else blob[2],
            }
            if visible and blob is not None:
                err = math.dist(pred, blob[:2])
                entry["error_px"] = round(err, 1)
                ok &= err <= tol
            elif visible != (blob is not None):
                ok = False
                entry["error_px"] = "missing" if visible else "unexpected"
            details[color] = entry
        details["optical_axis_world"] = [round(float(x), 3) for x in axis]
        details["angle_from_nadir_deg"] = round(
            math.degrees(math.acos(max(-1, min(1, -axis[2])))), 2
        )
        details["joints_deg"] = [round(jt[i] / DEG, 2) for i in (1, 2, 3)]
        details["tolerance_px"] = round(tol, 1)
        log(f"{label}: {json.dumps(details)}")
        return ok, details

    def snapshot(self, name, targets, label):
        sim = self.gz.sim_now()
        m = self.fresh_video_frame(sim + 0.1)
        if m is None:
            return False, {"error": "no video frame"}
        rgb = image_to_rgb(m)
        path = save_image(rgb, self.out(name))
        self.images.append(path)
        ok, det = self.analyze_frame(rgb, Gz._stamp(m), targets, "video_camera", label)
        det["image"] = path
        return ok, det

    def test_gimbal(self):
        self.gz.subscribe_video()
        self.gz.set_gimbal(0.0, 0.0, 0.0)
        self.sim_sleep(2.5)
        rows, frames, all_ok = {}, {}, True
        # 1. pitch straight down
        slew, err, _ = self.step("pitch", -90 * DEG, 2.5)
        rows["pitch_0_to_-90"] = (slew, err)
        ok, frames["down"] = self.snapshot("gimbal_pitch-90.png", GROUND_TARGETS, "pitch -90")
        all_ok &= ok and frames["down"]["angle_from_nadir_deg"] < 5.0
        # 2. yaw +30 (still looking down): the picture turns
        slew, err, _ = self.step("yaw", 30 * DEG, 2.0)
        rows["yaw_0_to_+30"] = (slew, err)
        ok, frames["down_yaw30"] = self.snapshot(
            "gimbal_pitch-90_yaw+30.png", GROUND_TARGETS, "pitch -90 yaw +30"
        )
        all_ok &= ok
        # 3. roll +15 and back
        slew, err, _ = self.step("roll", 15 * DEG, 1.5)
        rows["roll_0_to_+15"] = (slew, err)
        slew2, err2, _ = self.step("roll", 0.0, 1.5)
        rows["roll_+15_to_0"] = (slew2, err2)
        # 4. look straight ahead, then yaw +30: the scene must shift right
        self.step("yaw", 0.0, 1.5)
        slew, err, _ = self.step("pitch", 0.0, 2.5)
        rows["pitch_-90_to_0"] = (slew, err)
        ok, frames["forward"] = self.snapshot("gimbal_forward.png", POLE_TARGETS, "forward")
        all_ok &= ok
        slew, err, _ = self.step("yaw", 30 * DEG, 2.0)
        rows["yaw_0_to_+30_forward"] = (slew, err)
        ok, frames["forward_yaw30"] = self.snapshot(
            "gimbal_forward_yaw+30.png", POLE_TARGETS, "forward yaw +30"
        )
        all_ok &= ok
        if frames["forward"].get("red", {}).get("found_px") and frames["forward_yaw30"].get(
            "red", {}
        ).get("found_px"):
            shift = (
                frames["forward_yaw30"]["red"]["found_px"][0]
                - frames["forward"]["red"]["found_px"][0]
            )
            frames["red_pole_shift_px"] = round(shift, 1)
            w, _, hfov = self.cam_cfg["video_camera"]
            frames["red_pole_shift_expected_px"] = round(
                (w / 2) / math.tan(hfov / 2) * math.tan(30 * DEG), 1
            )
            all_ok &= shift > 0  # camera turned LEFT -> scene moves RIGHT
        else:
            all_ok = False  # the drone faces north here, so the red pole must be seen both times
        # back to looking down for the next tests
        self.gz.set_gimbal(0.0, 0.0, -90 * DEG)
        self.sim_sleep(2.0)
        slew_ok = all(SLEW_EXPECTED * 0.85 <= r[0] <= SLEW_EXPECTED * 1.15 for r in rows.values())
        err_ok = all(r[1] < STEADY_MAX_DEG for r in rows.values())
        table = {
            k: {"slew_deg_s": round(v[0], 1), "steady_error_deg": round(v[1], 3)}
            for k, v in rows.items()
        }
        self.record(
            "gimbal_steps",
            slew_ok and err_ok,
            "slew deg/s and steady error deg per step: "
            + ", ".join(
                f"{k}: {v['slew_deg_s']} / {v['steady_error_deg']}" for k, v in table.items()
            ),
            steps=table,
        )
        self.record(
            "gimbal_camera_views",
            all_ok,
            f"pitch -90: optical axis {frames['down'].get('angle_from_nadir_deg')} deg from nadir; "
            f"targets found where predicted (tolerance {frames['down'].get('tolerance_px')} px); "
            f"yaw +30 forward: red pole moved {frames.get('red_pole_shift_px')} px right "
            f"(expected about {frames.get('red_pole_shift_expected_px')})",
            frames=frames,
        )

    # ------------------------------------------------------------------ photo
    def test_photo(self):
        w, h, _ = self.cam_cfg["photo_camera"]
        time.sleep(1.0)
        with self.gz.lock:
            before = len(self.gz.photos)
        t0 = time.time()
        self.gz.trigger_photo()
        self.sim_sleep(2.0)
        self.wait(lambda: len(self.gz.photos) > before, 30)  # rendering may lag on a busy PC
        self.sim_sleep(2.0)  # a second photo would show up now
        with self.gz.lock:
            got = self.gz.photos[before:]
        sizes = [(m.width, m.height) for _, m in got]
        ok = len(got) == 1 and sizes[0] == (w, h)
        data = {"photos_received": len(got), "sizes": sizes, "expected": [w, h]}
        if got:
            m = got[0][1]
            rgb = image_to_rgb(m)
            path = save_image(rgb, self.out(f"photo_{w}x{h}.jpg"))
            self.images.append(path)
            data["image"] = path
            data["latency_s"] = round(got[0][0] - t0, 3)
            frame = [d.value[0] for d in m.header.data if d.key == "frame_id"]
            data["frame_id"] = frame[0] if frame else None
            vok, vdet = self.analyze_frame(
                rgb, Gz._stamp(m), GROUND_TARGETS, "photo_camera", "photo"
            )
            data["targets"] = vdet
            ok &= vok
        self.record(
            "photo_trigger",
            ok,
            f"1 trigger -> {len(got)} photo(s) {sizes} (expected 1 x {w}x{h}), "
            f"frame_id {data.get('frame_id')}",
            **data,
        )

    # ------------------------------------------------------------------ real-time factor
    def test_rtf(self):
        # video is subscribed since the gimbal test; add one photo per second
        with self.gz.lock:
            v0, p0 = self.gz.video_count, len(self.gz.photos)
        sim0 = self.gz.sim_now()
        w0 = time.time()
        for _ in range(20):
            self.gz.trigger_photo()
            time.sleep(1.0)
        w1 = time.time()
        sim1 = self.gz.sim_now()
        with self.gz.lock:
            v1, p1 = self.gz.video_count, len(self.gz.photos)
        rtf, rtf_rep, n = self.rtf_window(w0, w1)
        fps_sim = (v1 - v0) / max(1e-6, sim1 - sim0)
        fps_wall = (v1 - v0) / (w1 - w0)
        data = {
            "machine_load_1min": round(os.getloadavg()[0], 1),
            "cpus": os.cpu_count(),
            "rtf_with_cameras": round(rtf, 3) if rtf else None,
            "rtf_reported_mean": round(rtf_rep, 3) if rtf_rep else None,
            "rtf_no_camera_during_hover": round(self.hold_rtf, 3)
            if getattr(self, "hold_rtf", None)
            else None,
            "video_fps_sim_time": round(fps_sim, 2),
            "video_fps_wall": round(fps_wall, 2),
            "photos_in_20_triggers": p1 - p0,
        }
        # Functional: video keeps its 25 Hz in simulation time and every trigger gives a photo.
        # The RTF itself depends on how busy the computer is: below 0.5 on an overloaded machine
        # (load above the CPU count) the result is "partial", not "fail".
        functional = fps_sim > 20.0 and (p1 - p0) >= 18
        if functional and rtf is not None and rtf > 0.5:
            ok = True
        elif functional and os.getloadavg()[0] > (os.cpu_count() or 1):
            ok = "partial"
        else:
            ok = False
        self.record(
            "real_time_factor",
            ok,
            f"RTF {data['rtf_with_cameras']} with 1280x720 video subscribed + 1 photo/s "
            f"(no cameras: {data['rtf_no_camera_during_hover']}); video {data['video_fps_sim_time']} "
            f"fps sim time ({data['video_fps_wall']} fps wall); {p1 - p0}/20 photos; machine load "
            f"{data['machine_load_1min']} on {data['cpus']} CPUs",
            **data,
        )

    # ------------------------------------------------------------------ payload
    def drop(self, name, servo):
        q = self.gz.payload_odom[name]
        with self.gz.lock:
            z0 = q[-1][3] if q else None
        att0 = time.time()
        alt0 = self.mav.rel_alt()
        t_cmd = time.time()
        r_open = self.mav.command(183, servo, 1400)  # MAV_CMD_DO_SET_SERVO
        t_open = time.time()
        frames = []
        sim_open = self.gz.sim_now()
        for k, dt in enumerate((0.25, 0.5, 0.8)):
            m = self.fresh_video_frame(sim_open + dt, timeout=15.0)
            if m is not None:
                p = save_image(image_to_rgb(m), self.out(f"drop_{name}_{k}_{dt:.2f}s.png"))
                frames.append(p)
                self.images.append(p)

        def is_detached():
            with self.gz.lock:
                return any(s[1] == "detached" for s in self.gz.payload_state[name])

        detached = self.wait(is_detached, 15, 0.02)
        # latencies from the arrival times of the messages (wall clock, from the command)
        with self.gz.lock:
            t_det = next(
                (t for t, s in self.gz.payload_state[name] if s == "detached" and t >= t_cmd), None
            )
            t_servo = next(
                (t for t, v in self.gz.servo[str(servo)] if v >= 1399.5 and t >= t_cmd), None
            )
        detach_latency = (t_det - t_cmd) if t_det else None
        servo_latency = (t_servo - t_cmd) if t_servo else None

        def landed():
            with self.gz.lock:
                pts = [s for s in q if s[4] >= t_open]
            if len(pts) < 15:
                return None
            last = pts[-10:]
            if last[-1][3] < 0.3 and max(s[3] for s in last) - min(s[3] for s in last) < 0.01:
                return last[-1]
            return None

        land = self.wait(landed, 90, 0.2)
        time.sleep(max(0.0, 3.0 - (time.time() - t_open)))
        r_close = self.mav.command(183, servo, 500)
        time.sleep(3.0)
        # where is it 5 s (sim) after landing? (DART pushes an object that sank in on impact back
        # out slowly, so the resting height can still change a little)
        z_later = None
        if land is not None:
            self.sim_sleep(5.0)
            with self.gz.lock:
                z_later = q[-1][3] if q else None
        att = self.mav.window(self.mav.attitude, att0, time.time())
        roll = max(abs(s[1]) for s in att) / DEG if att else math.nan
        pitch = max(abs(s[2]) for s in att) / DEG if att else math.nan
        alt1 = self.mav.rel_alt()
        with self.gz.lock:
            servo_vals = sorted({round(v) for t, v in self.gz.servo[str(servo)] if t >= att0})
            pts = [s for s in q if s[4] >= t_open]
        t_ground = next((s[0] for s in pts if s[3] < 0.3), None)
        fall_time = (t_ground - pts[0][0]) if (pts and t_ground is not None) else None
        res = {
            "ack_open": r_open,
            "ack_close": r_close,
            "servo_values_seen_us": servo_vals,
            "detached": bool(detached),
            "servo_latency_s": round(servo_latency, 3) if servo_latency else None,
            "detach_latency_s": round(detach_latency, 3) if detach_latency else None,
            "z_5s_after_landing_m": round(z_later, 4) if z_later is not None else None,
            "z_before_m": round(z0, 3) if z0 is not None else None,
            "landed_at_enu": [round(land[1], 2), round(land[2], 2), round(land[3], 3)]
            if land
            else None,
            "fall_time_s": round(fall_time, 2) if fall_time is not None else None,
            "drone_max_abs_roll_deg": round(roll, 2),
            "drone_max_abs_pitch_deg": round(pitch, 2),
            "drone_alt_change_m": round(alt1 - alt0, 2) if alt0 and alt1 else None,
            "video_frames_during_fall": frames,
        }
        ok = (
            r_open == 0
            and r_close == 0
            and bool(detached)
            and land is not None
            and land[3] < 0.2
            and roll < 10
            and pitch < 10
            and 1400 in servo_vals
            and 500 in servo_vals
        )
        return ok, res

    def test_payload(self):
        self.gz.subscribe_dynamic_pose()
        self.gz.set_gimbal(0.0, 0.0, -90 * DEG)
        self.sim_sleep(1.5)
        ok_p, person = self.drop("person", 11)
        ok_t, tent = self.drop("tent", 13)
        node_log = ""
        if self.a.payload_log and os.path.isfile(self.a.payload_log):
            with open(self.a.payload_log, errors="replace") as f:
                node_log = f.read()
        logged = {
            k: (f'RELEASE payload "{k}"' in node_log, f'payload "{k}" LANDED' in node_log)
            for k in ("person", "tent")
        }
        with self.gz.lock:
            dyn = [s for s in self.gz.dyn_pose if s[1] == "payload_person"]
        dyn_note = None
        if dyn:
            z_first = dyn[0][4]
            dyn_note = f"dynamic_pose/info payload_person first z = {z_first:.3f} m " + (
                "(RELATIVE to the drone model, not world)" if z_first < 1.0 else "(world)"
            )
        self.record(
            "payload_person_drop",
            ok_p and all(logged["person"]),
            f"servo 11 1400 (ACK {person['ack_open']}) -> detached {person['detached']}, landed at "
            f"{person['landed_at_enu']} after {person['fall_time_s']} s; drone max roll/pitch "
            f"{person['drone_max_abs_roll_deg']}/{person['drone_max_abs_pitch_deg']} deg, alt change "
            f"{person['drone_alt_change_m']} m; servo values seen {person['servo_values_seen_us']}; "
            f"node log release/landed {logged['person']}",
            **person,
        )
        self.record(
            "payload_tent_drop",
            ok_t and all(logged["tent"]),
            f"servo 13 1400 (ACK {tent['ack_open']}) -> detached {tent['detached']}, landed at "
            f"{tent['landed_at_enu']} after {tent['fall_time_s']} s; drone max roll/pitch "
            f"{tent['drone_max_abs_roll_deg']}/{tent['drone_max_abs_pitch_deg']} deg; servo values "
            f"{tent['servo_values_seen_us']}; node log release/landed {logged['tent']}",
            **tent,
        )
        if dyn_note:
            self.results["payload_person_drop"]["dynamic_pose_note"] = dyn_note
            log(dyn_note)

    # ------------------------------------------------------------------ main
    def run(self):
        tests = set(self.a.tests.split(","))
        if not self.wait(lambda: self.gz.latest_odom() and self.gz.latest_joint(), 300):
            self.record(
                "gazebo_topics", False, "no /sim/drone/odometry or /sim/gimbal/joint_states"
            )
            return
        self.record(
            "gazebo_topics",
            self.gz.subscribed_ok,
            "subscribed: /sim/drone/odometry, /sim/gimbal/joint_states, camera, payload topics",
        )
        self.inspection_shot("model_on_ground.png")
        self.mav = Mav(self.a.mav)
        log("waiting for the SITL heartbeat")
        self.mav.start()
        self.test_parm()
        if not self.arm_and_takeoff():
            return
        try:
            self.test_hold()
            if "gimbal" in tests:
                self.test_gimbal()
            if "photo" in tests:
                self.test_photo()
            if "rtf" in tests:
                if self.gz.video_count == 0:
                    self.gz.subscribe_video()
                self.test_rtf()
            if "cruise" in tests:  # after the camera tests: the drone turns while it travels
                self.test_cruise()
            if "payload" in tests:
                if self.gz.video_count == 0:
                    self.gz.subscribe_video()
                self.test_payload()
        finally:
            self.land()
            self.inspection_shot("model_after_landing.png")

    def inspection_shot(self, name):
        """Save one picture of the drone from the test world's fixed inspection camera."""
        m = self.gz.inspection_photo()
        if m is None:
            log(f"inspection camera: no picture for {name}")
            return
        path = save_image(image_to_rgb(m), self.out(name))
        self.images.append(path)
        log(f"inspection camera: saved {path}")

    def write(self):
        with open(self.out("results.json"), "w") as f:
            json.dump({"results": self.results, "images": self.images}, f, indent=2, default=str)
        if self.mav is not None:
            with self.mav.lock, open(self.out("statustext.log"), "w") as f:
                for t, text in self.mav.statustexts:
                    f.write(f"{time.strftime('%H:%M:%S', time.localtime(t))} {text}\n")
        lines = [
            f"{k:24s} {v['result'].upper():5s} {v['evidence']}" for k, v in self.results.items()
        ]
        with open(self.out("summary.txt"), "w") as f:
            f.write("\n".join(lines) + "\n")
            f.write("images:\n" + "\n".join(f"  {p}" for p in self.images) + "\n")
        return all(v["result"] == "pass" for v in self.results.values()) and bool(self.results)


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    p.add_argument("--mav", default="tcp:127.0.0.1:5770", help="SITL MAVLink address")
    p.add_argument("--world", default="sim_models_test")
    p.add_argument("--model-sdf", required=True, help="the generated SDF that was spawned")
    p.add_argument("--camera-model", default="zr10")
    p.add_argument("--parm", required=True, help="astra_sitl.parm (values are read back from SITL)")
    p.add_argument("--tests", default="flight,cruise,gimbal,photo,rtf,payload")
    p.add_argument("--payload-log", default="", help="payload_release log file to check")
    p.add_argument("--out", required=True)
    a = p.parse_args()
    os.makedirs(a.out, exist_ok=True)
    acc = Acceptance(a)
    try:
        acc.run()
    except Exception as exc:  # noqa: BLE001 - always write what we have
        import traceback

        traceback.print_exc()
        acc.record("driver", False, f"exception: {exc}")
    finally:
        if acc.mav:
            acc.mav.stop()
    ok = acc.write()
    log(f"ALL {'PASSED' if ok else 'NOT PASSED'}; see {acc.out('summary.txt')}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
