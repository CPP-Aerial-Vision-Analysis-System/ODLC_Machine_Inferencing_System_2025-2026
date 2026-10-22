"""
sim_demo/ros_common.py -- PURPOSE: ROS 2 pieces shared by all four sim_demo nodes: the QoS
profiles, and FlightData, a helper that subscribes to the MAVROS topics describing the drone
(state, GPS, height, heading, speed, mission progress, status texts) and keeps them in one
thread-safe place together with a PoseHistory (where the drone was over time) and a
ZoomTracker (how far the camera was zoomed).

QoS choice: every MAVROS/sensor subscription is BEST_EFFORT, which is compatible with both
BEST_EFFORT and RELIABLE publishers, so a QoS mismatch can never silently cut a topic.
"""
from __future__ import annotations

import json
import math
import threading
import time
from typing import Callable

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .geo import quaternion_to_yaw_deg, enu_yaw_to_compass_deg
from .pose_history import PoseHistory, PoseSample
from .zoom import ZoomTracker

NAN = float('nan')

# BEST_EFFORT: receives from reliable AND best-effort publishers.
QOS_SENSOR = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
# Same as new_od's publisher and main_controller's subscriber for /image_detection.
QOS_DETECTIONS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                            history=HistoryPolicy.KEEP_LAST, depth=10)
# For things a late subscriber (e.g. RViz started later) should still get.
QOS_LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
QOS_IMAGE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=2)
# siyi_emulator publishes /sim/camera/photo_info RELIABLE + TRANSIENT_LOCAL (depth 10).
QOS_PHOTO_INFO = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            history=HistoryPolicy.KEEP_LAST, depth=50)
EMULATOR_STATE_TOPIC = '/sim/camera/emulator_state'
PHOTO_INFO_TOPIC = '/sim/camera/photo_info'


class FlightData:
    """Subscribes to the MAVROS topics about the drone and keeps the latest values.

    Read the attributes under `lock` (or use snapshot()). Optional callbacks are called from the
    ROS executor thread right after the stored values were updated:
        on_statustext(t, text, source)   source = 'fcu' (/recv) or 'team' (/send)
        on_state(t, connected, armed, mode)
        on_reached(t, seq)
        on_rel_alt(t, alt)
        on_waypoints(t, items)           items = list of (command, lat, lon, alt)
        on_camera_status(t, text)
    It also listens to the camera emulator's simulator-only topics: /sim/camera/emulator_state
    (zoom and gimbal, kept in `camera_state`, zoom fed to the ZoomTracker) and, when a
    photo_info index is given, /sim/camera/photo_info (exact record of each photo).
    """

    def __init__(self, node: Node, zoom_fixed: float = 0.0, history_s: float = 1800.0,
                 on_statustext: Callable | None = None, on_state: Callable | None = None,
                 on_reached: Callable | None = None, on_rel_alt: Callable | None = None,
                 on_waypoints: Callable | None = None, on_camera_status: Callable | None = None,
                 subscribe_camera_topics: bool = True, photo_info=None):
        from mavros_msgs.msg import State, StatusText, VfrHud, WaypointList, WaypointReached
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import NavSatFix
        from std_msgs.msg import Float64, String

        self.node = node
        self.lock = threading.Lock()
        self.history = PoseHistory(max_age_s=history_s)
        self.zoom = ZoomTracker(zoom_fixed)
        self.connected = False
        self.armed = False
        self.mode = ''
        self.state_t = 0.0
        self.lat: float | None = None
        self.lon: float | None = None
        self.alt_msl = NAN
        self.fix_t = 0.0
        self.rel_alt = NAN
        self.heading = NAN          # compass degrees
        self.heading_src = ''
        self.ground_speed = NAN
        self.climb = NAN
        self.last_reached = -1
        self.waypoints: list[tuple[int, float, float, float]] = []
        self.statustexts: list[tuple[float, str, str]] = []   # (t, source, text), last 500
        self.camera_state: dict = {}     # latest /sim/camera/emulator_state (zoom, gimbal, ...)
        self.camera_state_t = 0.0
        self.photo_info = photo_info
        self._cb = dict(statustext=on_statustext, state=on_state, reached=on_reached,
                        rel_alt=on_rel_alt, waypoints=on_waypoints, camera_status=on_camera_status)

        sub = node.create_subscription
        sub(State, '/mavros/state', self._on_state, QOS_SENSOR)
        sub(NavSatFix, '/mavros/global_position/global', self._on_fix, QOS_SENSOR)
        sub(Float64, '/mavros/global_position/rel_alt', self._on_rel_alt, QOS_SENSOR)
        sub(Float64, '/mavros/global_position/compass_hdg', self._on_hdg, QOS_SENSOR)
        sub(VfrHud, '/mavros/vfr_hud', self._on_vfr, QOS_SENSOR)
        sub(PoseStamped, '/mavros/local_position/pose', self._on_local_pose, QOS_SENSOR)
        sub(WaypointReached, '/mavros/mission/reached', self._on_reached, QOS_SENSOR)
        sub(WaypointList, '/mavros/mission/waypoints', self._on_waypoints, QOS_SENSOR)
        sub(StatusText, '/mavros/statustext/recv', lambda m: self._on_text(m, 'fcu'), QOS_SENSOR)
        sub(StatusText, '/mavros/statustext/send', lambda m: self._on_text(m, 'team'), QOS_SENSOR)
        if subscribe_camera_topics:
            sub(String, '/camera/command', self._on_camera_command, QOS_SENSOR)
            sub(String, '/camera/status', self._on_camera_status, QOS_SENSOR)
        sub(String, EMULATOR_STATE_TOPIC, self._on_emulator_state, QOS_SENSOR)
        if photo_info is not None:
            sub(String, PHOTO_INFO_TOPIC, lambda m: photo_info.add_json(str(m.data)), QOS_PHOTO_INFO)

    # ------------------------------------------------------------------ callbacks
    def _call(self, name: str, *args) -> None:
        cb = self._cb.get(name)
        if cb is not None:
            try:
                cb(*args)
            except Exception as exc:  # noqa: BLE001 - a display bug must not stop data flow
                self.node.get_logger().error(f'{name} handler failed: {exc}')

    def _on_state(self, msg) -> None:
        t = time.time()
        with self.lock:
            self.connected, self.armed, self.mode, self.state_t = (
                bool(msg.connected), bool(msg.armed), str(msg.mode), t)
        self._call('state', t, bool(msg.connected), bool(msg.armed), str(msg.mode))

    def _on_fix(self, msg) -> None:
        t = time.time()
        lat, lon = float(msg.latitude), float(msg.longitude)
        if (lat == 0.0 and lon == 0.0) or math.isnan(lat) or math.isnan(lon):
            return    # "Null Island": no GPS lock yet (same rule as video_cam)
        with self.lock:
            self.lat, self.lon, self.alt_msl, self.fix_t = lat, lon, float(msg.altitude), t
            self.history.add(PoseSample(t=t, lat=lat, lon=lon, alt_msl=float(msg.altitude),
                                        rel_alt=self.rel_alt, heading_deg=self.heading,
                                        ground_speed=self.ground_speed))

    def _on_rel_alt(self, msg) -> None:
        t = time.time()
        with self.lock:
            self.rel_alt = float(msg.data)
        self._call('rel_alt', t, float(msg.data))

    def _on_hdg(self, msg) -> None:
        with self.lock:
            self.heading, self.heading_src = float(msg.data) % 360.0, 'compass_hdg'

    def _on_vfr(self, msg) -> None:
        with self.lock:
            self.ground_speed = float(msg.groundspeed)
            self.climb = float(msg.climb)
            if self.heading_src in ('', 'vfr_hud', 'local_pose'):
                self.heading, self.heading_src = float(msg.heading) % 360.0, 'vfr_hud'

    def _on_local_pose(self, msg) -> None:
        q = msg.pose.orientation
        with self.lock:
            if self.heading_src in ('', 'local_pose'):
                yaw = quaternion_to_yaw_deg(q.x, q.y, q.z, q.w)
                self.heading, self.heading_src = enu_yaw_to_compass_deg(yaw), 'local_pose'

    def _on_reached(self, msg) -> None:
        t = time.time()
        with self.lock:
            self.last_reached = int(msg.wp_seq)
        self._call('reached', t, int(msg.wp_seq))

    def _on_waypoints(self, msg) -> None:
        t = time.time()
        items = [(int(w.command), float(w.x_lat), float(w.y_long), float(w.z_alt))
                 for w in msg.waypoints]
        with self.lock:
            self.waypoints = items
        self._call('waypoints', t, items)

    def _on_text(self, msg, source: str) -> None:
        t = time.time()
        text = str(msg.text)
        self.zoom.on_statustext(t, text)
        with self.lock:
            self.statustexts.append((t, source, text))
            if len(self.statustexts) > 500:
                del self.statustexts[:100]
        self._call('statustext', t, text, source)

    def _on_camera_command(self, msg) -> None:
        self.zoom.on_camera_command(time.time(), str(msg.data))

    def _on_emulator_state(self, msg) -> None:
        t = time.time()
        try:
            state = json.loads(str(msg.data))
        except ValueError:
            return
        if not isinstance(state, dict):
            return
        zoom = (state.get('zoom') or {}).get('current')
        if isinstance(zoom, (int, float)):
            self.zoom.on_emulator_zoom(t, float(zoom))
        with self.lock:
            self.camera_state, self.camera_state_t = state, t

    def _on_camera_status(self, msg) -> None:
        self._call('camera_status', time.time(), str(msg.data))

    # ------------------------------------------------------------------ reading
    def snapshot(self) -> dict:
        """A consistent copy of the latest values (safe to use from any thread)."""
        with self.lock:
            cam = self.camera_state if time.time() - self.camera_state_t < 5.0 else {}
            gimbal = cam.get('gimbal') or {}
            zoom = cam.get('zoom') or {}
            return dict(connected=self.connected, armed=self.armed, mode=self.mode,
                        state_t=self.state_t, lat=self.lat, lon=self.lon, alt_msl=self.alt_msl,
                        fix_t=self.fix_t, rel_alt=self.rel_alt, heading=self.heading,
                        ground_speed=self.ground_speed, climb=self.climb,
                        last_reached=self.last_reached,
                        cam_pitch=gimbal.get('pitch_deg'), cam_yaw=gimbal.get('yaw_deg'),
                        cam_zoom=zoom.get('current'), cam_model=cam.get('camera_model'))

    def latest_pose(self) -> PoseSample | None:
        """Newest recorded pose sample (or None)."""
        with self.lock:
            return self.history.latest()

    def capture_pose(self, origin, lat, lon, file_time: float, latency_s: float = 1.0):
        """Thread-safe PoseHistory.capture_pose (see pose_history.py)."""
        with self.lock:
            return self.history.capture_pose(origin, lat, lon, file_time, latency_s=latency_s)


def detector_publishers(node: Node, topic: str = '/image_detection') -> list[str]:
    """Names of the nodes currently publishing the detector topic (e.g. ['new_od'])."""
    try:
        return sorted({info.node_name for info in node.get_publishers_info_by_topic(topic)})
    except Exception:  # noqa: BLE001 - graph queries can fail during shutdown
        return []
