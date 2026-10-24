"""
sim_demo/demo_panel.py -- PURPOSE: the window people WATCH during the simulated search
("ASTRA Simulation - Search Demo"). It shows, live:
  * a LARGE camera point-of-view picture (/sim/camera/image, the gimbal camera in Gazebo) with
    height, speed, heading and flight mode on top, and a white flash when a photo is taken;
  * the newest photo from the team's photo folder with the detector's boxes drawn on it
    (boxes come from the detector's /image_detection message for that photo's file name);
  * the list of detections (class, confidence, latitude/longitude, photo) and the detection
    main_controller will fly to for each class;
  * the mission phase strip (Waiting > Taking off > Searching (lane i of n) > Processing
    photos > Flying to target > Dropping > Returning > Landed);
  * telemetry and an event log (autopilot + team status texts, camera replies, detections);
  * big buttons: TAKE PHOTO (/camera/trigger, the team's own shutter topic), LOOK DOWN /
    LOOK FORWARD (the team's /camera/command protocol), UPLOAD MISSION + START (uploads the
    world's mission, waits out the settling window, arms, starts AUTO) and RTL;
  * START OVER (click it twice within 5 s): puts the whole run back to the start while every
    window stays open (simulation/scripts/start_over.sh -> run_reset.py: a flying drone lands, a
    fresh drone with both payloads, a fresh autopilot and team stack, a new run folder). Its steps
    appear in the Events list and the status line; the mission buttons stay locked meanwhile, and
    the old run's phases, detections and photos leave the display;
  * OPEN / CLOSE MISSION PLANNER: starts or closes the Windows ground station through WSL, with
    a "Mission Planner: running / not running" light and the connect hint (mission_planner.py).
A big amber banner says so whenever the truth_detector stand-in (not the AI model) is the
detector. Nothing blocks the window: ROS spins on a background thread, the autopilot commands
run on their own thread, the Mission Planner checks and buttons on another one, and START OVER
is a separate background program. It starts fine with nothing running ("waiting for ...").

Run:  ros2 run sim_demo demo_panel --ros-args -p world:=runway
Screenshot without a screen:
      QT_QPA_PLATFORM=offscreen ros2 run sim_demo demo_panel --ros-args \
          -p screenshot_path:=/tmp/panel.png -p screenshot_after_s:=6.0 -p quit_after_screenshot:=true
"""
from __future__ import annotations

import math
import os
import re
import signal
import sys
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .detections import DETECTION_TOPIC, DetectionFrame, best_per_class, image_result_to_frame
from .mission_control import MissionCommander
from .mission_file import analyze, analyze_items, load_waypoints_file
from .mission_planner import CONNECT_HINT, MissionPlannerMonitor
from .phases import DROPPING, FLYING_TO_TARGET, PHASES, PROCESSING, PhaseTracker
from .photos import PhotoWatcher, list_photos, resolve_photo_dir
from .ros_common import QOS_DETECTIONS, QOS_IMAGE, QOS_SENSOR, FlightData, detector_publishers
from .world_info import load_world

WINDOW_TITLE = 'ASTRA Simulation - Search Demo'
QUIET_TEAM_TEXTS = ('No objects detected', 'SUCCESS: Captured', 'Simulation photo captured')
#: START OVER needs a second click within this many seconds (a click by mistake must not end a flight).
RESET_CONFIRM_S = 5.0
#: After the second click, start_over.sh must write its first progress line within this time [s].
RESET_REPLY_S = 60.0
RESET_TIP = ('Put the whole run back to the start, every window stays open: a flying drone lands first, then a '
             'fresh drone with both payloads, a fresh autopilot and team code, and a new run folder. Click twice '
             '(within 5 s).')


def prepare_qt_environment() -> str:
    """Make PyQt5 start reliably. Call BEFORE creating the QApplication.

    * The pip build of OpenCV (cv2) sets QT_QPA_PLATFORM_PLUGIN_PATH / QT_QPA_FONTDIR to its
      own bundled Qt plugins when anything imports cv2; the system PyQt5 then crashes
      ("could not load the Qt platform plugin xcb"). Remove those two variables if they
      point into a cv2 folder.
    * With no screen at all (no DISPLAY / WAYLAND_DISPLAY), draw offscreen instead of
      aborting, so screenshots still work.
    Returns a note for the log ('' if nothing was changed).
    """
    notes = []
    for var in ('QT_QPA_PLATFORM_PLUGIN_PATH', 'QT_QPA_FONTDIR'):
        if 'cv2' in os.environ.get(var, ''):
            os.environ.pop(var)
            notes.append(f'removed {var} (set by OpenCV)')
    if (not os.environ.get('QT_QPA_PLATFORM') and not os.environ.get('DISPLAY')
            and not os.environ.get('WAYLAND_DISPLAY')):
        os.environ['QT_QPA_PLATFORM'] = 'offscreen'
        notes.append('no display found: drawing offscreen (use screenshot_path to see it)')
    return '; '.join(notes)


def detector_status(publishers: list[str], recent_frames: list) -> tuple[str, str, str, bool]:
    """Which detector is working, for the DETECTOR pill and the stand-in banner.

    Decided by the messages actually received (their `method`), because sim_bringup's
    detector:=truth mode keeps new_od running but idle (watching an empty folder): a publisher
    that never publishes must not count as a second detector.

    Args:
        publishers: node names publishing /image_detection. recent_frames: latest DetectionFrames.
    Returns:
        (pill_state, pill_text, tooltip, stand_in_active)
    """
    truth_msgs = any(f.is_stand_in for f in recent_frames)
    real_msgs = any(not f.is_stand_in for f in recent_frames)
    idle_note = ' (new_od also runs, idle)' if 'new_od' in publishers else ''
    if truth_msgs and real_msgs:
        return 'bad', 'TWO detectors!', 'both new_od and truth_detector publish results', True
    if truth_msgs:
        return 'warn', 'STAND-IN', 'truth_detector (answer key)' + idle_note, True
    if real_msgs:
        return 'ok', 'REAL (new_od)', 'detection/new_od YOLO model', False
    # Nothing received yet: go by who is listening/publishing.
    if 'truth_detector' in publishers:
        return 'warn', 'STAND-IN', 'truth_detector (answer key), no photo yet' + idle_note, True
    if 'new_od' in publishers:
        return 'ok', 'REAL (new_od)', 'detection/new_od, no photo yet', False
    if publishers:
        return 'ok', publishers[0], ', '.join(publishers), False
    return 'off', 'not running', 'nobody publishes /image_detection', False


class MavrosAdapter:
    """MAVROS service calls used as the fallback path of the mission buttons (thread-safe:
    call it from any thread except the ROS executor thread)."""

    def __init__(self, node: Node):
        from mavros_msgs.srv import CommandBool, CommandLong, SetMode, WaypointPull, WaypointPush
        self._types = dict(CommandBool=CommandBool, CommandLong=CommandLong, SetMode=SetMode,
                           WaypointPull=WaypointPull, WaypointPush=WaypointPush)
        self.c_mode = node.create_client(SetMode, '/mavros/set_mode')
        self.c_arm = node.create_client(CommandBool, '/mavros/cmd/arming')
        self.c_cmd = node.create_client(CommandLong, '/mavros/cmd/command')
        self.c_push = node.create_client(WaypointPush, '/mavros/mission/push')
        self.c_pull = node.create_client(WaypointPull, '/mavros/mission/pull')

    def available(self) -> bool:
        """True when MAVROS answers (its set_mode service exists)."""
        return self.c_mode.service_is_ready()

    @staticmethod
    def _call(client, req, timeout: float = 10.0):
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        fut = client.call_async(req)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            time.sleep(0.05)
        return fut.result() if fut.done() else None

    def push_mission(self, items) -> bool:
        """Upload mission items through /mavros/mission/push."""
        from mavros_msgs.msg import Waypoint
        req = self._types['WaypointPush'].Request()
        req.start_index = 0
        for it in items:
            w = Waypoint()
            w.frame, w.command = int(it.frame), int(it.command)
            w.is_current = bool(it.seq == 0)
            w.autocontinue = bool(it.autocontinue)
            w.param1, w.param2, w.param3, w.param4 = it.param1, it.param2, it.param3, it.param4
            w.x_lat, w.y_long, w.z_alt = float(it.lat), float(it.lon), float(it.alt)
            req.waypoints.append(w)
        resp = self._call(self.c_push, req, 30.0)
        return bool(resp and resp.success)

    def pull(self) -> bool:
        """Ask MAVROS to re-read the mission from the autopilot."""
        resp = self._call(self.c_pull, self._types['WaypointPull'].Request(), 15.0)
        return bool(resp and resp.success)

    def set_mode(self, name: str) -> bool:
        """Change flight mode (ArduPilot names: GUIDED, AUTO, RTL ...)."""
        req = self._types['SetMode'].Request()
        req.custom_mode = name
        resp = self._call(self.c_mode, req, 5.0)
        return bool(resp and resp.mode_sent)

    def arm(self) -> tuple[bool, str]:
        """Try to arm. Returns (accepted, reason)."""
        req = self._types['CommandBool'].Request()
        req.value = True
        resp = self._call(self.c_arm, req, 5.0)
        if resp is None:
            return False, 'MAVROS arming service did not answer'
        return bool(resp.success), '' if resp.success else f'arming refused (result {resp.result})'

    def command_long(self, cmd: int, params) -> bool:
        """Send a MAVLink COMMAND_LONG through MAVROS."""
        req = self._types['CommandLong'].Request()
        req.command = int(cmd)
        p = (list(params) + [0.0] * 7)[:7]
        (req.param1, req.param2, req.param3, req.param4, req.param5, req.param6,
         req.param7) = [float(v) for v in p]
        resp = self._call(self.c_cmd, req, 5.0)
        return bool(resp and resp.success)


class StartOverLink:
    """The panel's side of START OVER (the work itself is sim_demo/run_reset.py).

    launch() starts simulation/scripts/start_over.sh --no-fresh in the background, detached from the
    panel (its own session: it keeps running even if the panel closes, and stop_sim.sh still finds it by
    the simulator's session tag). poll() reads the progress lines it writes (run_reset.ProgressTracker).
    The panel's own Qt thread only reads a small file here, so the window never waits for the reset.
    Tests pass a temporary sim home and a fake launcher, so no script ever runs.
    """

    def __init__(self, home=None, launcher=None):
        from . import run_reset
        self._rr = run_reset
        self._launcher = launcher or run_reset.launch_start_over
        self.tracker = run_reset.ProgressTracker(run_reset.progress_path(Path(home) if home else
                                                                         run_reset.sim_home()))
        self.tracker.skip_existing()       # restore an active reset's lockout without replaying history
        self.pid = 0
        self.log_path = ''

    @property
    def busy(self) -> bool:
        """True while a START OVER runs (also one started from a terminal with start_over.sh)."""
        return self.tracker.busy

    @property
    def step_label(self) -> str:
        """'5/14' while it runs ('' before its first step)."""
        t = self.tracker
        return f'{t.n}/{t.total}' if t.running and t.total else ''

    def launch(self) -> str:
        """Start start_over.sh in the background; returns its log file. Raises OSError if bash is missing."""
        self.pid, self.log_path = self._launcher(['--no-fresh'])
        return self.log_path

    def poll(self) -> list:
        """New progress (run_reset.ProgressUpdate list, oldest first)."""
        return self.tracker.poll()

    def ended_without_progress(self) -> str:
        """After launch(): '' while the script still runs; else the last line of its log (it ended before
        writing any progress, e.g. 'sim_demo is not built')."""
        if not self.pid or self._rr.ProcessOps().alive(self.pid):
            return ''
        try:
            lines = [ln.strip() for ln in Path(self.log_path).read_text(encoding='utf-8', errors='replace')
                     .splitlines() if ln.strip()]
        except OSError:
            lines = []
        # (the scripts colour their messages: drop the terminal colour codes for the panel)
        last = re.sub(r'\x1b\[[0-9;]*m', '', lines[-1]).strip() if lines else ''
        return last or 'no output'


class PanelNode(Node):
    """ROS side of the panel: subscriptions, publishers, mission state (no Qt here)."""

    def __init__(self):
        super().__init__('demo_panel')
        p = self.declare_parameter
        p('world', 'runway')
        p('worlds_dir', '')
        p('truth_file', '')
        p('photo_dir', '')
        p('camera_topic', '/sim/camera/image')
        p('detector', 'auto')                       # auto / real / truth (label hint only)
        p('mavlink_url', 'udpin:127.0.0.1:14553')   # extra MAVProxy output for the panel
        p('mission_link', 'auto')                   # auto / mavlink / mavros
        p('auto_start', False)                      # press START automatically
        p('auto_start_delay_s', 5.0)
        p('settle_timeout_s', 150.0)
        p('look_down_on_start', True)
        p('look_down_command', 'gimbal_set_angles 0,-90')
        p('look_forward_command', 'gimbal_set_angles 0,0')
        p('hit_hold_s', 8.0)                        # Auto photo view keeps a hit this long
        p('window_width', 1600)
        p('window_height', 960)
        p('fullscreen', False)
        p('screenshot_path', '')                    # save PNG(s) of the window (docs / tests)
        p('screenshot_after_s', 6.0)                # first screenshot this long after start
        p('screenshot_every_s', 0.0)                # > 0: repeat (files get _01, _02 ... suffixes)
        p('screenshot_count', 1)                    # how many screenshots in total
        p('quit_after_screenshot', False)           # quit after the last screenshot
        p('mission_planner_exe', '')                # '' = $MISSION_PLANNER_EXE or the usual folder
        gp = self.get_parameter
        self.cfg = {name: gp(name).value for name in (
            'world', 'camera_topic', 'detector', 'mavlink_url', 'mission_link', 'auto_start',
            'auto_start_delay_s', 'settle_timeout_s', 'look_down_on_start', 'look_down_command',
            'look_forward_command', 'hit_hold_s', 'window_width', 'window_height', 'fullscreen',
            'screenshot_path', 'screenshot_after_s', 'screenshot_every_s', 'screenshot_count',
            'quit_after_screenshot', 'mission_planner_exe')}
        log = self.get_logger()
        self.lock = threading.Lock()
        self.events: deque = deque(maxlen=600)     # (t, source, text, level)
        self.world = None
        self.world_error = ''
        try:
            self.world = load_world(str(gp('world').value), str(gp('worlds_dir').value),
                                    str(gp('truth_file').value))
            for w in self.world.warnings:
                self.event('panel', w, 'warn')
        except Exception as exc:  # noqa: BLE001
            self.world_error = str(exc)
            self.event('panel', f'World not loaded: {exc}', 'warn')
        self.mission_items = []
        self.mission_summary = None                # kept for reset_run (START OVER)
        self.phases = PhaseTracker()
        if self.world and self.world.mission_path:
            try:
                self.mission_items = load_waypoints_file(self.world.mission_path)
                self.mission_summary = analyze_items(self.mission_items, self.world.hold_index)
                self.phases.set_mission(self.mission_summary)
            except Exception as exc:  # noqa: BLE001
                self.event('panel', f'Mission file not readable: {exc}', 'warn')
        self.photo_dir, how = resolve_photo_dir(str(gp('photo_dir').value))
        self.photo_dir_how = how
        self.frames: list[DetectionFrame] = []
        self.frame_queue: deque = deque()          # new frames for the window to pick up
        self.frame_by_name: dict[str, DetectionFrame] = {}
        self.last_hit: DetectionFrame | None = None
        self.last_hit_t = 0.0
        self.image_msg = None
        self.image_seq = 0
        self.image_times: deque = deque(maxlen=60)
        self.trigger_count = 0
        self.detector_nodes: list[str] = []

        self.flight = FlightData(self, on_statustext=self._on_text, on_state=self._on_state,
                                 on_reached=self._on_reached, on_rel_alt=self._on_rel_alt,
                                 on_waypoints=self._on_waypoints,
                                 on_camera_status=self._on_camera_status)
        from interfaces.msg import ImageResult
        from sensor_msgs.msg import Image
        from std_msgs.msg import Bool, String
        self._Bool, self._String = Bool, String
        self.create_subscription(Image, str(self.cfg['camera_topic']), self._on_image, QOS_IMAGE)
        self.create_subscription(ImageResult, DETECTION_TOPIC, self._on_detection, QOS_DETECTIONS)
        self.create_subscription(Bool, '/camera/trigger', self._on_trigger, QOS_SENSOR)
        self.trigger_pub = self.create_publisher(Bool, '/camera/trigger', 10)
        self.command_pub = self.create_publisher(String, '/camera/command', 10)
        self.mavros = MavrosAdapter(self)
        self.create_timer(2.0, self._refresh_detectors)
        log.info(f'Photo folder: {self.photo_dir} ({how})')

    # ------------------------------------------------------------------ helpers
    def event(self, source: str, text: str, level: str = '') -> None:
        """Add a line to the event log (thread-safe)."""
        self.events.append((time.time(), source, text, level))

    def reset_run(self) -> None:
        """START OVER began or finished: forget the old run (mission phases, detections, shutter count).
        The subscriptions stay: the new autopilot and team stack come back on the same topics."""
        with self.lock:
            self.phases = PhaseTracker()
            if self.mission_summary is not None:
                self.phases.set_mission(self.mission_summary)
            self.frames, self.frame_by_name = [], {}
            self.frame_queue.clear()
            self.last_hit, self.last_hit_t = None, 0.0
            self.trigger_count = 0

    def _refresh_detectors(self) -> None:
        nodes = detector_publishers(self, DETECTION_TOPIC)
        with self.lock:
            self.detector_nodes = nodes

    # ------------------------------------------------------------------ ROS callbacks
    def _on_image(self, msg) -> None:
        with self.lock:
            self.image_msg = msg
            self.image_seq += 1
            self.image_times.append(time.time())

    def _on_trigger(self, msg) -> None:
        if msg.data:
            with self.lock:
                self.trigger_count += 1     # every shutter press (panel or mapping trigger)

    def _on_detection(self, msg) -> None:
        f = image_result_to_frame(msg)
        with self.lock:
            self.frames.append(f)
            self.frame_queue.append(f)
            self.frame_by_name[f.image_name] = f
            if len(self.frames) > 3000:
                del self.frames[:500]
            if f.boxes:
                self.last_hit, self.last_hit_t = f, time.time()
        if f.boxes:
            found = ', '.join(f'{b.cls.upper()} {b.score * 100:.0f}%' for b in f.boxes)
            tag = ' (stand-in)' if f.is_stand_in else ''
            self.event('detector', f'{found} in {f.image_name}{tag}')

    def _on_text(self, t, text, source) -> None:
        with self.lock:
            self.phases.on_statustext(t, text)
        if source == 'team' and text.startswith(QUIET_TEAM_TEXTS):
            return
        self.event('autopilot' if source == 'fcu' else 'team', text,
                   'warn' if text.lower().startswith(('prearm', 'arm:', 'failed', 'warn')) else '')

    def _on_state(self, t, connected, armed, mode) -> None:
        with self.lock:
            before = (self.phases.connected, self.phases.armed, self.phases.mode)
            self.phases.on_state(t, connected, armed, mode)
        if before[0] != connected:
            self.event('panel', 'MAVROS connected to the autopilot' if connected
                       else 'MAVROS lost the autopilot', '' if connected else 'warn')
        if before[1] != armed:
            self.event('autopilot', 'ARMED - motors on' if armed else 'DISARMED - motors off')
        if before[2] != mode.upper() and mode:
            self.event('autopilot', f'Flight mode: {mode}')

    def _on_reached(self, t, seq) -> None:
        with self.lock:
            self.phases.on_reached(t, seq)

    def _on_rel_alt(self, t, alt) -> None:
        with self.lock:
            self.phases.on_rel_alt(t, alt)

    def _on_waypoints(self, t, items) -> None:
        hold = self.world.hold_index if self.world else -1
        with self.lock:
            if self.phases.phase in PHASES[:3]:
                self.phases.set_mission(analyze([c for c, _, _, _ in items],
                                                [a for _, _, _, a in items], hold))

    def _on_camera_status(self, t, text) -> None:
        self.event('camera', text, 'warn' if 'FAIL' in text or 'ERROR' in text else '')

    # ------------------------------------------------------------------ actions (any thread)
    def take_photo(self) -> None:
        """TAKE PHOTO: publish true on the team's /camera/trigger topic."""
        self.trigger_pub.publish(self._Bool(data=True))
        self.event('panel', 'TAKE PHOTO -> /camera/trigger (video_cam takes the photo)')

    def camera_command(self, text: str, label: str) -> None:
        """Publish a /camera/command string (team protocol, e.g. 'gimbal_set_angles 0,-90')."""
        self.command_pub.publish(self._String(data=text))
        self.event('panel', f'{label} -> /camera/command "{text}"')


class DemoWindow:
    """The Qt window (built lazily so importing this module does not need a display).

    mission_planner: the worker behind OPEN / CLOSE MISSION PLANNER (default: a new
    mission_planner.MissionPlannerMonitor). Tests pass a fake with the same five methods
    (start, stop, snapshot, request_open, request_close), so they never touch Windows.
    start_over: the link behind START OVER (default: a StartOverLink on the simulator's state
    folder). Tests pass a StartOverLink with a temporary folder and a fake launcher.
    """

    def __init__(self, node: PanelNode, app, mission_planner=None, start_over=None):
        from PyQt5 import QtCore, QtGui, QtWidgets

        from . import panel_widgets as pw
        self.QtCore, self.QtGui, self.QtWidgets, self.pw = QtCore, QtGui, QtWidgets, pw
        self.node = node
        self.app = app
        cfg = node.cfg
        self.win = QtWidgets.QMainWindow()
        self.win.setWindowTitle(WINDOW_TITLE)
        self.win.resize(int(cfg['window_width']), int(cfg['window_height']))
        self.win.setMinimumSize(1200, 760)
        app.setStyleSheet(pw.STYLE_SHEET)
        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(10, 8, 10, 10)
        root.setSpacing(6)

        # ---- header: title, world, status pills
        head = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel('ASTRA SEARCH DEMO')
        title.setObjectName('title')
        world_name = node.world.name if node.world else f'(world "{cfg["world"]}" not found)'
        targets = ''
        if node.world and node.world.targets:
            counts = {}
            for t in node.world.targets:
                counts[t.cls] = counts.get(t.cls, 0) + 1
            targets = ' | hidden: ' + ', '.join(
                f'{n} {"hiker" if c == "person" else c}{"s" if n > 1 else ""}' for c, n in counts.items())
        sub = QtWidgets.QLabel(f'world: {world_name}{targets}')
        sub.setObjectName('subtitle')
        head.addWidget(title)
        head.addSpacing(14)
        head.addWidget(sub)
        head.addStretch(1)
        self.pill_mavros = pw.StatusPill('MAVROS')
        self.pill_camera = pw.StatusPill('CAMERA')
        self.pill_photos = pw.StatusPill('PHOTOS')
        self.pill_detector = pw.StatusPill('DETECTOR')
        for pill in (self.pill_mavros, self.pill_camera, self.pill_photos, self.pill_detector):
            head.addWidget(pill)
        root.addLayout(head)
        self.banner = QtWidgets.QLabel(
            'TRUTH STAND-IN ACTIVE: the boxes come from the simulator\'s answer key, NOT from the '
            'AI model. (Start the demo with detector:=real to test the team\'s new_od model.)')
        self.banner.setWordWrap(True)
        self.banner.setStyleSheet(f'QLabel {{ background: {pw.THEME["amber"]}; color: #111; '
                                  'font-weight: bold; font-size: 14px; padding: 6px 10px; '
                                  'border-radius: 6px; }')
        self.banner.hide()
        root.addWidget(self.banner)

        # ---- phase strip
        self.phase_strip = pw.PhaseStrip(PHASES)
        root.addWidget(self.phase_strip)

        # ---- middle: live view | newest photo + detections
        self.video = pw.VideoView()
        self.video.set_placeholder(f'Waiting for the camera picture on {cfg["camera_topic"]} ...\n'
                                   '(is the simulator running?)')
        self.photo = pw.PhotoView()
        self.photo.set_placeholder('No photos yet.\nPhotos appear here when the drone takes them.\n'
                                   f'Folder: {node.photo_dir}')
        mode_box = QtWidgets.QWidget()
        mlay = QtWidgets.QHBoxLayout(mode_box)
        mlay.setContentsMargins(0, 0, 0, 0)
        mlay.setSpacing(4)
        self.mode_buttons = {}
        group = QtWidgets.QButtonGroup(mode_box)
        group.setExclusive(True)
        for key, text in (('auto', 'Auto'), ('newest', 'Newest'), ('hit', 'Last hit')):
            b = QtWidgets.QPushButton(text)
            b.setObjectName('modeBtn')
            b.setCheckable(True)
            b.clicked.connect(lambda _c=False, k=key: self._set_photo_mode(k))
            group.addButton(b)
            mlay.addWidget(b)
            self.mode_buttons[key] = b
        self.mode_buttons['auto'].setChecked(True)
        self.photo_mode = 'auto'
        self.photo_reason = ''         # extra caption text, e.g. "tent main_controller chose"
        self.pinned_name = ''

        self.best_label = QtWidgets.QLabel('main_controller will fly to: nothing yet')
        self.best_label.setObjectName('bestLine')
        self.best_label.setTextFormat(QtCore.Qt.RichText)
        self.best_label.setWordWrap(True)
        self.table = QtWidgets.QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(['Time', 'Class', 'Confidence', 'Latitude',
                                              'Longitude', 'Photo'])
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        hh.setStretchLastSection(True)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.cellClicked.connect(self._on_row_clicked)
        det_box = QtWidgets.QWidget()
        dlay = QtWidgets.QVBoxLayout(det_box)
        dlay.setContentsMargins(0, 0, 0, 0)
        dlay.addWidget(self.best_label)
        dlay.addWidget(self.table, 1)
        self.det_count = QtWidgets.QLabel('0')
        self.det_count.setObjectName('cardTitle')

        right = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        right.addWidget(pw.card('Newest photo  +  detector boxes', self.photo, mode_box))
        right.addWidget(pw.card('Detections', det_box, self.det_count))
        right.setSizes([520, 360])
        middle = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        middle.addWidget(pw.card('Live camera (gimbal point of view)', self.video))
        middle.addWidget(right)
        middle.setSizes([960, 620])
        root.addWidget(middle, 1)

        # ---- bottom: telemetry | controls | events
        tele = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(tele)
        grid.setContentsMargins(4, 0, 4, 0)
        grid.setHorizontalSpacing(18)
        grid.setVerticalSpacing(2)
        self.tele = {}
        names = [('link', 'MAVROS link'), ('mode', 'Flight mode'), ('armed', 'Motors'),
                 ('alt', 'Height above home'), ('speed', 'Ground speed'), ('hdg', 'Heading'),
                 ('gps', 'GPS position'), ('photos', 'Photos saved (shutter presses)')]
        for i, (key, label) in enumerate(names):
            r, c = (i % 4) * 2, i // 4
            n = QtWidgets.QLabel(label)
            n.setObjectName('teleName')
            v = QtWidgets.QLabel('--')
            v.setObjectName('teleValue')
            grid.addWidget(n, r, c)
            grid.addWidget(v, r + 1, c)
            self.tele[key] = v
        controls = QtWidgets.QWidget()
        clay = QtWidgets.QGridLayout(controls)
        clay.setContentsMargins(2, 2, 2, 2)
        clay.setSpacing(6)
        self.btn_photo = QtWidgets.QPushButton('TAKE PHOTO')
        self.btn_photo.setObjectName('photoBtn')
        self.btn_photo.setMinimumHeight(46)
        self.btn_down = QtWidgets.QPushButton('LOOK DOWN')
        self.btn_fwd = QtWidgets.QPushButton('LOOK FORWARD')
        self.btn_start = QtWidgets.QPushButton('UPLOAD MISSION + START')
        self.btn_start.setObjectName('startBtn')
        self.btn_rtl = QtWidgets.QPushButton('RTL (FLY HOME)')
        self.btn_rtl.setObjectName('rtlBtn')
        for b in (self.btn_down, self.btn_fwd, self.btn_start, self.btn_rtl):
            b.setMinimumHeight(38)
        # START OVER: right under the mission buttons, across both columns, in its own colour. It needs a
        # second click within RESET_CONFIRM_S (the text then says so): no dialog box to click away.
        self.btn_reset = QtWidgets.QPushButton('START OVER')
        self.btn_reset.setObjectName('resetBtn')
        self.btn_reset.setMinimumHeight(34)
        self.btn_reset.setToolTip(RESET_TIP)
        clay.addWidget(self.btn_photo, 0, 0, 1, 2)
        clay.addWidget(self.btn_down, 1, 0)
        clay.addWidget(self.btn_fwd, 1, 1)
        clay.addWidget(self.btn_start, 2, 0)
        clay.addWidget(self.btn_rtl, 2, 1)
        clay.addWidget(self.btn_reset, 3, 0, 1, 2)
        # Mission Planner (the Windows ground station) can be opened and closed from here. The
        # checks and presses run on a worker thread (mission_planner.MissionPlannerMonitor):
        # starting a Windows program takes seconds, and the window must never freeze.
        self.btn_mp_open = QtWidgets.QPushButton('OPEN MISSION PLANNER')
        self.btn_mp_close = QtWidgets.QPushButton('CLOSE MISSION PLANNER')
        # Own row layout across both grid columns: the two equal halves never squeeze the
        # columns above (UPLOAD MISSION + START needs the wider left column).
        mp_row = QtWidgets.QHBoxLayout()
        mp_row.setSpacing(6)
        for b in (self.btn_mp_open, self.btn_mp_close):
            b.setObjectName('mpBtn')       # smaller text: the long names fit side by side
            b.setMinimumHeight(30)
            b.setEnabled(False)            # until the first check says they can work here
            mp_row.addWidget(b, 1)
        clay.addLayout(mp_row, 4, 0, 1, 2)
        self.mp_hint = QtWidgets.QLabel(CONNECT_HINT)
        self.mp_hint.setObjectName('mpHint')
        clay.addWidget(self.mp_hint, 5, 0, 1, 2)
        self.cmd_status = QtWidgets.QLabel('Ready.')
        self.cmd_status.setObjectName('cmdStatus')
        self.cmd_status.setWordWrap(True)
        clay.addWidget(self.cmd_status, 6, 0, 1, 2)
        self.btn_photo.clicked.connect(self._on_photo)
        self.btn_down.clicked.connect(
            lambda: node.camera_command(str(cfg['look_down_command']), 'LOOK DOWN'))
        self.btn_fwd.clicked.connect(
            lambda: node.camera_command(str(cfg['look_forward_command']), 'LOOK FORWARD'))
        self.btn_start.clicked.connect(self._on_start)
        self.btn_rtl.clicked.connect(self._on_rtl)
        self.btn_reset.clicked.connect(self._on_reset)
        self.btn_mp_open.clicked.connect(self._on_mp_open)
        self.btn_mp_close.clicked.connect(self._on_mp_close)
        self._start_tip = ''               # UPLOAD MISSION + START's tooltip when START OVER does not lock it
        if not node.mission_items:
            self.btn_start.setEnabled(False)
            self._start_tip = 'No mission file found for this world.'
            self.btn_start.setToolTip(self._start_tip)
        self.log = pw.EventLog()
        bottom = QtWidgets.QHBoxLayout()
        bottom.setSpacing(8)
        # The Mission Planner light sits in the Controls card's title row (no extra height).
        self.pill_mp = pw.StatusPill('Mission Planner')
        self.pill_mp.set_state('off', 'checking ...')
        t_card = pw.card('Telemetry', tele)
        c_card = pw.card('Controls', controls, self.pill_mp)
        l_card = pw.card('Events', self.log)
        t_card.setMinimumWidth(380)
        c_card.setMinimumWidth(420)        # every button label fits, also in the smallest window
        self.controls_card = c_card
        bottom.addWidget(t_card, 3)
        bottom.addWidget(c_card, 3)
        bottom.addWidget(l_card, 5)
        bw = QtWidgets.QWidget()
        bw.setLayout(bottom)
        bw.setFixedHeight(312)             # 214 + the Mission Planner row and its hint + the START OVER row
        root.addWidget(bw)
        self.win.setCentralWidget(central)

        # ---- state for the update loops
        self.watcher = PhotoWatcher(node.photo_dir, report_existing=False)
        self.photo_names: list[str] = [os.path.basename(p) for _, p in list_photos(node.photo_dir)]
        self.preexisting = len(self.photo_names)
        self.new_photos = 0
        self.shown_photo = ''
        self.shown_frame_id = None
        self.photo_cache: dict[str, tuple] = {}
        self.loading: set[str] = set()
        self.loaded_q: deque = deque()     # finished photo loads (worker thread -> Qt thread)
        self._photo_gen = 0                # +1 at START OVER: a load that finishes later is from the old run
        self.loader = ThreadPoolExecutor(max_workers=1, thread_name_prefix='photo_loader')
        self.last_image_seq = -1
        self.tele['gps'].setStyleSheet('font-size: 15px;')
        self.commander = MissionCommander(str(cfg['mavlink_url']), self._cmd_status_cb,
                                          node.mavros, str(cfg['mission_link']))
        self.commander.start()
        # Mission Planner worker: checks every 2 s on its own thread; its result sentences go
        # through _cmd_status_cb (status line + event log), like the mission buttons' messages.
        if mission_planner is None:
            mission_planner = MissionPlannerMonitor(self._cmd_status_cb,
                                                    str(cfg.get('mission_planner_exe', '')))
        self.mission_planner = mission_planner
        self.mission_planner.start()
        self._mp_shown = None              # the PlannerState drawn last (redraw only on change)
        # START OVER: the background script's progress file is read every 0.5 s (_poll_start_over).
        try:
            self.start_over = start_over or StartOverLink()
        except Exception as exc:  # noqa: BLE001 - e.g. run_reset not importable: the panel works without it
            self.start_over = None
            self.btn_reset.setEnabled(False)
            self.btn_reset.setToolTip(f'START OVER is not available here ({exc}).')
        self._reset_armed_until = 0.0      # first click seen: a second one before this time confirms
        self._reset_launched_t = 0.0       # second click: waiting for start_over.sh's first progress line
        self._reset_shown = None           # what the START OVER button shows (redraw only on change)
        self._cmd_text = ('Ready. UPLOAD MISSION + START flies the search; TAKE PHOTO works '
                          'any time.')
        if node.world_error:
            self._cmd_text = f'World not loaded: {node.world_error}'
        # An already-running reset owns the controls; show its current action even if this newly opened
        # panel could not load its world metadata. The world warning remains in the Events list.
        if self.start_over is not None and self.start_over.busy:
            self._cmd_text = f'START OVER {self.start_over.step_label}: {self.start_over.tracker.step_text}'

        self._timers = []
        for ms, fn in ((40, self._update_video), (250, self._update_ui),
                       (500, self._poll_photos), (500, self._poll_start_over), (200, lambda: None)):
            tm = QtCore.QTimer(self.win)
            tm.timeout.connect(fn)
            tm.start(ms)
            self._timers.append(tm)
        if cfg['auto_start'] and node.mission_items:
            QtCore.QTimer.singleShot(int(float(cfg['auto_start_delay_s']) * 1000), self._on_start)
        if cfg['screenshot_path']:
            QtCore.QTimer.singleShot(int(float(cfg['screenshot_after_s']) * 1000), self.screenshot)
        node.event('panel', f'Panel ready. Photos are read from {node.photo_dir}')

    # ------------------------------------------------------------------ actions
    def _set_photo_mode(self, mode: str) -> None:
        self.photo_mode, self.pinned_name = mode, ''
        self.shown_photo = ''          # force a refresh

    def _on_row_clicked(self, row: int, _col: int) -> None:
        item = self.table.item(row, 5)
        if item is not None:
            self.pinned_name = item.text()
            self.photo_mode = 'pinned'
            for b in self.mode_buttons.values():
                b.setAutoExclusive(False)
                b.setChecked(False)
                b.setAutoExclusive(True)
            self.shown_photo = ''

    def _on_photo(self) -> None:
        self.node.take_photo()
        self.video.flash('PHOTO REQUESTED')

    def _cmd_status_cb(self, text: str, level: str) -> None:
        self._cmd_text = text
        self.node.event('panel', text, {'error': 'error', 'warn': 'warn'}.get(level, ''))

    def _on_start(self) -> None:
        node = self.node
        if self._resetting():
            self._cmd_text = 'START OVER is still running: UPLOAD MISSION + START works again when it is done.'
            return
        if self.commander.busy:
            self._cmd_text = 'A start sequence is already running ...'
            return
        before = None
        if node.cfg['look_down_on_start']:
            def before():
                node.camera_command(str(node.cfg['look_down_command']), 'LOOK DOWN (before take-off)')
        self.commander.request_start(node.mission_items, float(node.cfg['settle_timeout_s']), before)
        self._cmd_text = 'Start sequence running ...'

    def _on_rtl(self) -> None:
        self.commander.request_rtl()
        self.node.event('panel', 'RTL requested')

    # ------------------------------------------------------------------ START OVER
    def _resetting(self) -> bool:
        """True from the confirming click until START OVER has finished (also one started in a terminal)."""
        if self.start_over is None:
            return False
        return self.start_over.busy or self._reset_launched_t > 0.0

    def _on_reset(self) -> None:
        """START OVER: the first click asks for a second one within RESET_CONFIRM_S; the second click starts
        start_over.sh in the background (returns at once; the steps then arrive in _poll_start_over)."""
        if self.start_over is None or self._resetting():
            return
        if self.commander.busy:
            self._cmd_text = ('UPLOAD MISSION + START is still running: wait until it has finished (or press RTL), '
                              'then START OVER.')
            return
        now = time.time()
        if now > self._reset_armed_until:
            self._reset_armed_until = now + RESET_CONFIRM_S
            self._cmd_text = (f'START OVER puts everything back to the start (2-3 min): a flying drone lands first, '
                              f'then a fresh drone, autopilot and team code. Click it again within '
                              f'{RESET_CONFIRM_S:.0f} s to go ahead.')
            self.node.event('panel', f'START OVER clicked once: click again within {RESET_CONFIRM_S:.0f} s to '
                                     'confirm')
            return
        self._reset_armed_until = 0.0
        try:
            log = self.start_over.launch()
        except Exception as exc:  # noqa: BLE001 - e.g. bash missing: say so, the panel keeps working
            self._cmd_text = f'START OVER could not start: {exc}'
            self.node.event('panel', self._cmd_text, 'error')
            return
        self._reset_launched_t = now
        self._cmd_text = 'START OVER started: every window stays open. Its steps appear in the Events list.'
        self.node.event('panel', f'START OVER confirmed -> simulation/scripts/start_over.sh (log: {log})')

    def _poll_start_over(self) -> None:
        """Qt timer (0.5 s): show START OVER's new progress lines, clear the old run at its start and end."""
        so = self.start_over
        if so is None:
            return
        try:
            updates = so.poll()
        except Exception as exc:  # noqa: BLE001 - an unreadable progress file must not stop the panel
            updates = []
            self.node.get_logger().warn(f'START OVER progress not readable: {exc}')
        for u in updates:
            if u.new_reset:
                # From now on the old run is history: its photos are being archived and its detections
                # would mislead (the new run has none yet).
                self._reset_launched_t = 0.0
                self._reset_display()
            if u.detail:                       # a tool's progress line: status line only
                self._cmd_text = f'START OVER {so.step_label}: {u.text}'
                continue
            level = {'warn': 'warn', 'error': 'error'}.get(u.level, '')
            self.node.event('start over', u.text, level)
            if u.finished is True:
                self._reset_display()          # the new run starts clean (nothing from the landing etc.)
                folder = os.path.basename(so.tracker.run_dir.rstrip('/'))
                self._cmd_text = (('START OVER done: the drone flies the search again' if 'is flying' in u.text
                                   else 'START OVER done: ready. Press UPLOAD MISSION + START (or Write, Arm, '
                                        'Auto in Mission Planner)') + (f'. New run: {folder}' if folder else '.'))
            elif u.finished is False:
                self._cmd_text = f'START OVER FAILED: {u.text.split(": ", 1)[-1]}'
            elif u.kind == 'step' or so.busy:
                self._cmd_text = f'START OVER {so.step_label}: {u.text}'
            else:
                self._cmd_text = u.text        # e.g. the run watcher's "Run finished" line
        if self._reset_launched_t and not updates:
            why = so.ended_without_progress()
            if why or time.time() - self._reset_launched_t > RESET_REPLY_S:
                self._reset_launched_t = 0.0
                self._cmd_text = (f'START OVER did not start: {why or "no reply"} (log: {so.log_path})')
                self.node.event('panel', self._cmd_text, 'error')

    def _reset_display(self) -> None:
        """Forget the old run on screen: phases, detections, photo list and caches (the windows stay)."""
        self.node.reset_run()
        self.table.setRowCount(0)
        self.best_label.setText('main_controller will fly to: nothing yet')
        self.photo_names, self.new_photos, self.preexisting = [], 0, 0
        self.photo_cache.clear()
        self.loading.clear()
        self.loaded_q.clear()
        self._photo_gen += 1
        self.shown_photo, self.shown_frame_id, self.pinned_name = '', None, ''
        if self.photo_mode == 'pinned':
            self._set_photo_mode('auto')
            self.mode_buttons['auto'].setChecked(True)
        self.photo.set_photo(None, (0, 0), '', [], '', self.pw.THEME['muted'])

    def _update_reset_button(self) -> None:
        """START OVER's text, colour and clickability; UPLOAD MISSION + START is locked while it runs."""
        if self.start_over is None:
            return
        now = time.time()
        resetting = self._resetting()
        if self._reset_armed_until and (resetting or now > self._reset_armed_until):
            if not resetting:
                self._cmd_text = 'START OVER not confirmed (no second click within 5 s): nothing was changed.'
            self._reset_armed_until = 0.0
        if resetting:
            label = self.start_over.step_label
            state, text, enabled, tip = ('busy', f'STARTING OVER {label} ...' if label else 'STARTING OVER ...',
                                         False, 'START OVER is running: its steps appear in the Events list.')
        elif self._reset_armed_until:
            left = max(1, math.ceil(self._reset_armed_until - now))
            state, text, enabled, tip = ('armed', f'CLICK AGAIN TO START OVER ({left} s)', True,
                                         'Click again to start over now.')
        elif self.commander.busy:
            state, text, enabled, tip = ('idle', 'START OVER', False,
                                         'Wait until UPLOAD MISSION + START has finished (or press RTL).')
        else:
            state, text, enabled, tip = 'idle', 'START OVER', True, RESET_TIP
        start_ok = bool(self.node.mission_items) and not resetting
        shown = (state, text, enabled, tip, start_ok)
        if shown == self._reset_shown:
            return                             # nothing changed since the last tick: skip restyling
        self._reset_shown = shown
        self.btn_reset.setText(text)
        self.btn_reset.setEnabled(enabled)
        self.btn_reset.setToolTip(tip)
        if bool(self.btn_reset.property('armed')) != (state == 'armed'):
            # The style sheet has an armed colour (QPushButton#resetBtn[armed="true"]): re-read it.
            self.btn_reset.setProperty('armed', state == 'armed')
            self.btn_reset.style().unpolish(self.btn_reset)
            self.btn_reset.style().polish(self.btn_reset)
        self.btn_start.setEnabled(start_ok)
        self.btn_start.setToolTip('Wait until START OVER is done.' if resetting else self._start_tip)

    def _on_mp_open(self) -> None:
        """OPEN MISSION PLANNER: hand the press to the worker thread (returns at once)."""
        if self.mission_planner.request_open():
            self._cmd_text = 'Opening Mission Planner on Windows ...'
            self.node.event('panel', 'OPEN MISSION PLANNER -> starting MissionPlanner.exe on Windows')

    def _on_mp_close(self) -> None:
        """CLOSE MISSION PLANNER: hand the press to the worker thread (returns at once)."""
        if self.mission_planner.request_close():
            self._cmd_text = 'Closing Mission Planner on Windows ...'
            self.node.event('panel', 'CLOSE MISSION PLANNER -> asking MissionPlanner.exe to close')

    def screenshot(self) -> None:
        """Save the window as a PNG (parameter screenshot_path); repeat every screenshot_every_s
        until screenshot_count pictures exist (numbered _01, _02 ...), optionally then quit."""
        cfg = self.node.cfg
        path = str(cfg['screenshot_path'])
        count = max(1, int(cfg['screenshot_count']))
        self._shots = getattr(self, '_shots', 0) + 1
        if count > 1:
            stem, ext = os.path.splitext(path)
            path = f'{stem}_{self._shots:02d}{ext or ".png"}'
        try:
            os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
            ok = self.win.grab().save(path)
            self.node.get_logger().info(f'Screenshot {"saved" if ok else "FAILED"}: {path}')
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().error(f'Screenshot failed: {exc}')
        every = float(cfg['screenshot_every_s'])
        if self._shots < count and every > 0:
            self.QtCore.QTimer.singleShot(int(every * 1000), self.screenshot)
        elif cfg['quit_after_screenshot']:
            self.QtCore.QTimer.singleShot(200, self.app.quit)

    # ------------------------------------------------------------------ update loops
    def _update_video(self) -> None:
        node = self.node
        with node.lock:
            msg, seq = node.image_msg, node.image_seq
            times = list(node.image_times)
        if msg is None or seq == self.last_image_seq:
            return
        self.last_image_seq = seq
        img = self.pw.ros_image_to_qimage(msg.encoding, msg.width, msg.height, msg.step, msg.data)
        if img is None:
            self.video.set_placeholder(f'Camera picture has an unsupported encoding: {msg.encoding}')
            return
        fps = 0.0
        if len(times) >= 2 and times[-1] > times[0]:
            fps = (len(times) - 1) / (times[-1] - times[0])
        self.video.set_frame(img, fps)

    def _poll_photos(self) -> None:
        for ev in self.watcher.poll():
            if ev.preexisting:
                continue
            self.photo_names.append(ev.name)
            self.new_photos += 1
            self.video.flash(f'PHOTO #{self.new_photos}')
        if len(self.photo_names) > 5000:
            del self.photo_names[:1000]

    def _load_photo(self, name: str, gen: int = 0):
        """Worker thread: read + downscale a photo (JPEG decoder scales while decoding). `gen` = the
        photo generation it was asked for (START OVER starts a new one, see _drain_loaded_photos)."""
        QtGui, QtCore = self.QtGui, self.QtCore
        path = os.path.join(self.node.photo_dir, name)
        reader = QtGui.QImageReader(path)
        size = reader.size()
        orig = (size.width(), size.height())
        if size.width() > 1600:
            reader.setScaledSize(size.scaled(1600, 1600, QtCore.Qt.KeepAspectRatio))
        img = reader.read()
        return name, (img if not img.isNull() else None), orig, gen

    def _drain_loaded_photos(self) -> None:
        """Qt thread: take finished photo loads from the worker and cache them."""
        while self.loaded_q:
            fut = self.loaded_q.popleft()
            try:
                name, img, orig, gen = fut.result()
            except Exception:  # noqa: BLE001 - unreadable file: try again next time
                continue
            if gen != self._photo_gen:
                continue                   # a photo of the run before START OVER: never show it again
            self.loading.discard(name)
            if img is not None:
                self.photo_cache[name] = (img, orig)
                if len(self.photo_cache) > 12:
                    self.photo_cache.pop(next(iter(self.photo_cache)))
                self.shown_photo = ''      # redraw with the image

    def _choose_photo(self, last_hit, last_hit_t, frames, phase_idx, target_cls) -> str:
        """Which photo to show. Auto: while the drone goes to a target, the photo of the
        detection it flies to; else a fresh hit (for hit_hold_s); else the newest photo."""
        self.photo_reason = ''
        if self.photo_mode == 'pinned' and self.pinned_name:
            return self.pinned_name
        if self.photo_mode == 'hit':
            return last_hit.image_name if last_hit else ''
        newest = self.photo_names[-1] if self.photo_names else ''
        if self.photo_mode == 'auto':
            if PHASES[phase_idx] in (PROCESSING, FLYING_TO_TARGET, DROPPING):
                best = best_per_class(frames)
                cls = target_cls if target_cls in best else next(
                    (c for c in ('tent', 'person') if c in best), None)
                if cls is not None:
                    self.photo_reason = {PROCESSING: f'best {cls} so far',
                                         FLYING_TO_TARGET: f'flying to this {cls}',
                                         DROPPING: f'dropping on this {cls}'}[PHASES[phase_idx]]
                    return best[cls][1].image_name
            if last_hit is not None and time.time() - last_hit_t < float(self.node.cfg['hit_hold_s']):
                return last_hit.image_name
        return newest

    def _update_photo(self, frame_by_name, last_hit, last_hit_t, detector_nodes, frames=(),
                      phase_idx=0, target_cls='') -> None:
        pw = self.pw
        name = self._choose_photo(last_hit, last_hit_t, list(frames), phase_idx, target_cls)
        if not name:
            return
        frame = frame_by_name.get(name)
        frame_id = (id(frame) if frame else None, self.photo_mode, self.photo_reason)
        if name == self.shown_photo and frame_id == self.shown_frame_id:
            return
        cached = self.photo_cache.get(name)
        if cached is None:
            if name not in self.loading:
                self.loading.add(name)
                # The worker only appends to a deque; the Qt timer picks the result up.
                self.loader.submit(self._load_photo, name, self._photo_gen).add_done_callback(
                    self.loaded_q.append)
            return
        img, orig = cached
        boxes, extra = [], []
        if frame is None:
            if detector_nodes:
                status, color = 'detector: checking this photo ...', pw.THEME['amber']
            else:
                status, color = 'no detector running', pw.THEME['red']
        else:
            boxes = frame.boxes
            if boxes:
                status = f'{len(boxes)} target{"s" if len(boxes) > 1 else ""} found'
                color = pw.THEME['green']
            else:
                status, color = 'no targets in this photo', pw.THEME['muted']
                if frame.classes:
                    extra = ['Detector list (no person/tent boxes in the message):'] + [
                        f'{c} {s:.2f}' for c, s in zip(frame.classes, frame.confidences)]
            if frame.is_stand_in:
                status += '  (stand-in)'
        idx = ''
        if name in self.photo_names:
            idx = f'photo {self.photo_names.index(name) + 1}/{len(self.photo_names)}  '
        mode = {'auto': 'AUTO', 'newest': 'NEWEST', 'hit': 'LAST HIT', 'pinned': 'PINNED'}[
            self.photo_mode]
        if self.photo_reason:
            mode = self.photo_reason.upper()
        self.photo.set_photo(img, orig, f'[{mode}] {idx}{name}', boxes, status, color,
                             highlight=bool(boxes), extra_lines=extra)
        self.shown_photo, self.shown_frame_id = name, frame_id

    def _update_ui(self) -> None:
        node = self.node
        pw = self.pw
        snap = node.flight.snapshot()
        self._drain_loaded_photos()
        with node.lock:
            ph = node.phases
            phase_idx, detail = ph.phase_index, ph.detail
            visited = {PHASES.index(e.data['phase']) for e in ph.history}
            visited.add(0)
            since = ph.phase_since
            frames = list(node.frames)
            new = list(node.frame_queue)
            node.frame_queue.clear()
            frame_by_name = dict(node.frame_by_name)
            last_hit, last_hit_t = node.last_hit, node.last_hit_t
            detector_nodes = list(node.detector_nodes)
            img_times = list(node.image_times)
            triggers = node.trigger_count
            target_cls = ph.current_target
        since_txt = ''
        if since:
            s = int(time.time() - since)
            since_txt = f'for {s // 60}:{s % 60:02d}'
        self.phase_strip.set_state(phase_idx, visited, detail, since_txt)

        # Telemetry + HUD.
        alt = snap['rel_alt'] if not math.isnan(snap['rel_alt']) else None
        spd = snap['ground_speed'] if not math.isnan(snap['ground_speed']) else None
        hdg = snap['heading'] if not math.isnan(snap['heading']) else None
        link_ok = snap['connected'] and time.time() - snap['state_t'] < 5.0
        self.tele['link'].setText('connected' if link_ok else 'waiting ...')
        self.tele['link'].setStyleSheet(f'color: {pw.THEME["green"] if link_ok else pw.THEME["amber"]}')
        self.tele['mode'].setText(snap['mode'] or '--')
        self.tele['armed'].setText('ARMED' if snap['armed'] else 'off')
        self.tele['armed'].setStyleSheet(f'color: {pw.THEME["red"] if snap["armed"] else pw.THEME["text"]}')
        self.tele['alt'].setText('--' if alt is None else f'{alt:.1f} m  ({alt * 3.28084:.0f} ft)')
        self.tele['speed'].setText('--' if spd is None else f'{spd:.1f} m/s')
        self.tele['hdg'].setText('--' if hdg is None else f'{hdg:03.0f}°')
        self.tele['gps'].setText('--' if snap['lat'] is None else f'{snap["lat"]:.6f}, {snap["lon"]:.6f}')
        # e.g. "23  (81)": 23 photos arrived, the shutter was pressed 81 times (the team's
        # video_cam drops presses while its lens worker is busy - useful to see).
        photos_txt = f'{self.new_photos}' + (f'  ({triggers})' if triggers else '')
        self.tele['photos'].setText(photos_txt)
        self.tele['photos'].setToolTip(
            f'{self.new_photos} photos saved this run, {triggers} shutter presses on /camera/trigger'
            + (f', {self.preexisting} older photos were already in the folder' if self.preexisting
               else ''))
        self.video.set_hud({'alt_m': alt, 'speed': spd, 'heading': hdg, 'mode': snap['mode'],
                            'cam_pitch': snap.get('cam_pitch'), 'cam_zoom': snap.get('cam_zoom'),
                            **({'armed': snap['armed']} if link_ok else {})})

        # Pills.
        self.pill_mavros.set_state('ok' if link_ok else 'warn', 'connected' if link_ok else 'waiting',
                                   'MAVROS /mavros/state')
        fresh = img_times and time.time() - img_times[-1] < 3.0
        self.pill_camera.set_state('ok' if fresh else 'warn', 'live' if fresh else 'no picture',
                                   node.cfg['camera_topic'])
        folder_ok = os.path.isdir(node.photo_dir)
        self.pill_photos.set_state('ok' if self.new_photos else ('warn' if folder_ok else 'off'),
                                   str(self.new_photos) if folder_ok else 'no folder yet',
                                   node.photo_dir)
        state, text, tip, stand_in = detector_status(detector_nodes, frames[-40:])
        self.pill_detector.set_state(state, text, tip)
        self.banner.setVisible(stand_in)
        self._update_mission_planner()

        # Detections table + best line.
        for f in new:
            for b in f.boxes:
                self.table.insertRow(0)
                cells = [time.strftime('%H:%M:%S', time.localtime(f.received)), b.cls.upper(),
                         f'{b.score * 100:.0f}%', f'{f.lat:.6f}', f'{f.lon:.6f}', f.image_name]
                for c, text in enumerate(cells):
                    item = self.QtWidgets.QTableWidgetItem(text)
                    if c == 1:
                        item.setForeground(self.QtGui.QColor(pw.CLASS_COLORS.get(b.cls, pw.THEME['text'])))
                        f2 = item.font()
                        f2.setBold(True)
                        item.setFont(f2)
                    item.setToolTip(f'detector method: {f.method}')
                    self.table.setItem(0, c, item)
        while self.table.rowCount() > 300:
            self.table.removeRow(self.table.rowCount() - 1)
        n_boxes = sum(len(f.boxes) for f in frames)
        self.det_count.setText(f'{n_boxes} DETECTIONS IN {len(frames)} CHECKED PHOTOS')
        best = best_per_class(frames)
        if best:
            parts = []
            for cls in ('tent', 'person'):
                if cls in best:
                    score, f, _ = best[cls]
                    col = pw.CLASS_COLORS[cls]
                    where = f'{f.lat:.6f}, {f.lon:.6f}' if f.has_gps else 'lat/lon 0,0 (no GPS in name)'
                    parts.append(f'<span style="color:{col}"><b>{cls.upper()}</b></span> '
                                 f'{score * 100:.0f}% at {where}')
            sep = ' &nbsp;&nbsp;|&nbsp;&nbsp; '
            self.best_label.setText('main_controller will fly to: &nbsp;' + sep.join(parts))
        self._update_photo(frame_by_name, last_hit, last_hit_t, detector_nodes, frames,
                           phase_idx, target_cls)

        # Events: drain what the ROS thread queued since the last tick.
        for _ in range(200):
            try:
                t, source, text, level = node.events.popleft()
            except IndexError:
                break
            self.log.add(t, source, text, level)
        self._update_reset_button()        # (may change the status text: a first click that timed out)
        self.cmd_status.setText(self._cmd_text)
        self.btn_start.setText('STARTING ...' if self.commander.busy else 'UPLOAD MISSION + START')

    def _update_mission_planner(self) -> None:
        """Mission Planner light + button states from the worker's latest check. Only reads a
        snapshot: no Windows call ever happens on the Qt thread."""
        st = self.mission_planner.snapshot()
        if st == self._mp_shown:
            return                         # nothing changed since the last tick: skip restyling
        self._mp_shown = st
        self.pill_mp.set_state('warn' if st.busy else ('ok' if st.running else 'off'),
                               f'{st.busy} ...' if st.busy else st.label, st.message)
        # A button is only clickable when it makes sense: OPEN while it is closed, CLOSE while
        # it is open, neither while an action runs or when this computer cannot do it at all.
        ready = st.checked and st.available and not st.busy
        self.btn_mp_open.setEnabled(ready and st.running is False)
        self.btn_mp_close.setEnabled(ready and st.running is True)
        if not ready:
            open_tip = close_tip = (f'Busy: {st.busy} Mission Planner ...' if st.busy else st.message)
        else:
            open_tip = ('Mission Planner is already open.' if st.running else
                        f'Start Mission Planner on Windows (it stays open when this panel closes). '
                        f'Then: {CONNECT_HINT}.')
            close_tip = ('Close Mission Planner like clicking its X (forced after 5 s if it does not '
                         'close). Safe during a flight.' if st.running else 'Mission Planner is not open.')
        self.btn_mp_open.setToolTip(open_tip)
        self.btn_mp_close.setToolTip(close_tip)

    def close(self) -> None:
        """Stop background work (called when the app quits). Mission Planner itself stays open."""
        for tm in self._timers:
            tm.stop()
        self.commander.stop()
        self.mission_planner.stop()
        self.loader.shutdown(wait=False)


def main(args=None) -> None:
    """Entry point: ros2 run sim_demo demo_panel."""
    note = prepare_qt_environment()
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    node = PanelNode()
    if note:
        node.get_logger().info(f'Qt: {note}')
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name='ros_spin', daemon=True)
    spin_thread.start()

    from PyQt5 import QtWidgets
    app = QtWidgets.QApplication(sys.argv[:1])
    app.setApplicationName(WINDOW_TITLE)
    window = DemoWindow(node, app)
    if node.cfg['fullscreen']:
        window.win.showFullScreen()
    else:
        window.win.show()
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    code = 0
    try:
        code = app.exec_()
    finally:
        window.close()
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
