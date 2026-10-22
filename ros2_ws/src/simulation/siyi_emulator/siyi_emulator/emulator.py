"""
siyi_emulator/emulator.py -- PURPOSE: the SIYI camera emulator itself, without ROS. It pretends to be
the gimbal camera at 192.168.144.25 so last year's video_cam driver runs unchanged against Gazebo:

  * UDP 37260: receives SIYI SDK frames, answers them (command_handler.py) and pushes 0x0B after a photo;
  * TCP 82:    the media web server video_cam lists and downloads photos from (media_server.py);
  * Gazebo:    moves the gimbal joints 50 times a second, stabilised against the drone attitude
               (gimbal_control.py), and asks the triggered photo camera for a frame on each 0x0C
               (gz_link.py), then writes the JPEG to the emulated SD card (photo.py, sd_card.py).

Threads: "udp" (requests), "control" (gimbal + zoom at control_rate_hz), "capture" (photos, one at a
time), the HTTP server threads, and gz-transport's own callback threads. One RLock protects the
camera + gimbal state.

Photo timeline (video_cam waits 0.8 s for 0x0B, then 0.5 s, then polls the listing every 0.5 s for 15 s):
  t=0      0x0C func 0 arrives -> photo job queued, 0x0B timer armed (feedback_deadline_s, 0.5 s)
  t~0      worker publishes /sim/camera/photo/trigger (re-sent after retrigger_after_s if no frame)
  t~0.1    Gazebo frame arrives -> 0x0B info_type 0 pushed now (or by the timer, whichever is first)
  t~0.2    digital zoom + JPEG + SD card write (atomic) -> listed by getmedialist
  no frame within photo_timeout_s (10 s) -> labelled test pattern (test_pattern_fallback) or photo lost.
  (video_cam keeps polling for 15 s, so a slow render on a busy PC is still found.)

Run without ROS (handy for tests and debugging):
    python3 -m siyi_emulator.emulator --bind-ip 127.0.0.3 --no-gazebo
"""

from __future__ import annotations

import argparse
import dataclasses
import datetime as _dt
import itertools
import logging
import math
import queue
import signal
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

from . import photo as ph
from . import sdk_protocol as sp
from .camera_state import CameraState, get_profile
from .command_handler import CommandHandler, HandlerHooks
from .gimbal_control import (MODE_NAMES, GimbalLimits, GimbalModel, Joints, body_to_camera,
                             camera_heading_pitch_roll, camera_world_pose, rot_from_quat)
from .gz_link import (CameraInfoSample, GzLink, GzTopics, JointSample, NullGzLink, OdomSample,
                      RawImage)
from .media_server import MediaServer
from .netutil import explain_bind_error, resolve_bind_ip
from .sd_card import PhotoFile, SdCard

MODE_BY_NAME = {"lock": 0, "follow": 1, "fpv": 2}


class EmulatorStartError(RuntimeError):
    """The emulator could not start (port in use, no permission ...). The message says how to fix it."""


# ---------------------------------------------------------------------------------------------
# Configuration (every field is also a ROS parameter of the same name, see node.py and README)
# ---------------------------------------------------------------------------------------------
@dataclass
class EmulatorConfig:
    """All settings of the emulator, with their defaults and units."""

    # which camera to pretend to be
    camera_model: str = "zr10"            # 'zr10' (2560x1440, DEFAULT) or 'a8mini' (3840x2160)
    strict: bool = False                  # True = silent on 0x04/0x06/0x16/0x18/0x19 like a real A8 mini
    max_zoom: float = 0.0                 # > 0 overrides the model's max zoom (x)
    photo_width: int = 0                  # > 0 overrides the model's photo width (px)
    photo_height: int = 0                 # > 0 overrides the model's photo height (px)
    # network
    camera_ip: str = "192.168.144.25"     # where clients expect the camera
    bind_ip: str = ""                     # '' = camera_ip if it exists here, else 127.0.0.1
    ctrl_port: int = 37260                # UDP SDK port (0 = any free port, tests)
    media_port: int = 82                  # TCP media port (0 = any free port, tests)
    media_url_host: str = ""              # host[:port] in photo URLs; '' = the request's Host header
    # SD card / photos
    sd_card_dir: str = "~/.ros/siyi_emulator/sd_card"
    photo_dir_name: str = "101SIYI_IMG"
    clear_sd_on_start: bool = True
    restart_numbering_after_format: bool = False
    sd_card_present: bool = True          # False = simulate "no TF card" (record_sta 2, photos fail)
    jpeg_quality: int = 92
    write_sidecar: bool = True            # IMG_NNNN.json with camera pose / zoom next to each photo
    # Frame wait: a loaded PC with software rendering took > 3 s in the acceptance run; video_cam polls the
    # listing for 15 s after the 0x0B (pushed at feedback_deadline_s anyway), so 10 s is still on time.
    photo_timeout_s: float = 10.0         # wait this long for the Gazebo frame
    retrigger_after_s: float = 3.0        # re-send the trigger if no frame yet (max 2 times)
    feedback_deadline_s: float = 0.5      # push 0x0B no later than this after 0x0C (video_cam waits 0.8 s)
    test_pattern_fallback: bool = True    # no Gazebo frame -> save a labelled test pattern instead
    # gimbal
    initial_mode: str = "follow"          # lock | follow | fpv
    initial_yaw_deg: float = 0.0          # + = left
    initial_pitch_deg: float = -90.0      # -90 = straight down (nobody else points it in the sim)
    max_slew_dps: float = 90.0
    max_rate_dps: float = 90.0            # 0x07 speed 100 = this many deg/s
    rate_yaw_sign: float = -1.0           # SIYI speed + = right -> angle frame (+ = left)
    stabilize: bool = True                # keep the camera level when the drone tilts
    control_rate_hz: float = 50.0
    zoom_speed_x_per_s: float = 4.0       # zoom travel speed (x per second)
    # Gazebo
    use_gazebo: bool = True
    gz_partition: str = ""                # '' = keep $GZ_PARTITION
    gz_yaw_cmd_topic: str = "/sim/gimbal/yaw/cmd_pos"
    gz_roll_cmd_topic: str = "/sim/gimbal/roll/cmd_pos"
    gz_pitch_cmd_topic: str = "/sim/gimbal/pitch/cmd_pos"
    gz_joint_states_topic: str = "/sim/gimbal/joint_states"
    gz_odometry_topic: str = "/sim/drone/odometry"
    gz_photo_image_topic: str = "/sim/camera/photo/image"
    gz_photo_camera_info_topic: str = "/sim/camera/photo/camera_info"
    gz_photo_trigger_topic: str = "/sim/camera/photo/trigger"
    yaw_joint: str = "gimbal_yaw"
    roll_joint: str = "gimbal_roll"
    pitch_joint: str = "gimbal_pitch"
    optical_zoom_topics: str = ""         # comma list of CameraZoomPlugin topics; '' = digital zoom only
    hfov_rad: float = ph.DEFAULT_HFOV_RAD  # lens HFOV of the Gazebo photo camera (metadata, zoom plugin)
    optical_zoom_ref_hfov_rad: float = ph.ZOOM_PLUGIN_REF_HFOV_RAD
    # logging
    log_commands: bool = False            # log every SDK command at INFO

    @classmethod
    def field_names(cls) -> List[str]:
        """Names of all settings (used to declare the ROS parameters)."""
        return [f.name for f in dataclasses.fields(cls)]

    def updated(self, values: Dict[str, Any]) -> Tuple["EmulatorConfig", List[str]]:
        """Return a copy with ``values`` applied (types coerced to the defaults' types).

        Returns ``(config, unknown_keys)``.
        """
        known = {f.name: f for f in dataclasses.fields(self)}
        changes, unknown = {}, []
        for key, value in values.items():
            if key not in known:
                unknown.append(key)
                continue
            changes[key] = _coerce(value, type(getattr(self, key)))
        return dataclasses.replace(self, **changes), unknown

    def gz_topics(self) -> GzTopics:
        """The :class:`GzTopics` described by this config."""
        zoom = tuple(t.strip() for t in str(self.optical_zoom_topics).split(",") if t.strip())
        return GzTopics(self.gz_yaw_cmd_topic, self.gz_roll_cmd_topic, self.gz_pitch_cmd_topic,
                        self.gz_joint_states_topic, self.gz_odometry_topic, self.gz_photo_image_topic,
                        self.gz_photo_camera_info_topic, self.gz_photo_trigger_topic, zoom)


def _coerce(value: Any, kind: type) -> Any:
    """Convert a string/number from the command line or ROS to the field's type."""
    if kind is bool:
        if isinstance(value, str):
            low = value.strip().lower()
            if low in ("1", "true", "yes", "on"):
                return True
            if low in ("0", "false", "no", "off"):
                return False
            raise ValueError(f"not a boolean: {value!r}")
        return bool(value)
    if kind is int:
        return int(float(value))
    if kind is float:
        return float(value)
    return str(value)


# ---------------------------------------------------------------------------------------------
# Small records
# ---------------------------------------------------------------------------------------------
@dataclass
class _CaptureJob:
    job_id: int
    addr: Optional[Tuple[str, int]]
    submitted: float
    timer: Optional[threading.Timer] = None
    feedback_sent: bool = False
    lock: threading.Lock = field(default_factory=threading.Lock)


@dataclass
class PhotoEvent:
    """Given to photo listeners after each saved photo (the ROS node publishes it as JSON)."""

    photo: PhotoFile
    metadata: Dict[str, Any]


class _EmulatorHooks(HandlerHooks):
    """Connects the command handler to the running emulator."""

    def __init__(self, emulator: "SiyiEmulator") -> None:
        super().__init__(emulator.gimbal)
        self.emu = emulator

    def attitude(self):
        return self.emu.current_attitude()

    def take_photo(self, addr):
        self.emu.request_photo(addr)

    def push_feedback(self, addr, info_type):
        self.emu.push_feedback(addr, info_type)

    def format_sd(self) -> int:
        return self.emu.sd.format()


# ---------------------------------------------------------------------------------------------
# The emulator
# ---------------------------------------------------------------------------------------------
class SiyiEmulator:
    """The running camera emulator. Create, :meth:`start`, ..., :meth:`stop`.

    Args:
        config:   :class:`EmulatorConfig`.
        gz_link:  a :class:`GzLink`-like object; None = build one from the config (NullGzLink when
                  use_gazebo is False). Tests pass a fake to inject frames.
        logger:   ``logging.Logger`` (the ROS node passes one that forwards to the ROS logger).
        clock:    monotonic clock in seconds.
    """

    MAX_QUEUED_PHOTOS = 20
    MAX_RETRIGGERS = 2
    HISTORY = 150            # pose samples kept (~3 s at 50 Hz) to match photos to poses

    def __init__(self, config: Optional[EmulatorConfig] = None, gz_link=None,
                 logger: Optional[logging.Logger] = None, clock: Callable[[], float] = time.monotonic) -> None:
        self.cfg = config or EmulatorConfig()
        self.log = logger or logging.getLogger("siyi_emulator")
        self.clock = clock
        cfg = self.cfg
        self.profile = get_profile(cfg.camera_model, cfg.max_zoom, cfg.photo_width, cfg.photo_height)
        self.camera = CameraState(self.profile, strict=cfg.strict, sd_card_present=cfg.sd_card_present,
                                  zoom_speed_x_per_s=cfg.zoom_speed_x_per_s)
        limits = GimbalLimits(yaw_min_deg=self.profile.yaw_min_deg, yaw_max_deg=self.profile.yaw_max_deg,
                              pitch_min_deg=self.profile.pitch_min_deg, pitch_max_deg=self.profile.pitch_max_deg)
        mode = MODE_BY_NAME.get(str(cfg.initial_mode).strip().lower())
        if mode is None:
            self.log.warning("initial_mode '%s' unknown (lock|follow|fpv) - using follow", cfg.initial_mode)
            mode = 1
        self.gimbal = GimbalModel(limits, cfg.max_slew_dps, cfg.rate_yaw_sign, cfg.max_rate_dps, mode,
                                  cfg.stabilize, cfg.initial_yaw_deg, cfg.initial_pitch_deg)
        self.lock = threading.RLock()
        self.sd = SdCard(cfg.sd_card_dir, cfg.photo_dir_name, cfg.clear_sd_on_start,
                         cfg.restart_numbering_after_format)
        # Child loggers ("siyi_emulator.sdk" ...) propagate to the parent's handlers, so the ROS node
        # only has to attach its forwarding handler to self.log.
        self.handler = CommandHandler(self.camera, self.gimbal, _EmulatorHooks(self), self.lock, clock,
                                      self.log.getChild("sdk"), cfg.log_commands)
        if gz_link is not None:
            self.gz = gz_link
        elif cfg.use_gazebo:
            joints = {"yaw": cfg.yaw_joint, "roll": cfg.roll_joint, "pitch": cfg.pitch_joint}
            self.gz = GzLink(cfg.gz_topics(), joints, cfg.gz_partition, self.log.getChild("gz"))
        else:
            self.gz = NullGzLink("use_gazebo is false")
        self.optical_zoom_enabled = bool(cfg.gz_topics().optical_zoom)
        self.photo_listeners: List[Callable[[PhotoEvent], None]] = []

        self.bind_ip = ""
        self.media: Optional[MediaServer] = None
        self._sock: Optional[socket.socket] = None
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []
        self._jobs: "queue.Queue[_CaptureJob]" = queue.Queue(self.MAX_QUEUED_PHOTOS)
        self._job_ids = itertools.count(1)
        self._push_seq = itertools.count(1)
        self._clients: Dict[Tuple[str, int], float] = {}
        self._warned: set = set()
        self._started_at = 0.0
        # latest Gazebo data
        self._odom: Optional[OdomSample] = None
        self._joints: Optional[JointSample] = None
        self._odom_hist: Deque[OdomSample] = deque(maxlen=self.HISTORY)
        self._joint_hist: Deque[JointSample] = deque(maxlen=self.HISTORY)
        self._camera_info: Optional[CameraInfoSample] = None
        self._latest_sim_time: Optional[float] = None
        self._att_prev: Optional[Tuple[float, float, float, float]] = None   # (t, yaw, pitch, roll)
        self._att_rates = (0.0, 0.0, 0.0)
        # photo frame hand-over between the gz thread and the capture worker
        self._frame_cond = threading.Condition()
        self._waiting_frame = False
        self._frame_min_stamp: Optional[float] = None
        self._captured: Optional[RawImage] = None
        self._last_timeout_at: Optional[float] = None
        self.frames_received = 0       # every photo frame Gazebo sent
        self.frames_dropped = 0        # frames nobody was waiting for, or too old
        self.frames_stale = 0          # frames older than the trigger (from an earlier, re-sent trigger)
        self.triggers_timed_out = 0
        self.photos_saved = 0
        self.last_photo: Optional[PhotoEvent] = None

    # ------------------------------------------------------------------ life cycle
    @property
    def udp_port(self) -> int:
        """The UDP port in use (after start)."""
        return self._sock.getsockname()[1] if self._sock else self.cfg.ctrl_port

    @property
    def media_port(self) -> int:
        """The TCP media port in use (after start)."""
        return self.media.port if self.media else self.cfg.media_port

    def start(self) -> None:
        """Bind both servers, connect to Gazebo and start the threads.

        Raises:
            EmulatorStartError: a port could not be opened (the message explains the fix).
        """
        cfg = self.cfg
        self.bind_ip, warning = resolve_bind_ip(cfg.camera_ip, cfg.bind_ip)
        if warning:
            self.log.warning(warning)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind((self.bind_ip, int(cfg.ctrl_port)))
        except OSError as exc:
            sock.close()
            raise EmulatorStartError(explain_bind_error(exc, self.bind_ip, int(cfg.ctrl_port), "UDP")) from exc
        sock.settimeout(0.5)
        self._sock = sock
        self.media = MediaServer(self.sd, self.bind_ip, int(cfg.media_port), cfg.media_url_host,
                                 self.log.getChild("media"))
        try:
            self.media.start()
        except OSError as exc:
            sock.close()
            self._sock = None
            raise EmulatorStartError(explain_bind_error(exc, self.bind_ip, int(cfg.media_port), "TCP")) from exc

        self.gz.on_odometry = self._on_odometry
        self.gz.on_joint_states = self._on_joint_states
        self.gz.on_photo = self._on_photo
        self.gz.on_camera_info = self._on_camera_info
        if cfg.use_gazebo and not self.gz.start() and not self.gz.available:
            self.log.warning("Gazebo link not available: %s. The SDK and media server still work; photos will be "
                             "%s.", self.gz.reason, "test patterns" if cfg.test_pattern_fallback else "lost")

        self._stop.clear()
        self._started_at = self.clock()
        for name, target in (("siyi-udp", self._udp_loop), ("siyi-control", self._control_loop),
                             ("siyi-capture", self._capture_loop)):
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self._threads.append(thread)
        self.log.info(
            "SIYI camera emulator ready: %s (hardware id %s%s) at %s - SDK UDP %d, media http://%s:%d/ - "
            "photos %dx%d, max zoom %.1fx%s - SD card %s - gimbal %s, yaw %.0f pitch %.0f - Gazebo: %s",
            self.profile.display_name, self.profile.hardware_id, ", STRICT" if cfg.strict else "", self.bind_ip,
            self.udp_port, self.bind_ip, self.media_port, self.profile.photo_width, self.profile.photo_height,
            self.profile.max_zoom, " (optical via CameraZoomPlugin)" if self.optical_zoom_enabled else "",
            self.sd.photo_dir, MODE_NAMES[self.gimbal.mode], cfg.initial_yaw_deg, cfg.initial_pitch_deg,
            self.gz.reason if cfg.use_gazebo else "disabled")

    def stop(self) -> None:
        """Stop all threads and close the sockets (safe to call twice)."""
        self._stop.set()
        with self._frame_cond:
            self._frame_cond.notify_all()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._threads = []
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        if self.media is not None:
            self.media.stop()
        try:
            self.gz.stop()
        except Exception:
            pass

    def run_forever(self) -> None:
        """Block until :meth:`stop` is called (or Ctrl-C in the caller)."""
        while not self._stop.wait(0.5):
            pass

    # ------------------------------------------------------------------ helpers
    def _warn_once(self, key: str, text: str, *args: Any) -> None:
        if key not in self._warned:
            self._warned.add(key)
            self.log.warning(text, *args)

    def _send(self, data: bytes, addr: Optional[Tuple[str, int]]) -> None:
        sock = self._sock
        if sock is None or addr is None:
            return
        try:
            sock.sendto(data, addr)
        except OSError as exc:
            self._warn_once("send_" + str(addr), "cannot send to %s: %s", addr, exc)

    # ------------------------------------------------------------------ UDP server
    def _udp_loop(self) -> None:
        while not self._stop.is_set():
            sock = self._sock
            if sock is None:
                break
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    break
                time.sleep(0.05)
                continue
            if addr not in self._clients:
                if len(self._clients) > 64:
                    self._clients.clear()
                self.log.info("SDK client %s:%d connected (first command 0x%02X)", addr[0], addr[1],
                              data[7] if len(data) > 7 else 0)
            self._clients[addr] = self.clock()
            frames, problems = sp.split_frames(data)
            for problem in problems:
                self._warn_once("frame:" + problem.split("(")[0], "ignored bad SDK bytes from %s:%d: %s",
                                addr[0], addr[1], problem)
            for frame in frames:
                reply = self.handler.handle(frame, addr)
                if reply is not None:
                    self._send(reply.to_bytes(), addr)

    def push_feedback(self, addr: Optional[Tuple[str, int]], info_type: int) -> None:
        """Send an unsolicited 0x0B Function Feedback frame (info_type 0 = photo OK) to ``addr``."""
        frame = sp.Frame(sp.CMD_FUNCTION_FEEDBACK, bytes([int(info_type) & 0xFF]),
                         seq=next(self._push_seq) & 0xFFFF, ctrl=sp.CTRL_ACK_PACK)
        self._send(frame.to_bytes(), addr)

    # ------------------------------------------------------------------ Gazebo callbacks
    def _note_sim_time(self, stamp: Optional[float]) -> None:
        if stamp is not None and (self._latest_sim_time is None or stamp > self._latest_sim_time):
            self._latest_sim_time = stamp

    def _on_odometry(self, sample: OdomSample) -> None:
        with self.lock:
            self._odom = sample
            self._odom_hist.append(sample)
            self._note_sim_time(sample.stamp)

    def _on_joint_states(self, sample: JointSample) -> None:
        with self.lock:
            self._joints = sample
            self._joint_hist.append(sample)
            self._note_sim_time(sample.stamp)

    def _on_camera_info(self, sample: CameraInfoSample) -> None:
        self._camera_info = sample

    def _on_photo(self, raw: RawImage) -> None:
        with self._frame_cond:
            self.frames_received += 1
            self.log.debug("photo frame %dx%d stamp=%s (oldest accepted %s, waiting=%s)", raw.width, raw.height,
                           raw.stamp, self._frame_min_stamp, self._waiting_frame)
            if not self._waiting_frame:
                self.frames_dropped += 1
                if self._last_timeout_at is not None and raw.received - self._last_timeout_at < 60.0:
                    self._warn_once("late_frame", "a Gazebo photo frame arrived %.1f s after its photo had timed out "
                                    "and was discarded: rendering is slower than photo_timeout_s (%.1f s). The "
                                    "computer may be overloaded; raise photo_timeout_s.",
                                    raw.received - self._last_timeout_at + self.cfg.photo_timeout_s,
                                    self.cfg.photo_timeout_s)
                return
            if (self._frame_min_stamp is not None and raw.stamp is not None
                    and raw.stamp < self._frame_min_stamp - 0.002):
                self.frames_dropped += 1        # an old frame (e.g. from a re-sent trigger): not ours
                self.frames_stale += 1
                return
            self._captured = raw
            self._frame_cond.notify_all()

    # ------------------------------------------------------------------ attitude
    def _measured_joints(self, max_age_s: float = 1.0) -> Optional[Joints]:
        sample = self._joints
        if sample is None or self.clock() - sample.received > max_age_s:
            return None
        if not all(role in sample.positions for role in ("yaw", "roll", "pitch")):
            return None
        return Joints(sample.positions["yaw"], sample.positions["roll"], sample.positions["pitch"])

    def current_attitude(self) -> Tuple[float, float, float, float, float, float]:
        """(yaw, pitch, roll) deg as the real camera reports them, plus their rates (deg/s).

        Uses the measured joints from Gazebo when fresh, else the commanded joints.
        """
        with self.lock:
            joints = self._measured_joints() or self.gimbal.joint_cmd
            yaw, pitch, roll = self.gimbal.reported_attitude(joints)
            rates = self._att_rates
        return (yaw, pitch, roll) + tuple(rates)

    def _update_rates(self, now: float) -> None:
        yaw, pitch, roll = self.current_attitude()[:3]
        if self._att_prev is not None:
            t0, y0, p0, r0 = self._att_prev
            dt = now - t0
            if dt > 1e-3:
                alpha = 0.3      # light smoothing of the finite difference
                new = ((yaw - y0) / dt, (pitch - p0) / dt, (roll - r0) / dt)
                self._att_rates = tuple(alpha * n + (1 - alpha) * o for n, o in zip(new, self._att_rates))
        self._att_prev = (now, yaw, pitch, roll)

    # ------------------------------------------------------------------ control loop (50 Hz)
    def _control_loop(self) -> None:
        period = 1.0 / max(1.0, float(self.cfg.control_rate_hz))
        last = self.clock()
        last_zoom_cmd: Optional[float] = None
        last_zoom_pub = 0.0
        while not self._stop.is_set():
            now = self.clock()
            dt = now - last
            last = now
            try:
                with self.lock:
                    if self._odom is not None:
                        self.gimbal.set_drone_attitude(*self._odom.orientation)
                    joints = self.gimbal.step(dt)
                    zoom_now = self.camera.zoom.current(now)
                self.gz.publish_joint_targets(*joints)
                self._update_rates(now)
                if self.optical_zoom_enabled:
                    optical, _ = ph.split_zoom(zoom_now, self.profile.optical_zoom_max)
                    cmd = ph.zoom_plugin_command(optical, self.cfg.hfov_rad, self.cfg.optical_zoom_ref_hfov_rad)
                    # re-send every second too: a message published before Gazebo connected is lost
                    if last_zoom_cmd is None or abs(cmd - last_zoom_cmd) > 1e-3 or now - last_zoom_pub > 1.0:
                        self.gz.publish_optical_zoom(cmd)
                        last_zoom_cmd, last_zoom_pub = cmd, now
                self._health_checks(now)
            except Exception as exc:     # keep the gimbal loop alive whatever happens
                self._warn_once("control:" + type(exc).__name__, "control loop error: %s", exc)
            self._stop.wait(max(0.0, period - (self.clock() - now)))

    def _health_checks(self, now: float) -> None:
        """One-time hints when Gazebo data never arrives (wrong partition, model not spawned ...)."""
        if not self.gz.available or now - self._started_at < 15.0:
            return
        if self._odom is None:
            self._warn_once("no_odom", "no drone odometry on gz topic %s after 15 s: the camera is stabilised "
                            "as if the drone were level. Is the drone spawned, does its OdometryPublisher use "
                            "this topic (with <dimensions>3</dimensions>), and is GZ_PARTITION the same?",
                            self.cfg.gz_odometry_topic)
        if self._joints is None:
            self._warn_once("no_joints", "no gimbal joint states on gz topic %s after 15 s: 0x0D reports the "
                            "commanded angles instead of the measured ones.", self.cfg.gz_joint_states_topic)

    # ------------------------------------------------------------------ photos
    def request_photo(self, addr: Optional[Tuple[str, int]]) -> None:
        """0x0C func 0: queue a photo for ``addr`` and arm the 0x0B deadline timer (non-blocking)."""
        if not self.camera.sd_card_present:
            self.log.warning("photo refused: the emulated camera has NO TF CARD (sd_card_present:=false)")
            self.push_feedback(addr, sp.FEEDBACK_PHOTO_FAILED)
            return
        job = _CaptureJob(next(self._job_ids), addr, self.clock())
        job.timer = threading.Timer(max(0.0, self.cfg.feedback_deadline_s), self._feedback_once,
                                    args=(job, sp.FEEDBACK_PHOTO_OK))
        job.timer.daemon = True
        try:
            self._jobs.put_nowait(job)
        except queue.Full:
            self.log.warning("photo refused: %d photos already waiting", self.MAX_QUEUED_PHOTOS)
            self.push_feedback(addr, sp.FEEDBACK_PHOTO_FAILED)
            return
        job.timer.start()

    def _feedback_once(self, job: _CaptureJob, info_type: int) -> None:
        with job.lock:
            if job.feedback_sent:
                return
            job.feedback_sent = True
        if job.timer is not None:
            job.timer.cancel()
        self.push_feedback(job.addr, info_type)

    def _capture_loop(self) -> None:
        while not self._stop.is_set():
            try:
                job = self._jobs.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self._process_job(job)
            except Exception as exc:
                self.log.error("photo %d failed: %s", job.job_id, exc)
                self._feedback_once(job, sp.FEEDBACK_PHOTO_FAILED)

    def _acquire_frame(self) -> Tuple[Optional[RawImage], str]:
        """Trigger the Gazebo photo camera and wait for its frame. Returns (frame or None, reason)."""
        if not self.gz.available:
            return None, f"no Gazebo link ({self.gz.reason})"
        with self._frame_cond:
            self._captured = None
            self._waiting_frame = True
            self._frame_min_stamp = self._latest_sim_time
        try:
            self.gz.trigger_photo()
            retriggers = 0
            start = self.clock()
            deadline = start + max(0.1, self.cfg.photo_timeout_s)
            next_retrigger = start + max(0.05, self.cfg.retrigger_after_s)
            while True:
                with self._frame_cond:
                    if self._captured is not None:
                        return self._captured, "gazebo"
                    now = self.clock()
                    if now >= deadline or self._stop.is_set():
                        break
                    self._frame_cond.wait(timeout=max(0.001, min(deadline, next_retrigger) - now))
                    if self._captured is not None:
                        return self._captured, "gazebo"
                if self.clock() >= next_retrigger:
                    if retriggers < self.MAX_RETRIGGERS:
                        retriggers += 1
                        self.gz.trigger_photo()
                        self.log.debug("no photo frame yet, trigger re-sent (%d)", retriggers)
                        next_retrigger = self.clock() + max(0.05, self.cfg.retrigger_after_s)
                    else:
                        next_retrigger = deadline
            subs = self.gz.trigger_has_subscribers()
            hint = "" if subs is not False else " and nothing listens to the trigger topic (is the drone spawned?)"
            self._last_timeout_at = self.clock()
            self.triggers_timed_out += 1
            self.log.warning("no photo frame within %.1f s (%d trigger(s) sent, %d frame(s) received so far, "
                             "%d stale)%s", self.cfg.photo_timeout_s, retriggers + 1, self.frames_received,
                             self.frames_stale, hint)
            return None, (f"no frame on {self.cfg.gz_photo_image_topic} within {self.cfg.photo_timeout_s:.1f} s "
                          f"after triggering {self.cfg.gz_photo_trigger_topic}{hint}")
        finally:
            with self._frame_cond:
                self._waiting_frame = False

    def _process_job(self, job: _CaptureJob) -> None:
        cfg = self.cfg
        out_size = (self.profile.photo_width, self.profile.photo_height)
        raw, reason = self._acquire_frame()
        img = None
        source = "gazebo"
        if raw is not None:
            self._feedback_once(job, sp.FEEDBACK_PHOTO_OK)          # the shutter has fired
            try:
                img = ph.gz_image_to_bgr(raw.width, raw.height, raw.step, raw.data, raw.pixel_format)
            except ValueError as exc:
                reason = f"unusable Gazebo frame: {exc}"
                img = None
            if img is not None and (img.shape[1], img.shape[0]) != out_size:
                self._warn_once("frame_size", "Gazebo photo frame is %dx%d but the %s photo is %dx%d: resizing. "
                                "Generate the drone model with matching photo_width/photo_height.",
                                img.shape[1], img.shape[0], self.profile.display_name, out_size[0], out_size[1])
        if img is None:
            if not cfg.test_pattern_fallback:
                self.log.error("photo lost: %s (test_pattern_fallback is false)", reason)
                self._feedback_once(job, sp.FEEDBACK_PHOTO_FAILED)
                return
            self._warn_once("pattern:" + reason[:40], "saving a TEST PATTERN instead of a real photo: %s", reason)
            img = ph.make_test_pattern(out_size[0], out_size[1],
                                       [self.sd.peek_next_name(), reason[:60]], seed=job.job_id)
            source = "test_pattern"
            self._feedback_once(job, sp.FEEDBACK_PHOTO_OK)
        shot_time = raw.received if raw is not None else self.clock()
        with self.lock:
            zoom_total = self.camera.zoom.current(shot_time)
        optical_max = self.profile.optical_zoom_max if self.optical_zoom_enabled else 1.0
        optical, digital = ph.split_zoom(zoom_total, optical_max)
        img = ph.apply_digital_zoom(img, digital, out_size)
        jpeg = ph.encode_jpeg(img, cfg.jpeg_quality)
        meta = self._photo_metadata(raw, source, zoom_total, optical, digital, out_size, job) if cfg.write_sidecar else None
        photo = self.sd.write_photo(jpeg, meta)
        self.photos_saved += 1
        elapsed_ms = (self.clock() - job.submitted) * 1000.0
        self.log.info("photo %s saved: %dx%d, %.2f MB, zoom %.1fx%s, %s, %.0f ms after 0x0C",
                      photo.name, out_size[0], out_size[1], len(jpeg) / 1e6, zoom_total,
                      f" (optical {optical:.1f}x)" if optical > 1.0 else "", source, elapsed_ms)
        event = PhotoEvent(photo, meta or {"file": photo.rel_path, "index": photo.index})
        self.last_photo = event
        for listener in list(self.photo_listeners):
            try:
                listener(event)
            except Exception as exc:
                self._warn_once("listener", "photo listener failed: %s", exc)

    # ------------------------------------------------------------------ metadata
    @staticmethod
    def _closest(history: Deque, stamp: Optional[float], fallback):
        if stamp is None or not history:
            return fallback
        best = min(history, key=lambda s: abs((s.stamp if s.stamp is not None else -1e9) - stamp))
        return best if best.stamp is not None else fallback

    def _photo_metadata(self, raw: Optional[RawImage], source: str, zoom_total: float, optical: float,
                        digital: float, out_size: Tuple[int, int], job: _CaptureJob) -> Dict[str, Any]:
        """Everything needed to geolocate the photo (drone pose, gimbal, camera pose, zoom, FOV)."""
        stamp = raw.stamp if raw is not None else None
        with self.lock:
            odom = self._closest(self._odom_hist, stamp, self._odom)
            jsample = self._closest(self._joint_hist, stamp, self._joints)
            joints_cmd = self.gimbal.joint_cmd
            mode = self.gimbal.mode
        if jsample is not None and all(r in jsample.positions for r in ("yaw", "roll", "pitch")):
            joints = Joints(jsample.positions["yaw"], jsample.positions["roll"], jsample.positions["pitch"])
            joints_source = "measured"
        else:
            joints, joints_source = joints_cmd, "commanded"
        R_wb = rot_from_quat(*odom.orientation) if odom is not None else rot_from_quat(0, 0, 0, 1)
        position = odom.position if odom is not None else (0.0, 0.0, 0.0)
        pose = camera_world_pose(position, R_wb, joints, self.gimbal.mount_xyz)
        _, cam_pitch, cam_roll = camera_heading_pitch_roll(R_wb @ body_to_camera(joints))
        lens_hfov = self.cfg.hfov_rad
        info = self._camera_info
        if info is not None and not self.optical_zoom_enabled:
            lens_hfov = ph.hfov_from_camera_info(info.width, info.fx) or lens_hfov
        return {
            "emulator": "siyi_emulator",
            "camera_model": self.profile.display_name,
            "wall_time": _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="milliseconds"),
            "sim_time": stamp,
            "trigger_to_frame_ms": None if raw is None else round((raw.received - job.submitted) * 1000.0, 1),
            "image": {"width": out_size[0], "height": out_size[1], "source": source,
                      "jpeg_quality": self.cfg.jpeg_quality},
            "zoom": {"total": round(zoom_total, 3), "optical": round(optical, 3), "digital": round(digital, 3)},
            "hfov_rad": {"lens": lens_hfov, "effective": ph.zoomed_hfov(lens_hfov, zoom_total)},
            "gimbal": {
                "mode": MODE_NAMES[mode],
                "yaw_deg": round(math.degrees(joints.yaw), 3),
                "pitch_deg": round(math.degrees(cam_pitch), 3),
                "roll_deg": round(math.degrees(cam_roll), 3),
                "joints_rad": {"yaw": joints.yaw, "roll": joints.roll, "pitch": joints.pitch},
                "joints_source": joints_source,
            },
            "drone": {
                "position_enu_m": list(position),
                "orientation_xyzw": list(odom.orientation) if odom is not None else [0.0, 0.0, 0.0, 1.0],
                "source": "odometry" if odom is not None else "none (level drone assumed)",
            },
            "camera_world": {
                "position_enu_m": pose.position.tolist(),
                "look_direction_enu": pose.look_direction.tolist(),
                "optical_quat_xyzw": list(pose.quat_optical_xyzw),
                "heading_compass_deg": round(pose.compass_heading_deg, 3),
                "pitch_deg": round(math.degrees(pose.pitch_rad), 3),
            },
        }

    # ------------------------------------------------------------------ status (for ROS / debugging)
    def status(self) -> Dict[str, Any]:
        """A JSON-friendly snapshot: model, network, Gazebo freshness, gimbal, zoom, photos."""
        now = self.clock()
        yaw, pitch, roll = self.current_attitude()[:3]
        with self.lock:
            zoom = self.camera.zoom.current(now)
            target = self.camera.zoom.target
            mode = MODE_NAMES[self.gimbal.mode]
        odom_age = None if self._odom is None else round(now - self._odom.received, 3)
        joint_age = None if self._joints is None else round(now - self._joints.received, 3)
        return {
            "camera_model": self.profile.display_name, "strict": self.cfg.strict,
            "bind_ip": self.bind_ip, "ctrl_port": self.udp_port, "media_port": self.media_port,
            "gazebo": self.gz.reason if self.cfg.use_gazebo else "disabled",
            "odometry_age_s": odom_age, "joint_states_age_s": joint_age,
            "gimbal": {"mode": mode, "yaw_deg": round(yaw, 2), "pitch_deg": round(pitch, 2), "roll_deg": round(roll, 2)},
            "zoom": {"current": round(zoom, 2), "target": round(target, 2), "max": self.camera.zoom.max_zoom},
            "photos_saved": self.photos_saved, "photos_on_card": self.sd.count(),
            "photo_frames": {"received": self.frames_received, "dropped": self.frames_dropped,
                             "stale": self.frames_stale, "timeouts": self.triggers_timed_out},
            "last_photo": self.last_photo.photo.name if self.last_photo else None,
            "sd_card_dir": str(self.sd.photo_dir), "clients": len(self._clients),
            "commands_received": self.handler.commands_received(),
        }


# ---------------------------------------------------------------------------------------------
# Stand-alone command line (no ROS)
# ---------------------------------------------------------------------------------------------
def build_arg_parser() -> argparse.ArgumentParser:
    """Command-line options of the stand-alone emulator (the common ones; --param sets any other)."""
    p = argparse.ArgumentParser(
        prog="siyi_emulator",
        description="SIYI gimbal-camera emulator (SDK UDP + media HTTP + Gazebo), without ROS.")
    p.add_argument("--camera-model", help="zr10 (default) or a8mini")
    p.add_argument("--strict", action="store_true", help="stay silent where a real A8 mini is silent")
    p.add_argument("--camera-ip", help="address clients expect (default 192.168.144.25)")
    p.add_argument("--bind-ip", help="address to listen on (default: camera-ip if local, else 127.0.0.1)")
    p.add_argument("--ctrl-port", type=int, help="UDP SDK port (default 37260)")
    p.add_argument("--media-port", type=int, help="HTTP media port (default 82)")
    p.add_argument("--sd-card-dir", help="folder of the emulated SD card")
    p.add_argument("--no-gazebo", action="store_true", help="do not connect to Gazebo (test patterns)")
    p.add_argument("--params-file", help="ROS-style YAML (siyi_emulator: ros__parameters: {...})")
    p.add_argument("--param", action="append", default=[], metavar="NAME=VALUE",
                   help="set any EmulatorConfig field, e.g. --param initial_pitch_deg=0 (repeatable)")
    p.add_argument("--log-level", default="INFO", help="DEBUG, INFO, WARNING (default INFO)")
    return p


def config_from_args(args: argparse.Namespace) -> EmulatorConfig:
    """Build an EmulatorConfig from parsed command-line arguments (file < flags < --param)."""
    values: Dict[str, Any] = {}
    if args.params_file:
        import yaml  # PyYAML ships with ROS; only needed for this option
        with open(args.params_file, "r", encoding="utf-8") as handle:
            doc = yaml.safe_load(handle) or {}
        for node_values in doc.values():
            if isinstance(node_values, dict):
                values.update(node_values.get("ros__parameters", {}))
    flag_map = {"camera_model": args.camera_model, "camera_ip": args.camera_ip, "bind_ip": args.bind_ip,
                "ctrl_port": args.ctrl_port, "media_port": args.media_port, "sd_card_dir": args.sd_card_dir}
    values.update({k: v for k, v in flag_map.items() if v is not None})
    if args.strict:
        values["strict"] = True
    if args.no_gazebo:
        values["use_gazebo"] = False
    for item in args.param:
        if "=" not in item:
            raise SystemExit(f"--param needs NAME=VALUE, got {item!r}")
        key, value = item.split("=", 1)
        values[key.strip()] = value.strip()
    config, unknown = EmulatorConfig().updated(values)
    if unknown:
        raise SystemExit(f"unknown setting(s): {', '.join(unknown)}. Valid: {', '.join(EmulatorConfig.field_names())}")
    return config


def main(argv: Optional[List[str]] = None) -> int:
    """Run the emulator without ROS until Ctrl-C. Returns the process exit code."""
    args = build_arg_parser().parse_args(argv)
    logging.basicConfig(level=getattr(logging, str(args.log_level).upper(), logging.INFO),
                        format="[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s", datefmt="%H:%M:%S")
    log = logging.getLogger("siyi_emulator")
    emulator = SiyiEmulator(config_from_args(args), logger=log)
    try:
        emulator.start()
    except EmulatorStartError as exc:
        log.error("cannot start: %s", exc)
        return 2
    signal.signal(signal.SIGTERM, lambda *_: emulator.stop())
    try:
        emulator.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        emulator.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
