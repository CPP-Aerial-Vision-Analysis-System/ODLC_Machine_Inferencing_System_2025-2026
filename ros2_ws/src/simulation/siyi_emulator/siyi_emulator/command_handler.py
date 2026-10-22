"""
siyi_emulator/command_handler.py -- PURPOSE: decide what the emulated camera does and answers for
every SIYI SDK command. It takes one parsed frame (from sdk_protocol.py), changes the camera state
(camera_state.py) or the gimbal (gimbal_control.py), and returns the reply frame, or None when the
real camera stays silent. Photos, SD formatting and the 0x0B push are delegated to "hooks" so this
file has no sockets, threads or Gazebo in it and can be unit-tested directly.

Replies (payload formats from video_cam.md 4.3 / 19.A and the SIYI SDK manual V0.1.1):

    cmd   name                 reply (all little-endian)                         strict mode
    0x01  firmware version     uint32 x2 (A8 mini) or x3 (ZR10)                  answered
    0x02  hardware id          12 ASCII chars, '6B..' ZR10 / '73..' A8 mini      answered
    0x04  auto focus           01                                                SILENT
    0x05  manual zoom          uint16 zoom*10 (zoom right now)                   answered
    0x06  manual focus         01                                                SILENT
    0x07  gimbal speed         01                                                answered
    0x08  centre               01                                                answered
    0x0A  system info          7/8 bytes, [3] record_sta, [4] gimbal mode        answered
    0x0C  photo / mode         never answered; photo -> 0x0B push (hook)         -
    0x0D  attitude             int16 x6: yaw,pitch,roll [deg*10], rates [deg/s*10] answered
    0x0E  set angles           int16 x3: CURRENT yaw,pitch,roll [deg*10]         answered
    0x0F  absolute zoom        01                                                answered
    0x16  max zoom             int part, tenths                                  SILENT
    0x18  current zoom         int part, tenths                                  SILENT
    0x19  gimbal mode          0 lock / 1 follow / 2 fpv                         SILENT
    0x41  single axis          as 0x0E (sent with cmd 0x0E by the A8 mini, 0x41 by the ZR10)
    0x48  format SD            never answered; deletes every photo (hook)        -

Unknown or malformed commands are ignored with a one-time warning; nothing here can crash the
emulator. Replies echo the request SEQ and use CTRL = 0x02 (ack_pack); video_cam checks neither.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import Counter
from typing import Callable, Optional, Tuple, Union

from . import sdk_protocol as sp
from .camera_state import CameraState
from .gimbal_control import GimbalModel

Addr = Optional[Tuple[str, int]]
Attitude = Tuple[float, float, float, float, float, float]   # yaw, pitch, roll [deg], rates [deg/s]
HandlerResult = Union[None, bytes, Tuple[int, bytes]]


class HandlerHooks:
    """What the command handler needs from the rest of the emulator. Override in the emulator/tests.

    Default implementations do nothing (photos are not taken, attitude comes from the commanded
    joints), which is enough for pure unit tests of the reply formats.
    """

    def __init__(self, gimbal: Optional[GimbalModel] = None) -> None:
        self._gimbal = gimbal

    def attitude(self) -> Attitude:
        """Current reported attitude: (yaw, pitch, roll) deg and their rates deg/s."""
        if self._gimbal is None:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        yaw, pitch, roll = self._gimbal.reported_attitude(self._gimbal.joint_cmd)
        return (yaw, pitch, roll, 0.0, 0.0, 0.0)

    def take_photo(self, addr: Addr) -> None:
        """0x0C func 0 arrived from ``addr``: take a photo and push 0x0B to ``addr``."""

    def push_feedback(self, addr: Addr, info_type: int) -> None:
        """Send an unsolicited 0x0B frame with ``info_type`` to ``addr``."""

    def format_sd(self) -> int:
        """0x48: delete every photo; return how many files were deleted."""
        return 0


class CommandHandler:
    """Turns SDK frames into state changes and replies.

    Args:
        camera:  the :class:`CameraState` (model profile, zoom, strict flag, SD presence ...).
        gimbal:  the :class:`GimbalModel`.
        hooks:   :class:`HandlerHooks` (photo, 0x0B push, SD format, measured attitude).
        lock:    lock shared with the emulator's control loop (an RLock; created if None).
        clock:   monotonic time source in seconds (injectable for tests).
        logger:  ``logging.Logger``; ``log_commands=True`` logs every command at INFO.
    """

    def __init__(self, camera: CameraState, gimbal: GimbalModel, hooks: Optional[HandlerHooks] = None,
                 lock: Optional[threading.RLock] = None, clock: Callable[[], float] = time.monotonic,
                 logger: Optional[logging.Logger] = None, log_commands: bool = False) -> None:
        self.camera = camera
        self.gimbal = gimbal
        self.hooks = hooks or HandlerHooks(gimbal)
        self.lock = lock or threading.RLock()
        self.clock = clock
        self.log = logger or logging.getLogger("siyi_emulator.commands")
        self.log_commands = bool(log_commands)
        self.stats: Counter = Counter()          # cmd_id -> how many received
        self._warned: set = set()
        self._dispatch = {
            sp.CMD_HEARTBEAT: self._heartbeat,
            sp.CMD_FIRMWARE_VERSION: self._firmware,
            sp.CMD_HARDWARE_ID: self._hardware_id,
            sp.CMD_AUTO_FOCUS: self._ack_only,
            sp.CMD_MANUAL_ZOOM: self._manual_zoom,
            sp.CMD_MANUAL_FOCUS: self._ack_only,
            sp.CMD_GIMBAL_ROTATION: self._rotation,
            sp.CMD_CENTER: self._center,
            sp.CMD_SYSTEM_INFO: self._system_info,
            sp.CMD_FUNCTION_FEEDBACK: self._ignore_feedback,
            sp.CMD_CAPTURE: self._capture,
            sp.CMD_GIMBAL_ATTITUDE: self._attitude,
            sp.CMD_SET_ANGLES: self._set_angles,
            sp.CMD_ABSOLUTE_ZOOM: self._absolute_zoom,
            sp.CMD_ZOOM_RANGE: self._zoom_range,
            sp.CMD_CURRENT_ZOOM: self._current_zoom,
            sp.CMD_GIMBAL_MODE: self._gimbal_mode,
            sp.CMD_SINGLE_AXIS: self._single_axis,
            sp.CMD_FORMAT_SD: self._format_sd,
        }

    # ------------------------------------------------------------------ public API
    def handle(self, frame: sp.Frame, addr: Addr = None) -> Optional[sp.Frame]:
        """Process one frame. Returns the reply frame, or None when the camera stays silent."""
        cmd = frame.cmd_id
        with self.lock:
            self.stats[cmd] += 1
        level = logging.INFO if self.log_commands else logging.DEBUG
        self.log.log(level, "SDK <- 0x%02X %s seq=%d data=[%s] from %s", cmd, frame.name, frame.seq,
                     sp.hexdump(frame.data), _fmt_addr(addr))
        func = self._dispatch.get(cmd)
        if func is None:
            self._warn_once(("unknown", cmd), f"SDK command 0x{cmd:02X} is not emulated - ignored (no reply)")
            return None
        if self.camera.is_silent(cmd):
            self._info_once(("strict", cmd), f"strict mode: not answering 0x{cmd:02X} ({frame.name}), "
                                             "like a real A8 mini")
            return None
        try:
            with self.lock:
                result = func(frame, addr)
        except (sp.FrameError, ValueError) as exc:
            self._warn_once(("bad", cmd), f"malformed 0x{cmd:02X} ({frame.name}) ignored: {exc}")
            return None
        except Exception as exc:  # pragma: no cover - defensive: never let a command crash the server
            self.log.error("error while handling 0x%02X (%s): %s", cmd, frame.name, exc)
            return None
        if result is None:
            return None
        reply_cmd, payload = result if isinstance(result, tuple) else (cmd, result)
        reply = sp.Frame(reply_cmd, bytes(payload), seq=frame.seq, ctrl=sp.CTRL_ACK_PACK)
        self.log.log(level, "SDK -> 0x%02X data=[%s]", reply_cmd, sp.hexdump(reply.data))
        return reply

    def commands_received(self) -> int:
        """Total number of frames handled so far (thread-safe)."""
        with self.lock:
            return int(sum(self.stats.values()))

    # ------------------------------------------------------------------ helpers
    def _warn_once(self, key, text: str) -> None:
        if key not in self._warned:
            self._warned.add(key)
            self.log.warning(text)

    def _info_once(self, key, text: str) -> None:
        if key not in self._warned:
            self._warned.add(key)
            self.log.info(text)

    def _current_angles(self) -> bytes:
        yaw, pitch, roll = self.hooks.attitude()[:3]
        return sp.encode_angles(yaw, pitch, roll)

    @staticmethod
    def _need(frame: sp.Frame, n: int) -> None:
        if len(frame.data) < n:
            raise sp.FrameError(f"payload needs {n} byte(s), got {len(frame.data)}")

    # ------------------------------------------------------------------ commands
    def _heartbeat(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        self._info_once("heartbeat", "TCP heartbeat (0x00) received over UDP - ignored, as on the real camera")
        return None

    def _firmware(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        return self.camera.firmware_payload()

    def _hardware_id(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        return self.camera.hardware_id_payload()

    def _ack_only(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        """0x04 auto focus / 0x06 manual focus: the emulated lens is always sharp; just say OK."""
        return b"\x01"

    def _manual_zoom(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        self._need(frame, 1)
        direction = int.from_bytes(frame.data[:1], "little", signed=True)
        now = self.clock()
        here = self.camera.zoom.manual(1 if direction > 0 else (-1 if direction < 0 else 0), now)
        return sp.encode_zoom_x10(round(here * 10.0) / 10.0)

    def _rotation(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        turn_yaw, turn_pitch = sp.decode_rotation_request(frame.data)
        self.gimbal.set_rate_command(turn_yaw, turn_pitch)
        return b"\x01"

    def _center(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        # center_pos: 1 one-key centre, 2 centre downward, 3 centre, 4 downward (SDK 0x08 table)
        pos = frame.data[0] if frame.data else 1
        if pos in (2, 4):
            self.gimbal.set_attitude(0.0, -90.0)
        else:
            self.gimbal.center()
        return b"\x01"

    def _system_info(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        return self.camera.system_info_payload(self.gimbal.mode)

    def _ignore_feedback(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        return None   # 0x0B is something the CAMERA sends; a client sending it is ignored

    def _capture(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        """0x0C: never acknowledged. func 0 = photo (repeated SEQ 0 is fine: no de-duplication)."""
        self._need(frame, 1)
        func = frame.data[0]
        if func == sp.FUNC_TAKE_PHOTO:
            self.hooks.take_photo(addr)
        elif func == sp.FUNC_HDR_TOGGLE:
            self.camera.hdr_on = not self.camera.hdr_on
        elif func == sp.FUNC_RECORD_TOGGLE:
            info = self.camera.toggle_recording()
            if info == sp.FEEDBACK_RECORD_FAILED:
                self.hooks.push_feedback(addr, info)
            self.log.info("recording %s (emulated: no video file is written)",
                          "started" if self.camera.recording else "stopped")
        elif func in (sp.FUNC_LOCK_MODE, sp.FUNC_FOLLOW_MODE, sp.FUNC_FPV_MODE):
            mode = {sp.FUNC_LOCK_MODE: sp.MODE_LOCK, sp.FUNC_FOLLOW_MODE: sp.MODE_FOLLOW,
                    sp.FUNC_FPV_MODE: sp.MODE_FPV}[func]
            if mode != self.gimbal.mode:
                self.log.info("gimbal mode -> %s", sp.MODE_NAMES[mode])
            self.gimbal.set_mode(mode)
        elif func in (sp.FUNC_ENABLE_HDMI, sp.FUNC_ENABLE_CVBS, sp.FUNC_DISABLE_HDMI_CVBS):
            self.camera.video_output = 1 if func == sp.FUNC_ENABLE_CVBS else 0
            self._info_once(("video_out", func), "video output setting stored (takes effect after reboot)")
        elif func == sp.FUNC_TILT_DOWNWARD:
            self.gimbal.set_attitude(0.0, -90.0)
        elif func == sp.FUNC_ZOOM_LINKAGE:
            self._info_once("zoom_linkage", "zoom linkage (0x0C func 10) is not emulated - ignored")
        else:
            self._warn_once(("func", func), f"0x0C func_type {func} is not emulated - ignored")
        return None

    def _attitude(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        return sp.encode_attitude(*self.hooks.attitude())

    def _set_angles(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        yaw, pitch = sp.decode_set_angles_request(frame.data)
        reply = self._current_angles()            # the ACK carries the attitude BEFORE moving
        self.gimbal.set_attitude(yaw, pitch)
        return reply

    def _absolute_zoom(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        zoom = sp.decode_zoom(frame.data)
        target = self.camera.zoom.set_absolute(zoom, self.clock())
        if abs(target - zoom) > 0.05:
            self._info_once(("zoom_clamp", round(zoom, 1)),
                            f"zoom {zoom:.1f}x requested, {self.camera.profile.display_name} limit -> {target:.1f}x")
        return b"\x01"

    def _zoom_range(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        return sp.encode_zoom(self.camera.zoom.max_zoom)

    def _current_zoom(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        return sp.encode_zoom(self.camera.zoom.reported(self.clock()))

    def _gimbal_mode(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        return bytes([self.gimbal.mode])

    def _single_axis(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        angle, axis = sp.decode_single_axis_request(frame.data)
        if axis not in (0, 1):
            raise sp.FrameError(f"axis flag must be 0 (yaw) or 1 (pitch), got {axis}")
        reply = self._current_angles()
        if axis == 0:
            self.gimbal.set_yaw(angle)
        else:
            self.gimbal.set_pitch(angle)
        return (self.camera.profile.single_axis_reply_cmd, reply)

    def _format_sd(self, frame: sp.Frame, addr: Addr) -> HandlerResult:
        deleted = self.hooks.format_sd()
        self.log.info("SD card formatted by 0x48 (%d file(s) deleted)", deleted)
        return None


def _fmt_addr(addr: Addr) -> str:
    return f"{addr[0]}:{addr[1]}" if addr else "-"
