"""
siyi_emulator/camera_state.py -- PURPOSE: everything the emulated camera "remembers" that is not the
gimbal: which camera model it pretends to be (ZR10 or A8 mini), the zoom (with realistic travel
time), recording / HDR flags, whether the micro SD ("TF") card is present, and the bytes of the
0x01 / 0x02 / 0x0A answers.

Pure Python. Time is passed in explicitly (``now`` in seconds from any monotonic clock), so the
zoom state machine can be tested without sleeping.

Model profiles (sources in the comments of PROFILES below):

    key      product id  photo size   zoom              yaw range     pitch range   0x41 answer
    zr10     0x6B '6B'   2560x1440    1-30x (10x opt.)  -135..135     -90..25       0x41
    a8mini   0x73 '73'   3840x2160    1x at 4K (*)      -135..135     -90..25       0x0E

(*) "Zoom is not supported in 4K mode" (A8 mini manual v1.10 p.16); override with max_zoom.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field, replace
from typing import Dict, Optional, Tuple

from . import sdk_protocol as sp


# ---------------------------------------------------------------------------------------------
# Camera model profiles
# ---------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class CameraProfile:
    """Fixed facts about one SIYI camera model.

    Attributes:
        key:                 short name used in parameters ('zr10' or 'a8mini').
        display_name:        what video_cam prints ('ZR10', 'A8MINI').
        product_id:          first byte of the hardware ID (0x6B ZR10, 0x73 A8 mini).
        photo_width/height:  still photo size in pixels.
        max_zoom:            highest zoom factor the camera accepts (x).
        optical_zoom_max:    part of max_zoom that is real optics (x); the rest is digital.
        yaw_min/max_deg:     commandable yaw range for 0x0E / 0x41 / 0x07 (deg, + = left).
        pitch_min/max_deg:   commandable pitch range (deg, + = up, -90 = straight down).
        single_axis_reply_cmd: command ID of the answer to 0x41 (the A8 mini answers 0x0E).
        system_info_len:     length of the 0x0A answer (7 on the A8 mini, 8 with zoom_linkage).
        firmware_versions:   (major, minor, patch) triples sent in the 0x01 answer.
        hfov_deg:            real lens horizontal field of view at 1x (for documentation and the
                             photo metadata fallback when Gazebo sends no camera_info).
    """

    key: str
    display_name: str
    product_id: int
    photo_width: int
    photo_height: int
    max_zoom: float
    optical_zoom_max: float
    yaw_min_deg: float
    yaw_max_deg: float
    pitch_min_deg: float
    pitch_max_deg: float
    single_axis_reply_cmd: int
    system_info_len: int
    firmware_versions: Tuple[Tuple[int, int, int], ...]
    hfov_deg: float

    @property
    def hardware_id(self) -> str:
        """12-character ASCII hardware ID; the first two characters are the product ID in hex.

        video_cam does ``int(hw_id[:2], 16)`` (camera_interface.py:334) and maps 0x6B -> ZR10,
        0x73 -> A8MINI (config.py:37-44). The other 10 characters only need to be printable.
        """
        return f"{self.product_id:02X}" + "0000000001"


PROFILES: Dict[str, CameraProfile] = {
    # SIYI ZR10: 10x optical / 30x hybrid zoom, 2K stills (ZR10 manual v1.7 p.14: "Still Photo
    # Resolution 2K (2560 x 1440)", HFOV 61.5 deg at 1x). Angle range for 0x0E: "ZR10: Same as A8
    # mini" (SDK V0.1.1 0x0E text); the manual lists +-160 deg yaw mechanically. Last year's code
    # defaults to this camera (video_cam config.py:49 DEFAULT_CAMERA_MODEL = 'ZR10').
    "zr10": CameraProfile(
        key="zr10", display_name="ZR10", product_id=0x6B,
        photo_width=2560, photo_height=1440,
        max_zoom=30.0, optical_zoom_max=10.0,
        yaw_min_deg=-135.0, yaw_max_deg=135.0, pitch_min_deg=-90.0, pitch_max_deg=25.0,
        single_axis_reply_cmd=sp.CMD_SINGLE_AXIS, system_info_len=8,
        firmware_versions=((0, 3, 4), (0, 3, 9), (0, 1, 6)),
        hfov_deg=61.5,
    ),
    # SIYI A8 mini: fixed lens, 81 deg HFOV, 4K stills, 6x digital zoom but "Zoom is not supported
    # in 4K mode" (A8 mini manual v1.10 p.15-16). Answers 0x41 with a 0x0E frame (SDK appendix 2,
    # "A8 mini responded with 0x0E"). 0x0A answer is 7 bytes (no zoom_linkage). Firmware numbers
    # are the ones SIYI tested the SDK with (camera v0.2.8, gimbal v0.3.9).
    "a8mini": CameraProfile(
        key="a8mini", display_name="A8MINI", product_id=0x73,
        photo_width=3840, photo_height=2160,
        max_zoom=1.0, optical_zoom_max=1.0,
        yaw_min_deg=-135.0, yaw_max_deg=135.0, pitch_min_deg=-90.0, pitch_max_deg=25.0,
        single_axis_reply_cmd=sp.CMD_SET_ANGLES, system_info_len=7,
        firmware_versions=((0, 2, 8), (0, 3, 9)),
        hfov_deg=81.0,
    ),
}

# Commands the real A8 mini does NOT answer over UDP (SIYI SDK appendix 2 support table; see
# video_cam.md section 0.3 and 19.A). In strict mode the emulator stays silent on these.
STRICT_SILENT_COMMANDS = frozenset({
    sp.CMD_AUTO_FOCUS,      # 0x04 "Only supported by optical zoom cameras"
    sp.CMD_MANUAL_FOCUS,    # 0x06 idem
    sp.CMD_ZOOM_RANGE,      # 0x16
    sp.CMD_CURRENT_ZOOM,    # 0x18
    sp.CMD_GIMBAL_MODE,     # 0x19
})


def normalise_model_name(name: str) -> str:
    """Turn 'A8 mini', 'A8_MINI', 'a8-mini', 'ZR10' ... into a PROFILES key ('a8mini', 'zr10')."""
    key = "".join(ch for ch in str(name).lower() if ch.isalnum())
    return key


def get_profile(name: str, max_zoom: float = 0.0, photo_width: int = 0, photo_height: int = 0) -> CameraProfile:
    """Return the profile for a model name, optionally with some fields overridden.

    Args:
        name:         'zr10' or 'a8mini' (any spelling accepted by :func:`normalise_model_name`).
        max_zoom:     > 0 overrides the profile's maximum zoom (x).
        photo_width:  > 0 overrides the still width (pixels); photo_height likewise.

    Raises:
        ValueError: unknown model name (the message lists the valid ones).
    """
    key = normalise_model_name(name)
    if key not in PROFILES:
        raise ValueError(f"unknown camera_model '{name}'. Valid: {', '.join(sorted(PROFILES))}")
    profile = PROFILES[key]
    changes = {}
    if max_zoom and max_zoom > 0:
        changes["max_zoom"] = max(1.0, float(max_zoom))
        changes["optical_zoom_max"] = min(profile.optical_zoom_max, changes["max_zoom"])
    if photo_width and photo_width > 0:
        changes["photo_width"] = int(photo_width)
    if photo_height and photo_height > 0:
        changes["photo_height"] = int(photo_height)
    return replace(profile, **changes) if changes else profile


# ---------------------------------------------------------------------------------------------
# Zoom: a small state machine with travel time
# ---------------------------------------------------------------------------------------------
class ZoomState:
    """Current zoom factor with a constant travel speed (x per second).

    Two ways to move it, exactly like the SDK:
      * absolute (0x0F): travel towards a target and stop there;
      * manual (0x05): +1 zoom in / -1 zoom out until 0 (stop) or a limit.

    The position is computed from the last event, so nothing needs to be "stepped":
    ``current(now)`` can be asked at any time.

    Args:
        max_zoom:    upper limit (x); the lower limit is always 1.0.
        speed_x_per_s: travel speed; <= 0 means instant.
        now:         time of creation (s).
    """

    MIN_ZOOM = 1.0

    def __init__(self, max_zoom: float, speed_x_per_s: float = 4.0, now: float = 0.0) -> None:
        self.max_zoom = max(self.MIN_ZOOM, float(max_zoom))
        self.speed = float(speed_x_per_s)
        self._t0 = float(now)
        self._z0 = self.MIN_ZOOM
        self._target = self.MIN_ZOOM

    def _clamp(self, zoom: float) -> float:
        return max(self.MIN_ZOOM, min(self.max_zoom, float(zoom)))

    def current(self, now: float) -> float:
        """Zoom factor at time ``now`` (x), between 1.0 and max_zoom."""
        if self.speed <= 0.0:
            return self._target
        dt = max(0.0, float(now) - self._t0)
        travel = self.speed * dt
        if self._target >= self._z0:
            return min(self._target, self._z0 + travel)
        return max(self._target, self._z0 - travel)

    def reported(self, now: float) -> float:
        """Zoom rounded to one decimal, as 0x05 / 0x18 report it."""
        return round(self.current(now) * 10.0) / 10.0

    @property
    def target(self) -> float:
        """Where the zoom is travelling to (x)."""
        return self._target

    def moving(self, now: float) -> bool:
        """True while the zoom has not reached its target."""
        return abs(self.current(now) - self._target) > 1e-9

    def set_absolute(self, zoom: float, now: float) -> float:
        """0x0F: travel to ``zoom`` (clamped to 1..max). Returns the clamped target."""
        self._z0 = self.current(now)
        self._t0 = float(now)
        self._target = self._clamp(zoom)
        return self._target

    def manual(self, direction: int, now: float) -> float:
        """0x05: 1 = zoom in, -1 = zoom out, 0 = stop where it is. Returns the zoom right now."""
        here = self.current(now)
        self._z0 = here
        self._t0 = float(now)
        if direction > 0:
            self._target = self.max_zoom
        elif direction < 0:
            self._target = self.MIN_ZOOM
        else:
            self._target = here
        return here

    def set_max_zoom(self, max_zoom: float, now: float) -> None:
        """Change the upper limit (keeps the current position, clamped)."""
        here = self.current(now)
        self.max_zoom = max(self.MIN_ZOOM, float(max_zoom))
        self._z0 = self._clamp(here)
        self._t0 = float(now)
        self._target = self._clamp(self._target)


# ---------------------------------------------------------------------------------------------
# The camera's remembered state
# ---------------------------------------------------------------------------------------------
@dataclass
class CameraState:
    """Non-gimbal state of the emulated camera.

    Attributes:
        profile:        the camera model being emulated.
        strict:         True = stay silent where the real A8 mini is silent (STRICT_SILENT_COMMANDS).
        sd_card_present: False simulates "no TF card" (0x0A record_sta = 2, photos fail).
        recording:      0x0C func 2 toggles it (record_sta 1); no video file is written.
        hdr_on:         0x0C func 1 toggles it (reported in 0x0A byte 1).
        mounting_inverted: reported in 0x0A byte 5 (1 normal, 2 inverted).
        video_output:   0x0A byte 6 (0 HDMI on / CVBS off, 1 HDMI off / CVBS on).
        zoom:           the :class:`ZoomState`.
    """

    profile: CameraProfile
    strict: bool = False
    sd_card_present: bool = True
    recording: bool = False
    hdr_on: bool = False
    mounting_inverted: bool = False
    video_output: int = 0
    zoom: ZoomState = field(default=None)  # type: ignore[assignment]
    zoom_speed_x_per_s: float = 4.0

    def __post_init__(self) -> None:
        if self.zoom is None:
            self.zoom = ZoomState(self.profile.max_zoom, self.zoom_speed_x_per_s)

    # -------------------------------------------------------------- strict-mode policy
    def is_silent(self, cmd_id: int) -> bool:
        """True when the emulator must NOT answer ``cmd_id`` (strict mode, A8 mini behaviour)."""
        return self.strict and cmd_id in STRICT_SILENT_COMMANDS

    # -------------------------------------------------------------- answers
    @property
    def record_sta(self) -> int:
        """0x0A byte 3: 0 idle, 1 recording, 2 NO TF CARD."""
        if not self.sd_card_present:
            return sp.RECORD_NO_TF_CARD
        return sp.RECORD_RECORDING if self.recording else sp.RECORD_IDLE

    def system_info_payload(self, gimbal_mode: int) -> bytes:
        """0x0A answer bytes.

        Layout (0-based, what video_cam reads at camera_interface.py:455-463):
            [0] reserved  [1] hdr_sta  [2] reserved  [3] record_sta  [4] gimbal_motion_mode
            [5] mounting_dir (1 normal, 2 inverted)  [6] hdmi/cvbs  [7] zoom_linkage (8-byte models)
        """
        payload = bytes([
            0,
            1 if self.hdr_on else 0,
            0,
            self.record_sta,
            int(gimbal_mode) & 0xFF,
            2 if self.mounting_inverted else 1,
            int(self.video_output) & 0xFF,
            0,
        ])
        return payload[: self.profile.system_info_len]

    def firmware_payload(self) -> bytes:
        """0x01 answer: one uint32 per firmware (camera, gimbal[, zoom])."""
        words = [sp.encode_firmware_word(*v) for v in self.profile.firmware_versions]
        return struct.pack("<" + "I" * len(words), *words)

    def hardware_id_payload(self) -> bytes:
        """0x02 answer: the 12-byte ASCII hardware ID."""
        return self.profile.hardware_id.encode("ascii")

    def toggle_recording(self) -> Optional[int]:
        """0x0C func 2. Returns the 0x0B info_type to push (or None when there is no card)."""
        if not self.sd_card_present:
            return sp.FEEDBACK_RECORD_FAILED
        self.recording = not self.recording
        return sp.FEEDBACK_RECORD_STARTED if self.recording else sp.FEEDBACK_RECORD_STOPPED
