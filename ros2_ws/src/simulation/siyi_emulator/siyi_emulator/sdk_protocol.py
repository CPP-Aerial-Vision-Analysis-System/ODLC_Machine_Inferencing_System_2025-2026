"""
siyi_emulator/sdk_protocol.py -- PURPOSE: build and parse SIYI SDK frames
(55 66 | ctrl | data_len | seq | cmd_id | data | CRC16) and pack/unpack the payload of every
command the emulator understands. The emulator uses this to read commands sent by the team's
video_cam driver and to send replies with a correct CRC-16/XMODEM.

Pure Python (standard library only) so it can be unit-tested anywhere and reused by test tools.

Frame layout (SIYI Gimbal Camera External SDK Protocol V0.1.1, chapter 1; all numbers little-endian):

    byte 0-1  STX       0x55 0x66
    byte 2    CTRL      bit0 = need_ack (sender wants a reply), bit1 = ack_pack (this IS a reply)
    byte 3-4  Data_len  number of DATA bytes (uint16)
    byte 5-6  SEQ       frame counter 0..65535 (uint16)
    byte 7    CMD_ID    which command
    byte 8..  DATA      Data_len bytes
    last 2    CRC16     CRC-16/XMODEM (poly 0x1021, init 0) of every byte before it (uint16)

Two frames from video_cam.md section 4.2 are reproduced bit-exactly by build_frame() (see the tests):
    take photo          55 66 01 01 00 00 00 0c 00 34 ce
    set angles (0,-90)  55 66 01 04 00 00 00 0e 00 00 7c fc 4f a4
"""

from __future__ import annotations

import binascii
import struct
from dataclasses import dataclass
from typing import Iterator, List, Optional, Tuple

# ---------------------------------------------------------------------------------------------
# Frame constants
# ---------------------------------------------------------------------------------------------
STX = b"\x55\x66"           # start-of-frame marker ("0x6655, low byte first")
HEADER_LEN = 8              # STX(2) + CTRL(1) + Data_len(2) + SEQ(2) + CMD_ID(1)
CRC_LEN = 2
MIN_FRAME_LEN = HEADER_LEN + CRC_LEN   # a frame with no data is 10 bytes
MAX_DATA_LEN = 4096         # sanity limit: real SIYI frames carry at most a few dozen bytes

CTRL_NEED_ACK = 0x01        # bit0: "please answer this"
CTRL_ACK_PACK = 0x02        # bit1: "this frame is an answer"

# ---------------------------------------------------------------------------------------------
# Command IDs (names follow the SIYI SDK manual; video_cam/config.py uses the same numbers)
# ---------------------------------------------------------------------------------------------
CMD_HEARTBEAT = 0x00          # TCP only; ignored over UDP
CMD_FIRMWARE_VERSION = 0x01
CMD_HARDWARE_ID = 0x02
CMD_AUTO_FOCUS = 0x04
CMD_MANUAL_ZOOM = 0x05        # manual zoom with auto focus
CMD_MANUAL_FOCUS = 0x06
CMD_GIMBAL_ROTATION = 0x07    # rate (speed) control
CMD_CENTER = 0x08             # one-key centering
CMD_SYSTEM_INFO = 0x0A        # camera system information (record_sta, gimbal mode ...)
CMD_FUNCTION_FEEDBACK = 0x0B  # pushed BY the camera after a photo (never sent by a client)
CMD_CAPTURE = 0x0C            # photo / record / gimbal mode ("func_type"), never acknowledged
CMD_GIMBAL_ATTITUDE = 0x0D
CMD_SET_ANGLES = 0x0E
CMD_ABSOLUTE_ZOOM = 0x0F      # absolute zoom with auto focus
CMD_ZOOM_RANGE = 0x16         # max zoom in the current state
CMD_CURRENT_ZOOM = 0x18
CMD_GIMBAL_MODE = 0x19
CMD_SINGLE_AXIS = 0x41        # single-axis attitude control
CMD_FORMAT_SD = 0x48          # format the TF (micro SD) card, never acknowledged

COMMAND_NAMES = {
    CMD_HEARTBEAT: "heartbeat",
    CMD_FIRMWARE_VERSION: "firmware version",
    CMD_HARDWARE_ID: "hardware id",
    CMD_AUTO_FOCUS: "auto focus",
    CMD_MANUAL_ZOOM: "manual zoom",
    CMD_MANUAL_FOCUS: "manual focus",
    CMD_GIMBAL_ROTATION: "gimbal rotation (speed)",
    CMD_CENTER: "center",
    CMD_SYSTEM_INFO: "system info",
    CMD_FUNCTION_FEEDBACK: "function feedback",
    CMD_CAPTURE: "capture / mode (func_type)",
    CMD_GIMBAL_ATTITUDE: "gimbal attitude",
    CMD_SET_ANGLES: "set gimbal angles",
    CMD_ABSOLUTE_ZOOM: "absolute zoom",
    CMD_ZOOM_RANGE: "zoom range",
    CMD_CURRENT_ZOOM: "current zoom",
    CMD_GIMBAL_MODE: "gimbal mode",
    CMD_SINGLE_AXIS: "single-axis angle",
    CMD_FORMAT_SD: "format SD card",
}

# 0x0C func_type values (SDK manual 0x0C table)
FUNC_TAKE_PHOTO = 0
FUNC_HDR_TOGGLE = 1
FUNC_RECORD_TOGGLE = 2
FUNC_LOCK_MODE = 3
FUNC_FOLLOW_MODE = 4
FUNC_FPV_MODE = 5
FUNC_ENABLE_HDMI = 6
FUNC_ENABLE_CVBS = 7
FUNC_DISABLE_HDMI_CVBS = 8
FUNC_TILT_DOWNWARD = 9
FUNC_ZOOM_LINKAGE = 10

# 0x0B info_type values (pushed by the camera)
FEEDBACK_PHOTO_OK = 0
FEEDBACK_PHOTO_FAILED = 1        # "check TF card"
FEEDBACK_HDR_ON = 2
FEEDBACK_HDR_OFF = 3
FEEDBACK_RECORD_FAILED = 4
FEEDBACK_RECORD_STARTED = 5
FEEDBACK_RECORD_STOPPED = 6

# Gimbal motion modes (0x0A byte 4, 0x19 reply)
MODE_LOCK = 0
MODE_FOLLOW = 1
MODE_FPV = 2
MODE_NAMES = {MODE_LOCK: "lock", MODE_FOLLOW: "follow", MODE_FPV: "fpv"}

# 0x0A record_sta values
RECORD_IDLE = 0
RECORD_RECORDING = 1
RECORD_NO_TF_CARD = 2
RECORD_DATA_LOSS = 3

INT16_MIN = -32768
INT16_MAX = 32767


class FrameError(ValueError):
    """Raised when bytes are not a valid SIYI frame (too short, wrong start, bad CRC ...)."""


# ---------------------------------------------------------------------------------------------
# CRC and frame building / parsing
# ---------------------------------------------------------------------------------------------
def crc16_xmodem(data: bytes) -> int:
    """Return the CRC-16/XMODEM of ``data`` (poly 0x1021, init 0x0000, no reflection).

    This is exactly ``binascii.crc_hqx(data, 0)``, the same call video_cam uses
    (camera_interface.py:108-110), so both sides always agree.
    """
    return binascii.crc_hqx(bytes(data), 0)


@dataclass(frozen=True)
class Frame:
    """One SIYI SDK frame.

    Attributes:
        cmd_id: command number (0..255), e.g. 0x0E for "set gimbal angles".
        data:   payload bytes (may be empty).
        seq:    sequence number 0..65535 (video_cam reuses 0 for every photo command).
        ctrl:   CTRL byte; bit0 = need_ack, bit1 = ack_pack.
    """

    cmd_id: int
    data: bytes = b""
    seq: int = 0
    ctrl: int = CTRL_NEED_ACK

    @property
    def need_ack(self) -> bool:
        """True when the sender asked for an answer (CTRL bit0)."""
        return bool(self.ctrl & CTRL_NEED_ACK)

    @property
    def is_ack(self) -> bool:
        """True when this frame is itself an answer (CTRL bit1)."""
        return bool(self.ctrl & CTRL_ACK_PACK)

    @property
    def name(self) -> str:
        """Human-readable command name, e.g. ``'set gimbal angles'``."""
        return command_name(self.cmd_id)

    def to_bytes(self) -> bytes:
        """Serialise the frame (header + data + CRC) ready to send over UDP."""
        return build_frame(self.cmd_id, self.data, self.seq, self.ctrl)


def command_name(cmd_id: int) -> str:
    """Return a readable name for a command ID, e.g. ``0x18 -> 'current zoom'``."""
    return COMMAND_NAMES.get(cmd_id, f"unknown command 0x{cmd_id:02X}")


def build_frame(cmd_id: int, data: bytes = b"", seq: int = 0, ctrl: int = CTRL_NEED_ACK) -> bytes:
    """Build a complete SIYI frame.

    Args:
        cmd_id: command number 0..255.
        data:   payload bytes.
        seq:    sequence number (wrapped to 0..65535).
        ctrl:   CTRL byte (0x01 request needing an answer, 0x02 answer, 0x00 fire-and-forget).

    Returns:
        The frame bytes, CRC included.
    """
    data = bytes(data)
    if len(data) > 0xFFFF:
        raise ValueError(f"payload too long ({len(data)} bytes)")
    head = (
        STX
        + bytes([ctrl & 0xFF])
        + struct.pack("<H", len(data))
        + struct.pack("<H", int(seq) & 0xFFFF)
        + bytes([int(cmd_id) & 0xFF])
    )
    body = head + data
    return body + struct.pack("<H", crc16_xmodem(body))


def parse_frame(buf: bytes) -> Frame:
    """Parse exactly one frame that starts at ``buf[0]``.

    Args:
        buf: bytes that start with 55 66 (extra bytes after the frame are ignored).

    Returns:
        The decoded :class:`Frame`.

    Raises:
        FrameError: too short, wrong start bytes, truncated payload, absurd length or bad CRC.
    """
    buf = bytes(buf)
    if len(buf) < MIN_FRAME_LEN:
        raise FrameError(f"too short ({len(buf)} bytes, need at least {MIN_FRAME_LEN})")
    if buf[:2] != STX:
        raise FrameError(f"bad start bytes {buf[:2].hex(' ')} (expected 55 66)")
    data_len = struct.unpack_from("<H", buf, 3)[0]
    if data_len > MAX_DATA_LEN:
        raise FrameError(f"data length {data_len} is not plausible")
    frame_len = HEADER_LEN + data_len + CRC_LEN
    if len(buf) < frame_len:
        raise FrameError(f"truncated frame ({len(buf)} of {frame_len} bytes)")
    crc_recv = struct.unpack_from("<H", buf, HEADER_LEN + data_len)[0]
    crc_calc = crc16_xmodem(buf[: HEADER_LEN + data_len])
    if crc_recv != crc_calc:
        raise FrameError(f"CRC mismatch (received {crc_recv:04x}, calculated {crc_calc:04x})")
    return Frame(
        cmd_id=buf[7],
        data=buf[HEADER_LEN: HEADER_LEN + data_len],
        seq=struct.unpack_from("<H", buf, 5)[0],
        ctrl=buf[2],
    )


def frame_length_at(buf: bytes, offset: int) -> Optional[int]:
    """Total length of the frame starting at ``offset`` (from its header), or None if unknown yet."""
    if len(buf) - offset < HEADER_LEN:
        return None
    data_len = struct.unpack_from("<H", buf, offset + 3)[0]
    return HEADER_LEN + data_len + CRC_LEN


def split_frames(datagram: bytes) -> Tuple[List[Frame], List[str]]:
    """Extract every valid frame from one UDP datagram.

    A datagram normally holds exactly one frame, but this is tolerant: it skips garbage until the
    next ``55 66``, can return several frames sent back-to-back, and never raises.

    Args:
        datagram: raw bytes received from the socket.

    Returns:
        ``(frames, problems)``: the valid frames in order, and a short text for every piece that
        was rejected (used for one-time warnings).
    """
    frames: List[Frame] = []
    problems: List[str] = []
    buf = bytes(datagram)
    i = 0
    while i < len(buf):
        start = buf.find(STX, i)
        if start < 0:
            if i < len(buf):
                problems.append(f"{len(buf) - i} byte(s) without a 55 66 start marker")
            break
        if start > i:
            problems.append(f"skipped {start - i} garbage byte(s) before 55 66")
        try:
            frame = parse_frame(buf[start:])
        except FrameError as exc:
            problems.append(str(exc))
            i = start + 1          # resynchronise on the next possible start marker
            continue
        frames.append(frame)
        i = start + HEADER_LEN + len(frame.data) + CRC_LEN
    return frames, problems


def iter_frames(datagram: bytes) -> Iterator[Frame]:
    """Yield the valid frames of a datagram (see :func:`split_frames`); problems are dropped."""
    frames, _ = split_frames(datagram)
    yield from frames


# ---------------------------------------------------------------------------------------------
# Payload helpers (angles, attitude, zoom)
# ---------------------------------------------------------------------------------------------
def deg_to_raw(deg: float) -> int:
    """Degrees (or deg/s) -> the protocol's int16 tenths (60.5 -> 605), clamped to int16."""
    raw = int(round(float(deg) * 10.0))
    return max(INT16_MIN, min(INT16_MAX, raw))


def raw_to_deg(raw: int) -> float:
    """The protocol's int16 tenths -> degrees (605 -> 60.5)."""
    return int(raw) / 10.0


def encode_attitude(yaw: float, pitch: float, roll: float,
                    yaw_rate: float = 0.0, pitch_rate: float = 0.0, roll_rate: float = 0.0) -> bytes:
    """0x0D reply: 12 bytes ``<hhhhhh`` = yaw, pitch, roll [deg*10], then their rates [deg/s*10]."""
    return struct.pack("<hhhhhh", deg_to_raw(yaw), deg_to_raw(pitch), deg_to_raw(roll),
                       deg_to_raw(yaw_rate), deg_to_raw(pitch_rate), deg_to_raw(roll_rate))


def decode_attitude(data: bytes) -> Tuple[float, float, float, float, float, float]:
    """Inverse of :func:`encode_attitude`: returns (yaw, pitch, roll, yaw_rate, pitch_rate, roll_rate)."""
    if len(data) < 12:
        raise FrameError(f"attitude payload needs 12 bytes, got {len(data)}")
    return tuple(raw_to_deg(v) for v in struct.unpack_from("<hhhhhh", data, 0))  # type: ignore[return-value]


def encode_angles(yaw: float, pitch: float, roll: float) -> bytes:
    """0x0E / 0x41 reply: 6 bytes ``<hhh`` = current yaw, pitch, roll [deg*10]."""
    return struct.pack("<hhh", deg_to_raw(yaw), deg_to_raw(pitch), deg_to_raw(roll))


def decode_angles(data: bytes) -> Tuple[float, float, float]:
    """Inverse of :func:`encode_angles`: returns (yaw, pitch, roll) in degrees."""
    if len(data) < 6:
        raise FrameError(f"angles payload needs 6 bytes, got {len(data)}")
    return tuple(raw_to_deg(v) for v in struct.unpack_from("<hhh", data, 0))  # type: ignore[return-value]


def encode_set_angles_request(yaw: float, pitch: float) -> bytes:
    """0x0E request payload: ``<hh`` target yaw, pitch [deg*10] (what video_cam sends)."""
    return struct.pack("<hh", deg_to_raw(yaw), deg_to_raw(pitch))


def decode_set_angles_request(data: bytes) -> Tuple[float, float]:
    """Parse a 0x0E request payload -> (yaw_deg, pitch_deg)."""
    if len(data) < 4:
        raise FrameError(f"set-angles payload needs 4 bytes, got {len(data)}")
    yaw_raw, pitch_raw = struct.unpack_from("<hh", data, 0)
    return raw_to_deg(yaw_raw), raw_to_deg(pitch_raw)


def decode_single_axis_request(data: bytes) -> Tuple[float, int]:
    """Parse a 0x41 request payload ``<hB`` -> (angle_deg, axis) with axis 0 = yaw, 1 = pitch."""
    if len(data) < 3:
        raise FrameError(f"single-axis payload needs 3 bytes, got {len(data)}")
    angle_raw, axis = struct.unpack_from("<hB", data, 0)
    return raw_to_deg(angle_raw), axis


def decode_rotation_request(data: bytes) -> Tuple[int, int]:
    """Parse a 0x07 request payload ``<bb`` -> (turn_yaw, turn_pitch), each -100..100."""
    if len(data) < 2:
        raise FrameError(f"rotation payload needs 2 bytes, got {len(data)}")
    yaw, pitch = struct.unpack_from("<bb", data, 0)
    return max(-100, min(100, yaw)), max(-100, min(100, pitch))


def zoom_to_int_tenth(zoom: float) -> Tuple[int, int]:
    """Zoom factor -> (integer part, tenths digit), the 0x0F/0x16/0x18 encoding.

    Rounds to the nearest tenth first, so 4.95 becomes (5, 0) and never (4, 10).
    """
    tenths = int(round(max(0.0, float(zoom)) * 10.0))
    integer, tenth = divmod(tenths, 10)
    return min(integer, 255), tenth


def int_tenth_to_zoom(integer: int, tenth: int) -> float:
    """(integer part, tenths digit) -> zoom factor, e.g. (4, 5) -> 4.5."""
    return int(integer) + int(tenth) / 10.0


def encode_zoom(zoom: float) -> bytes:
    """0x16 / 0x18 reply: 2 bytes (integer part, tenths)."""
    integer, tenth = zoom_to_int_tenth(zoom)
    return bytes([integer, tenth])


def decode_zoom(data: bytes) -> float:
    """Inverse of :func:`encode_zoom` (also parses the 0x0F request payload)."""
    if len(data) < 2:
        raise FrameError(f"zoom payload needs 2 bytes, got {len(data)}")
    return int_tenth_to_zoom(data[0], data[1])


def encode_zoom_x10(zoom: float) -> bytes:
    """0x05 reply: uint16 zoom*10 (e.g. 4.5x -> 45)."""
    return struct.pack("<H", max(0, min(0xFFFF, int(round(float(zoom) * 10.0)))))


def decode_zoom_x10(data: bytes) -> float:
    """Inverse of :func:`encode_zoom_x10`."""
    if len(data) < 2:
        raise FrameError(f"zoom*10 payload needs 2 bytes, got {len(data)}")
    return struct.unpack_from("<H", data, 0)[0] / 10.0


def encode_firmware_word(major: int, minor: int, patch: int, high_byte: int = 0x6E) -> int:
    """Firmware version word as SIYI sends it: 0x6E030203 means v3.2.3 (high byte ignored)."""
    return ((high_byte & 0xFF) << 24) | ((major & 0xFF) << 16) | ((minor & 0xFF) << 8) | (patch & 0xFF)


def hexdump(data: bytes) -> str:
    """Bytes -> ``'55 66 01 ...'`` for log messages."""
    return bytes(data).hex(" ")
