"""
test/test_sdk_protocol.py -- PURPOSE: unit tests of siyi_emulator.sdk_protocol: CRC-16/XMODEM, frame
building/parsing (the two known frames of video_cam.md 4.2 reproduced bit-exactly, plus SIYI's own
example frames), tolerant splitting of datagrams, and every payload encoder/decoder.
Run: python3 -m pytest test/   (no ROS needed)
"""

import struct

import pytest

from siyi_emulator import sdk_protocol as sp

# Frames copied from video_cam.md section 4.2 (and video_cam/config.py:131) and the SIYI SDK examples.
TAKE_PHOTO = bytes.fromhex("55 66 01 01 00 00 00 0c 00 34 ce")
SET_ANGLES_0_M90 = bytes.fromhex("55 66 01 04 00 00 00 0e 00 00 7c fc 4f a4")
SIYI_EXAMPLES = [
    "55 66 01 02 00 01 00 0f 04 05 60 bb",   # absolute zoom 4.5x (seq 1)
    "55 66 01 01 00 00 00 0c 02 76 ee",      # start recording
    "55 66 01 02 00 00 00 07 64 64 3d cf",   # pan/tilt 100 100
    "55 66 01 01 00 00 00 08 01 d1 12",      # one-key centering
    "55 66 01 00 00 00 00 0a 0f 75",         # system information
    "55 66 01 01 00 00 00 0c 03 57 fe",      # lock mode
    "55 66 01 01 00 00 00 0c 04 b0 8e",      # follow mode
    "55 66 01 01 00 00 00 0c 05 91 9e",      # FPV mode
    "55 66 01 00 00 00 00 0d e8 05",         # request attitude
    "55 66 01 01 00 00 00 00 00 59 8b",      # TCP heartbeat
]


def test_crc_matches_binascii_and_known_value():
    # The CRC of the take-photo frame body is 0xce34 (sent little-endian as 34 ce).
    assert sp.crc16_xmodem(TAKE_PHOTO[:-2]) == 0xCE34
    assert sp.crc16_xmodem(b"") == 0


def test_take_photo_frame_is_bit_exact():
    assert sp.build_frame(sp.CMD_CAPTURE, bytes([sp.FUNC_TAKE_PHOTO]), seq=0, ctrl=sp.CTRL_NEED_ACK) == TAKE_PHOTO


def test_set_angles_facing_down_frame_is_bit_exact():
    payload = sp.encode_set_angles_request(0.0, -90.0)
    assert payload == bytes.fromhex("00 00 7c fc")
    assert sp.build_frame(sp.CMD_SET_ANGLES, payload, seq=0) == SET_ANGLES_0_M90


@pytest.mark.parametrize("hexframe", SIYI_EXAMPLES)
def test_siyi_example_frames_round_trip(hexframe):
    raw = bytes.fromhex(hexframe)
    frame = sp.parse_frame(raw)
    assert frame.to_bytes() == raw


def test_parse_fields():
    frame = sp.parse_frame(SET_ANGLES_0_M90)
    assert frame.cmd_id == sp.CMD_SET_ANGLES
    assert frame.seq == 0
    assert frame.ctrl == 1 and frame.need_ack and not frame.is_ack
    assert sp.decode_set_angles_request(frame.data) == (0.0, -90.0)
    assert frame.name == "set gimbal angles"


def test_parse_rejects_bad_frames():
    with pytest.raises(sp.FrameError, match="too short"):
        sp.parse_frame(b"\x55\x66\x01")
    with pytest.raises(sp.FrameError, match="bad start"):
        sp.parse_frame(b"\x00" * 12)
    bad_crc = bytearray(TAKE_PHOTO)
    bad_crc[-1] ^= 0xFF
    with pytest.raises(sp.FrameError, match="CRC mismatch"):
        sp.parse_frame(bytes(bad_crc))
    truncated = sp.build_frame(sp.CMD_SET_ANGLES, b"\x01\x02\x03\x04")[:-3]
    with pytest.raises(sp.FrameError, match="truncated"):
        sp.parse_frame(truncated)
    absurd = b"\x55\x66\x01\xff\xff\x00\x00\x0e" + b"\x00" * 4
    with pytest.raises(sp.FrameError, match="not plausible"):
        sp.parse_frame(absurd)


def test_split_frames_is_tolerant():
    datagram = b"\x00\x01garbage" + TAKE_PHOTO + SET_ANGLES_0_M90 + b"\x55\x66\x01"
    frames, problems = sp.split_frames(datagram)
    assert [f.cmd_id for f in frames] == [sp.CMD_CAPTURE, sp.CMD_SET_ANGLES]
    assert problems  # the garbage and the dangling start were reported, not raised


def test_split_frames_resyncs_after_bad_crc():
    bad = bytearray(TAKE_PHOTO)
    bad[-2] ^= 0x55
    frames, problems = sp.split_frames(bytes(bad) + TAKE_PHOTO)
    assert len(frames) == 1 and frames[0].cmd_id == sp.CMD_CAPTURE
    assert any("CRC" in p for p in problems)


def test_split_frames_never_raises_on_random_bytes():
    import random
    rng = random.Random(7)
    for _ in range(300):
        blob = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 80)))
        if rng.random() < 0.3:
            blob = b"\x55\x66" + blob
        sp.split_frames(blob)   # must not raise


def test_seq_wraps_and_ack_ctrl():
    raw = sp.build_frame(0x0D, b"", seq=70000, ctrl=sp.CTRL_ACK_PACK)
    frame = sp.parse_frame(raw)
    assert frame.seq == 70000 & 0xFFFF
    assert frame.is_ack and not frame.need_ack


def test_attitude_and_angles_payloads():
    data = sp.encode_attitude(30.0, -90.0, 1.5, -2.0, 0.0, 12.3)
    assert len(data) == 12
    assert struct.unpack("<hhhhhh", data) == (300, -900, 15, -20, 0, 123)
    assert sp.decode_attitude(data) == (30.0, -90.0, 1.5, -2.0, 0.0, 12.3)
    angles = sp.encode_angles(-135.0, 25.0, 0.0)
    assert len(angles) == 6 and sp.decode_angles(angles) == (-135.0, 25.0, 0.0)


def test_deg_to_raw_rounds_and_clamps():
    assert sp.deg_to_raw(60.5) == 605
    assert sp.deg_to_raw(-0.04) == 0
    assert sp.deg_to_raw(1e6) == sp.INT16_MAX
    assert sp.deg_to_raw(-1e6) == sp.INT16_MIN


def test_zoom_encodings():
    assert sp.zoom_to_int_tenth(4.5) == (4, 5)
    assert sp.zoom_to_int_tenth(4.95) == (5, 0)        # never (4, 10)
    assert sp.zoom_to_int_tenth(30.0) == (30, 0)
    assert sp.encode_zoom(5.0) == b"\x05\x00"
    assert sp.decode_zoom(b"\x04\x05") == 4.5
    assert sp.encode_zoom_x10(4.5) == struct.pack("<H", 45)
    assert sp.decode_zoom_x10(struct.pack("<H", 123)) == 12.3


def test_request_decoders():
    assert sp.decode_single_axis_request(struct.pack("<hB", -450, 1)) == (-45.0, 1)
    assert sp.decode_rotation_request(struct.pack("<bb", 100, -100)) == (100, -100)
    with pytest.raises(sp.FrameError):
        sp.decode_set_angles_request(b"\x00")
    with pytest.raises(sp.FrameError):
        sp.decode_single_axis_request(b"\x00\x00")


def test_firmware_word_layout():
    # SIYI example: 0x6E030203 means v3.2.3 (high byte ignored)
    assert sp.encode_firmware_word(3, 2, 3) == 0x6E030203
