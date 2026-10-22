"""
test/test_command_handler.py -- PURPOSE: unit tests of siyi_emulator.command_handler: the reply format of
EVERY emulated command (cmd id, length, values, SEQ echo, CTRL 0x02), which commands stay silent
(0x0C, 0x48, strict-mode 0x04/0x06/0x16/0x18/0x19), the side effects (gimbal targets, zoom, mode,
photo/format hooks) and that malformed or unknown frames never raise.
Run: python3 -m pytest test/   (no ROS needed)
"""

import struct

import pytest

from siyi_emulator import sdk_protocol as sp
from siyi_emulator.camera_state import CameraState, get_profile
from siyi_emulator.command_handler import CommandHandler, HandlerHooks
from siyi_emulator.gimbal_control import GimbalLimits, GimbalModel


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class RecordingHooks(HandlerHooks):
    def __init__(self, gimbal):
        super().__init__(gimbal)
        self.photos, self.feedback, self.formats = [], [], 0

    def take_photo(self, addr):
        self.photos.append(addr)

    def push_feedback(self, addr, info_type):
        self.feedback.append((addr, info_type))

    def format_sd(self):
        self.formats += 1
        return 4


def make(model="zr10", strict=False):
    profile = get_profile(model)
    cam = CameraState(profile, strict=strict)
    limits = GimbalLimits(profile.yaw_min_deg, profile.yaw_max_deg, profile.pitch_min_deg, profile.pitch_max_deg)
    gimbal = GimbalModel(limits, initial_pitch_deg=0.0)
    hooks = RecordingHooks(gimbal)
    clock = FakeClock()
    return CommandHandler(cam, gimbal, hooks, clock=clock), cam, gimbal, hooks, clock


def ask(handler, cmd, data=b"", seq=7, addr=("10.0.0.9", 5555)):
    return handler.handle(sp.Frame(cmd, data, seq=seq, ctrl=sp.CTRL_NEED_ACK), addr)


def settle(gimbal, seconds=4.0):
    for _ in range(int(seconds / 0.01)):
        gimbal.step(0.01)


def test_reply_echoes_seq_and_uses_ack_ctrl():
    h, *_ = make()
    reply = ask(h, sp.CMD_HARDWARE_ID, seq=1234)
    assert reply.seq == 1234 and reply.ctrl == sp.CTRL_ACK_PACK
    assert sp.parse_frame(reply.to_bytes()) == reply           # valid CRC on the wire


@pytest.mark.parametrize("model,prefix", [("zr10", "6B"), ("a8mini", "73")])
def test_hardware_id(model, prefix):
    h, *_ = make(model)
    reply = ask(h, sp.CMD_HARDWARE_ID)
    assert reply.cmd_id == 0x02 and len(reply.data) == 12
    text = reply.data.decode("ascii")
    assert text[:2] == prefix and int(text[:2], 16) in (0x6B, 0x73)


def test_firmware_lengths():
    assert len(ask(make("a8mini")[0], sp.CMD_FIRMWARE_VERSION).data) == 8
    assert len(ask(make("zr10")[0], sp.CMD_FIRMWARE_VERSION).data) == 12


def test_system_info_record_sta_and_mode():
    h, cam, gimbal, *_ = make()
    data = ask(h, sp.CMD_SYSTEM_INFO).data
    assert len(data) >= 5 and data[3] == 0 and data[4] == 1          # idle, follow
    ask(h, sp.CMD_CAPTURE, bytes([sp.FUNC_LOCK_MODE]))
    assert ask(h, sp.CMD_SYSTEM_INFO).data[4] == 0                    # lock
    cam.sd_card_present = False
    assert ask(h, sp.CMD_SYSTEM_INFO).data[3] == sp.RECORD_NO_TF_CARD


def test_capture_photo_is_never_acknowledged_and_calls_hook_every_time():
    h, _, _, hooks, _ = make()
    photo = sp.parse_frame(bytes.fromhex("55 66 01 01 00 00 00 0c 00 34 ce"))   # video_cam's constant packet
    for _ in range(3):                                                           # same SEQ 0 each time
        assert h.handle(photo, ("1.2.3.4", 99)) is None
    assert hooks.photos == [("1.2.3.4", 99)] * 3


def test_mode_functions_and_gimbal_mode_query():
    h, _, gimbal, *_ = make()
    for func, mode in ((sp.FUNC_LOCK_MODE, 0), (sp.FUNC_FPV_MODE, 2), (sp.FUNC_FOLLOW_MODE, 1)):
        assert ask(h, sp.CMD_CAPTURE, bytes([func])) is None
        assert gimbal.mode == mode
        reply = ask(h, sp.CMD_GIMBAL_MODE)
        assert reply.cmd_id == 0x19 and reply.data == bytes([mode])


def test_record_toggle_and_tilt_down():
    h, cam, gimbal, hooks, _ = make()
    ask(h, sp.CMD_CAPTURE, bytes([sp.FUNC_RECORD_TOGGLE]))
    assert cam.recording and ask(h, sp.CMD_SYSTEM_INFO).data[3] == 1
    ask(h, sp.CMD_CAPTURE, bytes([sp.FUNC_TILT_DOWNWARD]))
    assert gimbal.pitch_tgt == -90.0
    cam.sd_card_present = False
    ask(h, sp.CMD_CAPTURE, bytes([sp.FUNC_RECORD_TOGGLE]))
    assert hooks.feedback[-1][1] == sp.FEEDBACK_RECORD_FAILED


def test_attitude_reply_12_bytes():
    h, _, gimbal, *_ = make()
    gimbal.set_attitude(30.0, -45.0)
    settle(gimbal)
    reply = ask(h, sp.CMD_GIMBAL_ATTITUDE)
    assert reply.cmd_id == 0x0D and len(reply.data) == 12
    yaw, pitch, roll, *_ = sp.decode_attitude(reply.data)
    assert (yaw, pitch, roll) == pytest.approx((30.0, -45.0, 0.0), abs=0.11)


def test_set_angles_replies_current_attitude_then_moves():
    h, _, gimbal, *_ = make()
    reply = ask(h, sp.CMD_SET_ANGLES, sp.encode_set_angles_request(30.0, -90.0))
    assert reply.cmd_id == 0x0E and len(reply.data) == 6
    assert sp.decode_angles(reply.data) == pytest.approx((0.0, 0.0, 0.0), abs=0.11)   # BEFORE moving
    assert (gimbal.yaw_tgt, gimbal.pitch_tgt) == (30.0, -90.0)
    settle(gimbal)
    reply = ask(h, sp.CMD_SET_ANGLES, sp.encode_set_angles_request(30.0, -90.0))
    assert sp.decode_angles(reply.data) == pytest.approx((30.0, -90.0, 0.0), abs=0.11)


def test_set_angles_is_clamped():
    h, _, gimbal, *_ = make()
    ask(h, sp.CMD_SET_ANGLES, sp.encode_set_angles_request(170.0, 60.0))
    assert (gimbal.yaw_tgt, gimbal.pitch_tgt) == (135.0, 25.0)


@pytest.mark.parametrize("model,reply_cmd", [("a8mini", 0x0E), ("zr10", 0x41)])
def test_single_axis_reply_cmd_per_model(model, reply_cmd):
    h, _, gimbal, *_ = make(model)
    reply = ask(h, sp.CMD_SINGLE_AXIS, struct.pack("<hB", -450, 1))      # pitch -45
    assert reply.cmd_id == reply_cmd and len(reply.data) == 6
    assert gimbal.pitch_tgt == -45.0 and gimbal.yaw_tgt == 0.0
    ask(h, sp.CMD_SINGLE_AXIS, struct.pack("<hB", 200, 0))               # yaw +20
    assert gimbal.yaw_tgt == 20.0 and gimbal.pitch_tgt == -45.0


def test_single_axis_bad_axis_is_ignored():
    h, *_ = make()
    assert ask(h, sp.CMD_SINGLE_AXIS, struct.pack("<hB", 10, 5)) is None


def test_rotation_and_center():
    h, _, gimbal, *_ = make()
    reply = ask(h, sp.CMD_GIMBAL_ROTATION, struct.pack("<bb", 100, 0))
    assert reply.cmd_id == 0x07 and reply.data == b"\x01"
    assert gimbal.rate_yaw_dps == pytest.approx(-90.0)            # +100 = right
    reply = ask(h, sp.CMD_GIMBAL_ROTATION, struct.pack("<bb", 0, 0))
    assert not gimbal.rate_active
    gimbal.set_attitude(40.0, -40.0)
    assert ask(h, sp.CMD_CENTER, b"\x01").data == b"\x01"
    assert (gimbal.yaw_tgt, gimbal.pitch_tgt) == (0.0, 0.0)
    ask(h, sp.CMD_CENTER, b"\x02")                                  # "centre downward"
    assert gimbal.pitch_tgt == -90.0
    ask(h, sp.CMD_CENTER, b"")                                      # no payload = one-key centre
    assert gimbal.pitch_tgt == 0.0


def test_zoom_commands():
    h, cam, _, _, clock = make("zr10")
    assert ask(h, sp.CMD_ZOOM_RANGE).data == bytes([30, 0])
    assert ask(h, sp.CMD_CURRENT_ZOOM).data == bytes([1, 0])
    reply = ask(h, sp.CMD_ABSOLUTE_ZOOM, bytes([5, 0]))
    assert reply.cmd_id == 0x0F and reply.data == b"\x01"
    clock.t += 3.0                                                     # video_cam sleeps 3 s
    integer, tenth = ask(h, sp.CMD_CURRENT_ZOOM).data
    assert abs(integer + tenth / 10.0 - 5.0) <= 0.6
    reply = ask(h, sp.CMD_MANUAL_ZOOM, struct.pack("<b", 1))           # zoom in
    assert reply.cmd_id == 0x05 and struct.unpack("<H", reply.data)[0] == 50
    clock.t += 1.0
    reply = ask(h, sp.CMD_MANUAL_ZOOM, struct.pack("<b", 0))           # stop
    assert struct.unpack("<H", reply.data)[0] == 90                    # 5 + 4 x/s * 1 s
    ask(h, sp.CMD_ABSOLUTE_ZOOM, bytes([99, 0]))                        # beyond the max -> clamped
    assert cam.zoom.target == 30.0


def test_a8mini_does_not_zoom_in_4k():
    h, cam, _, _, clock = make("a8mini")
    ask(h, sp.CMD_ABSOLUTE_ZOOM, bytes([5, 0]))
    clock.t += 3.0
    assert ask(h, sp.CMD_CURRENT_ZOOM).data == bytes([1, 0])


def test_focus_commands_ack_in_permissive_mode():
    h, *_ = make()
    assert ask(h, sp.CMD_AUTO_FOCUS, struct.pack("<BHH", 1, 960, 540)).data == b"\x01"
    assert ask(h, sp.CMD_MANUAL_FOCUS, struct.pack("<b", 1)).data == b"\x01"


@pytest.mark.parametrize("cmd,data", [
    (sp.CMD_AUTO_FOCUS, struct.pack("<BHH", 1, 960, 540)),
    (sp.CMD_MANUAL_FOCUS, struct.pack("<b", 1)),
    (sp.CMD_ZOOM_RANGE, b""),
    (sp.CMD_CURRENT_ZOOM, b""),
    (sp.CMD_GIMBAL_MODE, b""),
])
def test_strict_mode_is_silent_like_a8_mini(cmd, data):
    h, *_ = make("a8mini", strict=True)
    assert ask(h, cmd, data) is None


def test_strict_mode_still_answers_the_rest():
    h, *_ = make("a8mini", strict=True)
    for cmd, data in ((sp.CMD_HARDWARE_ID, b""), (sp.CMD_SYSTEM_INFO, b""), (sp.CMD_GIMBAL_ATTITUDE, b""),
                      (sp.CMD_SET_ANGLES, sp.encode_set_angles_request(0, -90)), (sp.CMD_ABSOLUTE_ZOOM, b"\x02\x00"),
                      (sp.CMD_MANUAL_ZOOM, b"\x01"), (sp.CMD_GIMBAL_ROTATION, b"\x00\x00"), (sp.CMD_CENTER, b"\x01")):
        assert ask(h, cmd, data) is not None, hex(cmd)


def test_format_sd_is_silent_and_calls_hook():
    h, _, _, hooks, _ = make()
    assert ask(h, sp.CMD_FORMAT_SD) is None
    assert hooks.formats == 1


def test_unknown_malformed_and_client_feedback_are_ignored():
    h, *_ = make()
    assert ask(h, 0x99) is None                                   # unknown command
    assert ask(h, sp.CMD_SET_ANGLES, b"\x01") is None             # payload too short
    assert ask(h, sp.CMD_CAPTURE, b"") is None                    # no func_type
    assert ask(h, sp.CMD_MANUAL_ZOOM, b"") is None
    assert ask(h, sp.CMD_FUNCTION_FEEDBACK, b"\x00") is None      # 0x0B from a client
    assert ask(h, sp.CMD_HEARTBEAT, b"\x00") is None              # TCP heartbeat over UDP
    assert ask(h, sp.CMD_CAPTURE, b"\x2a") is None                # unknown func_type
    assert h.stats[0x99] == 1
