"""
test/test_camera_state.py -- PURPOSE: unit tests of siyi_emulator.camera_state: model profiles and their
hardware IDs (what video_cam's detect_model() parses), the zoom state machine (absolute travel,
manual ramp, limits, the "within 0.6x after 3 s" rule video_cam relies on), and the 0x01/0x02/0x0A
payloads (record_sta never 2 unless "no TF card").
Run: python3 -m pytest test/   (no ROS needed)
"""

import struct

import pytest

from siyi_emulator import sdk_protocol as sp
from siyi_emulator.camera_state import (PROFILES, STRICT_SILENT_COMMANDS, CameraState, ZoomState,
                                        get_profile, normalise_model_name)


def test_profiles_match_video_cam_detection_table():
    # video_cam/config.py:37-44: int(hw_id[:2], 16) -> 0x6B ZR10, 0x73 A8MINI
    zr10, a8 = get_profile("zr10"), get_profile("a8mini")
    assert int(zr10.hardware_id[:2], 16) == 0x6B and len(zr10.hardware_id) == 12
    assert int(a8.hardware_id[:2], 16) == 0x73 and len(a8.hardware_id) == 12
    assert (zr10.photo_width, zr10.photo_height) == (2560, 1440)    # video_cam 2K profile
    assert (a8.photo_width, a8.photo_height) == (3840, 2160)        # video_cam 4K profile
    assert a8.single_axis_reply_cmd == sp.CMD_SET_ANGLES             # "A8 mini responded with 0x0E"
    assert zr10.single_axis_reply_cmd == sp.CMD_SINGLE_AXIS
    for p in PROFILES.values():
        assert (p.yaw_min_deg, p.yaw_max_deg, p.pitch_min_deg, p.pitch_max_deg) == (-135.0, 135.0, -90.0, 25.0)


@pytest.mark.parametrize("name", ["A8 mini", "A8_MINI", "a8-mini", "A8MINI", "a8mini"])
def test_model_name_spellings(name):
    assert normalise_model_name(name) == "a8mini"
    assert get_profile(name).key == "a8mini"


def test_unknown_model_lists_valid_names():
    with pytest.raises(ValueError, match="a8mini, zr10"):
        get_profile("zt30")


def test_profile_overrides():
    p = get_profile("a8mini", max_zoom=6.0, photo_width=1920, photo_height=1080)
    assert p.max_zoom == 6.0 and (p.photo_width, p.photo_height) == (1920, 1080)
    assert p.optical_zoom_max == 1.0          # never more optics than the camera has
    assert get_profile("zr10", max_zoom=1.0).optical_zoom_max == 1.0


def test_strict_silent_set_is_the_a8_mini_table():
    assert STRICT_SILENT_COMMANDS == {0x04, 0x06, 0x16, 0x18, 0x19}
    cam = CameraState(get_profile("zr10"), strict=True)
    assert cam.is_silent(0x18) and not cam.is_silent(0x0D)
    assert not CameraState(get_profile("zr10"), strict=False).is_silent(0x18)


# ------------------------------------------------------------------------------- zoom
def test_absolute_zoom_travels_at_speed_and_stops():
    z = ZoomState(max_zoom=30.0, speed_x_per_s=4.0, now=0.0)
    assert z.current(0.0) == 1.0
    assert z.set_absolute(5.0, now=10.0) == 5.0
    assert z.current(10.5) == pytest.approx(3.0)
    assert z.moving(10.5)
    assert z.current(11.0) == pytest.approx(5.0)
    assert z.current(20.0) == pytest.approx(5.0) and not z.moving(20.0)
    z.set_absolute(2.0, now=20.0)                       # back out
    assert z.current(20.5) == pytest.approx(3.0)
    assert z.current(30.0) == pytest.approx(2.0)


def test_video_cam_rack_rule_within_0p6_after_3s():
    # video_cam: 0x0F(5.0), sleep 3 s, 0x18 must be within ZOOM_TOLERANCE_X = 0.6 (config.py:61-69)
    z = ZoomState(max_zoom=30.0, speed_x_per_s=4.0)
    z.set_absolute(5.0, now=0.0)
    assert abs(z.reported(3.0) - 5.0) <= 0.6


def test_zoom_is_clamped_to_model_limits():
    z = ZoomState(max_zoom=1.0)                         # A8 mini in 4K mode
    assert z.set_absolute(5.0, now=0.0) == 1.0
    assert z.current(10.0) == 1.0
    z30 = ZoomState(max_zoom=30.0, speed_x_per_s=0)     # instant
    assert z30.set_absolute(99.0, now=0.0) == 30.0
    assert z30.set_absolute(0.2, now=0.0) == 1.0


def test_manual_zoom_ramp_and_stop():
    z = ZoomState(max_zoom=10.0, speed_x_per_s=2.0)
    assert z.manual(+1, now=0.0) == 1.0                 # returns the zoom right now
    assert z.current(1.0) == pytest.approx(3.0)
    assert z.manual(0, now=1.5) == pytest.approx(4.0)   # stop
    assert z.current(5.0) == pytest.approx(4.0)
    z.manual(-1, now=5.0)
    assert z.current(100.0) == 1.0                      # hits the lower limit and stays
    z.manual(+1, now=100.0)
    assert z.current(1000.0) == 10.0                    # hits the upper limit and stays


def test_reported_zoom_is_one_decimal():
    z = ZoomState(max_zoom=30.0, speed_x_per_s=1.0)
    z.set_absolute(2.0, now=0.0)
    assert z.reported(0.333) == 1.3


# ------------------------------------------------------------------------------- payloads
def test_system_info_layout_for_both_models():
    zr = CameraState(get_profile("zr10"))
    data = zr.system_info_payload(gimbal_mode=1)
    assert len(data) == 8
    assert data[3] == sp.RECORD_IDLE and data[4] == 1 and data[5] == 1      # idle, follow, normal mount
    a8 = CameraState(get_profile("a8mini"))
    assert len(a8.system_info_payload(0)) == 7                              # A8 mini: no zoom_linkage byte


def test_record_sta_only_2_without_card():
    cam = CameraState(get_profile("zr10"))
    assert cam.record_sta == 0
    assert cam.toggle_recording() == sp.FEEDBACK_RECORD_STARTED and cam.record_sta == 1
    assert cam.toggle_recording() == sp.FEEDBACK_RECORD_STOPPED and cam.record_sta == 0
    cam.sd_card_present = False
    assert cam.record_sta == sp.RECORD_NO_TF_CARD
    assert cam.toggle_recording() == sp.FEEDBACK_RECORD_FAILED


def test_firmware_and_hardware_id_payloads():
    a8 = CameraState(get_profile("a8mini"))
    words = struct.unpack("<II", a8.firmware_payload())
    assert words[0] & 0xFFFFFF == 0x000208          # camera v0.2.8 (SIYI's tested firmware)
    assert len(CameraState(get_profile("zr10")).firmware_payload()) == 12
    assert a8.hardware_id_payload().decode()[:2] == "73"
