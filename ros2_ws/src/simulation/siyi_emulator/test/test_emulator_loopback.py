"""
test/test_emulator_loopback.py -- PURPOSE: end-to-end tests of the running emulator over real sockets on
127.0.0.1 (UDP SDK + HTTP media), with a fake Gazebo link (conftest.FakeGzLink). Checks the photo flow
exactly as video_cam experiences it (0x0C -> 0x0B within 0.8 s -> new file in the listing -> download
-> decodable 2K/4K JPEG), slow/missing Gazebo frames, "no TF card", strict silence, malformed
datagrams, gimbal joint publishing and stabilisation through odometry, zoom applied to photos, format.
Run: python3 -m pytest test/   (no ROS or Gazebo needed; needs OpenCV + numpy)
"""

import json
import math
import socket
import time

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from conftest import FakeGzLink  # noqa: E402
from siyi_emulator import sdk_protocol as sp  # noqa: E402
from siyi_emulator.emulator import EmulatorConfig, EmulatorStartError, SiyiEmulator  # noqa: E402
from siyi_emulator.gimbal_control import quat_from_rot, rot_from_rpy  # noqa: E402
from siyi_emulator.sdk_client import SiyiClient  # noqa: E402


def client_for(emu, timeout=1.0):
    return SiyiClient("127.0.0.1", emu.udp_port, emu.media_port, timeout=timeout)


def wait_until(pred, timeout=5.0, step=0.05):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = pred()
        if value:
            return value
        time.sleep(step)
    return pred()


def decode(jpeg):
    return cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)


def test_photo_flow_like_video_cam(start_emulator):
    gz = FakeGzLink(frame_delay_s=0.1)
    emu = start_emulator(gz)
    events = []
    emu.photo_listeners.append(events.append)
    with client_for(emu) as c:
        assert c.hardware_id().startswith("6B")
        assert c.directories() == [{"name": "101SIYI_IMG", "path": "101SIYI_IMG"}]
        t0 = time.monotonic()
        info = c.take_photo(feedback_timeout=0.8)             # video_cam's exact constant packet
        feedback_s = time.monotonic() - t0
        assert info == sp.FEEDBACK_PHOTO_OK and feedback_s < 0.8
        items = wait_until(lambda: c.media_list(), timeout=5.0)
        assert [i["name"] for i in items] == ["IMG_0001.jpg"]
        img = decode(c.download(items[0]["url"]))
        assert img.shape == (1440, 2560, 3)                  # ZR10 -> video_cam 2K profile
        ok, reencoded = cv2.imencode(".jpg", img)
        assert ok and len(reencoded) > 30000                 # video_cam 2K minimum file size
    sidecar = json.loads((emu.sd.photo_dir / "IMG_0001.json").read_text())
    assert sidecar["image"]["source"] == "gazebo" and sidecar["camera_model"] == "ZR10"
    assert wait_until(lambda: events) and events[0].photo.name == "IMG_0001.jpg"
    assert gz.triggers_sent == 1


def test_repeated_seq0_photos_each_make_a_file(start_emulator):
    emu = start_emulator(FakeGzLink(frame_delay_s=0.02))
    with client_for(emu) as c:
        for n in range(1, 4):
            assert c.take_photo() == 0
            assert wait_until(lambda: len(c.media_list()) == n, timeout=5.0)
    assert [p.name for p in emu.sd.list_photos()] == ["IMG_0001.jpg", "IMG_0002.jpg", "IMG_0003.jpg"]


def test_slow_gazebo_frame_still_gets_feedback_in_time(start_emulator):
    emu = start_emulator(FakeGzLink(frame_delay_s=1.4), feedback_deadline_s=0.5, retrigger_after_s=5.0)
    with client_for(emu) as c:
        t0 = time.monotonic()
        assert c.take_photo(feedback_timeout=0.8) == 0
        assert time.monotonic() - t0 < 0.8                   # pushed by the deadline timer
        assert wait_until(lambda: c.media_list(), timeout=5.0)


def test_slow_render_on_a_busy_pc_still_gives_a_real_photo(start_emulator):
    # Acceptance run on a loaded PC: renders took > 3 s. With the defaults (10 s wait, re-trigger at 3 s)
    # the real frame must still be used, 0x0B must still come inside 0.8 s, and the extra frame from the
    # re-sent trigger must be dropped (not mistaken for the next photo).
    gz = FakeGzLink(frame_delay_s=4.0)
    emu = start_emulator(gz)
    with client_for(emu) as c:
        t0 = time.monotonic()
        assert c.take_photo(feedback_timeout=0.8) == 0
        assert time.monotonic() - t0 < 0.8
        assert wait_until(lambda: c.media_list(), timeout=8.0)
    meta = json.loads((emu.sd.photo_dir / "IMG_0001.json").read_text())
    assert meta["image"]["source"] == "gazebo" and meta["trigger_to_frame_ms"] > 3500
    assert wait_until(lambda: emu.frames_dropped >= 1, timeout=6.0)       # the re-trigger's frame
    assert emu.photos_saved == 1 and emu.triggers_timed_out == 0


def test_lost_trigger_is_resent(start_emulator):
    gz = FakeGzLink(frame_delay_s=0.05)
    first = {"done": False}
    original = gz.trigger_photo

    def lose_first_trigger():
        if not first["done"]:
            first["done"] = True
            gz.triggers_sent += 1
            return True                  # swallowed, like a trigger sent before Gazebo connected
        return original()

    gz.trigger_photo = lose_first_trigger
    emu = start_emulator(gz, retrigger_after_s=0.3)
    with client_for(emu) as c:
        assert c.take_photo() == 0
        assert wait_until(lambda: c.media_list(), timeout=5.0)
    assert gz.triggers_sent == 2
    assert json.loads((emu.sd.photo_dir / "IMG_0001.json").read_text())["image"]["source"] == "gazebo"


def test_without_gazebo_a_labelled_test_pattern_is_saved(start_emulator):
    emu = start_emulator(None, camera_model="a8mini")         # NullGzLink
    with client_for(emu) as c:
        assert c.take_photo() == 0
        items = wait_until(lambda: c.media_list(), timeout=5.0)
        img = decode(c.download(items[0]["url"]))
        assert img.shape == (2160, 3840, 3)                   # A8 mini -> 4K profile
    assert json.loads((emu.sd.photo_dir / "IMG_0001.json").read_text())["image"]["source"] == "test_pattern"


def test_without_gazebo_and_without_fallback_the_photo_fails(start_emulator):
    emu = start_emulator(None, test_pattern_fallback=False)
    with client_for(emu) as c:
        assert c.take_photo() == sp.FEEDBACK_PHOTO_FAILED
        time.sleep(0.3)
        assert c.media_list() == []


def test_no_tf_card(start_emulator):
    emu = start_emulator(FakeGzLink(), sd_card_present=False)
    with client_for(emu) as c:
        assert c.system_info()["record_sta"] == sp.RECORD_NO_TF_CARD
        assert c.take_photo() == sp.FEEDBACK_PHOTO_FAILED


def test_strict_mode_silence_over_the_network(start_emulator):
    emu = start_emulator(FakeGzLink(), camera_model="a8mini", strict=True)
    with client_for(emu, timeout=0.3) as c:
        assert c.current_zoom() is None and c.zoom_range() is None and c.gimbal_mode() is None
        assert c.auto_focus() is False and c.manual_focus(1) is False
        assert c.hardware_id().startswith("73")
        reply = c.single_axis("pitch", -45)
        assert reply is not None and reply[0] == sp.CMD_SET_ANGLES      # A8 mini answers 0x41 with 0x0E


def test_garbage_does_not_kill_the_server(start_emulator):
    emu = start_emulator(FakeGzLink())
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as raw:
        for junk in (b"", b"\x55\x66", b"hello", b"\x55\x66\x01\xff\xff" + b"\x00" * 20, bytes(range(256))):
            raw.sendto(junk, ("127.0.0.1", emu.udp_port))
    with client_for(emu) as c:
        assert c.hardware_id() is not None


def test_gimbal_targets_reach_gazebo_and_attitude_reports_measured(start_emulator):
    gz = FakeGzLink()
    emu = start_emulator(gz)
    with client_for(emu) as c:
        c.set_angles(30.0, -90.0)
        assert wait_until(lambda: gz.joint_targets and abs(gz.joint_targets[-1][2] + math.pi / 2) < 1e-3, 4.0)
        yaw, roll, pitch = gz.joint_targets[-1]
        assert yaw == pytest.approx(math.radians(30.0), abs=1e-3)
        att = c.attitude()
        assert (att["yaw"], att["pitch"], att["roll"]) == pytest.approx((30.0, -90.0, 0.0), abs=0.2)


def test_stabilisation_uses_drone_odometry(start_emulator):
    gz = FakeGzLink()
    emu = start_emulator(gz, initial_pitch_deg=-45.0)
    q = quat_from_rot(rot_from_rpy(math.radians(15.0), 0.0, 0.0))     # drone rolled 15 deg
    for _ in range(20):
        gz.send_odometry(q)
        time.sleep(0.02)
    assert wait_until(lambda: abs(gz.joint_targets[-1][1]) > math.radians(5.0), 3.0)   # roll joint works
    with client_for(emu) as c:
        att = c.attitude()                                    # camera itself stays level
        assert att["pitch"] == pytest.approx(-45.0, abs=0.5) and att["roll"] == pytest.approx(0.0, abs=0.5)


def test_zoom_is_applied_to_the_photo(start_emulator):
    emu = start_emulator(FakeGzLink(), zoom_speed_x_per_s=0.0)        # instant zoom
    with client_for(emu) as c:
        assert c.absolute_zoom(2.0)
        assert c.current_zoom() == 2.0
        assert c.take_photo() == 0
        items = wait_until(lambda: c.media_list(), timeout=5.0)
        img = decode(c.download(items[0]["url"]))
    white = np.argwhere(img.min(axis=2) > 200)
    height = white[:, 0].max() - white[:, 0].min() + 1
    assert 72 <= height <= 88                              # the 40 px square is twice as big
    meta = json.loads((emu.sd.photo_dir / "IMG_0001.json").read_text())
    assert meta["zoom"]["digital"] == pytest.approx(2.0)
    assert meta["hfov_rad"]["effective"] == pytest.approx(2 * math.atan(math.tan(1.4137 / 2) / 2), abs=1e-6)


def test_optical_zoom_topics_get_plugin_commands(start_emulator):
    gz = FakeGzLink()
    emu = start_emulator(gz, optical_zoom_topics="/sim/camera/photo/zoom", zoom_speed_x_per_s=0.0)
    assert wait_until(lambda: gz.zoom_commands, 2.0)
    assert gz.zoom_commands[-1] == pytest.approx(2.0 / 1.4137, abs=1e-3)       # 1x sent at start
    with client_for(emu) as c:
        c.absolute_zoom(5.0)
        assert wait_until(lambda: abs(gz.zoom_commands[-1] - 5.9105) < 0.01, 2.0)
        assert c.take_photo() == 0
        assert wait_until(lambda: c.media_list(), timeout=5.0)
    meta = json.loads((emu.sd.photo_dir / "IMG_0001.json").read_text())
    assert meta["zoom"] == {"total": 5.0, "optical": 5.0, "digital": 1.0}      # all optics, no crop


def test_format_sd_over_the_network(start_emulator):
    emu = start_emulator(FakeGzLink())
    with client_for(emu) as c:
        c.take_photo()
        assert wait_until(lambda: c.media_list(), timeout=5.0)
        c.format_sd()
        assert wait_until(lambda: c.media_list() == [], timeout=2.0)
        c.take_photo()
        items = wait_until(lambda: c.media_list(), timeout=5.0)
        assert items[0]["name"] == "IMG_0002.jpg"


def test_port_in_use_gives_a_helpful_error(start_emulator, tmp_path):
    emu = start_emulator(FakeGzLink())
    config, _ = EmulatorConfig().updated(dict(bind_ip="127.0.0.1", ctrl_port=emu.udp_port, media_port=0,
                                              sd_card_dir=str(tmp_path / "other")))
    with pytest.raises(EmulatorStartError, match="already used"):
        SiyiEmulator(config, gz_link=FakeGzLink()).start()


def test_status_snapshot(start_emulator):
    emu = start_emulator(FakeGzLink())
    status = emu.status()
    assert status["camera_model"] == "ZR10" and status["gimbal"]["mode"] == "follow"
    assert json.dumps(status)                               # JSON-friendly
