"""
test/test_video_cam_compat.py -- PURPOSE: run last year's REAL camera code (video_cam's CameraInterface
and PipelineOrchestrator, unchanged) against the emulator on 127.0.0.1, method by method: model
detection, SD init, the full capture -> index -> download -> verify -> atomic save pipeline (2K and
4K), every gimbal / zoom / focus / mode / format call, and the strict A8-mini silences.
Skipped automatically when video_cam is not importable (build it in the same workspace and source it).
Run: python3 -m pytest test/test_video_cam_compat.py -v
"""

import os
import time

import pytest

camera_interface = pytest.importorskip("video_cam.camera_interface")
pipeline_orchestrator = pytest.importorskip("video_cam.pipeline_orchestrator")
storage_manager = pytest.importorskip("video_cam.storage_manager")

from conftest import FakeGzLink  # noqa: E402


class RecordingLogger:
    """Stands in for the rclpy logger video_cam expects (info/warn/error)."""

    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(("info", str(msg)))

    def warn(self, msg):
        self.lines.append(("warn", str(msg)))

    warning = warn

    def error(self, msg):
        self.lines.append(("error", str(msg)))

    def debug(self, msg):
        pass

    def text(self):
        return "\n".join(f"{lvl}: {m}" for lvl, m in self.lines)


def make_camera(emu, logger, sdk_timeout=None):
    cam = camera_interface.CameraInterface(camera_ip="127.0.0.1", ctrl_port=emu.udp_port,
                                           media_port=emu.media_port, http_timeout=5.0, logger=logger)
    if sdk_timeout is not None:
        cam.sdk_socket.settimeout(sdk_timeout)
    return cam


def capture_with_pipeline(cam, tmp_path, logger, resolution):
    storage = storage_manager.StorageManager(str(tmp_path / "video_cam"), logger=logger)
    pipe = pipeline_orchestrator.PipelineOrchestrator(cam, storage, rotate_180=False, logger=logger)
    pipe.set_resolution(resolution)
    pipe.initialize_sd_card()
    file_info = pipe.capture_and_index()                    # phases 1+2: 0x0C, 0x0B, poll the listing
    assert file_info is not None, logger.text()
    result = pipe.download_and_save(file_info, filename_override="38.316500 , -76.550000.jpg")   # phase 3
    assert result is not None, logger.text()
    return file_info, result


def test_zr10_permissive_full_api(start_emulator, tmp_path):
    emu = start_emulator(FakeGzLink(frame_delay_s=0.1))
    log = RecordingLogger()
    cam = make_camera(emu, log)
    try:
        assert cam.detect_model() == "ZR10"
        info = cam.get_system_info()
        assert info["sd_card_present"] is True and info["record_sta"] == 0 and info["gimbal_motion_mode"] == "follow"

        file_info, (saved_path, img) = capture_with_pipeline(cam, tmp_path, log, "2K")
        assert file_info["name"] == "IMG_0001.jpg"
        assert img.shape == (1440, 2560, 3) and os.path.getsize(saved_path) > 30000
        assert os.path.basename(saved_path) == "38.316500 , -76.550000.jpg"
        assert "No capture feedback received" not in log.text()     # 0x0B arrived inside 0.8 s
        assert "Photo directory: 101SIYI_IMG" in log.text()

        assert cam.set_gimbal_angles(30.0, -45.0) is not None
        time.sleep(1.5)
        att = cam.request_gimbal_attitude()
        assert abs(att["yaw_deg"] - 30.0) < 0.3 and abs(att["pitch_deg"] + 45.0) < 0.3
        assert cam.set_single_axis_angle("yaw", 10.0) is not None
        assert cam.rotate_gimbal(50, 0) is True and cam.stop_gimbal_rotation() is True
        assert cam.center_gimbal() is True

        assert cam.absolute_zoom_autofocus(5.0) is True
        time.sleep(3.0)                                             # what _apply_capture_zoom does
        assert abs(cam.get_current_zoom_magnification() - 5.0) <= 0.6
        assert cam.get_supported_zoom_range() == {"max_zoom": 30.0}
        assert cam.manual_zoom(1) is not None and cam.manual_zoom(0) is not None
        assert cam.auto_focus(960, 540) is True and cam.manual_focus(1) is True and cam.manual_focus(0) is True

        assert cam.set_gimbal_motion_mode("lock") is True
        assert cam.get_gimbal_mode() == "lock"
        assert cam.set_gimbal_motion_mode("follow") is True and cam.get_gimbal_mode() == "follow"

        assert cam.format_sd_card() is True
        time.sleep(0.3)
        assert cam.get_media_list("101SIYI_IMG") == []
    finally:
        cam.close()


def test_a8mini_strict_4k(start_emulator, tmp_path):
    emu = start_emulator(FakeGzLink(width=3840, height=2160, frame_delay_s=0.1), camera_model="a8mini", strict=True)
    log = RecordingLogger()
    cam = make_camera(emu, log, sdk_timeout=0.3)                    # shorten the 2 s silences
    try:
        assert cam.detect_model() == "A8MINI"
        assert cam.get_current_zoom_magnification() is None         # silent like the real A8 mini
        assert cam.get_supported_zoom_range() is None
        assert cam.get_gimbal_mode() is None
        assert cam.auto_focus(960, 540) is False and cam.manual_focus(1) is False
        assert cam.set_single_axis_angle("pitch", -45.0) is not None   # 0x41 answered with 0x0E
        assert cam.get_system_info()["sd_card_present"] is True
        cam.sdk_socket.settimeout(2.0)
        _, (saved_path, img) = capture_with_pipeline(cam, tmp_path, log, "4K")
        assert img.shape == (2160, 3840, 3) and os.path.getsize(saved_path) > 50000
    finally:
        cam.close()


def test_no_tf_card_is_reported_like_the_real_camera(start_emulator):
    emu = start_emulator(FakeGzLink(), sd_card_present=False)
    log = RecordingLogger()
    cam = make_camera(emu, log)
    try:
        assert cam.sd_card_present() is False
        assert cam.send_capture_command("2K") is False              # 0x0B info_type 1 -> rejected
        assert "Capture rejected by camera" in log.text()
    finally:
        cam.close()
