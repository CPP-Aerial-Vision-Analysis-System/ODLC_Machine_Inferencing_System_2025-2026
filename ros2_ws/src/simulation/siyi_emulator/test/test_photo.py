"""
test/test_photo.py -- PURPOSE: unit tests of siyi_emulator.photo: gz.msgs.Image byte layouts -> OpenCV
BGR (RGB/BGR/RGBA/grey, padded rows), digital zoom as a centre crop + resize, the zoom/FOV maths
including the CameraZoomPlugin command, JPEG encoding, and the test pattern (decodable, big enough
that video_cam's OpenCV re-encode stays above its 50 kB minimum).
Run: python3 -m pytest test/   (no ROS needed; needs OpenCV + numpy)
"""

import math

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from siyi_emulator import photo as ph  # noqa: E402


def test_rgb_to_bgr_and_padded_rows():
    w, h = 4, 2
    rgb = np.zeros((h, w, 3), np.uint8)
    rgb[:, :, 0] = 255                              # pure red in RGB
    step = w * 3 + 4                                # 4 padding bytes per row
    padded = np.zeros((h, step), np.uint8)
    padded[:, : w * 3] = rgb.reshape(h, w * 3)
    bgr = ph.gz_image_to_bgr(w, h, step, padded.tobytes(), ph.PF_RGB_INT8)
    assert bgr.shape == (h, w, 3)
    assert (bgr[:, :, 2] == 255).all() and (bgr[:, :, 0] == 0).all()


@pytest.mark.parametrize("fmt,channels", [(ph.PF_BGR_INT8, 3), (ph.PF_RGBA_INT8, 4), (ph.PF_BGRA_INT8, 4), (ph.PF_L_INT8, 1)])
def test_other_pixel_formats(fmt, channels):
    data = (np.arange(2 * 3 * channels) % 255).astype(np.uint8).tobytes()
    out = ph.gz_image_to_bgr(3, 2, 0, data, fmt)
    assert out.shape == (2, 3, 3) and out.dtype == np.uint8


def test_bad_images_raise_value_error():
    with pytest.raises(ValueError, match="unsupported"):
        ph.gz_image_to_bgr(2, 2, 0, b"\x00" * 12, 14)
    with pytest.raises(ValueError, match="too short"):
        ph.gz_image_to_bgr(10, 10, 0, b"\x00" * 10, ph.PF_RGB_INT8)


def test_crop_rect_and_digital_zoom():
    assert ph.crop_rect(2560, 1440, 1.0) == (0, 0, 2560, 1440)
    assert ph.crop_rect(2560, 1440, 2.0) == (640, 360, 1280, 720)
    img = np.zeros((1440, 2560, 3), np.uint8)
    img[700:740, 1260:1300] = 255                  # 40x40 white square in the centre
    zoomed = ph.apply_digital_zoom(img, 2.0, (2560, 1440))
    assert zoomed.shape == (1440, 2560, 3)
    white = np.argwhere(zoomed[:, :, 0] > 128)
    height = white[:, 0].max() - white[:, 0].min() + 1
    assert 76 <= height <= 84                       # the square doubled in size
    assert ph.apply_digital_zoom(img, 1.0, (2560, 1440)) is img      # 1x = untouched


def test_split_zoom():
    assert ph.split_zoom(5.0, 10.0) == (5.0, 1.0)
    assert ph.split_zoom(30.0, 10.0) == (10.0, 3.0)
    assert ph.split_zoom(5.0, 1.0) == (1.0, 5.0)
    assert ph.split_zoom(0.5, 10.0) == (1.0, 1.0)


def test_zoomed_hfov_and_plugin_command():
    hfov = 1.4137
    assert ph.zoomed_hfov(hfov, 1.0) == pytest.approx(hfov)
    assert math.tan(ph.zoomed_hfov(hfov, 5.0) / 2) == pytest.approx(math.tan(hfov / 2) / 5)
    # CameraZoomPlugin: HFOV = 2.0 / cmd, so 1x must be sent as 2.0/1.4137 and 5x as 2.0/zoomed
    assert ph.zoom_plugin_command(1.0, hfov) == pytest.approx(2.0 / hfov)
    assert ph.zoom_plugin_command(5.0, hfov) == pytest.approx(5.910, abs=0.01)
    assert ph.hfov_from_camera_info(2560, (2560 / 2) / math.tan(hfov / 2)) == pytest.approx(hfov)
    assert ph.hfov_from_camera_info(0, 100.0) is None


def test_jpeg_and_test_pattern_size_like_video_cam_checks():
    pattern = ph.make_test_pattern(3840, 2160, ["IMG_0001.jpg"])
    jpeg = ph.encode_jpeg(pattern, 92)
    decoded = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (2160, 3840, 3)
    ok, reencoded = cv2.imencode(".jpg", decoded)   # what video_cam's storage_manager does
    assert ok and len(reencoded) >= 50000           # 4K profile minimum (video_cam config.py:202)


def test_resize_to():
    img = np.zeros((10, 20, 3), np.uint8)
    assert ph.resize_to(img, (20, 10)) is img
    assert ph.resize_to(img, (40, 20)).shape == (20, 40, 3)
