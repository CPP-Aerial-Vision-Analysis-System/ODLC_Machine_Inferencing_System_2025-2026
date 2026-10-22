"""
example_detector/test/test_blob_logic.py -- PURPOSE: unit tests of the template detector's logic (no ROS): the
photo-name rules the team's video_cam uses, and that colour blobs come back as person/tent boxes in
FULL-PHOTO pixels with sane scores. Run:  python3 -m pytest ros2_ws/src/simulation/sim_plugin_examples -q
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from example_detector import blob_logic  # noqa: E402

np = pytest.importorskip('numpy')
cv2 = pytest.importorskip('cv2')


def test_is_photo_skips_temporary_and_sidecar_files():
    assert blob_logic.is_photo('36.215026 , -96.006800.jpg')
    assert blob_logic.is_photo('IMG_0001.JPG')
    assert not blob_logic.is_photo('IMG_0001.tmp.jpg')            # video_cam is still writing it
    assert not blob_logic.is_photo('36.215026 , -96.006800.json')  # the photo's sidecar
    assert not blob_logic.is_photo('.hidden.jpg')


def test_latlon_from_name():
    assert blob_logic.latlon_from_name('36.215026 , -96.006800.jpg') == pytest.approx((36.215026, -96.0068))
    assert blob_logic.latlon_from_name('/some/folder/36.5 , -96.25.jpg') == pytest.approx((36.5, -96.25))
    assert blob_logic.latlon_from_name('IMG_0001.jpg') == (0.0, 0.0)            # no GPS fix at the shutter
    assert blob_logic.latlon_from_name('99.0 , 10.0.jpg') == (0.0, 0.0)          # not a position


def _image_with_patches(width=1280, height=720):
    """A grey-green field with one orange patch (person) and one blue patch (tent), in BGR."""
    img = np.full((height, width, 3), (60, 110, 70), dtype=np.uint8)
    img[100:140, 200:220] = (0, 140, 255)       # orange (BGR) 20 x 40 px, centre (210, 120)
    img[400:480, 900:1000] = (180, 60, 20)      # blue (BGR) 100 x 80 px, centre (950, 440)
    return img


def test_detect_finds_person_and_tent_in_full_photo_pixels():
    dets = blob_logic.detect(_image_with_patches(), min_area_px=100, downscale=4)
    by_class = {d.class_id: d for d in dets}
    assert set(by_class) == {'0', '1'}
    person, tent = by_class['0'], by_class['1']
    assert person.name == 'person' and tent.name == 'tent'
    # centres and sizes come back in full-photo pixels (within the 4x downscale's rounding)
    assert person.cx == pytest.approx(210, abs=6) and person.cy == pytest.approx(120, abs=6)
    assert tent.cx == pytest.approx(950, abs=6) and tent.cy == pytest.approx(440, abs=6)
    assert tent.w == pytest.approx(100, abs=8) and tent.h == pytest.approx(80, abs=8)
    for d in dets:
        assert 0.0 <= d.score <= 1.0


def test_detect_without_downscale_matches():
    dets = blob_logic.detect(_image_with_patches(), min_area_px=100, downscale=1)
    tent = [d for d in dets if d.class_id == '1'][0]
    assert tent.cx == pytest.approx(949.5, abs=1) and tent.w == pytest.approx(100, abs=1)


def test_small_blobs_are_ignored():
    img = _image_with_patches()
    dets = blob_logic.detect(img, min_area_px=5000, downscale=1)      # both patches are smaller
    assert dets == []


def test_empty_field_gives_no_detections():
    img = np.full((360, 640, 3), (60, 110, 70), dtype=np.uint8)
    assert blob_logic.detect(img) == []
