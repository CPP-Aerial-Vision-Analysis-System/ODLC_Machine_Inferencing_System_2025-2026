"""
Unit tests for video_cam/photo_metadata.py (the photo sidecar file).

test_photo_metadata.py -- PURPOSE: unit tests for the JSON "sidecar" video_cam writes next to every
photo (drone position, height, heading, camera field of view, zoom, gimbal angles at the shutter)
so main_controller can aim the payload at the target itself (October 2026). Checks: the sidecar
sits next to the photo with the same name and a .json extension, the photo name contract
("<lat> , <lon>.jpg") is untouched, missing / NaN values become null, the file is written
atomically and reads back, and files of another format are refused.

Needs nothing but Python (no ROS, no build):
    python3 -m pytest ros2_ws/src/video_cam/test/test_photo_metadata.py -v
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from video_cam.photo_metadata import (  # noqa: E402
    SIDECAR_FORMAT,
    build_photo_metadata,
    read_sidecar,
    sidecar_path_for,
    write_sidecar,
)


def example(**over):
    args = dict(image_name='36.214934 , -96.007241.jpg', shutter_unix_s=1792208863.966,
                trigger_unix_s=1792208863.9, lat=36.2149341234, lon=-96.0072412345,
                alt_msl_m=240.12, fix_age_s=0.41, rel_alt_m=30.02, rel_alt_age_s=0.4,
                heading_deg=90.1, heading_age_s=0.4, camera_model='ZR10', hfov_deg=81.0,
                zoom_x=1.0, zoom_source='readback', zoom_changing=False,
                image_top_direction_deg=0.0, rotate_180=False,
                gimbal={'yaw_deg': 0.1, 'pitch_deg': -90.0, 'roll_deg': 0.0})
    args.update(over)
    return build_photo_metadata(**args)


def test_sidecar_is_next_to_the_photo_with_the_same_name():
    assert sidecar_path_for('/x/mapping_photos/36.214934 , -96.007241.jpg') == \
        '/x/mapping_photos/36.214934 , -96.007241.json'
    assert sidecar_path_for('IMG_0042.jpg') == 'IMG_0042.json'


def test_photo_name_contract_is_untouched():
    # detection/geotag.py parses "<lat> , <lon>.jpg"; the sidecar must not change the photo's name.
    meta = example()
    assert meta['image_name'] == '36.214934 , -96.007241.jpg'
    assert ' , ' in meta['image_name'] and meta['image_name'].endswith('.jpg')


def test_fields_and_full_precision_position():
    meta = example()
    assert meta['format'] == SIDECAR_FORMAT
    # kept to 1e-9 degree (0.1 mm), far finer than the 6 decimals of the file name (11 cm)
    assert math.isclose(meta['position']['lat'], 36.2149341234, abs_tol=1e-9)
    assert math.isclose(meta['position']['lon'], -96.0072412345, abs_tol=1e-9)
    assert meta['rel_alt_m'] == 30.02 and meta['heading_deg'] == 90.1
    assert meta['camera']['hfov_deg'] == 81.0 and meta['camera']['zoom_x'] == 1.0
    assert meta['camera']['image_top_direction_deg'] == 0.0
    assert meta['gimbal']['pitch_deg'] == -90.0 and meta['gimbal']['source'] == 'camera 0x0D'


def test_unknown_values_become_null():
    meta = example(heading_deg=None, rel_alt_m=float('nan'), lat=float('inf'), gimbal=None,
                   hfov_deg='not a number')
    assert meta['heading_deg'] is None and meta['rel_alt_m'] is None
    assert meta['position']['lat'] is None and meta['gimbal'] is None
    assert meta['camera']['hfov_deg'] is None
    json.dumps(meta, allow_nan=False)            # still strict JSON (no NaN / Infinity)


def test_write_then_read_back(tmp_path):
    photo = tmp_path / '36.214934 , -96.007241.jpg'
    path = sidecar_path_for(str(photo))
    assert write_sidecar(path, example())
    assert not os.path.exists(path + '.tmp')     # the temporary file was renamed away
    back = read_sidecar(path)
    assert back == example()
    assert math.isclose(back['position']['lat'], 36.214934123)


def test_other_formats_and_missing_files_are_refused(tmp_path):
    path = tmp_path / 'x.json'
    path.write_text(json.dumps({'format': 'something else'}))
    assert read_sidecar(str(path)) is None
    path.write_text('{not json')
    assert read_sidecar(str(path)) is None
    assert read_sidecar(str(tmp_path / 'missing.json')) is None


def test_write_failure_returns_false(tmp_path):
    assert write_sidecar(str(tmp_path / 'no_such_dir' / 'x.json'), example()) is False
