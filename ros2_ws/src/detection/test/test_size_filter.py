"""Tests for detection.size_filter: boxes are judged in metres, using the photo sidecar's height and field of view."""

import json
import math

from detection import size_filter as sf


def _photo(tmp_path, rel_alt=50.0, hfov=81.0, zoom=1.0):
    img = tmp_path / 'p.jpg'
    img.write_bytes(b'x')
    (tmp_path / 'p.json').write_text(json.dumps({'rel_alt_m': rel_alt, 'camera': {'hfov_deg': hfov, 'zoom_x': zoom}}))
    return str(img)


def test_gsd_from_height_and_field_of_view(tmp_path):
    gsd = sf.ground_sample_distance(_photo(tmp_path), 2560)
    assert abs(gsd - 2 * 50.0 * math.tan(math.radians(40.5)) / 2560) < 1e-9


def test_zoom_and_ground_height_change_the_scale(tmp_path):
    base = sf.ground_sample_distance(_photo(tmp_path), 2560)
    assert abs(sf.ground_sample_distance(_photo(tmp_path, zoom=2.0), 2560) - base / 2) < 1e-9
    assert sf.ground_sample_distance(_photo(tmp_path), 2560, ground_height_m=10.0) < base


def test_missing_sidecar_gives_none(tmp_path):
    assert sf.ground_sample_distance(str(tmp_path / 'nothing.jpg'), 2560) is None


def test_big_person_boxes_are_dropped_and_small_ones_kept():
    gsd = 0.033
    hiker = {'class': 'person', 'bbox': [0, 0, 34, 14]}             # 1.1 m
    tree = {'class': 'person', 'bbox': [0, 0, 240, 200]}            # 7.9 m
    tent = {'class': 'tent', 'bbox': [0, 0, 72, 66]}                # 2.4 m
    round_crown = {'class': 'tent', 'bbox': [0, 0, 190, 190]}       # 6.3 m
    other = {'class': 'object', 'bbox': [0, 0, 900, 900]}
    kept, dropped = sf.apply_size_filter([hiker, tree, tent, round_crown, other], gsd)
    assert kept == [hiker, tent, other] and dropped == [tree, round_crown]


def test_without_a_scale_nothing_is_dropped():
    dets = [{'class': 'person', 'bbox': [0, 0, 999, 999]}]
    assert sf.apply_size_filter(dets, None) == (dets, [])
