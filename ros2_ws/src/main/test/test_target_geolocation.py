"""
Unit tests for main/src/target_geolocation.py (the drop-aiming math).

test_target_geolocation.py -- PURPOSE: unit tests for the drop aiming added in October 2026
(project a detection box centre onto the ground instead of dropping at the photo's own position).
Checks the geometry with hand-computable cases (centre pixel, image edges, every heading, a turned
photo, a tilted camera, the sky), the ArduPilot latitude/longitude formula, the JPEG size reader,
the sidecar contract with video_cam's writer, every fallback, and -- as a regression test -- the
best hiker and tent detections of three recorded simulator runs, which the old code missed by
14-25 m and which must now land within 2 m of the ground truth.

Needs nothing but Python (no ROS, no build):
    python3 -m pytest ros2_ws/src/main/test/test_target_geolocation.py -v
"""

import json
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'video_cam'))   # sidecar writer (contract test)

import target_geolocation as geo  # noqa: E402

W, H = 2560, 1440                 # ZR10 "2K" photo, the simulator default
HFOV = 81.0
F = (W / 2) / math.tan(math.radians(HFOV / 2))     # 1498.7 px


def close(a, b, tol=1e-6):
    return abs(a - b) <= tol


# ------------------------------------------------------------------ geometry
def test_centre_pixel_is_straight_below():
    e, n = geo.pixel_to_ground_offset(W / 2, H / 2, W, H, HFOV, 30.0, camera_heading_deg=123.0)
    assert close(e, 0.0) and close(n, 0.0)


def test_top_edge_is_ahead_when_facing_north():
    # Top-centre pixel: half the vertical footprint ahead (north). 720 px * 30 m / f.
    e, n = geo.pixel_to_ground_offset(W / 2, 0, W, H, HFOV, 30.0, camera_heading_deg=0.0)
    assert close(e, 0.0) and close(n, 720 * 30.0 / F)


def test_right_edge_is_half_the_footprint_width():
    # Footprint width = 2 * 30 m * tan(40.5 deg) = 51.2 m: the right edge is 25.6 m to the right.
    e, n = geo.pixel_to_ground_offset(W, H / 2, W, H, HFOV, 30.0, camera_heading_deg=0.0)
    assert close(e, 30.0 * math.tan(math.radians(40.5)), 1e-6) and close(n, 0.0)
    assert close(2 * e, 51.2, 0.05)


@pytest.mark.parametrize('heading, expect', [
    (0.0, (1.0, 0.0)),       # facing north: photo right = east
    (90.0, (0.0, -1.0)),     # facing east: photo right = south
    (180.0, (-1.0, 0.0)),    # facing south: photo right = west
    (270.0, (0.0, 1.0)),     # facing west: photo right = north
])
def test_photo_right_turns_with_the_heading(heading, expect):
    du = 300.0
    e, n = geo.pixel_to_ground_offset(W / 2 + du, H / 2, W, H, HFOV, 30.0, heading)
    d = du * 30.0 / F
    assert close(e, expect[0] * d, 1e-6) and close(n, expect[1] * d, 1e-6)


def test_photo_turned_180_flips_the_answer():
    a = geo.pixel_to_ground_offset(1000, 300, W, H, HFOV, 30.0, 45.0, image_top_direction_deg=0.0)
    b = geo.pixel_to_ground_offset(W - 1000, H - 300, W, H, HFOV, 30.0, 45.0,
                                   image_top_direction_deg=180.0)
    assert close(a[0], b[0], 1e-6) and close(a[1], b[1], 1e-6)


def test_photo_turned_90_top_points_to_the_right_side():
    # Saved photo turned so its top points to the drone's right (east when facing north). A point
    # at the top of THAT photo is east of the drone. The natural picture is 2560 wide, so the
    # saved one is 1440 x 2560 and the focal length stays the same.
    e, n = geo.pixel_to_ground_offset(H / 2, 0, H, W, HFOV, 30.0, 0.0,
                                      image_top_direction_deg=90.0)
    assert close(e, (W / 2) * 30.0 / F, 1e-6) and close(n, 0.0, 1e-6)


def test_zoom_shrinks_the_offset():
    one = geo.pixel_to_ground_offset(W, H / 2, W, H, HFOV, 30.0, 0.0, zoom=1.0)
    five = geo.pixel_to_ground_offset(W, H / 2, W, H, HFOV, 30.0, 0.0, zoom=5.0)
    assert close(five[0], one[0] / 5.0, 1e-6)


def test_tilted_camera_centre_is_ahead():
    # Pitch -60 (30 deg forward of straight down): the centre ray meets the ground 30/tan(60) ahead.
    e, n = geo.pixel_to_ground_offset(W / 2, H / 2, W, H, HFOV, 30.0, 0.0, camera_pitch_deg=-60.0)
    assert close(e, 0.0, 1e-6) and close(n, 30.0 / math.tan(math.radians(60.0)), 1e-6)


def test_sky_and_impossible_inputs_give_none():
    horizon = geo.pixel_to_ground_offset(W / 2, 0, W, H, HFOV, 30.0, 0.0, camera_pitch_deg=0.0)
    assert horizon is None
    assert geo.pixel_to_ground_offset(W / 2, H / 2, W, H, HFOV, 0.0, 0.0) is None
    assert geo.pixel_to_ground_offset(W / 2, H / 2, 0, H, HFOV, 30.0, 0.0) is None
    assert geo.pixel_to_ground_offset(W / 2, H / 2, W, H, 0.0, 30.0, 0.0) is None
    # near the horizon: farther than the allowed range
    assert geo.pixel_to_ground_offset(W / 2, 0, W, H, HFOV, 30.0, 0.0, camera_pitch_deg=-30.0,
                                      max_range_m=100.0) is None


# ------------------------------------------------------------------ latitude / longitude
def test_offset_round_trip_and_scale():
    lat0, lon0 = 36.2143, -96.0068
    lat, lon = geo.offset_to_latlon(lat0, lon0, 30.0, -40.0)
    e, n = geo.latlon_to_offset(lat0, lon0, lat, lon)
    assert close(e, 30.0, 1e-6) and close(n, -40.0, 1e-6)
    # 1e-7 degree of latitude = 0.011131884502145034 m (ArduPilot LOCATION_SCALING_FACTOR)
    lat1, _ = geo.offset_to_latlon(lat0, lon0, 0.0, 0.011131884502145034)
    assert close((lat1 - lat0) * 1e7, 1.0, 1e-6)


def test_compass_names():
    assert geo.compass_name(0, 10) == 'north'
    assert geo.compass_name(-10, -10) == 'south-west'
    assert geo.compass_name(10, 0) == 'east'


# ------------------------------------------------------------------ files
JPEG_2560x1440 = bytes.fromhex(
    'ffd8'                                             # start of image
    'ffe0 0010 4a46494600 0101 00 0001 0001 0000'      # APP0 (JFIF), 16 bytes
    'ffc0 0011 08 05a0 0a00 03 012200 021101 031101'   # SOF0: 8 bit, height 1440, width 2560
    'ffd9'.replace(' ', ''))


def test_jpeg_size_from_the_header(tmp_path):
    p = tmp_path / 'a.jpg'
    p.write_bytes(JPEG_2560x1440)
    assert geo.jpeg_size(str(p)) == (2560, 1440)


def test_jpeg_size_of_a_real_encoder_file(tmp_path):
    cv2 = pytest.importorskip('cv2')
    np = pytest.importorskip('numpy')
    p = tmp_path / 'b.jpg'
    cv2.imwrite(str(p), np.zeros((90, 160, 3), dtype=np.uint8))
    assert geo.jpeg_size(str(p)) == (160, 90)


def test_jpeg_size_rejects_other_files(tmp_path):
    p = tmp_path / 'c.jpg'
    p.write_bytes(b'not a jpeg at all')
    assert geo.jpeg_size(str(p)) is None
    assert geo.jpeg_size(str(tmp_path / 'missing.jpg')) is None


def test_sidecar_contract_with_video_cam_writer(tmp_path):
    photo_metadata = pytest.importorskip('video_cam.photo_metadata')
    photo = tmp_path / '36.215153 , -96.006786.jpg'
    meta = photo_metadata.build_photo_metadata(
        image_name=photo.name, shutter_unix_s=1.0, lat=36.2151531, lon=-96.0067861,
        rel_alt_m=30.0, heading_deg=270.0, camera_model='ZR10', hfov_deg=81.0, zoom_x=1.0,
        gimbal={'yaw_deg': 0.0, 'pitch_deg': -90.0, 'roll_deg': 0.0})
    assert photo_metadata.write_sidecar(photo_metadata.sidecar_path_for(str(photo)), meta)
    assert geo.sidecar_path_for(str(photo)) == photo_metadata.sidecar_path_for(str(photo))
    back = geo.read_photo_sidecar(str(photo))
    assert back is not None and back['heading_deg'] == 270.0
    assert geo.photo_position(back, None, None) == (36.2151531, -96.0067861)


def test_read_sidecar_refuses_other_formats(tmp_path):
    photo = tmp_path / 'x.jpg'
    (tmp_path / 'x.json').write_text(json.dumps({'format': 'other'}))
    assert geo.read_photo_sidecar(str(photo)) is None
    assert geo.read_photo_sidecar(str(tmp_path / 'none.jpg')) is None


def test_photo_position_prefers_the_sidecar_and_refuses_null_island():
    meta = {'position': {'lat': 1.5, 'lon': 2.5}}
    assert geo.photo_position(meta, 9.0, 9.0) == (1.5, 2.5)
    assert geo.photo_position(None, 9.0, 8.0) == (9.0, 8.0)
    assert geo.photo_position({'position': {'lat': None}}, 9.0, 8.0) == (9.0, 8.0)
    assert geo.photo_position(None, 0.0, 0.0) is None
    assert geo.photo_position(None, None, None) is None


# ------------------------------------------------------------------ aim_at_box and its fallbacks
def sidecar(**over):
    m = {'format': geo.SIDECAR_FORMAT, 'rel_alt_m': 30.0, 'heading_deg': 0.0,
         'camera': {'hfov_deg': 81.0, 'zoom_x': 1.0, 'zoom_changing': False,
                    'image_top_direction_deg': 0.0},
         'gimbal': {'yaw_deg': 0.0, 'pitch_deg': -90.0, 'roll_deg': 0.0}}
    m.update(over)
    return m


def test_aim_projects_the_box():
    r = geo.aim_at_box(sidecar(), 36.0, -96.0, W, H / 2, (W, H))
    assert r.method == 'box' and close(r.east_m, 25.62, 0.01) and close(r.north_m, 0.0, 1e-6)
    lat, lon = geo.offset_to_latlon(36.0, -96.0, r.east_m, r.north_m)
    assert close(r.lat, lat) and close(r.lon, lon)
    assert 'gimbal reported' in r.note


@pytest.mark.parametrize('meta, size, why', [
    (None, (W, H), 'no sidecar'),
    (sidecar(), None, 'size unknown'),
    (sidecar(rel_alt_m=None), (W, H), 'no height'),
    (sidecar(heading_deg=None), (W, H), 'no heading'),
    (sidecar(rel_alt_m=1.0), (W, H), 'above the ground'),
    (sidecar(camera={'zoom_changing': True}), (W, H), 'zoom was moving'),
])
def test_aim_falls_back_to_the_photo_position(meta, size, why):
    r = geo.aim_at_box(meta, 36.0, -96.0, 100.0, 100.0, size)
    assert r.method == 'photo' and (r.lat, r.lon) == (36.0, -96.0) and why in r.note


def test_untrusted_gimbal_pitch_uses_the_default():
    # A camera reporting pitch 0 (looking at the horizon) during a survey is not believed.
    r = geo.aim_at_box(sidecar(gimbal={'pitch_deg': 0.0, 'yaw_deg': 0.0}), 36.0, -96.0,
                       W / 2, H / 2, (W, H), default_pitch_deg=-90.0)
    assert r.method == 'box' and close(r.offset_m, 0.0, 1e-6) and 'not trusted' in r.note


def test_gimbal_yaw_left_turns_the_picture():
    # Drone faces north, camera turned 90 deg LEFT (SIYI yaw +90): the camera faces west, so the
    # photo's top is west.
    r = geo.aim_at_box(sidecar(gimbal={'pitch_deg': -90.0, 'yaw_deg': 90.0}), 36.0, -96.0,
                       W / 2, 0, (W, H))
    assert close(r.north_m, 0.0, 1e-6) and r.east_m < 0


def test_ground_height_reduces_the_height():
    r1 = geo.aim_at_box(sidecar(), 36.0, -96.0, W, H / 2, (W, H), ground_height_m=0.0)
    r2 = geo.aim_at_box(sidecar(), 36.0, -96.0, W, H / 2, (W, H), ground_height_m=10.0)
    assert close(r2.east_m, r1.east_m * 20.0 / 30.0, 1e-6)


def test_aim_beyond_the_limit_falls_back():
    r = geo.aim_at_box(sidecar(gimbal={'pitch_deg': -35.0, 'yaw_deg': 0.0}), 36.0, -96.0,
                       W / 2, 0, (W, H), max_offset_m=50.0)
    assert r.method == 'photo' and 'does not project' in r.note


# ------------------------------------------------------------------ recorded simulator runs
# The world "runway": ground truth of sim_worlds/truth/runway.yaml (lat/lon in ArduPilot's
# convention). Each case: the best detection main_controller chose in a run recorded on
# 2026-10-16 BEFORE the fix (photo name = drone GPS at the shutter, box centre in pixels,
# heading of the photo's top from the camera emulator's footprint), and the old code's error.
TRUTH = {'tent': (36.21502595, -96.00680007), 'person': (36.21513243, -96.00621469)}
RUNS = [
    # run 20:14 (tent 14.1 m, hiker 24.4 m before)
    ('tent', '36.215153 , -96.006792.jpg', 572.0, 706.0, 269.9, 14.1),
    ('person', '36.214934 , -96.006332.jpg', 174.5, 226.0, 89.9, 24.4),
    # run 20:31 (tent 14.1 m, hiker 25.3 m)
    ('tent', '36.215153 , -96.006800.jpg', 572.0, 778.0, 270.0, 14.1),
    ('person', '36.214934 , -96.006354.jpg', 175.0, 104.0, 89.9, 25.3),
    # run 20:45, the live desktop demo (tent 14.2 m, hiker 24.4 m)
    ('tent', '36.215153 , -96.006786.jpg', 572.0, 702.0, 270.0, 14.2),
    ('person', '36.214934 , -96.006093.jpg', 174.0, 1325.5, 90.0, 24.4),
]


@pytest.mark.parametrize('cls, name, u, v, heading, old_error', RUNS)
def test_recorded_runs_now_land_within_2_m(cls, name, u, v, heading, old_error):
    stem = os.path.splitext(name)[0]
    lat, lon = (float(x) for x in stem.split(' , '))
    meta = sidecar(heading_deg=heading)          # 30 m survey height, camera straight down
    r = geo.aim_at_box(meta, lat, lon, u, v, (W, H))
    assert r.method == 'box'
    e_old, n_old = geo.latlon_to_offset(TRUTH[cls][0], TRUTH[cls][1], lat, lon)
    e_new, n_new = geo.latlon_to_offset(TRUTH[cls][0], TRUTH[cls][1], r.lat, r.lon)
    assert abs(math.hypot(e_old, n_old) - old_error) < 0.3     # the old aim point, as recorded
    assert math.hypot(e_new, n_new) < 2.0                       # the new one: within 2 m
