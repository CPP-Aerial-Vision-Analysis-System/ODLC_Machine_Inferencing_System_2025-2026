"""
sim_demo/test/test_photo_info.py -- PURPOSE: unit tests of photo_info.py, which pairs each
saved photo file with the camera emulator's exact record (/sim/camera/photo_info JSON):
JSON parsing in the emulator's real format, camera axes from the optical quaternion, FIFO
matching, skipping a photo whose download failed, and the footprint the record implies.
"""
import json
import math
from datetime import datetime, timezone

import pytest

from sim_demo.camera_model import axes_from_quaternion, camera_axes, footprint
from sim_demo.geo import GeoOrigin
from sim_demo.photo_geometry import PhotoGeometry
from sim_demo.photo_info import PhotoInfoIndex, parse_photo_info
from sim_demo.photos import PhotoEvent
from sim_demo.zoom import ZoomTracker

HOME = GeoOrigin(36.2143, -96.0068, 210.0)
# Optical frame looking straight down with the image top towards north:
# x_right = east (1,0,0), y_down = south (0,-1,0), z_fwd = down (0,0,-1).
# That rotation matrix is a 180 deg turn about the east axis: quaternion (1, 0, 0, 0).
NADIR_NORTH_Q = [1.0, 0.0, 0.0, 0.0]


def _info(t, x, y, z=30.0, zoom=1.0, name='IMG_0001.jpg', q=NADIR_NORTH_Q):
    """A /sim/camera/photo_info message like siyi_emulator publishes."""
    return json.dumps({
        'emulator': 'siyi_emulator', 'camera_model': 'ZR10',
        'wall_time': datetime.fromtimestamp(t, timezone.utc).isoformat(timespec='milliseconds'),
        'sim_time': 12.5, 'image': {'width': 2560, 'height': 1440, 'source': 'gazebo'},
        'zoom': {'total': zoom, 'optical': 1.0, 'digital': zoom},
        'hfov_rad': {'lens': 1.4137, 'effective': 2 * math.atan(math.tan(1.4137 / 2) / zoom)},
        'gimbal': {'mode': 'follow', 'yaw_deg': 0.0, 'pitch_deg': -90.0, 'roll_deg': 0.0},
        'drone': {'position_enu_m': [x, y, z + 0.1], 'orientation_xyzw': [0, 0, 0, 1]},
        'camera_world': {'position_enu_m': [x, y, z], 'look_direction_enu': [0, 0, -1],
                         'optical_quat_xyzw': q, 'heading_compass_deg': 0.0, 'pitch_deg': -90.0},
        'sd_name': name, 'sd_path': f'/tmp/sd/{name}', 'size_bytes': 123456,
        'url': f'http://192.168.144.25:82/101SIYI_IMG/{name}'})


def test_quaternion_axes():
    x, y, z = axes_from_quaternion(*NADIR_NORTH_Q)
    assert x == pytest.approx((1, 0, 0)) and y == pytest.approx((0, -1, 0)) and z == pytest.approx((0, 0, -1))


def test_parse_real_format():
    p = parse_photo_info(_info(1000.0, 10.0, 20.0, zoom=5.0))
    assert p.t == pytest.approx(1000.0, abs=0.001) and p.sd_name == 'IMG_0001.jpg'
    assert (p.intr.width, p.intr.height, p.intr.zoom) == (2560, 1440, 5.0)
    assert (p.pose.east, p.pose.north, p.pose.up) == (10.0, 20.0, 30.0)
    assert camera_axes(p.pose)[2] == pytest.approx((0, 0, -1))
    fp = footprint(p.intr, p.pose)
    width = 2 * 30.0 * math.tan(1.4137 / 2) / 5.0
    assert max(q[0] for q in fp) - min(q[0] for q in fp) == pytest.approx(width, rel=1e-6)
    assert parse_photo_info('{"not": "a photo"}') is None
    assert parse_photo_info('garbage') is None


def test_fifo_matching_and_failed_download():
    idx = PhotoInfoIndex(HOME)
    idx.add_json(_info(100.0, 0.0, 50.0, name='IMG_0001.jpg'))
    idx.add_json(_info(101.0, 5.0, 50.0, name='IMG_0002.jpg'))      # this download fails
    idx.add_json(_info(102.0, 10.0, 50.0, name='IMG_0003.jpg'))
    lat, lon, _ = HOME.to_geo(0.8, 50.0)                              # GPS ~1 m from camera
    assert idx.match(101.5, lat, lon).sd_name == 'IMG_0001.jpg'
    lat, lon, _ = HOME.to_geo(10.5, 50.0)
    assert idx.match(103.0, lat, lon).sd_name == 'IMG_0003.jpg'     # IMG_0002 skipped (far)
    assert idx.match(104.0, lat, lon) is None                         # nothing left
    assert all(p.matched for p in idx.photos)


def test_name_time_rule_survives_a_late_shutter():
    """Seen in the real simulator: video_cam named the photo, then waited ~3 s for the camera
    lock before the shutter, so the camera was 16 m further on than the name's GPS."""
    idx = PhotoInfoIndex(HOME)
    idx.add_json(_info(98.0, -11.0, 50.0, name='IMG_0001.jpg'))     # earlier, download failed
    idx.add_json(_info(103.0, 16.5, 50.0, name='IMG_0002.jpg'))     # the real one (late shutter)
    lat, lon, _ = HOME.to_geo(0.0, 50.0)                              # name chosen at t = 100
    # The nearest-position rule alone would pick the wrong (failed) photo here:
    assert idx.match(104.0, lat, lon).sd_name == 'IMG_0001.jpg'
    idx2 = PhotoInfoIndex(HOME)
    idx2.add_json(_info(98.0, -11.0, 50.0, name='IMG_0001.jpg'))
    idx2.add_json(_info(103.0, 16.5, 50.0, name='IMG_0002.jpg'))
    rec = idx2.match(104.0, lat, lon, name_time=100.0)
    assert rec.sd_name == 'IMG_0002.jpg'
    assert all(p.matched for p in idx2.photos)                        # the failed one is dropped


def test_no_gps_in_name_takes_oldest_and_window():
    idx = PhotoInfoIndex(HOME, window_s=10.0)
    idx.add_json(_info(100.0, 0.0, 0.0, name='A'))
    assert idx.match(200.0, None, None) is None                       # too old
    idx.add_json(_info(195.0, 0.0, 0.0, name='B'))
    assert idx.match(200.0, None, None).sd_name == 'B'


class _NoFlight:
    zoom = ZoomTracker()

    def capture_pose(self, *a, **k):
        return None, 'none'


def test_photo_geometry_prefers_emulator_record(tmp_path):
    idx = PhotoInfoIndex(HOME)
    idx.add_json(_info(100.0, 52.0, 92.0, zoom=2.0))
    geo = PhotoGeometry(HOME, _NoFlight(), photo_info=idx)
    lat, lon, _ = HOME.to_geo(52.5, 92.3)
    sol = geo.solve(PhotoEvent(str(tmp_path / 'x.jpg'), 'x.jpg', 101.0, lat, lon, 1000))
    assert sol.how == 'emulator' and sol.intr.zoom == 2.0 and sol.footprint is not None
    sol2 = geo.solve(PhotoEvent(str(tmp_path / 'y.jpg'), 'y.jpg', 102.0, lat, lon, 1000))
    assert sol2.how == 'none' and sol2.pose is None                  # record already used
