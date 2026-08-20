"""Unit tests for ortho_mapping.geo_math — the georeferencing core.

Pure numpy math: no ROS, no camera, no disk. Run with `pytest tests/unit`.
"""

import math
import os
import sys

import numpy as np

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__), "..", "..", "ros2_ws", "src", "ortho_mapping"
    ),
)

from ortho_mapping.geo_math import (  # noqa: E402
    bearing_deg,
    distance_m,
    enu_to_latlon,
    ground_footprint,
    latlon_to_enu,
    meters_per_degree,
    nadir_gsd_m,
    rotation_enu_from_camera,
)

# SIYI A8 mini still: 3840x2160, 81 deg HFOV
W, H, HFOV = 3840, 2160, 81.0


def test_meters_per_degree_sanity():
    m_lat, m_lon = meters_per_degree(0.0)      # equator
    assert abs(m_lat - 110574) < 100
    assert abs(m_lon - 111320) < 100
    m_lat45, m_lon45 = meters_per_degree(45.0)
    assert abs(m_lon45 - 78847) < 200          # cos(45) shrinkage
    assert m_lat45 > m_lat                     # ellipsoid: degree-lat grows poleward


def test_enu_roundtrip():
    ref_lat, ref_lon = 38.315386, -76.550875   # Maryland-ish (SUAS latitudes)
    east, north = latlon_to_enu(38.316000, -76.549000, ref_lat, ref_lon)
    lat, lon = enu_to_latlon(east, north, ref_lat, ref_lon)
    assert abs(lat - 38.316000) < 1e-9
    assert abs(lon - -76.549000) < 1e-9
    assert north > 0 and east > 0              # NE of the reference


def test_bearing_cardinal_directions():
    assert abs(bearing_deg(38.0, -76.0, 38.001, -76.0) - 0.0) < 0.5      # north
    assert abs(bearing_deg(38.0, -76.0, 38.0, -75.999) - 90.0) < 0.5    # east
    assert abs(bearing_deg(38.0, -76.0, 37.999, -76.0) - 180.0) < 0.5   # south
    assert abs(bearing_deg(38.0, -76.0, 38.0, -76.001) - 270.0) < 0.5   # west


def test_distance_m():
    # 0.001 deg latitude is ~111 m everywhere
    d = distance_m(38.0, -76.0, 38.001, -76.0)
    assert abs(d - 111.0) < 1.0


def test_rotation_nadir_yaw0():
    """Nadir, heading north: image right -> east, image down -> south,
    optical axis -> straight down."""
    rot = rotation_enu_from_camera(yaw_deg=0.0, pitch_deg=-90.0, roll_deg=0.0)
    cam_x, cam_y, cam_z = rot[:, 0], rot[:, 1], rot[:, 2]
    assert np.allclose(cam_x, [1, 0, 0], atol=1e-9)    # image right = east
    assert np.allclose(cam_y, [0, -1, 0], atol=1e-9)   # image down = south
    assert np.allclose(cam_z, [0, 0, -1], atol=1e-9)   # optical axis = down


def test_rotation_nadir_yaw90():
    """Nadir, heading east: image top must point east."""
    rot = rotation_enu_from_camera(yaw_deg=90.0, pitch_deg=-90.0, roll_deg=0.0)
    image_up_world = rot @ np.array([0.0, -1.0, 0.0])  # -y cam = image up
    assert np.allclose(image_up_world, [1, 0, 0], atol=1e-9)


def test_rotation_level_forward():
    """Level camera, heading north: optical axis points north."""
    rot = rotation_enu_from_camera(yaw_deg=0.0, pitch_deg=0.0, roll_deg=0.0)
    assert np.allclose(rot[:, 2], [0, 1, 0], atol=1e-9)


def test_footprint_nadir_dimensions():
    alt = 30.0
    fp = ground_footprint(0.0, 0.0, alt, 0.0, -90.0, 0.0, W, H, HFOV)
    assert fp is not None
    east_span = fp[:, 0].max() - fp[:, 0].min()
    north_span = fp[:, 1].max() - fp[:, 1].min()
    expected_east = 2 * alt * math.tan(math.radians(HFOV / 2))     # ~51.2 m
    # fy = fx, so the vertical half-angle scales with (H-1)/(W) pixel ratio
    assert abs(east_span - expected_east) < 0.2
    assert 0.5 * expected_east < north_span < expected_east        # 16:9 aspect
    # Centred on the aircraft
    assert abs(fp[:, 0].mean()) < 1e-6
    assert abs(fp[:, 1].mean()) < 1e-6


def test_footprint_corner_order_nadir_north():
    """Image TL corner must land north-west of the aircraft at heading 0."""
    fp = ground_footprint(0.0, 0.0, 30.0, 0.0, -90.0, 0.0, W, H, HFOV)
    tl, tr, br, bl = fp
    assert tl[0] < 0 and tl[1] > 0     # west, north
    assert tr[0] > 0 and tr[1] > 0     # east, north
    assert br[0] > 0 and br[1] < 0     # east, south
    assert bl[0] < 0 and bl[1] < 0     # west, south


def test_footprint_yaw_rotates_footprint():
    """Footprint at heading 90 == footprint at heading 0 rotated 90 deg CW."""
    fp0 = ground_footprint(0.0, 0.0, 30.0, 0.0, -90.0, 0.0, W, H, HFOV)
    fp90 = ground_footprint(0.0, 0.0, 30.0, 90.0, -90.0, 0.0, W, H, HFOV)
    # CW compass rotation by 90: (e, n) -> (n, -e)
    rotated = np.stack([fp0[:, 1], -fp0[:, 0]], axis=1)
    assert np.allclose(fp90, rotated, atol=1e-6)


def test_footprint_pitch_tilts_forward():
    """Pitching up from nadir (toward the horizon) at heading 0 pushes the
    footprint's top edge (image top = forward) further north."""
    fp_nadir = ground_footprint(0.0, 0.0, 30.0, 0.0, -90.0, 0.0, W, H, HFOV)
    fp_tilt = ground_footprint(0.0, 0.0, 30.0, 0.0, -60.0, 0.0, W, H, HFOV)
    assert fp_tilt is not None
    assert fp_tilt[0][1] > fp_nadir[0][1]      # TL corner further north
    assert fp_tilt[1][1] > fp_nadir[1][1]      # TR corner further north


def test_footprint_rejects_level_camera():
    """A level camera never intersects the ground plane usefully."""
    assert ground_footprint(0.0, 0.0, 30.0, 0.0, 0.0, 0.0, W, H, HFOV) is None


def test_footprint_rejects_extreme_oblique():
    """Barely-below-horizon views blow past the span guard."""
    assert ground_footprint(0.0, 0.0, 30.0, 0.0, -15.0, 0.0, W, H, HFOV) is None


def test_footprint_rejects_zero_altitude():
    assert ground_footprint(0.0, 0.0, 0.0, 0.0, -90.0, 0.0, W, H, HFOV) is None


def test_nadir_gsd():
    # 30 m AGL, 3840 px across 51.2 m -> ~1.33 cm/px
    gsd = nadir_gsd_m(30.0, W, HFOV)
    assert abs(gsd - 0.01334) < 0.0005
