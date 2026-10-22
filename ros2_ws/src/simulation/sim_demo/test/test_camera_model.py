"""
sim_demo/test/test_camera_model.py -- PURPOSE: unit tests of the camera projection and
footprint math in camera_model.py: image orientation conventions, round trips pixel <-> ground,
footprint size against the textbook formula, zoom, and box projection/clipping.
"""
import math

import pytest

from sim_demo.camera_model import (CameraIntrinsics, CameraPose, box_corners_3d, camera_axes,
                                   footprint, ground_sample_distance, pixel_to_ground,
                                   point_in_polygon, polygon_area, project_box, project_point)

INTR = CameraIntrinsics(2560, 1440, 1.4137, 1.0)


def test_intrinsics_match_a8_mini_numbers():
    # ARCHITECTURE.md section 5: fx = (W/2)/tan(40.5 deg) = 749.3 px at 1280 wide.
    assert CameraIntrinsics(1280, 720).fx == pytest.approx(749.3, abs=0.3)
    assert CameraIntrinsics.from_profile('a8mini').fx == pytest.approx(2248.0, abs=1.0)


def test_zoom_narrows_field_of_view():
    z5 = CameraIntrinsics(2560, 1440, 1.4137, 5.0)
    assert z5.fx == pytest.approx(INTR.fx * 5.0)
    assert math.tan(z5.effective_hfov_rad / 2) == pytest.approx(math.tan(1.4137 / 2) / 5.0)


def test_nadir_axes_heading_north():
    x, y, z = camera_axes(CameraPose(0, 0, 50, yaw_deg=0.0, pitch_deg=-90.0))
    assert z == pytest.approx((0, 0, -1), abs=1e-9)      # looks down
    assert y == pytest.approx((0, -1, 0), abs=1e-9)      # image down = south (top = north)
    assert x == pytest.approx((1, 0, 0), abs=1e-9)       # image right = east


def test_nadir_axes_heading_east():
    x, y, _ = camera_axes(CameraPose(0, 0, 50, yaw_deg=90.0, pitch_deg=-90.0))
    assert x == pytest.approx((0, -1, 0), abs=1e-9)      # right of an eastbound drone = south
    assert y == pytest.approx((-1, 0, 0), abs=1e-9)      # image down = behind = west


def test_forward_looking_axes():
    x, y, z = camera_axes(CameraPose(0, 0, 50, yaw_deg=0.0, pitch_deg=0.0))
    assert z == pytest.approx((0, 1, 0), abs=1e-9)
    assert y == pytest.approx((0, 0, -1), abs=1e-9)
    assert x == pytest.approx((1, 0, 0), abs=1e-9)


def test_point_below_is_image_centre():
    pose = CameraPose(10, 20, 40, yaw_deg=37.0)
    u, v = project_point(INTR, pose, (10, 20, 0))
    assert (u, v) == pytest.approx((1280.0, 720.0), abs=1e-6)


def test_north_appears_at_top_when_heading_north():
    pose = CameraPose(0, 0, 40, yaw_deg=0.0)
    u, v = project_point(INTR, pose, (0, 5, 0))
    assert u == pytest.approx(1280.0, abs=1e-6)
    assert v < 720.0
    u, v = project_point(INTR, pose, (5, 0, 0))
    assert u > 1280.0                                   # east is right


def test_pixel_ground_round_trip():
    pose = CameraPose(5, -3, 45, yaw_deg=123.0, pitch_deg=-80.0, roll_deg=2.0)
    for u, v in ((100, 100), (2400, 300), (1280, 720), (50, 1400)):
        g = pixel_to_ground(INTR, pose, u, v)
        assert g is not None
        uu, vv = project_point(INTR, pose, (g[0], g[1], 0.0))
        assert (uu, vv) == pytest.approx((u, v), abs=1e-6)


def test_point_behind_camera_is_none():
    assert project_point(INTR, CameraPose(0, 0, 40), (0, 0, 80)) is None


def test_footprint_matches_formula():
    h = 40.0
    fp = footprint(INTR, CameraPose(0, 0, h, yaw_deg=0.0))
    width_m = 2 * h * math.tan(1.4137 / 2)
    height_m = width_m * 1440 / 2560
    xs = [p[0] for p in fp]
    ys = [p[1] for p in fp]
    assert max(xs) - min(xs) == pytest.approx(width_m, rel=1e-6)
    assert max(ys) - min(ys) == pytest.approx(height_m, rel=1e-6)
    assert polygon_area(fp) == pytest.approx(width_m * height_m, rel=1e-6)
    assert fp[0][1] > 0 > fp[2][1]       # top-left corner lies north (image top = heading)


def test_footprint_rotates_with_heading_and_shrinks_with_zoom():
    fp = footprint(INTR, CameraPose(0, 0, 40, yaw_deg=90.0))
    xs = [p[0] for p in fp]
    ys = [p[1] for p in fp]
    assert (max(ys) - min(ys)) > (max(xs) - min(xs))      # image width now along north-south
    fp5 = footprint(CameraIntrinsics(2560, 1440, 1.4137, 5.0), CameraPose(0, 0, 40))
    assert polygon_area(fp5) == pytest.approx(polygon_area(footprint(INTR, CameraPose(0, 0, 40))) / 25)


def test_oblique_footprint_above_horizon_is_none():
    assert footprint(INTR, CameraPose(0, 0, 40, pitch_deg=-10.0)) is None


def test_point_in_polygon():
    sq = [(0, 0), (10, 0), (10, 10), (0, 10)]
    assert point_in_polygon(5, 5, sq)
    assert not point_in_polygon(15, 5, sq)
    assert not point_in_polygon(-0.1, 5, sq)


def test_gsd():
    assert ground_sample_distance(INTR, 40.0) == pytest.approx(40.0 / INTR.fx)


def test_project_box_centred_tent():
    corners = box_corners_3d(0, 0, 0, 2.4, 2.1, 1.4, 0.0)
    pb = project_box(INTR, CameraPose(0, 0, 40), corners)
    assert pb is not None
    assert pb.visible_fraction == pytest.approx(1.0)
    assert pb.cx == pytest.approx(1280.0, abs=1.0)
    # 2.4 m wide at ~38.6-40 m: about 2.4 * fx / 40 px (top face is closer, so a bit more).
    assert 2.4 * INTR.fx / 40 <= pb.full_width <= 2.4 * INTR.fx / 38.6 + 1
    assert pb.center_norm < 0.01


def test_project_box_clipped_at_edge():
    fp = footprint(INTR, CameraPose(0, 0, 40))
    east_edge = max(p[0] for p in fp)
    corners = box_corners_3d(east_edge, 0, 0, 2.0, 2.0, 0.2, 0.0)
    pb = project_box(INTR, CameraPose(0, 0, 40), corners)
    assert pb is not None
    assert 0.3 < pb.visible_fraction < 0.7
    assert pb.x2 == pytest.approx(2560.0)


def test_project_box_outside_is_none():
    corners = box_corners_3d(500, 0, 0, 2.0, 2.0, 1.0, 0.0)
    assert project_box(INTR, CameraPose(0, 0, 40), corners) is None
