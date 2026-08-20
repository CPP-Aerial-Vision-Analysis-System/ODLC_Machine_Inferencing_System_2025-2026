"""Unit tests for ortho_mapping.gps_mosaic — the GPS-only fallback mapper.

Covers filename parsing, geodesy, calibration guards, and a full
synthetic mosaic run (both blend modes). Pure numpy/opencv, no ROS.
"""

import os
import sys

import cv2
import numpy as np

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__), "..", "..", "ros2_ws", "src", "ortho_mapping"
    ),
)

from ortho_mapping.gps_mosaic import (  # noqa: E402
    Calibration,
    _circular_median,
    _circular_std,
    altitude_calibration,
    build_mosaic,
    chain_calibrate,
    load_photos,
    meters_per_degree,
    order_by_gps,
    parse_latlon,
    world_file,
)

REF_LAT, REF_LON = 34.0419, -117.8146   # the real mappingImages locale


# --- filename parsing -------------------------------------------------------

def test_parse_real_style_name():
    assert parse_latlon("34.041850 , -117.814620.jpg") == (34.041850, -117.814620)


def test_parse_rejects_non_gps_names():
    assert parse_latlon("IMG_0079.jpg") is None
    assert parse_latlon("0.000000 , 0.000000.jpg") is None
    assert parse_latlon("34.0,-117.8.jpg") is None          # missing ' , '
    assert parse_latlon("200.0 , -117.8.jpg") is None        # out of range


# --- geodesy ----------------------------------------------------------------

def test_meters_per_degree_at_site():
    m_lat, m_lon = meters_per_degree(34.0419)
    assert abs(m_lat - 110923) < 200
    assert abs(m_lon - 92385) < 300     # cos(34) shrinkage


def test_altitude_calibration_gsd():
    calib = altitude_calibration(30.0, 81.0, 3840)
    # 2*30*tan(40.5) / 3840 ~= 1.33 cm/px
    assert abs(calib.gsd_m - 0.01333) < 0.0003
    assert calib.heading_deg == 0.0
    assert calib.method == "altitude"


# --- circular stats ---------------------------------------------------------

def test_circular_median_wraps():
    assert abs(_circular_median([179.0, -179.0, 180.0])) > 170   # near +/-180


def test_circular_std_agreement():
    assert _circular_std([10.0, 11.0, 9.0, 10.5]) < 2.0
    assert _circular_std([0.0, 90.0, 180.0, 270.0]) > 40.0       # scattered


# --- world file -------------------------------------------------------------

def test_world_file_north_up():
    lines = world_file(34.05, -117.81, 0.03, 34.05).strip().splitlines()
    assert len(lines) == 6
    assert float(lines[1]) == 0.0 and float(lines[2]) == 0.0
    assert float(lines[3]) < 0                              # north-up
    assert abs(float(lines[5]) - 34.05) < 1e-9


# --- synthetic mosaic -------------------------------------------------------

def _make_grid(tmp_path, rows=2, cols=3, step_m=15.0, gsd=0.02):
    """A north-up grid of overlapping fake nadir tiles named by GPS."""
    m_lat, m_lon = meters_per_degree(REF_LAT)
    w, h = 400, 300
    scene = np.random.RandomState(3).randint(
        40, 210, (2400, 2400, 3), dtype=np.uint8)
    base_x, base_y = 200, 800   # keep every crop inside the scene
    photos_dir = tmp_path / "imgs"
    photos_dir.mkdir()
    for r in range(rows):
        for c in range(cols):
            east = c * step_m
            north = r * step_m
            # crop a window of the shared scene so overlaps actually match
            cx = base_x + int(east / gsd)
            cy = base_y - int(north / gsd)
            tile = scene[cy:cy + h, cx:cx + w].copy()
            assert tile.size, f"crop out of bounds at r={r} c={c}"
            lat = REF_LAT + north / m_lat
            lon = REF_LON + east / m_lon
            cv2.imwrite(str(photos_dir / f"{lat:.6f} , {lon:.6f}.jpg"), tile)
    return photos_dir


def test_load_photos_reads_grid(tmp_path):
    photos_dir = _make_grid(tmp_path)
    photos = load_photos(str(photos_dir))
    assert len(photos) == 6
    # ENU centered near mean; easts span ~0..30 m
    easts = sorted(p.east for p in photos)
    assert easts[-1] - easts[0] > 25.0


def test_build_mosaic_center_mode(tmp_path):
    # 4 m spacing with ~8 m-wide tiles -> heavy overlap -> near-full coverage
    photos_dir = _make_grid(tmp_path, rows=3, cols=4, step_m=4.0)
    photos = load_photos(str(photos_dir))
    calib = Calibration(gsd_m=0.02, heading_deg=0.0, method="test")
    result = build_mosaic(photos, calib, out_gsd_m=0.04, blend="center",
                          log=lambda m: None)
    assert result is not None
    assert result["placed"] == 12
    assert result["coverage"] > 0.9
    # non-black content dominates
    assert (result["image"].sum(axis=2) > 30).mean() > 0.85


def test_build_mosaic_feather_mode(tmp_path):
    photos_dir = _make_grid(tmp_path, rows=3, cols=4, step_m=4.0)
    photos = load_photos(str(photos_dir))
    calib = Calibration(gsd_m=0.02, heading_deg=0.0, method="test")
    result = build_mosaic(photos, calib, out_gsd_m=0.04, blend="feather",
                          log=lambda m: None)
    assert result is not None and result["placed"] == 12


def _make_snake(tmp_path, rows=2, cols=4, step_m=5.0, gsd=0.02):
    """A lawnmower survey: odd rows are flown the other way, so their
    photos are rotated 180 degrees — the case the old translation-only
    auto-calibration could not handle."""
    m_lat, m_lon = meters_per_degree(REF_LAT)
    w, h = 400, 300
    rng = np.random.RandomState(7)
    scene = rng.randint(30, 220, (2400, 2400, 3), dtype=np.uint8)
    scene = cv2.GaussianBlur(scene, (3, 3), 0)   # give SIFT smooth gradients
    base_x, base_y = 200, 900
    photos_dir = tmp_path / "imgs"
    photos_dir.mkdir()
    for r in range(rows):
        for c in range(cols):
            east = c * step_m
            north = r * step_m
            cx = base_x + int(east / gsd)
            cy = base_y - int(north / gsd)
            tile = scene[cy:cy + h, cx:cx + w].copy()
            assert tile.size, f"crop out of bounds at r={r} c={c}"
            if r % 2 == 1:
                tile = np.rot90(tile, 2).copy()   # opposite flight direction
            lat = REF_LAT + north / m_lat
            lon = REF_LON + east / m_lon
            cv2.imwrite(str(photos_dir / f"{lat:.6f} , {lon:.6f}.png"), tile)
    return photos_dir


def test_order_by_gps_walks_the_line(tmp_path):
    photos_dir = _make_snake(tmp_path, rows=1, cols=5)
    photos = load_photos(str(photos_dir))
    order = order_by_gps(photos)
    easts = [photos[i].east for i in order]
    assert easts == sorted(easts) or easts == sorted(easts, reverse=True)


def test_chain_calibrate_recovers_scale_and_rotated_rows(tmp_path):
    photos_dir = _make_snake(tmp_path, gsd=0.02)
    photos = load_photos(str(photos_dir))
    calib = chain_calibrate(photos, log=lambda m: None)
    assert calib is not None
    assert calib.method == "snake"
    # true scale recovered from imagery + GPS, no altitude given
    assert abs(calib.gsd_m - 0.02) < 0.003
    m_lat, _ = meters_per_degree(REF_LAT)
    row2_lat = REF_LAT + 5.0 / m_lat
    for p in photos:
        assert p.heading_deg is not None
        rotated = abs(p.lat - row2_lat) < abs(p.lat - REF_LAT)
        if rotated:                       # flown the other way -> ~180 deg
            assert abs(abs(p.heading_deg) - 180.0) < 10.0
        else:
            assert abs(p.heading_deg) < 10.0


def test_chain_calibrate_refuses_unmatchable_images(tmp_path):
    """Featureless flat tiles: matching must fail and calibration must
    decline (returning None -> altitude fallback), never invent a scale."""
    m_lat, m_lon = meters_per_degree(REF_LAT)
    photos_dir = tmp_path / "flat"
    photos_dir.mkdir()
    for k in range(6):
        tile = np.full((300, 400, 3), 128, np.uint8)
        lat = REF_LAT + (5.0 * k) / m_lat
        cv2.imwrite(str(photos_dir / f"{lat:.6f} , {REF_LON:.6f}.png"), tile)
    photos = load_photos(str(photos_dir))
    assert chain_calibrate(photos, log=lambda m: None) is None
    assert all(p.heading_deg is None for p in photos)


def test_build_mosaic_autocoarsens(tmp_path):
    photos_dir = _make_grid(tmp_path, rows=3, cols=4, step_m=4.0)
    photos = load_photos(str(photos_dir))
    calib = Calibration(gsd_m=0.02, heading_deg=0.0, method="test")
    # Force a tiny MP cap -> output GSD must coarsen, not blow up
    result = build_mosaic(photos, calib, out_gsd_m=0.001, blend="center",
                          max_canvas_mp=0.5, log=lambda m: None)
    assert result is not None
    assert result["out_gsd_m"] > 0.001
    assert result["image"].shape[0] * result["image"].shape[1] <= 0.6e6
