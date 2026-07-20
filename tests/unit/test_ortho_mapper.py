"""End-to-end tests for ortho_mapping.mapper on synthetic missions.

Builds small fake photo sets (with GPS filenames / sidecars), runs the full
batch pipeline, and checks the output map, world file, and report.
"""

import json
import math
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

from ortho_mapping.canvas import MosaicCanvas, world_file_content  # noqa: E402
from ortho_mapping.config import MappingConfig  # noqa: E402
from ortho_mapping.geo_math import ground_footprint, meters_per_degree  # noqa: E402
from ortho_mapping.mapper import build_map  # noqa: E402

REF_LAT, REF_LON = 38.315, -76.550
IMG_W, IMG_H, HFOV = 320, 240, 81.0
ALT = 30.0   # -> footprint ~51 x 38 m


def _textured_image(seed: int) -> np.ndarray:
    """Random-ish but structured texture so ECC has something to lock onto."""
    rng = np.random.RandomState(seed)
    img = rng.randint(40, 200, (IMG_H, IMG_W, 3), dtype=np.uint8)
    img = cv2.GaussianBlur(img, (7, 7), 0)
    cv2.circle(img, (IMG_W // 2, IMG_H // 2), 30, (255, 255, 255), -1)
    return img


def _make_mission(tmp_path, grid=(2, 3), spacing_m=(15.0, 20.0),
                  with_sidecars=True, heading=0.0):
    """Write a lawnmower-ish grid of geotagged photos; returns the dir."""
    m_lat, m_lon = meters_per_degree(REF_LAT)
    d_lat = spacing_m[0] / m_lat
    d_lon = spacing_m[1] / m_lon
    photos_dir = tmp_path / "mapping_photos"
    photos_dir.mkdir()
    idx = 0
    for row in range(grid[0]):
        for col in range(grid[1]):
            lat = REF_LAT + row * d_lat
            lon = REF_LON + col * d_lon
            name = f"{lat:.6f} , {lon:.6f}.jpg"
            assert cv2.imwrite(str(photos_dir / name), _textured_image(idx))
            if with_sidecars:
                sidecar = {
                    "version": 1,
                    "timestamp_unix": 1000.0 + idx,
                    "latitude": lat,
                    "longitude": lon,
                    "rel_alt_m": ALT,
                    "compass_hdg_deg": heading,
                    "gimbal": {"yaw_deg": 0.0, "pitch_deg": -90.0,
                               "roll_deg": 0.0},
                    "resolution": "4K",
                    "rotate_180_applied": True,
                }
                with open(str(photos_dir / name.replace(".jpg", ".json")),
                          "w", encoding="utf-8") as fh:
                    json.dump(sidecar, fh)
            idx += 1
    return photos_dir


def _base_config(photos_dir, tmp_path, **overrides) -> MappingConfig:
    defaults = dict(
        images_dir=str(photos_dir),
        output_dir=str(tmp_path / "out"),
        output_basename="testmap",
        gsd_m=0.25,          # coarse -> tiny, fast canvases
        hfov_deg=HFOV,
        default_altitude_agl_m=ALT,
        refine=False,        # deterministic geometry unless a test opts in
    )
    defaults.update(overrides)
    return MappingConfig(**defaults)


def test_full_pipeline_with_sidecars(tmp_path):
    photos_dir = _make_mission(tmp_path)
    result = build_map(_base_config(photos_dir, tmp_path), log=lambda m: None)

    assert result.ok, result.message
    assert result.used_images == 6
    assert os.path.isfile(result.image_path)
    assert os.path.isfile(result.world_file_path)
    assert os.path.isfile(result.report_path)

    # Overlapping grid -> the painted bounding box should be nearly solid
    assert result.coverage > 0.9

    # Canvas must cover footprint extent: ~51+40 m east, ~38+15 m north
    img = cv2.imread(result.image_path)
    h, w = img.shape[:2]
    assert abs(w * 0.25 - (51.2 + 40.0)) < 3.0
    assert abs(h * 0.25 - (38.3 + 15.0)) < 3.0

    # Mosaic should be mostly non-black
    assert (img.sum(axis=2) > 30).mean() > 0.9


def test_world_file_georeference(tmp_path):
    photos_dir = _make_mission(tmp_path)
    result = build_map(_base_config(photos_dir, tmp_path), log=lambda m: None)
    assert result.ok

    lines = open(result.world_file_path).read().strip().splitlines()
    assert len(lines) == 6
    x_scale, rot1, rot2, y_scale, tl_lon, tl_lat = map(float, lines)
    m_lat, m_lon = meters_per_degree(REF_LAT)
    assert abs(x_scale * m_lon - 0.25) < 0.001    # 25 cm/px east
    assert abs(y_scale * m_lat + 0.25) < 0.001    # north-up (negative)
    assert rot1 == 0.0 and rot2 == 0.0
    # Top-left pixel must sit NW of the whole photo grid
    assert tl_lat > REF_LAT
    assert tl_lon < REF_LON

    report = json.load(open(result.report_path))
    assert report["images_used"] == 6
    assert len(report["per_image"]) == 6


def test_pipeline_filename_fallback_no_sidecars(tmp_path):
    """Old-style folders (no .json) must still map via filename GPS +
    track-derived headings + the default altitude."""
    photos_dir = _make_mission(tmp_path, with_sidecars=False)
    result = build_map(_base_config(photos_dir, tmp_path), log=lambda m: None)
    assert result.ok, result.message
    assert result.used_images == 6

    report = json.load(open(result.report_path))
    assert all(item["heading_source"] in ("track", "fixed")
               for item in report["per_image"])


def test_pipeline_with_refinement_enabled(tmp_path):
    """ECC refinement path must run without breaking the build."""
    photos_dir = _make_mission(tmp_path)
    result = build_map(
        _base_config(photos_dir, tmp_path, refine=True), log=lambda m: None)
    assert result.ok, result.message
    assert result.used_images == 6
    # bounded: no refinement shift may exceed the 3 m GPS bound
    report = json.load(open(result.report_path))
    for item in report["per_image"]:
        assert item["shift_px"] <= 3.0 / 0.25 + 1e-6


def test_pipeline_skips_junk_and_maps_rest(tmp_path):
    photos_dir = _make_mission(tmp_path, grid=(1, 2))
    (photos_dir / "IMG_0001.jpg").write_bytes(b"\xff\xd8garbage")
    (photos_dir / "notes.txt").write_text("not an image")
    result = build_map(_base_config(photos_dir, tmp_path), log=lambda m: None)
    assert result.ok
    assert result.used_images == 2
    assert any("IMG_0001" in s["file"] for s in result.skipped)


def test_pipeline_empty_dir_fails_cleanly(tmp_path):
    photos_dir = tmp_path / "empty"
    photos_dir.mkdir()
    result = build_map(_base_config(photos_dir, tmp_path), log=lambda m: None)
    assert not result.ok
    assert "no usable" in result.message


def test_pipeline_missing_dir_fails_cleanly(tmp_path):
    cfg = _base_config(tmp_path / "does_not_exist", tmp_path)
    result = build_map(cfg, log=lambda m: None)
    assert not result.ok


def test_canvas_gsd_autocoarsen(tmp_path):
    """A max_pixels cap far below the requested GSD must coarsen, not fail."""
    canvas = MosaicCanvas(0, 100, 0, 100, gsd_m=0.01, max_pixels=1e4,
                          logger=lambda m: None)
    assert canvas.width * canvas.height <= 1.2e4
    assert canvas.gsd_m > 0.01


def test_canvas_single_image_geometry():
    """One nadir image painted at a known spot lands in the right pixels."""
    canvas = MosaicCanvas(-40, 40, -40, 40, gsd_m=0.25, max_pixels=1e7,
                          logger=lambda m: None)
    fp = ground_footprint(0.0, 0.0, ALT, 0.0, -90.0, 0.0, IMG_W, IMG_H, HFOV)
    status = canvas.paste(np.full((IMG_H, IMG_W, 3), 200, np.uint8), fp)
    assert status["painted"]

    painted = canvas.weight > 0
    ys, xs = np.nonzero(painted)
    width_m = (xs.max() - xs.min()) * 0.25
    height_m = (ys.max() - ys.min()) * 0.25
    assert abs(width_m - 2 * ALT * math.tan(math.radians(HFOV / 2))) < 1.0
    assert abs(height_m - 38.3) < 1.5
    # centred on the canvas centre (aircraft at ENU origin)
    assert abs((xs.max() + xs.min()) / 2 - canvas.width / 2) < 4
    assert abs((ys.max() + ys.min()) / 2 - canvas.height / 2) < 4


def test_world_file_content_format():
    text = world_file_content(38.3, -76.5, 0.05, 38.3)
    lines = text.strip().splitlines()
    assert len(lines) == 6
    assert float(lines[1]) == 0.0 and float(lines[2]) == 0.0
    assert float(lines[3]) < 0    # north-up
    assert abs(float(lines[5]) - 38.3) < 1e-9
