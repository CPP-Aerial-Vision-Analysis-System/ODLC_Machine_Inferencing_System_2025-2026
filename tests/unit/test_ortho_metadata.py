"""Unit tests for ortho_mapping.metadata — pose collection and fallbacks."""

import json
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

from ortho_mapping.metadata import (  # noqa: E402
    assign_headings,
    collect_poses,
    parse_latlon_from_name,
    read_image_size,
)


# ---------------------------------------------------------------------------
# Filename parsing (the "<lat> , <lon>.jpg" video_cam convention)
# ---------------------------------------------------------------------------

def test_parse_valid_gps_filename():
    assert parse_latlon_from_name("38.315386 , -76.550875.jpg") == (
        38.315386, -76.550875)


def test_parse_rejects_null_island():
    assert parse_latlon_from_name("0.000000 , 0.000000.jpg") is None


def test_parse_rejects_sd_card_names():
    assert parse_latlon_from_name("IMG_0079.jpg") is None
    assert parse_latlon_from_name("sim_20260714-120000.jpg") is None


def test_parse_rejects_out_of_range():
    assert parse_latlon_from_name("138.0 , -76.0.jpg") is None


def test_parse_requires_space_comma_space():
    # plain comma is NOT the convention new_od parses either
    assert parse_latlon_from_name("38.315386,-76.550875.jpg") is None


# ---------------------------------------------------------------------------
# Header-only image size probing
# ---------------------------------------------------------------------------

def _write_image(path, w=320, h=240):
    img = np.random.randint(0, 255, (h, w, 3), dtype=np.uint8)
    assert cv2.imwrite(str(path), img)


def test_read_jpeg_size(tmp_path):
    p = tmp_path / "a.jpg"
    _write_image(p, 320, 240)
    assert read_image_size(str(p)) == (320, 240)


def test_read_png_size(tmp_path):
    p = tmp_path / "a.png"
    _write_image(p, 123, 77)
    assert read_image_size(str(p)) == (123, 77)


def test_read_size_garbage_file(tmp_path):
    p = tmp_path / "junk.jpg"
    p.write_bytes(b"not an image at all")
    assert read_image_size(str(p)) is None


# ---------------------------------------------------------------------------
# collect_poses
# ---------------------------------------------------------------------------

def _sidecar(path, **kwargs):
    with open(str(path), "w", encoding="utf-8") as fh:
        json.dump(kwargs, fh)


def test_collect_filename_only(tmp_path):
    _write_image(tmp_path / "38.100000 , -76.200000.jpg")
    poses, skipped = collect_poses(str(tmp_path), default_alt_agl_m=25.0)
    assert len(poses) == 1 and not skipped
    p = poses[0]
    assert (p.lat, p.lon) == (38.1, -76.2)
    assert p.alt_agl_m == 25.0          # default fallback
    assert not p.has_sidecar
    assert p.width == 320 and p.height == 240


def test_collect_prefers_sidecar(tmp_path):
    img = tmp_path / "38.100000 , -76.200000.jpg"
    _write_image(img)
    _sidecar(tmp_path / "38.100000 , -76.200000.json",
             latitude=38.1000005, longitude=-76.2000005,
             rel_alt_m=31.5, compass_hdg_deg=274.3, timestamp_unix=1000.0,
             gimbal={"yaw_deg": 1.0, "pitch_deg": -89.0, "roll_deg": 0.5})
    poses, skipped = collect_poses(str(tmp_path), default_alt_agl_m=25.0)
    assert len(poses) == 1 and not skipped
    p = poses[0]
    assert p.has_sidecar
    assert p.alt_agl_m == 31.5
    assert p.heading_deg == 274.3
    assert p.timestamp == 1000.0
    assert p.gimbal_pitch_deg == -89.0


def test_collect_skips_unusable(tmp_path):
    _write_image(tmp_path / "IMG_0001.jpg")            # no GPS anywhere
    (tmp_path / "broken.jpg").write_bytes(b"garbage")  # bad header
    _write_image(tmp_path / "38.100000 , -76.200000.jpg")
    poses, skipped = collect_poses(str(tmp_path), default_alt_agl_m=25.0)
    assert len(poses) == 1
    assert len(skipped) == 2
    reasons = " ".join(s["reason"] for s in skipped)
    assert "no GPS" in reasons


def test_collect_sidecar_with_bad_alt_falls_back(tmp_path):
    _write_image(tmp_path / "38.100000 , -76.200000.jpg")
    _sidecar(tmp_path / "38.100000 , -76.200000.json",
             latitude=38.1, longitude=-76.2, rel_alt_m=-3.0)
    poses, _ = collect_poses(str(tmp_path), default_alt_agl_m=25.0)
    assert poses[0].alt_agl_m == 25.0


def test_collect_sorts_by_timestamp(tmp_path):
    for i, (name, ts) in enumerate([
            ("38.100000 , -76.200000.jpg", 300.0),
            ("38.101000 , -76.200000.jpg", 100.0),
            ("38.102000 , -76.200000.jpg", 200.0)]):
        _write_image(tmp_path / name)
        _sidecar(tmp_path / name.replace(".jpg", ".json"),
                 latitude=None, longitude=None, timestamp_unix=ts)
    poses, _ = collect_poses(str(tmp_path), default_alt_agl_m=25.0)
    assert [p.timestamp for p in poses] == [100.0, 200.0, 300.0]


# ---------------------------------------------------------------------------
# Heading assignment
# ---------------------------------------------------------------------------

def _pose_line(tmp_path, lats):
    """Images in a south->north line, timestamped in order."""
    for i, lat in enumerate(lats):
        name = f"{lat:.6f} , -76.200000.jpg"
        _write_image(tmp_path / name)
        _sidecar(tmp_path / name.replace(".jpg", ".json"),
                 timestamp_unix=float(i))
    poses, _ = collect_poses(str(tmp_path), default_alt_agl_m=25.0)
    return poses


def test_track_heading_northbound(tmp_path):
    poses = _pose_line(tmp_path, [38.1000, 38.1002, 38.1004])  # ~22 m steps N
    assign_headings(poses, source="track")
    for p in poses:
        assert p.heading_source == "track"
        assert abs(p.heading_deg - 0.0) < 1.0 or abs(p.heading_deg - 360.0) < 1.0


def test_auto_prefers_sidecar_heading(tmp_path):
    poses = _pose_line(tmp_path, [38.1000, 38.1002])
    poses[0].heading_deg = 123.0
    assign_headings(poses, source="auto")
    assert poses[0].heading_deg == 123.0
    assert poses[0].heading_source == "sidecar"
    assert poses[1].heading_source == "track"   # no sidecar heading -> track


def test_fixed_heading(tmp_path):
    poses = _pose_line(tmp_path, [38.1000, 38.1002])
    assign_headings(poses, source="fixed", fixed_heading_deg=270.0)
    assert all(p.heading_deg == 270.0 for p in poses)


def test_close_points_inherit_heading(tmp_path):
    # Two photos in (nearly) the same spot then one far to the east:
    # the stationary pair must inherit the eastbound leg's heading.
    for i, (lat, lon) in enumerate([
            (38.1000, -76.2000),
            (38.1000, -76.20000001),
            (38.1000, -76.1995)]):   # ~44 m east
        name = f"p{i}_38.100000 , {lon:.8f}.jpg"  # not parseable; use sidecar
        _write_image(tmp_path / f"img{i}.jpg")
        _sidecar(tmp_path / f"img{i}.json",
                 latitude=lat, longitude=lon, timestamp_unix=float(i))
    poses, _ = collect_poses(str(tmp_path), default_alt_agl_m=25.0)
    assign_headings(poses, source="track")
    assert abs(poses[0].heading_deg - 90.0) < 1.0
    assert abs(poses[1].heading_deg - 90.0) < 1.0
