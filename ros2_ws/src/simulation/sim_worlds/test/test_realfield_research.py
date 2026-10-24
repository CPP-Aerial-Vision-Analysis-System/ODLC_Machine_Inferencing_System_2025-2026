"""
sim_worlds/test/test_realfield_research.py -- PURPOSE: check terrain research fixes with small synthetic arrays,
without raw GIS downloads or Gazebo: grass reflectance offsets and fallback, lidar haze and roof heights,
tree confirmation and roof exclusion, building roofs, poles, and the site pack's source credits.
"""

from contextlib import contextmanager
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from sim_worlds.realfield import grids, pack, prepare, site, structures, trees_detect
from sim_worlds.worldgen import sdf


@pytest.mark.parametrize('entry,expected', [
    ({'boa_add_offset': -900}, -900),
    ({'scene': 'S2A_N0400_T14SQF'}, -1000),
    ({'scene': 'S2A_N0300_T14SQF'}, 0),
    ({}, -1000),
])
def test_sentinel_offset_sources(entry, expected):
    """Explicit product metadata wins; older processing baselines do not get a modern offset."""
    assert prepare.s2_offset(entry) == expected


def season_fixture(monkeypatch, count=100, pure=True):
    """Two known reflectance scenes with controlled overlap; no rasterio or files are required."""
    old = np.broadcast_to([1000, 1500, 1000, 3000], (10, 10, 4)).astype(np.float32).copy()
    new = old * [0.8, 0.9, 0.7, 0.9]
    old += 1000
    new += 1000
    old.reshape(-1, 4)[count:] = 0
    new.reshape(-1, 4)[count:] = 0
    monkeypatch.setattr(prepare, 'raw_files', lambda *args: ['2023.tif', '2026.tif'])
    monkeypatch.setattr(prepare, '_reproject', lambda sources, *args: (old if '2023' in sources[0] else new, None))
    monkeypatch.setattr(prepare, 'naip_grass_share',
                        lambda *args: np.full((10, 10), float(pure), dtype=np.float32))
    return {'sources': [{'layer': 's2', 'file': f'{year}.tif', 'boa_add_offset': -1000}
                        for year in (2023, 2026)]}


@pytest.mark.parametrize('pure', [True, False])
def test_season_grade_removes_storage_offset(monkeypatch, pure):
    """Pure grass and the NDVI fallback both recover the known physical reflectance ratios."""
    manifest = season_fixture(monkeypatch, pure=pure)
    meta = {}
    prepare.prepare_season_grade('', manifest, meta, SimpleNamespace(res=0.25), lambda msg: None)
    grade = meta['season_grade']
    assert grade['factors_rgb'] == pytest.approx([0.8, 0.9, 0.7])
    assert grade['unclipped_rgb'] == pytest.approx([0.8, 0.9, 0.7])
    assert grade['cells'] == 100
    assert pack.season_grade(meta)[0] == pytest.approx([0.8, 0.9, 0.7])
    json.dumps(meta, allow_nan=False)


@pytest.mark.parametrize('count', [0, 1, 49])
def test_season_grade_sparse_overlap_keeps_site_fallback(monkeypatch, count):
    """No overlap and too few grass cells produce valid JSON and preserve the measured site fallback."""
    manifest = season_fixture(monkeypatch, count=count)
    meta = {}
    prepare.prepare_season_grade('', manifest, meta, SimpleNamespace(res=0.25), lambda msg: None)
    assert meta['season_grade']['factors_rgb'] is None
    assert meta['season_grade']['cells'] == 0
    assert pack.season_grade(meta)[0] == site.GRASS_GRADE_RGB
    json.dumps(meta, allow_nan=False)


def test_pure_grass_share_rejects_bare_soil_and_tree_pixels():
    """Grass grading uses the photo's grass fraction rather than including forest or concrete."""
    photo = np.broadcast_to([90, 100, 90, 135], (4, 4, 4)).astype(np.uint8).copy()
    photo[:2, 2:] = [100, 100, 100, 100]  # bare/concrete: NDVI zero
    photo[2:, :2] = [60, 70, 60, 160]  # crown: high NDVI and low brightness
    grass = prepare.naip_grass_share(grids.Raster(photo, 0, 1, 0.25), 2, 2, 2)
    assert grass == pytest.approx(np.array([[1, 0], [0, 1]]))


def test_lidar_haze_does_not_hide_a_real_canopy(monkeypatch, tmp_path):
    """A 50 m return above the same cell as a 20 m tree is rejected before the DSM maximum."""
    X, Y = np.meshgrid(np.arange(4) * 0.5 + 0.25, 2 - (np.arange(4) * 0.5 + 0.25))
    pts = SimpleNamespace(x=np.r_[X.ravel(), X.ravel()], y=np.r_[Y.ravel(), Y.ravel()],
                          z=np.r_[np.full(16, 120), np.full(16, 150)],
                          classification=np.zeros(32), withheld=np.zeros(32, dtype=bool))

    @contextmanager
    def open_lidar(path):
        """Yield a tiny point cloud with the same reader interface as laspy."""
        yield SimpleNamespace(chunk_iterator=lambda size: [pts])

    transformer = SimpleNamespace(from_crs=lambda *args, **kwargs:
                                  SimpleNamespace(transform=lambda x, y: (x, y)))
    monkeypatch.setitem(sys.modules, 'laspy', SimpleNamespace(open=open_lidar))
    monkeypatch.setitem(sys.modules, 'pyproj', SimpleNamespace(Transformer=transformer))
    monkeypatch.setattr(prepare, 'raw_files', lambda *args: ['synthetic.laz'])
    monkeypatch.setattr(prepare, 'world_grid', lambda *args: (0, 2, 4, 4))
    meta = {}
    canopy = prepare.prepare_chm('', {}, str(tmp_path), meta,
                                 grids.Raster(np.full((4, 4), 100), 0, 2, 0.5), lambda msg: None)
    assert canopy.data == pytest.approx(np.full((4, 4), 20))
    assert meta['chm_cleaning']['haze_points_dropped'] == 16


def test_building_heights_prefer_lidar_and_ignore_new_empty_footprints():
    """A roof uses lidar in preference to AI height; ground-only footprints keep their fallback height."""
    roof = {'polygon': [(1, 1), (4, 1), (4, 4), (1, 4)], 'height': 50, 'source': 'microsoft'}
    prepare.building_heights_from_lidar([roof], grids.Raster(np.full((10, 10), 4.2), 0, 10, 1))
    assert structures.building_height(roof) == (4.2, 'lidar 2017 median')
    empty = {'polygon': roof['polygon'], 'height': 3, 'source': 'microsoft'}
    prepare.building_heights_from_lidar([empty], grids.Raster(np.ones((10, 10)), 0, 10, 1))
    assert 'height_lidar' not in empty
    assert structures.building_height(empty)[0] == 3


def test_building_roofs_triangulate_in_either_ring_direction():
    """Concave building outlines get complete roofs with upward-facing triangles."""
    outline = [(0, 0), (3, 0), (3, 1), (1, 1), (1, 3), (0, 3)]
    for poly in (outline, outline[::-1]):
        triangles = structures.triangulate(poly)
        signed = [structures._area2([poly[i] for i in triangle]) / 2 for triangle in triangles]
        assert len(triangles) == 4
        assert all(area > 0 for area in signed)
        assert sum(signed) == pytest.approx(5)


@pytest.mark.parametrize('confirmed,roofed,expected', [(True, False, 2), (False, False, 0), (True, True, 0)])
def test_photo_trees_need_confirmation_and_exclude_roofs(confirmed, roofed, expected):
    """Two separate photo crowns stay two trees; unconfirmed vegetation and roofs produce none."""
    X, Y = np.meshgrid(np.arange(160) * 0.25 + 0.125, 40 - (np.arange(160) * 0.25 + 0.125))
    crown = ((X - 10) ** 2 + (Y - 10) ** 2 < 4 ** 2) | ((X - 30) ** 2 + (Y - 30) ** 2 < 4 ** 2)
    photo = grids.Raster(crown, 0, 40, 0.25)
    env = grids.Raster(np.zeros((40, 40)), 0, 40, 1)
    meta = grids.Raster(np.full((40, 40), 5 if confirmed else 0), 0, 40, 1)
    buildings = [{'polygon': [(0, 0), (40, 0), (40, 40), (0, 40)]}] if roofed else []
    roofs = trees_detect.building_mask(buildings, 0, 40, 80, 80, 0.5, 1)
    trees, stats = trees_detect.photo_only_trees(crown, photo, [], meta, env, roofs, lambda msg: None)
    assert len(trees) == stats['kept'] == expected
    if trees:
        assert all(3.5 < tree['r'] < 4.5 for tree in trees)


def test_manual_poles_have_physical_collision_and_research_positions():
    """The three slender obstacles sit in SB1 and extend from below ground to their measured heights."""
    poles = site.manual_structures_xy(site.home_at(210.2))
    expected = [(250.6, -246.4), (268.6, -245.4), (357.6, -246.4)]
    for pole, xy in zip(poles, expected):
        assert [pole['x'], pole['y']] == pytest.approx(xy, abs=1)
        parts = structures.pole_parts(pole['height_m'], pole['diameter_m'])
        lo, hi = sdf.parts_bbox(parts)
        assert lo[2] == pytest.approx(-0.5)
        assert hi[2] == pytest.approx(pole['height_m'])
        assert parts[0]['collision']
    assert not site.POLE_WIRE['default_on']


def test_site_pack_microsoft_credit_matches_manifest(pkg_dir):
    """The compact tracked pack keeps Microsoft's license separate from the OSM attribution."""
    data = Path(pkg_dir) / 'data'
    manifest = json.loads((data / 'real_field_manifest.json').read_text(encoding='utf-8'))
    packed = json.loads((data / 'suas_field_real' / 'pack.json').read_text(encoding='utf-8'))
    source = next(item for item in manifest['sources'] if item['id'] == 'ms_buildings')
    credit = next(item for item in packed['sources'] if item['id'] == 'ms_buildings')
    assert credit['licence'] == source['licence']
    assert credit['attribution'] == source['attribution']
    assert 'CDLA Permissive 2.0' in (data / 'suas_field_real' / 'LICENSES.md').read_text(encoding='utf-8')
