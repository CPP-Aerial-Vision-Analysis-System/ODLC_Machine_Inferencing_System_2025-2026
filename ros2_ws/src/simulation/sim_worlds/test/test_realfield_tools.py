"""
sim_worlds/test/test_realfield_tools.py -- PURPOSE: check the small tools around the real-terrain world without Gazebo, Blender or a GPU:
finding Blender, the Blender camera views, the geometry signature that `verify` uses to prove two PCs build the same world, and the
upscaler's "is the picture up to date" rule. numpy + Pillow only.
"""

import json
import os

import numpy as np
import pytest

from sim_worlds.realfield import blender_tools, glb, site, upscale, verify
from sim_worlds.realfield.sitepack import SitePack


def _write_tri(path, z=1.0):
    pos = np.array([[0, 0, 0], [1, 0, 0], [0, 1, z]], dtype=np.float32)
    nrm = np.tile(np.array([[0, 0, 1]], dtype=np.float32), (3, 1))
    glb.write_glb(path, [{'pos': pos, 'nrm': nrm, 'uv': None, 'idx': np.array([0, 1, 2]), 'mat': None}])


def test_find_blender_uses_the_explicit_path_then_the_environment(tmp_path, monkeypatch):
    fake = tmp_path / 'blender'
    fake.write_text('x')
    assert blender_tools.find_blender(str(fake)) == str(fake)
    monkeypatch.setenv('BLENDER_EXE', str(fake))
    assert blender_tools.find_blender() == str(fake)
    monkeypatch.delenv('BLENDER_EXE')
    assert blender_tools.find_blender(str(tmp_path / 'nope')) in (None, blender_tools.find_blender())


def test_report_says_how_to_install_when_blender_is_missing():
    lines = []
    assert blender_tools.report(None, log=lines.append) == 1
    assert 'winget install --id BlenderFoundation.Blender' in lines[0]


def test_views_sit_above_the_real_ground_and_use_the_photo_sun(tmp_path):
    path = tmp_path / 'views.json'
    views = blender_tools.make_views(str(path))
    cfg = json.loads(path.read_text(encoding='utf-8'))
    assert cfg['sun_direction_enu'] == list(site.SUN_DIRECTION_ENU) and cfg['hfov_deg'] == 81.0
    pk = SitePack()
    for v in views:
        if v['name'].endswith('nadir50'):
            assert v['z'] - float(pk.ground_z(v['x'], v['y'])) == pytest.approx(50.0, abs=0.01)
            assert v['pitch_deg'] == -90
    assert {'search_boundary_1_nadir50', 'search_boundary_2_oblique', 'pad_nadir50', 'overview'} <= {v['name'] for v in views}


def test_geometry_signature_is_stable_and_notices_a_5_mm_move(tmp_path):
    a, b, c = (str(tmp_path / n) for n in ('a.glb', 'b.glb', 'c.glb'))
    _write_tri(a)
    _write_tri(b)
    _write_tri(c, z=1.005)                                   # one vertex 5 mm higher
    sa, sb, sc = verify.signature(a), verify.signature(b), verify.signature(c)
    assert sa == sb and (sa['v'], sa['t']) == (3, 1)
    assert sa['h'] != sc['h']


def test_geometry_walks_the_three_models(tmp_path):
    for model, name in (('rf_ground', 'b00.glb'), ('rf_trees', 'crowns_b00.glb'), ('rf_buildings', 'buildings_near.glb')):
        d = tmp_path / model / 'meshes'
        d.mkdir(parents=True)
        _write_tri(str(d / name))
    assert sorted(verify.geometry(str(tmp_path))) == ['rf_buildings/meshes/buildings_near.glb', 'rf_ground/meshes/b00.glb',
                                                      'rf_trees/meshes/crowns_b00.glb']


def test_upscaled_picture_is_up_to_date_only_for_the_same_pack(tmp_path):
    pk = SitePack()
    assert not upscale.up_to_date(pk, str(tmp_path))                          # nothing there
    (tmp_path / upscale.FILE).write_bytes(b'x')
    (tmp_path / upscale.META).write_text(json.dumps({'fingerprint': upscale.fingerprint(pk)}))
    assert upscale.up_to_date(pk, str(tmp_path))
    (tmp_path / upscale.META).write_text(json.dumps({'fingerprint': 'other'}))
    assert not upscale.up_to_date(pk, str(tmp_path))
