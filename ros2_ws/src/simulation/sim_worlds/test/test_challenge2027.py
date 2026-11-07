"""PURPOSE: Verify repeatable 2027 recipes, actual terrain contact, mission contracts and external assets."""

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from sim_worlds import mission
from sim_worlds.challenge2027.bundle import write_bundle
from sim_worlds.challenge2027.scenario import (
    CASES, COLORS, GRID_CELLS, GRID_MIN, GRID_STEP, make_scene, surface_height,
)
from sim_worlds.world_config import load_world_config


@pytest.fixture(scope='module')
def scene():
    """One deterministic recipe shared by the nonmutating tests."""
    return make_scene()


def test_seed_reproduces_every_scene_choice(scene):
    assert scene == make_scene()
    different = make_scene(2028)
    assert different['hiker'] != scene['hiker']
    assert different['terrain']['heights'] != scene['terrain']['heights']
    json.dumps(scene, allow_nan=False)


@pytest.mark.parametrize('seed', [-1, True, 1.5, '1', 2**31])
def test_invalid_seed_rejected(seed):
    with pytest.raises(ValueError, match='seed'):
        make_scene(seed)


def test_unknown_case_rejected():
    with pytest.raises(ValueError, match='case'):
        make_scene(case='typo')


@pytest.mark.parametrize('case', CASES)
def test_variants_have_one_hiker_or_an_explicit_negative(case):
    s = make_scene(case=case)
    assert (s['hiker'] is None) == (case == 'negative')
    if s['hiker']:
        assert all(s['hiker'][k] in COLORS for k in ('shirt_color', 'pants_color', 'suitcase_color'))
        suitcase = s['hiker']['suitcase']
        assert math.hypot(suitcase['x']-s['hiker']['x'], suitcase['y']-s['hiker']['y']) < 2
    assert (s['lighting']['elevation_deg'] < 20) == (case == 'low_sun')
    if case == 'low_contrast':
        assert s['hiker']['shirt_color'] == 'green'
    if case == 'occluded':
        assert any(o['kind']=='bush' and math.hypot(o['x']-s['hiker']['x'],o['y']-s['hiker']['y']) < 1
                   for o in s['objects'])


def test_search_area_and_brief_conversions(scene):
    assert math.prod(scene['search_area']['size'])/4046.8564224 == pytest.approx(10, rel=.001)
    assert scene['delivery']['min_agl_m'] == pytest.approx(150*.3048)
    assert scene['delivery']['target_radius_m'] == pytest.approx(50*.3048)
    assert scene['delivery']['max_mass_kg'] == pytest.approx(3*.45359237)
    assert scene['delivery']['max_size_m'] == pytest.approx([11*.0254,9*.0254,4*.0254])
    assert 'preliminary' in scene['source']['status']
    assert 'assumptions' in scene['training_lz_policy']['note']


def test_every_object_uses_the_actual_collision_surface(scene):
    grid = scene['terrain']['heights']
    entries = scene['objects'] + scene['landing_zones'] + [scene['hiker'], scene['hiker']['suitcase']]
    for entry in entries:
        assert entry['z'] == pytest.approx(surface_height(entry['x'],entry['y'],grid), abs=1e-9)
    assert surface_height(0,0,grid) == 0
    assert len(grid) == GRID_CELLS+1
    assert all(len(row)==GRID_CELLS+1 for row in grid)


def test_interpolation_matches_both_exported_triangle_planes():
    grid=[[0.0]*(GRID_CELLS+1) for _ in range(GRID_CELLS+1)]
    grid[0][1],grid[1][0],grid[1][1]=1,9,11
    x,y=GRID_MIN
    assert surface_height(x+.75*GRID_STEP,y+.25*GRID_STEP,grid) == pytest.approx(3.25)
    assert surface_height(x+.25*GRID_STEP,y+.75*GRID_STEP,grid) == pytest.approx(7.25)
    with pytest.raises(ValueError,match='outside'):
        surface_height(x-1,y,grid)


def test_good_landing_disks_are_flat_and_bad_slope_is_real(scene):
    grid=scene['terrain']['heights']
    for zone in scene['landing_zones']:
        if zone['valid']:
            heights=[surface_height(zone['x']+dx,zone['y']+dy,grid)
                     for dx in range(-9,10,3) for dy in range(-9,10,3) if dx*dx+dy*dy<=81]
            assert max(heights)-min(heights)<.1
    slope=next(z for z in scene['landing_zones'] if z['id']=='slope')
    rise=surface_height(slope['x']+4,slope['y'],grid)-surface_height(slope['x']-4,slope['y'],grid)
    assert math.degrees(math.atan(rise/8)) == pytest.approx(8,abs=.1)
    assert {r for z in scene['landing_zones'] for r in z['reasons']} == {
        'obstruction','slope','overhead_wire','water'}
    nearest=min(scene['landing_zones'],key=lambda z:math.hypot(z['x']-scene['hiker']['x'],z['y']-scene['hiker']['y']))
    assert not nearest['valid']


def _fake_blender_assets(output, scene):
    """Minimal files isolate packaging tests from expensive Blender jobs."""
    meshes=output/'models'/scene['world_name']/'meshes'
    meshes.mkdir(parents=True)
    for name in ('terrain.obj','terrain.png','scenery.dae'):
        (meshes/name).write_text('test-only asset',encoding='utf-8')
    (output/'assets.json').write_text(json.dumps({'hiker_bbox':{
        'size':[1,2,.4],'center':[scene['hiker']['x'],scene['hiker']['y'],scene['hiker']['z']+.2],'yaw':0}}))


def test_bundle_loaders_mission_and_portable_resource_paths(tmp_path,scene):
    _fake_blender_assets(tmp_path,scene)
    result=write_bundle(scene,tmp_path)
    cfg=load_world_config(str(tmp_path/result['config']))
    assert all(Path(p).exists() for p in (cfg.world_file,cfg.truth,cfg.mission))
    root=ET.parse(cfg.world_file).getroot()
    assert root.find('world').get('name') == cfg.world_name
    assert float(root.find('world/spherical_coordinates/elevation').text)==cfg.home.alt
    model=ET.parse(tmp_path/'models'/scene['world_name']/'model.sdf').getroot()
    visual=model.find('.//visual/geometry/mesh/uri').text
    collision=model.find('.//collision/geometry/mesh/uri').text
    assert visual==collision and visual.endswith('terrain.obj')
    for uri in model.findall('.//uri'):
        assert uri.text.startswith('model://')
        assert (tmp_path/'models'/uri.text.removeprefix('model://')).is_file()
    items=mission.read_waypoints(cfg.mission)
    assert mission.check_team_shape(items,alt_min=50,alt_max=122)==[]
    assert result['minimum_survey_agl_m'] >= 50
    truth=yaml.safe_load(Path(cfg.truth).read_text())
    assert [t['class'] for t in truth['targets']]==['person']
    assert truth['targets'][0]['size']==[1,2,.4]
    public=json.loads((tmp_path/'task.json').read_text())
    assert 'hiker' not in public
    assert all(set(zone)=={'id','x','y','radius_m'} for zone in public['landing_zone_candidates'])
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'sim_bringup'))
    from sim_bringup.sim_paths import load_world_config as bringup_load
    brought=bringup_load(str(tmp_path/result['config']),None)
    assert brought.world_file==cfg.world_file
    assert brought.mission==cfg.mission


def test_assembling_missing_blender_export_fails_early(tmp_path,scene):
    with pytest.raises(ValueError,match='Blender'):
        write_bundle(scene,tmp_path)
