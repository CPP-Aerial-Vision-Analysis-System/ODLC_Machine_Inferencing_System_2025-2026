"""
sim_worlds/test/test_world_files.py -- PURPOSE: check the committed world files against the simulation contract:
configs (contract fields, home == <spherical_coordinates>, spawn on the pad), truth files (valid, targets inside
their search areas, distances), world SDFs (well-formed, contract systems, 1 ms physics, demo GUI, every
model:// and Fuel URI resolvable / pinned).
"""

import math
import os
import re
import xml.etree.ElementTree as ET

import pytest

from sim_worlds import planner, truth as truthmod
from sim_worlds.world_config import available_worlds, load_world_config
from sim_worlds.worldgen import assets

WORLDS = ('runway', 'suas_field')
CONTRACT_SYSTEMS = ('gz::sim::systems::Physics', 'gz::sim::systems::UserCommands',
                    'gz::sim::systems::SceneBroadcaster', 'gz::sim::systems::Sensors', 'gz::sim::systems::Imu',
                    'gz::sim::systems::NavSat')


def _world(pkg_dir, name):
    cfg = load_world_config(name, share=pkg_dir)
    with open(cfg.world_file, encoding='utf-8') as f:
        text = f.read()
    return cfg, text, ET.fromstring(text)


def test_both_worlds_exist(pkg_dir):
    assert set(WORLDS) <= set(available_worlds(pkg_dir))


@pytest.mark.parametrize('name', WORLDS)
def test_config_contract(pkg_dir, name):
    cfg, text, root = _world(pkg_dir, name)
    for p in (cfg.world_file, cfg.mission, cfg.truth):
        assert os.path.isfile(p), p
    assert cfg.raw['world_file'] == f'worlds/{name}.sdf'
    assert cfg.raw['mission'] == f'missions/{name}_survey.waypoints'
    assert cfg.raw['truth'] == f'truth/{name}.yaml'
    w = root.find('world')
    assert w.get('name') == cfg.world_name == name
    sc = w.find('spherical_coordinates')
    assert sc.find('surface_model').text == 'EARTH_WGS84'
    assert sc.find('world_frame_orientation').text == 'ENU'
    assert float(sc.find('latitude_deg').text) == pytest.approx(cfg.home.lat, abs=1e-7)
    assert float(sc.find('longitude_deg').text) == pytest.approx(cfg.home.lon, abs=1e-7)
    assert float(sc.find('elevation').text) == pytest.approx(cfg.home.alt, abs=0.05)
    assert float(sc.find('heading_deg').text) == pytest.approx(0.0)
    assert abs(cfg.spawn['x']) < 6.0 and abs(cfg.spawn['y']) < 6.0 and 0.0 < cfg.spawn['z'] < 1.0


@pytest.mark.parametrize('name', WORLDS)
def test_world_sdf_contract(pkg_dir, name):
    cfg, text, root = _world(pkg_dir, name)
    w = root.find('world')
    assert float(w.find('physics/max_step_size').text) == pytest.approx(0.001)
    assert float(w.find('physics/real_time_factor').text) == pytest.approx(1.0)
    systems = [p.get('name') for p in w.findall('plugin')]
    for s in CONTRACT_SYSTEMS:
        assert s in systems, f'{s} missing'
    sensors = next(p for p in w.findall('plugin') if p.get('name') == 'gz::sim::systems::Sensors')
    assert sensors.find('render_engine').text == 'ogre2'
    gui = w.find('gui')
    files = [p.get('filename') for p in gui.findall('plugin')]
    for needed in ('MinimalScene', 'CameraTracking', 'WorldControl', 'WorldStats', 'ImageDisplay'):
        assert needed in files
    assert 'ClickToFly' not in files
    img = next(p for p in gui.findall('plugin') if p.get('filename') == 'ImageDisplay')
    assert img.get('name') == 'Drone camera (live)'
    assert img.find('topic').text == '/sim/camera/video/image'
    ct = next(p for p in gui.findall('plugin') if p.get('filename') == 'CameraTracking')
    assert ct.find('follow_target').text == 'iris'
    assert 'iris' not in [m.get('name') for m in w.findall('model')]       # the bringup spawns the drone


@pytest.mark.parametrize('name', WORLDS)
def test_world_uris_resolve(pkg_dir, name):
    cfg, text, root = _world(pkg_dir, name)
    for uri in set(re.findall(r'model://([^<\s"]+)', text)):
        path = os.path.join(pkg_dir, 'models', uri)
        assert os.path.exists(path) or os.path.isdir(path), f'model://{uri} not in models/'
    known = {(m.owner.lower(), m.name.lower(), m.version) for m in assets.all_fuel_models()}
    for owner, name_, ver in set(re.findall(r'https://fuel\.gazebosim\.org/1\.0/([^/]+)/models/([^/]+)/(\d+)/files/', text)):
        assert (owner, name_, int(ver)) in known, f'unpinned/unknown Fuel model {owner}/{name_}/{ver}'


@pytest.mark.parametrize('name', WORLDS)
def test_truth_file(pkg_dir, name):
    cfg = load_world_config(name, share=pkg_dir)
    t = truthmod.load_truth(cfg.truth)
    assert truthmod.validate_truth(t) == []
    assert t['world_name'] == name
    assert truthmod.home_of(t).sitl_string() == cfg.home.sitl_string()
    areas = {a['name']: a for a in cfg.search_areas}
    assert {a['name'] for a in t['search_areas']} == set(areas)
    for area_name in areas:
        rect = planner.rect_from_area(areas[area_name])
        inside = [e for e in t['targets'] if e['area'] == area_name]
        assert sorted(e['class'] for e in inside) == ['person', 'tent'], f'{area_name}: one hiker + one tent'
        for e in inside:
            assert rect.contains(e['x'], e['y'], 5.0), f'{e["id"]} too close to the edge of {area_name}'
            assert e['class_id'] == truthmod.CLASS_IDS[e['class']]
            assert len(e['size']) == 3 and all(v > 0 for v in e['size'])
    ids = [e['id'] for e in t['targets'] + t['distractors']]
    assert len(ids) == len(set(ids))


def test_runway_target_distances(pkg_dir):
    t = truthmod.load_truth(load_world_config('runway', share=pkg_dir).truth)
    for e in t['targets']:
        assert 60.0 <= math.hypot(e['x'], e['y']) <= 120.0
    h, tent = truthmod.targets(t, 'person')[0], truthmod.targets(t, 'tent')[0]
    assert math.hypot(h['x'] - tent['x'], h['y'] - tent['y']) >= 30.0


def test_suas_field_targets_and_rules(pkg_dir):
    t = truthmod.load_truth(load_world_config('suas_field', share=pkg_dir).truth)
    assert len(truthmod.targets(t, 'person')) == 2 and len(truthmod.targets(t, 'tent')) == 2
    poses = [e['pose'] for e in truthmod.targets(t, 'person')]
    assert len(set(poses)) == 2, 'the two hikers should not share a pose'
    assert t['altitude_rule_agl_m'] == {'min': 45.7, 'max': 121.9}
    assert len(t['flight_boundary']['latlon']) == 11
    people = [e for e in t['distractors'] if e['class'] == 'person' and str(e['area']).startswith('search_boundary')]
    assert people, 'each search boundary has some standing people (distractors)'


def test_nearest_target(pkg_dir):
    t = truthmod.load_truth(load_world_config('runway', share=pkg_dir).truth)
    h = truthmod.targets(t, 'person')[0]
    best, d = truthmod.nearest_target(t, h['lat'], h['lon'])
    assert best['id'] == h['id'] and d < 0.01
