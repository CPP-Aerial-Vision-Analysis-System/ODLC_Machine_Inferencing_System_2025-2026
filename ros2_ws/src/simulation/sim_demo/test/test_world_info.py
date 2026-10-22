"""
sim_demo/test/test_world_info.py -- PURPOSE: unit tests of the tolerant world/truth loader
(world_info.py): the contract's config format, targets given in metres or GPS, class aliases
(mannequin/hiker -> person), keyed and listed targets, polygons, and graceful failure.
"""
import pytest

from sim_demo.geo import GeoOrigin
from sim_demo.world_info import load_world, normalize_class, parse_polygon, parse_truth

ORIGIN = GeoOrigin(38.3154, -76.549, 10.0)


def test_load_fixture_world(world):
    assert world.name == 'testfield'
    assert world.origin.lat == pytest.approx(38.3154)
    assert world.mission_path.endswith('missions/testfield_survey.waypoints')
    assert [t.name for t in world.targets] == ['hiker', 'tent']
    hiker, tent = world.targets
    assert hiker.cls == 'person' and hiker.posture == 'lying'
    assert (hiker.east, hiker.north) == (30.0, 40.0)
    assert hiker.size == (1.8, 0.5, 0.3)                    # default lying-person size
    lat, lon, _ = world.origin.to_geo(30.0, 40.0)
    assert hiker.lat == pytest.approx(lat) and hiker.lon == pytest.approx(lon)
    assert tent.cls == 'tent' and tent.size == (2.4, 2.1, 1.4)
    e, n, _ = world.origin.to_enu(38.315760, -76.548200)
    assert (tent.east, tent.north) == (pytest.approx(e), pytest.approx(n))
    assert len(world.search_area) == 4
    assert any('rock' in w for w in world.warnings)         # the boulder was skipped, with a reason


@pytest.mark.parametrize('word,cls', [('mannequin', 'person'), ('Hiker', 'person'),
                                      ('person', 'person'), ('dome_tent', 'tent'),
                                      ('TENT', 'tent'), ('rock', None), ('', None)])
def test_normalize_class(word, cls):
    assert normalize_class(word) == cls


def test_keyed_targets_and_pose_lists():
    data = {'targets': {
        'mannequin_1': {'position': [10.0, -5.0, 0.0], 'posture': 'standing'},
        'tent_1': {'gps': '38.3155, -76.5489', 'type': 'tent'},
        'tent_2': {'pose': [1, 2, 0, 0, 0, 1.5708], 'class': 'tent'},
    }}
    targets, area, warns = parse_truth(data, ORIGIN)
    names = {t.name: t for t in targets}
    assert set(names) == {'mannequin_1', 'tent_1', 'tent_2'}
    assert names['mannequin_1'].cls == 'person' and names['mannequin_1'].size[2] == 1.8
    assert (names['mannequin_1'].east, names['mannequin_1'].north) == (10.0, -5.0)
    assert names['tent_1'].lat == pytest.approx(38.3155)
    assert names['tent_2'].yaw_deg == pytest.approx(90.0, abs=0.01)
    assert area == [] and warns == []


def test_document_keyed_by_name_without_targets_key():
    data = {'hiker': {'x': 1, 'y': 2}, 'tent': {'lat': 38.3155, 'lon': -76.5489}, 'note': 'x'}
    targets, _, _ = parse_truth(data, ORIGIN)
    assert sorted(t.cls for t in targets) == ['person', 'tent']


def test_polygon_formats():
    sq_m = [[0, 0], [10, 0], [10, 10], [0, 10]]
    assert parse_polygon(sq_m, ORIGIN) == [(0, 0), (10, 0), (10, 10), (0, 10)]
    geo = [{'lat': 38.3154, 'lon': -76.549}, {'lat': 38.3156, 'lon': -76.549},
           {'lat': 38.3156, 'lon': -76.5488}]
    pts = parse_polygon(geo, ORIGIN)
    assert pts[0] == (pytest.approx(0.0, abs=1e-6), pytest.approx(0.0, abs=1e-6))
    rect = parse_polygon({'center': [50, 50], 'size': [100, 40]}, ORIGIN)
    assert len(rect) == 4 and rect[0] == (pytest.approx(0.0), pytest.approx(30.0))
    assert parse_polygon({'latlon': [[38.3154, -76.549], [38.3155, -76.549], [38.3155, -76.5489]]},
                         ORIGIN)[1][1] > 10
    assert parse_polygon('garbage', ORIGIN) == []


def test_real_sim_worlds_schema():
    """The exact format sim_worlds writes (trimmed copy of truth/runway.yaml)."""
    import os
    here = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fixtures', 'simworlds_schema')
    info = load_world(os.path.join(here, 'config', 'runway_like.yaml'))
    assert info.name == 'runway'
    assert info.origin.lat == pytest.approx(36.2143) and info.origin.lon == pytest.approx(-96.0068)
    names = {t.name: t for t in info.targets}
    assert set(names) == {'tent', 'hiker'}                  # distractors are not targets
    tent, hiker = names['tent'], names['hiker']
    assert tent.cls == 'tent' and hiker.cls == 'person' and hiker.posture == 'lying'
    assert (tent.east, tent.north) == (pytest.approx(-0.006), pytest.approx(80.812))
    assert tent.up == pytest.approx(0.0, abs=1e-6)           # center_z - height / 2
    assert hiker.up == pytest.approx(0.178 - 0.357 / 2, abs=1e-6)
    assert tent.yaw_deg == pytest.approx(134.48, abs=0.01)   # 2.3471 rad
    assert tent.size == (2.4, 2.2, 1.35)
    assert tent.lat == pytest.approx(36.21502595)            # ArduPilot-convention GPS kept
    # lat/lon and x/y agree in our (ArduPilot) convention to the millimetre:
    e, n, _ = info.origin.to_enu(hiker.lat, hiker.lon)
    assert (e, n) == (pytest.approx(52.568, abs=0.003), pytest.approx(92.666, abs=0.003))
    assert info.search_areas[0][0] == 'search_box'
    assert info.search_area == [(-35.0, 45.0), (85.0, 45.0), (85.0, 145.0), (-35.0, 145.0)]
    # The standing pilot (class person) is kept as a look-alike for include_distractors.
    assert [(d.name, d.cls) for d in info.distractors] == [('pilot', 'person')]
    assert info.distractors[0].size == (0.551, 0.48, 1.635)


def test_config_search_areas_with_radian_yaw():
    poly = parse_polygon({'center': [0, 0], 'size': [100, 20], 'yaw': 1.5708}, ORIGIN)
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    assert max(ys) - min(ys) == pytest.approx(100.0, abs=0.01)   # rotated 90 deg: along north
    assert max(xs) - min(xs) == pytest.approx(20.0, abs=0.01)


def test_missing_world_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_world('nowhere', str(tmp_path))


def test_missing_truth_is_a_warning(tmp_path):
    (tmp_path / 'config').mkdir()
    (tmp_path / 'config' / 'w.yaml').write_text('home: "1.0,2.0,3.0,0"\ntruth: truth/none.yaml\n')
    info = load_world('w', str(tmp_path))
    assert info.targets == [] and any('not found' in w for w in info.warnings)
