"""
sim_worlds/test/test_realfield_world.py -- PURPOSE: check the real-terrain WORLD stage (realfield/world_build.py) and
the planner's ground-height option without Gazebo: the legal survey altitude rule, the flight-height basis of the lane
spacing, and a complete world build from the committed site pack (files, targets on open real ground, same seed =
same layout, different seed = different layout). Needs numpy, PyYAML, Pillow only.
"""

import math

import pytest
import yaml

from sim_worlds import geo, planner, survey
from sim_worlds.realfield import site, world_build


def test_flight_altitude_rule():
    """The height clears 150 ft over the highest ground and 400 ft over the lowest; impossible slopes raise."""
    alt, agl = world_build.flight_altitude(2.4, 11.9, 1.0)
    assert alt - 11.9 >= site.RULE_MIN_AGL_M + 1.0 - 1e-6
    assert alt - 2.4 <= site.RULE_MAX_AGL_M
    assert alt == pytest.approx(11.9 + site.RULE_MIN_AGL_M + 1.0, abs=0.1)
    assert agl == pytest.approx(alt - 11.9)
    with pytest.raises(ValueError):
        world_build.flight_altitude(0.0, 130.0, 1.0)          # a hill taller than the legal band is thick


def test_planner_footprint_uses_the_ground_basis():
    """On sloped ground the mission flies altitude_m above home but the lanes are spaced for the camera height over
    the highest ground (smaller footprint, so closer lanes); the plan still reports the flight height."""
    home = geo.Home(36.2147411, -96.006723, 210.2, 0.0)
    rect = survey.Rect(0.0, 0.0, 260.0, 170.0, 0.0)
    flat = planner.plan_survey_mission(home, rect, planner.SurveySettings(altitude_m=58.7))[1]
    sloped = planner.plan_survey_mission(home, rect, planner.SurveySettings(altitude_m=58.7, agl_basis_m=46.8))[1]
    assert sloped.footprint.width_m < flat.footprint.width_m
    assert sloped.spacing_m <= flat.spacing_m
    assert sloped.altitude_m == 58.7
    items, _, _ = planner.plan_survey_mission(home, rect, planner.SurveySettings(altitude_m=58.7, agl_basis_m=46.8))
    assert {round(i.z, 1) for i in items if i.frame == 3 and i.command == 16} == {58.7}


def test_search_rect_lanes_follow_the_long_side():
    """Lanes run along the long side of a search boundary, whichever way it lies."""
    wide = world_build.search_rect((0.0, 0.0, 300.0, 100.0))
    tall = world_build.search_rect((0.0, 0.0, 100.0, 300.0))
    assert (wide.length, wide.width, wide.yaw) == (300.0, 100.0, 0.0)
    assert (tall.length, tall.width) == (300.0, 100.0) and tall.yaw == pytest.approx(math.pi / 2)


def test_pose_and_colour_lists_are_checked():
    """Two comma-separated values from the allowed list, or nothing (the seed then chooses)."""
    assert world_build._two(None, 'pose', ('a', 'b')) is None
    assert world_build._two('a, b', 'pose', ('a', 'b')) == ['a', 'b']
    with pytest.raises(ValueError):
        world_build._two('a', 'pose', ('a', 'b'))
    with pytest.raises(ValueError):
        world_build._two('a,z', 'pose', ('a', 'b'))


@pytest.fixture(scope='module')
def built(tmp_path_factory):
    """Build the world twice (seed 2026 twice, seed 7 once) into temporary folders from the committed pack."""
    out = {}
    for key, seed in (('a', 2026), ('b', 2026), ('c', 7)):
        d = tmp_path_factory.mktemp(f'world_{key}')
        world_build.run(str(d), seed=seed, asset_mode='primitive', log=lambda *_: None)
        out[key] = d
    return out


def test_world_files_and_targets(built):
    """All four files are written; two hikers and two tents lie inside their boundary on open, level real ground."""
    d = built['a']
    for rel in ('worlds/suas_field_real.sdf', 'config/suas_field_real.yaml', 'truth/suas_field_real.yaml',
                'missions/suas_field_real_survey.waypoints', 'missions/suas_field_real_sb2_survey.waypoints'):
        assert (d / rel).is_file(), rel
    truth = yaml.safe_load((d / 'truth/suas_field_real.yaml').read_text(encoding='utf-8'))
    classes = sorted(t['class'] for t in truth['targets'])
    assert classes == ['person', 'person', 'tent', 'tent']
    ground = world_build.Ground(world_build.SitePack())
    for t in truth['targets']:
        assert ground.class_at(t['x'], t['y']) in world_build.OPEN_GROUND
        assert ground.slope(t['x'], t['y']) <= world_build.MAX_SLOPE
        assert ground.clear_of_trees(t['x'], t['y'], 0.5, gap=0.5)
        assert t['z'] == pytest.approx(ground.z_under(t['x'], t['y'], 0.5), abs=0.35)
    text = (d / 'worlds/suas_field_real.sdf').read_text(encoding='utf-8')
    assert 'model://rf_ground' in text and 'model://rf_trees' in text
    assert '<direction>-0.570 0.444 -0.691</direction>' in text           # the sun of the aerial photo
    assert 'sim_worlds_ground' not in text                                  # no flat plane under the real terrain


def test_same_seed_same_world_different_seed_different_world(built):
    """The layout is reproducible: seed 2026 twice gives the same truth positions, seed 7 gives other ones."""
    def pos(key):
        t = yaml.safe_load((built[key] / 'truth/suas_field_real.yaml').read_text(encoding='utf-8'))
        return [(e['id'], e['x'], e['y']) for e in t['targets']]
    assert pos('a') == pos('b')
    assert pos('a') != pos('c')


def test_missions_obey_the_altitude_rule(built):
    """Each mission flies one height, >= 150 ft over the highest and <= 400 ft over the lowest ground."""
    from sim_worlds import mission
    pk = world_build.SitePack()
    ground = world_build.Ground(pk)
    for suffix, sb in (('', 'search_boundary_1'), ('_sb2', 'search_boundary_2')):
        items = mission.read_waypoints(str(built['a'] / f'missions/suas_field_real{suffix}_survey.waypoints'))
        alts = {round(i.z, 2) for i in items if i.frame == 3 and i.command in (16, 22)}
        assert len(alts) == 1
        gmin, gmax = ground.range_in(world_build.search_rect(site.search_box_xy(sb, pk.home)))
        alt = alts.pop()
        assert alt - gmax >= site.RULE_MIN_AGL_M
        assert alt - gmin <= site.RULE_MAX_AGL_M
