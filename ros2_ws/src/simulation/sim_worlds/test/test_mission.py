"""
sim_worlds/test/test_mission.py -- PURPOSE: unit tests of the QGC WPL 110 mission writer/reader and of the
"team shape" checks (sim_worlds.mission, sim_worlds.planner), including the committed mission files.
The index rules come from waypoint_mavros/waypoint.py and main/src/main_controller.py of last year's code.
"""

import math
import os

import pytest

from sim_worlds import geo, mission, planner, survey
from sim_worlds.world_config import load_world_config

HOME = geo.Home(36.2143, -96.0068, 210.0, 0.0)


def _mission(speed=5.5):
    rect = survey.Rect(25.0, 95.0, 120.0, 100.0, 0.0)
    items, plan, sp = planner.plan_survey_mission(HOME, rect, planner.SurveySettings(30.0, speed_mps=speed))
    return items, plan


def test_lead_in_moves_only_the_first_lane_start():
    # lead_in_s x lane speed metres before lane 1 (team camera set-up time); same items and indices otherwise
    rect = survey.Rect(25.0, 95.0, 120.0, 100.0, 0.0)
    base, plan, _ = planner.plan_survey_mission(HOME, rect, planner.SurveySettings(30.0, speed_mps=5.0))
    led, _, _ = planner.plan_survey_mission(HOME, rect, planner.SurveySettings(30.0, speed_mps=5.0, lead_in_s=10.0))
    assert [it.command for it in led] == [it.command for it in base]
    assert mission.check_team_shape(led, 30.0, 30.0) == []
    (x0, y0), (x1, y1) = plan.waypoints[0], plan.waypoints[1]
    lx, ly = geo.latlon_to_enu(led[2].x, led[2].y, HOME)
    assert math.hypot(lx - x0, ly - y0) == pytest.approx(50.0, abs=0.01)          # 10 s x 5 m/s
    assert math.hypot(lx - x1, ly - y1) == pytest.approx(math.hypot(x1 - x0, y1 - y0) + 50.0, abs=0.01)  # on the lane
    for a, b in zip(led[3:], base[3:]):
        assert (a.x, a.y) == (b.x, b.y)
    assert planner.lead_in_point((0.0, 0.0), (0.0, 0.0), 5.0) == (0.0, 0.0)


def test_team_shape_and_indices():
    items, plan = _mission()
    assert mission.check_team_shape(items, 30.0, 30.0) == []
    idx = mission.team_indices(items)
    n = len(items)
    assert items[0].command == mission.NAV_WAYPOINT and items[0].frame == mission.FRAME_GLOBAL
    assert idx['takeoff_index'] == 1
    assert idx['digicam_indices'] == [3]                       # "Mission: 3 DigiCamCtrl", like the team's tlogs
    assert idx['rtl_index'] == n - 1
    assert idx['last_before_rtl'] == n - 2 == idx['hold_trigger_wp'] == idx['last_nav_before_rtl']
    assert idx['buffer_wp'] == n - 3                           # = end of the last lane
    assert idx['insert_index'] == n - 1                        # targets go before the RTL
    # buffer_wp is the last lane end, the hold point is the area centre
    last_lane_end = plan.waypoints[-1]
    assert geo.latlon_to_enu(items[idx['buffer_wp']].x, items[idx['buffer_wp']].y, HOME) == pytest.approx(last_lane_end, abs=1e-3)
    assert geo.latlon_to_enu(items[idx['last_before_rtl']].x, items[idx['last_before_rtl']].y, HOME) == pytest.approx((25.0, 95.0), abs=1e-3)
    # DO_CHANGE_SPEED follows the DigiCamCtrl item
    assert items[4].command == mission.DO_CHANGE_SPEED and items[4].param2 == 5.5


def test_without_speed_item():
    items, _ = _mission(speed=0)
    assert mission.check_team_shape(items) == []
    assert not any(it.command == mission.DO_CHANGE_SPEED for it in items)


def test_write_read_round_trip(tmp_path):
    items, _ = _mission()
    p = tmp_path / 'm.waypoints'
    mission.write_waypoints(str(p), items)
    text = p.read_text()
    assert text.splitlines()[0] == 'QGC WPL 110'
    assert all(len(ln.split('\t')) == 12 for ln in text.splitlines()[1:])
    back = mission.read_waypoints(str(p))
    assert len(back) == len(items)
    for a, b in zip(items, back):
        assert (a.seq, a.current, a.frame, a.command, a.autocontinue) == (b.seq, b.current, b.frame, b.command, b.autocontinue)
        assert a.x == pytest.approx(b.x, abs=1e-8) and a.y == pytest.approx(b.y, abs=1e-8) and a.z == pytest.approx(b.z, abs=1e-6)


def test_read_rejects_bad_files(tmp_path):
    p = tmp_path / 'bad.waypoints'
    p.write_text('hello\n')
    with pytest.raises(ValueError):
        mission.read_waypoints(str(p))
    p.write_text('QGC WPL 110\n0\t1\t0\t16\n')
    with pytest.raises(ValueError):
        mission.read_waypoints(str(p))


def test_shape_checker_catches_mistakes():
    items, _ = _mission()
    # a DO item right before RTL: the controller would never hold
    bad = items[:-2] + [mission.MissionItem(0, 0, 0, mission.DO_DIGICAM_CONTROL), items[-1]]
    for i, it in enumerate(bad):
        it.seq = i
    assert any('R-1' in p for p in mission.check_team_shape(bad))
    # no RTL
    assert any('RETURN_TO_LAUNCH' in p for p in mission.check_team_shape(items[:-1]))
    # no DigiCamCtrl
    nodc = [it for it in items if it.command != mission.DO_DIGICAM_CONTROL]
    for i, it in enumerate(nodc):
        it.seq = i
    assert any('DIGICAM' in p for p in mission.check_team_shape(nodc))
    # altitude band
    items2, _ = _mission()
    assert any('below' in p for p in mission.check_team_shape(items2, alt_min=45.7))


@pytest.mark.parametrize('world,alt,band', [('runway', 30.0, (20.0, 40.0)), ('suas_field', 50.0, (45.7, 121.9))])
def test_committed_missions(pkg_dir, world, alt, band):
    cfg = load_world_config(world, share=pkg_dir)
    items = mission.read_waypoints(cfg.mission)
    assert mission.check_team_shape(items, *band) == []
    assert all(it.z == pytest.approx(alt) for it in items if it.command in (mission.NAV_WAYPOINT, mission.NAV_TAKEOFF)
               and it.seq > 0)
    area = next(a for a in cfg.search_areas if a['name'] == cfg.survey['area'])
    rect = planner.rect_from_area(area)
    lead_in = float(cfg.survey.get('lead_in_s', 0.0) or 0.0)
    for it in items[2:]:
        if it.command == mission.NAV_WAYPOINT:
            x, y = geo.latlon_to_enu(it.x, it.y, cfg.home)
            if it.seq == 2 and lead_in > 0:
                # lane 1 starts BEFORE the area (planner lead_in_s): on the lane's line, outside the near edge
                u, v = rect.to_rect(x, y)
                assert abs(u) > rect.length / 2.0 and abs(v) <= rect.width / 2.0 + 0.5, \
                    f'lead-in start ({x:.1f}, {y:.1f}) should be on lane 1, before {area["name"]}'
                continue
            assert rect.contains(x, y, -0.5), f'waypoint {it.seq} at ({x:.1f}, {y:.1f}) is outside {area["name"]}'
    # the mission can be re-made from the config alone and gives the same mission (the config stores the area to
    # the millimetre, so allow 1e-7 degree = about 1 cm)
    items2, _, _ = planner.plan_survey_mission(cfg.home, rect, planner.settings_from_config(cfg.survey))
    assert len(items2) == len(items)
    for a, b in zip(items2, items):
        assert (a.seq, a.current, a.frame, a.command, a.autocontinue) == (b.seq, b.current, b.frame, b.command, b.autocontinue)
        assert (a.param1, a.param2, a.param3, a.param4, a.z) == pytest.approx((b.param1, b.param2, b.param3, b.param4, b.z))
        assert a.x == pytest.approx(b.x, abs=1e-7) and a.y == pytest.approx(b.y, abs=1e-7)


def test_extra_mission_sb2(pkg_dir):
    p = os.path.join(pkg_dir, 'missions', 'suas_field_sb2_survey.waypoints')
    items = mission.read_waypoints(p)
    assert mission.check_team_shape(items, 45.7, 121.9) == []


def test_path_length_positive():
    items, plan = _mission()
    assert mission.path_length_m(items, HOME) > plan.path_length_m > 0
    assert 'buffer_wp' in mission.describe(items, HOME)
    assert math.isfinite(mission.path_length_m(items, HOME))
