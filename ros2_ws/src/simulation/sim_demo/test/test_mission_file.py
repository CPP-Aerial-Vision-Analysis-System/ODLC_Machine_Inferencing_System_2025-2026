"""
sim_demo/test/test_mission_file.py -- PURPOSE: unit tests of mission_file.py: QGC WPL parsing
and the survey structure analysis (indices must equal what the team's waypoint_manager and
main_controller compute), lane numbering, and the survey bounding box.
"""
import pytest

from sim_demo.geo import GeoOrigin
from sim_demo.mission_file import (analyze, analyze_items, load_waypoints_file,
                                   parse_waypoints_text, survey_bounds_en)


def test_parse_fixture(world):
    items = load_waypoints_file(world.mission_path)
    assert len(items) == 10
    assert items[0].command == 16 and items[0].current == 1
    assert items[1].name == 'TAKEOFF' and items[1].alt == 30.0
    assert items[2].name == 'DO_DIGICAM_CONTROL'
    assert items[9].name == 'RETURN_TO_LAUNCH'


def test_structure_matches_team_rules(world):
    s = analyze_items(load_waypoints_file(world.mission_path))
    assert s.takeoff_index == 1 and s.rtl_index == 9
    assert s.last_before_rtl == 8 and s.buffer_wp == 7          # waypoint_manager rules
    assert s.last_nav_before_rtl == 8                           # main_controller trigger
    assert s.digicam_index == 2
    assert s.survey_seqs == [3, 4, 5, 6, 7, 8]
    assert s.lanes == [(3, 4), (5, 6), (7, 8)] and s.lane_count == 3
    assert s.survey_alt == 30.0


@pytest.mark.parametrize('reached,lane', [(-1, 1), (1, 1), (2, 1), (3, 1), (4, 2), (5, 2),
                                          (6, 3), (7, 3), (8, 3), (9, 3)])
def test_lane_numbers(world, reached, lane):
    s = analyze_items(load_waypoints_file(world.mission_path))
    assert s.lane_for_reached(reached) == lane


def test_no_digicam_and_trailing_do_item():
    # home, takeoff, wp, wp, wp, DO_SET_SERVO, RTL -> trigger is the last NAV (index 4); an odd
    # waypoint left at the end is the hold point.
    s = analyze([16, 22, 16, 16, 16, 183, 20])
    assert s.last_before_rtl == 5 and s.last_nav_before_rtl == 4
    assert s.survey_seqs == [2, 3] and s.lanes == [(2, 3)] and s.hold_index == 4


SIM_WORLDS_RUNWAY = [16, 22, 16, 203, 178, 16, 16, 16, 16, 16, 16, 20]   # sim_worlds runway mission


@pytest.mark.parametrize('hold', [-1, 10])
def test_sim_worlds_mission_structure(hold):
    # 0 home, 1 takeoff, 2 lane-1 start, 3 DIGICAM (camera on leaving WP 2), 4 speed,
    # 5 lane-1 end, 6/7 lane 2, 8/9 lane 3, 10 hold point (centre), 11 RTL.
    s = analyze(SIM_WORLDS_RUNWAY, [30.0] * 12, hold)
    assert s.digicam_index == 3 and s.buffer_wp == 9 and s.last_nav_before_rtl == 10
    assert s.survey_seqs == [2, 5, 6, 7, 8, 9]
    assert s.lanes == [(2, 5), (6, 7), (8, 9)] and s.hold_index == 10
    assert [s.lane_for_reached(r) for r in (1, 2, 5, 6, 7, 8, 9, 10)] == [1, 1, 2, 2, 3, 3, 3, 3]


def test_bad_files():
    with pytest.raises(ValueError):
        parse_waypoints_text('hello\n1 2 3')
    with pytest.raises(ValueError):
        parse_waypoints_text('QGC WPL 110\n0 1 0 16 0 0')


def test_survey_bounds(world):
    items = load_waypoints_file(world.mission_path)
    poly = survey_bounds_en(items, analyze_items(items), GeoOrigin(38.3154, -76.549))
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    assert min(xs) == pytest.approx(0.0, abs=0.5) and max(xs) == pytest.approx(100.0, abs=0.5)
    assert max(ys) == pytest.approx(60.0, abs=0.5)
