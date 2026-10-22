"""
sim_bringup/test/test_mission_tool.py -- PURPOSE: unit tests for the mission file reader of mission_tool (no autopilot
needed): QGC WPL 110 parsing, the checks against what the team's code expects (TAKEOFF, DO_DIGICAM_CONTROL,
two survey waypoints after it, RETURN_TO_LAUNCH last) and the MISSION_ITEM_INT coordinate scaling.
"""

import pytest

from sim_bringup import mission_tool as mt

GOOD = """QGC WPL 110
0\t1\t0\t16\t0\t0\t0\t0\t36.2118\t-96.0055\t200.5\t1
1\t0\t3\t22\t0\t0\t0\t0\t0\t0\t30\t1
2\t0\t3\t16\t0\t0\t0\t0\t36.2120\t-96.0050\t30\t1
3\t0\t3\t203\t0\t0\t0\t0\t1\t0\t0\t1
4\t0\t3\t16\t0\t0\t0\t0\t36.2125\t-96.0050\t30\t1
5\t0\t3\t16\t0\t0\t0\t0\t36.2125\t-96.0040\t30\t1
6\t0\t3\t16\t0\t0\t0\t0\t36.2120\t-96.0040\t30\t1
7\t0\t3\t20\t0\t0\t0\t0\t0\t0\t0\t1
"""


def test_parse_good_mission():
    items = mt.parse_waypoints(GOOD)
    assert len(items) == 8
    assert [i.command for i in items] == [16, 22, 16, 203, 16, 16, 16, 20]
    assert items[0].x == pytest.approx(36.2118) and items[1].z == 30
    assert items[3].name == 'DO_DIGICAM_CONTROL' and items[7].name == 'RETURN_TO_LAUNCH'


def test_describe_mission_matches_the_team_code_needs():
    info = mt.describe_mission(mt.parse_waypoints(GOOD))
    assert info['takeoff'] == 1 and info['digicam'] == 3 and info['rtl'] == 7
    assert info['nav_after_digicam'] == 3 and info['warnings'] == []


def test_describe_mission_warns_about_missing_items():
    text = 'QGC WPL 110\n0 1 0 16 0 0 0 0 1 2 3 1\n1 0 3 16 0 0 0 0 1 2 30 1\n'
    warnings = mt.describe_mission(mt.parse_waypoints(text))['warnings']
    assert any('TAKEOFF' in w for w in warnings)
    assert any('DO_DIGICAM_CONTROL' in w for w in warnings)
    assert any('RETURN_TO_LAUNCH' in w for w in warnings)


def test_bad_files_raise_clear_errors():
    with pytest.raises(ValueError, match='QGC WPL'):
        mt.parse_waypoints('hello\n0 1 0 16 0 0 0 0 1 2 3 1\n')
    with pytest.raises(ValueError, match='columns'):
        mt.parse_waypoints('QGC WPL 110\n0 1 0 16 0 0\n')
    with pytest.raises(ValueError, match='not a number'):
        mt.parse_waypoints('QGC WPL 110\n0 1 0 16 0 0 0 0 x 2 3 1\n')
    with pytest.raises(ValueError, match='no items'):
        mt.parse_waypoints('QGC WPL 110\n')


def test_int_coordinates_scale_only_locations():
    items = mt.parse_waypoints(GOOD)
    wp, cam = items[2], items[3]
    assert wp.x_int() == 362120000 and wp.y_int() == -960050000
    # DO_DIGICAM_CONTROL: x is the "shoot" flag and must stay 1, not 1e7
    assert cam.x_int() == 1 and cam.y_int() == 0


def test_check_mission_command_line(tmp_path, capsys):
    path = tmp_path / 'm.waypoints'
    path.write_text(GOOD)
    assert mt.main(['check-mission', '--mission', str(path)]) == 0
    assert '8 items' in capsys.readouterr().out
    bad = tmp_path / 'bad.waypoints'
    bad.write_text('QGC WPL 110\n0 1 0 16 0 0 0 0 1 2 3 1\n')
    assert mt.main(['check-mission', '--mission', str(bad)]) == 1
