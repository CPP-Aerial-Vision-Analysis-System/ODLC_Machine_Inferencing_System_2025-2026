"""
sim_bringup/test/test_mission_tool.py -- PURPOSE: unit tests for the mission file reader of mission_tool (no autopilot
needed): QGC WPL 110 parsing, the checks against what the team's code expects (TAKEOFF, DO_DIGICAM_CONTROL,
two survey waypoints after it, RETURN_TO_LAUNCH last) and the MISSION_ITEM_INT coordinate scaling. Also: the drone's
state (armed, mode) is read from the autopilot component only, while status texts from every component are kept.
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


# ---------------------------------------------------------------------------------------------
# The drone's state comes from the autopilot component only (regression: MAVROS's heartbeat, forwarded by ArduPilot
# since MAVROS moved to SITL SERIAL2, made the tool read "disarmed / STABILIZE" in the middle of a flight).
# ---------------------------------------------------------------------------------------------

class _Msg:
    """A tiny stand-in for a pymavlink message."""

    def __init__(self, mtype, comp, base_mode=0, custom_mode=0, text=''):
        self.mtype, self.comp = mtype, comp
        self.base_mode, self.custom_mode, self.text = base_mode, custom_mode, text

    def get_type(self):
        return self.mtype

    def get_srcSystem(self):
        return 1

    def get_srcComponent(self):
        return self.comp


class _Link:
    """Hands out a fixed list of messages, then nothing."""

    def __init__(self, msgs):
        self.msgs = list(msgs)

    def recv_match(self, blocking=True, timeout=0.2):
        return self.msgs.pop(0) if self.msgs else None

    def mode_mapping(self):
        return {'STABILIZE': 0, 'AUTO': 3, 'GUIDED': 4}


def _vehicle(msgs):
    from types import SimpleNamespace
    v = object.__new__(mt.Vehicle)          # skip __init__: no network
    v.m, v.mavlink = _Link(msgs), SimpleNamespace(MAV_MODE_FLAG_SAFETY_ARMED=128)
    v.statustexts, v.last, v.ts, v.tc = [], {}, 1, 1
    return v


def test_mavros_heartbeat_does_not_replace_the_autopilot_state():
    armed_auto = _Msg('HEARTBEAT', comp=1, base_mode=128 | 1, custom_mode=3)
    mavros = _Msg('HEARTBEAT', comp=191, base_mode=0, custom_mode=0)       # MAVROS: "disarmed, mode 0"
    v = _vehicle([armed_auto, mavros])
    v.pump(0.0)
    v.pump(0.0)
    assert v.armed is True
    assert v.mode == 'AUTO'


def test_team_status_texts_are_still_collected():
    v = _vehicle([_Msg('STATUSTEXT', comp=191, text='Detected tent'), _Msg('STATUSTEXT', comp=1, text='Mission: 3 DigiCamCtrl')])
    v.pump(0.0)
    v.pump(0.0)
    assert v.statustexts == ['Detected tent', 'Mission: 3 DigiCamCtrl']


def test_pump_returns_only_messages_from_the_autopilot():
    v = _vehicle([_Msg('HEARTBEAT', comp=191), _Msg('HEARTBEAT', comp=1, custom_mode=4)])
    assert v.pump(0.0, types=['HEARTBEAT']) is None
    assert v.pump(0.0, types=['HEARTBEAT']).custom_mode == 4
