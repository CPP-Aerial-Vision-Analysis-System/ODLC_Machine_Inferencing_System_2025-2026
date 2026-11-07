"""
sim_demo/test/test_mission_check.py -- PURPOSE: unit tests of mission_check.py, the plain-words mission checker that
simulation/scripts/mission.sh runs before every upload.

Covered: the real mission files the simulator ships (sim_worlds/missions) and the test-world fixture are accepted with
the team-code indices AGENTS.md section 3.4 lists; a Mission Planner style real-drone file (CRLF, frame 0 home at the
field's height above sea level) is accepted; every problem type produces exactly its error or warning; the file
errors (missing, not a mission, broken line, empty) give exit code 3; QGroundControl .plan files (simple items and a
survey pattern with stored waypoints) give the same indices as the .waypoints file and convert to QGC WPL 110.
Pure Python: runs with plain pytest, no ROS needed.
"""
import json
import os

import pytest

from sim_demo import mission_check as mc
from sim_demo.mission_file import MissionItem

HERE = os.path.dirname(os.path.abspath(__file__))
MISSIONS = os.path.normpath(os.path.join(HERE, '..', '..', 'sim_worlds', 'missions'))
FIXTURE = os.path.join(HERE, 'fixtures', 'worlds', 'missions', 'testfield_survey.waypoints')

HOME = (36.2143, -96.0068)


# ---- helpers --------------------------------------------------------------------------------------------------

def item(cmd, lat=0.0, lon=0.0, alt=0.0, frame=3, p1=0.0, p2=0.0, p3=0.0, p4=0.0, auto=1):
    """One MissionItem (seq is fixed by build())."""
    return MissionItem(seq=0, current=0, frame=frame, command=cmd, param1=p1, param2=p2, param3=p3, param4=p4,
                       lat=lat, lon=lon, alt=alt, autocontinue=auto)


def wp(k, alt=50.0, frame=3):
    """A survey waypoint k*20 m north-east of home."""
    return item(16, HOME[0] + 0.0002 * k, HOME[1] + 0.0002 * k, alt, frame)


def team_mission(alt=50.0):
    """The team's shape: HOME, TAKEOFF, lane-1 start, DIGICAM, SPEED, 5 lane ends/starts, hold, RTL (R = 11)."""
    items = [item(16, HOME[0], HOME[1], 210.0, frame=0), item(22, alt=alt), wp(1, alt), item(203, frame=0),
             item(178, frame=0, p1=1, p2=5.5, p3=-1)]
    items += [wp(k, alt) for k in range(2, 8)]
    items += [item(20)]
    return build(items)


def build(items):
    for i, it in enumerate(items):
        it.seq = i
    return items


def check(items, **kw):
    return mc.check_mission(items, source='test', **kw)


def has(texts, fragment):
    return any(fragment in t for t in texts)


def write(tmp_path, name, text, newline='\n'):
    p = tmp_path / name
    p.write_bytes(text.replace('\n', newline).encode('utf-8'))
    return str(p)


# ---- the shipped missions and the fixture ---------------------------------------------------------------------

@pytest.mark.parametrize('name', ['runway_survey.waypoints', 'suas_field_survey.waypoints',
                                  'suas_field_sb2_survey.waypoints'])
def test_shipped_missions_are_flyable(name):
    items, kind, renumbered = mc.load_mission(os.path.join(MISSIONS, name))
    rep = check(items, kind=kind, home=HOME, renumbered=renumbered)
    assert rep.ok, rep.errors
    s = rep.summary
    # AGENTS.md 3.4 (runway): takeoff 1, DigiCamCtrl 3, buffer_wp 9, hold 10, RTL 11, 12 items
    assert (s.takeoff_index, s.digicam_index, s.buffer_wp, s.last_before_rtl, s.rtl_index) == (1, 3, 9, 10, 11)
    assert len(items) == 12 and s.lane_count == 3 and rep.speed_mps > 0


def test_runway_altitude_is_below_the_suas_band_only_a_warning():
    items, _, _ = mc.load_mission(os.path.join(MISSIONS, 'runway_survey.waypoints'))
    rep = check(items)
    assert rep.ok
    assert has(rep.warnings, 'below the SUAS 2026 rule')
    assert mc.main([os.path.join(MISSIONS, 'runway_survey.waypoints'), '--quiet']) == 0
    assert mc.main([os.path.join(MISSIONS, 'runway_survey.waypoints'), '--strict', '--quiet']) == 1


def test_suas_field_mission_has_no_warnings():
    assert mc.main([os.path.join(MISSIONS, 'suas_field_survey.waypoints'), '--strict', '--quiet']) == 0


def test_fixture_with_digicam_right_after_takeoff_is_ok_with_a_note():
    items, _, _ = mc.load_mission(FIXTURE)
    rep = check(items)
    assert rep.ok, rep.errors
    assert has(rep.notes, 'right after the TAKEOFF')


def test_mission_planner_real_drone_file_with_crlf(tmp_path):
    # What Mission Planner saves for a real field: CRLF, home at the field's height above sea level (frame 0),
    # relative altitudes in feet-friendly metres, the team's shape.
    text = ('QGC WPL 110\n'
            '0\t1\t0\t16\t0\t0\t0\t0\t38.3154000\t-76.5490000\t4.200000\t1\n'
            '1\t0\t3\t22\t0\t0\t0\t0\t0\t0\t50.000000\t1\n'
            '2\t0\t3\t16\t0\t0\t0\t0\t38.3160000\t-76.5500000\t50.000000\t1\n'
            '3\t0\t3\t203\t0\t0\t0\t0\t0\t0\t0\t1\n'
            '4\t0\t3\t178\t1\t6\t-1\t0\t0\t0\t0\t1\n'
            '5\t0\t3\t16\t0\t0\t0\t0\t38.3160000\t-76.5470000\t50.000000\t1\n'
            '6\t0\t3\t16\t0\t0\t0\t0\t38.3165000\t-76.5470000\t50.000000\t1\n'
            '7\t0\t3\t16\t0\t0\t0\t0\t38.3165000\t-76.5500000\t50.000000\t1\n'
            '8\t0\t3\t16\t0\t0\t0\t0\t38.3162000\t-76.5485000\t50.000000\t1\n'
            '9\t0\t3\t20\t0\t0\t0\t0\t0\t0\t0\t1\n')
    path = write(tmp_path, 'field.waypoints', text, newline='\r\n')
    items, kind, renumbered = mc.load_mission(path)
    rep = check(items, kind=kind)
    assert rep.ok and not rep.warnings, (rep.errors, rep.warnings)
    assert (rep.summary.buffer_wp, rep.summary.last_before_rtl, rep.summary.rtl_index) == (7, 8, 9)
    assert mc.main([path, '--quiet']) == 0


def test_team_mission_helper_is_clean():
    rep = check(team_mission())
    assert rep.ok and not rep.warnings, (rep.errors, rep.warnings)


# ---- each problem type -----------------------------------------------------------------------------------------

def test_no_takeoff_is_an_error():
    items = team_mission()
    del items[1]
    rep = check(build(items))
    assert has(rep.errors, 'no TAKEOFF')


def test_takeoff_altitude_zero_is_an_error():
    items = team_mission()
    items[1].alt = 0.0
    assert has(check(items).errors, 'TAKEOFF altitude is 0.0 m')


def test_no_rtl_is_an_error():
    items = team_mission()[:-1]
    assert has(check(items).errors, 'no RETURN_TO_LAUNCH')


def test_two_rtls_is_an_error():
    items = team_mission()
    items.insert(6, item(20))
    assert has(check(build(items)).errors, '2 RETURN_TO_LAUNCH items')


def test_items_after_rtl_are_a_warning():
    items = team_mission() + [wp(9)]
    rep = check(build(items))
    assert rep.ok and has(rep.warnings, 'is not the last item')


def test_do_item_at_hold_point_is_an_error():
    items = team_mission()
    items.insert(-1, item(178, frame=0, p1=1, p2=3))        # DO_CHANGE_SPEED right before RTL
    rep = check(build(items))
    assert has(rep.errors, 'right before RETURN_TO_LAUNCH') and not rep.ok


def test_do_item_at_buffer_wp_is_an_error():
    items = team_mission()
    items.insert(-2, item(183, frame=0, p1=9, p2=1500))     # DO_SET_SERVO two before RTL
    rep = check(build(items))
    assert has(rep.errors, 'buffer_wp, two before RETURN_TO_LAUNCH')


def test_too_short_is_an_error():
    items = build([item(16, HOME[0], HOME[1], 210.0, frame=0), item(22, alt=50), item(203, frame=0), wp(1),
                   item(20)])
    assert has(check(items).errors, 'too short')


def test_no_digicam_is_an_error():
    items = team_mission()
    del items[3]
    assert has(check(build(items)).errors, 'no DO_DIGICAM_CONTROL')


def test_digicam_after_buffer_wp_is_an_error():
    items = team_mission()
    dc = items.pop(3)
    items.insert(-2, dc)                                   # between the last lane end and the hold: too late
    items = build(items)
    rep = check(items)
    assert has(rep.errors, 'at or after buffer_wp') or has(rep.errors, 'buffer_wp, two before')


def test_two_digicams_and_digicam_before_takeoff_are_warnings():
    items = team_mission()
    items.insert(1, item(203, frame=0))
    rep = check(build(items))
    assert has(rep.warnings, '2 DO_DIGICAM_CONTROL') and has(rep.warnings, 'before the TAKEOFF')
    assert has(rep.warnings, 'not item 1')


def test_nan_and_impossible_positions_are_errors():
    items = team_mission()
    items[5].alt = float('nan')
    items[6].lat = 123.0
    rep = check(items)
    assert has(rep.errors, 'NaN or infinity') and has(rep.errors, 'impossible position')


def test_zero_position_waypoint_is_a_warning():
    items = team_mission()
    items[5].lat = items[5].lon = 0.0
    assert has(check(items).warnings, 'latitude/longitude 0, 0')


def test_autocontinue_zero_is_a_warning():
    items = team_mission()
    items[6].autocontinue = 0
    assert has(check(items).warnings, 'autocontinue 0')


def test_above_400_ft_is_a_warning_with_feet():
    rep = check(team_mission(alt=130.0))
    assert has(rep.warnings, 'ABOVE 400 ft') and has(rep.warnings, '427 ft')


def test_search_at_ground_level_is_an_error_not_fine_for_a_test(tmp_path, capsys):
    # The reviewer's case: every search waypoint (and the hold point) at 0 m above home.
    items = team_mission()
    for it in items[2:-1]:
        if it.command == 16:
            it.alt = 0.0
    rep = check(items)
    assert not rep.ok and has(rep.errors, 'at or below 5 m above home')
    assert has(rep.errors, 'items 2, 5, 6, 7, 8, 9, 10;')
    assert not has(rep.warnings, 'Fine for a simulator test')
    path = str(tmp_path / 'alt0.waypoints')
    mc.write_waypoints(path, items)
    assert mc.main([path, '-q']) == 1                       # mission.sh load refuses it without --force
    assert 'RESULT: NOT READY' in capsys.readouterr().out


@pytest.mark.parametrize('alt', [-3.0, 2.0, 5.0])
def test_one_low_or_negative_item_is_an_error(alt):
    items = team_mission()
    items[6].alt = alt
    rep = check(items)
    assert has(rep.errors, 'lowest: item 6 at')
    assert has(rep.errors, 'BELOW the home position') == (alt < 0)


def test_hold_point_at_ground_level_is_an_error():
    items = team_mission()
    items[-2].alt = 0.0                                     # the hold point right before RTL
    assert has(check(items).errors, 'lowest: item 10 at 0.0 m')


def test_negative_height_in_frame_0_is_an_error():
    items = team_mission()
    for it in items[2:-1]:
        if it.command == 16:
            it.frame, it.alt = 0, 50.0                     # 50 m above sea, home is at 210 m: 160 m underground
    rep = check(items)
    assert has(rep.errors, 'BELOW the home position') and has(rep.errors, '-160.0 m')


def test_low_heights_say_fine_for_a_test_only_above_the_minimum():
    rep = check(team_mission(alt=10.0))
    assert rep.ok and has(rep.warnings, 'too low even for a simulator test')
    assert not has(rep.warnings, 'Fine for a simulator test')
    rep = check(team_mission(alt=30.0))
    assert rep.ok and has(rep.warnings, 'Fine for a simulator test')


def test_takeoff_as_item_0_without_home_is_an_error():
    # A hand-written file without the home line: item 0 = TAKEOFF, which the autopilot never flies.
    items = build(team_mission()[1:])
    rep = check(items)
    assert not rep.ok and has(rep.errors, 'item 0 is the TAKEOFF')
    assert not has(rep.errors, 'no TAKEOFF item') and not has(rep.warnings, 'not the HOME line')


def test_nan_servo_and_speed_parameters_do_not_crash():
    items = team_mission()
    items.insert(5, item(183, frame=0, p1=float('nan'), p2=1500))
    items[4].param1 = float('inf')
    rep = check(build(items))
    assert has(rep.errors, 'NaN or infinity')
    assert 'RESULT: NOT READY' in mc.format_report(rep)


def test_frame_0_altitudes_are_converted_and_warned():
    items = team_mission()
    for it in items[2:-1]:
        if it.command == 16:
            it.frame, it.alt = 0, 260.0                    # 260 m above sea = 50 m above the 210 m home
    rep = check(items)
    assert has(rep.warnings, 'above sea level (frame 0)')
    assert not has(rep.warnings, 'SUAS')                    # 50 m above home = 164 ft: inside the band
    assert abs(rep.heights_m[2] - 50.0) < 1e-6


def test_far_away_mission_is_an_error_only_against_the_real_home():
    items = team_mission()
    shifted = [MissionItem(**{**it.__dict__}) for it in items]
    for it in shifted:
        if it.lat != 0.0:
            it.lat += 2.0                                  # ~220 km north, home moved with it
    assert check(shifted).ok                               # consistent file on its own
    rep = check(shifted, home=HOME)
    assert has(rep.errors, 'the drone would fly away')


def test_far_waypoint_in_the_file_is_a_warning():
    items = team_mission()
    items[6].lat += 0.1                                    # ~11 km from the file's home
    assert has(check(items).warnings, 'km from the home position in the file')


def test_payload_servo_and_land_are_warnings():
    items = team_mission()
    items.insert(5, item(183, frame=0, p1=13, p2=1400))
    items.insert(-1, item(21, HOME[0], HOME[1], 0.0))
    rep = check(build(items))
    assert has(rep.warnings, 'channel 13 (tent payload)') and has(rep.warnings, 'is LAND')


def test_fast_lane_speed_is_a_warning_and_no_speed_a_note():
    items = team_mission()
    items[4].param2 = 20.0
    assert has(check(items).warnings, '20 m/s')
    items = team_mission()
    del items[4]
    rep = check(build(items))
    assert rep.speed_mps is None and has(rep.notes, 'WPNAV_SPEED')


def test_item0_not_home_is_a_warning():
    items = team_mission()
    items[0].frame = 3
    assert has(check(items).warnings, 'not the HOME line')


def test_renumbered_file_is_a_warning(tmp_path):
    text = open(os.path.join(MISSIONS, 'runway_survey.waypoints'), encoding='utf-8').read()
    lines = text.splitlines()
    lines[3] = '7' + lines[3][1:]                          # wrong seq on one line
    path = write(tmp_path, 'renum.waypoints', '\n'.join(lines) + '\n')
    items, _, renumbered = mc.load_mission(path)
    assert renumbered and [it.seq for it in items] == list(range(12))
    assert has(check(items, renumbered=True).warnings, 'not 0, 1, 2')


# ---- file problems: exit code 3, bad usage: exit code 2 ---------------------------------------------------------

def test_missing_file_exit_3(tmp_path, capsys):
    assert mc.main([str(tmp_path / 'nope.waypoints')]) == 3
    err = capsys.readouterr().err
    assert '[mission_check] ERROR:' in err and 'Likely cause' in err and 'Fix:' in err


@pytest.mark.parametrize('text,fragment', [
    ('', 'empty'),
    ('hello world\n', 'not a mission file'),
    ('QGC WPL 120\n0\t1\t0\t16\t0\t0\t0\t0\t1\t2\t3\t1\n', 'unsupported mission file version'),
    ('QGC WPL 110\n', 'no mission items'),
    ('QGC WPL 110\n0\t1\t0\t16\t0\t0\n', 'expected 12'),
    ('QGC WPL 110\n0\t1\t0\t16\t0\t0\t0\t0\tabc\t2\t3\t1\n', 'malformed'),
    ('{"fileType": "Plan"', 'JSON is broken'),
    ('{"fileType": "Plan"}', 'no "mission" section'),
])
def test_unusable_files_exit_3(tmp_path, capsys, text, fragment):
    path = write(tmp_path, 'bad.waypoints', text)
    assert mc.main([path]) == 3
    assert fragment in capsys.readouterr().err


def test_bom_is_accepted(tmp_path):
    text = open(os.path.join(MISSIONS, 'runway_survey.waypoints'), encoding='utf-8').read()
    p = tmp_path / 'bom.waypoints'
    p.write_bytes(b'\xef\xbb\xbf' + text.encode('utf-8'))
    assert mc.main([str(p), '--quiet']) == 0


def test_utf16_file_from_notepad_is_accepted(tmp_path):
    text = open(os.path.join(MISSIONS, 'runway_survey.waypoints'), encoding='utf-8').read()
    p = tmp_path / 'unicode.waypoints'
    p.write_bytes(text.replace('\n', '\r\n').encode('utf-16'))   # Notepad "Unicode": BOM FF FE + UTF-16 LE
    assert mc.main([str(p), '--quiet']) == 0


def test_bad_home_is_usage_error(capsys):
    assert mc.main([os.path.join(MISSIONS, 'runway_survey.waypoints'), '--home', 'oops']) == 2
    assert 'Fix:' in capsys.readouterr().err


@pytest.mark.parametrize('argv', [[], ['--frob', 'x.waypoints']])
def test_command_line_mistakes_use_the_shared_shape(argv, capsys):
    with pytest.raises(SystemExit) as exc:
        mc.main(argv)
    assert exc.value.code == 2
    err = capsys.readouterr().err
    assert err.startswith('[mission_check] ERROR:') and 'Likely cause' in err and 'Fix:' in err


def test_a_bug_in_the_checker_is_exit_4_not_1(monkeypatch, capsys):
    def boom(*a, **k):
        raise RuntimeError('simulated bug')
    monkeypatch.setattr(mc, 'check_mission', boom)
    assert mc.main([os.path.join(MISSIONS, 'runway_survey.waypoints')]) == 4
    err = capsys.readouterr().err
    assert 'the checker itself failed' in err and 'simulated bug' in err and 'Fix:' in err


def test_problem_mission_exit_1(tmp_path, capsys):
    items = team_mission()[:-1]                             # no RTL
    path = str(tmp_path / 'nortl.waypoints')
    mc.write_waypoints(path, items)
    assert mc.main([path]) == 1
    out = capsys.readouterr().out
    assert 'ERROR: no RETURN_TO_LAUNCH' in out and 'RESULT: NOT READY' in out


def test_json_output(capsys):
    assert mc.main([os.path.join(MISSIONS, 'runway_survey.waypoints'), '--json']) == 0
    data = json.loads(capsys.readouterr().out)
    assert data['ok'] and data['indices']['buffer_wp'] == 9 and data['indices']['rtl_index'] == 11
    assert data['items'][3]['name'] == 'DO_DIGICAM_CONTROL' and 'DigiCamCtrl' in data['items'][3]['role']


# ---- QGroundControl .plan ----------------------------------------------------------------------------------------

def plan_from(items, survey_from=None):
    """A .plan document for the given items (item 0 = home); items[survey_from:-2] go into a survey pattern."""
    def simple(it):
        return {'type': 'SimpleItem', 'command': it.command, 'frame': it.frame, 'autoContinue': bool(it.autocontinue),
                'params': [it.param1, it.param2, it.param3, it.param4 if it.param4 else None, it.lat, it.lon, it.alt]}
    body = [simple(it) for it in items[1:]]
    if survey_from is not None:
        start, stop = survey_from - 1, len(body) - 2
        body = body[:start] + [{'type': 'ComplexItem', 'complexItemType': 'survey',
                                'TransectStyleComplexItem': {'Items': body[start:stop]}}] + body[stop:]
    return {'fileType': 'Plan', 'version': 1, 'groundStation': 'QGroundControl',
            'mission': {'plannedHomePosition': [items[0].lat, items[0].lon, items[0].alt], 'items': body}}


@pytest.mark.parametrize('survey_from', [None, 5])
def test_plan_files_give_the_same_indices(tmp_path, survey_from):
    items = team_mission()
    path = write(tmp_path, 'm.plan', json.dumps(plan_from(items, survey_from)))
    got, kind, _ = mc.load_mission(path)
    assert kind == 'plan' and len(got) == len(items)
    assert [it.command for it in got] == [it.command for it in items]
    rep = check(got, kind=kind)
    assert rep.ok and (rep.summary.buffer_wp, rep.summary.last_before_rtl, rep.summary.rtl_index) == (9, 10, 11)


def test_plan_converts_to_waypoints(tmp_path):
    items = team_mission()
    src = write(tmp_path, 'm.plan', json.dumps(plan_from(items)))
    out = str(tmp_path / 'm.waypoints')
    assert mc.main([src, '--quiet', '--write-waypoints', out]) == 0
    back, kind, renumbered = mc.load_mission(out)
    assert kind == 'waypoints' and not renumbered
    assert [(it.command, it.frame, round(it.lat, 7), round(it.alt, 3)) for it in back] == \
           [(it.command, it.frame, round(it.lat, 7), round(it.alt, 3)) for it in items]


HOME_PLAN = {'plannedHomePosition': [36.2, -96.0, 210]}


@pytest.mark.parametrize('mission,fragment', [
    ({**HOME_PLAN, 'items': [{'type': 'ComplexItem', 'TransectStyleComplexItem': {'Items': [5]}}]},
     'not a mission item'),
    ({**HOME_PLAN, 'items': [{'type': 'ComplexItem', 'TransectStyleComplexItem': [1, 2]}]}, 'not a JSON object'),
    ({**HOME_PLAN, 'items': [{'type': 'ComplexItem', 'TransectStyleComplexItem': {'Items': 7}}]}, 'not a list'),
    ({**HOME_PLAN, 'items': [{'type': 'SimpleItem', 'command': 16, 'params': 5}]}, 'not a list of 7 numbers'),
    ({**HOME_PLAN, 'items': [{'type': 'SimpleItem', 'command': 16, 'params': 'abc'}]}, 'not a list of 7 numbers'),
    ({**HOME_PLAN, 'items': [{'type': 'SimpleItem', 'command': 16.5, 'params': []}]}, 'not a whole number'),
    ({**HOME_PLAN, 'items': [{'type': 'SimpleItem', 'command': 16, 'frame': 1e400, 'params': []}]},
     'not a whole number'),
    ({**HOME_PLAN, 'items': {'a': 1}}, 'is not a list of mission items'),
    ({**HOME_PLAN, 'items': [1, 2]}, 'unknown type'),
    ({'plannedHomePosition': 'x', 'items': []}, 'plannedHomePosition'),
])
def test_malformed_plan_is_exit_3_never_a_traceback(tmp_path, capsys, mission, fragment):
    path = write(tmp_path, 'weird.plan', json.dumps({'fileType': 'Plan', 'mission': mission}))
    assert mc.main([path]) == 3
    err = capsys.readouterr().err
    assert fragment in err and err.startswith('[mission_check] ERROR:') and 'Traceback' not in err


def test_plan_pattern_without_waypoints_exit_3(tmp_path, capsys):
    doc = plan_from(team_mission())
    doc['mission']['items'].insert(2, {'type': 'ComplexItem', 'complexItemType': 'survey', 'polygon': []})
    path = write(tmp_path, 'm.plan', json.dumps(doc))
    assert mc.main([path]) == 3
    assert 'without its generated waypoints' in capsys.readouterr().err


def test_run_as_a_plain_file(tmp_path):
    # The ground-laptop way: python3 .../sim_demo/mission_check.py FILE (no install, no ROS).
    import subprocess
    import sys
    script = os.path.join(os.path.dirname(HERE), 'sim_demo', 'mission_check.py')
    res = subprocess.run([sys.executable, script, os.path.join(MISSIONS, 'runway_survey.waypoints'), '-q'],
                         capture_output=True, text=True, timeout=60)
    assert res.returncode == 0, res.stderr
    assert 'RESULT: OK' in res.stdout
