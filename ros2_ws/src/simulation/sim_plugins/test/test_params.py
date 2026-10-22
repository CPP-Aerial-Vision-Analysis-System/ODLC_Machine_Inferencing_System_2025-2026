"""
sim_plugins/test/test_params.py -- PURPOSE: unit tests of the real-drone parameter file support (params.py):
Mission Planner / QGroundControl / ArduPilot formats are read, the "safe" filter drops exactly the hardware
and airframe lines, `none` and param_keep work, real-drone payload-servo problems are warned about, and the
SITL defaults file has short lines (ArduPilot reads defaults files in 100-byte pieces).
"""

from pathlib import Path

from sim_plugins import params

EXAMPLE = Path(__file__).resolve().parents[5] / 'simulation' / 'profiles' / 'params' / 'example_real_drone.param'


def test_parse_mission_planner_qgc_and_parm_formats():
    text = ('# comment\nWPNAV_SPEED,500\nRTL_ALT 3000   # trailing comment\n'
            '1\t1\tLAND_SPEED\t50\t9\n'          # QGroundControl .params
            'wpnav_radius=200\n'                 # lower case and '=' are accepted
            'BROKEN_LINE\nNAME,notanumber\n\nWPNAV_SPEED,450\n')
    lines, bad = params.parse_param_text(text)
    values = {p.name: p.value for p in lines}
    assert values == {'WPNAV_SPEED': '450', 'RTL_ALT': '3000', 'LAND_SPEED': '50', 'WPNAV_RADIUS': '200'}
    assert [b[0] for b in bad] == [6, 7]                 # line numbers of the unreadable lines


def test_safe_filter_drops_hardware_and_keeps_mission():
    for keep in ('WPNAV_SPEED', 'RTL_ALT', 'LAND_SPEED', 'FS_THR_ENABLE', 'LOIT_SPEED', 'ARMING_CHECK'):
        assert params.drop_reason(keep) == '', keep
    for drop in ('INS_ACCOFFS_X', 'COMPASS_OFS_X', 'SERIAL1_BAUD', 'BATT_MONITOR', 'FRAME_CLASS', 'MOT_SPIN_ARM',
                 'ATC_RAT_RLL_P', 'SERVO1_FUNCTION', 'SERVO11_FUNCTION', 'SYSID_THISMAV', 'STAT_FLTTIME',
                 'SIM_SPEEDUP', 'GPS_TYPE', 'RC1_MIN', 'SR1_POSITION', 'MNT1_TYPE', 'FENCE_ENABLE'):
        assert params.drop_reason(drop) != '', drop


def test_filter_modes_and_keep():
    lines, _ = params.parse_param_text('ATC_RAT_RLL_P,0.1\nWPNAV_SPEED,500\n')
    kept, dropped = params.filter_params(lines, 'safe')
    assert [p.name for p in kept] == ['WPNAV_SPEED'] and dropped[0][0] == 'ATC_RAT_RLL_P'
    kept, _ = params.filter_params(lines, 'safe', keep=['ATC_RAT_RLL_P'])
    assert [p.name for p in kept] == ['ATC_RAT_RLL_P', 'WPNAV_SPEED']
    kept, dropped = params.filter_params(lines, 'none')
    assert len(kept) == 2 and dropped == []


def test_real_drone_payload_servo_warning():
    lines, _ = params.parse_param_text('SERVO11_FUNCTION,1\nSERVO13_FUNCTION,0\n')
    warns = params.real_drone_warnings(lines)
    assert len(warns) == 1 and 'SERVO11_FUNCTION' in warns[0] and 'simulator keeps 0' in warns[0]
    warns = params.real_drone_warnings(lines, mode='none')
    assert 'simulated drop fails too' in warns[0]


def test_prepare_example_file(tmp_path):
    report = params.prepare_param_file(str(EXAMPLE), tmp_path)
    kept = {p.name for p in report.kept}
    assert {'WPNAV_SPEED', 'RTL_ALT', 'FS_BATT_ENABLE'} <= kept
    assert not kept & {'INS_ACCOFFS_X', 'FRAME_CLASS', 'SERVO11_FUNCTION', 'SERIAL1_BAUD'}
    assert report.bad == []
    out = Path(report.output)
    assert out.is_file() and out.name.endswith('.sim.parm') and ' ' not in out.name
    lines = out.read_text().splitlines()
    assert all(len(line) < 90 for line in lines)
    assert 'WPNAV_SPEED 500' in lines
    assert 'parameter(s) used' in report.summary()


def test_prepare_missing_or_empty_file(tmp_path):
    report = params.prepare_param_file(str(tmp_path / 'nope.param'), tmp_path)
    assert report.output == '' and 'cannot read' in report.warnings[0]
    empty = tmp_path / 'empty.param'
    empty.write_text('# nothing\n')
    report = params.prepare_param_file(str(empty), tmp_path, write=False)
    assert report.output == '' and 'no "NAME VALUE" lines' in report.warnings[0]


def test_output_names_do_not_clash():
    a = params.output_name('/a/drone.param')
    b = params.output_name('/b/drone.param')
    assert a != b and a.startswith('drone_') and b.startswith('drone_')
