"""
sim_demo/test/test_flight_recorder.py -- PURPOSE: check the live MAVLink flight recorder and the shared CSV helpers
without a simulator: fake MAVLink messages (objects shaped like pymavlink's) are fed to FlightRecorder and the CSV
files it writes are read back. Covers: telemetry rows on a steady beat with units and empty unknowns, autopilot and
team texts in separate columns with sender system/component, arming time (the FIRST arming is the zero), phases, the
aim text and the servo release in targets_and_drops.csv, chunked MAVLink 2 texts (also a full chunk ending in
spaces), the link-lost event, the 1-second flush and the 5-second fsync, the pid file, the note when demo.sh
disappears (window closed or START OVER), and the program's bad-option exit code. A .tlog replay test runs when
pymavlink is installed.
"""
import csv
import os
import struct

import pytest

from sim_demo import flight_recorder as fr
from sim_demo import run_csv


class Msg:
    """A stand-in for a pymavlink message: get_type / get_srcSystem / get_srcComponent + fields."""

    def __init__(self, kind, sys_id=1, comp=1, **fields):
        self._kind, self._sys, self._comp = kind, sys_id, comp
        self.__dict__.update(fields)

    def get_type(self):
        return self._kind

    def get_srcSystem(self):
        return self._sys

    def get_srcComponent(self):
        return self._comp


def hb(armed=False, mode=3, sys_id=1, comp=1, mav_type=2):
    return Msg('HEARTBEAT', sys_id, comp, type=mav_type, autopilot=3, base_mode=(128 if armed else 0) | 1,
               custom_mode=mode)


def pos(lat=36.2143, lon=-96.0068, alt_m=210.0, rel_m=0.0, hdg_cdeg=9000, vx=300, vy=400, vz=-100):
    return Msg('GLOBAL_POSITION_INT', lat=int(lat * 1e7), lon=int(lon * 1e7), alt=int(alt_m * 1000),
               relative_alt=int(rel_m * 1000), hdg=hdg_cdeg, vx=vx, vy=vy, vz=vz)


def text(t, severity=6, sys_id=1, comp=1, chunk=0):
    return Msg('STATUSTEXT', sys_id, comp, text=t, severity=severity, id=chunk)


def rows(path):
    with open(path, encoding='utf-8', newline='') as f:
        return list(csv.DictReader(f))


@pytest.fixture
def rec(tmp_path):
    r = fr.FlightRecorder(str(tmp_path), rate_hz=2.0, url='udpin:test')
    r.start(1000.0)
    yield r
    r.close(1e9, 'test end')


def test_columns_carry_units_and_unknown_is_empty(rec, tmp_path):
    rec.handle(hb(), 1000.1)
    rec.handle(pos(), 1000.2)
    rec.tick(1000.2)
    rec.close(1000.3)
    tel = rows(os.path.join(tmp_path, 'csv', 'telemetry.csv'))
    assert list(tel[0].keys()) == run_csv.TELEMETRY_COLUMNS
    assert all(any(c.endswith(u) for u in ('_m', '_deg', '_mps', '_v', '_pct', '_s', '_utc')) or
               c in ('flight_mode', 'armed', 'landed_state', 'gps_fix_type', 'gps_fix', 'gps_satellites',
                     'current_wp', 'last_reached_wp', 'mission_phase')
               for c in run_csv.TELEMETRY_COLUMNS)
    r = tel[0]
    assert float(r['lat_deg']) == pytest.approx(36.2143, abs=1e-6)
    assert float(r['alt_msl_m']) == pytest.approx(210.0)
    assert float(r['heading_deg']) == pytest.approx(90.0)
    assert float(r['groundspeed_mps']) == pytest.approx(5.0)     # hypot(3, 4) m/s from cm/s
    assert float(r['climb_rate_mps']) == pytest.approx(1.0)      # vz is positive DOWN
    assert r['flight_mode'] == 'AUTO' and r['armed'] == '0'
    assert r['battery_v'] == '' and r['t_since_arm_s'] == ''     # unknown -> empty, never 0
    assert r['wall_time_utc'].endswith('Z')


def test_telemetry_rate_is_a_steady_beat(rec, tmp_path):
    rec.handle(hb(), 1000.0)
    t = 1000.0
    while t < 1010.0:                                  # 10 s of ticks every 0.1 s, a heartbeat every second
        if abs(t - round(t)) < 0.05:
            rec.handle(hb(), t)
        rec.tick(t)
        t += 0.1
    rec.close(1010.0)
    tel = rows(os.path.join(tmp_path, 'csv', 'telemetry.csv'))
    assert 19 <= len(tel) <= 21                        # 2 rows per second
    times = [float(r['unix_time_s']) for r in tel]
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert min(gaps) >= 0.45 and max(gaps) <= 0.65


def test_autopilot_and_team_texts_are_separate_columns(rec, tmp_path):
    rec.handle(hb(), 1000.0)
    rec.handle(text('ArduCopter V4.6.3 (3fc7011a)'), 1000.5)
    rec.handle(text('Detected tent', sys_id=1, comp=191), 1001.0)          # MAVROS / team node
    rec.handle(text('PreArm: Gyros inconsistent', severity=2), 1001.5)
    rec.handle(text('hello from Mission Planner', sys_id=255, comp=190), 1002.0)
    rec.close(1003.0)
    ev = [r for r in rows(os.path.join(tmp_path, 'csv', 'events.csv')) if r['kind'] == 'statustext']
    assert [r['sender'] for r in ev] == ['autopilot', 'team', 'autopilot', 'gcs']
    assert ev[0]['autopilot_text'].startswith('ArduCopter') and ev[0]['team_text'] == ''
    assert ev[1]['team_text'] == 'Detected tent' and ev[1]['autopilot_text'] == ''
    assert ev[1]['src_component'] == '191'
    assert ev[2]['severity'] == '2' and ev[2]['severity_name'] == 'CRITICAL'
    assert rec.ardupilot_version == '4.6.3'


def test_arming_phases_aim_and_servo_release(tmp_path):
    rec = fr.FlightRecorder(str(tmp_path), rate_hz=2.0)
    rec.start(1000.0)
    rec.handle(hb(armed=False, mode=4), 1000.0)
    rec.handle(Msg('SERVO_OUTPUT_RAW', port=0, servo11_raw=500, servo13_raw=500), 1000.1)
    rec.handle(hb(armed=True, mode=3), 1010.0)                             # armed in AUTO
    rec.handle(pos(rel_m=30.0), 1030.0)
    rec.handle(text('Mission: 3 DigiCamCtrl'), 1031.0)
    rec.handle(text('Aim tent 36.2150261,-96.0067984', comp=191), 1100.0)
    rec.handle(pos(lat=36.21502, lon=-96.00680, rel_m=25.0), 1120.0)
    rec.handle(Msg('SERVO_OUTPUT_RAW', port=0, servo11_raw=500, servo13_raw=1400), 1125.0)
    rec.handle(text('Servo 13 -> 1400', comp=191), 1134.0)                  # the late team text
    rec.handle(Msg('MISSION_ITEM_REACHED', seq=5), 1135.0)
    rec.close(1200.0)
    ev = rows(os.path.join(tmp_path, 'csv', 'events.csv'))
    kinds = [r['kind'] for r in ev]
    assert 'armed' in kinds and 'servo' in kinds and 'wp_reached' in kinds
    armed = next(r for r in ev if r['kind'] == 'armed')
    assert float(armed['unix_time_s']) == pytest.approx(1010.0)
    later = next(r for r in ev if r['kind'] == 'wp_reached')
    assert float(later['t_since_arm_s']) == pytest.approx(125.0)
    ph = [r['phase_code'] for r in rows(os.path.join(tmp_path, 'csv', 'phases.csv'))]
    assert ph[:3] == ['ARMED', 'TAKEOFF', 'SEARCH']
    drops = rows(os.path.join(tmp_path, 'csv', 'targets_and_drops.csv'))
    assert len(drops) == 1
    d = drops[0]
    assert d['payload_class'] == 'tent' and float(d['aim_lat_deg']) == pytest.approx(36.2150261)
    assert d['release_source'] == 'servo_output_raw'                      # the autopilot's own output wins
    assert d['servo_channel'] == '13' and d['servo_open_pwm_us'] == '1400'
    assert float(d['release_t_since_arm_s']) == pytest.approx(115.0)
    assert float(d['release_height_above_home_m']) == pytest.approx(25.0)


def test_release_from_team_text_when_servo_outputs_are_not_streamed(tmp_path):
    rec = fr.FlightRecorder(str(tmp_path))
    rec.handle(hb(armed=True), 1000.0)
    rec.handle(text('Servo 11 -> 1400', comp=191), 1050.0)
    rec.close(1060.0)
    d = rows(os.path.join(tmp_path, 'csv', 'targets_and_drops.csv'))[0]
    assert d['payload_class'] == 'person' and d['release_source'] == 'team_text'


def test_chunked_statustext_is_joined(rec, tmp_path):
    rec.handle(hb(), 1000.0)
    first = 'x' * fr.STATUSTEXT_CHUNK
    rec.handle(text(first, chunk=7, comp=191), 1000.1)
    rec.handle(text('end', chunk=7, comp=191), 1000.2)
    full_with_spaces = 'y' * (fr.STATUSTEXT_CHUNK - 3) + '   '                 # a FULL chunk ending in spaces
    rec.handle(text(full_with_spaces, chunk=8, comp=191), 1000.3)
    rec.handle(text('rest', chunk=8, comp=191), 1000.4)
    rec.close(1001.0)
    ev = [r for r in rows(os.path.join(tmp_path, 'csv', 'events.csv')) if r['kind'] == 'statustext']
    assert [r['team_text'] for r in ev] == [first + 'end', full_with_spaces + 'rest']


def test_t_since_arm_counts_from_the_first_arming(tmp_path):
    rec = fr.FlightRecorder(str(tmp_path))
    rec.handle(hb(armed=True), 1000.0)
    rec.handle(hb(armed=False), 1010.0)
    rec.handle(hb(armed=True), 1020.0)                                     # armed again (e.g. --manual)
    rec.handle(text('later'), 1030.0)
    rec.close(1040.0)
    ev = rows(os.path.join(tmp_path, 'csv', 'events.csv'))
    assert [float(r['t_since_arm_s']) for r in ev if r['kind'] == 'armed'] == [0.0, 20.0]
    assert float(next(r for r in ev if r['team_text'] == 'later' or r['autopilot_text'] == 'later')
                 ['t_since_arm_s']) == pytest.approx(30.0)


def test_meta_name_pid_file_and_why_demo_ended(tmp_path, monkeypatch):
    r = fr.FlightRecorder(str(tmp_path), meta_name='flight_recorder_tlog.json')
    r.start(1.0)
    r.close(2.0)
    assert os.path.isfile(tmp_path / 'flight_recorder_tlog.json') and not os.path.exists(tmp_path / 'flight_recorder.json')
    p = fr._write_pid_file(str(tmp_path), 'udpin:127.0.0.1:14555')
    assert open(p, encoding='utf-8').read().split()[:2] == [str(os.getpid()), 'udpin:127.0.0.1:14555']
    fr._remove_pid_file(p)
    assert not os.path.exists(p)
    home = tmp_path / 'home'
    (home / 'state').mkdir(parents=True)
    monkeypatch.setenv('ASTRA_SIM_HOME', str(home))
    (tmp_path / 'demo.env').write_text('SESSION=demo-1\n', encoding='utf-8')
    assert 'window was closed' in fr._why_parent_ended(str(tmp_path))
    (home / 'state' / 'keep_open').write_text('demo-1\n', encoding='utf-8')
    assert 'handed this simulator to START OVER' in fr._why_parent_ended(str(tmp_path))


def test_link_lost_and_back(rec, tmp_path):
    rec.handle(hb(), 1000.0)
    rec.tick(1001.0)
    rec.tick(1007.0)                                   # 7 s without a heartbeat
    rec.handle(hb(), 1008.0)
    rec.close(1009.0)
    kinds = [r['kind'] for r in rows(os.path.join(tmp_path, 'csv', 'events.csv'))]
    assert kinds.index('link_lost') < kinds.index('link_back')


def test_other_systems_do_not_overwrite_the_vehicle(rec, tmp_path):
    rec.handle(hb(), 1000.0)
    rec.handle(pos(lat=36.0), 1000.1)
    rec.handle(Msg('GLOBAL_POSITION_INT', 2, 1, lat=0, lon=0, alt=0, relative_alt=0, hdg=0, vx=0, vy=0, vz=0), 1000.2)
    rec.handle(hb(sys_id=255, comp=190, mav_type=6), 1000.3)              # a ground station's heartbeat
    assert rec.pos['lat'] == pytest.approx(36.0) and rec.vehicle_sys == 1


def test_live_csv_flushes_every_second(tmp_path):
    clock = {'t': 0.0}
    path = os.path.join(tmp_path, 'a.csv')
    f = run_csv.LiveCsv(path, ['x'], flush_every_s=1.0, clock=lambda: clock['t'])
    f.write({'x': 1})
    assert run_csv.count_rows(path) == 0               # still in the buffer
    clock['t'] = 1.2
    f.write({'x': 2})
    assert run_csv.count_rows(path) == 2               # flushed: safe even if the program dies now
    synced = []
    real = run_csv._fsync
    run_csv._fsync = lambda fh: synced.append(clock['t'])
    try:
        for t in (2.4, 3.6, 5.1, 6.2):                 # flushed every second, put on the disk every 5 s
            clock['t'] = t
            f.write({'x': t})
    finally:
        run_csv._fsync = real
    assert synced == [5.1]
    f.close()
    f.close()                                          # twice is fine


def test_csv_helpers():
    assert run_csv.utc_iso(0.9996) == '1970-01-01T00:00:01.000Z'
    assert run_csv.utc_iso(None) == ''
    assert run_csv.num(float('nan')) == '' and run_csv.num(True) == 1 and run_csv.num(2.34567, 2) == 2.35
    assert run_csv.since(None, 1.0) == '' and run_csv.since(3.5, 1.0) == 2.5
    assert fr.mode_name(6) == 'RTL' and fr.mode_name(99) == 'MODE99'
    assert fr.parse_servo_classes('person:11, tent:13') == {11: 'person', 13: 'tent'}
    with pytest.raises(ValueError):
        fr.parse_servo_classes('tent')


def test_main_bad_options_exit_code(tmp_path, capsys):
    assert fr.main(['--servo', 'tent', '--run-dir', str(tmp_path)]) == fr.EXIT_BAD_ARGS
    out = capsys.readouterr().err
    assert 'ERROR' in out and 'Fix:' in out
    os.environ.pop('ASTRA_RUN_DIR', None)
    assert fr.main([]) == fr.EXIT_BAD_ARGS


def test_tlog_replay(tmp_path):
    """A real MAVLink log written with pymavlink is replayed into the same CSV files."""
    pytest.importorskip('pymavlink')
    os.environ.setdefault('MAVLINK20', '1')
    from pymavlink.dialects.v20 import ardupilotmega as mav
    from sim_demo.csv_export import replay_tlog
    tlog = os.path.join(tmp_path, 'flight.tlog')

    class Sink:
        def __init__(self):
            self.buf = b''

        def write(self, b):
            self.buf += b
    sink = Sink()
    link = mav.MAVLink(sink, srcSystem=1, srcComponent=1)
    with open(tlog, 'wb') as f:
        t = 1793466970.0
        for i in range(20):                            # 10 s at 2 Hz
            for m in (link.heartbeat_encode(2, 3, 129 if i >= 4 else 1, 3, 4),
                      link.global_position_int_encode(i * 500, int(36.2143e7), int(-96.0068e7), 210000,
                                                      i * 1000, 100, 0, -50, 9000)):
                sink.buf = b''
                link.send(m)
                f.write(struct.pack('>Q', int((t + i * 0.5) * 1e6)) + sink.buf)
    replay_tlog(str(tmp_path), tlog)
    tel = rows(os.path.join(tmp_path, 'csv', 'telemetry.csv'))
    assert len(tel) >= 15
    assert tel[-1]['armed'] == '1' and float(tel[-1]['height_above_home_m']) > 15.0
