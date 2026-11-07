"""
sim_demo/test/test_csv_export_summary.py -- PURPOSE: check the CSV exporter (csv_export.py) and the run summary
(run_summary.py) on a small fake run folder shaped exactly like a real one (run_data.json, score_report.json,
events.jsonl, phases.jsonl, photo sidecars, logs). Covers: every CSV is written with the right counts, positions and
times; photos.csv keeps the sea-level height (from telemetry) apart from the camera's WGS-84 height; the live
recorder's files are KEPT, never overwritten; --tlog fills a header-only telemetry.csv, refuses to replace recorded
rows or a log of another day (unless --force) and never touches flight_recorder.json; the landing point in lat/lon
is exact without the world files; the servo channel belongs to the right payload; run_summary.md/.json list the
result, phases and every warning/error with file:line, lines after the run's end apart, the Jetson's power mode and
load, what differs from the field; --refresh keeps the result; runs_index.csv keeps ONE row per run; a partial
summary never replaces a complete one; a run that is still going is shown, never written; the bundle leaves out
photos, huge files and secrets (by name and by content); Windows paths.
"""
import csv
import json
import os
import struct
import time
import zipfile

import pytest

from sim_demo import csv_export, run_summary
from sim_demo.geo import GeoOrigin

ORIGIN = GeoOrigin(36.2143, -96.0068)
T_ARM = 1793466970.22


def _write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)


def rows(path):
    with open(path, encoding='utf-8', newline='') as f:
        return list(csv.DictReader(f))


@pytest.fixture
def run_dir(tmp_path):
    """A finished run like 20261031-101431_runway, shrunk to two photos, two drops."""
    d = os.path.join(tmp_path, 'runs', '20261031-101431_runway')
    tent_land = ORIGIN.to_geo(0.18, 80.86)
    hiker_aim_en = ORIGIN.to_enu(36.2151332, -96.0062724)
    photo_a, photo_b = '36.214931 , -96.008147.jpg', '36.215153 , -96.006822.jpg'
    run_data = {
        'world': 'nowhere_world', 'worlds_dir': '/no/such/dir', 'detector': 'real', 'started': T_ARM,
        'ended': T_ARM + 310.0,
        'frames': [
            {'image_name': photo_a, 'lat': 36.214931, 'lon': -96.008147, 'boxes': [], 'method': 'sahi+yolo',
             'received': T_ARM + 49.0, 'waypoint_index': 4},
            {'image_name': photo_b, 'lat': 36.215153, 'lon': -96.006822, 'method': 'sahi+yolo',
             'received': T_ARM + 98.0, 'waypoint_index': 6,
             'boxes': [{'cls': 'tent', 'score': 0.863, 'cx': 700.5, 'cy': 400.0, 'w': 130.0, 'h': 120.0},
                       {'cls': 'person', 'score': 0.3371, 'cx': 100.0, 'cy': 90.0, 'w': 20.0, 'h': 30.0}]},
        ],
        'photos': [{'name': photo_a, 't': T_ARM + 43.1, 'lat': 36.214931, 'lon': -96.008147,
                    'footprint': [[1, 2], [3, 4], [5, 6]], 'pose_how': 'emulator', 'zoom': 1.0},
                   {'name': photo_b, 't': T_ARM + 97.0, 'lat': 36.215153, 'lon': -96.006822,
                    'footprint': None, 'pose_how': 'none', 'zoom': 1.0}],
        'drops': [{'cls': 'tent', 't': T_ARM + 191.58, 'release_lat': 36.2150259, 'release_lon': -96.0067984,
                   'landing_en': [0.18, 80.86]},
                  {'cls': 'person', 't': T_ARM + 218.49, 'release_lat': 36.2151332, 'release_lon': -96.0062722,
                   'landing_en': [47.4, 92.74]}],
        'aims': [{'cls': 'tent', 't': T_ARM + 164.1, 'lat': 36.2150261, 'lon': -96.0067984},
                 {'cls': 'person', 't': T_ARM + 164.1, 'lat': 36.2151332, 'lon': -96.0062724}],
        'timeline': [], 'notes': [], 'track': [],
    }
    score = {
        'world': 'nowhere_world', 'detector': 'real', 'started': T_ARM, 'ended': T_ARM + 310.0,
        'targets': [{'name': 'tent', 'cls': 'tent', 'lat': 36.21502595, 'lon': -96.00680007, 'detected': True,
                     'detections': 1, 'best_confidence': 0.863, 'aim_error_m': 0.15, 'drop_error_m': 0.19,
                     'drop_source': 'Gazebo landing point', 'first_detected_s': 98.0},
                    {'name': 'hiker', 'cls': 'person', 'lat': 36.21513243, 'lon': -96.00621469, 'detected': True,
                     'detections': 1, 'best_confidence': 0.95, 'aim_error_m': 5.18, 'drop_error_m': 5.16,
                     'drop_source': 'Gazebo landing point', 'first_detected_s': 103.0}],
        'aims': [{'class': 'person', 'lat': 36.2151332, 'lon': -96.0062724, 'time_s': 164.1, 'target': 'hiker',
                  'error_m': 5.18, 'east': round(hiker_aim_en[0], 2), 'north': round(hiker_aim_en[1], 2)}],
        'drops': [{'class': 'tent', 'time_s': 191.6, 'source': 'Gazebo landing point', 'error_m': 0.19,
                   'target': 'tent', 'east': 0.18, 'north': 80.86},
                  {'class': 'person', 'time_s': 218.5, 'source': 'Gazebo landing point', 'error_m': 5.16,
                   'target': 'hiker', 'east': 47.4, 'north': 92.74}],
        'false_detections': [{'photo': photo_b, 'class': 'person', 'confidence': 0.337, 'lat': 0, 'lon': 0}],
        'photos_taken': 2, 'photos_processed': 2, 'targets_found': 2, 'duration_s': 310.0,
    }
    events = [
        {'t': T_ARM - 97.4, 'kind': 'start', 'world': 'runway'},
        {'t': T_ARM - 23.0, 'kind': 'statustext', 'source': 'fcu', 'text': 'ArduCopter V4.6.3 (3fc7011a)',
         'phase': 'Waiting'},
        {'t': T_ARM, 'kind': 'armed', 'mode': 'AUTO'},
        {'t': T_ARM + 30.0, 'kind': 'reached', 'seq': 2, 'phase': 'Searching'},
        {'t': T_ARM + 43.1, 'kind': 'photo', 'name': photo_a},
        {'t': T_ARM + 164.1, 'kind': 'statustext', 'source': 'team', 'text': 'Aim tent 36.2150261,-96.0067984'},
        {'t': T_ARM + 186.0, 'kind': 'statustext', 'source': 'team', 'text': 'Servo 13 -> 1400'},
        {'t': T_ARM + 213.0, 'kind': 'statustext', 'source': 'team', 'text': 'Servo 11 -> 1400'},
    ]
    phases = [{'time': T_ARM - 3.6, 'kind': 'phase', 'phase': 'WAITING', 'text': 'waiting'},
              {'time': T_ARM, 'kind': 'phase', 'phase': 'ARMED', 'text': 'motors armed'},
              {'time': T_ARM + 0.1, 'kind': 'info', 'phase': 'ARMED', 'text': 'flight mode GUIDED -> AUTO'},
              {'time': T_ARM + 41.0, 'kind': 'phase', 'phase': 'SEARCH', 'text': 'lanes'},
              {'time': T_ARM + 310.0, 'kind': 'phase', 'phase': 'LANDED', 'text': 'landed'}]
    _write(os.path.join(d, 'run_data.json'), json.dumps(run_data))
    _write(os.path.join(d, 'score_report.json'), json.dumps(score))
    _write(os.path.join(d, 'events.jsonl'), '\n'.join(json.dumps(e) for e in events) + '\n{"t": 1, "kind"')
    _write(os.path.join(d, 'phases.jsonl'), '\n'.join(json.dumps(p) for p in phases) + '\n')
    _write(os.path.join(d, 'photos', photo_a.replace('.jpg', '.json')), json.dumps(
        {'shutter_unix_s': T_ARM + 42.46, 'position': {'lat': 36.2149306, 'lon': -96.0081469, 'alt_msl_m': 210.62},
         'rel_alt_m': 29.97, 'heading_deg': 349.95, 'camera': {'model': 'ZR10', 'zoom_x': 1.0},
         'gimbal': {'pitch_deg': -90.3}}))
    _write(os.path.join(d, 'photos', photo_a), 'jpeg')
    _write(os.path.join(d, 'demo.env'), 'WORLD=runway\nMISSION=/x/runway_survey.waypoints\nMANUAL=0\n'
                                        'DETECTOR=real\nCAMERA=zr10\nPROFILE=default\nTEAM_STACK=local\n')
    _write(os.path.join(d, 'sim.log'),
           '[INFO] [launch]: fine\n'
           '[pov_server-12] Traceback (most recent call last):\n'
           '[pov_server-12] ValueError: Only supported for TrueType fonts\n'
           '[mavproxy-5] AP: PreArm: Gyros inconsistent\n'
           '[score_run-9] [INFO] [1793467280.5] [score_run]: | tent | Drop error 0.19 m |\n')
    _write(os.path.join(d, 'team_stack.log'),
           '[mavros_node-1] [ERROR] [10:16:31] [mavros.time]: TM: Time jump detected. Resetting.\n'
           '[mavros_node-1] [ERROR] [10:17:22] [mavros.time]: TM: Time jump detected. Resetting.\n'
           '[siyi-4] [WARN] [10:15:54] [siyi]: Could not confirm 5.0x after 3 attempts - continuing.\n'
           '[gpstest-2] [WARN] [10:15:39] [gps]: Waiting for /mavros/set_stream_rate ...\n')
    return d


def test_export_writes_every_csv(run_dir):
    out = csv_export.export_run(run_dir, quiet=True)
    c = os.path.join(run_dir, 'csv')
    assert sorted(os.listdir(c)) == ['detections.csv', 'events.csv', 'phases.csv', 'photos.csv',
                                     'targets_and_drops.csv', 'telemetry.csv']
    assert out['counts'] == {'photos': 2, 'detections': 2}
    dets = rows(os.path.join(c, 'detections.csv'))
    assert [(r['class'], r['false_alarm']) for r in dets] == [('tent', '0'), ('person', '1')]
    assert float(dets[0]['t_since_arm_s']) == pytest.approx(98.0) and dets[0]['box_w_px'] == '130.0'
    photos = rows(os.path.join(c, 'photos.csv'))
    assert photos[0]['camera_model'] == 'ZR10' and float(photos[0]['height_above_home_m']) == pytest.approx(29.97)
    assert float(photos[0]['unix_time_s']) == pytest.approx(T_ARM + 42.46)        # the camera's shutter time
    assert photos[1]['detections'] == '2' and photos[1]['best_class'] == 'tent'
    ph = rows(os.path.join(c, 'phases.csv'))
    assert [r['phase_code'] for r in ph] == ['WAITING', 'ARMED', 'SEARCH', 'LANDED']   # 'info' lines left out
    ev = rows(os.path.join(c, 'events.csv'))
    assert ev[1]['sender'] == 'autopilot' and ev[1]['autopilot_text'].startswith('ArduCopter')
    assert any(r['kind'] == 'wp_reached' for r in ev) and not any(r['kind'] == 'photo' for r in ev)
    assert rows(os.path.join(c, 'telemetry.csv')) == []                            # no positions over time
    assert any('telemetry.csv has only its header' in n for n in out['notes'])


def test_drops_are_complete_and_exact(run_dir):
    csv_export.export_run(run_dir, quiet=True)
    d = {r['payload_class']: r for r in rows(os.path.join(run_dir, 'csv', 'targets_and_drops.csv'))}
    tent, person = d['tent'], d['person']
    assert tent['servo_channel'] == '13' and person['servo_channel'] == '11'     # each payload ITS channel
    lat, lon, _ = ORIGIN.to_geo(0.18, 80.86)
    assert float(tent['landing_lat_deg']) == pytest.approx(lat, abs=2e-7)        # no world files needed
    assert float(tent['landing_lon_deg']) == pytest.approx(lon, abs=2e-7)
    assert tent['truth_target'] == 'tent' and float(tent['drop_error_m']) == 0.19
    assert float(tent['aim_error_m']) == 0.15 and float(person['aim_error_m']) == 5.18
    assert float(tent['aim_t_since_arm_s']) == pytest.approx(164.1)
    assert float(person['release_t_since_arm_s']) == pytest.approx(218.49)


def test_live_files_are_kept(run_dir):
    c = os.path.join(run_dir, 'csv')
    for name, cols in (('telemetry.csv', 'wall_time_utc,lat_deg\nX,1\n'), ('events.csv', 'kind,source\nlive,mavlink\n'),
                       ('phases.csv', 'phase_code,source\nLIVE,mavlink\n')):
        _write(os.path.join(c, name), cols)
    _write(os.path.join(c, 'targets_and_drops.csv'),
           'payload_class,aim_lat_deg,release_wall_time_utc,release_source,servo_channel,servo_open_pwm_us,source\n'
           'tent,36.1,2026-10-31T17:19:15.000Z,servo_output_raw,13,1400,flight_recorder\n')
    out = csv_export.export_run(run_dir, quiet=True)
    assert set(out['kept']) == {'telemetry.csv', 'events.csv', 'phases.csv'}
    assert rows(os.path.join(c, 'phases.csv')) == [{'phase_code': 'LIVE', 'source': 'mavlink'}]
    tent = {r['payload_class']: r for r in rows(os.path.join(c, 'targets_and_drops.csv'))}['tent']
    assert tent['release_source'] == 'servo_output_raw' and tent['aim_lat_deg'] == '36.1'     # live cells kept
    assert tent['landing_east_m'] == '0.18' and tent['source'] == 'flight_recorder+score'     # completed


def test_exporter_rebuilds_its_own_files_but_force_never_empties_telemetry(run_dir):
    csv_export.export_run(run_dir, quiet=True)                                    # builds events.csv from .jsonl
    with open(os.path.join(run_dir, 'events.jsonl'), 'a', encoding='utf-8') as f:  # the scorer adds lines later
        f.write('\n' + json.dumps({'t': T_ARM + 300.0, 'kind': 'disarmed', 'mode': 'LAND'}) + '\n')
    out = csv_export.export_run(run_dir, quiet=True)
    assert 'events.csv' in out['written']                                        # rebuilt: not a recorded file
    assert any(r['kind'] == 'disarmed' for r in rows(os.path.join(run_dir, 'csv', 'events.csv')))
    tel = os.path.join(run_dir, 'csv', 'telemetry.csv')
    _write(tel, 'wall_time_utc,unix_time_s,alt_msl_m\nX,1,2\nY,2,3\n')            # recorded rows
    out = csv_export.export_run(run_dir, force=True, quiet=True)
    assert len(rows(tel)) == 2 and 'telemetry.csv' in out['kept']                 # --force alone never empties it
    assert any('nothing to rebuild it from' in n for n in out['notes'])


def test_photo_heights_sea_level_and_wgs84_apart(run_dir):
    # The camera sidecar's "alt_msl_m" (210.62) is MAVROS' WGS-84 height; sea level comes from MAVLink telemetry.
    tel = os.path.join(run_dir, 'csv', 'telemetry.csv')
    _write(tel, 'unix_time_s,alt_msl_m,height_above_home_m\n'
                f'{T_ARM + 41.5},239.5,29.5\n{T_ARM + 42.0},240.0,30.0\n{T_ARM + 43.0},241.0,31.0\n')
    csv_export.export_run(run_dir, quiet=True)
    p = rows(os.path.join(run_dir, 'csv', 'photos.csv'))
    assert float(p[0]['alt_wgs84_m']) == pytest.approx(210.62)
    assert float(p[0]['alt_msl_m']) == pytest.approx(240.46, abs=0.01)           # interpolated at the shutter time
    assert p[1]['alt_msl_m'] == ''                                              # no telemetry near that photo
    t = csv_export.Track(rows(tel))
    assert t.at(T_ARM + 100.0, 'alt_msl_m') is None and t.at(T_ARM + 42.0, 'alt_msl_m') == pytest.approx(240.0)


def _tlog(path, t0, n=20):
    """A real MAVLink log written with pymavlink: 10 s at 2 Hz, armed from the 5th sample."""
    os.environ.setdefault('MAVLINK20', '1')
    from pymavlink.dialects.v20 import ardupilotmega as mav

    class Sink:
        buf = b''

        def write(self, b):
            self.buf += b
    sink = Sink()
    link = mav.MAVLink(sink, srcSystem=1, srcComponent=1)
    with open(path, 'wb') as f:
        for i in range(n):
            for m in (link.heartbeat_encode(2, 3, 129 if i >= 4 else 1, 3, 4),
                      link.global_position_int_encode(i * 500, int(36.2143e7), int(-96.0068e7), 240000,
                                                      i * 1000, 100, 0, -50, 9000)):
                sink.buf = b''
                link.send(m)
                f.write(struct.pack('>Q', int((t0 + i * 0.5) * 1e6)) + sink.buf)
    return path


def test_tlog_fills_header_only_telemetry_and_refuses_the_rest(run_dir, tmp_path, capsys):
    pytest.importorskip('pymavlink')
    live_meta = os.path.join(run_dir, 'flight_recorder.json')
    _write(live_meta, '{"url": "udpin:127.0.0.1:14555"}')
    csv_export.export_run(run_dir, quiet=True)                                   # header-only telemetry.csv
    tlog = _tlog(str(tmp_path / 'same_flight.tlog'), T_ARM - 2.0)
    out = csv_export.export_run(run_dir, tlog=tlog, quiet=True)                  # was silently ignored before
    tel = rows(os.path.join(run_dir, 'csv', 'telemetry.csv'))
    assert len(tel) >= 15 and set(csv_export.LIVE_FILES) <= set(out['written']) and not out['kept']
    assert open(live_meta, encoding='utf-8').read() == '{"url": "udpin:127.0.0.1:14555"}'   # never touched
    assert os.path.isfile(os.path.join(run_dir, csv_export.TLOG_META))
    with pytest.raises(csv_export.ExportRefused):                               # recorded rows: refused ...
        csv_export.export_run(run_dir, tlog=tlog, quiet=True)
    assert csv_export.main([run_dir, '--tlog', tlog]) == csv_export.EXIT_FAILED
    e = capsys.readouterr().err
    assert '[export_csv] ERROR' in e and 'add --force' in e
    assert csv_export.export_run(run_dir, tlog=tlog, force=True, quiet=True)['written']   # ... unless --force


def test_tlog_of_the_next_run_is_refused(run_dir, tmp_path):
    # Runs follow each other minutes apart: the NEXT run's log (starting 20 min after this run's end) is not this one.
    pytest.importorskip('pymavlink')
    tlog = _tlog(str(tmp_path / 'next_run.tlog'), T_ARM + 310.0 + 20 * 60.0)
    with pytest.raises(csv_export.ExportRefused):
        csv_export.export_run(run_dir, tlog=tlog, quiet=True)


def test_tlog_of_another_day_is_refused(run_dir, tmp_path):
    pytest.importorskip('pymavlink')
    tlog = _tlog(str(tmp_path / 'other_day.tlog'), T_ARM - 2 * 86400.0)
    with pytest.raises(csv_export.ExportRefused) as e:
        csv_export.export_run(run_dir, tlog=tlog, quiet=True)
    assert 'not the log of this flight' in e.value.what
    assert not os.path.isfile(os.path.join(run_dir, 'csv', 'telemetry.csv'))     # nothing was written
    assert not os.path.isdir(os.path.join(run_dir, 'csv', '.tlog_replay'))       # the temporary folder is gone
    csv_export.export_run(run_dir, tlog=tlog, force=True, quiet=True)             # --force: "I am sure"
    assert len(rows(os.path.join(run_dir, 'csv', 'telemetry.csv'))) >= 15


def test_export_without_scorer_uses_events_and_photo_folder(tmp_path):
    d = os.path.join(tmp_path, 'crashed')
    _write(os.path.join(d, 'events.jsonl'), json.dumps(
        {'t': 5.0, 'kind': 'detection', 'image': 'a.jpg', 'lat': 1.0, 'lon': 2.0, 'method': 'x',
         'boxes': [{'cls': 'tent', 'score': 0.5, 'cx': 1, 'cy': 2, 'w': 3, 'h': 4}]}) + '\n')
    _write(os.path.join(d, 'photos', '36.1 , -96.2.jpg'), 'jpeg')
    out = csv_export.export_run(d, quiet=True)
    assert out['counts'] == {'photos': 1, 'detections': 1}
    p = rows(os.path.join(d, 'csv', 'photos.csv'))[0]
    assert p['lat_deg'] == '36.1' and p['source'] == 'photos folder'
    assert rows(os.path.join(d, 'csv', 'detections.csv'))[0]['false_alarm'] == ''   # unknown without a score


def test_exporter_main_errors(tmp_path, capsys):
    assert csv_export.main([str(tmp_path / 'nope')]) == csv_export.EXIT_FAILED
    e = capsys.readouterr().err
    assert '[export_csv] ERROR' in e and 'Fix:' in e
    assert csv_export.main(['a', 'b', '--tlog', 'x.tlog']) == csv_export.EXIT_USAGE


def test_summary_lists_result_phases_and_problems(run_dir):
    s = run_summary.finish_run(run_dir, result='landed: mission complete', exit_code=0)
    md = open(os.path.join(run_dir, 'run_summary.md'), encoding='utf-8').read()
    js = json.load(open(os.path.join(run_dir, 'run_summary.json'), encoding='utf-8'))
    assert js['result'] == 'landed: mission complete' and js['partial'] is False
    assert js['ardupilot']['version'] == '4.6.3'
    assert [p['code'] for p in js['phases']] == ['WAITING', 'ARMED', 'SEARCH', 'LANDED']
    assert js['phases'][2]['duration_s'] == pytest.approx(269.0)
    assert '`sim.log:2`' in md and '`team_stack.log:1`' in md                     # file:line of the first one
    grouped = [g for g in js['problems']['groups'] if 'Time jump' in g['text']]
    assert len(grouped) == 1 and grouped[0]['count'] == 2 and grouped[0]['time'] == '10:16:31'
    assert js['problems']['normal'] == 1          # the zoom readback warning (a PreArm text alone is no warning)
    assert not any('Drop error' in g['text'] for g in js['problems']['groups'])  # "error" in a report is not one
    assert 'tent | tent | yes' in md and s['detector']['kind'] == 'real'
    assert js['git']['commit'] == '' and 'not recorded' in js['git']['how']      # summarised after the run


def test_runs_index_has_one_row_per_run(run_dir):
    run_summary.finish_run(run_dir, result='landed')
    run_summary.finish_run(run_dir, result='landed again')
    idx = rows(os.path.join(os.path.dirname(run_dir), 'runs_index.csv'))
    assert len(idx) == 1 and idx[0]['result'] == 'landed again'
    assert idx[0]['tent_drop_error_m'] == '0.19' and idx[0]['hiker_aim_error_m'] == '5.18'
    assert idx[0]['ardupilot_version'] == '4.6.3' and idx[0]['targets_found'] == '2'


def test_partial_never_replaces_complete(run_dir):
    run_summary.finish_run(run_dir, result='landed')
    run_summary.finish_run(run_dir, partial=True, note='late recorder')
    js = json.load(open(os.path.join(run_dir, 'run_summary.json'), encoding='utf-8'))
    assert js['partial'] is False and js['result'] == 'landed'


def test_partial_summary_of_a_crashed_run(tmp_path):
    d = os.path.join(tmp_path, 'runs', 'crash')
    _write(os.path.join(d, 'sim.log'), '[ERROR] [gz_gui-2]: process has died [pid 1, exit code -6]\n')
    s = run_summary.finish_run(d, partial=True, note='window closed')
    md = open(os.path.join(d, 'run_summary.md'), encoding='utf-8').read()
    assert 'PARTIAL SUMMARY' in md and 'window closed' in md and s['problems']['errors'] == 1
    assert s['result'].startswith('unknown')


def test_bundle_leaves_out_photos_big_files_and_secrets(run_dir, tmp_path):
    _write(os.path.join(run_dir, 'lattice', 'api_token.txt'), 'secret')
    _write(os.path.join(run_dir, 'big.bin'), 'x' * 3000)
    _write(os.path.join(run_dir, 'jetson', 'id_ed25519'), 'key')                       # an SSH key by its name
    _write(os.path.join(run_dir, 'jetson', 'notes.txt'), 'ok\n-----BEGIN OPENSSH PRIVATE KEY-----\nabc\n')
    _write(os.path.join(run_dir, 'jetson', 'setup.log'), 'user=astra\npassword = hunter22x\n')
    out, inc, skipped = run_summary.bundle(run_dir, str(tmp_path / 'b.zip'), max_mb=0.002, note='test note')
    z = zipfile.ZipFile(out)
    names = z.namelist()
    assert any(n.endswith('sim.log') for n in names) and any(n.endswith('BUNDLE_MANIFEST.txt') for n in names)
    assert not any('/photos/' in n or 'api_token' in n or n.endswith('big.bin') for n in names)
    assert not any(n.endswith(('id_ed25519', 'notes.txt', 'setup.log')) for n in names)
    why = dict(skipped)
    assert 'photos/' in why and 'secret' in why[os.path.join('lattice', 'api_token.txt')]
    assert why[os.path.join('jetson', 'notes.txt')].startswith('line 2 ')                 # found by its content
    manifest = next(n for n in names if n.endswith('BUNDLE_MANIFEST.txt'))
    assert 'NOTE: test note' in z.read(manifest).decode()


def test_logs_after_the_end_are_apart_and_refresh_keeps_the_result(run_dir):
    end = time.mktime(time.strptime('2026-10-31 10:20:00', '%Y-%m-%d %H:%M:%S'))
    _write(os.path.join(run_dir, 'sim.log'),
           '2026-10-31 10:15:00 [gz-1] [ERROR] [x]: a real problem during the flight\n'
           '2026-10-31 10:25:00 [score_run-9] [ERROR] [score_run]: process has died [pid 9, exit code -2]\n'
           '2026-10-31 10:25:01 [launch]: Traceback (most recent call last):\n')
    s = run_summary.finish_run(run_dir, result='landed: mission complete', exit_code=0, logs_until=end)
    # 1 in sim.log during the flight + the 2 "Time jump" lines of team_stack.log (no date: always counted)
    assert s['problems']['errors'] == 3 and s['problems']['after_end']['errors'] == 2
    md = open(os.path.join(run_dir, 'run_summary.md'), encoding='utf-8').read()
    assert 'After the run ended (10:20:00)' in md and 'a real problem' in md
    s2 = run_summary.finish_run(run_dir, refresh=True)                          # e.g. after the simulator stopped
    assert s2['result'] == 'landed: mission complete' and s2['exit_code'] == 0 and s2['logs_until'] == end
    assert s2['problems']['after_end']['errors'] == 2


def test_groups_are_capped_per_log_file():
    hits = [{'file': 'a.log', 'line': i, 'level': 'error', 'time': '', 'text': f'error kind {chr(65 + i % 26)}{i // 26}x',
             'normal': '', 'after_end': False} for i in range(60)]
    hits += [{'file': 'team_stack.log', 'line': 1, 'level': 'error', 'time': '', 'text': 'the team problem',
              'normal': '', 'after_end': False}]
    groups = run_summary.group_hits(hits, per_file=10)
    assert sum(g['file'] == 'a.log' for g in groups) == 10 and any(g['file'] == 'team_stack.log' for g in groups)


def test_running_run_is_shown_never_written(run_dir, capsys):
    _write(os.path.join(run_dir, 'demo.log'), 'just now\n')                     # a log that changed seconds ago
    assert 'demo.log changed' in run_summary.run_in_progress(run_dir)
    assert run_summary.main(['show', run_dir]) == 0
    assert 'RUN IN PROGRESS' in capsys.readouterr().out
    assert not os.path.exists(os.path.join(run_dir, 'run_summary.md'))
    assert not os.path.exists(os.path.join(os.path.dirname(run_dir), 'runs_index.csv'))
    assert run_summary.main(['running', run_dir]) == 0
    old = time.time() - 3600
    for n in os.listdir(run_dir):
        if os.path.isfile(os.path.join(run_dir, n)):
            os.utime(os.path.join(run_dir, n), (old, old))
    assert run_summary.run_in_progress(run_dir) == ''
    assert run_summary.main(['show', run_dir]) == 0                             # finished: written now
    md = open(os.path.join(run_dir, 'run_summary.md'), encoding='utf-8').read()
    assert 'PARTIAL SUMMARY' in md and 'never wrote one' in md and 'crashed' not in md


def test_jetson_power_load_and_differences_from_the_field(run_dir):
    _write(os.path.join(run_dir, 'jetson', 'facts.env'), 'NVPMODEL=MAXN_SUPER\nJETSON_CLOCKS=off\nL4T=R36.4.7\n')
    _write(os.path.join(run_dir, 'jetson', 'tegrastats.log'),
           '10-31-2026 11:22:33 RAM 2950/7620MB (lfb 2x4MB) SWAP 0/3810MB CPU [12%@1510,8%@1510,off,20%@1510] '
           'GR3D_FREQ 40% cpu@48.2C gpu@47.4C tj@49.0C VDD_IN 6123mW/6123mW\n'
           '10-31-2026 11:22:34 RAM 3100/7620MB CPU [30%@1510,30%@1510] GR3D_FREQ 80% cpu@50.5C gpu@51.0C tj@52.0C\n')
    with open(os.path.join(run_dir, 'demo.env'), 'a', encoding='utf-8') as f:
        f.write('JETSON_HOST=192.168.55.1\nJETSON_LINK=usb\nTEAM_STACK=jetson\n')
    s = run_summary.build_summary(run_dir)
    j = s['jetson']
    assert j['power'] == {'NVPMODEL': 'MAXN_SUPER', 'JETSON_CLOCKS': 'off'} and j['facts']['L4T'] == 'R36.4.7'
    tg = j['tegrastats']
    assert tg['samples'] == 2 and tg['gpu_load_pct'] == {'avg': 60.0, 'max': 80.0} and tg['ram_used_mb_max'] == 3100
    assert tg['temp_max_c']['tj'] == 52.0
    assert s['team_stack']['where'] == 'jetson (192.168.55.1, usb)'
    md = run_summary.render_markdown(s)
    assert '## The Jetson' in md and 'NVPMODEL=MAXN_SUPER' in md and '## What still differs from the field' in md
    assert any('on the Jetson' in i for i in s['differences_from_field']['items'])
    _write(os.path.join(run_dir, 'jetson', 'differences.txt'), '# from the Jetson stream\n- the camera is simulated\n')
    assert run_summary.build_summary(run_dir)['differences_from_field']['items'] == ['the camera is simulated']


def test_jetson_tools_files_status_json_link_state_and_their_differences(run_dir):
    # The names the Jetson tools really write (simulation/jetson/pc/collect.sh, hil_pc.py summary).
    jd = os.path.join(run_dir, 'jetson')
    _write(os.path.join(jd, 'status_start.json'), json.dumps({'power_mode': 'MAXN_SUPER', 'clocks': 'off',
                                                              'l4t': 'R36.4.7', 'image': 'astra-team:jetson'}))
    _write(os.path.join(jd, 'status_end.json'), json.dumps({'power_mode': 'MAXN_SUPER', 'clocks': 'off', 'l4t': 'x'}))
    _write(os.path.join(jd, 'link_state.env'), 'LINK=ssh\nHOST=jetson.local\nA=up\n')
    _write(os.path.join(jd, 'summary.md'), '# Jetson\n\n## Link\n\n- ssh\n\n## Differences from the field\n\n'
                                           '- the camera pictures come from Gazebo\n- MAVLink goes over SSH, not TELEM2\n')
    s = run_summary.build_summary(run_dir)
    assert s['jetson']['power'] == {'power_mode': 'MAXN_SUPER', 'clocks': 'off', 'power_mode (end)': 'MAXN_SUPER',
                                    'clocks (end)': 'off'}
    assert s['jetson']['facts']['l4t'] == 'R36.4.7' and 'l4t (end)' not in s['jetson']['facts']
    assert s['jetson']['host'] == 'jetson.local' and s['jetson']['link'] == 'ssh'
    assert s['differences_from_field']['items'] == ['the camera pictures come from Gazebo',
                                                    'MAVLink goes over SSH, not TELEM2']
    assert 'summary.md' in s['differences_from_field']['source']


def test_pc_run_lists_what_differs_from_the_field(run_dir):
    s = run_summary.build_summary(run_dir)
    assert s['jetson'] == {} and any('this PC' in i for i in s['differences_from_field']['items'])
    assert any('backend was not recorded' in i for i in s['differences_from_field']['items'])
    _write(os.path.join(run_dir, 'run_meta', 'detector.json'), json.dumps(
        {'kind': 'real', 'model': {'name': 'yolo26m.pt', 'format': 'pytorch'},
         'check': {'load': {'device': 'cuda:0', 'gpu': 'NVIDIA GeForce RTX 4060 Laptop GPU'}}}))
    s = run_summary.build_summary(run_dir)
    assert s['detector']['backend'] == 'PyTorch on CUDA (NVIDIA GeForce RTX 4060 Laptop GPU)'
    assert any('not TensorRT' in i for i in s['differences_from_field']['items'])
    assert 'backend PyTorch on CUDA' in run_summary.render_markdown(s)
    assert run_summary.detection_backend({'backend': 'tensorrt 10.3'}) == 'tensorrt 10.3'
    names = [f['name'] for f in s['files']]
    assert 'run_summary.md' in names and 'run_summary.json' in names              # listed although written after


def test_start_over_run_takes_missing_settings_from_the_run_it_continued(run_dir, tmp_path):
    new = os.path.join(os.path.dirname(run_dir), '20261031-103000_runway')
    _write(os.path.join(new, 'start_over.env'), f'SESSION=demo-x\nWORLD=runway\nRUN_DIR={new}\n'
                                                f'PREVIOUS_RUN={run_dir}\nFLY=1\n')
    env = run_summary.run_env(new)
    assert env['DETECTOR'] == 'real' and env['CAMERA'] == 'zr10' and env['RUN_DIR'] == new
    s = run_summary.build_summary(new)
    assert s['previous_run'] == run_dir and s['detector']['kind'] == 'real'
    assert 'started by START OVER after `20261031-101431_runway`' in run_summary.render_markdown(s)


def test_commit_recorded_at_the_start_is_used_and_git_never_blocks(run_dir, monkeypatch, tmp_path):
    _write(os.path.join(run_dir, 'run_meta', 'source.json'),
           json.dumps({'repo': '/mnt/d/repo', 'commit': 'abc1234def', 'branch': 'gazebo-sim', 'dirty': True,
                       'changed_files': 3, 'how': 'git in /mnt/d/repo'}))
    s = run_summary.build_summary(run_dir, live=True)                         # no git call at the end of the run
    assert s['git']['commit'] == 'abc1234def' and 'run_meta/source.json' in s['git']['how']
    assert run_summary.build_summary(run_dir)['git']['commit'] == 'abc1234def'   # also when summarised later
    monkeypatch.setenv('ASTRA_WINDOWS_REPO', 'D:\\Projects ext\\repo')
    assert run_summary._windows_repo() == '/mnt/d/Projects ext/repo'


def test_windows_paths_and_times():
    assert run_summary.windows_path('/mnt/d/Projects ext/x') == 'D:\\Projects ext\\x'
    assert run_summary.windows_path('/home/me/.astra_sim/runs/r', distro='Ubuntu-22.04') == \
        '\\\\wsl.localhost\\Ubuntu-22.04\\home\\me\\.astra_sim\\runs\\r'
    assert run_summary.mmss(-3.6) == '-0:04' and run_summary.mmss(75.4) == '1:15' and run_summary.mmss(None) == '-'


def test_list_runs_and_cli(run_dir, capsys):
    run_summary.finish_run(run_dir, result='landed')
    runs = run_summary.list_runs(os.path.dirname(run_dir))
    assert runs[0]['run'] == '20261031-101431_runway' and runs[0]['targets'] == '2/2'
    assert run_summary.main(['errors', run_dir]) == 0
    out = capsys.readouterr().out
    assert 'ERROR sim.log:2' in out and 'PreArm' not in out
    assert run_summary.main(['show', os.path.join(run_dir, 'nope')]) == run_summary.EXIT_FAILED


def test_serious_status_texts_are_grouped(run_dir):
    hdr = ','.join(['wall_time_utc', 'unix_time_s', 't_since_arm_s', 'kind', 'src_system', 'src_component', 'sender',
                    'severity', 'severity_name', 'autopilot_text', 'team_text', 'detail', 'mission_phase', 'source'])
    lines = [hdr] + [f'2026-10-31T18:03:0{i}.000Z,{1793469780 + i},,statustext,1,1,autopilot,2,CRITICAL,'
                     'PreArm: Gyros inconsistent,,,WAITING,mavlink' for i in range(3)]
    lines.append('2026-10-31T18:09:00.000Z,1793470140,300,statustext,1,191,team,3,ERROR,,Servo 13 FAILED,,DROP,mavlink')
    lines.append('2026-10-31T18:09:01.000Z,1793470141,301,statustext,1,1,autopilot,6,INFO,Land complete,,,RTL,mavlink')
    _write(os.path.join(run_dir, 'csv', 'events.csv'), '\n'.join(lines) + '\n')
    s = run_summary.build_summary(run_dir)
    texts = {m['text']: m for m in s['serious_status_texts']}
    assert set(texts) == {'PreArm: Gyros inconsistent', 'Servo 13 FAILED'}       # INFO is not serious
    assert texts['PreArm: Gyros inconsistent']['count'] == 3 and texts['PreArm: Gyros inconsistent']['normal']
    assert texts['Servo 13 FAILED']['sender'] == 'team' and not texts['Servo 13 FAILED']['normal']
