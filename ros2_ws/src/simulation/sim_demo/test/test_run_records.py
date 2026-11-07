"""
sim_demo/test/test_run_records.py -- PURPOSE: check run_records.py, the piece that starts, stops and finishes the
records of one run for demo.sh and for START OVER: the recorder's MAVLink address is found through the runs START
OVER continued; a busy UDP port is detected (no second recorder that would steal the first one's packets); the START
OVER hook writes the OLD run's summary again (keeping its result), copies run_meta into the new run and starts the
NEW run's recorder; the run watcher's hook writes the new run's summary with the right result; a real flight
recorder process is started, found through its pid file and stopped politely (Linux only: WSL / the simulator).
"""
import json
import os
import socket
import sys
import time

import pytest

from sim_demo import run_records, run_summary

LINUX = sys.platform.startswith('linux')


def _write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text)


def _free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def runs(tmp_path):
    """An old demo.sh run (with a recorder and run_meta) and the new folder START OVER made after it."""
    old = os.path.join(tmp_path, 'runs', '20261031-110000_runway')
    new = os.path.join(tmp_path, 'runs', '20261031-111500_runway')
    _write(os.path.join(old, 'demo.env'), 'SESSION=demo-1\nWORLD=runway\nDETECTOR=real\nCAMERA=zr10\n'
                                          'RECORDER=udpin:127.0.0.1:14555\nMISSION=/m.waypoints\n')
    _write(os.path.join(old, 'run_meta', 'autopilot.json'), json.dumps({'reported': {'version': '4.6.3'}}))
    _write(os.path.join(old, 'phases.jsonl'), json.dumps({'time': 1.0, 'kind': 'phase', 'phase': 'ARMED'}) + '\n')
    _write(os.path.join(new, 'start_over.env'), f'SESSION=demo-1\nWORLD=runway\nRUN_DIR={new}\nPREVIOUS_RUN={old}\n')
    return old, new


def test_recorder_url_follows_the_runs_start_over_continued(runs):
    old, new = runs
    assert run_records.recorder_url(old) == 'udpin:127.0.0.1:14555'
    assert run_records.recorder_url(new) == 'udpin:127.0.0.1:14555'          # inherited through PREVIOUS_RUN
    _write(os.path.join(old, 'demo.env'), 'SESSION=demo-1\nRECORDER=\n')         # demo.sh --no-recorder
    assert run_records.recorder_url(new) == ''


def test_busy_port_is_detected():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)                   # like pymavlink's udpin
    s.bind(('127.0.0.1', 0))
    try:
        assert not run_records.port_free(f'udpin:127.0.0.1:{s.getsockname()[1]}')
    finally:
        s.close()
    assert run_records.port_free(f'udpin:127.0.0.1:{_free_udp_port()}') and run_records.port_free('tcp:x:1')


def test_start_recorder_without_url_or_on_a_busy_port_records_nothing(runs):
    _old, new = runs
    pid, text = run_records.start_recorder(new, '')
    assert pid is None and '--no-recorder' in text
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    try:
        pid, text = run_records.start_recorder(new, f'udpin:127.0.0.1:{s.getsockname()[1]}')
    finally:
        s.close()
    assert pid is None and text.startswith('WARNING') and 'Fix:' in text


def test_start_over_hook_refreshes_the_old_run_and_keeps_its_result(runs, monkeypatch):
    old, new = runs
    run_summary.finish_run(old, result='ended by START OVER (a new run started: x)', exit_code=0)
    started = {}
    monkeypatch.setattr(run_records, 'start_recorder',
                        lambda run, url, mission='', env=None, **kw: (started.update(run=run, url=url) or 4242, 'ok'))
    lines = run_records.start_over_hook(new, old, '/m.waypoints')
    js = json.load(open(os.path.join(old, 'run_summary.json'), encoding='utf-8'))
    assert js['result'].startswith('ended by START OVER') and not js['partial']
    assert started == {'run': new, 'url': 'udpin:127.0.0.1:14555'}
    assert ('info', 'ok') in lines
    meta = json.load(open(os.path.join(new, 'run_meta', 'autopilot.json'), encoding='utf-8'))
    assert meta['reported']['version'] == '4.6.3' and meta['inherited_from'] == old
    assert 'RECORDER=udpin:127.0.0.1:14555' in open(os.path.join(new, 'start_over.env'), encoding='utf-8').read()


def test_start_over_hook_writes_a_summary_for_an_old_run_without_one(runs, monkeypatch):
    old, new = runs
    monkeypatch.setattr(run_records, 'start_recorder', lambda *a, **kw: (None, 'no live CSV for this run: x'))
    run_records.start_over_hook(new, old)
    js = json.load(open(os.path.join(old, 'run_summary.json'), encoding='utf-8'))
    assert js['result'] == 'ended by START OVER (a new run started: 20261031-111500_runway)'


def test_watcher_end_hook_results(runs):
    _old, new = runs
    lines = run_records.watcher_end_hook(new, 0, 'the drone has landed')
    js = json.load(open(os.path.join(new, 'run_summary.json'), encoding='utf-8'))
    assert js['result'] == 'landed: mission complete' and js['exit_code'] == 0 and 'run summary:' in lines[0]
    run_records.watcher_end_hook(new, 130, 'a new START OVER began', new_start_over=True)
    js = json.load(open(os.path.join(new, 'run_summary.json'), encoding='utf-8'))
    assert js['result'] == 'ended by START OVER (a new run started)'


def test_cli_errors(tmp_path, capsys):
    assert run_records.main(['stop', str(tmp_path / 'nope')]) == run_records.EXIT_FAILED
    e = capsys.readouterr().err
    assert '[run_records] ERROR' in e and 'Fix:' in e


@pytest.mark.skipif(not LINUX, reason='starts and signals a real process (Linux /proc): runs in WSL')
def test_real_recorder_starts_writes_its_pid_file_and_stops(runs):
    pytest.importorskip('pymavlink')
    _old, new = runs
    url = f'udpin:127.0.0.1:{_free_udp_port()}'
    pid, text = run_records.start_recorder(new, url, wait_s=2.0)
    assert pid, text
    for _ in range(50):
        if run_records.recorder_pid(new) == pid:
            break
        time.sleep(0.1)
    assert run_records.recorder_pid(new) == pid
    assert 'still recording' in run_summary.run_in_progress(new)
    assert not run_records.port_free(url)                                    # a second one would be refused
    assert run_records.stop_recorder(new) == 'stopped'
    assert run_records.recorder_pid(new) is None and not os.path.exists(os.path.join(new, 'flight_recorder.pid'))
    log = open(os.path.join(new, 'flight_recorder.log'), encoding='utf-8').read()
    assert 'stopped (stopped by signal SIGINT)' in log
    assert os.path.isfile(os.path.join(new, 'csv', 'telemetry.csv'))
