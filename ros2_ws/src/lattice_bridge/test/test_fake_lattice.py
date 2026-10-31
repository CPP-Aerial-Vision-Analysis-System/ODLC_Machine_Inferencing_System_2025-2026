"""
lattice_bridge/test/test_fake_lattice.py -- PURPOSE: prove the pretend Lattice behaves like the real one where it
matters: it takes a valid entity, refuses a wrong token and a wrongly built entity (and says why), discards an update
that is not newer, takes an entity off the picture when isLive is false, can be told to fail / go slow on purpose,
draws a page that cannot be hijacked by an entity name, and never listens to the whole network by default. Added for
the Gazebo runs (2026-10-31): the journal file (one line per PUT, stamped with its ARRIVAL time, kept after the server
stops, also for PUTs refused for a wrong path or a bad body, and never missing an update that was still being
answered when the server stopped), /history.json, and the linger pause that lets a bridge stopped at the same moment
deliver its last update.
Real sockets on 127.0.0.1 with an ephemeral port; no sleeping except short deliberate delays.
"""

import json
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime, timedelta, timezone

import pytest

from lattice_bridge import fake_lattice
from lattice_bridge.entities import build_uav_asset
from lattice_bridge.fake_lattice import FakeLattice, main
from lattice_bridge.timeutil import parse_utc
from support_net import (AUTH_HEADERS, GOOD_TOKEN, OTHER_TOKEN, SANDBOX_TOKEN, FakeClock, area_at, drop_at,
                         http_request, put_entity, target_at, uav_at)


@pytest.fixture()
def clock():
    return FakeClock()


@pytest.fixture()
def fake(clock):
    server = FakeLattice(token=GOOD_TOKEN, clock=clock.now).start()
    yield server
    server.stop()


def test_a_valid_entity_is_taken_and_appears_on_the_picture(fake, clock):
    entity = uav_at(clock)
    result = put_entity(fake.url, entity)
    assert result.status == 200 and result.json == {}
    assert list(fake.entities()) == [entity['entityId']]
    assert fake.stats()['accepted'] == 1


@pytest.mark.parametrize('headers', [{}, {'authorization': 'Bearer ' + OTHER_TOKEN}, {'authorization': 'Basic abc'},
                                     {'authorization': 'Bearer '}])
def test_a_missing_or_wrong_token_is_refused_and_nothing_is_stored(fake, clock, headers):
    body = json.dumps(uav_at(clock)).encode()
    result = http_request(fake.url, 'PUT', '/api/v1/entities', body, {'content-type': 'application/json', **headers})
    assert result.status == 401 and fake.entities() == {} and fake.stats()['refused_auth'] == 1


def test_the_sandbox_token_is_required_when_the_server_has_one(clock):
    with FakeLattice(token=GOOD_TOKEN, sandbox_token=SANDBOX_TOKEN, clock=clock.now) as server:
        assert put_entity(server.url, uav_at(clock)).status == 401
        good = put_entity(server.url, uav_at(clock), headers={'anduril-sandbox-authorization': 'Bearer ' + SANDBOX_TOKEN})
        assert good.status == 200


def test_open_mode_accepts_any_non_empty_bearer_token(clock):
    with FakeLattice(clock=clock.now) as server:
        assert put_entity(server.url, uav_at(clock), headers={'authorization': 'Bearer anything'}).status == 200
        assert put_entity(server.url, uav_at(clock), headers={'authorization': 'Bearer '}).status == 401


def test_a_wrongly_built_entity_is_refused_with_the_reason(fake, clock):
    broken = uav_at(clock)
    del broken['aliases']
    result = put_entity(fake.url, broken)
    assert result.status == 400 and 'aliases' in result.json['message'] and fake.entities() == {}
    assert fake.stats()['refused_contract'] == 1


@pytest.mark.parametrize('body', [b'not json', b'[1, 2]', b'"text"', b'\xff\xfe'])
def test_a_body_that_is_not_one_json_object_is_refused(fake, body):
    result = http_request(fake.url, 'PUT', '/api/v1/entities', body, {**AUTH_HEADERS})
    assert result.status == 400


def test_an_update_that_is_not_newer_is_discarded_but_still_answered_with_200(fake, clock):
    first = uav_at(clock, lat=36.2100)
    put_entity(fake.url, first)
    same_time_other_place = json.loads(json.dumps(first))
    same_time_other_place['location']['position']['latitudeDegrees'] = 36.2200
    assert put_entity(fake.url, same_time_other_place).status == 200
    held = fake.entities()[first['entityId']]
    assert held['location']['position']['latitudeDegrees'] == 36.21 and fake.stats()['discarded_older'] == 1
    clock.advance(1)
    newer = uav_at(clock, lat=36.2300)
    put_entity(fake.url, newer)
    assert fake.entities()[first['entityId']]['location']['position']['latitudeDegrees'] == 36.23


def test_is_live_false_takes_the_entity_off_the_picture_but_keeps_the_history(fake, clock):
    put_entity(fake.url, uav_at(clock))
    clock.advance(1)
    goodbye = uav_at(clock, is_live=False)
    assert put_entity(fake.url, goodbye).status == 200
    assert fake.entities() == {} and len(fake.history) == 2 and fake.stats()['went_offline'] == 1


def test_one_entity_can_be_read_back_and_an_unknown_one_is_404(fake, clock):
    entity = target_at(clock)
    put_entity(fake.url, entity)
    found = http_request(fake.url, 'GET', '/api/v1/entities/' + entity['entityId'])
    assert found.status == 200 and found.json['entityId'] == entity['entityId']
    assert http_request(fake.url, 'GET', '/api/v1/entities/nope').status == 404
    assert http_request(fake.url, 'GET', '/health').body == b'ok'
    assert len(http_request(fake.url, 'GET', '/entities.json').json) == 1
    assert http_request(fake.url, 'GET', '/stats.json').json['accepted'] == 1


def test_other_paths_and_methods_are_404(fake):
    assert http_request(fake.url, 'PUT', '/api/v1/other', b'{}', AUTH_HEADERS).status == 404
    assert http_request(fake.url, 'GET', '/nothing').status == 404
    assert http_request(fake.url, 'POST', '/elsewhere', b'{}').status == 404


def test_a_huge_body_is_refused_without_reading_it(fake):
    claimed = {**AUTH_HEADERS, 'content-length': str(5 * 1024 * 1024)}       # say it is huge; the server must not wait for it
    result = http_request(fake.url, 'PUT', '/api/v1/entities', b'x', claimed)
    assert result.status == 413


# ------------------------------------------------------------------------------------------ misbehaving on purpose
def test_down_answers_503_to_everything_until_it_is_switched_off(fake, clock):
    assert http_request(fake.url, 'POST', '/control', b'{"down": true}').status == 200
    assert put_entity(fake.url, uav_at(clock)).status == 503 and fake.entities() == {}
    http_request(fake.url, 'POST', '/control', b'{"down": false}')
    assert put_entity(fake.url, uav_at(clock)).status == 200


def test_the_next_n_requests_can_fail_with_a_chosen_status_and_retry_after(fake, clock):
    fake.configure(fail_status=429, fail_count=2, retry_after_s=7)
    first = put_entity(fake.url, uav_at(clock))
    assert first.status == 429 and first.headers['retry-after'] == '7'
    assert put_entity(fake.url, uav_at(clock)).status == 429
    assert put_entity(fake.url, uav_at(clock)).status == 200


def test_a_delay_slows_every_answer(fake, clock):
    fake.configure(delay_s=0.3)
    started = time.monotonic()
    put_entity(fake.url, uav_at(clock))
    assert time.monotonic() - started >= 0.28


def test_a_bad_control_message_is_refused(fake):
    assert http_request(fake.url, 'POST', '/control', b'nonsense').status == 400
    assert http_request(fake.url, 'POST', '/control', b'[1]').status == 400


# ------------------------------------------------------------------------------------------ the page
def test_the_page_draws_everything_that_is_on_the_picture(fake, clock):
    for entity in (uav_at(clock), target_at(clock, 'person'), target_at(clock, 'tent', 1, lat=36.2152, lon=-96.0069),
                   area_at(clock), drop_at(clock)):
        assert put_entity(fake.url, entity).status == 200
    page = http_request(fake.url, 'GET', '/').body.decode()
    assert page.count('<polygon') >= 2 and '<circle' in page and 'rotate(45' in page
    assert '5 entities on the picture' in page and 'http-equiv="refresh"' in page


def test_the_page_is_fine_when_the_picture_is_empty(fake):
    page = http_request(fake.url, 'GET', '/map').body.decode()
    assert 'Nothing on the picture yet' in page and '0 entities' in page


def test_an_entity_name_cannot_inject_markup_into_the_page(fake, clock):
    when = clock.now()
    nasty = build_uav_asset('astra-runway20261024t153000z-uav', 36.2143, -96.0068, fix_time=when, now=when,
                            name='<script>alert(1)</script>')
    assert put_entity(fake.url, nasty).status == 200
    page = http_request(fake.url, 'GET', '/').body.decode()
    assert '<script>alert' not in page and '&lt;script&gt;' in page


def test_it_listens_on_this_computer_only_by_default(clock):
    server = FakeLattice(clock=clock.now)
    assert server.host == '127.0.0.1'
    assert FakeLattice(host='0.0.0.0', clock=clock.now).url.startswith('http://127.0.0.1:')


def test_stop_can_be_called_twice(clock):
    server = FakeLattice(clock=clock.now).start()
    server.stop()
    server.stop()


def test_the_command_line_has_help():
    with pytest.raises(SystemExit) as stopped:
        main(['--help'])
    assert stopped.value.code == 0


def test_time_for_an_expired_entity_is_judged_on_the_servers_clock(fake, clock):
    entity = uav_at(clock)                                                  # expires 120 s after creation
    clock.advance(timedelta(seconds=130).total_seconds())
    assert put_entity(fake.url, entity).status == 400


# ---------------------------------------------------------------------------------------------------
# The journal, /history.json and the linger pause (added for the Gazebo runs, 2026-10-31)
# ---------------------------------------------------------------------------------------------------
def _journal_lines(path):
    with open(path, encoding='utf-8') as handle:
        return [json.loads(line) for line in handle if line.strip()]


def test_the_journal_records_every_request_with_its_outcome_and_a_final_line(tmp_path, clock):
    path = tmp_path / 'deep' / 'folder' / 'journal.jsonl'                  # the folders are made for it
    server = FakeLattice(token=GOOD_TOKEN, clock=clock.now, journal_path=str(path)).start()
    try:
        first = uav_at(clock)
        assert put_entity(server.url, first).status == 200                  # accepted
        assert put_entity(server.url, first).status == 200                  # same time again: discarded
        assert put_entity(server.url, first, headers={'authorization': 'Bearer ' + OTHER_TOKEN}).status == 401
        broken = dict(first, entityId='')
        assert put_entity(server.url, broken).status == 400                 # breaks the contract
        clock.advance(1.0)
        assert put_entity(server.url, uav_at(clock, is_live=False)).status == 200
    finally:
        server.stop()
    lines = _journal_lines(path)
    assert [line['result'] for line in lines] == ['accepted', 'discarded_older', 'refused_auth', 'refused_contract',
                                                  'accepted', 'stopped']
    assert lines[0]['entity'] == first and lines[0]['entityId'] == first['entityId'] and lines[0]['status'] == 200
    assert 'entity' not in lines[2] and 'entity' not in lines[3] and 'contract' in lines[3]['message']
    assert lines[4]['isLive'] is False
    assert lines[-1]['stats']['accepted'] == 2 and lines[-1]['on_picture'] == []
    assert all(line['t'].endswith('Z') for line in lines)                    # the server's UTC receive time
    assert 'unfinished' not in lines[-1]                                     # nothing was still running at the stop


def test_puts_refused_for_a_wrong_path_or_a_bad_body_are_journaled_and_counted_too(tmp_path, clock):
    """Every PUT is counted in `requests`, so every PUT must also have a journal line and an outcome counter."""
    path = tmp_path / 'journal.jsonl'
    server = FakeLattice(token=GOOD_TOKEN, clock=clock.now, journal_path=str(path)).start()
    try:
        assert http_request(server.url, 'PUT', '/api/v1/other', b'{}', AUTH_HEADERS).status == 404
        claimed = {**AUTH_HEADERS, 'content-length': str(5 * 1024 * 1024)}   # "huge": refused without reading it
        assert http_request(server.url, 'PUT', '/api/v1/entities', b'x', claimed).status == 413
        stats = server.stats()
    finally:
        server.stop()
    lines = _journal_lines(path)
    assert [(line['result'], line.get('status')) for line in lines] == [
        ('refused_path', 404), ('refused_malformed', 413), ('stopped', None)]
    assert '/api/v1/other' in lines[0]['message']
    assert stats['requests'] == 2 and stats['refused_path'] == 1 and stats['refused_malformed'] == 1


def test_the_journal_time_is_when_the_request_arrived_not_after_an_injected_delay(tmp_path):
    path = tmp_path / 'journal.jsonl'
    server = FakeLattice(token=GOOD_TOKEN, journal_path=str(path)).start()   # the real clock this time
    try:
        server.configure(delay_s=0.6)
        sent = datetime.now(timezone.utc)
        assert put_entity(server.url, uav_at(FakeClock(wall=sent))).status == 200
    finally:
        server.stop()
    arrived = parse_utc(_journal_lines(path)[0]['t'])
    assert abs((arrived - sent).total_seconds()) < 0.4                        # not 0.6 s later, after the delay


def _put_in_the_background(server, clock):
    """Start one PUT in a thread; returns (thread, list that receives its HTTP status)."""
    answers = []
    entity = uav_at(clock)
    worker = threading.Thread(target=lambda: answers.append(put_entity(server.url, entity).status), daemon=True)
    worker.start()
    deadline = time.monotonic() + 5
    while server.stats()['requests'] < 1 and time.monotonic() < deadline:   # it has arrived and is being answered
        time.sleep(0.01)
    return worker, answers


def test_an_update_still_being_answered_when_the_server_stops_lands_before_the_stopped_line(tmp_path, clock):
    """The race the reviewer found: stop() used to write "stopped" first, then the late update got 200 and no line."""
    path = tmp_path / 'journal.jsonl'
    server = FakeLattice(token=GOOD_TOKEN, clock=clock.now, journal_path=str(path)).start()
    server.configure(delay_s=0.5)                                             # keeps the PUT running during stop()
    worker, answers = _put_in_the_background(server, clock)
    server.stop()
    worker.join(timeout=5)
    assert answers == [200]
    lines = _journal_lines(path)
    assert [line['result'] for line in lines] == ['accepted', 'stopped'] and 'unfinished' not in lines[-1]


def test_an_update_that_outlasts_the_stop_wait_is_not_taken_and_the_stopped_line_says_so(tmp_path, clock, monkeypatch):
    monkeypatch.setattr(fake_lattice, 'STOP_WAIT_S', 0.1)                      # a short wait keeps the test quick
    path = tmp_path / 'journal.jsonl'
    server = FakeLattice(token=GOOD_TOKEN, clock=clock.now, journal_path=str(path)).start()
    server.configure(delay_s=0.7)
    worker, answers = _put_in_the_background(server, clock)
    server.stop()                                                             # gives up waiting after 0.1 s
    worker.join(timeout=5)
    assert answers == [503] and server.entities() == {} and server.stats()['too_late'] == 1
    lines = _journal_lines(path)
    assert [line['result'] for line in lines] == ['stopped'] and lines[0]['unfinished'] == 1


def test_a_stopped_server_can_be_started_again_and_takes_updates(clock):
    server = FakeLattice(token=GOOD_TOKEN, clock=clock.now).start()
    server.stop()
    server.port = 0                                                           # the old port may still be held
    server.start()
    try:
        assert put_entity(server.url, uav_at(clock)).status == 200
    finally:
        server.stop()


def test_a_journal_that_cannot_be_written_is_switched_off_and_the_server_still_answers(tmp_path, clock, capsys):
    server = FakeLattice(token=GOOD_TOKEN, clock=clock.now, journal_path=str(tmp_path)).start()   # a folder, not a file
    try:
        assert server.journal_error
        assert put_entity(server.url, uav_at(clock)).status == 200
    finally:
        server.stop()
    assert 'switched off' in capsys.readouterr().err


def test_history_json_lists_every_taken_update_oldest_first(fake, clock):
    put_entity(fake.url, uav_at(clock))
    clock.advance(1.0)
    put_entity(fake.url, uav_at(clock, is_live=False))
    result = http_request(fake.url, 'GET', '/history.json')
    assert result.status == 200
    assert [entity.get('isLive') for entity in result.json] == [True, False]
    assert fake.entities() == {}                                              # off the picture, still in the history


@pytest.mark.parametrize('text, env, expected', [
    ('', {}, ''),
    ('/tmp/j.jsonl', {}, '/tmp/j.jsonl'),
    ('$RUN/lattice/j.jsonl', {'RUN': '/runs/r1'}, '/runs/r1/lattice/j.jsonl'),
    ('${RUN}_x/j.jsonl', {'RUN': '/runs/r1'}, '/runs/r1_x/j.jsonl'),
])
def test_journal_paths_expand_variables(text, env, expected):
    from lattice_bridge.fake_lattice import resolve_journal_path
    assert resolve_journal_path(text, env) == (expected, '')


@pytest.mark.parametrize('env', [{}, {'ASTRA_RUN_DIR': ''}])
def test_a_journal_path_with_a_missing_variable_turns_the_journal_off_with_a_warning(env):
    from lattice_bridge.fake_lattice import resolve_journal_path
    path, warning = resolve_journal_path('$ASTRA_RUN_DIR/lattice/j.jsonl', env)
    assert path == '' and 'ASTRA_RUN_DIR' in warning and 'not set' in warning


def test_the_command_line_refuses_a_silly_linger():
    with pytest.raises(SystemExit) as stopped:
        main(['--linger-s', '-1'])
    assert stopped.value.code == 2


@pytest.mark.skipif(os.name != 'posix', reason='sends SIGTERM to a child process (POSIX only)')
def test_after_a_stop_signal_the_server_keeps_answering_for_the_linger_time(tmp_path):
    """What the simulator needs: everything gets SIGINT/SIGTERM at once, the bridge's last update must still land."""
    journal = tmp_path / 'journal.jsonl'
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(sys.path))
    env['LATTICE_TOKEN'] = GOOD_TOKEN                                       # the made-up test token (support_net.py)
    child = subprocess.Popen([sys.executable, '-m', 'lattice_bridge.fake_lattice', '--port', '0', '--linger-s', '1.5',
                              '--journal', str(journal)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                             env=env)
    try:
        first = child.stdout.readline()
        assert 'listening on http://127.0.0.1:' in first, first
        url = first.split('listening on ')[1].split()[0]
        child.send_signal(signal.SIGTERM)
        stopped_at = time.monotonic()
        time.sleep(0.3)                                                     # well inside the 1.5 s pause
        clock = FakeClock(wall=datetime.now(timezone.utc))
        assert put_entity(url, uav_at(clock, is_live=False)).status == 200    # the "goodbye" still lands
        assert child.wait(timeout=10) == 0
        assert time.monotonic() - stopped_at >= 1.2
    finally:
        if child.poll() is None:
            child.kill()
        child.stdout.close()
    lines = _journal_lines(journal)
    assert [line['result'] for line in lines] == ['accepted', 'stopped'] and lines[0]['isLive'] is False
