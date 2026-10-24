"""
lattice_bridge/test/test_jsonl_sink.py -- PURPOSE: prove the JSONL recorder: one line per request with exactly
{t, kind, payload}, append-only, readable back, never raising on bad content or I/O trouble, size-bounded,
thread safe, tolerant of a half-written last line, and usable as the oracle of "what would Lattice contain now".
"""

import json
import logging
import threading
from datetime import datetime, timedelta, timezone

import pytest

from lattice_bridge.entities import build_target_track, build_uav_asset
from lattice_bridge.sinks import Sink
from lattice_bridge.sinks.jsonl_sink import JsonlSink, latest_by_entity_id, read_records
from support import MISSION, NOW


def uav(seconds, lat=36.2):
    when = NOW + timedelta(seconds=seconds)
    return build_uav_asset(f'astra-{MISSION}-uav', lat, -96.0, fix_time=when, now=when)


def test_one_line_per_request_with_exactly_three_keys(tmp_path):
    path = tmp_path / 'run' / 'requests.jsonl'
    with JsonlSink(path) as sink:
        assert sink.write('entity', uav(0), NOW) is True
        assert sink.write('entity', uav(1), NOW + timedelta(seconds=1)) is True
    lines = path.read_text(encoding='utf-8').splitlines()
    assert len(lines) == 2
    for line, seconds in zip(lines, (0, 1)):
        obj = json.loads(line)
        assert set(obj) == {'t', 'kind', 'payload'}
        assert obj['kind'] == 'entity'
        assert obj['t'] == (NOW + timedelta(seconds=seconds)).strftime('%Y-%m-%dT%H:%M:%S.%fZ')
        assert obj['payload']['entityId'] == f'astra-{MISSION}-uav'
    assert path.read_bytes().endswith(b'\n') and b'\r' not in path.read_bytes()


def test_the_parent_folder_is_created_and_the_file_is_opened_lazily(tmp_path):
    path = tmp_path / 'a' / 'b' / 'c.jsonl'
    sink = JsonlSink(path)
    assert not path.parent.exists()
    sink.write('note', 'hello', NOW)
    sink.close()
    assert path.exists()


def test_lines_are_flushed_immediately(tmp_path):
    path = tmp_path / 'r.jsonl'
    sink = JsonlSink(path)
    sink.write('note', 'x', NOW)
    assert path.read_text(encoding='utf-8').count('\n') == 1             # readable BEFORE close(): crash-safe
    sink.close()


def test_append_only_across_instances(tmp_path):
    path = tmp_path / 'r.jsonl'
    for i in range(3):
        with JsonlSink(path) as sink:
            sink.write('entity', uav(i), NOW + timedelta(seconds=i))
    assert len(read_records(path).records) == 3
    first = path.read_text(encoding='utf-8').splitlines()[0]
    with JsonlSink(path) as sink:
        sink.write('note', 'more', NOW)
    assert path.read_text(encoding='utf-8').splitlines()[0] == first


def test_the_clock_is_only_read_when_no_time_is_given(tmp_path):
    ticks = iter([NOW, NOW + timedelta(seconds=9)])
    sink = JsonlSink(tmp_path / 'r.jsonl', clock=lambda: next(ticks))
    sink.write('note', 'a')
    sink.write('note', 'b', NOW + timedelta(hours=1))
    sink.write('note', 'c')
    sink.close()
    times = [r.t for r in read_records(tmp_path / 'r.jsonl').records]
    assert times == [NOW, NOW + timedelta(hours=1), NOW + timedelta(seconds=9)]


def test_default_clock_writes_utc_with_z(tmp_path):
    sink = JsonlSink(tmp_path / 'r.jsonl')
    sink.write('note', 'now')
    sink.close()
    text = (tmp_path / 'r.jsonl').read_text(encoding='utf-8')
    assert json.loads(text)['t'].endswith('Z')
    assert read_records(tmp_path / 'r.jsonl').records[0].t.tzinfo is not None


def test_read_back_gives_typed_records_in_order(tmp_path):
    path = tmp_path / 'r.jsonl'
    with JsonlSink(path) as sink:
        sink.write('note', 'start', NOW)
        sink.write('entity', uav(1), NOW + timedelta(seconds=1))
    result = read_records(path)
    assert result.bad_lines == 0
    assert [(r.kind, r.line_no) for r in result.records] == [('note', 1), ('entity', 2)]
    assert result.records[0].payload == 'start' and result.records[1].payload['isLive'] is True
    assert result.records[1].t == NOW + timedelta(seconds=1)


def test_other_time_zones_are_stored_as_utc(tmp_path):
    plus2 = timezone(timedelta(hours=2))
    with JsonlSink(tmp_path / 'r.jsonl') as sink:
        assert sink.write('note', 'x', datetime(2026, 10, 24, 17, 30, 0, tzinfo=plus2))
    assert read_records(tmp_path / 'r.jsonl').records[0].t == NOW
    assert '"t":"2026-10-24T15:30:00.000000Z"' in (tmp_path / 'r.jsonl').read_text(encoding='utf-8')


# ------------------------------------------------------------------------------------ never raises
@pytest.mark.parametrize('payload', [float('nan'), {'x': float('inf')}, {1, 2}, object(), b'bytes', lambda: 1])
def test_content_that_is_not_json_is_refused_without_raising(tmp_path, payload):
    path = tmp_path / 'r.jsonl'
    sink = JsonlSink(path)
    assert sink.write('entity', payload, NOW) is False
    assert sink.write('entity', {'ok': 1}, NOW) is True                  # the sink keeps working
    sink.close()
    assert len(read_records(path).records) == 1
    assert sink.failed == 1 and sink.written == 1


@pytest.mark.parametrize('kind', ['', 'Entity', 'with space', 'x' * 40, '1abc', None, 5, 'a/b', 'a\nb'])
def test_bad_kinds_are_refused(tmp_path, kind):
    sink = JsonlSink(tmp_path / 'r.jsonl')
    assert sink.write(kind, {}, NOW) is False
    sink.close()


@pytest.mark.parametrize('moment', [datetime(2026, 10, 24, 15, 30, 0), 'now', 5, 1761319800.0])
def test_naive_or_non_time_values_are_refused(tmp_path, moment):
    sink = JsonlSink(tmp_path / 'r.jsonl')
    assert sink.write('note', 'x', moment) is False
    sink.close()


def test_a_circular_payload_does_not_crash(tmp_path):
    loop = {}
    loop['me'] = loop
    sink = JsonlSink(tmp_path / 'r.jsonl')
    assert sink.write('note', loop, NOW) is False
    sink.close()


class _Collector(logging.Handler):
    """A log handler that keeps the records. (caplog is not used: with ROS sourced, the launch_testing pytest
    plugins reconfigure logging, and a handler attached to the logger itself works in every environment.)"""

    def __init__(self):
        super().__init__(level=logging.WARNING)
        self.records = []

    def emit(self, record):
        self.records.append(record)


def test_io_errors_are_swallowed_and_logged_once_in_a_while(tmp_path):
    blocked = tmp_path / 'afile'
    blocked.write_text('i am a file, not a folder')
    sink = JsonlSink(blocked / 'sub' / 'r.jsonl')                        # mkdir under a file must fail
    logger = logging.getLogger('lattice_bridge.jsonl_sink')
    collector, old_level = _Collector(), logger.level
    logger.addHandler(collector)
    logger.setLevel(logging.WARNING)
    try:
        results = [sink.write('note', i, NOW) for i in range(250)]
    finally:
        logger.removeHandler(collector)
        logger.setLevel(old_level)
    assert results == [False] * 250 and sink.failed == 250
    assert 1 <= len(collector.records) <= 4                              # failure #1, #100, #200 - not 250 lines
    assert 'failure #1' in collector.records[0].getMessage()


def test_a_directory_as_the_target_does_not_crash(tmp_path):
    sink = JsonlSink(tmp_path)
    assert sink.write('note', 'x', NOW) is False


def test_writing_after_close_is_refused_and_close_is_idempotent(tmp_path):
    sink = JsonlSink(tmp_path / 'r.jsonl')
    sink.write('note', 'x', NOW)
    sink.close()
    sink.close()
    assert sink.write('note', 'y', NOW) is False
    assert len(read_records(tmp_path / 'r.jsonl').records) == 1


def test_context_manager_closes(tmp_path):
    with JsonlSink(tmp_path / 'r.jsonl') as sink:
        sink.write('note', 'x', NOW)
    assert sink.write('note', 'y', NOW) is False


# ------------------------------------------------------------------------------------ bounds
def test_the_size_limit_stops_the_file_growing(tmp_path):
    path = tmp_path / 'r.jsonl'
    sink = JsonlSink(path, max_bytes=500)
    results = [sink.write('note', 'x' * 40, NOW) for _ in range(30)]
    sink.close()
    assert results.count(True) < 30 and results[0] is True and results[-1] is False
    assert path.stat().st_size <= 500
    assert len(read_records(path).records) == results.count(True)


def test_the_size_limit_counts_what_is_already_in_the_file(tmp_path):
    path = tmp_path / 'r.jsonl'
    path.write_bytes(b'x' * 480)
    sink = JsonlSink(path, max_bytes=500)
    assert sink.write('note', 'this will not fit in twenty bytes', NOW) is False
    sink.close()


@pytest.mark.parametrize('bad', [0, -1, 1.5, '10', None, True])
def test_bad_max_bytes_raise(tmp_path, bad):
    with pytest.raises(ValueError):
        JsonlSink(tmp_path / 'r.jsonl', max_bytes=bad)


# ------------------------------------------------------------------------------------ threads
def test_many_threads_write_whole_lines(tmp_path):
    path = tmp_path / 'r.jsonl'
    sink = JsonlSink(path)

    def worker(index):
        for i in range(100):
            assert sink.write('entity', {'entityId': f'e{index}', 'n': i, 'pad': 'z' * 200}, NOW)

    threads = [threading.Thread(target=worker, args=(i,)) for i in range(6)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    sink.close()
    result = read_records(path, strict=True)
    assert len(result.records) == 600 and result.bad_lines == 0
    per_thread = {}
    for record in result.records:
        per_thread.setdefault(record.payload['entityId'], []).append(record.payload['n'])
    assert all(values == list(range(100)) for values in per_thread.values())      # each thread's order is kept


# ------------------------------------------------------------------------------------ reading
def test_a_half_written_last_line_is_skipped_and_counted(tmp_path):
    path = tmp_path / 'r.jsonl'
    with JsonlSink(path) as sink:
        sink.write('note', 'good', NOW)
    with open(path, 'ab') as handle:
        handle.write(b'{"t":"2026-10-24T15:30:00.000000Z","kind":"entity","payl')           # power cut mid-write
    result = read_records(path)
    assert len(result.records) == 1 and result.bad_lines == 1
    with pytest.raises(ValueError, match='line 2'):
        read_records(path, strict=True)


@pytest.mark.parametrize('line', [
    '[]', '5', 'null', '{}', '{"t":"2026-10-24T15:30:00.000000Z","kind":"entity"}',
    '{"t":"2026-10-24T15:30:00.000000Z","kind":"entity","payload":1,"extra":2}',
    '{"t":"2026-10-24T15:30:00","kind":"entity","payload":1}', '{"t":5,"kind":"entity","payload":1}',
    '{"t":"2026-10-24T15:30:00.000000Z","kind":"Entity","payload":1}',
    '{"t":"2026-10-24T15:30:00.000000Z","kind":7,"payload":1}', 'not json at all', '\xff\xfe',
])
def test_invalid_lines_are_counted_not_returned(tmp_path, line):
    path = tmp_path / 'r.jsonl'
    path.write_bytes(line.encode('latin-1') + b'\n' + b'\n')
    result = read_records(path)
    assert result.records == [] and result.bad_lines == 1


def test_blank_lines_are_ignored(tmp_path):
    path = tmp_path / 'r.jsonl'
    path.write_text('\n\n{"t":"2026-10-24T15:30:00.000000Z","kind":"note","payload":"x"}\n\n', encoding='utf-8')
    result = read_records(path)
    assert len(result.records) == 1 and result.bad_lines == 0


def test_a_missing_file_raises_file_not_found(tmp_path):
    with pytest.raises(FileNotFoundError):
        read_records(tmp_path / 'nope.jsonl')


def test_latest_by_entity_id_is_what_lattice_would_hold(tmp_path):
    path = tmp_path / 'r.jsonl'
    track = build_target_track(f'astra-{MISSION}-tgt-person-1', 'person', 36.2, -96.0, 0.5, measured_time=NOW,
                               now=NOW, expiry_time=NOW + timedelta(hours=1))
    with JsonlSink(path) as sink:
        sink.write('note', 'start', NOW)
        sink.write('entity', uav(0, lat=36.2), NOW)
        sink.write('entity', track, NOW)
        sink.write('entity', uav(1, lat=36.3), NOW + timedelta(seconds=1))
        sink.write('entity', {'no': 'id'}, NOW)
        sink.write('entity', 'not a dict', NOW)
    latest = latest_by_entity_id(path)
    assert sorted(latest) == [f'astra-{MISSION}-tgt-person-1', f'astra-{MISSION}-uav']
    assert latest[f'astra-{MISSION}-uav']['location']['position']['latitudeDegrees'] == 36.3
    assert latest[f'astra-{MISSION}-tgt-person-1']['tracked']['trackQualityWrapper'] == 8


def test_it_is_a_sink(tmp_path):
    sink = JsonlSink(tmp_path / 'r.jsonl')
    assert isinstance(sink, Sink)
    sink.close()


def test_the_recording_contains_no_headers_or_secrets(tmp_path):
    path = tmp_path / 'r.jsonl'
    with JsonlSink(path) as sink:
        sink.write('entity', uav(0), NOW)
    text = path.read_text(encoding='utf-8').lower()
    for word in ('authorization', 'bearer', 'token', 'password', 'secret'):
        assert word not in text
