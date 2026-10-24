"""
lattice_bridge/test/test_sink_base.py -- PURPOSE: prove the sink contract of sinks/base.py: the typed errors carry
what the Publisher needs (retry hint, "systemic" flag, status), error text is bounded and printable, BaseSink
cannot be used half-implemented, its legacy write() never raises, adapt_sink() wraps plain write()/close()
objects, and the recorder (JsonlSink) is a real BaseSink.
"""

import logging

import pytest

from lattice_bridge.sinks import Sink
from lattice_bridge.sinks.base import (
    AuthSinkError, BaseSink, ConfigSinkError, FatalSinkError, ObjectInfo, RetryableSinkError, SinkError,
    adapt_sink, short_text,
)
from lattice_bridge.sinks.jsonl_sink import JsonlSink, read_records
from support import NOW
from support_net import FakeClock, uav_at


class Recorder(BaseSink):
    """The smallest legal sink: remembers what it got, can be told to fail."""
    name = 'recorder'

    def __init__(self, error=None):
        self.got = []
        self.error = error
        self.closed = 0

    def publish_entity(self, entity, t=None):
        if self.error is not None:
            raise self.error
        self.got.append((entity, t))

    def close(self):
        self.closed += 1


# ------------------------------------------------------------------------------------ the typed errors
def test_error_hierarchy_lets_callers_catch_by_meaning():
    assert issubclass(RetryableSinkError, SinkError) and issubclass(FatalSinkError, SinkError)
    assert issubclass(AuthSinkError, FatalSinkError) and issubclass(ConfigSinkError, FatalSinkError)
    assert not issubclass(RetryableSinkError, FatalSinkError)
    assert not issubclass(FatalSinkError, RetryableSinkError)


def test_retryable_error_carries_the_servers_retry_hint_and_status():
    err = RetryableSinkError('slow down', status=429, retry_after_s=7.5)
    assert (err.message, err.status, err.retry_after_s) == ('slow down', 429, 7.5)
    assert str(err) == 'slow down'
    assert RetryableSinkError('x').retry_after_s is None and RetryableSinkError('x').status is None


def test_fatal_errors_say_whether_they_are_systemic():
    assert FatalSinkError('bad body', status=400).systemic is False
    assert FatalSinkError('bad cert', systemic=True).systemic is True
    assert AuthSinkError('no', status=401).systemic is True and AuthSinkError('no').status is None
    assert ConfigSinkError('no endpoint').systemic is True


def test_error_text_is_one_bounded_printable_line():
    err = RetryableSinkError('line one\nline two\x00\x07' + 'x' * 1000)
    assert '\n' not in err.message and '\x00' not in err.message and len(err.message) <= 300
    assert err.message.endswith('...')
    assert short_text('  padded  ') == 'padded'
    assert short_text(12345) == '12345'
    assert short_text('abcdef', limit=5) == 'ab...'


# ------------------------------------------------------------------------------------ the abstract base
def test_a_half_written_sink_cannot_be_created():
    class NoClose(BaseSink):
        def publish_entity(self, entity, t=None):
            return None

    class NoPublish(BaseSink):
        def close(self):
            return None

    with pytest.raises(TypeError):
        NoClose()
    with pytest.raises(TypeError):
        NoPublish()
    with pytest.raises(TypeError):
        BaseSink()


def test_every_base_sink_also_satisfies_the_older_protocol():
    assert isinstance(Recorder(), Sink)


def test_objects_are_refused_by_default_and_the_message_names_the_sink():
    with pytest.raises(FatalSinkError, match='recorder sink does not store objects'):
        Recorder().upload_object('a/b.jpg', b'x')


def test_describe_defaults_to_the_name():
    assert Recorder().describe() == 'recorder'


def test_context_manager_closes_the_sink():
    sink = Recorder()
    with sink as same:
        assert same is sink
    assert sink.closed == 1


def test_object_info_is_a_plain_frozen_record():
    info = ObjectInfo('a/b.jpg', 3, 'abc')
    assert (info.path, info.size_bytes, info.checksum, info.expiry_time) == ('a/b.jpg', 3, 'abc', None)
    with pytest.raises(Exception):
        info.path = 'other'


# ------------------------------------------------------------------------------------ legacy write()
def test_write_delivers_entities_and_returns_true():
    sink = Recorder()
    entity = uav_at(FakeClock())
    assert sink.write('entity', entity, NOW) is True
    assert sink.got == [(entity, NOW)]


def test_write_refuses_other_kinds_without_raising():
    sink = Recorder()
    assert sink.write('note', 'hello') is False
    assert sink.got == []


@pytest.mark.parametrize('error', [
    RetryableSinkError('down'), FatalSinkError('no'), AuthSinkError('401'), SinkError('plain'),
    RuntimeError('a bug in a sink'), ValueError('another'),
])
def test_write_never_raises_whatever_the_sink_does(error, caplog):
    sink = Recorder(error=error)
    with caplog.at_level(logging.WARNING, logger='lattice_bridge.sink'):
        assert sink.write('entity', {'entityId': 'x'}) is False
    assert any('not delivered' in r.getMessage() or 'unexpected' in r.getMessage() for r in caplog.records)


# ------------------------------------------------------------------------------------ adapt_sink
def test_adapt_sink_returns_a_base_sink_unchanged():
    sink = Recorder()
    assert adapt_sink(sink) is sink


def test_adapt_sink_wraps_an_object_with_write_and_close():
    class Legacy:
        def __init__(self):
            self.calls = []
            self.closed = False
            self.accept = True

        def write(self, kind, payload, t=None):
            self.calls.append((kind, payload, t))
            return self.accept

        def close(self):
            self.closed = True

    legacy = Legacy()
    sink = adapt_sink(legacy)
    assert isinstance(sink, BaseSink) and sink.name == 'Legacy'
    sink.publish_entity({'entityId': 'x'}, NOW)
    assert legacy.calls == [('entity', {'entityId': 'x'}, NOW)]
    legacy.accept = False
    with pytest.raises(FatalSinkError, match='refused the entity'):
        sink.publish_entity({'entityId': 'y'})
    sink.close()
    assert legacy.closed is True


@pytest.mark.parametrize('thing', [None, 5, 'text', object(), type('OnlyWrite', (), {'write': lambda *a: True})()])
def test_adapt_sink_rejects_things_that_are_not_sinks(thing):
    with pytest.raises(TypeError):
        adapt_sink(thing)


# ------------------------------------------------------------------------------------ the recorder
def test_the_jsonl_recorder_is_a_base_sink_the_publisher_can_use(tmp_path):
    clock = FakeClock()
    entity = uav_at(clock)
    path = tmp_path / 'rec.jsonl'
    with JsonlSink(path) as sink:
        assert isinstance(sink, BaseSink) and isinstance(sink, Sink)
        assert sink.name == 'jsonl' and sink.describe() == 'jsonl file rec.jsonl'
        sink.publish_entity(entity, clock.now())
    records = read_records(path).records
    assert len(records) == 1 and records[0].kind == 'entity' and records[0].payload == entity
    assert records[0].t == clock.now()


def test_the_recorder_reports_a_refused_entity_as_a_fatal_error(tmp_path):
    sink = JsonlSink(tmp_path / 'rec.jsonl')
    sink.close()
    with pytest.raises(FatalSinkError):
        sink.publish_entity({'entityId': 'x'}, NOW)
    other = JsonlSink(tmp_path / 'rec2.jsonl')
    with pytest.raises(FatalSinkError):
        other.publish_entity({'entityId': 'x', 'bad': float('nan')}, NOW)
    other.close()
