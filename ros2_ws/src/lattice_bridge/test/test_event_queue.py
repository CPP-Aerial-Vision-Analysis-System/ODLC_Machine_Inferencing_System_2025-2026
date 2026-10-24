"""
lattice_bridge/test/test_event_queue.py -- PURPOSE: prove the small queue between the reader thread and the bridge
loop, and the background reader that feeds it: only the newest position is kept, a flood cannot grow memory, target
reports and releases are the last things to be lost (notices go first), a consumer never waits longer than it asked,
and a source that raises cannot kill the reader. Real threads, but every wait is a fraction of a second.
"""

import threading
import time
from datetime import datetime, timedelta, timezone

import pytest

from lattice_bridge.events import DroneFix, DropEvent, SourceNotice, TargetSighting
from lattice_bridge.sources.background import BackgroundSource
from lattice_bridge.sources.common import EventQueue

T0 = datetime(2026, 10, 24, 15, 30, 0, tzinfo=timezone.utc)


def fix(seconds):
    when = T0 + timedelta(seconds=seconds)
    return DroneFix(fix_time=when, arrival=when, lat_deg=36.215, lon_deg=-96.0068)


def sighting(index):
    return TargetSighting('person', 36.215, -96.006, 0.9, T0 + timedelta(seconds=index), seq=index)


def notice(index):
    return SourceNotice('info', f'n{index}', T0)


def test_only_the_newest_position_is_kept():
    queue = EventQueue()
    for second in range(5):
        queue.put(fix(second))
    [only] = queue.get_many(10, 0)
    assert only.fix_time == T0 + timedelta(seconds=4) and queue.dropped_fixes == 4


def test_other_events_keep_their_order_around_the_position():
    queue = EventQueue()
    queue.put(sighting(1))
    queue.put(fix(1))
    queue.put(DropEvent('tent', T0))
    assert [type(e).__name__ for e in queue.get_many(10, 0)] == ['TargetSighting', 'DroneFix', 'DropEvent']


def test_a_flood_cannot_grow_the_queue_and_notices_are_lost_before_targets():
    queue = EventQueue(max_events=5)
    queue.put(notice(0))
    queue.put(sighting(1))
    queue.put(notice(1))
    queue.put(sighting(2))
    queue.put(sighting(3))
    for index in range(4, 9):
        queue.put(sighting(index))
    items = queue.get_many(50, 0)
    assert len(items) == 5 and queue.dropped_other == 5
    assert not any(isinstance(e, SourceNotice) for e in items)                       # both notices went first
    assert [e.seq for e in items] == [4, 5, 6, 7, 8]                                  # then the oldest targets


def test_get_many_never_waits_longer_than_asked_and_returns_at_once_when_something_is_there():
    queue = EventQueue()
    started = time.monotonic()
    assert queue.get_many(10, 0.15) == []
    assert 0.1 <= time.monotonic() - started < 1.0
    queue.put(sighting(1))
    started = time.monotonic()
    assert len(queue.get_many(10, 5.0)) == 1 and time.monotonic() - started < 0.5


def test_a_waiting_consumer_wakes_up_when_a_producer_puts_something():
    queue = EventQueue()
    threading.Timer(0.05, queue.put, args=(sighting(1),)).start()
    started = time.monotonic()
    assert len(queue.get_many(10, 3.0)) == 1 and time.monotonic() - started < 1.5


def test_the_batch_size_is_respected():
    queue = EventQueue()
    for index in range(10):
        queue.put(sighting(index))
    assert len(queue.get_many(3, 0)) == 3 and len(queue) == 7


def test_a_queue_must_hold_at_least_two_events():
    with pytest.raises(ValueError):
        EventQueue(max_events=1)


class Inner:
    """A scripted source for the background reader."""
    name = 'inner'
    altitude_reference = 'amsl'

    def __init__(self, batches):
        self.batches, self.started, self.closed, self.lock = list(batches), False, 0, threading.Lock()

    def start(self):
        self.started = True

    def poll(self, timeout_s):
        with self.lock:
            item = self.batches.pop(0) if self.batches else []
        if isinstance(item, Exception):
            raise item
        if not item:
            time.sleep(min(timeout_s, 0.01))
        return item

    def now(self):
        return T0

    def describe(self):
        return 'inner source'

    def stats(self):
        return {'received': 1}

    def close(self):
        self.closed += 1


def test_the_background_reader_delivers_what_the_inner_source_produces():
    inner = Inner([[sighting(1)], [], [sighting(2)]])
    source = BackgroundSource(inner, poll_s=0.01)
    source.start()
    try:
        got = []
        deadline = time.monotonic() + 3.0
        while len(got) < 2 and time.monotonic() < deadline:
            got += source.poll(0.1)
        assert [e.seq for e in got] == [1, 2] and inner.started
        assert (source.name, source.altitude_reference, source.describe()) == ('inner', 'amsl', 'inner source')
        assert source.stats()['received'] == 1 and 'queue_dropped_other' in source.stats()
    finally:
        source.close()


def test_the_reader_survives_a_source_that_raises():
    inner = Inner([RuntimeError('boom'), [sighting(7)]])
    source = BackgroundSource(inner, poll_s=0.01, error_pause_s=0.01)
    source.start()
    try:
        got = []
        deadline = time.monotonic() + 3.0
        while not got and time.monotonic() < deadline:
            got += source.poll(0.1)
        assert [e.seq for e in got] == [7] and source.stats()['reader_errors'] == 1
    finally:
        source.close()


def test_close_stops_the_thread_closes_the_inner_source_and_is_repeatable():
    inner = Inner([])
    source = BackgroundSource(inner, poll_s=0.01)
    source.start()
    thread = source._thread
    source.close()
    source.close()
    assert not thread.is_alive() and inner.closed == 2 and source.now() == T0
