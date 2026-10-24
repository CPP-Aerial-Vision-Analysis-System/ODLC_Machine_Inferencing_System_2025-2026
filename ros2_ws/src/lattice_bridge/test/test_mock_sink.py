"""
lattice_bridge/test/test_mock_sink.py -- PURPOSE: prove the in-memory test sink: it records deliveries, plays
Lattice's own "only a later sourceUpdateTime counts, isLive=false deletes" rule, fails on command (queued errors, a
"down" switch, a hook) and is safe to use from several threads, so the Publisher tests can trust it.
"""

import threading
from datetime import timedelta

import pytest

from lattice_bridge.sinks.base import BaseSink, FatalSinkError, RetryableSinkError
from lattice_bridge.sinks.mock_sink import MockSink
from support import NOW
from support_net import FakeClock, uav_at


def entity_at(seconds, entity_id='astra-x-uav', lat=36.2):
    clock = FakeClock()
    clock.advance(seconds)
    return uav_at(clock, entity_id=entity_id, lat=lat)


def test_it_is_a_base_sink_and_describes_itself():
    sink = MockSink()
    assert isinstance(sink, BaseSink) and sink.describe() == 'mock (in memory)'


def test_deliveries_are_recorded_in_order_with_private_copies():
    sink = MockSink()
    first = entity_at(0)
    sink.publish_entity(first, NOW)
    first['location']['position']['latitudeDegrees'] = 0.5            # changing the original changes nothing
    sink.publish_entity(entity_at(1, 'astra-x-other'), NOW + timedelta(seconds=1))
    assert sink.delivered_ids() == ['astra-x-uav', 'astra-x-other']
    assert sink.deliveries[0].entity['location']['position']['latitudeDegrees'] == 36.2
    assert [d.number for d in sink.deliveries] == [1, 2]
    assert sink.deliveries[0].t == NOW and sink.calls == 2


def test_the_picture_keeps_the_newest_version_per_id():
    sink = MockSink()
    sink.publish_entity(entity_at(0, lat=36.0))
    sink.publish_entity(entity_at(5, lat=36.5))
    assert sink.entities()['astra-x-uav']['location']['position']['latitudeDegrees'] == 36.5
    assert sink.stale_discards == 0


def test_an_older_or_equal_source_time_is_discarded_and_counted_like_lattice_does():
    sink = MockSink()
    sink.publish_entity(entity_at(10, lat=36.5))
    sink.publish_entity(entity_at(5, lat=36.1))                       # older
    sink.publish_entity(entity_at(10, lat=36.9))                      # equal
    assert sink.entities()['astra-x-uav']['location']['position']['latitudeDegrees'] == 36.5
    assert sink.stale_discards == 2
    assert len(sink.deliveries) == 3                                  # still delivered, just ignored by "Lattice"


def test_the_time_rule_can_be_switched_off():
    sink = MockSink(enforce_source_time=False)
    sink.publish_entity(entity_at(10, lat=36.5))
    sink.publish_entity(entity_at(5, lat=36.1))
    assert sink.entities()['astra-x-uav']['location']['position']['latitudeDegrees'] == 36.1
    assert sink.stale_discards == 0


def test_a_goodbye_removes_the_entity_from_the_picture():
    sink = MockSink()
    live = entity_at(0)
    sink.publish_entity(live)
    bye = entity_at(3)
    bye['isLive'] = False
    sink.publish_entity(bye)
    assert sink.entities() == {}


def test_queued_errors_are_used_one_per_call_then_success_resumes():
    sink = MockSink()
    sink.fail_next(RetryableSinkError('boom'), times=2)
    for _ in range(2):
        with pytest.raises(RetryableSinkError):
            sink.publish_entity(entity_at(0))
    sink.publish_entity(entity_at(0))
    assert [a.ok for a in sink.attempts] == [False, False, True]
    assert [a.error for a in sink.attempts] == ['RetryableSinkError', 'RetryableSinkError', None]
    assert len(sink.deliveries) == 1


def test_go_down_fails_everything_until_come_back():
    sink = MockSink()
    sink.go_down()
    for _ in range(3):
        with pytest.raises(RetryableSinkError):
            sink.publish_entity(entity_at(0))
    sink.come_back()
    sink.publish_entity(entity_at(0))
    assert sink.calls == 4 and len(sink.deliveries) == 1


def test_go_down_can_use_a_chosen_error():
    sink = MockSink()
    sink.go_down(FatalSinkError('refused', status=400))
    with pytest.raises(FatalSinkError):
        sink.publish_entity(entity_at(0))


def test_the_hook_sees_every_entity_and_may_fail_it():
    sink = MockSink()
    seen = []

    def hook(entity):
        seen.append(entity['entityId'])
        if len(seen) == 2:
            raise RetryableSinkError('hook says no')

    sink.set_hook(hook)
    sink.publish_entity(entity_at(0))
    with pytest.raises(RetryableSinkError):
        sink.publish_entity(entity_at(1))
    assert seen == ['astra-x-uav', 'astra-x-uav'] and len(sink.deliveries) == 1
    assert [a.ok for a in sink.attempts] == [True, False]


def test_the_hook_runs_outside_the_lock_so_a_blocked_request_does_not_block_readers():
    sink = MockSink()
    entered, release = threading.Event(), threading.Event()

    def hook(entity):
        entered.set()
        release.wait(5)

    sink.set_hook(hook)
    worker = threading.Thread(target=lambda: sink.publish_entity(entity_at(0)))
    worker.start()
    assert entered.wait(5)
    assert sink.calls == 0 and sink.entities() == {}                  # readable while a request is held open
    release.set()
    worker.join(5)
    assert not worker.is_alive() and sink.calls == 1


def test_closing_refuses_later_publishes_and_counts_the_closes():
    sink = MockSink()
    sink.close()
    sink.close()
    assert sink.close_calls == 2
    with pytest.raises(FatalSinkError, match='closed'):
        sink.publish_entity(entity_at(0))


def test_objects_are_kept_and_the_checksum_is_the_sha256():
    import hashlib
    sink = MockSink()
    info = sink.upload_object('astra/m/x.jpg', b'bytes')
    assert info.path == 'astra/m/x.jpg' and info.size_bytes == 5
    assert info.checksum == hashlib.sha256(b'bytes').hexdigest()
    assert sink.objects() == {'astra/m/x.jpg': b'bytes'}
    sink.go_down()
    with pytest.raises(RetryableSinkError):
        sink.upload_object('a', b'b')


def test_the_legacy_write_works_on_top_of_it():
    sink = MockSink()
    assert sink.write('entity', entity_at(0)) is True
    sink.go_down()
    assert sink.write('entity', entity_at(1)) is False


def test_many_threads_can_deliver_at_once():
    sink = MockSink()
    clock = FakeClock()

    def work(n):
        for i in range(50):
            sink.publish_entity(uav_at(clock, entity_id=f'astra-x-{n}-{i}'))

    threads = [threading.Thread(target=work, args=(n,)) for n in range(8)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(10)
    assert sink.calls == 400 and len(sink.deliveries) == 400 and len(sink.entities()) == 400
