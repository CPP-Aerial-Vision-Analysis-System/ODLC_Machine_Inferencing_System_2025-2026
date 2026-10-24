"""
lattice_bridge/test/test_publisher_basics.py -- PURPOSE: prove the everyday rules of the Publisher with a fake
clock and the in-memory sink (no threads, no real time): submit() never touches the sink, only the latest version
of an entity waits, older data is ignored, unchanged content is skipped (and refreshed before it expires), each
entity is rate limited on its own, the newest data goes first, the queue is bounded and drops the oldest data,
bad input is refused without raising, expired entities are never sent, and the counters always add up.
"""

import copy
from datetime import timedelta

import pytest

from lattice_bridge.publisher import (
    CLOSED, COALESCED, DROPPED, QUEUED, REJECTED, STALE, SUBMIT_RESULTS, UNCHANGED, PublisherConfig,
)
from support_net import FakeClock, assert_accounting, drop_at, make_publisher, target_at, uav_at


def test_submit_files_the_entity_and_never_calls_the_sink():
    pub, sink, clock = make_publisher()
    assert pub.submit(uav_at(clock)) == QUEUED
    assert sink.calls == 0
    assert pub.health()['queue_depth'] == 1 and pub.health()['sent'] == 0


def test_pump_delivers_an_exact_copy_and_updates_the_health():
    pub, sink, clock = make_publisher()
    entity = uav_at(clock)
    pub.submit(entity)
    assert pub.pump() == 1
    assert sink.deliveries[0].entity == entity
    assert sink.deliveries[0].t == clock.now()
    health = pub.health()
    assert (health['sent'], health['queue_depth'], health['connected'], health['state']) == (1, 0, True, 'online')
    assert health['last_error'] is None and health['last_success_at'] is not None
    assert pub.pump() == 0
    assert_accounting(health)


def test_the_publisher_keeps_its_own_copy_of_what_it_was_given():
    pub, sink, clock = make_publisher()
    entity = uav_at(clock)
    original = copy.deepcopy(entity)
    pub.submit(entity)
    entity['location']['position']['latitudeDegrees'] = 0.5          # the caller reuses its dict
    entity['isLive'] = False
    pub.pump()
    assert sink.deliveries[0].entity == original


def test_every_submit_answer_is_one_of_the_documented_words():
    assert SUBMIT_RESULTS == {QUEUED, COALESCED, UNCHANGED, STALE, DROPPED, REJECTED, CLOSED}


# ------------------------------------------------------------------------------------ latest wins
def test_only_the_latest_version_of_an_entity_is_sent():
    pub, sink, clock = make_publisher()
    answers = []
    for i in range(3):
        answers.append(pub.submit(uav_at(clock, lat=36.2 + i * 0.001)))
        clock.advance(0.1)
    assert answers == [QUEUED, COALESCED, COALESCED]
    assert pub.pump() == 1
    assert sink.deliveries[0].entity['location']['position']['latitudeDegrees'] == 36.202
    health = pub.health()
    assert (health['enqueued'], health['coalesced'], health['sent']) == (3, 2, 1)
    assert_accounting(health)


def test_different_entities_do_not_replace_each_other():
    pub, sink, clock = make_publisher()
    pub.submit(uav_at(clock))
    pub.submit(target_at(clock, 'person', 1))
    pub.submit(target_at(clock, 'tent', 1))
    assert pub.pump() == 3
    assert len(set(sink.delivered_ids())) == 3 and pub.health()['coalesced'] == 0


def test_older_data_than_what_is_waiting_is_ignored():
    pub, sink, clock = make_publisher()
    clock.advance(10)
    assert pub.submit(uav_at(clock)) == QUEUED                            # measured at t=10
    assert pub.submit(uav_at(clock, age_s=5, lat=36.9)) == STALE          # measured at t=5
    pub.pump()
    assert sink.deliveries[0].entity['location']['position']['latitudeDegrees'] == 36.2143
    assert pub.health()['stale'] == 1
    assert_accounting(pub.health())


def test_older_data_than_what_was_delivered_is_ignored_too():
    pub, sink, clock = make_publisher()
    clock.advance(10)
    pub.submit(uav_at(clock))
    pub.pump()
    clock.advance(1)
    assert pub.submit(uav_at(clock, age_s=4, lat=36.9)) == STALE          # measured at t=7, delivered t=10
    assert pub.pump() == 0 and pub.health()['stale'] == 1


def test_equal_measurement_times_are_not_stale():
    pub, sink, clock = make_publisher()
    pub.submit(uav_at(clock, lat=36.1))
    assert pub.submit(uav_at(clock, lat=36.2)) == COALESCED               # same instant, later call wins
    pub.pump()
    assert sink.deliveries[0].entity['location']['position']['latitudeDegrees'] == 36.2


# ------------------------------------------------------------------------------------ unchanged content
def test_identical_content_with_new_time_stamps_is_skipped():
    pub, sink, clock = make_publisher()
    pub.submit(uav_at(clock))
    pub.pump()
    clock.advance(5)
    assert pub.submit(uav_at(clock)) == UNCHANGED                         # new sourceUpdateTime and expiryTime only
    assert pub.pump() == 0 and sink.calls == 1
    health = pub.health()
    assert health['unchanged'] == 1 and health['queue_depth'] == 0
    assert_accounting(health)


def test_texts_that_change_every_second_do_not_count_as_news():
    pub, sink, clock = make_publisher()
    pub.submit(uav_at(clock, battery_percent=87.0, link_age_s=0.8))
    pub.pump()
    clock.advance(5)
    quiet = uav_at(clock, battery_percent=86.0, link_age_s=1.9)           # other words, same health states
    assert pub.submit(quiet) == UNCHANGED
    clock.advance(1)
    low = uav_at(clock, battery_percent=20.0, link_age_s=1.9)             # battery state WARN: real news
    assert pub.submit(low) == QUEUED


def test_a_real_change_is_sent():
    pub, sink, clock = make_publisher()
    pub.submit(uav_at(clock))
    pub.pump()
    clock.advance(5)
    assert pub.submit(uav_at(clock, lat=36.3)) == QUEUED


def test_unchanged_content_is_refreshed_before_the_entity_can_expire():
    pub, sink, clock = make_publisher()                                   # the drone lives 120 s: refresh at 60 s
    pub.submit(uav_at(clock))
    pub.pump()
    clock.advance(59)
    assert pub.submit(uav_at(clock)) == UNCHANGED
    clock.advance(2)
    assert pub.submit(uav_at(clock)) == QUEUED                            # 61 s old: time to tell Lattice we live
    assert pub.pump() == 1 and sink.calls == 2


def test_the_refresh_fraction_is_configurable():
    pub, sink, clock = make_publisher(refresh_fraction=0.0)
    pub.submit(uav_at(clock))
    pub.pump()
    clock.advance(1)
    assert pub.submit(uav_at(clock)) == QUEUED                            # 0: never skip


def test_a_change_that_is_undone_before_sending_cancels_the_waiting_update():
    pub, sink, clock = make_publisher()
    pub.submit(uav_at(clock))
    pub.pump()
    clock.advance(2)
    assert pub.submit(uav_at(clock, lat=36.9)) == QUEUED                  # waits for its turn (rate limit)
    clock.advance(0.1)
    assert pub.submit(uav_at(clock)) == UNCHANGED                         # back to what Lattice already holds
    assert pub.pending() == []
    health = pub.health()
    assert health['coalesced'] == 1 and health['unchanged'] == 1
    assert_accounting(health)


def test_force_sends_even_when_nothing_changed():
    pub, sink, clock = make_publisher()
    pub.submit(drop_at(clock))
    pub.pump()
    clock.advance(1)
    assert pub.submit(drop_at(clock), force=True) == QUEUED
    assert pub.pump() == 1


def test_a_goodbye_submitted_by_the_caller_is_never_called_unchanged():
    pub, sink, clock = make_publisher()
    pub.submit(uav_at(clock))
    pub.pump()
    clock.advance(1)
    bye = uav_at(clock, is_live=False)
    assert pub.submit(bye) == QUEUED
    pub.pump()
    assert sink.deliveries[-1].entity['isLive'] is False
    assert sink.entities() == {}


# ------------------------------------------------------------------------------------ rate limit
def test_one_delivery_per_entity_per_interval_and_the_newest_goes_out():
    pub, sink, clock = make_publisher(min_interval_s=1.0)
    pub.submit(uav_at(clock, lat=36.1))
    assert pub.pump() == 1
    clock.advance(0.25)                                                   # exact binary fractions: no rounding noise
    pub.submit(uav_at(clock, lat=36.2))
    assert pub.pump() == 0
    assert pub.next_delay() == pytest.approx(0.75)
    clock.advance(0.25)
    pub.submit(uav_at(clock, lat=36.3))
    assert pub.pump() == 0
    assert pub.next_delay() == pytest.approx(0.5)
    clock.advance(0.5)
    assert pub.pump() == 1
    assert [d.entity['location']['position']['latitudeDegrees'] for d in sink.deliveries] == [36.1, 36.3]
    assert pub.health()['coalesced'] == 1


def test_the_limit_is_per_entity_so_others_are_not_held_up():
    pub, sink, clock = make_publisher(min_interval_s=1.0)
    pub.submit(uav_at(clock))
    pub.pump()
    clock.advance(0.1)
    pub.submit(uav_at(clock, lat=36.9))                                   # waits
    pub.submit(target_at(clock))                                          # a different entity: due now
    assert pub.pump() == 1
    assert sink.deliveries[-1].entity['entityId'].endswith('tgt-person-1')


def test_force_skips_the_rate_limit():
    pub, sink, clock = make_publisher(min_interval_s=10.0)
    pub.submit(uav_at(clock))
    pub.pump()
    clock.advance(0.1)
    pub.submit(uav_at(clock, lat=36.9), force=True)
    assert pub.pump() == 1


def test_a_zero_interval_means_no_rate_limit():
    pub, sink, clock = make_publisher(min_interval_s=0.0)
    for i in range(3):
        pub.submit(uav_at(clock, lat=36.1 + i * 0.01))
        assert pub.pump() == 1
        clock.advance(0.001)


def test_next_delay_is_none_when_nothing_waits_and_zero_when_something_is_due():
    pub, sink, clock = make_publisher()
    assert pub.next_delay() is None
    pub.submit(uav_at(clock))
    assert pub.next_delay() == 0.0


# ------------------------------------------------------------------------------------ order
def test_the_newest_data_is_sent_first():
    pub, sink, clock = make_publisher()
    for name, age in (('a', 30), ('b', 5), ('c', 20), ('d', 0)):
        pub.submit(uav_at(clock, entity_id=f'astra-x-{name}', age_s=age))
    assert pub.pending() == ['astra-x-d', 'astra-x-b', 'astra-x-c', 'astra-x-a']
    assert pub.pump() == 4
    assert sink.delivered_ids() == ['astra-x-d', 'astra-x-b', 'astra-x-c', 'astra-x-a']


def test_equal_times_are_sent_most_recently_filed_first():
    pub, sink, clock = make_publisher()
    for name in 'abc':
        pub.submit(uav_at(clock, entity_id=f'astra-x-{name}'))
    pub.pump()
    assert sink.delivered_ids() == ['astra-x-c', 'astra-x-b', 'astra-x-a']


def test_one_pump_sends_at_most_max_sends_per_pump():
    pub, sink, clock = make_publisher(max_sends_per_pump=2)
    for name in 'abcde':
        pub.submit(uav_at(clock, entity_id=f'astra-x-{name}'))
    assert pub.pump() == 2 and pub.pump() == 2 and pub.pump() == 1 and pub.pump() == 0


# ------------------------------------------------------------------------------------ bounded queue
def test_the_queue_is_bounded_and_the_oldest_data_is_dropped_first():
    pub, sink, clock = make_publisher(max_queue=3)
    answers = []
    for i in range(5):
        answers.append(pub.submit(uav_at(clock, entity_id=f'astra-x-{i}')))
        clock.advance(1)
    assert answers == [QUEUED] * 5                                        # each newcomer is newer than the victim
    assert pub.pending() == ['astra-x-4', 'astra-x-3', 'astra-x-2']
    health = pub.health()
    assert (health['queue_depth'], health['dropped_overflow'], health['dropped']) == (3, 2, 2)
    assert_accounting(health)
    pub.pump()
    assert sink.delivered_ids() == ['astra-x-4', 'astra-x-3', 'astra-x-2']


def test_a_newcomer_with_the_oldest_data_is_the_one_dropped():
    pub, sink, clock = make_publisher(max_queue=2)
    clock.advance(100)
    pub.submit(uav_at(clock, entity_id='astra-x-a'))
    pub.submit(uav_at(clock, entity_id='astra-x-b'))
    assert pub.submit(uav_at(clock, entity_id='astra-x-old', age_s=90)) == DROPPED
    assert sorted(pub.pending()) == ['astra-x-a', 'astra-x-b']
    assert pub.health()['dropped_overflow'] == 1
    assert_accounting(pub.health())


def test_replacing_a_waiting_entity_does_not_grow_the_queue():
    pub, sink, clock = make_publisher(max_queue=2)
    for _ in range(50):
        pub.submit(uav_at(clock, entity_id='astra-x-a'))
        clock.advance(0.01)
    assert pub.health()['queue_depth'] == 1 and pub.health()['dropped'] == 0


def test_memory_about_entities_is_bounded_too():
    pub, sink, clock = make_publisher(max_tracked=10, max_queue=1000, min_interval_s=0.0)
    for i in range(200):
        pub.submit(uav_at(clock, entity_id=f'astra-x-{i}'))
        pub.pump()
        clock.advance(0.01)
    assert len(pub._tracks) <= 10
    assert len(sink.deliveries) == 200


# ------------------------------------------------------------------------------------ bad input
@pytest.mark.parametrize('bad', [
    None, 'text', 5, 2.5, [], [{'entityId': 'x'}], b'bytes', {}, {'entityId': ''}, {'entityId': 5},
    {'entityId': None}, {'entityId': 'x' * 300}, {'entityId': 'x', 'v': float('nan')},
    {'entityId': 'x', 'v': float('inf')}, {'entityId': 'x', 'v': {1, 2}}, {'entityId': 'x', 'v': object()},
    {'entityId': 'x', 'blob': 'y' * (2 << 20)},
])
def test_things_that_are_not_entities_are_refused_without_raising(bad):
    pub, sink, clock = make_publisher()
    assert pub.submit(bad) == REJECTED
    assert pub.health()['rejected'] == 1 and pub.health()['queue_depth'] == 0
    assert pub.pump() == 0 and sink.calls == 0


def test_a_self_referencing_dict_is_refused():
    pub, sink, clock = make_publisher()
    loop = {'entityId': 'x'}
    loop['me'] = loop
    assert pub.submit(loop) == REJECTED


def test_a_dict_with_a_broken_getter_cannot_crash_submit():
    class Evil(dict):
        def get(self, *args, **kwargs):
            raise RuntimeError('boom')

    pub, sink, clock = make_publisher()
    assert pub.submit(Evil(entityId='x')) == REJECTED
    assert pub.health()['internal_errors'] == 1


def test_an_entity_without_times_is_still_delivered():
    pub, sink, clock = make_publisher()
    assert pub.submit({'entityId': 'astra-x-plain', 'isLive': True}) == QUEUED
    assert pub.pump() == 1                                                # the server, not we, judges the content


# ------------------------------------------------------------------------------------ expiry
def test_an_entity_that_expired_while_waiting_is_never_sent():
    pub, sink, clock = make_publisher()
    pub.submit(uav_at(clock))                                             # lives 120 s
    clock.advance(200)
    assert pub.pump() == 0 and sink.calls == 0
    health = pub.health()
    assert health['dropped_expired'] == 1 and health['queue_depth'] == 0
    assert_accounting(health)


def test_an_entity_about_to_expire_is_not_sent_either():
    pub, sink, clock = make_publisher(expiry_margin_s=1.0)
    pub.submit(target_at(clock, expiry_hours=0.5 / 3600))                 # expires in half a second
    assert pub.pump() == 0 and pub.health()['dropped_expired'] == 1


def test_long_lived_entities_survive_a_long_wait():
    pub, sink, clock = make_publisher()
    pub.submit(target_at(clock))                                          # 8 hours
    clock.advance(3600)
    assert pub.pump() == 1


# ------------------------------------------------------------------------------------ configuration
@pytest.mark.parametrize('kwargs', [
    {'max_queue': 0}, {'max_queue': True}, {'max_queue': 1.5}, {'max_queue': -3},
    {'min_interval_s': -1}, {'min_interval_s': float('nan')}, {'min_interval_s': True},
    {'backoff_initial_s': -0.1}, {'backoff_multiplier': 0.5}, {'backoff_multiplier': 50},
    {'backoff_max_s': 0.5, 'backoff_initial_s': 1.0}, {'jitter_fraction': 1.5}, {'jitter_fraction': -0.1},
    {'refresh_fraction': 2}, {'systemic_cooldown_s': -5}, {'max_sends_per_pump': 0}, {'max_tracked': 0},
    {'idle_poll_s': 0}, {'goodbye_ttl_s': 0}, {'ignore_paths': ['a.b']}, {'ignore_paths': ('',)},
    {'expiry_margin_s': float('inf')},
])
def test_nonsense_settings_are_refused_at_start_up(kwargs):
    with pytest.raises(ValueError):
        PublisherConfig(**kwargs)


def test_the_defaults_are_the_documented_ones():
    cfg = PublisherConfig()
    assert (cfg.min_interval_s, cfg.backoff_initial_s, cfg.backoff_multiplier, cfg.backoff_max_s) == (1.0, 1.0, 2.0, 60.0)
    assert cfg.jitter_fraction == 0.2 and cfg.systemic_cooldown_s == 60.0 and cfg.refresh_fraction == 0.5


def test_the_config_is_immutable():
    with pytest.raises(Exception):
        PublisherConfig().max_queue = 5


def test_a_sink_must_be_a_sink():
    from lattice_bridge.publisher import Publisher
    with pytest.raises(TypeError):
        Publisher(object())


def test_a_plain_write_close_object_is_accepted_as_a_sink(tmp_path):
    from lattice_bridge.publisher import Publisher
    from lattice_bridge.sinks.jsonl_sink import JsonlSink, read_records

    class Legacy:
        """Only the old two-method protocol."""
        def __init__(self, inner):
            self.inner = inner

        def write(self, kind, payload, t=None):
            return self.inner.write(kind, payload, t)

        def close(self):
            self.inner.close()

    clock = FakeClock()
    path = tmp_path / 'rec.jsonl'
    pub = Publisher(Legacy(JsonlSink(path)), clock=clock.monotonic, wall=clock.now, sleep=clock.sleep)
    pub.submit(uav_at(clock))
    assert pub.pump() == 1
    pub.stop(1)
    assert len(read_records(path).records) == 1


def test_clock_steps_forward_only_by_the_test():
    clock = FakeClock()
    assert clock.now() == clock.now()
    before = clock.now()
    clock.advance(3)
    assert clock.now() - before == timedelta(seconds=3)
