"""
lattice_bridge/test/test_publisher_retry.py -- PURPOSE: prove how the Publisher reacts when the sink fails, with a
fake clock so that minutes of back-off take microseconds: the exact back-off schedule (1, 2, 4 ... 60 s), the jitter
bounds, the server's Retry-After hint, one probe per wait, a success resetting the schedule, a rejected entity
being dropped while the rest carry on, bad credentials causing ONE request a minute (no retry storm), unexpected
sink exceptions being survived, and the log lines being few.
"""

import logging
import random

import pytest

from lattice_bridge.publisher import PublisherConfig, backoff_delay
from lattice_bridge.sinks.base import AuthSinkError, ConfigSinkError, FatalSinkError, RetryableSinkError, SinkError
from support_net import FakeClock, assert_accounting, make_publisher, run_for, target_at, uav_at

CFG = PublisherConfig()


# ------------------------------------------------------------------------------------ the pure formula
def test_the_schedule_doubles_from_one_second_and_stops_at_a_minute():
    got = [backoff_delay(n, CFG, 0.5) for n in range(1, 10)]
    assert got == [1, 2, 4, 8, 16, 32, 60, 60, 60]


def test_the_jitter_is_plus_or_minus_twenty_percent():
    assert backoff_delay(1, CFG, 0.0) == pytest.approx(0.8)
    assert backoff_delay(1, CFG, 1.0) == pytest.approx(1.2)
    assert backoff_delay(3, CFG, 0.0) == pytest.approx(3.2)
    assert backoff_delay(3, CFG, 1.0) == pytest.approx(4.8)
    assert backoff_delay(9, CFG, 0.0) == pytest.approx(48.0)          # the cap applies BEFORE the jitter
    assert backoff_delay(9, CFG, 1.0) == pytest.approx(72.0)


def test_random_draws_always_stay_inside_the_bounds_and_actually_vary():
    rng = random.Random(7)
    for failures in range(1, 12):
        base = backoff_delay(failures, CFG, 0.5)
        draws = [backoff_delay(failures, CFG, rng.random()) for _ in range(500)]
        assert all(0.8 * base - 1e-9 <= d <= 1.2 * base + 1e-9 for d in draws)
        assert max(draws) - min(draws) > 0.3 * base                       # the spread is real, not constant


def test_the_numbers_come_from_the_config():
    cfg = PublisherConfig(backoff_initial_s=0.5, backoff_multiplier=3.0, backoff_max_s=10.0, jitter_fraction=0.0)
    assert [backoff_delay(n, cfg, 0.123) for n in range(1, 6)] == [0.5, 1.5, 4.5, 10.0, 10.0]


@pytest.mark.parametrize('failures', [0, -5, 1])
def test_a_nonsense_failure_count_counts_as_the_first_failure(failures):
    assert backoff_delay(failures, CFG, 0.5) == 1.0


def test_a_huge_failure_count_cannot_overflow():
    assert backoff_delay(10 ** 9, CFG, 0.5) == 60.0
    wild = PublisherConfig(backoff_multiplier=10.0, backoff_max_s=86400.0)
    assert backoff_delay(10 ** 9, wild, 0.5) == 86400.0


@pytest.mark.parametrize('draw', [float('nan'), float('inf'), None, 'x', -3.0, 7.0])
def test_a_broken_random_draw_is_clamped_not_fatal(draw):
    delay = backoff_delay(1, CFG, draw)
    assert 0.8 <= delay <= 1.2


def test_the_servers_retry_after_hint_can_only_lengthen_the_wait():
    assert backoff_delay(1, CFG, 0.5, retry_after_s=30) == 30
    assert backoff_delay(3, CFG, 0.5, retry_after_s=1) == 4            # shorter than our own: ignored
    assert backoff_delay(1, CFG, 0.5, retry_after_s=10 ** 6) == CFG.retry_after_cap_s
    for junk in (float('nan'), float('inf'), -5, 0, True, None, 'soon'):
        assert backoff_delay(1, CFG, 0.5, retry_after_s=junk) == 1.0


# ------------------------------------------------------------------------------------ in the publisher
def fail_rounds(pub, clock, rounds):
    """Pump, read how long the publisher says to wait, wait exactly that long; returns the waits."""
    waits = []
    for _ in range(rounds):
        assert pub.pump() == 1
        wait = pub.health()['next_attempt_in_s']
        waits.append(wait)
        clock.advance(wait + 1e-6)
    return waits


def test_the_publisher_follows_the_schedule_exactly():
    pub, sink, clock = make_publisher()
    sink.go_down()
    pub.submit(uav_at(clock, ttl_s=3600))
    assert fail_rounds(pub, clock, 9) == pytest.approx([1, 2, 4, 8, 16, 32, 60, 60, 60], abs=1e-3)
    health = pub.health()
    assert (health['failed_retryable'], health['consecutive_failures'], health['sent']) == (9, 9, 0)
    assert (health['connected'], health['state'], health['queue_depth']) == (False, 'offline', 1)
    assert health['retried'] == 8                                      # every attempt but the first is a retry
    assert_accounting(health)


def test_nothing_is_sent_during_a_wait_and_exactly_one_probe_follows():
    pub, sink, clock = make_publisher()
    sink.go_down()
    pub.submit(uav_at(clock))
    pub.pump()
    clock.advance(0.99)
    assert pub.pump() == 0 and sink.calls == 1
    assert pub.next_delay() == pytest.approx(0.01)
    clock.advance(0.011)
    assert pub.pump() == 1 and sink.calls == 2


def test_many_waiting_entities_cause_a_single_probe_not_a_burst():
    pub, sink, clock = make_publisher()
    sink.go_down()
    for n in range(10):
        pub.submit(uav_at(clock, entity_id=f'astra-x-{n}'))
    assert pub.pump() == 1 and sink.calls == 1
    assert pub.health()['queue_depth'] == 10


def test_the_newest_entity_is_the_one_that_probes():
    pub, sink, clock = make_publisher()
    sink.go_down()
    pub.submit(uav_at(clock, entity_id='astra-x-old', age_s=60))
    pub.submit(uav_at(clock, entity_id='astra-x-new'))
    pub.pump()
    assert [a.entity_id for a in sink.attempts] == ['astra-x-new']


def test_a_success_resets_the_schedule():
    pub, sink, clock = make_publisher()
    sink.go_down()
    pub.submit(uav_at(clock))
    assert fail_rounds(pub, clock, 3) == pytest.approx([1, 2, 4], abs=1e-3)
    sink.come_back()
    assert pub.pump() == 1                                             # the probe succeeds
    health = pub.health()
    assert (health['connected'], health['consecutive_failures'], health['state']) == (True, 0, 'online')
    sink.go_down()
    clock.advance(2)
    pub.submit(uav_at(clock, lat=36.9))
    assert fail_rounds(pub, clock, 2) == pytest.approx([1, 2], abs=1e-3)


def test_failed_entities_stay_queued_and_the_error_is_reported():
    pub, sink, clock = make_publisher()
    sink.go_down(RetryableSinkError('connection refused', status=None))
    pub.submit(uav_at(clock))
    pub.pump()
    health = pub.health()
    assert health['queue_depth'] == 1 and health['sent'] == 0
    assert health['last_error'] == 'RetryableSinkError: connection refused'
    assert health['last_error_at'] is not None and health['last_success_at'] is None


def test_the_error_report_includes_the_http_status():
    pub, sink, clock = make_publisher()
    sink.go_down(RetryableSinkError('server error', status=503))
    pub.submit(uav_at(clock))
    pub.pump()
    assert pub.health()['last_error'] == 'RetryableSinkError: server error [HTTP 503]'


def test_a_newer_version_replaces_the_failed_one_during_the_outage():
    pub, sink, clock = make_publisher()
    sink.go_down()
    pub.submit(uav_at(clock, lat=36.1))
    pub.pump()
    clock.advance(2)
    assert pub.submit(uav_at(clock, lat=36.2)) == 'coalesced'
    sink.come_back()
    clock.advance(10)
    pub.pump()
    delivered = [d.entity['location']['position']['latitudeDegrees'] for d in sink.deliveries]
    assert delivered == [36.2]
    assert_accounting(pub.health())


def test_a_version_that_arrives_while_another_is_in_flight_wins_when_the_flight_fails():
    pub, sink, clock = make_publisher()
    newer = uav_at(clock, lat=36.9)                                      # same instant as the one in flight
    calls = []

    def hook(entity):
        calls.append(entity['location']['position']['latitudeDegrees'])
        if len(calls) == 1:
            clock.advance(1)
            assert pub.submit(uav_at(clock, lat=36.7)) == 'queued'       # arrives while the first is in flight
            raise RetryableSinkError('link dropped')

    sink.set_hook(hook)
    pub.submit(uav_at(clock, lat=36.1))
    pub.pump()
    health = pub.health()
    assert health['queue_depth'] == 1 and health['coalesced'] == 1       # the failed one was superseded
    sink.set_hook(None)
    clock.advance(5)
    pub.pump()
    assert [d.entity['location']['position']['latitudeDegrees'] for d in sink.deliveries] == [36.7]
    assert_accounting(pub.health())
    assert newer is not None


def test_older_data_arriving_while_another_is_in_flight_is_ignored():
    pub, sink, clock = make_publisher()
    answers = []

    def hook(entity):
        answers.append(pub.submit(uav_at(clock, age_s=30, lat=36.9)))    # measured before the one in flight

    sink.set_hook(hook)
    pub.submit(uav_at(clock, lat=36.1))
    pub.pump()
    assert answers == ['stale']
    assert pub.health()['stale'] == 1 and pub.health()['queue_depth'] == 0


def test_the_retry_after_hint_of_the_server_sets_the_wait():
    pub, sink, clock = make_publisher()
    sink.fail_next(RetryableSinkError('slow down', status=429, retry_after_s=30))
    pub.submit(uav_at(clock))
    pub.pump()
    assert pub.health()['next_attempt_in_s'] == pytest.approx(30)
    clock.advance(29)
    assert pub.pump() == 0
    clock.advance(1.001)
    assert pub.pump() == 1 and pub.health()['connected'] is True


def test_a_huge_retry_after_is_capped():
    pub, sink, clock = make_publisher(retry_after_cap_s=120.0)
    sink.fail_next(RetryableSinkError('later', status=503, retry_after_s=86400))
    pub.submit(uav_at(clock))
    pub.pump()
    assert pub.health()['next_attempt_in_s'] == pytest.approx(120)


@pytest.mark.parametrize('draw, expected', [(0.0, 0.8), (1.0, 1.2), (0.5, 1.0)])
def test_the_random_draw_reaches_the_first_wait(draw, expected):
    pub, sink, clock = make_publisher(rng=draw)
    sink.go_down()
    pub.submit(uav_at(clock))
    pub.pump()
    assert pub.health()['next_attempt_in_s'] == pytest.approx(expected)


def test_a_broken_random_generator_cannot_break_the_publisher():
    from lattice_bridge.publisher import Publisher
    clock = FakeClock()
    from lattice_bridge.sinks.mock_sink import MockSink
    sink = MockSink()
    sink.go_down()

    def broken():
        raise RuntimeError('rng exploded')

    pub = Publisher(sink, clock=clock.monotonic, wall=clock.now, sleep=clock.sleep, rng=broken)
    pub.submit(uav_at(clock))
    pub.pump()
    assert pub.health()['next_attempt_in_s'] == pytest.approx(1.0)


# ------------------------------------------------------------------------------------ fatal errors
def test_a_rejected_entity_is_dropped_and_the_others_carry_on_at_once():
    pub, sink, clock = make_publisher()
    sink.fail_next(FatalSinkError('bad request', status=400))
    pub.submit(uav_at(clock, entity_id='astra-x-b', age_s=0))
    pub.submit(uav_at(clock, entity_id='astra-x-a', age_s=10))
    assert pub.pump() == 2                                             # same pump: no waiting after a rejection
    assert sink.delivered_ids() == ['astra-x-a']
    health = pub.health()
    assert (health['dropped_fatal'], health['failed_fatal'], health['sent']) == (1, 1, 1)
    assert (health['connected'], health['state'], health['consecutive_failures']) == (True, 'online', 0)
    assert 'bad request' in health['last_error'] and 'FatalSinkError' in health['last_error']
    assert_accounting(health)


def test_a_rejection_after_an_outage_means_the_link_is_back():
    pub, sink, clock = make_publisher()
    sink.fail_next(RetryableSinkError('down'))
    pub.submit(uav_at(clock))
    pub.pump()
    clock.advance(2)
    sink.fail_next(FatalSinkError('bad request', status=400))
    pub.pump()
    assert pub.health()['connected'] is True and pub.health()['state'] == 'online'
    assert pub.health()['next_attempt_in_s'] is None                    # nothing left to wait for


# ------------------------------------------------------------------------------------ bad credentials
def test_bad_credentials_cause_one_request_not_a_storm():
    pub, sink, clock = make_publisher()
    sink.go_down(AuthSinkError('HTTP 401: token refused', status=401))
    pub.submit(uav_at(clock))
    assert pub.pump() == 1
    health = pub.health()
    assert (health['state'], health['connected'], health['queue_depth']) == ('auth_failed', False, 1)
    assert health['systemic_failures'] == 1 and health['dropped'] == 0
    assert health['next_attempt_in_s'] == pytest.approx(60)
    assert run_for(pub, clock, 59, step=0.25) == 0                       # silent for the whole cooldown
    assert sink.calls == 1


def test_the_probe_after_the_cooldown_delivers_the_waiting_entity_once_the_token_works():
    pub, sink, clock = make_publisher()
    sink.go_down(AuthSinkError('HTTP 401', status=401))
    pub.submit(uav_at(clock, ttl_s=3600))
    pub.pump()
    clock.advance(61)
    assert pub.pump() == 1 and sink.calls == 2                          # a second probe, still refused
    sink.come_back()                                                    # somebody fixed the token
    clock.advance(61)
    assert pub.pump() == 1
    assert pub.health()['state'] == 'online' and sink.delivered_ids() == ['astra-' + 'runway20261024t153000z-uav']


@pytest.mark.parametrize('error', [
    AuthSinkError('HTTP 403', status=403), ConfigSinkError('no endpoint'),
    FatalSinkError('certificate refused', systemic=True),
])
def test_ten_minutes_of_systemic_failure_make_about_ten_requests(error):
    pub, sink, clock = make_publisher()
    sink.go_down(error)
    pub.submit(uav_at(clock, ttl_s=3600))
    run_for(pub, clock, 600, step=0.5)
    assert 9 <= sink.calls <= 11
    assert pub.health()['queue_depth'] == 1                              # nothing was thrown away meanwhile


def test_the_state_name_tells_a_credential_problem_from_another_blockage():
    pub, sink, clock = make_publisher()
    sink.go_down(FatalSinkError('certificate refused', systemic=True))
    pub.submit(uav_at(clock))
    pub.pump()
    assert pub.health()['state'] == 'blocked'


def test_ten_minutes_of_plain_outage_make_a_handful_of_requests():
    pub, sink, clock = make_publisher()
    sink.go_down()
    pub.submit(uav_at(clock, ttl_s=3600))
    run_for(pub, clock, 600, step=0.5)
    assert 12 <= sink.calls <= 20                                        # 1+2+4+8+16+32 s then every minute


# ------------------------------------------------------------------------------------ odd sink behaviour
def test_an_unexpected_exception_in_the_sink_is_survived_and_treated_as_transient():
    pub, sink, clock = make_publisher()
    sink.set_hook(lambda entity: (_ for _ in ()).throw(RuntimeError('a bug in the sink')))
    pub.submit(uav_at(clock))
    assert pub.pump() == 1
    health = pub.health()
    assert health['unexpected_errors'] == 1 and health['queue_depth'] == 1 and health['state'] == 'offline'
    assert 'unexpected RuntimeError' in health['last_error']
    sink.set_hook(None)
    clock.advance(2)
    assert pub.pump() == 1 and pub.health()['sent'] == 1


def test_a_bare_sink_error_counts_as_retryable():
    pub, sink, clock = make_publisher()
    sink.go_down(SinkError('something'))
    pub.submit(uav_at(clock))
    pub.pump()
    assert pub.health()['state'] == 'offline' and pub.health()['queue_depth'] == 1


def test_an_error_text_with_control_characters_is_stored_as_one_clean_line():
    pub, sink, clock = make_publisher()
    sink.go_down(RetryableSinkError('first\nsecond\x00' + 'z' * 2000))
    pub.submit(uav_at(clock))
    pub.pump()
    text = pub.health()['last_error']
    assert '\n' not in text and '\x00' not in text and len(text) < 400


def test_the_inflight_marker_never_gets_stuck():
    pub, sink, clock = make_publisher()
    sink.go_down()
    pub.submit(uav_at(clock))
    for _ in range(5):
        pub.pump()
        clock.advance(100)
        assert pub.health()['inflight'] == 0


# ------------------------------------------------------------------------------------ the log
def test_an_outage_is_announced_once_and_the_recovery_once(caplog):
    pub, sink, clock = make_publisher()
    with caplog.at_level(logging.INFO, logger='lattice_bridge.publisher'):
        sink.go_down()
        pub.submit(uav_at(clock, ttl_s=3600))
        for _ in range(8):
            pub.pump()
            clock.advance(pub.health()['next_attempt_in_s'] + 1e-6)
        sink.come_back()
        pub.pump()
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    infos = [r for r in caplog.records if r.levelno == logging.INFO]
    assert len(warnings) <= 3, [r.getMessage() for r in warnings]        # the first, then one per log_interval_s
    assert 'not reachable' in warnings[0].getMessage() and 'next try in' in warnings[0].getMessage()
    assert len(infos) == 1 and 'reachable again' in infos[0].getMessage()


def test_a_wrong_token_is_reported_without_flooding_the_log(caplog):
    pub, sink, clock = make_publisher()
    sink.go_down(AuthSinkError('HTTP 401', status=401))
    with caplog.at_level(logging.INFO, logger='lattice_bridge.publisher'):
        pub.submit(uav_at(clock, ttl_s=3600))
        run_for(pub, clock, 600, step=0.5)
    errors = [r for r in caplog.records if r.levelno >= logging.ERROR]
    assert 1 <= len(errors) <= 11
    assert all('refuses every request' in r.getMessage() for r in errors)


def test_a_dropped_entity_is_logged_with_its_id(caplog):
    pub, sink, clock = make_publisher()
    sink.fail_next(FatalSinkError('bad request', status=400))
    with caplog.at_level(logging.WARNING, logger='lattice_bridge.publisher'):
        pub.submit(target_at(clock))
        pub.pump()
    assert any('tgt-person-1' in r.getMessage() and 'dropped' in r.getMessage() for r in caplog.records)


def test_a_log_handler_that_raises_does_not_stop_the_publisher():
    import logging as std_logging
    from lattice_bridge.publisher import Publisher
    from lattice_bridge.sinks.mock_sink import MockSink

    class Exploding(std_logging.Logger):
        def log(self, *args, **kwargs):
            raise RuntimeError('log disk full')

    clock = FakeClock()
    sink = MockSink()
    sink.go_down()
    pub = Publisher(sink, clock=clock.monotonic, wall=clock.now, sleep=clock.sleep, logger=Exploding('x'))
    pub.submit(uav_at(clock))
    pub.pump()
    sink.come_back()
    clock.advance(5)
    assert pub.pump() == 1 and pub.health()['sent'] == 1
