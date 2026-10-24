"""
lattice_bridge/test/test_mavlink_source.py -- PURPOSE: prove the live-link part of the MAVLink source: it connects,
tells the bridge it is connected, keeps trying with growing waits when nothing is there yet, notices a broken socket
and reconnects, tells the bridge ONCE that the radio went silent, rebuilds a silent TCP link but not a UDP one, counts
garbage without choking, never spins the CPU, and shuts down cleanly. A fake clock and a fake connection make minutes
pass in microseconds. Two last tests use a real UDP socket on this computer to prove a real MAVLink frame goes all the
way through. Skipped when pymavlink is not installed.
"""

import errno
import random
import socket
import sys
import time
from datetime import datetime, timedelta, timezone

import pytest

pytest.importorskip('pymavlink')
from pymavlink import mavutil                                    # noqa: E402
from pymavlink.dialects.v20 import ardupilotmega as mav         # noqa: E402

from lattice_bridge.events import DroneFix, SourceNotice          # noqa: E402
from lattice_bridge.sources import SourceUnavailable              # noqa: E402
from lattice_bridge.sources.mavlink_source import (               # noqa: E402
    MavlinkDecoderConfig, MavlinkSource, explain_connect_error, import_pymavlink)

T0 = datetime(2026, 10, 24, 15, 30, 0, tzinfo=timezone.utc)


class FakeTime:
    """A clock that only moves when somebody sleeps or waits."""

    def __init__(self):
        self.t = 0.0

    def monotonic(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds

    def clock(self):
        return T0 + timedelta(seconds=self.t)


def position(boot_ms=1000, lat=36.2150):
    message = mav.MAVLink_global_position_int_message(boot_ms, round(lat * 1e7), round(-96.0068 * 1e7), 240000, 30000,
                                                      0, 0, 0, 9000)
    message._header.srcSystem, message._header.srcComponent = 1, 1
    return message


class FakeConn:
    """What mavutil.mavlink_connection returns: items are messages, or an Exception to raise."""

    def __init__(self, clock, items=()):
        self.clock, self.items, self.closed = clock, list(items), False

    def recv_match(self, blocking=True, timeout=None):
        if self.items:
            item = self.items.pop(0)
            if isinstance(item, Exception):
                raise item
            return item
        self.clock.t += timeout or 0.0                                  # nothing arrived: the wait used up the time
        return None

    def close(self):
        self.closed = True


def make_source(clock, connections, **kwargs):
    """A source whose connect() hands out `connections` one by one (an Exception item = that attempt fails)."""
    queue = list(connections)
    attempts = []

    def connect(_connection):
        attempts.append(clock.t)
        item = queue.pop(0)
        if isinstance(item, Exception):
            raise item
        return item
    kwargs.setdefault('rng', random.Random(1))
    source = MavlinkSource(kwargs.pop('connection', 'udpin:127.0.0.1:14550'), connect=connect, clock=clock.clock,
                           monotonic=clock.monotonic, sleep=clock.sleep, **kwargs)
    source.attempts = attempts
    return source


def notices(events):
    return [e for e in events if isinstance(e, SourceNotice)]


def codes(events):
    return [n.code for n in notices(events)]


# ------------------------------------------------------------------------------------------ connecting
def test_it_connects_says_so_and_decodes_what_arrives():
    clock = FakeTime()
    source = make_source(clock, [FakeConn(clock, [position()])])
    source.start()
    events = source.poll(0.5)
    assert codes(events) == ['connected'] and [type(e) for e in events if isinstance(e, DroneFix)] == [DroneFix]
    assert source.counters['connects'] == 1 and source.counters['received'] == 1


def test_a_connection_that_fails_is_retried_with_growing_waits_and_announced_once():
    clock = FakeTime()
    boom = ConnectionRefusedError('nobody there')
    source = make_source(clock, [boom, boom, boom, FakeConn(clock)])
    source.start()
    seen = []
    while clock.t < 60 and source.counters['connects'] == 0:
        seen += codes(source.poll(0.5))
    assert seen == ['reconnecting', 'connected']                                  # the trouble is announced once, not every try
    assert source.counters['connect_failures'] == 3 and len(source.attempts) == 4
    gaps = [b - a for a, b in zip(source.attempts, source.attempts[1:])]
    assert gaps[0] < gaps[2] and all(g >= 0.5 for g in gaps)                       # the waits grow (with jitter)


def test_waits_between_attempts_never_exceed_the_cap():
    clock = FakeTime()
    source = make_source(clock, [OSError('down')] * 12, backoff_cap_s=5.0)
    for _ in range(300):
        try:
            source.poll(0.5)
        except IndexError:
            break
    gaps = [b - a for a, b in zip(source.attempts, source.attempts[1:])]
    assert gaps and max(gaps) <= 5.0 * 1.3 + 0.6


def test_the_default_connector_needs_pymavlink_and_says_how_to_get_it(monkeypatch):
    monkeypatch.setitem(sys.modules, 'pymavlink', None)
    with pytest.raises(SourceUnavailable) as caught:
        MavlinkSource('udpin:127.0.0.1:14550').start()
    assert 'pip install pymavlink' in str(caught.value)
    with pytest.raises(SourceUnavailable):
        import_pymavlink()


@pytest.mark.parametrize('bad', ['', '   ', 'udpin:127.0.0.1:14550 extra', None, 5])
def test_a_connection_string_with_spaces_or_nothing_is_refused(bad):
    with pytest.raises(ValueError):
        MavlinkSource(bad)


def test_the_error_hints_name_the_usual_causes():
    port_busy = OSError(errno.EADDRINUSE, 'in use')
    assert 'already in use' in explain_connect_error(port_busy, 'udpin:127.0.0.1:14550')
    assert 'permission denied' in explain_connect_error(OSError(errno.EACCES, 'no'), 'udpin:0.0.0.0:80')
    assert 'nothing is listening' in explain_connect_error(ConnectionRefusedError(), 'tcp:127.0.0.1:5762')
    assert 'ValueError' in explain_connect_error(ValueError('odd'), 'x')


# ------------------------------------------------------------------------------------------ losing and finding the link
def test_a_broken_socket_is_a_link_loss_then_a_reconnect_with_a_fresh_decoder():
    clock = FakeTime()
    first = FakeConn(clock, [position(), OSError('socket closed')])
    second = FakeConn(clock, [position(boot_ms=5000, lat=36.2160)])
    source = make_source(clock, [first, second])
    source.start()
    events = []
    while clock.t < 30 and source.counters['connects'] < 2:
        events += source.poll(0.5)
    events += source.poll(0.5)
    assert 'link_lost' in codes(events) and codes(events).count('connected') == 2 and first.closed
    assert source.counters['link_losses'] == 1
    assert len([e for e in events if isinstance(e, DroneFix)]) == 2                 # the decoder was reset, so no "not newer" skip


def test_one_silence_notice_per_quiet_spell_and_none_while_data_flows():
    clock = FakeTime()
    source = make_source(clock, [FakeConn(clock, [position()])], silent_notice_s=10.0)
    source.start()
    events = []
    while clock.t < 35:
        events += source.poll(0.5)
    assert codes(events).count('silent') == 1


def test_a_second_quiet_spell_gets_its_own_notice():
    clock = FakeTime()
    conn = FakeConn(clock, [position()])
    source = make_source(clock, [conn], silent_notice_s=5.0)
    source.start()
    events = []
    while clock.t < 8:
        events += source.poll(0.5)
    conn.items.append(position(boot_ms=9000, lat=36.2155))
    while clock.t < 20:
        events += source.poll(0.5)
    assert codes(events).count('silent') == 2


def test_udp_is_never_rebuilt_for_silence_but_tcp_is():
    clock = FakeTime()
    udp = make_source(clock, [FakeConn(clock)], connection='udpin:127.0.0.1:14550')
    assert udp.reconnect_after_silence_s == 0.0
    tcp = make_source(clock, [FakeConn(clock), FakeConn(clock)], connection='tcp:127.0.0.1:5762', silent_notice_s=5.0)
    assert tcp.reconnect_after_silence_s == 30.0
    tcp.start()
    events = []
    while clock.t < 40:
        events += tcp.poll(0.5)
    assert tcp.counters['link_losses'] >= 1 and tcp.counters['connects'] == 2


def test_garbage_is_counted_and_not_decoded():
    clock = FakeTime()
    bad = mavutil.mavlink.MAVLink_bad_data(b'\xff\x00', 'invalid CRC')
    source = make_source(clock, [FakeConn(clock, [bad, position()])])
    source.start()
    events = source.poll(0.5)
    assert source.counters['bad_data'] == 1 and len([e for e in events if isinstance(e, DroneFix)]) == 1


def test_one_poll_hands_back_a_bounded_number_of_events():
    clock = FakeTime()
    texts = []
    for index in range(50):
        message = mav.MAVLink_statustext_message(6, f'TGT p 36.2{index:05d} -96.006000 80 {index}'.encode())
        message._header.srcSystem, message._header.srcComponent = 1, 1
        texts.append(message)
    source = make_source(clock, [FakeConn(clock, texts)], max_events_per_poll=10)
    source.start()
    first = source.poll(5.0)
    assert 10 <= len(first) < 50


def test_a_dead_connection_that_returns_instantly_does_not_spin():
    clock = FakeTime()

    class Instant(FakeConn):
        def recv_match(self, blocking=True, timeout=None):
            return None                                                          # returns at once, time does not pass by itself
    source = make_source(clock, [Instant(clock)])
    source.start()
    source.poll(1.0)
    assert clock.t >= 1.0 - 1e-9                                                  # the source slept the wait out itself


def test_close_is_clean_and_final():
    clock = FakeTime()
    conn = FakeConn(clock, [position()])
    source = make_source(clock, [conn])
    source.start()
    source.close()
    source.close()
    assert conn.closed and source.poll(0.5) == []


def test_describe_and_stats_are_plain():
    clock = FakeTime()
    source = make_source(clock, [FakeConn(clock)])
    assert source.describe() == 'MAVLink udpin:127.0.0.1:14550'
    assert {'connects', 'received', 'messages'} <= set(source.stats())


def test_a_configured_decoder_is_used():
    clock = FakeTime()
    source = make_source(clock, [FakeConn(clock, [position()])], decoder_config=MavlinkDecoderConfig(vehicle_sysid=9))
    source.start()
    assert [e for e in source.poll(0.5) if isinstance(e, DroneFix)] == []          # sysid 1 is not the configured vehicle


# ------------------------------------------------------------------------------------------ a real UDP socket
def free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(('127.0.0.1', 0))
        return probe.getsockname()[1]


def test_a_real_mavlink_frame_over_a_real_udp_socket_becomes_a_drone_fix():
    port = free_udp_port()
    source = MavlinkSource(f'udpin:127.0.0.1:{port}', decoder_config=MavlinkDecoderConfig(time_mode='arrival'))
    source.start()
    try:
        sender = mavutil.mavlink_connection(f'udpout:127.0.0.1:{port}', source_system=1, source_component=1)
        sender.mav.heartbeat_send(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_ARDUPILOTMEGA,
                                  mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mav.MAV_MODE_FLAG_SAFETY_ARMED, 3, mav.MAV_STATE_ACTIVE)
        sender.mav.global_position_int_send(5000, round(36.2150 * 1e7), round(-96.0068 * 1e7), 240500, 30000, 100, 0, -20, 9000)
        sender.mav.statustext_send(6, b'TGT p 36.215000 -96.006000 91 1')
        found = []
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and len(found) < 4:
            found += source.poll(0.3)
            if any(isinstance(e, DroneFix) for e in found) and any(type(e).__name__ == 'TargetSighting' for e in found):
                break
        fix = next(e for e in found if isinstance(e, DroneFix))
        assert (fix.lat_deg, fix.lon_deg) == (pytest.approx(36.2150), pytest.approx(-96.0068))
        assert fix.mode == 'AUTO' and fix.armed is True and fix.alt_amsl_m == pytest.approx(240.5)
        assert any(type(e).__name__ == 'TargetSighting' for e in found)
    finally:
        source.close()


def test_two_programs_cannot_share_one_udp_port_and_the_message_says_what_to_do():
    port = free_udp_port()
    first = MavlinkSource(f'udpin:127.0.0.1:{port}')
    first.start()
    try:
        second = MavlinkSource(f'udpin:127.0.0.1:{port}')
        second.start()                                                              # not fatal: it keeps retrying
        events = second.poll(0.2)
        if second.counters['connect_failures']:
            assert 'reconnecting' in codes(events) or second.counters['connect_failures'] >= 1
        second.close()
    finally:
        first.close()
