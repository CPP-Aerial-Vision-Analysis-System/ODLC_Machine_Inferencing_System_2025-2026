"""
lattice_bridge/test/test_timeutil.py -- PURPOSE: prove the time rules: naive datetimes are rejected, every
output is UTC text with a Z, the text round-trips exactly, and nothing accepts a time that is not a time.
"""

from datetime import datetime, timedelta, timezone

import pytest

from lattice_bridge.timeutil import format_utc, from_unix_seconds, parse_utc, plus_seconds, require_aware
from support import NOW, rng

PLUS_2 = timezone(timedelta(hours=2))
MINUS_7 = timezone(timedelta(hours=-7))


def test_format_has_six_fraction_digits_and_z():
    assert format_utc(NOW) == '2026-10-24T15:30:00.000000Z'
    assert format_utc(NOW + timedelta(microseconds=5)) == '2026-10-24T15:30:00.000005Z'


def test_naive_datetime_is_rejected_everywhere():
    naive = datetime(2026, 10, 24, 15, 30, 0)
    with pytest.raises(ValueError, match='naive'):
        format_utc(naive)
    with pytest.raises(ValueError, match='naive'):
        require_aware('fix_time', naive)
    with pytest.raises(ValueError, match='naive'):
        plus_seconds(naive, 1)


@pytest.mark.parametrize('bad', [None, 5, 1.5, '2026-10-24T15:30:00Z', b'x', (2026, 10, 24), object()])
def test_non_datetimes_are_rejected(bad):
    with pytest.raises(ValueError, match='datetime'):
        require_aware('t', bad)


@pytest.mark.parametrize('zone', [PLUS_2, MINUS_7])
def test_other_time_zones_are_converted_to_utc(zone):
    local = datetime(2026, 10, 24, 18, 30, 0, tzinfo=zone)
    assert require_aware('t', local).utcoffset() == timedelta(0)
    assert format_utc(local) == format_utc(local.astimezone(timezone.utc))
    assert format_utc(local).endswith('Z')
    assert parse_utc(format_utc(local)) == local


def test_text_round_trips_exactly():
    r = rng(1)
    for _ in range(500):
        moment = datetime(2000, 1, 1, tzinfo=timezone.utc) + timedelta(
            microseconds=r.randrange(0, 3_000_000_000_000_000))
        assert parse_utc(format_utc(moment)) == moment


def test_parse_accepts_the_forms_lattice_documents():
    assert parse_utc('2025-04-07T00:19:47.706196Z') == datetime(2025, 4, 7, 0, 19, 47, 706196, tzinfo=timezone.utc)
    assert parse_utc('2025-04-07T00:19:47Z') == datetime(2025, 4, 7, 0, 19, 47, tzinfo=timezone.utc)
    assert parse_utc('2025-04-07T00:19:47.7Z').microsecond == 700000
    assert parse_utc('2025-04-07T00:19:47.706Z').microsecond == 706000


@pytest.mark.parametrize('bad', [
    '', 'now', '2026-10-24T15:30:00', '2026-10-24T15:30:00+00:00', '2026-10-24T15:30:00+02:00',
    '2026-10-24 15:30:00Z', '2026-10-24T15:30:00z', '2026-13-24T15:30:00Z', '2026-10-32T15:30:00Z',
    '2026-10-24T25:30:00Z', '2026-10-24T15:30:60Z', '2026-10-24T15:30:00.1234567Z', '2026-10-24T15:30:00.Z',
    ' 2026-10-24T15:30:00Z', '2026-10-24T15:30:00Z\n', '٢٠٢٦-10-24T15:30:00Z', '2026-10-24T15:30:00Zjunk',
    None, 5, b'2026-10-24T15:30:00Z', ['2026-10-24T15:30:00Z'],
])
def test_parse_rejects_everything_else(bad):
    with pytest.raises(ValueError):
        parse_utc(bad)


def test_parse_error_messages_stay_short():
    with pytest.raises(ValueError) as info:
        parse_utc('x' * 10000)
    assert len(str(info.value)) < 200


def test_from_unix_seconds():
    assert from_unix_seconds(1761319800) == datetime(2025, 10, 24, 15, 30, 0, tzinfo=timezone.utc)
    assert from_unix_seconds(1761319800.5).microsecond == 500000


@pytest.mark.parametrize('bad', [0, -1, 1.0, 946684799, 4102444801, float('nan'), float('inf'), -float('inf'),
                                 True, False, None, '1761319800', 10 ** 400])
def test_from_unix_seconds_rejects_garbage(bad):
    with pytest.raises(ValueError):
        from_unix_seconds(bad)


def test_plus_seconds():
    assert plus_seconds(NOW, 120) == NOW + timedelta(seconds=120)
    assert plus_seconds(NOW, -0.5) == NOW - timedelta(milliseconds=500)
    local = datetime(2026, 10, 24, 17, 30, 0, tzinfo=PLUS_2)             # same instant as NOW
    assert plus_seconds(local, 0) == NOW and plus_seconds(local, 0).utcoffset() == timedelta(0)
    for bad in (float('nan'), float('inf'), None, '5', True, 10 ** 30):
        with pytest.raises(ValueError):
            plus_seconds(NOW, bad)
