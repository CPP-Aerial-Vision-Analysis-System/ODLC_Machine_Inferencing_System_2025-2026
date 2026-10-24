"""
lattice_bridge/timeutil.py -- PURPOSE: the only place that turns times into text and back. Lattice wants
UTC time stamps written like "2026-10-24T15:30:00.000000Z" (ISO 8601, always UTC, always the letter Z at the
end). A time without a time zone ("naive") is a classic source of silent bugs (is it local time? UTC?), so
this module REFUSES naive times: every function that takes a time raises ValueError for one.

Rules:
  * a time zone aware datetime in any zone is accepted and converted to UTC (an instant is an instant);
  * a naive datetime is rejected with a clear message;
  * nothing here reads the clock. Callers pass the times in, so every result is repeatable in tests.

Pure Python (datetime, math, re): no ROS, no I/O.
"""

from __future__ import annotations

import re
from datetime import datetime, timedelta, timezone

from .geo import is_real_number

UTC = timezone.utc

# Exactly what format_utc() writes (6 fraction digits) or what Lattice's own examples show (1 to 6 digits).
_UTC_TEXT = re.compile(
    r'(?P<y>[0-9]{4})-(?P<mo>[0-9]{2})-(?P<d>[0-9]{2})T'
    r'(?P<h>[0-9]{2}):(?P<mi>[0-9]{2}):(?P<s>[0-9]{2})(?:\.(?P<f>[0-9]{1,6}))?Z'
)

#: Unix time stamps outside this window (year 2000 to year 2100) are treated as garbage.
_UNIX_MIN = 946684800.0
_UNIX_MAX = 4102444800.0


def require_aware(name: str, value) -> datetime:
    """Return `value` as a UTC datetime, or raise ValueError.

    `name` is only used in the error message (e.g. "fix_time"). A naive datetime is rejected because
    nobody can tell which time zone it was meant to be in.
    """
    if not isinstance(value, datetime):
        raise ValueError(f'{name} must be a datetime with a time zone, got {type(value).__name__}')
    if value.tzinfo is None or value.tzinfo.utcoffset(value) is None:
        raise ValueError(f'{name} is a naive datetime (no time zone); pass a timezone-aware UTC time')
    return value.astimezone(UTC)


def format_utc(value: datetime) -> str:
    """Write a datetime as Lattice text: 'YYYY-MM-DDTHH:MM:SS.ffffffZ' (always UTC, 6 fraction digits)."""
    moment = require_aware('time', value)
    return moment.strftime('%Y-%m-%dT%H:%M:%S.%f') + 'Z'


def parse_utc(text) -> datetime:
    """Read Lattice-style UTC text back into an aware datetime; ValueError if it is not exactly that form.

    Accepted: 'YYYY-MM-DDTHH:MM:SS[.f{1,6}]Z'. Rejected: no Z, an offset like +02:00, a space instead of
    the T, impossible dates (month 13), leap seconds (second 60), anything that is not a str.
    """
    if not isinstance(text, str):
        raise ValueError(f'time text must be a str, got {type(text).__name__}')
    match = _UTC_TEXT.fullmatch(text)
    if match is None:
        raise ValueError(f'not a UTC time like 2026-10-24T15:30:00.000000Z: {text[:40]!r}')
    fraction = (match.group('f') or '')
    micro = int((fraction + '000000')[:6])
    try:
        return datetime(
            int(match.group('y')), int(match.group('mo')), int(match.group('d')),
            int(match.group('h')), int(match.group('mi')), int(match.group('s')), micro, tzinfo=UTC,
        )
    except ValueError as exc:
        raise ValueError(f'impossible date or time {text[:40]!r}: {exc}') from None


def from_unix_seconds(seconds) -> datetime:
    """Seconds since 1970-01-01 UTC (a ROS stamp, a MAVLink time_unix_usec / 1e6) -> aware UTC datetime.

    Raises ValueError for NaN, infinity, non-numbers and anything outside the years 2000..2100 (a clock
    that was never set reads near 1970, and 0 is the usual "no time" value).
    """
    if not is_real_number(seconds):
        raise ValueError(f'unix time must be a finite number, got {seconds!r}'[:80])
    if not _UNIX_MIN <= seconds <= _UNIX_MAX:
        raise ValueError(f'unix time {seconds!r} is outside the years 2000..2100 (clock not set?)')
    return datetime.fromtimestamp(float(seconds), tz=UTC)


def plus_seconds(value: datetime, seconds: float) -> datetime:
    """`value` + `seconds` as a UTC datetime (seconds may be fractional; NaN/inf raise ValueError)."""
    moment = require_aware('time', value)
    if not is_real_number(seconds):
        raise ValueError(f'seconds must be a finite number, got {seconds!r}'[:80])
    try:
        return moment + timedelta(seconds=float(seconds))
    except OverflowError:
        raise ValueError('time arithmetic overflow') from None
