"""
lattice_bridge/sources/mavlink_source.py -- PURPOSE: read the drone's MAVLink stream (the same stream
Mission Planner shows) and turn it into bridge events. This is the source for the GROUND LAPTOP: in flight the
only link to the ground is the telemetry radio, so telemetry AND the Jetson's `TGT` target reports both arrive
here as MAVLink. In the simulator MAVProxy already sends the stream to UDP 127.0.0.1:14550.

Two classes with one job each:

  MavlinkDecoder  PURE. Takes one message object at a time (anything with get_type(), get_srcSystem(),
                  get_srcComponent() and the usual field attributes, so a real pymavlink message or a tiny
                  fake in a test) and returns events. All the knowledge about units, sentinel values and
                  clocks lives here:
                    GLOBAL_POSITION_INT  position (degrees x 1e7, mm, cm/s, centi-degrees) -> DroneFix
                    HEARTBEAT            flight mode + armed flag (only from the autopilot, never from a
                                         ground station or the Jetson's own heartbeat)
                    VFR_HUD / ATTITUDE   compass heading
                    SYS_STATUS           battery percent (-1 = unknown is ignored)
                    SYSTEM_TIME          the pair (unix time, time since boot) that turns the autopilot's
                                         time_boot_ms into UTC, if the autopilot's clock is trustworthy
                    STATUSTEXT           put chunks back together, then `TGT ...` reports and the
                                         "Servo 13 -> 1400" release line
                  Everything else is ignored.
  MavlinkSource   the connection: opens it (pymavlink, imported only when the source starts), reads with a
                  time limit, reconnects with growing waits when it breaks, notices a silent link, and
                  feeds every message to the decoder.

Time, the easy-to-get-wrong part: `time_boot_ms` is milliseconds since the AUTOPILOT booted, not a date.
It becomes UTC only via SYSTEM_TIME, and only if the autopilot's clock is right; the simulator's clock is
not (it fell 10 to 34 s behind real time in a recorded run). ClockJudge (sources/common.py) uses the
autopilot's time only while it agrees with the arrival time within 2 s and otherwise stamps the position with
the moment it arrived. Either way the result is a timezone-aware UTC datetime that never lies in the future.

pymavlink is imported lazily and only here (start()); the rest of the package, and the decoder, work without it.
"""

from __future__ import annotations

import errno
import logging
import math
import os
import random
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional, Tuple

from .. import constants as C
from ..events import DroneFix, Event, SourceNotice
from ..geo import is_real_number, require_lat_lon
from ..timeutil import from_unix_seconds
from . import SourceUnavailable
from .common import (
    ClockJudge, DroneStateAccumulator, StatusTextAssembler, TextEventDecoder, backoff_delay,
)

LOG = logging.getLogger('lattice_bridge.mavlink')

# --- MAVLink constants the decoder needs (written out so the decoder works without pymavlink) ----------
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_AUTOPILOT_INVALID = 8
MAV_AUTOPILOT_ARDUPILOTMEGA = 3
#: Heartbeats from these kinds of component are NOT the vehicle: antenna tracker, ground station, onboard
#: computer (the Jetson's MAVROS), gimbal, ADS-B, camera and the other peripherals.
NON_VEHICLE_TYPES = frozenset({5, 6, 18, 26, 27, 30, 31, 32, 33, 34, 36, 37, 38, 39, 40, 41, 42})
#: Copter-like airframes: quadrotor, coaxial, helicopter, hexa-, octo-, tri-, dodeca-, deca-rotor.
COPTER_TYPES = frozenset({2, 3, 4, 13, 14, 15, 29, 35})
#: GLOBAL_POSITION_INT.hdg uses this value for "heading unknown".
HDG_UNKNOWN = 65535

#: ArduCopter custom flight modes, as in pymavlink's mavutil.mode_mapping_acm (tests compare the two).
ARDUCOPTER_MODES = {
    0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED', 5: 'LOITER', 6: 'RTL', 7: 'CIRCLE',
    8: 'POSITION', 9: 'LAND', 10: 'OF_LOITER', 11: 'DRIFT', 13: 'SPORT', 14: 'FLIP', 15: 'AUTOTUNE',
    16: 'POSHOLD', 17: 'BRAKE', 18: 'THROW', 19: 'AVOID_ADSB', 20: 'GUIDED_NOGPS', 21: 'SMART_RTL',
    22: 'FLOWHOLD', 23: 'FOLLOW', 24: 'ZIGZAG', 25: 'SYSTEMID', 26: 'AUTOROTATE', 27: 'AUTO_RTL',
}


def copter_mode_name(msg) -> Optional[str]:
    """The flight mode name of an ArduCopter HEARTBEAT, or None for any other kind of vehicle.

    None means "not known", never a guess: a mode we cannot name is left out of the Lattice entity.
    An ArduCopter mode number that is not in the table becomes "MODE <n>" (it is real data).
    """
    if int(msg.autopilot) != MAV_AUTOPILOT_ARDUPILOTMEGA or int(msg.type) not in COPTER_TYPES:
        return None
    if not int(msg.base_mode) & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
        return None
    number = int(msg.custom_mode)
    return ARDUCOPTER_MODES.get(number, f'MODE {number}')


@dataclass
class MavlinkDecoderConfig:
    """Settings of the decoder (every one is also an option of the bridge profile, see config/bridge.yaml)."""
    vehicle_sysid: Optional[int] = None            # None = lock onto the first autopilot heard
    time_mode: str = 'auto'                        # 'auto' | 'sensor' | 'arrival' (see ClockJudge)
    max_clock_skew_s: float = 2.0
    heading_max_age_s: float = 5.0
    battery_max_age_s: float = 60.0
    mode_max_age_s: float = 10.0
    tgt_severities: Tuple[int, ...] = (C.STATUSTEXT_SEVERITY_INFO,)
    drop_channels: Optional[Dict[int, str]] = None  # None = {11: person, 13: tent}
    drop_open_pwm: int = 1400
    chunk_timeout_s: float = 5.0
    extra: Dict[str, Any] = field(default_factory=dict)


class MavlinkDecoder:
    """Turn MAVLink message objects into events (see the module docstring). Pure: no sockets, no clock."""

    def __init__(self, config: Optional[MavlinkDecoderConfig] = None,
                 mode_namer: Callable[[Any], Optional[str]] = copter_mode_name):
        self.config = config or MavlinkDecoderConfig()
        self._mode_namer = mode_namer
        self.clock = ClockJudge(self.config.time_mode, self.config.max_clock_skew_s)
        self.accumulator = DroneStateAccumulator(
            heading_max_age_s=self.config.heading_max_age_s, battery_max_age_s=self.config.battery_max_age_s,
            mode_max_age_s=self.config.mode_max_age_s)
        self.assembler = StatusTextAssembler(timeout_s=self.config.chunk_timeout_s)
        self.texts = TextEventDecoder(severities=self.config.tgt_severities,
                                      drop_channels=self.config.drop_channels,
                                      drop_open_pwm=self.config.drop_open_pwm)
        self.vehicle_sysid: Optional[int] = self.config.vehicle_sysid
        self._boot_offset_s: Optional[float] = None
        self._last_boot_ms: Optional[int] = None
        self._last_fix_time: Optional[datetime] = None
        self._handlers = {
            'GLOBAL_POSITION_INT': self._global_position, 'HEARTBEAT': self._heartbeat,
            'VFR_HUD': self._vfr_hud, 'ATTITUDE': self._attitude, 'SYS_STATUS': self._sys_status,
            'SYSTEM_TIME': self._system_time, 'STATUSTEXT': self._statustext,
        }
        self.stats: Dict[str, int] = {
            'messages': 0, 'ignored': 0, 'bad_messages': 0, 'fixes': 0, 'bad_fixes': 0, 'fixes_not_newer': 0,
            'heartbeats': 0, 'other_heartbeats': 0, 'other_vehicle_messages': 0, 'reboots': 0,
            'clock_fallbacks': 0, 'statustexts': 0,
        }

    # ------------------------------------------------------------------------------------------ public
    def reset(self) -> None:
        """Forget everything learned from the stream (after a reconnect: the vehicle may have rebooted)."""
        self.clock.reset()
        self.accumulator.reset()
        self._boot_offset_s = None
        self._last_boot_ms = None
        self._last_fix_time = None
        if self.config.vehicle_sysid is None:
            self.vehicle_sysid = None

    @property
    def last_fix(self) -> Optional[DroneFix]:
        """The most recent DroneFix produced (None before the first one)."""
        return self.accumulator.last_fix

    def feed(self, msg, arrival: datetime) -> List[Event]:
        """Decode one message received at `arrival` (aware UTC). Returns events; never raises."""
        self.stats['messages'] += 1
        try:
            handler = self._handlers.get(msg.get_type())
            if handler is None:
                self.stats['ignored'] += 1
                return []
            events = handler(msg, arrival, int(msg.get_srcSystem()), int(msg.get_srcComponent()))
            self.stats['clock_fallbacks'] = self.clock.fallbacks
            return events
        except (ValueError, TypeError, AttributeError, KeyError, OverflowError, ArithmeticError):
            self.stats['bad_messages'] += 1
            return []

    def merged_stats(self) -> Dict[str, int]:
        """Counters of the decoder, the text assembler and the text decoder in one dict (prefixed)."""
        merged = dict(self.stats)
        merged.update({f'text_{k}': v for k, v in self.assembler.stats.items()})
        merged.update({k: v for k, v in self.texts.stats.items()})
        return merged

    # ------------------------------------------------------------------------------------------ handlers
    def _vehicle_ok(self, sysid: int) -> bool:
        """True when a message from `sysid` is about our vehicle (locks onto the first one heard)."""
        if self.vehicle_sysid is None:
            self.vehicle_sysid = sysid
            return True
        if sysid != self.vehicle_sysid:
            self.stats['other_vehicle_messages'] += 1
            return False
        return True

    def _heartbeat(self, msg, arrival: datetime, sysid: int, compid: int) -> List[Event]:
        if int(msg.autopilot) == MAV_AUTOPILOT_INVALID or int(msg.type) in NON_VEHICLE_TYPES:
            self.stats['other_heartbeats'] += 1
            return []
        first = self.vehicle_sysid is None
        if not self._vehicle_ok(sysid):
            return []
        self.stats['heartbeats'] += 1
        armed = bool(int(msg.base_mode) & MAV_MODE_FLAG_SAFETY_ARMED)
        self.accumulator.set_mode(self._mode_namer(msg), armed, arrival)
        if first:
            return [SourceNotice('info', f'vehicle found: system {sysid}, type {int(msg.type)}', arrival)]
        return []

    def _system_time(self, msg, arrival: datetime, sysid: int, compid: int) -> List[Event]:
        if self.vehicle_sysid is not None and sysid != self.vehicle_sysid:
            return []
        unix_usec, boot_ms = int(msg.time_unix_usec), int(msg.time_boot_ms)
        if unix_usec <= 0:                                  # 0 = "the autopilot has no real time yet"
            return []
        unix_s = unix_usec / 1e6
        from_unix_seconds(unix_s)                           # ValueError unless it is a believable date (2000-2100)
        self._boot_offset_s = unix_s - boot_ms / 1000.0
        return []

    def _global_position(self, msg, arrival: datetime, sysid: int, compid: int) -> List[Event]:
        if not self._vehicle_ok(sysid):
            return []
        boot_ms = int(msg.time_boot_ms)
        if self._last_boot_ms is not None and boot_ms + 5000 < self._last_boot_ms:
            self.stats['reboots'] += 1                      # time since boot went backwards: the autopilot restarted
            self._boot_offset_s = None
            self.clock.reset()
        self._last_boot_ms = boot_ms
        try:
            lat, lon = require_lat_lon(int(msg.lat) / 1e7, int(msg.lon) / 1e7, 'drone position')
        except ValueError:
            self.stats['bad_fixes'] += 1                    # includes (0, 0) = "no position yet"
            return []
        sensor_utc = None
        if self._boot_offset_s is not None:
            try:
                sensor_utc = from_unix_seconds(self._boot_offset_s + boot_ms / 1000.0)
            except ValueError:
                sensor_utc = None
        fix_time, used_sensor = self.clock.choose(sensor_utc, arrival)
        if self._last_fix_time is not None and fix_time <= self._last_fix_time:
            self.stats['fixes_not_newer'] += 1              # a duplicate or out-of-order position: skip it
            return []
        heading = None
        hdg = int(msg.hdg)
        if 0 <= hdg < 36000:                                # 65535 = unknown
            heading = hdg / 100.0
        fix = self.accumulator.build_fix(
            fix_time=fix_time, arrival=arrival, lat_deg=lat, lon_deg=lon,
            alt_amsl_m=_metres(msg.alt), rel_alt_m=_metres(msg.relative_alt), heading_deg=heading,
            velocity_ned_mps=_velocity(msg), time_source='autopilot' if used_sensor else 'arrival')
        self._last_fix_time = fix_time
        self.stats['fixes'] += 1
        return [fix]

    def _vfr_hud(self, msg, arrival: datetime, sysid: int, compid: int) -> List[Event]:
        if self.vehicle_sysid is not None and sysid != self.vehicle_sysid:
            return []
        heading = float(msg.heading)
        if is_real_number(heading) and 0.0 <= heading <= 360.0:
            self.accumulator.set_heading(heading % 360.0, arrival)
        return []

    def _attitude(self, msg, arrival: datetime, sysid: int, compid: int) -> List[Event]:
        if self.vehicle_sysid is not None and sysid != self.vehicle_sysid:
            return []
        yaw = float(msg.yaw)
        if is_real_number(yaw):
            self.accumulator.set_heading(math.degrees(yaw) % 360.0, arrival)
        return []

    def _sys_status(self, msg, arrival: datetime, sysid: int, compid: int) -> List[Event]:
        if self.vehicle_sysid is not None and sysid != self.vehicle_sysid:
            return []
        remaining = int(msg.battery_remaining)
        if 0 <= remaining <= 100:                            # -1 = the autopilot does not know
            self.accumulator.set_battery(float(remaining), arrival)
        return []

    def _statustext(self, msg, arrival: datetime, sysid: int, compid: int) -> List[Event]:
        self.stats['statustexts'] += 1
        events: List[Event] = []
        lines = self.assembler.feed(sysid, compid, msg.severity, msg.text, getattr(msg, 'id', 0),
                                    getattr(msg, 'chunk_seq', 0), now_s=arrival.timestamp())
        for line in lines:
            events.extend(self.texts.decode(line.text, line.severity, arrival, self.accumulator.last_fix))
        return events


def _type_of(msg) -> str:
    """The MAVLink message name of `msg`, or '' for anything that is not a message object."""
    try:
        return str(msg.get_type())
    except Exception:                                       # not a message at all (a hostile or broken object)
        return ''


def _metres(millimetres) -> Optional[float]:
    """Millimetres (an int from MAVLink) -> metres, or None when the value is outside what a height can be."""
    value = int(millimetres) / 1000.0
    return value if C.ALT_MIN_M <= value <= C.ALT_MAX_M else None


def _velocity(msg) -> Optional[Tuple[float, float, float]]:
    """(north, east, down) in m/s from the GLOBAL_POSITION_INT cm/s fields, or None if they are not sane."""
    try:
        v = (int(msg.vx) / 100.0, int(msg.vy) / 100.0, int(msg.vz) / 100.0)
    except (TypeError, ValueError):
        return None
    return v if all(abs(x) <= C.SPEED_MAX_MPS for x in v) else None


# ===================================================================================================
# The connection
# ===================================================================================================

def import_pymavlink():
    """Import pymavlink.mavutil (only the MAVLink source does this). Raises SourceUnavailable if missing."""
    os.environ.setdefault('MAVLINK20', '1')                 # also decode MAVLink 2 (STATUSTEXT chunk fields)
    try:
        from pymavlink import mavutil
    except ImportError:
        raise SourceUnavailable(
            'the MAVLink source needs the Python package pymavlink, which is not installed. Install it with '
            '"python3 -m pip install pymavlink" (it is only needed for --source mavlink), or use --source '
            'replay / --source ros.') from None
    return mavutil


def _default_connect(connection: str):
    """Open a pymavlink connection ('udpin:127.0.0.1:14550', 'tcp:host:5762', '/dev/ttyUSB0,57600' ...)."""
    mavutil = import_pymavlink()
    return mavutil.mavlink_connection(connection, robust_parsing=True)


def explain_connect_error(exc: BaseException, connection: str) -> str:
    """A plain-words hint for the usual reasons a connection cannot be opened."""
    if isinstance(exc, OSError) and exc.errno == errno.EADDRINUSE:
        return (f'{connection}: that UDP port is already in use by another program (the simulator scripts, '
                f'Mission Planner or another bridge?). Only one program can listen on a port: give the bridge '
                f'its own MAVProxy output (see docs/lattice/BRIDGE.md) or stop the other program.')
    if isinstance(exc, OSError) and exc.errno in (errno.EACCES, errno.EPERM):
        return f'{connection}: permission denied ({exc}); use a port above 1024 or check the device permissions.'
    if isinstance(exc, (ConnectionRefusedError, FileNotFoundError)):
        return f'{connection}: nothing is listening there yet ({exc.__class__.__name__}); retrying.'
    return f'{connection}: {exc.__class__.__name__}: {exc}'


class MavlinkSource:
    """A Source that reads MAVLink from `connection` (see the module docstring).

    The parts that touch the outside world are injectable so the whole source is tested without a socket:
    `connect(connection)` returns an object with recv_match(blocking=, timeout=) and close(); `clock()` returns
    the current aware UTC time; `monotonic()` and `sleep(s)` drive the retry timing.
    """

    name = 'mavlink'
    altitude_reference = 'amsl'

    def __init__(self, connection: str = 'udpin:127.0.0.1:14550', *,
                 decoder_config: Optional[MavlinkDecoderConfig] = None,
                 connect: Optional[Callable[[str], Any]] = None,
                 clock: Optional[Callable[[], datetime]] = None,
                 monotonic: Optional[Callable[[], float]] = None,
                 sleep: Optional[Callable[[float], None]] = None,
                 rng: Optional[random.Random] = None,
                 silent_notice_s: float = 10.0, reconnect_after_silence_s: Optional[float] = None,
                 backoff_base_s: float = 1.0, backoff_cap_s: float = 30.0,
                 max_events_per_poll: int = 500, recv_slice_s: float = 0.25):
        if not isinstance(connection, str) or not connection.strip() or any(c.isspace() for c in connection):
            raise ValueError(f'connection must be a MAVLink connection string without spaces, got {str(connection)[:40]!r}')
        self.connection = connection.strip()
        self.decoder = MavlinkDecoder(decoder_config)
        self._connect_fn = connect
        self._clock = clock or (lambda: datetime.now(timezone.utc))
        self._monotonic = monotonic or time.monotonic
        self._sleep = sleep or time.sleep
        self._rng = rng if rng is not None else random.Random()
        self.silent_notice_s = float(silent_notice_s)
        if reconnect_after_silence_s is None:               # UDP has no connection to lose; others are rebuilt
            reconnect_after_silence_s = 0.0 if self.connection.startswith('udp') else 30.0
        self.reconnect_after_silence_s = float(reconnect_after_silence_s)
        self.backoff_base_s = float(backoff_base_s)
        self.backoff_cap_s = float(backoff_cap_s)
        self.max_events_per_poll = int(max_events_per_poll)
        self.recv_slice_s = float(recv_slice_s)
        self._conn: Any = None
        self._attempts = 0
        self._next_attempt = 0.0
        self._last_rx = 0.0
        self._silent_noticed = False
        self._last_error_log = -1e9
        self._pending_notices: List[SourceNotice] = []
        self._closed = False
        self.counters = {'connects': 0, 'connect_failures': 0, 'link_losses': 0, 'received': 0, 'bad_data': 0}

    # ------------------------------------------------------------------------------------------ Source API
    def start(self) -> None:
        """Check that pymavlink is available (when the default connector is used) and try the first connection."""
        if self._connect_fn is None:
            import_pymavlink()                              # fail fast and clearly, before the main loop starts
        self._ensure_connected()

    def now(self) -> datetime:
        """The current time (aware UTC)."""
        return self._clock()

    def describe(self) -> str:
        """One line for the log: the connection string (MAVLink strings carry no secrets)."""
        return f'MAVLink {self.connection}'

    def stats(self) -> Dict[str, int]:
        """All counters (connection, decoder, text decoder)."""
        merged = dict(self.counters)
        merged.update(self.decoder.merged_stats())
        return merged

    def close(self) -> None:
        """Close the connection. Safe to call twice."""
        self._closed = True
        self._drop_connection()

    def poll(self, timeout_s: float) -> List[Event]:
        """Read for up to `timeout_s` seconds; return the events decoded (see the module docstring)."""
        events: List[Event] = []
        if self._closed:
            return events
        started = self._monotonic()
        while True:
            remaining = timeout_s - (self._monotonic() - started)
            if self._conn is None and not self._ensure_connected():
                events.extend(self._take_notices())
                if remaining <= 0:
                    break
                self._sleep(min(0.1, remaining))
                continue
            events.extend(self._take_notices())
            wait = max(0.0, min(self.recv_slice_s, remaining))
            read_started = self._monotonic()
            try:
                msg = self._conn.recv_match(blocking=True, timeout=wait)
            except Exception as exc:                        # a broken socket / serial port: reconnect, never crash
                self._link_lost(f'read failed: {exc.__class__.__name__}: {exc}')
                msg = None
            if msg is not None:
                self._last_rx = self._monotonic()
                self._silent_noticed = False
                self.counters['received'] += 1
                if _type_of(msg) == 'BAD_DATA':
                    self.counters['bad_data'] += 1
                else:
                    events.extend(self.decoder.feed(msg, self._clock()))
            else:
                events.extend(self._check_silence())
                spent = self._monotonic() - read_started
                if self._conn is not None and spent < wait * 0.5:
                    self._sleep(wait - spent)               # a dead connection that returns at once must not spin the CPU
            if len(events) >= self.max_events_per_poll or self._monotonic() - started >= timeout_s:
                break
        events.extend(self._take_notices())
        return events

    # ------------------------------------------------------------------------------------------ internals
    def _notice(self, code: str, message: str) -> None:
        self._pending_notices.append(SourceNotice(code, message, self._clock()))

    def _take_notices(self) -> List[Event]:
        notices, self._pending_notices = self._pending_notices, []
        return list(notices)

    def _ensure_connected(self) -> bool:
        """Open the connection if it is closed and the retry wait is over. True when connected."""
        if self._conn is not None:
            return True
        now_m = self._monotonic()
        if now_m < self._next_attempt or self._closed:
            return False
        self._attempts += 1
        try:
            self._conn = (self._connect_fn or _default_connect)(self.connection)
        except SourceUnavailable:
            raise
        except Exception as exc:
            self._conn = None
            self.counters['connect_failures'] += 1
            delay = backoff_delay(self._attempts, base_s=self.backoff_base_s, cap_s=self.backoff_cap_s,
                                  rng=self._rng)
            self._next_attempt = now_m + delay
            if now_m - self._last_error_log >= 30.0 or self._attempts == 1:
                LOG.warning('cannot open %s (attempt %d, next try in %.0f s): %s', self.connection,
                            self._attempts, delay, explain_connect_error(exc, self.connection))
                self._last_error_log = now_m
            if self._attempts == 1:
                self._notice('reconnecting', explain_connect_error(exc, self.connection))
            return False
        self._attempts = 0
        self._next_attempt = 0.0
        self._last_rx = now_m
        self._silent_noticed = False
        self.decoder.reset()
        self.counters['connects'] += 1
        self._notice('connected', f'listening on {self.connection}')
        return True

    def _drop_connection(self) -> None:
        conn, self._conn = self._conn, None
        if conn is not None:
            try:
                conn.close()
            except Exception:                               # closing a dead socket may fail; nothing to do
                pass

    def _link_lost(self, why: str) -> None:
        self.counters['link_losses'] += 1
        LOG.warning('MAVLink link lost on %s: %s', self.connection, why)
        self._notice('link_lost', why)
        self._drop_connection()
        self._next_attempt = self._monotonic() + backoff_delay(
            1, base_s=self.backoff_base_s, cap_s=self.backoff_cap_s, rng=self._rng)

    def _check_silence(self) -> List[Event]:
        """Called when a read timed out: tell the bridge once that nothing has arrived, reconnect if configured."""
        silent_for = self._monotonic() - self._last_rx
        if self.silent_notice_s > 0 and silent_for >= self.silent_notice_s and not self._silent_noticed:
            self._silent_noticed = True
            self._notice('silent', f'no MAVLink data for {silent_for:.0f} s on {self.connection}')
        if self.reconnect_after_silence_s > 0 and silent_for >= self.reconnect_after_silence_s:
            self._link_lost(f'no MAVLink data for {silent_for:.0f} s: reconnecting')
            self._next_attempt = 0.0
        return []
