"""
lattice_bridge/sources/common.py -- PURPOSE: the building blocks that the MAVLink source, the ROS source and
the replay source all share, so the three of them cannot drift apart:

  * ClockJudge             decide which clock a position's time should come from (the sensor's own clock or the
                           moment the message arrived here), and refuse a sensor clock that is clearly wrong
  * backoff_delay          the wait before the next reconnect attempt (grows 1, 2, 4 ... 30 s, with jitter)
  * StatusTextAssembler    put MAVLink STATUSTEXT chunks back together; ignore duplicate chunks
  * TextEventDecoder       turn one finished STATUSTEXT line into events: a `TGT` target report, or the
                           "Servo 13 -> 1400" line that means "the payload was released"
  * DroneStateAccumulator  remember the latest heading, battery, flight mode and armed flag, and fold them
                           into the next position as one DroneFix
  * EventQueue             a bounded, thread-safe queue between a producer thread and the bridge's main loop

Why a shared module: the rules about WHAT counts as a valid target report, a release or a fresh heading are
facts about the mission, not about the transport. They live here once and every source uses them.

Everything is plain Python with no I/O and no clock reads (times and "now" are passed in), so it is all
unit-tested with fake inputs. Standard library only.
"""

from __future__ import annotations

import re
import threading
import time
from collections import OrderedDict, deque
from dataclasses import dataclass
from datetime import datetime
from typing import Deque, Dict, Iterable, List, Optional, Tuple

from .. import constants as C
from ..events import DropEvent, DroneFix, Event, SourceNotice, TargetSighting
from ..geo import is_real_number
from ..report_codec import decode_tgt
from ..timeutil import require_aware

# ===================================================================================================
# Which clock to believe
# ===================================================================================================

#: --time modes: 'auto' = use the sensor's clock only while it agrees with ours; 'sensor' = use it whenever it
#: is sane; 'arrival' = always use the moment the message arrived.
CLOCK_MODES = ('auto', 'sensor', 'arrival')
#: In 'sensor' mode a clock further than this from ours is still refused: the entity builders reject times
#: more than 60 s in the future, and a position older than this is not "now" any more.
_SENSOR_MODE_LIMIT_S = 55.0


class ClockJudge:
    """Choose the time to stamp a position with: the sensor's own time, or the arrival time.

    The autopilot reports time since ITS boot (`time_boot_ms`), which becomes real UTC only through a
    reference pair from SYSTEM_TIME. That pair is only as good as the autopilot's clock. Measured on the
    simulator (a recorded SITL run, 2026-10-24): the autopilot clock fell 10 s, then 31 s, then 34 s behind the
    computer's clock, because the simulator runs slower than real time. A bridge that trusted it would stamp
    positions half a minute in the past. On the real drone the clock comes from GPS and is exact.

    So in 'auto' mode the sensor time is used only while it agrees with the arrival time within `max_skew_s`
    (default 2 s: radio delay is well under that). The first time it disagrees the judge stops trusting it
    for good ("sticky") until reset(): flipping back and forth would make timestamps jump around. A
    measurement can never be newer than the moment it arrived, so a sensor time slightly ahead of the arrival
    time is cut back to the arrival time.
    """

    def __init__(self, mode: str = 'auto', max_skew_s: float = 2.0):
        if mode not in CLOCK_MODES:
            raise ValueError(f'time mode must be one of {CLOCK_MODES}, got {str(mode)[:20]!r}')
        if not is_real_number(max_skew_s) or not 0.0 < max_skew_s <= _SENSOR_MODE_LIMIT_S:
            raise ValueError(f'max_skew_s must be greater than 0 and at most {_SENSOR_MODE_LIMIT_S:.0f}')
        self.mode = mode
        self.max_skew_s = float(max_skew_s)
        self.distrusted = False
        self.last_skew_s: Optional[float] = None
        self.fallbacks = 0

    def reset(self) -> None:
        """Forget what was learned about the sensor clock (call after a reconnect or a reboot)."""
        self.distrusted = False
        self.last_skew_s = None

    def choose(self, sensor_utc: Optional[datetime], arrival: datetime) -> Tuple[datetime, bool]:
        """Return (time to use, True when it is the sensor's time). `arrival` is aware UTC."""
        arrival = require_aware('arrival', arrival)
        if self.mode == 'arrival' or sensor_utc is None:
            return arrival, False
        sensor = require_aware('sensor time', sensor_utc)
        skew = (sensor - arrival).total_seconds()
        self.last_skew_s = skew
        if self.mode == 'auto' and self.distrusted:
            self.fallbacks += 1
            return arrival, False
        limit = self.max_skew_s if self.mode == 'auto' else _SENSOR_MODE_LIMIT_S
        if abs(skew) > limit:
            if self.mode == 'auto':
                self.distrusted = True
            self.fallbacks += 1
            return arrival, False
        return (sensor if sensor <= arrival else arrival), True


# ===================================================================================================
# Reconnect timing
# ===================================================================================================

def backoff_delay(attempt: int, *, base_s: float = 1.0, cap_s: float = 30.0, jitter: float = 0.2,
                  rng=None) -> float:
    """Seconds to wait before reconnect attempt number `attempt` (1 = the first retry).

    Doubles each time (base, 2 x base, 4 x base ...) up to `cap_s`, then adds a random spread of +-`jitter`
    (a fraction) so that several bridges restarted together do not all knock on the door at the same moment.
    `rng` is anything with .uniform(a, b); None means no jitter (used by tests).
    """
    steps = min(max(int(attempt) - 1, 0), 30)
    delay = min(float(cap_s), float(base_s) * (2 ** steps))
    if rng is not None and jitter > 0:
        delay += rng.uniform(-delay * jitter, delay * jitter)
    return max(0.0, delay)


# ===================================================================================================
# STATUSTEXT: chunks -> whole lines
# ===================================================================================================

_CHUNK_CHARS = 50          # MAVLink STATUSTEXT.text is char[50]
_MAX_CHUNKS = 32           # a hostile sender cannot make us collect more than 32 x 50 characters per message


@dataclass(frozen=True)
class StatusText:
    """One finished STATUSTEXT line and who sent it."""
    sysid: int
    compid: int
    severity: int
    text: str


class _Partial:
    """The chunks of one long message collected so far."""
    __slots__ = ('first_seen', 'severity', 'chunks', 'last_seq')

    def __init__(self, first_seen: float, severity: int):
        self.first_seen = first_seen
        self.severity = severity
        self.chunks: Dict[int, str] = {}
        self.last_seq: Optional[int] = None


class StatusTextAssembler:
    """Put MAVLink STATUSTEXT chunks back together.

    MAVLink 2 can split a long text over several STATUSTEXT messages that share a non-zero `id` and count
    `chunk_seq` 0, 1, 2 ...; "any null character in the text field is taken to mean this was the last chunk",
    and a chunk shorter than 50 characters is the last one too. `id` 0 (and every MAVLink 1 message) means
    "the whole text is in this one message". Rules, each covered by a test:

      * a whole message is returned at once;
      * chunks are collected per (sender, id), in any order; when the last chunk is there and no earlier one
        is missing the text is joined and returned once;
      * a duplicated chunk (same id, same sequence number, same text) is ignored; a chunk that contradicts
        an earlier one throws that message away (it is corrupt) and is counted;
      * a message that was already finished is not returned a second time when its chunks arrive again;
      * incomplete messages are dropped after `timeout_s`, and at most `max_pending` are held: memory is bounded
        no matter what the radio delivers.
    """

    def __init__(self, *, max_pending: int = 16, timeout_s: float = 5.0, remember_done: int = 64):
        if max_pending < 1 or remember_done < 1 or not is_real_number(timeout_s) or timeout_s <= 0:
            raise ValueError('max_pending and remember_done must be >= 1 and timeout_s > 0')
        self.max_pending = max_pending
        self.timeout_s = float(timeout_s)
        self.remember_done = remember_done
        self._pending: 'OrderedDict[Tuple[int, int, int], _Partial]' = OrderedDict()
        self._done: 'OrderedDict[Tuple[int, int, int], float]' = OrderedDict()
        self.stats = {'whole': 0, 'assembled': 0, 'duplicate_chunks': 0, 'duplicate_messages': 0,
                      'conflicts': 0, 'incomplete_dropped': 0, 'rejected': 0}

    def feed(self, sysid: int, compid: int, severity: int, text, msg_id: Optional[int] = 0,
             chunk_seq: Optional[int] = 0, now_s: float = 0.0) -> List[StatusText]:
        """Take one STATUSTEXT message; return the finished lines (an empty list or one line).

        `now_s` is any steadily increasing clock in seconds (only differences are used). Never raises.
        """
        try:
            return self._feed(sysid, compid, severity, text, msg_id, chunk_seq, float(now_s))
        except (TypeError, ValueError, OverflowError):
            self.stats['rejected'] += 1
            return []

    def pending_count(self) -> int:
        """How many incomplete messages are being held."""
        return len(self._pending)

    # ------------------------------------------------------------------------------------------ internals
    def _feed(self, sysid, compid, severity, text, msg_id, chunk_seq, now_s: float) -> List[StatusText]:
        self._expire(now_s)
        if isinstance(text, (bytes, bytearray)):
            text = bytes(text).decode('utf-8', errors='replace')
        if not isinstance(text, str):
            self.stats['rejected'] += 1
            return []
        has_nul = '\x00' in text
        cut = text.split('\x00', 1)[0][:_CHUNK_CHARS]
        final = has_nul or len(cut) < _CHUNK_CHARS
        msg_id = 0 if msg_id is None else int(msg_id)
        chunk_seq = 0 if chunk_seq is None else int(chunk_seq)
        sysid, compid, severity = int(sysid), int(compid), int(severity)
        if not 0 <= msg_id <= 0xFFFF or not 0 <= chunk_seq < _MAX_CHUNKS:
            self.stats['rejected'] += 1
            return []
        if msg_id == 0:
            if not cut:
                return []
            self.stats['whole'] += 1
            return [StatusText(sysid, compid, severity, cut)]

        key = (sysid, compid, msg_id)
        if key in self._done:
            self.stats['duplicate_messages'] += 1
            return []
        partial = self._pending.get(key)
        if partial is None:
            if len(self._pending) >= self.max_pending:
                self._pending.popitem(last=False)
                self.stats['incomplete_dropped'] += 1
            partial = _Partial(now_s, severity)
            self._pending[key] = partial
        if chunk_seq in partial.chunks:
            if partial.chunks[chunk_seq] == cut:
                self.stats['duplicate_chunks'] += 1
            else:
                del self._pending[key]
                self.stats['conflicts'] += 1
            return []
        if partial.last_seq is not None and chunk_seq > partial.last_seq:
            del self._pending[key]
            self.stats['conflicts'] += 1
            return []
        partial.chunks[chunk_seq] = cut
        if final:
            if partial.last_seq is not None and partial.last_seq != chunk_seq:
                del self._pending[key]
                self.stats['conflicts'] += 1
                return []
            partial.last_seq = chunk_seq
            if any(seq > chunk_seq for seq in partial.chunks):
                del self._pending[key]
                self.stats['conflicts'] += 1
                return []
        if partial.last_seq is None or any(i not in partial.chunks for i in range(partial.last_seq + 1)):
            return []
        del self._pending[key]
        self._done[key] = now_s
        while len(self._done) > self.remember_done:
            self._done.popitem(last=False)
        joined = ''.join(partial.chunks[i] for i in range(partial.last_seq + 1))
        if not joined:
            return []
        self.stats['assembled'] += 1
        return [StatusText(sysid, compid, partial.severity, joined)]

    def _expire(self, now_s: float) -> None:
        """Drop incomplete messages that waited longer than the timeout, and old 'finished' markers."""
        for key in [k for k, p in self._pending.items() if now_s - p.first_seen > self.timeout_s]:
            del self._pending[key]
            self.stats['incomplete_dropped'] += 1
        for key in [k for k, t in self._done.items() if now_s - t > 60.0 * self.timeout_s]:
            del self._done[key]


# ===================================================================================================
# STATUSTEXT line -> events
# ===================================================================================================

#: The line main_controller sends after the autopilot accepted a servo command: "Servo 13 -> 1400".
_SERVO_LINE = re.compile(r'Servo (?P<channel>[0-9]{1,2}) -> (?P<pwm>[0-9]{1,4})')

#: main_controller.py: PERSON_SERVO_CHANNELS = [11], TENT_SERVO_CHANNELS = [13], PULLEY_OPEN = 1400.
DEFAULT_DROP_CHANNELS = {11: C.KIND_PERSON, 13: C.KIND_TENT}
DEFAULT_DROP_OPEN_PWM = 1400


class TextEventDecoder:
    """Read finished STATUSTEXT lines and produce TargetSighting / DropEvent events.

    Two kinds of line matter, everything else is ignored:

      * `TGT <p|t> <lat> <lon> <conf> <seq>`: a target report from the Jetson (report_codec.decode_tgt, the strict
        parser). Only accepted at one of `severities` (default INFO, what the Jetson sends). The text comes
        over a radio, so nothing about it is trusted: a look-alike with a wrong format, a wrong severity or
        the same text twice within `dedupe_window_s` is counted and ignored.
      * `Servo <channel> -> <pwm>` with the open value (default 1400) on a known channel (11 = person, 13 =
        tent): the payload was released. The release position is the drone's last known position if that
        is newer than `max_fix_age_s`, otherwise unknown (None): a position is never guessed.

    The report counter (`seq`) is tracked: a jump is logged by the caller from `stats` (`tgt_gaps`), a counter
    that goes backwards means the Jetson program restarted.
    """

    def __init__(self, *, severities: Iterable[int] = (C.STATUSTEXT_SEVERITY_INFO,),
                 drop_channels: Optional[Dict[int, str]] = None, drop_open_pwm: int = DEFAULT_DROP_OPEN_PWM,
                 dedupe_window_s: float = 300.0, drop_dedupe_s: float = 20.0, max_fix_age_s: float = 10.0):
        self.severities = frozenset(int(s) for s in severities)
        self.drop_channels = dict(DEFAULT_DROP_CHANNELS if drop_channels is None else drop_channels)
        for channel, kind in self.drop_channels.items():
            if not isinstance(channel, int) or kind not in C.TARGET_KINDS:
                raise ValueError('drop_channels must map integer channels to "person" or "tent"')
        self.drop_open_pwm = int(drop_open_pwm)
        self.dedupe_window_s = float(dedupe_window_s)
        self.drop_dedupe_s = float(drop_dedupe_s)
        self.max_fix_age_s = float(max_fix_age_s)
        self._recent: 'OrderedDict[str, datetime]' = OrderedDict()
        self._last_drop: Dict[str, datetime] = {}
        self._last_seq: Optional[int] = None
        self.stats = {'tgt_reports': 0, 'tgt_duplicates': 0, 'tgt_wrong_severity': 0, 'tgt_malformed': 0,
                      'tgt_gaps': 0, 'tgt_lost_estimate': 0, 'tgt_restarts': 0, 'drops': 0, 'drop_duplicates': 0,
                      'drops_without_position': 0, 'errors': 0}

    def decode(self, text, severity, arrival: datetime, last_fix: Optional[DroneFix] = None) -> List[Event]:
        """Events for one finished line (an empty list for anything uninteresting). Never raises."""
        try:
            return self._decode(text, severity, require_aware('arrival', arrival), last_fix)
        except (TypeError, ValueError, OverflowError):
            self.stats['errors'] += 1
            return []

    # ------------------------------------------------------------------------------------------ internals
    def _decode(self, text, severity, arrival: datetime, last_fix: Optional[DroneFix]) -> List[Event]:
        if not isinstance(text, str):
            return []
        text = text.rstrip('\x00')
        report = decode_tgt(text)
        if report is not None:
            return self._tgt(text, report, severity, arrival)
        if text.startswith(C.TGT_PREFIX + ' ') or text == C.TGT_PREFIX:
            self.stats['tgt_malformed'] += 1                 # looked like ours, was not valid: counted, ignored
            return []
        match = _SERVO_LINE.fullmatch(text)
        if match is not None:
            return self._servo(int(match.group('channel')), int(match.group('pwm')), severity, arrival, last_fix)
        return []

    def _tgt(self, text: str, report, severity, arrival: datetime) -> List[Event]:
        if severity not in self.severities:
            self.stats['tgt_wrong_severity'] += 1
            return []
        previous = self._recent.get(text)
        if previous is not None and (arrival - previous).total_seconds() <= self.dedupe_window_s:
            self.stats['tgt_duplicates'] += 1
            return []
        self._recent[text] = arrival
        self._recent.move_to_end(text)
        while len(self._recent) > 64:
            self._recent.popitem(last=False)
        self._note_seq(report.seq)
        self.stats['tgt_reports'] += 1
        return [TargetSighting(report.kind, report.lat_deg, report.lon_deg, report.confidence, arrival,
                               seq=report.seq, origin='tgt')]

    def _note_seq(self, seq: int) -> None:
        """Track the message counter: count gaps (lost reports) and restarts (counter went backwards)."""
        last = self._last_seq
        self._last_seq = seq
        if last is None:
            return
        expected = (last + 1) % (C.TGT_SEQ_MAX + 1)
        if seq == expected:
            return
        forward = (seq - expected) % (C.TGT_SEQ_MAX + 1)
        if forward < 1000:                                  # skipped a few: reports were lost on the way
            self.stats['tgt_gaps'] += 1
            self.stats['tgt_lost_estimate'] += forward
        else:                                               # jumped back (or far): the sender restarted
            self.stats['tgt_restarts'] += 1

    def _servo(self, channel: int, pwm: int, severity, arrival: datetime,
               last_fix: Optional[DroneFix]) -> List[Event]:
        kind = self.drop_channels.get(channel)
        if kind is None or pwm != self.drop_open_pwm or severity not in self.severities:
            return []
        previous = self._last_drop.get(kind)
        if previous is not None and (arrival - previous).total_seconds() <= self.drop_dedupe_s:
            self.stats['drop_duplicates'] += 1
            return []
        self._last_drop[kind] = arrival
        self.stats['drops'] += 1
        if last_fix is not None and 0.0 <= (arrival - last_fix.arrival).total_seconds() <= self.max_fix_age_s:
            return [DropEvent(kind, arrival, last_fix.lat_deg, last_fix.lon_deg)]
        self.stats['drops_without_position'] += 1
        return [DropEvent(kind, arrival)]


# ===================================================================================================
# Latest side values -> one DroneFix
# ===================================================================================================

class DroneStateAccumulator:
    """Keep the newest heading, battery, flight mode and armed flag and fold them into a DroneFix.

    Telemetry arrives as separate messages at separate times (position, heading, battery, heartbeat). The
    bridge wants one picture per position, so each side value is remembered with its arrival time and is used
    only while it is younger than its limit: a heading older than `heading_max_age_s` is left out rather than
    reported as if it were current ("None" means "not known", never "same as before").
    """

    def __init__(self, *, heading_max_age_s: float = 5.0, battery_max_age_s: float = 60.0,
                 mode_max_age_s: float = 10.0):
        self.heading_max_age_s = float(heading_max_age_s)
        self.battery_max_age_s = float(battery_max_age_s)
        self.mode_max_age_s = float(mode_max_age_s)
        self._heading: Optional[Tuple[float, datetime]] = None
        self._battery: Optional[Tuple[float, datetime]] = None
        self._mode: Optional[Tuple[Optional[str], Optional[bool], datetime]] = None
        self.last_fix: Optional[DroneFix] = None

    def reset(self) -> None:
        """Forget everything (a reconnect to a possibly different vehicle)."""
        self._heading = self._battery = self._mode = None
        self.last_fix = None

    def set_heading(self, degrees: float, arrival: datetime) -> None:
        """Remember a compass heading 0..360 (north = 0, clockwise). Out-of-range values are ignored."""
        if is_real_number(degrees) and 0.0 <= degrees <= 360.0:
            self._heading = (float(degrees), arrival)

    def set_battery(self, percent: float, arrival: datetime) -> None:
        """Remember a battery charge 0..100 %. Out-of-range values (such as -1 = unknown) are ignored."""
        if is_real_number(percent) and 0.0 <= percent <= 100.0:
            self._battery = (float(percent), arrival)

    def set_mode(self, mode: Optional[str], armed: Optional[bool], arrival: datetime) -> None:
        """Remember the flight mode and armed flag from a heartbeat (either may be None = unknown)."""
        self._mode = (mode, armed, arrival)

    def mode_and_armed(self, now: datetime) -> Tuple[Optional[str], Optional[bool]]:
        """The remembered mode and armed flag, or (None, None) when the last heartbeat is too old."""
        if self._mode is None or (now - self._mode[2]).total_seconds() > self.mode_max_age_s:
            return None, None
        return self._mode[0], self._mode[1]

    def build_fix(self, *, fix_time: datetime, arrival: datetime, lat_deg: float, lon_deg: float,
                  alt_amsl_m: Optional[float] = None, alt_hae_m: Optional[float] = None,
                  rel_alt_m: Optional[float] = None, heading_deg: Optional[float] = None,
                  velocity_ned_mps: Optional[Tuple[float, float, float]] = None,
                  time_source: str = 'arrival') -> DroneFix:
        """A DroneFix from a position plus the freshest side values. Raises ValueError for a bad position."""
        if heading_deg is None and self._heading is not None \
                and 0.0 <= (arrival - self._heading[1]).total_seconds() <= self.heading_max_age_s:
            heading_deg = self._heading[0]
        battery = None
        if self._battery is not None and 0.0 <= (arrival - self._battery[1]).total_seconds() <= self.battery_max_age_s:
            battery = self._battery[0]
        mode, armed = self.mode_and_armed(arrival)
        fix = DroneFix(fix_time=fix_time, arrival=arrival, lat_deg=lat_deg, lon_deg=lon_deg, alt_amsl_m=alt_amsl_m,
                       alt_hae_m=alt_hae_m, rel_alt_m=rel_alt_m, heading_deg=heading_deg,
                       velocity_ned_mps=velocity_ned_mps, mode=mode, armed=armed, battery_percent=battery,
                       time_source=time_source)
        self.last_fix = fix
        return fix


# ===================================================================================================
# A bounded queue between a reader thread and the main loop
# ===================================================================================================

class EventQueue:
    """Thread-safe, bounded, never blocks the producer.

    Producers (a MAVLink reader thread, ROS callbacks) call put(); the bridge's main loop calls get_many().
    Two rules protect the mission data: (1) only the NEWEST DroneFix is kept (older ones are worthless once a
    newer position exists, so a slow consumer never builds a backlog of positions); (2) when the queue is
    full the oldest NOTICE (news about the source itself) is dropped first, and only when there is none the oldest
    event of any kind, always counted, so memory stays bounded however fast the producer is. Target reports and
    releases are therefore the last thing to be lost.
    """

    def __init__(self, max_events: int = 1000):
        if max_events < 2:
            raise ValueError('max_events must be at least 2')
        self.max_events = max_events
        self._items: Deque[Event] = deque()
        self._cond = threading.Condition()
        self.dropped_fixes = 0
        self.dropped_other = 0

    def __len__(self) -> int:
        with self._cond:
            return len(self._items)

    def put(self, event: Event) -> None:
        """Add an event; never blocks and never raises."""
        with self._cond:
            if isinstance(event, DroneFix):
                for index, item in enumerate(self._items):
                    if isinstance(item, DroneFix):
                        del self._items[index]
                        self.dropped_fixes += 1
                        break
            if len(self._items) >= self.max_events:
                victim = next((i for i, item in enumerate(self._items) if isinstance(item, SourceNotice)), 0)
                del self._items[victim]
                self.dropped_other += 1
            self._items.append(event)
            self._cond.notify()

    def get_many(self, max_events: int, timeout_s: float) -> List[Event]:
        """Wait up to `timeout_s` for at least one event, then return up to `max_events` of them."""
        deadline = time.monotonic() + max(0.0, timeout_s)
        with self._cond:
            while not self._items:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return []
                self._cond.wait(remaining)
            out: List[Event] = []
            while self._items and len(out) < max_events:
                out.append(self._items.popleft())
            return out
