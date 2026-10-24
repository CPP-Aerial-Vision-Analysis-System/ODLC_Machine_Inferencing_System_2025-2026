"""
lattice_bridge/publisher.py -- PURPOSE: the part of the bridge that actually SENDS entities, built so that it can
never hurt the flight. Producers (the code that turns MAVLink telemetry and target reports into entities) call
`Publisher.submit(entity)` and carry on: submit() takes a lock for a few microseconds, files the entity and
returns. It never sleeps, never touches the network, never raises. One background thread does all the slow
and risky work (talking to a sink) and every failure it meets is caught, counted and logged.

The pipeline, in the order an entity meets the rules:

  1. STALE CHECK     an entity older (by sourceUpdateTime) than what is already waiting or already delivered is
                     ignored: Lattice would throw it away anyway ("latest wins" means latest MEASUREMENT).
  2. UNCHANGED CHECK identical content (ignoring the time stamps and the rolling expiry) to what was delivered
                     last is skipped, except that it is re-sent after `refresh_fraction` (half) of its lifetime
                     so a quiet entity never expires on the map.
  3. QUEUE           at most ONE waiting version per entity id ("latest wins": a newer version replaces the
                     waiting one, which is counted as coalesced). The queue holds at most `max_queue` ids; when it
                     overflows the entity with the OLDEST data is dropped and counted.
  4. RATE LIMIT      one delivery per entity per `min_interval_s` (default 1 s). While an entity waits for its turn
                     newer versions keep replacing it, so the newest state goes out.
  5. SEND            the sink is called outside every lock. Newest data first.
  6. RETRY           RetryableSinkError (link down, timeout, 5xx, 429 ...): the entity STAYS in the queue, the
                     whole publisher waits with exponential back-off (1 s, 2 s, 4 s ... up to 60 s, +-20 %
                     random jitter so many clients do not retry in step, and at least the server's own
                     Retry-After hint). One request per wait is a "probe"; the first success ends the outage.
  7. OFFLINE BUFFER  the queue IS the offline buffer: while the link is down it keeps the latest version of each
                     entity, bounded by `max_queue`; entities that expire while waiting are dropped (Lattice would
                     refuse them); when the link returns everything is flushed NEWEST FIRST, because the newest
                     state matters most and Lattice only accepts a sourceUpdateTime later than the one it holds.
  8. FATAL           FatalSinkError that concerns one entity (HTTP 400, 413 ...): that version is dropped and
                     counted, the rest carries on. FatalSinkError with systemic=True (HTTP 401/403, refused
                     certificate, wrong address): NOTHING is dropped and nothing is hammered; the publisher
                     probes once per `systemic_cooldown_s` (60 s), so a wrong token causes one request a minute,
                     never a retry storm.
  9. SHUTDOWN        stop() delivers what it still can within its time limit (data first), then sends an
                     isLive=false "goodbye" for every entity flagged with goodbye=True (the drone: its marker should
                     vanish when the bridge stops; targets and areas stay for the debrief), then closes the sink.

How to test it without real time: pass `clock` (monotonic seconds), `wall` (an aware UTC datetime), `sleep` and
`rng`, do NOT call start(), and drive it with pump(). start() runs the very same pump() in a thread.

Health: health() returns a plain dict of counters. The bookkeeping identity (checked by the tests):
    submitted + goodbyes_queued == enqueued + unchanged + stale
    enqueued == sent + coalesced + dropped_overflow + dropped_expired + dropped_fatal + dropped_shutdown
                + queue_depth + inflight

Python standard library only (json, logging, random, threading, time): no ROS.
"""

from __future__ import annotations

import json
import logging
import math
import random
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Any, Callable, Dict, List, Optional, Tuple

from .sinks.base import AuthSinkError, FatalSinkError, RetryableSinkError, SinkError, adapt_sink, short_text
from .timeutil import format_utc, parse_utc

LOG = logging.getLogger('lattice_bridge.publisher')

# What submit() answers. Plain words so a log line or a test reads naturally.
QUEUED = 'queued'          # filed as a new waiting version
COALESCED = 'coalesced'    # filed, and it replaced an older waiting version of the same entity
UNCHANGED = 'unchanged'    # skipped: Lattice already holds this content
STALE = 'stale'            # ignored: older than what is waiting or already delivered
DROPPED = 'dropped'        # the queue was full and this was the oldest data, so it was dropped
REJECTED = 'rejected'      # not an entity (no entityId, not JSON data, too big)
CLOSED = 'closed'          # the publisher is stopping or stopped
SUBMIT_RESULTS = frozenset({QUEUED, COALESCED, UNCHANGED, STALE, DROPPED, REJECTED, CLOSED})

# The states health() reports.
STATE_STARTING = 'starting'        # nothing sent yet
STATE_ONLINE = 'online'            # the last request got an answer
STATE_OFFLINE = 'offline'          # the last request failed in a way that may pass (back-off in force)
STATE_AUTH_FAILED = 'auth_failed'  # the credentials were refused (slow probing in force)
STATE_BLOCKED = 'blocked'          # another whole-connection problem (certificate, address) (slow probing)
STATE_STOPPING = 'stopping'
STATE_STOPPED = 'stopped'

#: Parts of an entity that change although nothing happened, so they are ignored when asking "is this the
#: same content as last time?". `*` means "every item of the list".
DEFAULT_IGNORED_PATHS: Tuple[str, ...] = (
    'provenance.sourceUpdateTime',       # the measurement time moves with every fix
    'expiryTime',                        # the drone's rolling expiry moves with every update
    'tracked.lastMeasurementTime',       # equals the measurement time
    'health.components.*.messages',      # texts like "Last telemetry 0.8 s ago." change every second
)

MAX_ENTITY_BYTES = 1 << 20      # an entity bigger than 1 MiB is not an entity
MAX_ENTITY_ID_LEN = 256
_MIN_TIME = datetime(1970, 1, 1, tzinfo=timezone.utc)      # "unknown time" sorts as the oldest
_MAX_LOG_KEYS = 32


def _number(name: str, value, low: float, high: float) -> float:
    """A finite number in [low, high] or ValueError (configuration mistakes should fail loudly at start-up)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number, got {value!r}')
    if not low <= value <= high:
        raise ValueError(f'{name} must be between {low} and {high}, got {value!r}')
    return float(value)


@dataclass(frozen=True)
class PublisherConfig:
    """Every tunable number of the Publisher, with the defaults explained (all times in seconds)."""

    #: Most waiting entities (one version each). Also the size of the offline buffer.
    max_queue: int = 256
    #: Shortest time between two deliveries of the same entity (the drone is published at ~1 Hz).
    min_interval_s: float = 1.0
    #: Back-off after consecutive retryable failures: initial * multiplier**(n-1), at most backoff_max_s,
    #: then +-jitter_fraction random. These are the numbers of the Lattice "Retry connections" guide.
    backoff_initial_s: float = 1.0
    backoff_multiplier: float = 2.0
    backoff_max_s: float = 60.0
    jitter_fraction: float = 0.2
    #: The longest pause a server's Retry-After hint can cause.
    retry_after_cap_s: float = 300.0
    #: Pause between probes after a systemic failure (wrong token, refused certificate ...).
    systemic_cooldown_s: float = 60.0
    #: An entity that expires within this many seconds is not sent (Lattice would refuse or delete it).
    expiry_margin_s: float = 1.0
    #: Unchanged content is re-sent once this fraction of its lifetime has passed.
    refresh_fraction: float = 0.5
    #: Most deliveries in one pump() call, so one call cannot run for ever.
    max_sends_per_pump: int = 50
    #: How many entity ids the publisher remembers things about (bounded memory).
    max_tracked: int = 1024
    #: How many entities may be flagged for a goodbye at shutdown.
    max_goodbye_entities: int = 64
    #: Lifetime given to a goodbye whose original expiry has already passed.
    goodbye_ttl_s: float = 120.0
    #: The worker never sleeps longer than this (a safety net, nothing depends on it).
    idle_poll_s: float = 5.0
    #: At most one warning per kind of problem per this many seconds.
    log_interval_s: float = 60.0
    #: Paths ignored when comparing content (see DEFAULT_IGNORED_PATHS).
    ignore_paths: Tuple[str, ...] = DEFAULT_IGNORED_PATHS

    def __post_init__(self):
        """Refuse nonsense values (negative waits, zero-sized queues ...)."""
        for name, low, high in (
            ('max_queue', 1, 100000), ('max_sends_per_pump', 1, 100000), ('max_tracked', 1, 1000000),
            ('max_goodbye_entities', 0, 100000),
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
                raise ValueError(f'{name} must be an integer from {low} to {high}, got {value!r}')
        _number('min_interval_s', self.min_interval_s, 0.0, 3600.0)
        _number('backoff_initial_s', self.backoff_initial_s, 0.0, 3600.0)
        _number('backoff_multiplier', self.backoff_multiplier, 1.0, 10.0)
        _number('backoff_max_s', self.backoff_max_s, 0.0, 86400.0)
        _number('jitter_fraction', self.jitter_fraction, 0.0, 1.0)
        _number('retry_after_cap_s', self.retry_after_cap_s, 0.0, 86400.0)
        _number('systemic_cooldown_s', self.systemic_cooldown_s, 0.0, 86400.0)
        _number('expiry_margin_s', self.expiry_margin_s, 0.0, 3600.0)
        _number('refresh_fraction', self.refresh_fraction, 0.0, 1.0)
        _number('goodbye_ttl_s', self.goodbye_ttl_s, 1.0, 86400.0)
        _number('idle_poll_s', self.idle_poll_s, 0.01, 3600.0)
        _number('log_interval_s', self.log_interval_s, 0.0, 86400.0)
        if self.backoff_max_s < self.backoff_initial_s:
            raise ValueError('backoff_max_s must not be smaller than backoff_initial_s')
        if not isinstance(self.ignore_paths, tuple) or not all(isinstance(p, str) and p for p in self.ignore_paths):
            raise ValueError('ignore_paths must be a tuple of dotted paths')


# =====================================================================================================
# Pure helpers (no clock, no I/O): the rules that deserve their own tests
# =====================================================================================================

def backoff_delay(failures: int, config: PublisherConfig, u: float, retry_after_s: Optional[float] = None) -> float:
    """Seconds to wait after the `failures`-th failure in a row (1 = the first one).

    base = backoff_initial_s * backoff_multiplier**(failures-1), at most backoff_max_s. `u` in [0, 1] is the
    random draw: the result is base * (1 +- jitter_fraction), u=0 giving the lowest and u=1 the highest. A
    server hint `retry_after_s` can only lengthen the wait (never beyond retry_after_cap_s).
    """
    count = max(1, int(failures))
    base = min(config.backoff_max_s, config.backoff_initial_s * config.backoff_multiplier ** min(count - 1, 64))
    draw = min(1.0, max(0.0, u if isinstance(u, (int, float)) and math.isfinite(u) else 0.5))
    delay = base * (1.0 + config.jitter_fraction * (2.0 * draw - 1.0))
    if isinstance(retry_after_s, (int, float)) and not isinstance(retry_after_s, bool) \
            and math.isfinite(retry_after_s) and retry_after_s > 0:
        delay = max(delay, min(float(retry_after_s), config.retry_after_cap_s))
    return delay


def _delete_path(node: Any, parts: List[str]) -> None:
    """Remove the dotted path (already split) from `node` in place; `*` walks every item of a list."""
    if not parts:
        return
    head, rest = parts[0], parts[1:]
    if head == '*':
        if isinstance(node, list):
            for item in node:
                _delete_path(item, rest)
        return
    if isinstance(node, dict) and head in node:
        if rest:
            _delete_path(node[head], rest)
        else:
            del node[head]


def content_fingerprint(entity: Dict[str, Any], ignore_paths: Tuple[str, ...] = DEFAULT_IGNORED_PATHS) -> str:
    """Canonical text of an entity without its volatile parts: equal text = "nothing new to tell Lattice".

    The entity is copied first (through JSON), so the caller's dict is never changed.
    """
    trimmed = json.loads(json.dumps(entity, sort_keys=True, allow_nan=False))
    for path in ignore_paths:
        _delete_path(trimmed, path.split('.'))
    return json.dumps(trimmed, sort_keys=True, separators=(',', ':'), allow_nan=False)


def make_goodbye(entity: Dict[str, Any], now: datetime, ttl_s: float = 120.0) -> Dict[str, Any]:
    """The "goodbye" update for `entity`: the same entity with isLive=false, which deletes it in Lattice.

    Lattice only accepts an update whose provenance.sourceUpdateTime is LATER than the one it holds, so the
    new time is `now`, or one microsecond after the entity's own time if that is later (a clock that went
    backwards must not produce a goodbye that Lattice ignores). Lattice also wants a future expiryTime on every
    entity: the old one is kept when it is still more than 30 s ahead, otherwise it becomes now + ttl_s.
    """
    goodbye = json.loads(json.dumps(entity, allow_nan=False))
    goodbye['isLive'] = False
    provenance = goodbye.setdefault('provenance', {})
    old = _time_at(goodbye, 'provenance', 'sourceUpdateTime')
    new_time = now if old is None or now > old else old + timedelta(microseconds=1)
    provenance['sourceUpdateTime'] = format_utc(new_time)
    expiry = _time_at(goodbye, 'expiryTime')
    if expiry is None or expiry <= now + timedelta(seconds=30):
        goodbye['expiryTime'] = format_utc(now + timedelta(seconds=ttl_s))
    return goodbye


def _time_at(entity: Dict[str, Any], *keys: str) -> Optional[datetime]:
    """The UTC time stored at entity[k1][k2]..., or None when missing or not Lattice time text."""
    node: Any = entity
    for key in keys:
        if not isinstance(node, dict):
            return None
        node = node.get(key)
    try:
        return parse_utc(node)
    except ValueError:
        return None


# =====================================================================================================
# The publisher
# =====================================================================================================

@dataclass
class _Item:
    """One waiting version of one entity."""
    entity_id: str
    entity: Dict[str, Any]
    fingerprint: str
    source_time: Optional[datetime]
    expiry_time: Optional[datetime]
    seq: int
    is_delete: bool
    force: bool = False
    goodbye: bool = False
    attempts: int = 0


@dataclass
class _Sent:
    """What was delivered last for one entity."""
    fingerprint: str
    sent_wall: datetime
    ttl_s: Optional[float]
    source_time: Optional[datetime]


@dataclass
class _Track:
    """Everything the publisher remembers about one entity id (bounded: see PublisherConfig.max_tracked)."""
    sent: Optional[_Sent] = None
    last_ok_mono: Optional[float] = None
    touched: bool = False                       # a sink call was made: Lattice may hold something
    goodbye: bool = False                       # flagged: send isLive=false at shutdown
    decided: bool = False                       # the goodbye flag has been set once (by a caller or the default)
    last_entity: Optional[Dict[str, Any]] = None    # newest content, kept only for flagged entities


def _order_key(item: _Item, drain: bool) -> Tuple[int, datetime, int]:
    """Bigger = sent earlier: newest data first; while draining at shutdown, goodbyes go last."""
    return (0 if (drain and item.goodbye) else 1, item.source_time or _MIN_TIME, item.seq)


class Publisher:
    """Thread-based, bounded, latest-wins publisher in front of a sink. See the module docstring."""

    def __init__(
        self,
        sink: Any,
        config: Optional[PublisherConfig] = None,
        *,
        clock: Optional[Callable[[], float]] = None,
        wall: Optional[Callable[[], datetime]] = None,
        sleep: Optional[Callable[[float], Any]] = None,
        rng: Optional[Callable[[], float]] = None,
        logger: Optional[logging.Logger] = None,
        close_sink: bool = True,
        goodbye_default: bool = False,
    ):
        self._sink = adapt_sink(sink)
        self._cfg = config or PublisherConfig()
        self._clock = clock or time.monotonic
        self._wall = wall or (lambda: datetime.now(timezone.utc))
        self._rng = rng or random.Random().random
        self._log = logger or LOG
        self._close_sink = bool(close_sink)
        self._goodbye_default = bool(goodbye_default)
        self._wakeup = threading.Event()
        self._sleep = sleep or self._wakeup.wait        # the default pause ends early when something arrives
        self._pause = sleep or time.sleep               # shutdown waits: nothing to wake up for, wait it out
        self._lock = threading.Lock()
        self._pump_lock = threading.Lock()
        self._stop_lock = threading.Lock()
        self._queue: Dict[str, _Item] = {}
        self._tracks: 'OrderedDict[str, _Track]' = OrderedDict()
        self._inflight: Optional[_Item] = None
        self._seq = 0
        self._goodbye_flags = 0
        self._blocked_until = 0.0
        self._consecutive_failures = 0
        self._state = STATE_STARTING
        self._connected = False
        self._last_error: Optional[str] = None
        self._last_error_wall: Optional[datetime] = None
        self._last_ok_wall: Optional[datetime] = None
        self._outage_started_mono: Optional[float] = None
        self._log_times: Dict[str, float] = {}
        self._logs: List[Tuple[int, str]] = []
        self._counters: Dict[str, int] = {name: 0 for name in (
            'submitted', 'enqueued', 'sent', 'goodbyes_queued', 'goodbyes_sent', 'retried', 'coalesced',
            'unchanged', 'stale', 'rejected', 'closed_rejected', 'failed_retryable', 'failed_fatal',
            'systemic_failures', 'dropped_overflow', 'dropped_expired', 'dropped_fatal', 'dropped_shutdown',
            'internal_errors', 'unexpected_errors',
        )}
        self._closing = False
        self._stopped = False
        self._stop_ok = False
        self._stop_deadline = 0.0
        self._thread: Optional[threading.Thread] = None

    # ================================================================================== producers
    def submit(self, entity: Any, *, force: bool = False, goodbye: Optional[bool] = None) -> str:
        """File `entity` for delivery. Never blocks on I/O, never raises. Returns one of SUBMIT_RESULTS.

        force=True skips the unchanged check and the rate limit (use it for one-off events such as a payload
        drop). goodbye=True flags the entity so stop() sends it an isLive=false update (use it for the drone);
        goodbye=False removes the flag; None keeps the current flag (new entities get `goodbye_default`).
        The entity is copied: changing the dict afterwards changes nothing here.
        """
        try:
            result = self._submit(entity, bool(force), goodbye)
        except Exception:                                   # a bug here must never reach the caller's callback
            with self._lock:
                self._counters['internal_errors'] += 1
            return REJECTED
        if result in (QUEUED, COALESCED, DROPPED):
            self._wakeup.set()
        return result

    def _submit(self, entity: Any, force: bool, goodbye: Optional[bool]) -> str:
        """The work of submit(), allowed to raise (submit() turns that into a counter)."""
        if not isinstance(entity, dict):
            return self._reject()
        entity_id = entity.get('entityId')
        if not isinstance(entity_id, str) or not entity_id or len(entity_id) > MAX_ENTITY_ID_LEN:
            return self._reject()
        try:
            text = json.dumps(entity, sort_keys=True, separators=(',', ':'), allow_nan=False)
        except (TypeError, ValueError, RecursionError):
            return self._reject()
        if len(text) > MAX_ENTITY_BYTES:
            return self._reject()
        snapshot = json.loads(text)
        fingerprint = content_fingerprint(snapshot, self._cfg.ignore_paths)
        source_time = _time_at(snapshot, 'provenance', 'sourceUpdateTime')
        expiry_time = _time_at(snapshot, 'expiryTime')
        wall_now = self._wall()
        with self._lock:
            if self._closing:
                self._counters['closed_rejected'] += 1
                return CLOSED
            self._counters['submitted'] += 1
            track = self._track(entity_id)
            waiting = self._queue.get(entity_id)
            inflight = self._inflight if self._inflight is not None and self._inflight.entity_id == entity_id else None
            references = [t for t in (
                waiting.source_time if waiting else None,
                inflight.source_time if inflight else None,
                track.sent.source_time if track.sent else None) if t is not None]
            if source_time is not None and references and source_time < max(references):
                self._counters['stale'] += 1
                return STALE
            self._apply_goodbye_flag(track, goodbye)
            if track.goodbye:
                track.last_entity = snapshot
            is_delete = snapshot.get('isLive') is False
            if (not force and not is_delete and waiting is None and inflight is None and track.sent is not None
                    and track.sent.fingerprint == fingerprint and self._is_fresh(track.sent, wall_now)):
                self._counters['unchanged'] += 1
                return UNCHANGED
            if (not force and not is_delete and waiting is not None and not waiting.force and track.sent is not None
                    and track.sent.fingerprint == fingerprint and self._is_fresh(track.sent, wall_now)):
                # what is waiting is a change that has been undone again: Lattice already holds the new state
                del self._queue[entity_id]
                self._counters['coalesced'] += 1
                self._counters['unchanged'] += 1
                return UNCHANGED
            self._seq += 1
            item = _Item(entity_id, snapshot, fingerprint, source_time, expiry_time, self._seq, is_delete,
                         force=force)
            replaced = waiting is not None
            if replaced:
                self._counters['coalesced'] += 1
            self._counters['enqueued'] += 1
            survived = self._insert(item)
            if not survived:
                return DROPPED
            return COALESCED if replaced else QUEUED

    def _reject(self) -> str:
        """Count one refused input and answer REJECTED."""
        with self._lock:
            self._counters['rejected'] += 1
        return REJECTED

    def _apply_goodbye_flag(self, track: _Track, goodbye: Optional[bool]) -> None:
        """Set or clear the goodbye flag of `track` (caller holds the lock).

        None means "no opinion": the first time an entity is seen it gets `goodbye_default`, afterwards its
        flag is left alone. An explicit True / False always wins (True only up to max_goodbye_entities).
        """
        if goodbye is None:
            if track.decided:
                return
            goodbye = self._goodbye_default
        track.decided = True
        if goodbye and not track.goodbye:
            if self._goodbye_flags < self._cfg.max_goodbye_entities:
                track.goodbye = True
                self._goodbye_flags += 1
        elif not goodbye and track.goodbye:
            track.goodbye = False
            track.last_entity = None
            self._goodbye_flags -= 1

    def _is_fresh(self, sent: _Sent, wall_now: datetime) -> bool:
        """True while the delivered copy still has most of its lifetime left (no need to refresh it)."""
        if sent.ttl_s is None or sent.ttl_s <= 0:
            return False
        age = (wall_now - sent.sent_wall).total_seconds()
        return 0 <= age < sent.ttl_s * self._cfg.refresh_fraction

    def _track(self, entity_id: str) -> _Track:
        """The memory about `entity_id`, created on demand and kept in least-recently-used order (lock held)."""
        track = self._tracks.get(entity_id)
        if track is None:
            track = _Track()
            self._tracks[entity_id] = track
            excess = len(self._tracks) - self._cfg.max_tracked
            if excess > 0:
                for old_id in list(self._tracks):
                    if excess <= 0:
                        break
                    old = self._tracks[old_id]
                    if old_id == entity_id or old_id in self._queue or old.goodbye:
                        continue
                    del self._tracks[old_id]
                    excess -= 1
        else:
            self._tracks.move_to_end(entity_id)
        return track

    def _insert(self, item: _Item) -> bool:
        """Put `item` in the queue (one version per id). If the queue is over its bound, drop the entity with
        the OLDEST data, which may be the new item itself. Returns True when `item` is still queued. Lock held.
        """
        self._queue[item.entity_id] = item
        survived = True
        while len(self._queue) > self._cfg.max_queue:
            victim = min(self._queue.values(), key=lambda it: (it.goodbye, it.source_time or _MIN_TIME, it.seq))
            del self._queue[victim.entity_id]
            self._counters['dropped_overflow'] += 1
            self._note('overflow', logging.WARNING,
                       f'queue full ({self._cfg.max_queue} entities): dropped the oldest data ({victim.entity_id})')
            if victim is item:
                survived = False
        return survived

    # ================================================================================== the worker
    def pump(self) -> int:
        """Do everything that is due right now, once. Returns the number of sink calls made.

        start() calls this in a loop in its thread; tests call it directly with a fake clock. Do not mix the
        two on one Publisher (a lock keeps them from running at the same time, but their timing would be odd).
        """
        with self._pump_lock:
            sends = 0
            while sends < self._cfg.max_sends_per_pump and not self._closing:
                item = self._take_next(drain=False)
                if item is None:
                    break
                self._deliver(item)
                sends += 1
            return sends

    def next_delay(self) -> Optional[float]:
        """Seconds until pump() will have something to do (0 = now), or None when nothing is waiting."""
        now = self._clock()
        with self._lock:
            if not self._queue:
                return None
            base = max(0.0, self._blocked_until - now)
            best: Optional[float] = None
            for entity_id, item in self._queue.items():
                wait = base
                if not (item.force or item.goodbye):
                    track = self._tracks.get(entity_id)
                    if track is not None and track.last_ok_mono is not None:
                        wait = max(wait, track.last_ok_mono + self._cfg.min_interval_s - now)
                if best is None or wait < best:
                    best = wait
            return max(0.0, best if best is not None else 0.0)

    def pending(self) -> List[str]:
        """The ids of the waiting entities in the order they would be sent (newest data first)."""
        with self._lock:
            items = sorted(self._queue.values(), key=lambda it: _order_key(it, False), reverse=True)
            return [it.entity_id for it in items]

    def _take_next(self, *, drain: bool) -> Optional[_Item]:
        """Pick the next entity that may be sent now and mark it in flight; None when nothing is due.

        Drops waiting entities that have expired (Lattice would refuse them) on the way.
        """
        now = self._clock()
        wall_now = self._wall()
        margin = timedelta(seconds=self._cfg.expiry_margin_s)
        with self._lock:
            if now < self._blocked_until or self._inflight is not None:
                return None
            best: Optional[_Item] = None
            best_key = None
            expired: List[str] = []
            for entity_id, item in self._queue.items():
                if item.expiry_time is not None and item.expiry_time <= wall_now + margin:
                    expired.append(entity_id)
                    continue
                if not (drain or item.force or item.goodbye):
                    track = self._tracks.get(entity_id)
                    if track is not None and track.last_ok_mono is not None \
                            and now < track.last_ok_mono + self._cfg.min_interval_s:
                        continue                            # its turn has not come yet
                key = _order_key(item, drain)
                if best_key is None or key > best_key:
                    best, best_key = item, key
            for entity_id in expired:
                del self._queue[entity_id]
                self._counters['dropped_expired'] += 1
            if best is None:
                return None
            del self._queue[best.entity_id]
            self._inflight = best
            if best.attempts > 0:
                self._counters['retried'] += 1
            return best

    def _deliver(self, item: _Item) -> None:
        """Call the sink for `item` (no lock held) and book the outcome."""
        stamp = self._wall()
        try:
            try:
                self._sink.publish_entity(item.entity, stamp)
            except RetryableSinkError as exc:
                self._after_retryable(item, exc)
            except FatalSinkError as exc:
                self._after_fatal(item, exc)
            except SinkError as exc:                           # a bare SinkError: treat it as retryable
                self._after_retryable(item, RetryableSinkError(exc.message, status=exc.status))
            except Exception as exc:                           # a sink bug: keep the data, back off, say so
                with self._lock:
                    self._counters['unexpected_errors'] += 1
                self._after_retryable(
                    item, RetryableSinkError(f'unexpected {type(exc).__name__}: {short_text(exc, 120)}'))
            else:
                self._after_success(item)
        finally:
            with self._lock:                                   # whatever happened, nothing stays "in flight"
                if self._inflight is item:
                    self._inflight = None
            self._emit_logs()

    def _after_success(self, item: _Item) -> None:
        """Book a delivery: counters, the "last delivered" memory, recovery from an outage."""
        now = self._clock()
        wall_now = self._wall()
        with self._lock:
            self._inflight = None
            self._counters['sent'] += 1
            if item.goodbye:
                self._counters['goodbyes_sent'] += 1
            track = self._track(item.entity_id)
            track.last_ok_mono = now
            if item.is_delete:
                track.sent = None
                track.touched = False
                if track.goodbye:
                    track.goodbye = False
                    self._goodbye_flags -= 1
                track.last_entity = None
            else:
                track.touched = True
                ttl = (item.expiry_time - wall_now).total_seconds() if item.expiry_time is not None else None
                track.sent = _Sent(item.fingerprint, wall_now, ttl, item.source_time)
            self._last_ok_wall = wall_now
            self._recover('Lattice is reachable again')

    def _recover(self, text: str) -> None:
        """Leave an outage state (lock held): reset the back-off and queue one INFO line about it."""
        was = self._state
        self._connected = True
        self._consecutive_failures = 0
        self._blocked_until = 0.0
        if not self._closing:
            self._state = STATE_ONLINE
        if was not in (STATE_ONLINE, STATE_STARTING, STATE_STOPPING):
            since = ''
            if self._outage_started_mono is not None:
                since = f' after {max(0.0, self._clock() - self._outage_started_mono):.0f} s'
            self._logs.append((logging.INFO, f'{text}{since}; {len(self._queue)} update(s) waiting are being sent'))
        self._outage_started_mono = None

    def _after_retryable(self, item: _Item, exc: RetryableSinkError) -> None:
        """Book a transient failure: keep the item, back off for everybody."""
        now = self._clock()
        wall_now = self._wall()
        with self._lock:
            self._inflight = None
            self._counters['failed_retryable'] += 1
            self._track(item.entity_id).touched = True
            self._consecutive_failures += 1
            delay = backoff_delay(self._consecutive_failures, self._cfg, self._draw(), exc.retry_after_s)
            self._blocked_until = now + delay
            first = self._state != STATE_OFFLINE
            self._connected = False
            self._state = STATE_OFFLINE
            if self._outage_started_mono is None:
                self._outage_started_mono = now
            self._remember_error(exc, wall_now)
            self._put_back(item)
            text = (f'Lattice is not reachable ({self._last_error}); {len(self._queue)} update(s) are kept; '
                    f'next try in {delay:.1f} s')
            if first:
                self._log_times['outage'] = now                    # the first line counts for the rate limit too
                self._logs.append((logging.WARNING, text))
            else:
                self._note('outage', logging.WARNING, text)

    def _after_fatal(self, item: _Item, exc: FatalSinkError) -> None:
        """Book a refusal: drop this version (it concerns this entity only) or pause (it concerns everything)."""
        now = self._clock()
        wall_now = self._wall()
        with self._lock:
            self._inflight = None
            self._counters['failed_fatal'] += 1
            self._track(item.entity_id).touched = True
            self._remember_error(exc, wall_now)
            if exc.systemic:
                self._counters['systemic_failures'] += 1
                self._connected = False
                self._state = STATE_AUTH_FAILED if isinstance(exc, AuthSinkError) else STATE_BLOCKED
                self._blocked_until = now + self._cfg.systemic_cooldown_s
                self._put_back(item)
                self._note('systemic', logging.ERROR,
                           f'Lattice refuses every request ({self._last_error}); nothing is dropped, one probe '
                           f'every {self._cfg.systemic_cooldown_s:.0f} s until it is fixed')
            else:
                self._counters['dropped_fatal'] += 1
                self._recover('Lattice answers again')               # it answered, so the link is fine
                self._note('fatal-drop', logging.WARNING,
                           f'Lattice rejected {item.entity_id} ({self._last_error}); that version was dropped')

    def _put_back(self, item: _Item) -> None:
        """Return a failed item to the queue unless a newer version of the same entity arrived meanwhile."""
        item.attempts += 1
        other = self._queue.get(item.entity_id)
        if other is None:
            self._insert(item)
            return
        mine, theirs = item.source_time or _MIN_TIME, other.source_time or _MIN_TIME
        self._counters['coalesced'] += 1                           # one of the two versions is now superseded
        if mine > theirs:
            self._queue[item.entity_id] = item

    def _draw(self) -> float:
        """One random number in [0, 1] for the back-off jitter (a broken generator must not break us)."""
        try:
            value = float(self._rng())
        except Exception:
            return 0.5
        return min(1.0, max(0.0, value)) if math.isfinite(value) else 0.5

    def _remember_error(self, exc: SinkError, wall_now: datetime) -> None:
        """Keep the newest error text for health() (lock held). Sinks never put secrets into their messages."""
        status = f' [HTTP {exc.status}]' if exc.status is not None else ''
        self._last_error = f'{type(exc).__name__}: {short_text(exc.message)}{status}'
        self._last_error_wall = wall_now

    def _note(self, key: str, level: int, text: str) -> None:
        """Queue a log line, but at most one per `key` every log_interval_s (lock held)."""
        now = self._clock()
        last = self._log_times.get(key)
        if last is not None and now - last < self._cfg.log_interval_s:
            return
        if key not in self._log_times and len(self._log_times) >= _MAX_LOG_KEYS:
            return
        self._log_times[key] = now
        self._logs.append((level, text))

    def _emit_logs(self) -> None:
        """Write the queued log lines (no lock held while logging)."""
        with self._lock:
            lines, self._logs = self._logs, []
        for level, text in lines:
            try:
                self._log.log(level, '%s', text)
            except Exception:                                  # a broken log handler must not stop the worker
                pass

    # ================================================================================== life cycle
    def start(self) -> 'Publisher':
        """Start the worker thread (once). Returns self so `Publisher(...).start()` reads well."""
        with self._stop_lock:
            if self._thread is None and not self._closing:
                self._thread = threading.Thread(target=self._run, name='lattice-publisher', daemon=True)
                self._thread.start()
        return self

    @property
    def running(self) -> bool:
        """True while the worker thread is alive."""
        thread = self._thread
        return thread is not None and thread.is_alive()

    def _run(self) -> None:
        """The worker loop: pump, then wait until the next thing is due or something arrives."""
        try:
            while not self._closing:
                self._wakeup.clear()                           # cleared BEFORE pumping: a submit() during the
                try:                                           # pump sets it again, so no wake-up is ever lost
                    sends = self.pump()
                    delay = self.next_delay()
                except Exception as exc:                       # keep the thread alive whatever happens
                    with self._lock:
                        self._counters['internal_errors'] += 1
                        self._note('internal', logging.ERROR, f'internal error in the worker: {short_text(exc)}')
                    self._emit_logs()
                    self._sleep(0.5)
                    continue
                self._emit_logs()
                if self._closing:
                    break
                if delay is None:
                    self._wakeup.wait(self._cfg.idle_poll_s)
                else:
                    if sends == 0 and delay <= 0:
                        delay = 0.01                           # due but nothing sent: never spin at full speed
                    self._sleep(min(delay, self._cfg.idle_poll_s))
            self._shutdown_sequence()
        except Exception as exc:                               # last resort: never let the thread die noisily
            with self._lock:
                self._counters['internal_errors'] += 1
            self._log.error('publisher worker stopped by an internal error: %s', short_text(exc))

    def stop(self, timeout_s: float = 10.0) -> bool:
        """Shut down in an orderly way within about `timeout_s` seconds. Safe to call twice, from any thread.

        Order: refuse new submissions; send what is still waiting (data first, within the time limit); send
        the goodbyes (isLive=false) of the flagged entities; close the sink. Returns True when nothing had to
        be thrown away (everything waiting was delivered), otherwise False.
        """
        with self._stop_lock:
            if self._stopped:
                return self._stop_ok
            timeout = max(0.0, float(timeout_s)) if isinstance(timeout_s, (int, float)) else 10.0
            with self._lock:
                self._closing = True
                self._state = STATE_STOPPING
                self._stop_deadline = self._clock() + timeout
            self._wakeup.set()
            thread = self._thread
            if thread is None:
                self._shutdown_sequence()
            elif thread is not threading.current_thread():
                thread.join(timeout + 1.0)
            with self._lock:
                clean = (self._counters['dropped_shutdown'] == 0 and not self._queue
                         and (thread is None or not thread.is_alive()))
            if self._close_sink:
                try:
                    self._sink.close()
                except Exception as exc:
                    self._log.warning('closing the sink failed: %s', short_text(exc))
            with self._lock:
                self._state = STATE_STOPPED
                self._connected = False
            self._stopped = True
            self._stop_ok = clean
            return clean

    def __enter__(self) -> 'Publisher':
        return self.start()

    def __exit__(self, *exc_info) -> None:
        self.stop()

    def _shutdown_sequence(self) -> None:
        """Flush, then say goodbye, within the deadline that stop() set. Runs in the worker (or in stop())."""
        with self._pump_lock:
            with self._lock:
                self._prepare_goodbyes()
            while True:
                with self._lock:
                    if not self._queue:
                        break
                    blocked = self._blocked_until
                now = self._clock()
                if now >= self._stop_deadline:
                    break
                if blocked > now:
                    if blocked >= self._stop_deadline:
                        break                                   # not worth waiting for: the deadline comes first
                    self._pause(blocked - now)
                    continue
                item = self._take_next(drain=True)
                if item is None:
                    break
                self._deliver(item)
            with self._lock:
                left = len(self._queue)
                self._queue.clear()
                self._counters['dropped_shutdown'] += left
                if left:
                    self._logs.append((logging.WARNING, f'shutdown: {left} update(s) could not be delivered in time'))
            self._emit_logs()

    def _prepare_goodbyes(self) -> None:
        """Queue the isLive=false update of every flagged entity (lock held)."""
        wall_now = self._wall()
        for entity_id, track in list(self._tracks.items()):
            if not track.goodbye:
                continue
            waiting = self._queue.get(entity_id)
            if not track.touched:
                # Lattice never heard of it, so there is nothing to delete; a waiting version just goes away.
                if waiting is not None:
                    del self._queue[entity_id]
                    self._counters['dropped_shutdown'] += 1
                continue
            base = waiting.entity if waiting is not None else track.last_entity
            if base is None:
                continue
            goodbye = make_goodbye(base, wall_now, self._cfg.goodbye_ttl_s)
            if waiting is not None:
                del self._queue[entity_id]
                self._counters['coalesced'] += 1
            self._seq += 1
            item = _Item(entity_id, goodbye, content_fingerprint(goodbye, self._cfg.ignore_paths),
                         _time_at(goodbye, 'provenance', 'sourceUpdateTime'), _time_at(goodbye, 'expiryTime'),
                         self._seq, True, force=True, goodbye=True)
            self._counters['goodbyes_queued'] += 1
            self._counters['enqueued'] += 1
            self._queue[entity_id] = item

    # ================================================================================== reporting
    def health(self) -> Dict[str, Any]:
        """A snapshot of the counters and the state, as a plain dict (safe to log or publish as JSON)."""
        now = self._clock()
        with self._lock:
            c = dict(self._counters)
            next_in = None
            if self._queue:
                next_in = max(0.0, self._blocked_until - now)
            dropped = c['dropped_overflow'] + c['dropped_expired'] + c['dropped_fatal'] + c['dropped_shutdown']
            report: Dict[str, Any] = dict(c)
            report.update({
                'state': self._state,
                'connected': self._connected,
                'dropped': dropped,
                'queue_depth': len(self._queue),
                'inflight': 1 if self._inflight is not None else 0,
                'consecutive_failures': self._consecutive_failures,
                'next_attempt_in_s': next_in,
                'last_error': self._last_error,
                'last_error_at': format_utc(self._last_error_wall) if self._last_error_wall else None,
                'last_success_at': format_utc(self._last_ok_wall) if self._last_ok_wall else None,
            })
        try:
            report['sink'] = self._sink.describe()
        except Exception:                                      # a broken describe() must not break health()
            report['sink'] = type(self._sink).__name__
        return report
