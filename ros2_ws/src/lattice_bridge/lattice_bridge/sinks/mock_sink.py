"""
lattice_bridge/sinks/mock_sink.py -- PURPOSE: a sink that lives entirely in memory, for unit tests. It keeps
every entity it was given, and it can be told to misbehave on purpose (go "down", fail the next N requests with
a chosen error, block inside a request), so the Publisher's retry, back-off, offline-buffer and shutdown rules can
be tested in microseconds, with no network, no sockets and no real waiting.

It also plays Lattice's own bookkeeping rule: an update is only taken into the picture when its
provenance.sourceUpdateTime is LATER than the one already held (the live PublishEntity reference says Lattice
"will discard the update" otherwise), and isLive=false removes the entity. `entities()` is therefore "what the
map would show now", and `stale_discards` counts deliveries that Lattice would have thrown away: a Publisher
that sends an older version after a newer one makes that number grow, which the tests watch.

Thread safe (one lock); the optional hook runs OUTSIDE the lock so a test can block a request on purpose.

Python standard library only.
"""

from __future__ import annotations

import copy
import hashlib
import threading
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional

from ..timeutil import parse_utc
from .base import BaseSink, FatalSinkError, ObjectInfo, RetryableSinkError, SinkError


@dataclass(frozen=True)
class Delivery:
    """One successful delivery: a private copy of the entity, the request time and a running number."""
    entity: Dict[str, Any]
    t: Optional[datetime]
    number: int


@dataclass(frozen=True)
class Attempt:
    """One call of publish_entity, successful or not (`error` is the exception class name, or None)."""
    entity_id: Optional[str]
    ok: bool
    error: Optional[str]


class MockSink(BaseSink):
    """An in-memory sink with scripted failures. See the module docstring."""

    name = 'mock'

    def __init__(self, *, enforce_source_time: bool = True):
        self.enforce_source_time = bool(enforce_source_time)
        self._lock = threading.Lock()
        self._deliveries: List[Delivery] = []
        self._attempts: List[Attempt] = []
        self._picture: Dict[str, Dict[str, Any]] = {}
        self._queued_errors: List[SinkError] = []
        self._down_error: Optional[SinkError] = None
        self._hook: Optional[Callable[[Dict[str, Any]], None]] = None
        self._objects: Dict[str, bytes] = {}
        self._closed = False
        self.stale_discards = 0
        self.close_calls = 0

    # ------------------------------------------------------------------------------ scripting
    def fail_next(self, error: SinkError, times: int = 1) -> None:
        """The next `times` publish_entity calls raise `error` (and deliver nothing)."""
        with self._lock:
            self._queued_errors.extend([error] * int(times))

    def go_down(self, error: Optional[SinkError] = None) -> None:
        """Every publish_entity call raises `error` (default: a RetryableSinkError) until come_back()."""
        with self._lock:
            self._down_error = error or RetryableSinkError('mock sink is down')

    def come_back(self) -> None:
        """Undo go_down() and forget any queued errors."""
        with self._lock:
            self._down_error = None
            self._queued_errors.clear()

    def set_hook(self, hook: Optional[Callable[[Dict[str, Any]], None]]) -> None:
        """Call `hook(entity)` at the start of every publish_entity (outside the lock).

        The hook may block (to hold a request open), record something, or raise a SinkError to fail it.
        """
        self._hook = hook

    # ------------------------------------------------------------------------------ the sink API
    def publish_entity(self, entity: Dict[str, Any], t: Optional[datetime] = None) -> None:
        """Record the delivery, or raise the scripted error."""
        hook = self._hook
        entity_id = entity.get('entityId') if isinstance(entity, dict) else None
        try:
            if hook is not None:
                hook(entity)
            with self._lock:
                if self._closed:
                    raise FatalSinkError('the mock sink is closed')
                if self._queued_errors:
                    raise self._queued_errors.pop(0)
                if self._down_error is not None:
                    raise self._down_error
                snapshot = copy.deepcopy(entity)
                self._deliveries.append(Delivery(snapshot, t, len(self._deliveries) + 1))
                self._apply_to_picture(snapshot)
        except SinkError as exc:
            with self._lock:
                self._attempts.append(Attempt(entity_id, False, type(exc).__name__))
            raise
        with self._lock:
            self._attempts.append(Attempt(entity_id, True, None))

    def upload_object(self, path: str, data: bytes, *, ttl_s: Optional[float] = None,
                      t: Optional[datetime] = None) -> ObjectInfo:
        """Keep the bytes in memory under `path`; the same scripted failures apply."""
        with self._lock:
            if self._closed:
                raise FatalSinkError('the mock sink is closed')
            if self._queued_errors:
                raise self._queued_errors.pop(0)
            if self._down_error is not None:
                raise self._down_error
            self._objects[path] = bytes(data)
        return ObjectInfo(path=path, size_bytes=len(data), checksum=hashlib.sha256(data).hexdigest())

    def close(self) -> None:
        """Mark the sink closed (later publishes raise FatalSinkError). Counts the calls."""
        with self._lock:
            self._closed = True
            self.close_calls += 1

    def describe(self) -> str:
        """'mock (in memory)'."""
        return 'mock (in memory)'

    # ------------------------------------------------------------------------------ results
    @property
    def deliveries(self) -> List[Delivery]:
        """A copy of the successful deliveries, oldest first."""
        with self._lock:
            return list(self._deliveries)

    @property
    def attempts(self) -> List[Attempt]:
        """A copy of every call, oldest first (failures included)."""
        with self._lock:
            return list(self._attempts)

    @property
    def calls(self) -> int:
        """How many times publish_entity was called (failures included)."""
        with self._lock:
            return len(self._attempts)

    def delivered_ids(self) -> List[str]:
        """The entityId of every successful delivery, in delivery order."""
        with self._lock:
            return [d.entity.get('entityId') for d in self._deliveries]

    def entities(self) -> Dict[str, Dict[str, Any]]:
        """The picture Lattice would hold now: newest accepted version per id; isLive=false removed it."""
        with self._lock:
            return copy.deepcopy(self._picture)

    def objects(self) -> Dict[str, bytes]:
        """The uploaded objects by path."""
        with self._lock:
            return dict(self._objects)

    # ------------------------------------------------------------------------------ internals
    def _apply_to_picture(self, entity: Dict[str, Any]) -> None:
        """Lattice's rule: an update counts only if its sourceUpdateTime is later than the stored one."""
        entity_id = entity.get('entityId')
        if not isinstance(entity_id, str):
            return
        old = self._picture.get(entity_id)
        if self.enforce_source_time and old is not None:
            new_time = _source_time(entity)
            old_time = _source_time(old)
            if new_time is not None and old_time is not None and new_time <= old_time:
                self.stale_discards += 1
                return
        if entity.get('isLive') is False:
            self._picture.pop(entity_id, None)
        else:
            self._picture[entity_id] = entity


def _source_time(entity: Dict[str, Any]) -> Optional[datetime]:
    """provenance.sourceUpdateTime as a datetime, or None when missing or malformed."""
    provenance = entity.get('provenance')
    text = provenance.get('sourceUpdateTime') if isinstance(provenance, dict) else None
    try:
        return parse_utc(text)
    except ValueError:
        return None
