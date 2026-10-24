"""
lattice_bridge/sinks/jsonl_sink.py -- PURPOSE: write every request the bridge would send to Lattice into a
plain text file, one JSON object per line ("JSONL"):

    {"kind":"entity","payload":{...the entity...},"t":"2026-10-24T15:30:01.000000Z"}

Two jobs with one file. (1) OFFLINE MODE: at the field there may be no internet; the recording keeps
everything, and it can be replayed into Lattice later. (2) TEST ORACLE: tests and the simulator end-to-end
check read the file back and assert exactly what would have been published, with no Lattice account.

Properties that matter:
  * APPEND-ONLY: lines are only ever added at the end, each written with one write() call and flushed, so a
    crash loses at most the line being written (readers skip such a torn last line).
  * NEVER RAISES for I/O trouble: write() returns False and logs a warning (disk full, folder gone, payload
    not JSON). A recorder must never take the bridge down, and the bridge never takes the flight down.
  * BOUNDED: stops accepting lines after `max_bytes` (default 256 MiB) so a runaway loop cannot fill the disk.
  * NO SECRETS: only the payload and the kind are stored, never headers or tokens (the sink is never given any).
  * Thread safe (one lock around the file), and `t` is passed in (the clock is only read when `t` is None).

Reading: read_records(path) gives back Record objects; latest_by_entity_id(path) keeps the newest entity
payload per entityId, which is what "what does Lattice look like now" means.

Python standard library only (json, logging, threading, os): no ROS.
"""

from __future__ import annotations

import json
import logging
import os
import re
import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Union

from ..timeutil import format_utc, parse_utc, require_aware
from .base import BaseSink, FatalSinkError

LOG = logging.getLogger('lattice_bridge.jsonl_sink')

KIND_ENTITY = 'entity'          # the body of one PUT /api/v1/entities
KIND_NOTE = 'note'              # free text for humans reading the file (start/stop markers, warnings)
DEFAULT_MAX_BYTES = 256 * 1024 * 1024
_KIND = re.compile(r'[a-z][a-z0-9_]{0,31}')


@dataclass(frozen=True)
class Record:
    """One line of a recording, read back."""
    t: datetime
    kind: str
    payload: Any
    line_no: int


@dataclass(frozen=True)
class ReadResult:
    """What read_records() found: the good records, and how many lines were unusable."""
    records: List[Record]
    bad_lines: int


class JsonlSink(BaseSink):
    """Append requests to a JSONL file. Use as a context manager or call close() yourself.

    It is also a BaseSink (sinks/base.py), so a Publisher can feed it: publish_entity() records the entity and
    raises FatalSinkError when the recorder refused it (the reason is in the log).
    """

    name = 'jsonl'

    def __init__(self, path: Union[str, Path], *, max_bytes: int = DEFAULT_MAX_BYTES, fsync: bool = False,
                 clock: Optional[Callable[[], datetime]] = None):
        if isinstance(max_bytes, bool) or not isinstance(max_bytes, int) or max_bytes < 1:
            raise ValueError('max_bytes must be a positive integer')
        self.path = Path(path)
        self.max_bytes = max_bytes
        self.fsync = bool(fsync)
        self._clock = clock or (lambda: datetime.now(timezone.utc))
        self._lock = threading.Lock()
        self._file = None
        self._size = 0
        self._closed = False
        self.written = 0            # lines accepted
        self.failed = 0             # lines refused or lost (see the log)

    # ---------------------------------------------------------------------------------- writing
    def write(self, kind: str, payload: Any, t: Optional[datetime] = None) -> bool:
        """Append one record; True when it was written, False (and a log line) when it was not."""
        try:
            if not isinstance(kind, str) or _KIND.fullmatch(kind) is None:
                raise ValueError(f'kind must be a short lower-case word, got {str(kind)[:30]!r}')
            moment = self._clock() if t is None else t
            stamp = format_utc(require_aware('t', moment))
            line = json.dumps({'t': stamp, 'kind': kind, 'payload': payload}, sort_keys=True,
                              separators=(',', ':'), allow_nan=False) + '\n'
            data = line.encode('utf-8')
        except (TypeError, ValueError) as exc:
            return self._fail(f'record not written (bad content): {exc}')
        with self._lock:
            if self._closed:
                return self._fail('record not written: the sink is closed')
            try:
                if self._file is None:
                    self._open()
                if self._size + len(data) > self.max_bytes:
                    return self._fail(f'record not written: {self.path.name} reached max_bytes={self.max_bytes}')
                self._file.write(data)
                self._file.flush()
                if self.fsync:
                    os.fsync(self._file.fileno())
                self._size += len(data)
                self.written += 1
                return True
            except OSError as exc:
                return self._fail(f'record not written (I/O error): {exc}')

    def publish_entity(self, entity: Any, t: Optional[datetime] = None) -> None:
        """The BaseSink call: record `entity` as an 'entity' line; FatalSinkError when write() said False."""
        if not self.write(KIND_ENTITY, entity, t):
            raise FatalSinkError(f'the recorder {self.path.name} did not store the entity (see its log)')

    def describe(self) -> str:
        """'jsonl file <name>' (the file name only, never the folder)."""
        return f'jsonl file {self.path.name}'

    def close(self) -> None:
        """Flush and close the file. Safe to call twice; later write() calls return False."""
        with self._lock:
            self._closed = True
            if self._file is not None:
                try:
                    self._file.flush()
                    self._file.close()
                except OSError as exc:
                    LOG.warning('closing %s failed: %s', self.path, exc)
                self._file = None

    def __enter__(self) -> 'JsonlSink':
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    # ---------------------------------------------------------------------------------- internals
    def _open(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = open(self.path, 'ab')
        self._size = self._file.seek(0, os.SEEK_END)

    def _fail(self, message: str) -> bool:
        """Count the failure; log the first one and then every 100th so a dead disk cannot flood the log."""
        self.failed += 1
        if self.failed == 1 or self.failed % 100 == 0:
            LOG.warning('%s (failure #%d, file %s)', message, self.failed, self.path)
        return False


# =====================================================================================================
# Reading a recording
# =====================================================================================================

def read_records(path: Union[str, Path], *, strict: bool = False) -> ReadResult:
    """Read a JSONL recording. Lines that are not a valid record are skipped and counted in `bad_lines`.

    With strict=True the first bad line raises ValueError instead. A missing file raises FileNotFoundError.
    A valid line is a JSON object with exactly the keys t (UTC text with Z), kind (short word) and payload.
    """
    records: List[Record] = []
    bad = 0
    with open(path, 'rb') as handle:
        for line_no, raw in enumerate(handle, start=1):
            if not raw.strip():
                continue
            try:
                obj = json.loads(raw.decode('utf-8'))
                if not isinstance(obj, dict) or set(obj) != {'t', 'kind', 'payload'}:
                    raise ValueError('a record needs exactly the keys t, kind and payload')
                if not isinstance(obj['kind'], str) or _KIND.fullmatch(obj['kind']) is None:
                    raise ValueError('bad kind')
                records.append(Record(parse_utc(obj['t']), obj['kind'], obj['payload'], line_no))
            except (ValueError, UnicodeError) as exc:           # json.JSONDecodeError is a ValueError
                if strict:
                    raise ValueError(f'{path}: line {line_no} is not a valid record: {exc}') from None
                bad += 1
    return ReadResult(records, bad)


def latest_by_entity_id(path: Union[str, Path]) -> Dict[str, Any]:
    """{entityId: newest entity payload} over all `entity` records (a later line replaces an earlier one).

    An entity whose newest record has isLive false is still returned (it is what was last sent); look at
    its `isLive` to see whether it was deleted.
    """
    latest: Dict[str, Any] = {}
    for record in read_records(path).records:
        if record.kind == KIND_ENTITY and isinstance(record.payload, dict):
            entity_id = record.payload.get('entityId')
            if isinstance(entity_id, str):
                latest[entity_id] = record.payload
    return latest
