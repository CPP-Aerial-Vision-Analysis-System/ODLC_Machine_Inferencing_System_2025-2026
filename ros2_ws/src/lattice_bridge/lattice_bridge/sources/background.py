"""
lattice_bridge/sources/background.py -- PURPOSE: let a source that BLOCKS while it waits for data (the MAVLink
socket) do its reading in its own thread, so a slow main loop can never make the radio stream back up.

Why this exists: the bridge's main loop does the "slow" jobs (build entities, hand them to the sink, which may
talk to the internet). If it were also the one reading the MAVLink socket, ten seconds of trouble in the sink
would be ten seconds in which no telemetry or target report is read, and the operating system's small socket
buffer would start throwing messages away; a `TGT` report lost that way never reaches Lattice. With a reader
thread the socket is always emptied, the events wait in a bounded EventQueue, and only the DroneFix events
(which are worthless once a newer position exists) are ever replaced.

BackgroundSource wraps any Source. start() starts the inner source and the thread; poll() waits on the queue;
close() stops the thread (waits for it, bounded) and closes the inner source. It is a Source itself, so the
bridge does not know the difference. A crash inside the inner poll() is caught, logged (rate-limited) and
retried after a pause: the reader thread never dies.
"""

from __future__ import annotations

import logging
import threading
import time
from datetime import datetime
from typing import Dict, List, Optional

from ..events import Event
from .common import EventQueue

LOG = logging.getLogger('lattice_bridge.background')


class BackgroundSource:
    """Run `inner.poll()` in a daemon thread and queue what it returns (see the module docstring)."""

    def __init__(self, inner, *, queue_size: int = 2000, poll_s: float = 0.2, error_pause_s: float = 1.0):
        self.inner = inner
        self.queue = EventQueue(queue_size)
        self.poll_s = float(poll_s)
        self.error_pause_s = float(error_pause_s)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._errors = 0
        self._last_error_log = -1e9

    @property
    def name(self) -> str:
        """The inner source's name."""
        return self.inner.name

    @property
    def altitude_reference(self) -> str:
        """The inner source's height reference ('amsl', 'hae' or 'none')."""
        return self.inner.altitude_reference

    def start(self) -> None:
        """Start the inner source (it may raise SourceUnavailable) and then the reader thread."""
        self.inner.start()
        self._thread = threading.Thread(target=self._run, name='lattice-bridge-reader', daemon=True)
        self._thread.start()

    def poll(self, timeout_s: float) -> List[Event]:
        """Return the queued events, waiting up to `timeout_s` for the first one."""
        return self.queue.get_many(500, timeout_s)

    def now(self) -> datetime:
        """The inner source's time."""
        return self.inner.now()

    def describe(self) -> str:
        """The inner source's description."""
        return self.inner.describe()

    def stats(self) -> Dict[str, int]:
        """The inner counters plus the queue's."""
        merged = dict(self.inner.stats())
        merged.update({'queue_dropped_fixes': self.queue.dropped_fixes, 'queue_dropped_other': self.queue.dropped_other,
                       'reader_errors': self._errors})
        return merged

    def close(self) -> None:
        """Stop the thread (waiting at most a few seconds) and close the inner source. Safe to call twice."""
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=3.0)
        self.inner.close()

    # ------------------------------------------------------------------------------------------ the thread
    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                events = self.inner.poll(self.poll_s)
            except Exception as exc:                        # the reader must outlive any bug in the inner source
                self._errors += 1
                now = time.monotonic()
                if now - self._last_error_log >= 30.0:
                    LOG.error('the %s source failed while reading (%s: %s); retrying', self.inner.name,
                              exc.__class__.__name__, exc)
                    self._last_error_log = now
                self._stop.wait(self.error_pause_s)
                continue
            for event in events:
                self.queue.put(event)
