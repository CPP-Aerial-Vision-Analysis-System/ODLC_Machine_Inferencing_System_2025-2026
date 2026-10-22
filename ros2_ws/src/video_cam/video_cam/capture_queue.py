"""
A small waiting line for photo requests, so no /camera/trigger is thrown away.

video_cam/capture_queue.py -- PURPOSE: keep a /camera/trigger that arrives while the camera is
still busy with the previous photo, and serve it a moment later instead of throwing it away.

Why (October 2026, "dropped photos"): siyi_node remembered at most one pending trigger (a
threading.Event) and threw away every trigger that came while the lens worker was zooming or
focusing. In the simulator 49 of 83 triggers were lost that way: the zoom/focus set-up after
DigiCamCtrl took ~15 s and the 5 s autofocus loop held the lens ~3 s of every 5 s. Now:
  * every trigger goes into this queue (oldest first);
  * siyi_node serves the oldest one as soon as the camera can take a photo (photos now have
    priority over lens housekeeping, which no longer holds the camera for seconds);
  * the queue is bounded (max_pending): if the camera really cannot keep up, the OLDEST request
    is dropped (the least useful one: the drone has already flown past that spot) and counted,
    so the log says exactly how many photos were lost and why.

Pure Python (threading + collections only): no ROS. Unit tests: ../test/test_capture_queue.py
"""

from collections import deque
import threading
from typing import Optional


class CaptureRequestQueue:
    """Bounded first-in-first-out queue of photo requests (their arrival times), thread-safe."""

    def __init__(self, max_pending: int = 5):
        """
        Set up an empty queue.

        Parameters
        ----------
        max_pending
            Most requests that may wait at once; one more drops the oldest.

        """
        self.max_pending = max(1, int(max_pending))
        self._items = deque()
        self._lock = threading.Lock()
        self.received = 0            # every request ever pushed
        self.served = 0              # requests handed out by pop()
        self.dropped = 0             # requests thrown away because the queue was full
        self.discarded = 0           # requests cleared on purpose (camera disabled, shutdown)
        self.max_depth = 0           # the longest the line has ever been

    def push(self, t: float) -> bool:
        """
        Add a request that arrived at time t (the new one is always kept).

        Returns
        -------
        bool
            True if it fitted; False if the queue was full and the OLDEST request was dropped
            to make room.

        """
        with self._lock:
            self.received += 1
            fitted = True
            if len(self._items) >= self.max_pending:
                self._items.popleft()
                self.dropped += 1
                fitted = False
            self._items.append(float(t))
            self.max_depth = max(self.max_depth, len(self._items))
            return fitted

    def pop(self) -> Optional[float]:
        """Take the oldest waiting request (its arrival time), or None if nobody is waiting."""
        with self._lock:
            if not self._items:
                return None
            self.served += 1
            return self._items.popleft()

    def clear(self) -> int:
        """Throw away every waiting request on purpose and return how many there were."""
        with self._lock:
            n = len(self._items)
            self._items.clear()
            self.discarded += n
            return n

    def __len__(self) -> int:
        """Return how many requests are waiting."""
        with self._lock:
            return len(self._items)

    @property
    def pending(self) -> int:
        """Count the requests waiting right now."""
        return len(self)

    def stats_text(self) -> str:
        """Return one log line, e.g. '83 triggers: 83 served, 0 dropped (queue full), ...'."""
        with self._lock:
            return (f'{self.received} triggers: {self.served} served, {self.dropped} dropped '
                    f'(queue full), {self.discarded} discarded, {len(self._items)} waiting '
                    f'(longest wait line {self.max_depth})')
