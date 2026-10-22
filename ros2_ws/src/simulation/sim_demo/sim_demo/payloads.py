"""
sim_demo/payloads.py -- PURPOSE: find out where each dropped payload hit the ground in Gazebo,
so the score report can say "the tent payload landed 3.2 m from the tent".

Two parts:
  * PayloadTracker (pure Python): fed (time, model name, x, y, z) samples, it notices when a
    payload starts falling and when it has come to rest, and reports the resting point.
  * GzPayloadSubscriber (optional): listens to the payloads' own Gazebo odometry topics
    /sim/payload/person/odometry and /sim/payload/tent/odometry (gz.msgs.Odometry, world
    position, from the sim_models drone model) with the gz-transport Python bindings
    (python3-gz-transport13, installed by 01_install_humble.sh) and feeds the tracker. If the
    bindings or the topics are missing, the scorer falls back to the drone's position at the
    moment the release servo opened (no wind in the simulation, so the payload falls almost
    straight down).
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field

# Names that identify payload models in Gazebo, and which target class each one is for.
DEFAULT_PAYLOAD_HINTS = (('person', 'person'), ('hiker', 'person'), ('mannequin', 'person'),
                         ('tent', 'tent'))


def payload_class(model_name: str, hints=DEFAULT_PAYLOAD_HINTS) -> str | None:
    """'person' / 'tent' for a payload model name like 'payload_tent' (None if unknown)."""
    low = model_name.lower()
    if 'payload' not in low and 'drop' not in low and 'bottle' not in low:
        return None
    for key, cls in hints:
        if key in low:
            return cls
    return None


@dataclass
class PayloadTrack:
    """What we know about one payload model.

    Attributes:
        name: Gazebo model name. cls: 'person' / 'tent' / None.
        start: first (x, y, z) seen. last: latest (t, x, y, z).
        falling_since: time it started falling (None = still attached / never fell).
        landed: (t, x, y, z) where it came to rest after falling (None = not yet).
    """

    name: str
    cls: str | None
    start: tuple[float, float, float] | None = None
    last: tuple[float, float, float, float] | None = None
    falling_since: float | None = None
    landed: tuple[float, float, float, float] | None = None
    _rest_since: float | None = None
    _z_hist: list = field(default_factory=list)


class PayloadTracker:
    """Detects falling and landing of payload models from pose samples (thread-safe)."""

    def __init__(self, fall_speed: float = 3.0, rest_speed: float = 0.15, rest_time: float = 1.0):
        """
        Args:
            fall_speed: downward speed [m/s] that means "it is falling" (3 m/s: faster than the
                drone ever descends in AUTO/RTL, reached 0.3 s into a free fall).
            rest_speed: speed [m/s] below which it counts as resting.
            rest_time: it must rest this long [s] after falling to count as landed.
        """
        self.fall_speed = fall_speed
        self.rest_speed = rest_speed
        self.rest_time = rest_time
        self.tracks: dict[str, PayloadTrack] = {}
        self._lock = threading.Lock()

    def add(self, t: float, name: str, x: float, y: float, z: float) -> None:
        """Feed one pose sample of a model (non-payload names are ignored)."""
        cls = payload_class(name)
        if cls is None and 'payload' not in name.lower():
            return
        with self._lock:
            tr = self.tracks.get(name)
            if tr is None:
                tr = self.tracks[name] = PayloadTrack(name=name, cls=cls, start=(x, y, z))
            if tr.last is not None:
                dt = t - tr.last[0]
                if dt > 1e-3:
                    vz = (z - tr.last[3]) / dt
                    speed = math.dist((x, y, z), tr.last[1:]) / dt
                    if tr.falling_since is None and vz < -self.fall_speed:
                        tr.falling_since = t
                    if tr.falling_since is not None and tr.landed is None:
                        if speed < self.rest_speed:
                            if tr._rest_since is None:
                                tr._rest_since = t
                            elif t - tr._rest_since >= self.rest_time:
                                tr.landed = (t, x, y, z)
                        else:
                            tr._rest_since = None
            tr.last = (t, x, y, z)

    def landing_points(self) -> dict[str, tuple[float, float, float, float]]:
        """{model name: (t, x, y, z)} for every payload that has landed."""
        with self._lock:
            return {n: tr.landed for n, tr in self.tracks.items() if tr.landed is not None}

    def snapshot(self) -> list[PayloadTrack]:
        """Copies of all tracks (for reports and markers)."""
        with self._lock:
            return [PayloadTrack(tr.name, tr.cls, tr.start, tr.last, tr.falling_since, tr.landed)
                    for tr in self.tracks.values()]


# The drone model (sim_models iris_a8mini) gives each payload an OdometryPublisher that sends
# its WORLD position at 10 Hz on these Gazebo topics (gz.msgs.Odometry).
DEFAULT_PAYLOAD_TOPICS = ('/sim/payload/person/odometry', '/sim/payload/tent/odometry')


class GzPayloadSubscriber:
    """Feeds a PayloadTracker from the payloads' Gazebo odometry topics (optional).

    Uses the gz-transport Python bindings directly (no ROS bridge needed). The environment
    variables GZ_PARTITION / GZ_IP must match the simulator's (the launch files pass them on).

    Usage:
        sub = GzPayloadSubscriber(tracker, log=print)
        ok = sub.start()     # False (with a logged reason) if gz-transport is unavailable
    """

    def __init__(self, tracker: PayloadTracker, topics=DEFAULT_PAYLOAD_TOPICS, log=print,
                 clock=None):
        """
        Args:
            tracker: the PayloadTracker to feed.
            topics: Gazebo odometry topics, one per payload; the model name is taken from the
                topic ('/sim/payload/tent/odometry' -> 'payload_tent').
            log: function used for one-line messages.
            clock: function returning the time in seconds (default time.time).
        """
        import time as _time
        self.tracker = tracker
        self.topics = [t for t in topics if t]
        self.log = log
        self.clock = clock or _time.time
        self._node = None
        self.samples = 0

    @staticmethod
    def model_name_for(topic: str) -> str:
        """'/sim/payload/tent/odometry' -> 'payload_tent'."""
        parts = [p for p in topic.split('/') if p]
        if len(parts) >= 3 and parts[-3] == 'payload':
            return f'payload_{parts[-2]}'
        return '_'.join(parts[:-1]) or topic

    def start(self) -> bool:
        """Subscribe. Returns False if the gz Python bindings are missing."""
        try:
            from gz.msgs10.odometry_pb2 import Odometry
            from gz.transport13 import Node
        except Exception as exc:  # noqa: BLE001
            self.log(f'Gazebo payload tracking off (gz-transport Python bindings missing: {exc})')
            return False
        try:
            self._node = Node()
            ok_any = False
            for topic in self.topics:
                name = self.model_name_for(topic)

                def handler(msg, _name=name):
                    self.samples += 1
                    p = msg.pose.position
                    self.tracker.add(self.clock(), _name, p.x, p.y, p.z)

                if self._node.subscribe(Odometry, topic, handler):
                    ok_any = True
                else:
                    self.log(f'Gazebo payload tracking: could not subscribe to {topic}')
            if ok_any:
                self.log('Gazebo payload tracking on ' + ', '.join(self.topics))
            return ok_any
        except Exception as exc:  # noqa: BLE001 - never take the node down for this
            self.log(f'Gazebo payload tracking failed: {exc}')
            return False
