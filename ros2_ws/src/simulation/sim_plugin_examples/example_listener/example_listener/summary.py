"""
example_listener/summary.py -- PURPOSE: the bookkeeping of the TEMPLATE listener, free of ROS so it is easy to test
and to change: it is fed one message at a time (as plain values) and keeps a running summary of the flight:

    flight     modes seen, armed time, highest altitude above home, distance flown (from the GPS fixes)
    mission    the mission items reached, in order
    camera     how many shutter presses (/camera/trigger) were seen
    detections photos reported, detections per class, the best confidence per class and in which photo
    messages   the last autopilot / team text messages (STATUSTEXT)

text() is what the node prints every few seconds, as_dict() what it saves as JSON when it stops.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

EARTH_RADIUS_M = 6371000.0
CLASS_NAMES = {'0': 'person', '1': 'tent'}


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance between two GPS positions in metres (haversine formula)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


@dataclass
class FlightSummary:
    """Running summary of one flight (see the module docstring). Times are seconds (any clock)."""
    modes: List[str] = field(default_factory=list)
    connected: bool = False
    armed: bool = False
    armed_since: Optional[float] = None
    armed_total_s: float = 0.0
    max_rel_alt_m: float = 0.0
    distance_m: float = 0.0
    last_fix: Optional[Tuple[float, float]] = None
    reached: List[int] = field(default_factory=list)
    shutter_presses: int = 0
    photos_reported: int = 0
    detections: Dict[str, int] = field(default_factory=dict)
    best: Dict[str, Tuple[float, str]] = field(default_factory=dict)
    messages: Deque[str] = field(default_factory=lambda: deque(maxlen=5))

    # ---- feeding it --------------------------------------------------------------------------------------
    def on_state(self, connected: bool, armed: bool, mode: str, now: float) -> None:
        """mavros_msgs/State: connection, armed, flight mode."""
        self.connected = bool(connected)
        if mode and (not self.modes or self.modes[-1] != mode):
            self.modes.append(mode)
        if armed and not self.armed:
            self.armed_since = now
        if not armed and self.armed and self.armed_since is not None:
            self.armed_total_s += now - self.armed_since
            self.armed_since = None
        self.armed = bool(armed)

    def on_fix(self, lat: float, lon: float) -> None:
        """sensor_msgs/NavSatFix: adds the distance from the previous fix (jumps over 200 m are ignored)."""
        if not (-90 <= lat <= 90 and -180 <= lon <= 180) or (lat == 0.0 and lon == 0.0):
            return
        if self.last_fix is not None:
            step = distance_m(self.last_fix[0], self.last_fix[1], lat, lon)
            if step < 200.0:
                self.distance_m += step
        self.last_fix = (lat, lon)

    def on_rel_alt(self, metres: float) -> None:
        """std_msgs/Float64 /mavros/global_position/rel_alt: height above home."""
        if math.isfinite(metres):
            self.max_rel_alt_m = max(self.max_rel_alt_m, float(metres))

    def on_reached(self, seq: int) -> None:
        """mavros_msgs/WaypointReached: mission item number."""
        self.reached.append(int(seq))

    def on_trigger(self, pressed: bool) -> None:
        """std_msgs/Bool /camera/trigger: one shutter press when true."""
        if pressed:
            self.shutter_presses += 1

    def on_statustext(self, text: str) -> None:
        """mavros_msgs/StatusText: keep the last few lines."""
        if text:
            self.messages.append(str(text).strip())

    def on_detection(self, image_name: str, results: List[Tuple[str, float]]) -> None:
        """interfaces/ImageResult: results = [(class_id, score)] of that photo."""
        self.photos_reported += 1
        for class_id, score in results:
            name = CLASS_NAMES.get(str(class_id), 'object')
            self.detections[name] = self.detections.get(name, 0) + 1
            if score > self.best.get(name, (-1.0, ''))[0]:
                self.best[name] = (float(score), image_name)

    # ---- reading it --------------------------------------------------------------------------------------
    def armed_seconds(self, now: float) -> float:
        """Total armed time so far (including the current armed period)."""
        extra = now - self.armed_since if self.armed and self.armed_since is not None else 0.0
        return self.armed_total_s + extra

    def text(self, now: float) -> str:
        """A few readable lines for the log."""
        best = ', '.join(f'{k} {v[0]:.2f} ({v[1]})' for k, v in sorted(self.best.items())) or '-'
        dets = ', '.join(f'{k} x{v}' for k, v in sorted(self.detections.items())) or 'none'
        return '\n'.join([
            f'flight     connected={self.connected} armed={self.armed} modes={" > ".join(self.modes) or "-"} '
            f'armed {self.armed_seconds(now):.0f} s, max height {self.max_rel_alt_m:.1f} m, '
            f'flown {self.distance_m:.0f} m',
            f'mission    reached items: {self.reached[-12:] or "-"}',
            f'camera     shutter presses: {self.shutter_presses}',
            f'detections photos reported: {self.photos_reported}; found: {dets}; best: {best}',
            f'messages   {" | ".join(self.messages) or "-"}',
        ])

    def as_dict(self, now: float) -> Dict:
        """Everything as plain JSON-friendly values."""
        return {
            'connected': self.connected, 'armed': self.armed, 'modes': list(self.modes),
            'armed_seconds': round(self.armed_seconds(now), 1), 'max_rel_alt_m': round(self.max_rel_alt_m, 2),
            'distance_m': round(self.distance_m, 1), 'reached': list(self.reached),
            'shutter_presses': self.shutter_presses, 'photos_reported': self.photos_reported,
            'detections': dict(self.detections),
            'best': {k: {'score': v[0], 'image': v[1]} for k, v in self.best.items()},
            'last_messages': list(self.messages),
        }
