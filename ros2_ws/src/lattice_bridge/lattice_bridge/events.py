"""
lattice_bridge/events.py -- PURPOSE: the four small, typed "events" that every source of the bridge (MAVLink
radio stream, ROS topics, a recorded run folder) hands to the bridge core. A source's whole job is to turn
messy outside data (bytes, ROS messages, log lines) into these events; the core then builds Lattice entities
from them. Keeping the events tiny and plain means a new source (say, a CSV file) is about fifty lines and
the core never needs to know where the data came from.

    DroneFix        the drone's position at one moment, plus whatever else is known about it (height,
                    heading, speed, flight mode, battery). One per telemetry position message.
    TargetSighting  a hiker or tent that the Jetson reported (a decoded `TGT` radio message, or in a replay
                    the position main_controller chose to aim at).
    DropEvent       a payload was released (the drone's position at that moment).
    SourceNotice    information about the source itself ("connected", "link lost", "replay finished").

The rule that makes the rest of the bridge simple: **an event object is always valid.** Every constructor
checks its numbers (finite, in range, never (0, 0)) and its times (timezone-aware) and raises ValueError
otherwise, so a source that cannot build an event simply drops that one piece of data and counts it. Nothing
downstream ever sees a NaN. Optional parts that are unknown are None; nothing is ever invented.

Times: every datetime here is timezone-aware UTC (the same rule as timeutil.py). `DroneFix.fix_time` is when
the position was measured and `DroneFix.arrival` is when this program received it; the difference is the
transport delay, and the age of `arrival` is what the bridge uses to decide that a link has gone quiet.

Pure Python (dataclasses, datetime): no ROS, no I/O, no clock reads.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Tuple, Union

from . import constants as C
from .geo import require_lat_lon, require_number
from .timeutil import require_aware

#: What a SourceNotice may say. `ended` is special: the bridge shuts down cleanly after it (a replay ran out).
NOTICE_CODES = frozenset({'connected', 'link_lost', 'reconnecting', 'silent', 'info', 'warning', 'ended'})

#: Where DroneFix.fix_time came from (kept for logs and tests): the autopilot's own clock, a ROS header
#: stamp, or simply the moment the message arrived here.
TIME_SOURCES = frozenset({'autopilot', 'ros_stamp', 'arrival', 'recorded'})


def _optional(name: str, value, low: float, high: float) -> Optional[float]:
    """None stays None; a number is checked against its range (ValueError otherwise) and returned as a float."""
    return None if value is None else require_number(name, value, low, high)


@dataclass(frozen=True)
class DroneFix:
    """The drone's position at one moment. `alt_amsl_m` and `alt_hae_m` are mutually exclusive.

    Heights: MAVLink delivers height above MEAN SEA LEVEL (`alt_amsl_m`); MAVROS' NavSatFix delivers height
    above the WGS84 ELLIPSOID (`alt_hae_m`). They are different references, so the event says which one it
    holds and the core corrects only the AMSL one (HAE = AMSL + geoid undulation). `rel_alt_m` is height
    above the take-off point (home), not above the terrain.
    """
    fix_time: datetime
    arrival: datetime
    lat_deg: float
    lon_deg: float
    alt_amsl_m: Optional[float] = None
    alt_hae_m: Optional[float] = None
    rel_alt_m: Optional[float] = None
    heading_deg: Optional[float] = None
    velocity_ned_mps: Optional[Tuple[float, float, float]] = None
    mode: Optional[str] = None
    armed: Optional[bool] = None
    battery_percent: Optional[float] = None
    time_source: str = 'arrival'

    def __post_init__(self):
        require_aware('fix_time', self.fix_time)
        require_aware('arrival', self.arrival)
        require_lat_lon(self.lat_deg, self.lon_deg, 'drone position')
        if self.alt_amsl_m is not None and self.alt_hae_m is not None:
            raise ValueError('a DroneFix holds a height above sea level OR above the ellipsoid, never both')
        _optional('alt_amsl_m', self.alt_amsl_m, C.ALT_MIN_M, C.ALT_MAX_M)
        _optional('alt_hae_m', self.alt_hae_m, C.ALT_MIN_M, C.ALT_MAX_M)
        _optional('rel_alt_m', self.rel_alt_m, C.ALT_MIN_M, C.ALT_MAX_M)
        _optional('heading_deg', self.heading_deg, 0.0, 360.0)
        _optional('battery_percent', self.battery_percent, 0.0, 100.0)
        if self.velocity_ned_mps is not None:
            try:
                north, east, down = self.velocity_ned_mps
            except (TypeError, ValueError):
                raise ValueError('velocity_ned_mps must be (north, east, down)') from None
            for label, value in (('north', north), ('east', east), ('down', down)):
                require_number(f'velocity {label}', value, -C.SPEED_MAX_MPS, C.SPEED_MAX_MPS)
        if self.armed is not None and not isinstance(self.armed, bool):
            raise ValueError('armed must be True, False or None')
        if self.mode is not None and (not isinstance(self.mode, str) or not self.mode.strip()):
            raise ValueError('mode must be a non-empty text or None')
        if self.time_source not in TIME_SOURCES:
            raise ValueError(f'time_source must be one of {sorted(TIME_SOURCES)}')


@dataclass(frozen=True)
class TargetSighting:
    """One report of a hiker or tent at a (projected) ground position.

    `confidence` is the detector's 0..1. `measured_time` is the best time we have for the measurement:
    for a radio `TGT` message that is the moment it arrived (the message carries no time of its own).
    `seq` is the radio message counter (None for sightings that did not come over the radio). `origin`
    says where it came from ('tgt' = radio message, 'aim' = a replayed main_controller decision).
    """
    kind: str
    lat_deg: float
    lon_deg: float
    confidence: float
    measured_time: datetime
    seq: Optional[int] = None
    origin: str = 'tgt'

    def __post_init__(self):
        if self.kind not in C.TARGET_KINDS:
            raise ValueError(f'kind must be one of {C.TARGET_KINDS}, got {str(self.kind)[:20]!r}')
        require_lat_lon(self.lat_deg, self.lon_deg, 'target position')
        require_number('confidence', self.confidence, 0.0, 1.0)
        require_aware('measured_time', self.measured_time)
        if self.seq is not None and (isinstance(self.seq, bool) or not isinstance(self.seq, int)
                                     or not 0 <= self.seq <= C.TGT_SEQ_MAX):
            raise ValueError(f'seq must be an integer from 0 to {C.TGT_SEQ_MAX} or None')


@dataclass(frozen=True)
class DropEvent:
    """A payload was released. The position is the DRONE's position at that moment (None = not known).

    A drop without a known position is still an event (the bridge counts and logs it) but it can never
    become a map point: nothing is guessed.
    """
    kind: str
    release_time: datetime
    lat_deg: Optional[float] = None
    lon_deg: Optional[float] = None

    def __post_init__(self):
        if self.kind not in C.TARGET_KINDS:
            raise ValueError(f'kind must be one of {C.TARGET_KINDS}, got {str(self.kind)[:20]!r}')
        require_aware('release_time', self.release_time)
        if (self.lat_deg is None) != (self.lon_deg is None):
            raise ValueError('a drop position needs both latitude and longitude, or neither')
        if self.lat_deg is not None:
            require_lat_lon(self.lat_deg, self.lon_deg, 'drop position')

    @property
    def has_position(self) -> bool:
        """True when the release position is known."""
        return self.lat_deg is not None


@dataclass(frozen=True)
class SourceNotice:
    """News about the source itself, for logs and the health line (see NOTICE_CODES)."""
    code: str
    message: str
    time: datetime

    def __post_init__(self):
        if self.code not in NOTICE_CODES:
            raise ValueError(f'notice code must be one of {sorted(NOTICE_CODES)}, got {str(self.code)[:20]!r}')
        require_aware('time', self.time)


#: Everything a source may hand over.
Event = Union[DroneFix, TargetSighting, DropEvent, SourceNotice]
