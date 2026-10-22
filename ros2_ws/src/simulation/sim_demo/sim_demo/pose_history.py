"""
sim_demo/pose_history.py -- PURPOSE: remember where the drone was over the last minutes, so
that when a photo file shows up (seconds after the shutter fired) we can find out where the
drone was, how high it flew and which way it pointed AT THE MOMENT THE PHOTO WAS TAKEN.

WHY: the photo file itself only carries the drone's latitude/longitude at shutter time (in
its name, see photos.py). To know which patch of ground the photo shows we also need the
height above the ground and the heading at that moment. The camera node chose the file name
from the same /mavros/global_position/global messages we record here, so the recorded sample
whose position equals the name's position IS the shutter moment.

All times are wall-clock seconds (time.time()), the same clock as file modification times.
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, replace

from .geo import GeoOrigin, local_distance_m, wrap_deg_360

NAN = float('nan')


@dataclass(frozen=True)
class PoseSample:
    """The drone's state at one moment.

    Attributes:
        t: wall-clock time [s].
        lat, lon: GPS [deg].
        alt_msl: altitude above sea level [m] (NaN if unknown).
        rel_alt: height above home [m] (NaN if unknown) - on flat ground = height above ground.
        heading_deg: compass heading of the nose [deg, 0 = north, clockwise] (NaN if unknown).
        ground_speed: [m/s] (NaN if unknown).
    """

    t: float
    lat: float
    lon: float
    alt_msl: float = NAN
    rel_alt: float = NAN
    heading_deg: float = NAN
    ground_speed: float = NAN


def _lerp(a: float, b: float, f: float) -> float:
    if math.isnan(a):
        return b
    if math.isnan(b):
        return a
    return a + (b - a) * f


def _lerp_angle(a: float, b: float, f: float) -> float:
    if math.isnan(a):
        return b
    if math.isnan(b):
        return a
    d = ((b - a + 540.0) % 360.0) - 180.0
    return wrap_deg_360(a + d * f)


class PoseHistory:
    """A time-ordered list of PoseSamples with lookups by time and by GPS position."""

    def __init__(self, max_age_s: float = 1800.0, max_samples: int = 50000):
        """
        Args:
            max_age_s: samples older than this (relative to the newest) are dropped.
            max_samples: hard cap on memory use.
        """
        self.max_age_s = max_age_s
        self.max_samples = max_samples
        self._t: list[float] = []
        self._s: list[PoseSample] = []

    def __len__(self) -> int:
        return len(self._s)

    def add(self, sample: PoseSample) -> None:
        """Append a sample (samples arriving out of order are inserted in place)."""
        if self._t and sample.t < self._t[-1]:
            i = bisect.bisect_right(self._t, sample.t)
            self._t.insert(i, sample.t)
            self._s.insert(i, sample)
        else:
            self._t.append(sample.t)
            self._s.append(sample)
        if len(self._s) > self.max_samples or (self._t[-1] - self._t[0]) > self.max_age_s * 1.2:
            cutoff = self._t[-1] - self.max_age_s
            k = max(bisect.bisect_left(self._t, cutoff), len(self._s) - self.max_samples)
            del self._t[:k]
            del self._s[:k]

    def latest(self) -> PoseSample | None:
        """The newest sample, or None if empty."""
        return self._s[-1] if self._s else None

    def at_time(self, t: float, max_gap_s: float = 5.0) -> PoseSample | None:
        """The state at time t, interpolated between the two neighbouring samples.

        Returns None if the history is empty or t is more than max_gap_s away from any sample.
        """
        if not self._s:
            return None
        i = bisect.bisect_left(self._t, t)
        if i <= 0:
            s = self._s[0]
            return s if abs(s.t - t) <= max_gap_s else None
        if i >= len(self._s):
            s = self._s[-1]
            return s if abs(s.t - t) <= max_gap_s else None
        a, b = self._s[i - 1], self._s[i]
        span = b.t - a.t
        f = 0.0 if span <= 0 else (t - a.t) / span
        return PoseSample(
            t=t, lat=_lerp(a.lat, b.lat, f), lon=_lerp(a.lon, b.lon, f),
            alt_msl=_lerp(a.alt_msl, b.alt_msl, f), rel_alt=_lerp(a.rel_alt, b.rel_alt, f),
            heading_deg=_lerp_angle(a.heading_deg, b.heading_deg, f),
            ground_speed=_lerp(a.ground_speed, b.ground_speed, f))

    def nearest_to(self, origin: GeoOrigin, lat: float, lon: float, t_min: float,
                   t_max: float) -> tuple[PoseSample | None, float]:
        """The sample between t_min and t_max closest to (lat, lon).

        Returns:
            (sample, distance_m), or (None, inf) if no sample is in the time window.
        """
        i0 = bisect.bisect_left(self._t, t_min)
        i1 = bisect.bisect_right(self._t, t_max)
        best, best_d = None, math.inf
        for s in self._s[i0:i1]:
            d = local_distance_m(origin, s.lat, s.lon, lat, lon)
            if d < best_d:
                best, best_d = s, d
        return best, best_d

    def capture_pose(self, origin: GeoOrigin, lat: float | None, lon: float | None,
                     file_time: float, window_s: float = 60.0, latency_s: float = 1.0,
                     max_match_m: float = 3.0,
                     tie_tolerance_m: float = 0.3) -> tuple[PoseSample | None, str]:
        """Best guess of the drone state when a photo was taken.

        Args:
            origin: geo origin (for distances).
            lat, lon: GPS from the photo's file name (None if the name has none).
            file_time: when the photo file appeared (its modification time) [s].
            window_s: how far back to search before file_time.
            latency_s: typical shutter-to-file delay; breaks ties between equally near samples
                and is used alone when there is no GPS in the name.
            max_match_m: the nearest recorded position must be within this distance of the
                name's GPS to count as the shutter moment.
            tie_tolerance_m: samples this much farther than the nearest one still count as
                equally near (the name's 6 decimals round positions by ~0.1 m).
        Returns:
            (sample, how): how = 'gps-match', 'time' or 'none'. For 'gps-match' the sample's
            lat/lon are replaced by the file name's (they are what the detector reports).
        """
        if not self._s:
            return None, 'none'
        expected_t = file_time - latency_s
        if lat is not None and lon is not None:
            # The recorded sample nearest to the name's position is the shutter moment. When
            # several are (almost) equally near - the drone hovered, or passed the same spot
            # twice - take the one closest to the usual shutter-to-file delay.
            i0 = bisect.bisect_left(self._t, file_time - window_s)
            i1 = bisect.bisect_right(self._t, file_time + 1.0)
            dists = [(local_distance_m(origin, s.lat, s.lon, lat, lon), s) for s in self._s[i0:i1]]
            if dists:
                d_min = min(d for d, _ in dists)
                if d_min <= max_match_m:
                    tie = d_min + tie_tolerance_m
                    best = min((s for d, s in dists if d <= tie), key=lambda s: abs(s.t - expected_t))
                    return replace(best, lat=lat, lon=lon), 'gps-match'
        s = self.at_time(expected_t, max_gap_s=10.0)
        if s is None:
            return None, 'none'
        return s, 'time'

    def samples(self, t_min: float = -math.inf, t_max: float = math.inf) -> list[PoseSample]:
        """Copy of the samples between t_min and t_max (oldest first)."""
        i0 = bisect.bisect_left(self._t, t_min)
        i1 = bisect.bisect_right(self._t, t_max)
        return list(self._s[i0:i1])
