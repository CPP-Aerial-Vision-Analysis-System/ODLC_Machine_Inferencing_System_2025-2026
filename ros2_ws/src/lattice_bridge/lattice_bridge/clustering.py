"""
lattice_bridge/clustering.py -- PURPOSE: decide which detections are the SAME physical target. The drone takes
many photos, so one hiker is detected in several of them, each time with a slightly different projected
position and confidence. Lattice must show ONE track entity per real target, with a stable id, not one new
entity per photo. TargetClusterer takes the detections one at a time and answers, for each one, with a
Decision: "create" a new entity, "update" the existing one, or "no_change" (nothing worth publishing).

The decisions, written down on purpose (tests check every line):
  * Same kind only: a person is never merged with a tent.
  * Same target = within `merge_radius_m` (default 12 m, see constants.CLUSTER_MERGE_RADIUS_M) of the
    target's current position. Nearest cluster wins when several are in range.
  * Best confidence wins: the reported confidence of a target is the HIGHEST confidence ever seen for it.
  * Position = the confidence-weighted mean of all sightings, weight = confidence squared. Why not simply
    "the position of the best photo"? One photo's projection error (GPS age, heading, height) is random, so
    the mean of several photos is closer to the truth, and squaring the weight lets a 0.9 sighting count
    81 / 9 = 9 times as much as a 0.3 sighting, so a poor sighting cannot drag the estimate far. Systematic
    errors (a wrong camera mounting angle) are not averaged away by any method; that is for calibration.
  * Clusters that grow into each other merge (the older id survives, the other is reported as `retired`, and
    the caller publishes an isLive=false update for it). So the end result does not depend on the order in
    which the photos arrived, except that ids follow the order in which targets were first seen.
  * Quiet by default: after the first publish, an update is only requested when the estimate moved at least
    `update_min_move_m` (1 m) from the LAST PUBLISHED position, or the best confidence grew by at least
    `update_min_conf_gain` (0.02), or two clusters merged. Everything else is "no_change", so a hiker seen
    in 20 photos does not cause 20 updates.
  * Bounded memory: each cluster stores a few running sums, never its sightings, and at most `max_clusters`
    clusters exist; a sighting that would start another one is ignored with a reason.
  * Bad data never raises: unknown kind, NaN, out-of-range or (0, 0) position, confidence outside 0..1 or
    below `min_confidence` give an "ignored" decision with a reason. A NAIVE measured_time is a programming
    error and raises ValueError (like every other function that takes a time).

sourceUpdateTime: a decision carries `last_seen`, the newest measurement time of the target; that is what
to pass as `measured_time` when building the track. `time_advanced` says whether it is later than the one
published before. If a late, older photo improves a target, `time_advanced` is False and Lattice might
discard the update (it wants sourceUpdateTime to change); the bridge decides what to do about that.

Pure Python: no ROS, no I/O, no clock reads (the measurement time is passed in with each detection).
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

from . import constants as C
from .geo import haversine_m, is_real_number, require_lat_lon, wrap_lon
from .ids import check_mission_id, target_display_name, target_entity_id
from .timeutil import require_aware

ACTION_CREATE = 'create'
ACTION_UPDATE = 'update'
ACTION_NO_CHANGE = 'no_change'
ACTION_IGNORED = 'ignored'

#: A sighting never weighs less than this (squared), so a confidence of 0.0 cannot make the mean undefined.
_MIN_WEIGHT_CONFIDENCE = 0.05


@dataclass(frozen=True)
class TargetState:
    """A snapshot of one target cluster (what the clusterer currently believes)."""
    entity_id: str
    kind: str
    number: int
    lat_deg: float
    lon_deg: float
    confidence: float          # the best confidence seen
    sightings: int
    first_seen: datetime
    last_seen: datetime


@dataclass(frozen=True)
class Decision:
    """The answer for one detection. Publish only when `should_publish` is True."""
    action: str
    reason: str = ''
    state: Optional[TargetState] = None
    time_advanced: bool = False
    retired: Tuple[TargetState, ...] = ()

    @property
    def should_publish(self) -> bool:
        """True for 'create' and 'update'."""
        return self.action in (ACTION_CREATE, ACTION_UPDATE)

    @property
    def entity_id(self) -> Optional[str]:
        """The id of the target this decision is about (None when ignored)."""
        return None if self.state is None else self.state.entity_id

    @property
    def retired_ids(self) -> Tuple[str, ...]:
        """Ids of targets that were merged into this one; publish isLive=false for each."""
        return tuple(s.entity_id for s in self.retired)

    def track_kwargs(self) -> Dict[str, Any]:
        """Keyword arguments for entities.build_target_track (add `now` and `expiry_time` yourself).

        For the goodbye update of a retired target use `state_track_kwargs(state, measured_time=...)`.
        """
        if self.state is None:
            raise ValueError('an ignored decision has no target to build')
        return state_track_kwargs(self.state)


def state_track_kwargs(state: TargetState, measured_time: Optional[datetime] = None) -> Dict[str, Any]:
    """Keyword arguments for entities.build_target_track from a TargetState (add `now`, `expiry_time`).

    `measured_time` overrides the time written as sourceUpdateTime. Use it for a goodbye update
    (`is_live=False`) of a target that was merged away: Lattice only accepts an update whose
    sourceUpdateTime CHANGED, and the retired target's own last_seen is the time of its last publish, so
    pass the time of the sighting that caused the merge (the new `Decision.state.last_seen`).
    """
    when = state.last_seen if measured_time is None else require_aware('measured_time', measured_time)
    return {
        'entity_id': state.entity_id,
        'kind': state.kind,
        'lat_deg': state.lat_deg,
        'lon_deg': state.lon_deg,
        'confidence': state.confidence,
        'measured_time': when,
        'first_seen_time': state.first_seen,
        'sightings': state.sightings,
        'name': target_display_name(state.kind, state.number),
    }


def _lon_delta(lon: float, ref_lon: float) -> float:
    """Longitude difference lon - ref_lon folded into [-180, 180) so the antimeridian is harmless."""
    return ((lon - ref_lon + 180.0) % 360.0) - 180.0


class _Cluster:
    """Running sums of one target. Positions are stored as offsets from the first sighting (the reference)."""

    __slots__ = ('kind', 'number', 'entity_id', 'ref_lat', 'ref_lon', 'w_sum', 'w_dlat', 'w_dlon',
                 'best_conf', 'sightings', 'first_seen', 'last_seen',
                 'pub_lat', 'pub_lon', 'pub_conf', 'pub_last_seen')

    def __init__(self, mission_id: str, kind: str, number: int, lat: float, lon: float,
                 conf: float, when: datetime):
        self.kind = kind
        self.number = number
        self.entity_id = target_entity_id(mission_id, kind, number)
        self.ref_lat, self.ref_lon = lat, lon
        self.w_sum = self.w_dlat = self.w_dlon = 0.0
        self.best_conf = conf
        self.sightings = 0
        self.first_seen = self.last_seen = when
        self.pub_lat = self.pub_lon = self.pub_conf = 0.0
        self.pub_last_seen = when
        self.add(lat, lon, conf, when)

    def position(self) -> Tuple[float, float]:
        """The confidence-weighted mean position."""
        return (self.ref_lat + self.w_dlat / self.w_sum,
                wrap_lon(self.ref_lon + self.w_dlon / self.w_sum))

    def add(self, lat: float, lon: float, conf: float, when: datetime) -> None:
        """Take one more sighting into the running sums."""
        weight = max(conf, _MIN_WEIGHT_CONFIDENCE) ** 2
        self.w_sum += weight
        self.w_dlat += weight * (lat - self.ref_lat)
        self.w_dlon += weight * _lon_delta(lon, self.ref_lon)
        self.sightings += 1
        self.best_conf = max(self.best_conf, conf)
        self.first_seen = min(self.first_seen, when)
        self.last_seen = max(self.last_seen, when)

    def absorb(self, other: '_Cluster') -> None:
        """Merge another cluster of the same kind into this one (the sums are re-based on our reference)."""
        shift_lat = other.ref_lat - self.ref_lat
        shift_lon = _lon_delta(other.ref_lon, self.ref_lon)
        self.w_sum += other.w_sum
        self.w_dlat += other.w_dlat + other.w_sum * shift_lat
        self.w_dlon += other.w_dlon + other.w_sum * shift_lon
        self.sightings += other.sightings
        self.best_conf = max(self.best_conf, other.best_conf)
        self.first_seen = min(self.first_seen, other.first_seen)
        self.last_seen = max(self.last_seen, other.last_seen)

    def state(self) -> TargetState:
        """A frozen snapshot of this cluster."""
        lat, lon = self.position()
        return TargetState(self.entity_id, self.kind, self.number, lat, lon, self.best_conf,
                           self.sightings, self.first_seen, self.last_seen)

    def mark_published(self) -> None:
        """Remember what Lattice has been told, so later updates are measured against it."""
        self.pub_lat, self.pub_lon = self.position()
        self.pub_conf = self.best_conf
        self.pub_last_seen = self.last_seen


class TargetClusterer:
    """Feed detections in, get create / update / no_change decisions with stable ids out.

    One instance per mission (the mission id is part of every entity id). Not thread safe: call it from one
    thread (the bridge's worker), never from a ROS callback that must not block (it is fast, but it is not
    the place for it).
    """

    def __init__(
        self,
        mission_id: str,
        *,
        merge_radius_m: float = C.CLUSTER_MERGE_RADIUS_M,
        update_min_move_m: float = C.CLUSTER_UPDATE_MIN_MOVE_M,
        update_min_conf_gain: float = C.CLUSTER_UPDATE_MIN_CONF_GAIN,
        min_confidence: float = 0.0,
        max_clusters: int = C.CLUSTER_MAX_CLUSTERS,
    ):
        self.mission_id = check_mission_id(mission_id)
        for name, value in (('merge_radius_m', merge_radius_m), ('update_min_move_m', update_min_move_m),
                            ('update_min_conf_gain', update_min_conf_gain), ('min_confidence', min_confidence)):
            if not is_real_number(value) or value < 0:
                raise ValueError(f'{name} must be a number >= 0, got {str(value)[:20]!r}')
        if merge_radius_m <= 0:
            raise ValueError('merge_radius_m must be greater than 0')
        if isinstance(max_clusters, bool) or not isinstance(max_clusters, int) or max_clusters < 1:
            raise ValueError('max_clusters must be an integer >= 1')
        self.merge_radius_m = float(merge_radius_m)
        self.update_min_move_m = float(update_min_move_m)
        self.update_min_conf_gain = float(update_min_conf_gain)
        self.min_confidence = float(min_confidence)
        self.max_clusters = max_clusters
        self._clusters: Dict[str, _Cluster] = {}
        self._counters = {kind: 0 for kind in C.TARGET_KINDS}

    # ---------------------------------------------------------------------------------- reading
    def __len__(self) -> int:
        return len(self._clusters)

    def targets(self) -> List[TargetState]:
        """Snapshots of all current targets, ordered by kind then number."""
        return [c.state() for c in sorted(self._clusters.values(), key=lambda c: (c.kind, c.number))]

    def get(self, entity_id: str) -> Optional[TargetState]:
        """The snapshot of one target, or None (never created, or merged away)."""
        cluster = self._clusters.get(entity_id)
        return None if cluster is None else cluster.state()

    # ---------------------------------------------------------------------------------- feeding
    def feed(self, kind, lat_deg, lon_deg, confidence, measured_time: datetime) -> Decision:
        """Take one detection and return what to do about it (see the module doc for the rules)."""
        when = require_aware('measured_time', measured_time)       # a programming error: raises
        if kind not in C.TARGET_KINDS:
            return Decision(ACTION_IGNORED, f'unknown kind {str(kind)[:20]!r}')
        try:
            lat, lon = require_lat_lon(lat_deg, lon_deg, 'detection')
        except ValueError as exc:
            return Decision(ACTION_IGNORED, f'bad position: {exc}')
        if not is_real_number(confidence) or not 0.0 <= confidence <= 1.0:
            return Decision(ACTION_IGNORED, f'confidence must be a number from 0 to 1, got {str(confidence)[:20]!r}')
        conf = float(confidence)
        if conf < self.min_confidence:
            return Decision(ACTION_IGNORED, f'confidence {conf:.2f} is below the minimum {self.min_confidence:.2f}')

        cluster = self._nearest(kind, lat, lon)
        if cluster is None:
            if len(self._clusters) >= self.max_clusters:
                return Decision(ACTION_IGNORED, f'cluster limit ({self.max_clusters}) reached')
            self._counters[kind] += 1
            cluster = _Cluster(self.mission_id, kind, self._counters[kind], lat, lon, conf, when)
            self._clusters[cluster.entity_id] = cluster
            cluster.mark_published()
            return Decision(ACTION_CREATE, 'new target', cluster.state(), True)

        cluster.add(lat, lon, conf, when)
        survivor, merged = self._merge_neighbours(cluster)
        return self._decide_update(survivor, tuple(merged))

    # ---------------------------------------------------------------------------------- internals
    def _nearest(self, kind: str, lat: float, lon: float) -> Optional[_Cluster]:
        """The same-kind cluster closest to the point and within the merge radius, or None."""
        best: Optional[_Cluster] = None
        best_distance = 0.0
        for cluster in self._clusters.values():
            if cluster.kind != kind:
                continue
            c_lat, c_lon = cluster.position()
            distance = haversine_m(lat, lon, c_lat, c_lon)
            if distance > self.merge_radius_m:
                continue
            if best is None or distance < best_distance or (distance == best_distance
                                                            and cluster.number < best.number):
                best, best_distance = cluster, distance
        return best

    def _merge_neighbours(self, cluster: _Cluster) -> Tuple[_Cluster, List[TargetState]]:
        """Merge every same-kind cluster that has come within the radius of `cluster`; the older id survives.

        Returns (surviving cluster, snapshots of the clusters that disappeared).
        """
        retired: List[TargetState] = []
        merged_any = True
        while merged_any:
            merged_any = False
            c_lat, c_lon = cluster.position()
            for other in sorted(self._clusters.values(), key=lambda c: c.number):
                if other is cluster or other.kind != cluster.kind:
                    continue
                o_lat, o_lon = other.position()
                if haversine_m(c_lat, c_lon, o_lat, o_lon) <= self.merge_radius_m:
                    survivor, loser = (cluster, other) if cluster.number < other.number else (other, cluster)
                    retired.append(loser.state())
                    survivor.absorb(loser)
                    del self._clusters[loser.entity_id]
                    cluster = survivor
                    merged_any = True
                    break
        return cluster, retired

    def _decide_update(self, cluster: _Cluster, merged: Tuple[TargetState, ...]) -> Decision:
        lat, lon = cluster.position()
        moved = haversine_m(cluster.pub_lat, cluster.pub_lon, lat, lon)
        gain = cluster.best_conf - cluster.pub_conf
        if not merged and moved < self.update_min_move_m and gain < self.update_min_conf_gain:
            return Decision(ACTION_NO_CHANGE, f'moved {moved:.2f} m, confidence +{gain:.2f}: below the thresholds',
                            cluster.state())
        advanced = cluster.last_seen > cluster.pub_last_seen
        cluster.mark_published()
        reason = 'targets merged' if merged else f'moved {moved:.2f} m, confidence +{gain:.2f}'
        return Decision(ACTION_UPDATE, reason, cluster.state(), advanced, merged)
