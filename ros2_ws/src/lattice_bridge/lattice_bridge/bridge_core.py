"""
lattice_bridge/bridge_core.py -- PURPOSE: the brain of the bridge, with no sockets, no ROS and no clock of its
own. It takes events from a source (a position of the drone, a target report, a release, news about the link),
decides what Lattice should be told, builds the entities with the phase-1 builders, checks each one against the
contract, and hands it to an "output" (a publisher in front of a sink). Because time is always passed in
(`now`), the whole thing is tested by feeding it a script of events and reading what comes out.

What it does, in plain words:

  * THE DRONE: keeps the newest position. About once a second (`drone_interval_s`) it publishes the drone, but
    only if a NEWER position arrived (Lattice ignores an update whose sourceUpdateTime did not change). The
    entity carries height (corrected from sea level to ellipsoid height with the configured geoid undulation),
    height above home, heading, speed, flight mode, battery, and how old the last telemetry is.
  * A SILENT LINK: when no position has arrived for `stale_after_s` seconds the drone is STALE. The bridge then
    stops refreshing it (it never repeats the old position as if it were new), so the entity fades out of
    Lattice by itself when its 2-minute expiry runs out (Lattice also shows it offline after a minute). With
    stale_policy "not_live" it additionally sends one isLive=false update after `offline_after_s` and puts the
    drone back on the map when telemetry returns. Nothing is ever invented.
  * TARGETS: every target report goes to the clusterer. One physical hiker or tent stays ONE entity with a
    stable id however many reports it gets; two real targets are two entities; a report that bridges two
    clusters merges them (the older id survives, the other is deleted with an isLive=false update).
  * AREAS: the search areas and the flight boundary are published once, at start.
  * DROP POINTS: a release with a known position becomes a map point; one without is counted and skipped.
  * SHUTDOWN: the drone gets a final isLive=false update, then the output is flushed and closed. After that
    nothing more is published.

Every update's sourceUpdateTime is strictly newer than the previous one for the same entity (Lattice's rule).
Every entity is checked with find_problems() before it leaves; an invalid one is counted and dropped, never sent.
Nothing here raises to the caller for bad data: a bad event is logged and counted.
"""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from typing import Any, Dict, List, Optional, Protocol, Sequence

from . import constants as C
from .clustering import ACTION_CREATE, ACTION_IGNORED, ACTION_NO_CHANGE, ACTION_UPDATE, TargetClusterer, state_track_kwargs
from .entities import build_area_geo, build_drop_point, build_target_track, build_uav_asset, find_problems
from .events import DroneFix, DropEvent, Event, SourceNotice, TargetSighting
from .ids import area_entity_id, check_mission_id, drop_entity_id, uav_entity_id
from .logutil import warn_if_altitude_approximate
from .timeutil import require_aware
from .world_areas import AreaSpec

LOG = logging.getLogger('lattice_bridge.bridge')

STALE_POLICIES = ('stop', 'not_live')
_ONE_MS = timedelta(milliseconds=1)


class EntityOut(Protocol):
    """Where finished, checked entities go. The bridge core needs only these three calls."""

    def publish(self, entity: Dict[str, Any], now: datetime) -> bool:
        """Hand over one entity. True = accepted (queued or sent). Must not raise and must not block for long."""

    def flush(self, timeout_s: float = 5.0) -> None:
        """Wait (bounded) until everything handed over so far has been sent or given up on."""

    def close(self) -> None:
        """Flush and release everything. Safe to call twice."""


class SinkOut:
    """The simplest EntityOut: write each entity straight into a Sink (jsonl file, in-memory test sink ...)."""

    def __init__(self, sink):
        self.sink = sink

    def publish(self, entity: Dict[str, Any], now: datetime) -> bool:
        """Write the entity as an `entity` record; False when the sink refused it."""
        return bool(self.sink.write('entity', entity, now))

    def flush(self, timeout_s: float = 5.0) -> None:
        """A straight-through output holds nothing back."""
        return None

    def close(self) -> None:
        """Close the sink."""
        self.sink.close()


@dataclass
class CoreSettings:
    """Everything the core needs to know (BridgeConfig.core_settings() builds this from the profile)."""
    mission_id: str
    geoid_undulation_m: Optional[float] = None
    drone_interval_s: float = 1.0
    stale_after_s: float = 10.0
    stale_policy: str = 'stop'
    offline_after_s: float = 30.0
    mission_duration_s: float = 3600.0
    target_margin_s: float = C.TARGET_EXPIRY_MARGIN_S
    area_ttl_s: float = C.AREA_TTL_S
    drop_ttl_s: float = C.AREA_TTL_S
    min_confidence: float = 0.0
    merge_radius_m: float = C.CLUSTER_MERGE_RADIUS_M
    update_min_move_m: float = C.CLUSTER_UPDATE_MIN_MOVE_M
    update_min_conf_gain: float = C.CLUSTER_UPDATE_MIN_CONF_GAIN
    max_clusters: int = C.CLUSTER_MAX_CLUSTERS
    uav_name: str = 'ASTRA UAV'
    areas: Sequence[AreaSpec] = field(default_factory=tuple)

    def validate(self) -> None:
        """Raise ValueError for a setting that cannot work."""
        check_mission_id(self.mission_id)
        if self.stale_policy not in STALE_POLICIES:
            raise ValueError(f'stale_policy must be one of {STALE_POLICIES}')
        for name in ('drone_interval_s', 'stale_after_s', 'offline_after_s', 'mission_duration_s', 'target_margin_s',
                     'area_ttl_s', 'drop_ttl_s'):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not value > 0:
                raise ValueError(f'{name} must be a number greater than 0')
        if self.offline_after_s < self.stale_after_s:
            raise ValueError('offline_after_s must not be smaller than stale_after_s')
        longest = max(self.mission_duration_s + self.target_margin_s, self.area_ttl_s, self.drop_ttl_s)
        if timedelta(seconds=longest) >= C.MAX_EXPIRY - timedelta(hours=1):
            raise ValueError(f'durations are too long: Lattice entities may live less than {C.LATTICE_MAX_EXPIRY.days} days')


class BridgeCore:
    """Turn events into published Lattice entities (see the module docstring). Single-threaded: call it from one thread."""

    def __init__(self, settings: CoreSettings, out: EntityOut):
        settings.validate()
        self.settings = settings
        self.out = out
        self.mission_id = settings.mission_id
        self.uav_id = uav_entity_id(self.mission_id)
        self.clusterer = TargetClusterer(
            self.mission_id, merge_radius_m=settings.merge_radius_m, update_min_move_m=settings.update_min_move_m,
            update_min_conf_gain=settings.update_min_conf_gain, min_confidence=settings.min_confidence,
            max_clusters=settings.max_clusters)
        self.mission_start: Optional[datetime] = None
        self.latest_fix: Optional[DroneFix] = None
        self.first_fix_time: Optional[datetime] = None
        self.last_heard: Optional[datetime] = None
        self.last_drone_publish: Optional[datetime] = None
        self.last_published_fix_time: Optional[datetime] = None
        self.link_state = 'never'                       # never | fresh | stale | offline
        self.source_ended = False
        self.closed = False
        self._source_times: Dict[str, datetime] = {}
        self._drop_counts = {kind: 0 for kind in C.TARGET_KINDS}
        self._stale_since: Optional[datetime] = None
        self.stats: Dict[str, int] = {key: 0 for key in (
            'events', 'fixes', 'fixes_ignored_old', 'drone_published', 'goodbyes',
            'stale_events', 'recoveries', 'sightings', 'targets_created', 'targets_updated', 'targets_unchanged',
            'targets_ignored', 'targets_retired', 'drops_published', 'drops_without_position', 'areas_published',
            'area_failures', 'rejected_invalid', 'publish_failed', 'handle_errors', 'notices')}

    # ------------------------------------------------------------------------------------------ lifecycle
    def start(self, now: datetime, altitude_reference: str = 'amsl') -> None:
        """Warn about the altitude setting if it matters, then publish the search areas and the flight boundary once."""
        now = require_aware('now', now)
        self.mission_start = now
        if altitude_reference == 'amsl':
            warn_if_altitude_approximate(self.settings.geoid_undulation_m, LOG)
        for spec in self.settings.areas:
            self._publish_area(spec, now)

    def handle(self, event: Event, now: datetime) -> None:
        """Process one event. Never raises: an error is logged and counted."""
        if self.closed:
            return
        self.stats['events'] += 1
        try:
            if isinstance(event, DroneFix):
                self._on_fix(event, now)
            elif isinstance(event, TargetSighting):
                self._on_sighting(event, now)
            elif isinstance(event, DropEvent):
                self._on_drop(event, now)
            elif isinstance(event, SourceNotice):
                self._on_notice(event)
        except Exception as exc:                        # one bad event must never stop the bridge
            self.stats['handle_errors'] += 1
            LOG.error('could not process a %s (%s: %s); it was dropped', type(event).__name__,
                      exc.__class__.__name__, exc)

    def tick(self, now: datetime) -> None:
        """Do the periodic work: publish the drone (about once a second) and watch for a silent link."""
        if self.closed or self.latest_fix is None or self.last_heard is None:
            return
        try:
            self._tick_drone(now)
        except Exception as exc:
            self.stats['handle_errors'] += 1
            LOG.error('drone update failed (%s: %s)', exc.__class__.__name__, exc)

    def shutdown(self, now: datetime) -> None:
        """Send the drone's isLive=false update (if it was ever published), flush and close. Safe to call twice."""
        if self.closed:
            return
        try:
            if self.link_state != 'offline':
                self._publish_goodbye(now, 'shutdown')
        except Exception as exc:
            LOG.error('final drone update failed (%s: %s)', exc.__class__.__name__, exc)
        self.closed = True
        for action, label in ((lambda: self.out.flush(5.0), 'flush'), (self.out.close, 'close')):
            try:
                action()
            except Exception as exc:                    # the output must never keep the program from exiting
                LOG.error('output %s failed (%s: %s)', label, exc.__class__.__name__, exc)

    def health_line(self, now: datetime, source_stats: Optional[Dict[str, int]] = None) -> str:
        """One line summarising the state of the bridge, for the periodic log."""
        age = '-' if self.last_heard is None else f'{max(0.0, (now - self.last_heard).total_seconds()):.1f}s'
        person = sum(1 for t in self.clusterer.targets() if t.kind == C.KIND_PERSON)
        tent = sum(1 for t in self.clusterer.targets() if t.kind == C.KIND_TENT)
        s = self.stats
        parts = [f'health: drone={self.link_state} fix_age={age} published={s["drone_published"]}',
                 f'targets={person}p/{tent}t created={s["targets_created"]} updated={s["targets_updated"]}',
                 f'drops={s["drops_published"]} areas={s["areas_published"]}',
                 f'rejected={s["rejected_invalid"]} failed={s["publish_failed"]} errors={s["handle_errors"]}']
        if source_stats:
            extras = [f'{k}={source_stats[k]}' for k in ('received', 'tgt_reports', 'tgt_gaps', 'tgt_duplicates',
                                                         'queue_dropped_other', 'connect_failures', 'link_losses')
                      if source_stats.get(k)]
            if extras:
                parts.append('source: ' + ' '.join(extras))
        return ' | '.join(parts)

    # ------------------------------------------------------------------------------------------ the drone
    def _on_fix(self, fix: DroneFix, now: datetime) -> None:
        if self.latest_fix is not None and fix.fix_time <= self.latest_fix.fix_time:
            self.stats['fixes_ignored_old'] += 1
            return
        self.stats['fixes'] += 1
        if self.first_fix_time is None:
            self.first_fix_time = fix.fix_time
        self.latest_fix = fix
        self.last_heard = fix.arrival if self.last_heard is None else max(self.last_heard, fix.arrival)
        if self.link_state in ('stale', 'offline'):
            silent = (fix.arrival - self._stale_since).total_seconds() if self._stale_since else 0.0
            LOG.info('drone telemetry is back after about %.0f s of silence', max(0.0, silent))
            self.stats['recoveries'] += 1
            self._stale_since = None
        self.link_state = 'fresh'

    def _tick_drone(self, now: datetime) -> None:
        age = max(0.0, (now - self.last_heard).total_seconds())
        s = self.settings
        if age > s.stale_after_s:
            if self.link_state == 'fresh':
                self.link_state = 'stale'
                self._stale_since = self.last_heard
                self.stats['stale_events'] += 1
                LOG.warning('no drone telemetry for %.0f s: the drone is no longer refreshed in Lattice '
                            '(its entry expires by itself; no position is repeated or invented)', age)
            if self.link_state == 'stale' and s.stale_policy == 'not_live' and age > s.offline_after_s:
                self._publish_goodbye(now, f'telemetry silent for {age:.0f} s')
                self.link_state = 'offline'
            return
        if self.last_drone_publish is not None and (now - self.last_drone_publish).total_seconds() < s.drone_interval_s - 1e-6:
            return
        fix = self.latest_fix
        if self.last_published_fix_time is not None and fix.fix_time <= self.last_published_fix_time:
            return                                          # nothing newer than what Lattice already has
        self._publish_drone(fix, now, age, is_live=True)

    def _drone_entity(self, fix: DroneFix, now: datetime, source_time: datetime, age: float, is_live: bool) -> Dict[str, Any]:
        """The drone entity for `fix`. HAE heights are passed through; AMSL heights get the geoid undulation."""
        if fix.alt_hae_m is not None:
            height, undulation = fix.alt_hae_m, 0.0        # already above the ellipsoid: never correct twice
        else:
            height, undulation = fix.alt_amsl_m, self.settings.geoid_undulation_m
        return build_uav_asset(
            self.uav_id, fix.lat_deg, fix.lon_deg, fix_time=source_time, now=max(now, source_time),
            alt_amsl_m=height, geoid_undulation_m=undulation, rel_alt_m=fix.rel_alt_m, heading_deg=fix.heading_deg,
            velocity_ned_mps=fix.velocity_ned_mps, mode=fix.mode, armed=fix.armed,
            battery_percent=fix.battery_percent, link_age_s=min(age, 86400.0), name=self.settings.uav_name,
            created_time=self.first_fix_time, is_live=is_live)

    def _publish_drone(self, fix: DroneFix, now: datetime, age: float, is_live: bool) -> None:
        previous = self._source_times.get(self.uav_id)
        source_time = fix.fix_time if previous is None or fix.fix_time > previous else previous + _ONE_MS
        entity = self._drone_entity(fix, now, source_time, age, is_live)
        if self._emit(entity, now, 'drone'):
            self.stats['drone_published'] += 1
            self.last_drone_publish = now
            self.last_published_fix_time = fix.fix_time
            self._source_times[self.uav_id] = source_time

    def _publish_goodbye(self, now: datetime, reason: str) -> None:
        """The final isLive=false update of the drone (only if the drone was ever published)."""
        previous = self._source_times.get(self.uav_id)
        if self.latest_fix is None or previous is None:
            return
        when = max(now, previous + _ONE_MS)             # strictly newer than the last update, or Lattice ignores it
        age = max(0.0, (now - self.last_heard).total_seconds()) if self.last_heard else 0.0
        entity = self._drone_entity(self.latest_fix, now, when, age, False)
        if self._emit(entity, now, 'drone goodbye'):
            self.stats['goodbyes'] += 1
            self._source_times[self.uav_id] = when
            LOG.info('drone marked not live (%s)', reason)

    # ------------------------------------------------------------------------------------------ targets
    def _target_expiry(self, now: datetime) -> datetime:
        """Mission end + margin (never in the past, never 30 days away)."""
        end = self.mission_start + timedelta(seconds=self.settings.mission_duration_s) if self.mission_start else now
        expiry = max(end, now) + timedelta(seconds=self.settings.target_margin_s)
        return min(expiry, now + C.MAX_EXPIRY - timedelta(minutes=1))

    def _next_source_time(self, entity_id: str, wanted: datetime) -> datetime:
        """`wanted`, or 1 ms after the previous update's time if that is not strictly newer (Lattice's rule)."""
        last = self._source_times.get(entity_id)
        chosen = wanted if last is None or wanted > last else last + _ONE_MS
        self._source_times[entity_id] = chosen
        return chosen

    def _on_sighting(self, sighting: TargetSighting, now: datetime) -> None:
        self.stats['sightings'] += 1
        decision = self.clusterer.feed(sighting.kind, sighting.lat_deg, sighting.lon_deg, sighting.confidence,
                                       sighting.measured_time)
        if decision.action == ACTION_IGNORED:
            self.stats['targets_ignored'] += 1
            LOG.info('target report ignored: %s', decision.reason)
            return
        if decision.action == ACTION_NO_CHANGE:
            self.stats['targets_unchanged'] += 1
            return
        kwargs = decision.track_kwargs()
        kwargs['measured_time'] = self._next_source_time(kwargs['entity_id'], kwargs['measured_time'])
        entity = build_target_track(now=now, expiry_time=self._target_expiry(now), **kwargs)
        if self._emit(entity, now, 'target'):
            key = 'targets_created' if decision.action == ACTION_CREATE else 'targets_updated'
            self.stats[key] += 1
            if decision.action == ACTION_CREATE:
                LOG.info('new %s target %s at %.6f, %.6f (confidence %.0f%%)', kwargs['kind'], kwargs['entity_id'],
                         kwargs['lat_deg'], kwargs['lon_deg'], kwargs['confidence'] * 100)
        for retired in decision.retired:
            goodbye = state_track_kwargs(retired, measured_time=decision.state.last_seen)
            goodbye['measured_time'] = self._next_source_time(retired.entity_id, goodbye['measured_time'])
            entity = build_target_track(now=now, expiry_time=self._target_expiry(now), is_live=False, **goodbye)
            if self._emit(entity, now, 'target merged away'):
                self.stats['targets_retired'] += 1
                LOG.info('targets merged: %s is now part of %s', retired.entity_id, decision.entity_id)

    # ------------------------------------------------------------------------------------------ drops, areas
    def _on_drop(self, drop: DropEvent, now: datetime) -> None:
        if not drop.has_position:
            self.stats['drops_without_position'] += 1
            LOG.warning('a %s payload was released but the drone position was not known: no map point '
                        '(nothing is guessed)', drop.kind)
            return
        self._drop_counts[drop.kind] += 1
        entity = build_drop_point(
            drop_entity_id(self.mission_id, drop.kind, self._drop_counts[drop.kind]), drop.kind, drop.lat_deg,
            drop.lon_deg, release_time=drop.release_time, now=now,
            expiry_time=now + timedelta(seconds=self.settings.drop_ttl_s))
        if self._emit(entity, now, 'drop point'):
            self.stats['drops_published'] += 1
            LOG.info('%s payload released at %.6f, %.6f', drop.kind, drop.lat_deg, drop.lon_deg)

    def _publish_area(self, spec: AreaSpec, now: datetime) -> None:
        try:
            entity = build_area_geo(
                area_entity_id(self.mission_id, spec.name), spec.name.replace('_', ' ').capitalize(),
                spec.corners_latlon, source_update_time=now, now=now,
                expiry_time=now + timedelta(seconds=self.settings.area_ttl_s), geo_type=spec.geo_type,
                description=spec.note or None)
        except ValueError as exc:
            self.stats['area_failures'] += 1
            LOG.error('area %r cannot be published: %s', spec.name, exc)
            return
        if self._emit(entity, now, f'area {spec.name}'):
            self.stats['areas_published'] += 1
        else:
            self.stats['area_failures'] += 1

    def _on_notice(self, notice: SourceNotice) -> None:
        self.stats['notices'] += 1
        level = {'link_lost': logging.WARNING, 'reconnecting': logging.WARNING, 'silent': logging.WARNING,
                 'warning': logging.WARNING}.get(notice.code, logging.INFO)
        LOG.log(level, 'source: %s', notice.message)
        if notice.code == 'ended':
            self.source_ended = True

    # ------------------------------------------------------------------------------------------ output
    def _emit(self, entity: Dict[str, Any], now: datetime, what: str) -> bool:
        """Check the entity against the contract, then hand it to the output. True when it was accepted."""
        problems = find_problems(entity, require_aware('now', now))
        if problems:
            self.stats['rejected_invalid'] += 1
            LOG.error('%s was NOT sent, it breaks the Lattice contract: %s', what, '; '.join(problems[:3]))
            return False
        try:
            accepted = bool(self.out.publish(entity, now))
        except Exception as exc:                        # an output must not raise, but if it does the bridge lives on
            accepted = False
            LOG.error('output failed for %s (%s: %s)', what, exc.__class__.__name__, exc)
        if not accepted:
            self.stats['publish_failed'] += 1
        return accepted


def run_loop(core: BridgeCore, source, stop: threading.Event, *, poll_s: float = 0.2,
             health_interval_s: float = 30.0) -> str:
    """Drive `core` from `source` until `stop` is set or the source reports that it ended.

    Returns 'stopped' or 'ended'. Each round: wait up to `poll_s` for events, hand them to the core, run the
    core's periodic work, and log a health line when `health_interval_s` has passed (in the source's own time,
    so a fast replay logs by recorded time). An exception from the source is logged and retried after a short
    pause; the loop itself never dies from outside trouble.
    """
    last_health = source.now()
    last_error_log: Optional[float] = None
    errors = 0
    while not stop.is_set():
        try:
            events: List[Event] = source.poll(poll_s)
        except Exception as exc:
            errors += 1
            if errors == 1 or errors % 100 == 0:
                LOG.error('the %s source failed (%s: %s); retrying', source.name, exc.__class__.__name__, exc)
            stop.wait(0.5)
            events = []
        now = source.now()
        for event in events:
            core.handle(event, now)
        core.tick(now)
        if health_interval_s > 0 and (now - last_health).total_seconds() >= health_interval_s:
            LOG.info(core.health_line(now, source.stats()))
            last_health = now
        if core.source_ended:
            return 'ended'
    return 'stopped'
