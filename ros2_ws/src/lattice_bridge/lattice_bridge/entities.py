"""
lattice_bridge/entities.py -- PURPOSE: build the JSON bodies ("entities") we send to Anduril Lattice, and
check them. Four builders turn plain numbers into plain dicts that are ready for
`PUT /api/v1/entities`:

    build_uav_asset      our drone (an "asset": a vehicle we control)       -> TEMPLATE_ASSET
    build_target_track   a detected hiker or tent (a "track": seen by a sensor) -> TEMPLATE_TRACK
    build_area_geo       a search area or the flight boundary (a "geo-entity": a polygon) -> TEMPLATE_GEO
    build_drop_point     where a payload was released (a geo-entity point)  -> TEMPLATE_GEO

and `validate_entity` is the contract checker: it takes ANY entity dict (ours or a hand-made one) and says
what is wrong with it, using the rules of the Lattice docs (required components, expiryTime in the future and
under 30 days, valid enum words, UTC times with Z, finite numbers, camelCase keys).

The rules every builder follows:
  * PURE: numbers and datetimes in, a dict out. No file or network access, and NO clock reads: the caller
    passes `now` (the publish time) and the measurement time, so the same inputs always give the same dict.
  * Missing optional inputs (None) leave the matching component OUT. A made-up value is never inserted.
  * A bad input (NaN, out of range, naive datetime, expiry in the past or 30+ days away) raises ValueError
    with a message that names the input. It never returns a half-wrong entity.
  * Times are written as UTC text with a Z ('2026-10-24T15:30:00.000000Z'); naive datetimes are refused.
  * Numbers are rounded to sensible sizes (8 decimals of a degree = about 1 mm; heights to 1 mm).

Altitude, the part that is easy to get wrong: MAVLink's GLOBAL_POSITION_INT.alt is height above MEAN SEA LEVEL
(AMSL). Lattice's altitudeHaeMeters is height above the WGS84 ELLIPSOID (HAE). They differ by the geoid
undulation N (about -30 m near the Skyway Range in Oklahoma, i.e. HAE = AMSL - 30). HAE = AMSL + N. N is a
CONFIGURATION VALUE here (`geoid_undulation_m`), never guessed. If it is None ("not configured") the AMSL
value is published as if it were HAE, the description says so, and `altitude_is_approximate()` lets the
caller log a warning. altitudeAglMeters is filled from MAVLink's relative_alt, which is height above HOME (the
take-off point), NOT above the terrain under the drone; on sloped ground it is only an approximation.

Pure Python (json, math, re, datetime): no ROS, no I/O.
"""

from __future__ import annotations

import json
import math
import re
from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence, Tuple

from . import constants as C
from .geo import (
    clean_polygon_ring, heading_to_attitude_enu, is_real_number, require_lat_lon, require_number,
)
from .timeutil import format_utc, parse_utc, plus_seconds, require_aware

Entity = Dict[str, Any]

#: The long text callers should LOG (once) when the geoid undulation is not configured.
ALTITUDE_WARNING = (
    'altitude: AMSL is published as HAE because no geoid undulation is configured; Lattice heights can be '
    'wrong by tens of metres (about -30 m near the Skyway Range, Oklahoma). Set geoid_undulation_m.'
)
#: The short version that goes into the drone's `description`, so people looking at the Lattice UI see it too.
ALTITUDE_NOTE = 'altitude: AMSL used as HAE (geoid undulation not configured)'

_ENTITY_ID = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}')
_MODE = re.compile(r'[A-Za-z0-9_() .-]{1,24}')
_KEY = re.compile(r'[a-z][A-Za-z0-9]*')
# Time fields that find_problems() checks one by one below (the generic loop skips them to avoid double reports).
_CHECKED_BELOW = {('', 'expiryTime'), ('', 'createdTime'), ('provenance.', 'sourceUpdateTime')}
_HEALTH_RANK = {C.HEALTH_STATUS_HEALTHY: 0, C.HEALTH_STATUS_WARN: 1, C.HEALTH_STATUS_FAIL: 2}


class EntityValidationError(ValueError):
    """Raised by validate_entity(); `.problems` is the full list of what is wrong (plain sentences)."""

    def __init__(self, problems: Sequence[str]):
        self.problems = tuple(problems)
        shown = '; '.join(self.problems[:5])
        more = f' (+{len(self.problems) - 5} more)' if len(self.problems) > 5 else ''
        super().__init__(f'invalid Lattice entity: {shown}{more}')


# ===================================================================================================
# Small input checks shared by the builders
# ===================================================================================================

def _entity_id(entity_id) -> str:
    if not isinstance(entity_id, str) or _ENTITY_ID.fullmatch(entity_id) is None:
        raise ValueError(f'entity_id must be 1-{C.MAX_ENTITY_ID_LEN} letters, digits or _ . : - '
                         f'(starting with a letter or digit), got {str(entity_id)[:40]!r}')
    return entity_id


def _text(name: str, value, max_len: int) -> str:
    """A human text: a non-empty printable string of at most max_len characters."""
    if not isinstance(value, str) or not value.strip() or not value.isprintable() or len(value) > max_len:
        raise ValueError(f'{name} must be a printable text of 1-{max_len} characters, got {str(value)[:40]!r}')
    return value


def _bool(name: str, value) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f'{name} must be True or False, got {str(value)[:20]!r}')
    return value


def _kind(kind) -> str:
    if kind not in C.TARGET_KINDS:
        raise ValueError(f'kind must be one of {C.TARGET_KINDS}, got {str(kind)[:20]!r}')
    return kind


def _check_times(now: datetime, expiry_time: datetime) -> Tuple[datetime, datetime]:
    """now and expiry as UTC; the expiry must be after now and at most C.MAX_EXPIRY (29 days) later."""
    now_utc = require_aware('now', now)
    expiry_utc = require_aware('expiry_time', expiry_time)
    if expiry_utc <= now_utc:
        raise ValueError('expiry_time must be in the future (later than now): Lattice deletes expired entities')
    if expiry_utc - now_utc > C.MAX_EXPIRY:
        raise ValueError(f'expiry_time is more than {C.MAX_EXPIRY.days} days away; Lattice requires less than '
                         f'{C.LATTICE_MAX_EXPIRY.days} days (we keep a safety margin)')
    return now_utc, expiry_utc


def _check_past_time(name: str, value: datetime, now: datetime) -> datetime:
    """A measurement or creation time as UTC. It may be old (stale data keeps its true time) but not future."""
    moment = require_aware(name, value)
    if (moment - now).total_seconds() > C.FUTURE_TOLERANCE_S:
        raise ValueError(f'{name} is {(moment - now).total_seconds():.0f} s in the future; '
                         f'check the clocks (allowed: {C.FUTURE_TOLERANCE_S:.0f} s)')
    return moment


def _round_position(lat: float, lon: float) -> Dict[str, float]:
    return {'latitudeDegrees': round(lat, 8), 'longitudeDegrees': round(lon, 8)}


def _skeleton(entity_id: str, name: str, description: str, is_live: bool, expiry: datetime,
              data_type: str, source_time: datetime, created: Optional[datetime]) -> Entity:
    """The components every entity needs: id, isLive, expiryTime, provenance, aliases (+ description)."""
    entity: Entity = {
        'entityId': entity_id,
        'description': description,
        'isLive': is_live,
        'expiryTime': format_utc(expiry),
        'aliases': {'name': name},
        'provenance': {
            'integrationName': C.INTEGRATION_NAME,
            'dataType': data_type,
            'sourceUpdateTime': format_utc(source_time),
        },
    }
    if created is not None:
        entity['createdTime'] = format_utc(created)
    return entity


# ===================================================================================================
# Altitude helpers
# ===================================================================================================

def altitude_is_approximate(geoid_undulation_m: Optional[float]) -> bool:
    """True when no geoid undulation is configured (None), so AMSL is being published as HAE.

    The caller (the bridge) should log ALTITUDE_WARNING once when this is True. An explicit 0.0 counts as
    configured: somebody decided that.
    """
    return geoid_undulation_m is None


def hae_from_amsl(alt_amsl_m, geoid_undulation_m: Optional[float] = None) -> float:
    """Height above the WGS84 ellipsoid from height above mean sea level: HAE = AMSL + N.

    N (`geoid_undulation_m`) is the height of the geoid above the ellipsoid at this place; it is negative in
    most of the USA. None means "not configured" and is treated as 0 (see altitude_is_approximate).
    """
    amsl = require_number('alt_amsl_m', alt_amsl_m, C.ALT_MIN_M, C.ALT_MAX_M)
    undulation = 0.0 if geoid_undulation_m is None else require_number(
        'geoid_undulation_m', geoid_undulation_m, -120.0, 120.0)
    return round(amsl + undulation, 3)


def confidence_to_track_quality(confidence) -> int:
    """Detector confidence 0..1 -> Lattice track quality 0..15 (an approximation, see the README)."""
    value = require_number('confidence', confidence, 0.0, 1.0)
    return int(math.floor(value * C.TRACK_QUALITY_MAX + 0.5))


# ===================================================================================================
# 1. Our drone
# ===================================================================================================

def _health_component(component_id: str, name: str, health: str, message: str) -> Dict[str, Any]:
    return {'id': component_id, 'name': name, 'health': health,
            'messages': [{'status': health, 'message': message}]}


def _uav_health(battery_percent, link_age_s) -> Optional[Dict[str, Any]]:
    """The `health` component from battery and radio-link age; None when neither is known."""
    components: List[Dict[str, Any]] = []
    if battery_percent is not None:
        percent = require_number('battery_percent', battery_percent, 0.0, 100.0)
        if percent < C.BATTERY_FAIL_BELOW_PERCENT:
            state = C.HEALTH_STATUS_FAIL
        elif percent < C.BATTERY_WARN_BELOW_PERCENT:
            state = C.HEALTH_STATUS_WARN
        else:
            state = C.HEALTH_STATUS_HEALTHY
        components.append(_health_component('battery-0', 'Battery', state, f'Battery at {percent:.0f}% charge.'))
    if link_age_s is not None:
        age = require_number('link_age_s', link_age_s, 0.0, 86400.0)
        if age <= C.LINK_OK_MAX_AGE_S:
            state = C.HEALTH_STATUS_HEALTHY
        elif age <= C.LINK_WARN_MAX_AGE_S:
            state = C.HEALTH_STATUS_WARN
        else:
            state = C.HEALTH_STATUS_FAIL
        components.append(_health_component('telemetry-link', 'Telemetry link', state,
                                            f'Last telemetry {age:.1f} s ago.'))
    if not components:
        return None
    worst = max((c['health'] for c in components), key=_HEALTH_RANK.__getitem__)
    return {'healthStatus': worst, 'components': components}


def _activity_text(mode, armed) -> Optional[str]:
    """status.platformActivity from the flight mode and the armed flag; None when neither is known."""
    text = None
    if mode is not None:
        if not isinstance(mode, str) or _MODE.fullmatch(mode) is None:
            raise ValueError(f'mode must be 1-{C.MAX_MODE_LEN} letters/digits/_ (), got {str(mode)[:30]!r}')
        text = mode
    if armed is not None:
        _bool('armed', armed)
        if armed is False:
            text = f'{text} (disarmed)' if text else 'DISARMED'
        elif text is None:
            text = 'ARMED'
    return text


def build_uav_asset(
    entity_id: str,
    lat_deg,
    lon_deg,
    *,
    fix_time: datetime,
    now: datetime,
    expiry_time: Optional[datetime] = None,
    alt_amsl_m=None,
    rel_alt_m=None,
    geoid_undulation_m: Optional[float] = None,
    heading_deg=None,
    velocity_ned_mps=None,
    mode: Optional[str] = None,
    armed: Optional[bool] = None,
    battery_percent=None,
    link_age_s=None,
    name: str = 'ASTRA UAV',
    description: Optional[str] = None,
    specific_type: Optional[str] = None,
    created_time: Optional[datetime] = None,
    is_live: bool = True,
) -> Entity:
    """Our drone as a Lattice asset entity.

    Required: `entity_id`, `lat_deg`, `lon_deg` (a real fix, not (0, 0)), `fix_time` (when the GPS fix was
    taken: becomes provenance.sourceUpdateTime) and `now` (the publish time; only used to check the expiry).
    `expiry_time` defaults to now + 120 s (C.UAV_TTL_S): publish every second and the drone disappears from
    the map about two minutes after the link dies. Pass `is_live=False` to build the "goodbye" update that
    makes Lattice delete the entity at shutdown (it still needs a future expiry_time, as Lattice asks). Give
    that update a NEW `fix_time` (the shutdown time): Lattice only accepts an update whose sourceUpdateTime
    changed, so a goodbye that repeats the last fix time could be ignored.

    Optional (None = component left out, never invented):
      alt_amsl_m, geoid_undulation_m -> location.position.altitudeHaeMeters = AMSL + N (see module doc)
      rel_alt_m          -> altitudeAglMeters, height above HOME, not above terrain
      heading_deg        -> location.attitudeEnu (compass degrees 0..360 -> ENU quaternion)
      velocity_ned_mps   -> (north, east, down) m/s -> location.velocityEnu + speedMps
      mode, armed        -> status.platformActivity ("AUTO", "GUIDED (disarmed)", "DISARMED" ...)
      battery_percent, link_age_s -> health.components (battery-0, telemetry-link) and the rolled-up healthStatus
    Lattice sets health.connectionStatus itself (online while updates arrive, offline after a minute without),
    so it is deliberately not set here.
    """
    entity_id = _entity_id(entity_id)
    lat, lon = require_lat_lon(lat_deg, lon_deg, 'UAV position')
    now_utc = require_aware('now', now)
    fix_utc = _check_past_time('fix_time', fix_time, now_utc)
    created = None if created_time is None else _check_past_time('created_time', created_time, now_utc)
    expiry = plus_seconds(now_utc, C.UAV_TTL_S) if expiry_time is None else expiry_time
    now_utc, expiry = _check_times(now_utc, expiry)
    _bool('is_live', is_live)
    display = _text('name', name, C.MAX_NAME_LEN)

    position = _round_position(lat, lon)
    approximate = False
    if alt_amsl_m is not None:
        position['altitudeHaeMeters'] = hae_from_amsl(alt_amsl_m, geoid_undulation_m)
        approximate = altitude_is_approximate(geoid_undulation_m)
    if rel_alt_m is not None:
        position['altitudeAglMeters'] = round(require_number('rel_alt_m', rel_alt_m, C.ALT_MIN_M, C.ALT_MAX_M), 3)

    if description is None:
        description = 'ASTRA UAV telemetry (MAVLink)' + (f' | {ALTITUDE_NOTE}' if approximate else '')
    description = _text('description', description, C.MAX_DESCRIPTION_LEN)

    location: Dict[str, Any] = {'position': position}
    if heading_deg is not None:
        location['attitudeEnu'] = heading_to_attitude_enu(heading_deg)
    if velocity_ned_mps is not None:
        try:
            v_north, v_east, v_down = velocity_ned_mps
        except (TypeError, ValueError):
            raise ValueError('velocity_ned_mps must be (north, east, down) in m/s') from None
        v_n = require_number('velocity north', v_north, -C.SPEED_MAX_MPS, C.SPEED_MAX_MPS)
        v_e = require_number('velocity east', v_east, -C.SPEED_MAX_MPS, C.SPEED_MAX_MPS)
        v_d = require_number('velocity down', v_down, -C.SPEED_MAX_MPS, C.SPEED_MAX_MPS)
        location['velocityEnu'] = {'e': round(v_e, 3), 'n': round(v_n, 3), 'u': round(-v_d, 3) + 0.0}
        location['speedMps'] = round(math.sqrt(v_n * v_n + v_e * v_e + v_d * v_d), 3)

    entity = _skeleton(entity_id, display, description, is_live, expiry, C.DATA_TYPE_UAV, fix_utc, created)
    entity['ontology'] = {'template': C.TEMPLATE_ASSET, 'platformType': C.PLATFORM_TYPE_UAV}
    if specific_type is not None:
        entity['ontology']['specificType'] = _text('specific_type', specific_type, C.MAX_NAME_LEN)
    entity['location'] = location
    entity['milView'] = {'disposition': C.UAV_DISPOSITION, 'environment': C.UAV_ENVIRONMENT}
    health = _uav_health(battery_percent, link_age_s)
    if health is not None:
        entity['health'] = health
    activity = _activity_text(mode, armed)
    if activity is not None:
        entity['status'] = {'platformActivity': activity}
    return entity


# ===================================================================================================
# 2. A detected hiker or tent
# ===================================================================================================

def build_target_track(
    entity_id: str,
    kind: str,
    lat_deg,
    lon_deg,
    confidence,
    *,
    measured_time: datetime,
    now: datetime,
    expiry_time: datetime,
    sightings: int = 1,
    first_seen_time: Optional[datetime] = None,
    alt_hae_m=None,
    name: Optional[str] = None,
    description: Optional[str] = None,
    is_live: bool = True,
) -> Entity:
    """A detected target as a Lattice track entity.

    `lat_deg`/`lon_deg` are the PROJECTED TARGET position (where the object lies on the ground, from
    main/src/target_geolocation.py), never the drone's own position. `confidence` is the detector's 0..1.
    `measured_time` (when the photo was taken) becomes provenance.sourceUpdateTime and
    tracked.lastMeasurementTime: Lattice only accepts an update when sourceUpdateTime changes, and it should
    be the time of the measurement, never "now" for old data. `expiry_time` is the end of the mission plus a
    margin (C.TARGET_EXPIRY_MARGIN_S is the recommended margin); it must stay under 30 days.

    Where the detection details live (the Lattice entity model has no confidence field):
      description                    '<Label> detected by ASTRA ODLC: confidence 95%, 3 sighting(s)'
      tracked.trackQualityWrapper    confidence mapped onto Lattice's 0..15 quality score (an approximation)
      tracked.lastMeasurementTime    the measurement time
      ontology.platformType          the class words of C.PLATFORM_TYPE_* (confirm them in the Schema Registry)
    No altitude is invented: pass `alt_hae_m` only if you really know it.
    """
    entity_id = _entity_id(entity_id)
    kind = _kind(kind)
    lat, lon = require_lat_lon(lat_deg, lon_deg, 'target position')
    quality = confidence_to_track_quality(confidence)
    percent = int(math.floor(float(confidence) * 100.0 + 0.5))
    now_utc = require_aware('now', now)
    measured = _check_past_time('measured_time', measured_time, now_utc)
    created = None if first_seen_time is None else _check_past_time('first_seen_time', first_seen_time, now_utc)
    now_utc, expiry = _check_times(now_utc, expiry_time)
    _bool('is_live', is_live)
    if isinstance(sightings, bool) or not isinstance(sightings, int) or not 1 <= sightings <= 1_000_000:
        raise ValueError(f'sightings must be an integer from 1 to 1000000, got {str(sightings)[:20]!r}')
    label = C.LABEL_BY_KIND[kind]
    display = _text('name', name if name is not None else f'{label} target (ASTRA)', C.MAX_NAME_LEN)
    if description is None:
        description = (f'{label} detected by ASTRA ODLC: confidence {percent}%, '
                       f'{sightings} sighting(s)')
    description = _text('description', description, C.MAX_DESCRIPTION_LEN)

    position = _round_position(lat, lon)
    if alt_hae_m is not None:
        position['altitudeHaeMeters'] = round(require_number('alt_hae_m', alt_hae_m, C.ALT_MIN_M, C.ALT_MAX_M), 3)

    entity = _skeleton(entity_id, display, description, is_live, expiry, C.DATA_TYPE_TARGET, measured, created)
    entity['ontology'] = {'template': C.TEMPLATE_TRACK, 'platformType': C.PLATFORM_TYPE_BY_KIND[kind]}
    entity['location'] = {'position': position}
    entity['milView'] = {'disposition': C.TARGET_DISPOSITION, 'environment': C.TARGET_ENVIRONMENT}
    entity['tracked'] = {'trackQualityWrapper': quality, 'lastMeasurementTime': format_utc(measured)}
    return entity


# ===================================================================================================
# 3. Map shapes: areas and drop points
# ===================================================================================================

def build_area_geo(
    entity_id: str,
    name: str,
    corners_latlon: Sequence[Sequence[float]],
    *,
    source_update_time: datetime,
    now: datetime,
    expiry_time: datetime,
    geo_type: str = C.GEO_TYPE_SEARCH_AREA,
    description: Optional[str] = None,
    is_live: bool = True,
) -> Entity:
    """A search area or the flight boundary as a Lattice geo-entity polygon.

    `corners_latlon` is a list of (latitude, longitude) corners, open or closed. The ring that is sent is
    closed (first corner repeated at the end, as Lattice requires) and runs counter-clockwise (the RFC 7946
    rule that the Lattice reference points to); a clockwise list is reversed for you. Self-crossing rings,
    fewer than 3 corners, zero area and (0, 0) corners raise ValueError.
    `geo_type`: C.GEO_TYPE_SEARCH_AREA (GEO_TYPE_GENERAL, default) or C.GEO_TYPE_FLIGHT_BOUNDARY
    (GEO_TYPE_CONTROL_AREA). `source_update_time` = when the geometry was defined or last changed (the
    mission start is fine). Expiry is explicit and finite: noExpiry is never used.
    """
    entity_id = _entity_id(entity_id)
    display = _text('name', name, C.MAX_NAME_LEN)
    ring = clean_polygon_ring(corners_latlon, name=f'area {display!r}')
    if not _in(geo_type, C.GEO_TYPES):
        raise ValueError(f'geo_type must be one of {sorted(C.GEO_TYPES)}, got {str(geo_type)[:40]!r}')
    now_utc = require_aware('now', now)
    source = _check_past_time('source_update_time', source_update_time, now_utc)
    now_utc, expiry = _check_times(now_utc, expiry_time)
    _bool('is_live', is_live)
    if description is None:
        description = f'{display}: ASTRA mission area'
    description = _text('description', description, C.MAX_DESCRIPTION_LEN)

    entity = _skeleton(entity_id, display, description, is_live, expiry, C.DATA_TYPE_AREA, source, None)
    entity['ontology'] = {'template': C.TEMPLATE_GEO}
    entity['geoDetails'] = {'type': geo_type}
    entity['geoShape'] = {'polygon': {'rings': [{'positions': [
        {'position': _round_position(lat, lon)} for lat, lon in ring]}]}}
    return entity


def build_drop_point(
    entity_id: str,
    kind: str,
    lat_deg,
    lon_deg,
    *,
    release_time: datetime,
    now: datetime,
    expiry_time: datetime,
    alt_hae_m=None,
    name: Optional[str] = None,
    description: Optional[str] = None,
    is_live: bool = True,
) -> Entity:
    """Where a payload was released, as a Lattice geo-entity point.

    The position is the DRONE's position at the release (that is what main_controller knows), not where the
    payload came to rest. `release_time` becomes provenance.sourceUpdateTime.
    """
    entity_id = _entity_id(entity_id)
    kind = _kind(kind)
    lat, lon = require_lat_lon(lat_deg, lon_deg, 'drop position')
    now_utc = require_aware('now', now)
    release = _check_past_time('release_time', release_time, now_utc)
    now_utc, expiry = _check_times(now_utc, expiry_time)
    _bool('is_live', is_live)
    label = C.LABEL_BY_KIND[kind]
    display = _text('name', name if name is not None else f'{label} payload drop (ASTRA)', C.MAX_NAME_LEN)
    if description is None:
        description = f'{label} payload released here (drone position at release)'
    description = _text('description', description, C.MAX_DESCRIPTION_LEN)
    position = _round_position(lat, lon)
    if alt_hae_m is not None:
        position['altitudeHaeMeters'] = round(require_number('alt_hae_m', alt_hae_m, C.ALT_MIN_M, C.ALT_MAX_M), 3)

    entity = _skeleton(entity_id, display, description, is_live, expiry, C.DATA_TYPE_DROP, release, None)
    entity['ontology'] = {'template': C.TEMPLATE_GEO}
    entity['geoDetails'] = {'type': C.GEO_TYPE_DROP_POINT}
    entity['geoShape'] = {'point': {'position': position}}
    return entity


# ===================================================================================================
# The contract checker
# ===================================================================================================

_MAX_DEPTH = 12


def _in(value, allowed) -> bool:
    """True when `value` is a string that is one of `allowed` (junk such as a list is simply "not in")."""
    return isinstance(value, str) and value in allowed


def _walk(value, path: str = '', depth: int = 0):
    """Yield (path, key, item) for every dict entry anywhere inside `value` (lists are entered).

    Raises ValueError when the data is nested more than _MAX_DEPTH levels (a real entity is about 6 deep).
    """
    if depth > _MAX_DEPTH:
        raise ValueError('nested too deeply')
    if isinstance(value, dict):
        for key, item in value.items():
            yield path, key, item
            yield from _walk(item, f'{path}{key}.', depth + 1)
    elif isinstance(value, list):
        for index, item in enumerate(value):
            yield from _walk(item, f'{path}[{index}].', depth + 1)


def _dig(entity: Dict[str, Any], *keys):
    """entity[k1][k2]... or None when any step is missing."""
    current: Any = entity
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


def _check_position(position, where: str, add) -> None:
    if not isinstance(position, dict):
        add(f'{where} must be an object with latitudeDegrees and longitudeDegrees')
        return
    lat, lon = position.get('latitudeDegrees'), position.get('longitudeDegrees')
    if not is_real_number(lat) or not -90.0 <= lat <= 90.0:
        add(f'{where}.latitudeDegrees must be a finite number from -90 to 90')
    if not is_real_number(lon) or not -180.0 <= lon <= 180.0:
        add(f'{where}.longitudeDegrees must be a finite number from -180 to 180')
    for key in ('altitudeHaeMeters', 'altitudeAglMeters'):
        if key in position and (not is_real_number(position[key])
                                or not C.ALT_MIN_M <= position[key] <= C.ALT_MAX_M):
            add(f'{where}.{key} must be a finite number from {C.ALT_MIN_M:.0f} to {C.ALT_MAX_M:.0f}')


def _check_geo_shape(shape, add) -> None:
    if not isinstance(shape, dict):
        add('geoShape must be an object')
        return
    kinds = [k for k in shape if _in(k, C.GEO_SHAPE_KINDS)]
    if len(kinds) != 1 or len(shape) != 1:
        add(f'geoShape must hold exactly one of {sorted(C.GEO_SHAPE_KINDS)}, has {sorted(shape)}')
        return
    if kinds[0] == 'point':
        _check_position(_dig(shape, 'point', 'position'), 'geoShape.point.position', add)
    elif kinds[0] == 'polygon':
        rings = _dig(shape, 'polygon', 'rings')
        if not isinstance(rings, list) or not rings:
            add('geoShape.polygon.rings must be a non-empty list')
            return
        for r_index, ring in enumerate(rings):
            positions = ring.get('positions') if isinstance(ring, dict) else None
            where = f'geoShape.polygon.rings[{r_index}]'
            if not isinstance(positions, list) or len(positions) < C.MIN_POLYGON_VERTICES + 1:
                add(f'{where}.positions needs at least {C.MIN_POLYGON_VERTICES + 1} points (closed ring)')
                continue
            points = []
            for p_index, item in enumerate(positions):
                position = item.get('position') if isinstance(item, dict) else None
                _check_position(position, f'{where}.positions[{p_index}].position', add)
                if isinstance(position, dict):
                    points.append((position.get('latitudeDegrees'), position.get('longitudeDegrees')))
            if points and points[0] != points[-1]:
                add(f'{where} is not closed: the first and last point must be the same')


def _check_health(health, add) -> None:
    if not isinstance(health, dict):
        add('health must be an object')
        return
    for key in ('connectionStatus', 'healthStatus'):
        valid = C.CONNECTION_STATUSES if key == 'connectionStatus' else C.HEALTH_STATUSES
        if key in health and not _in(health[key], valid):
            add(f'health.{key} {str(health[key])[:40]!r} is not a valid Lattice enum word')
    components = health.get('components', [])
    if not isinstance(components, list):
        add('health.components must be a list')
        return
    for index, comp in enumerate(components):
        if not isinstance(comp, dict) or not comp.get('id') or not comp.get('name'):
            add(f'health.components[{index}] needs a stable id and a name')
        elif not _in(comp.get('health'), C.HEALTH_STATUSES):
            add(f'health.components[{index}].health {str(comp.get("health"))[:40]!r} is not a valid enum word')


def find_problems(entity, now: datetime) -> List[str]:
    """Return a list of everything wrong with `entity` (an empty list means: it meets the contract).

    `now` (aware datetime) is the time the entity is about to be published; it is what "expiryTime in the
    future and under 30 days" is measured against. Checks: plain JSON data without NaN/infinity; camelCase
    keys; known top-level keys; required components (entityId, isLive, expiryTime, provenance.integrationName,
    provenance.sourceUpdateTime, aliases.name; template parts for asset / track / geo-entity); every enum
    word is a real Lattice word; every time is UTC text with Z; expiry is after `now` and at most 29 days
    away (our margin under Lattice's 30); sourceUpdateTime is not in the future; noExpiry is not used;
    latitude/longitude/altitude ranges; polygon rings closed; quaternion of unit length; quality 0..15.
    """
    now_utc = require_aware('now', now)
    if not isinstance(entity, dict):
        return ['entity must be a dict']
    try:
        json.dumps(entity, allow_nan=False)
    except (TypeError, ValueError, RecursionError) as exc:
        return [f'entity is not plain JSON data (NaN, infinity, a set or a loop inside?): {str(exc)[:100]}']
    problems: List[str] = []
    add = problems.append

    try:
        for path, key, value in _walk(entity):
            if not isinstance(key, str) or _KEY.fullmatch(key) is None:
                add(f'key {path}{str(key)[:30]!r} is not camelCase (REST JSON uses platformType, not platform_type)')
            elif key.endswith('Time') and (path, key) not in _CHECKED_BELOW:
                _check_time_text(f'{path}{key}', value, add)
    except ValueError:
        return ['entity is nested too deeply to be a Lattice entity']
    unknown = sorted(str(k) for k in entity if k not in C.ENTITY_TOP_LEVEL_KEYS)
    if unknown:
        add(f'unknown top-level keys {unknown[:5]} (not part of the PublishEntity body)')

    # --- the five components every entity needs ---
    entity_id = entity.get('entityId')
    if not isinstance(entity_id, str) or _ENTITY_ID.fullmatch(entity_id) is None:
        add('entityId must be a string of letters, digits and _ . : - (at most 128 characters)')
    if not isinstance(entity.get('isLive'), bool):
        add('isLive must be true or false')
    if entity.get('noExpiry'):
        add('noExpiry must not be used: entities must expire (Lattice requires expiryTime < 30 days)')
    expiry_text = entity.get('expiryTime')
    expiry = _check_time_text('expiryTime', expiry_text, add)
    if expiry is not None:
        if expiry <= now_utc:
            add('expiryTime is not in the future: Lattice deletes the entity immediately')
        elif expiry - now_utc > C.MAX_EXPIRY:
            add(f'expiryTime is {(expiry - now_utc).days} days away; the limit is under 30 days '
                f'(we allow at most {C.MAX_EXPIRY.days})')
    if 'createdTime' in entity:
        created = _check_time_text('createdTime', entity['createdTime'], add)
        if created is not None and (created - now_utc).total_seconds() > C.FUTURE_TOLERANCE_S:
            add('createdTime is in the future')
    provenance = entity.get('provenance')
    if not isinstance(provenance, dict) or not provenance.get('integrationName'):
        add('provenance.integrationName is required')
    elif provenance.get('integrationName') != C.INTEGRATION_NAME:
        add(f'provenance.integrationName must be {C.INTEGRATION_NAME!r}')
    if not isinstance(provenance, dict) or not provenance.get('dataType'):
        add('provenance.dataType is required (Lattice examples always set it)')
    source = _check_time_text('provenance.sourceUpdateTime', _dig(entity, 'provenance', 'sourceUpdateTime'), add)
    if source is not None and (source - now_utc).total_seconds() > C.FUTURE_TOLERANCE_S:
        add('provenance.sourceUpdateTime is in the future (it must be the measurement time)')
    name = _dig(entity, 'aliases', 'name')
    if not isinstance(name, str) or not name.strip() or len(name) > C.MAX_NAME_LEN:
        add('aliases.name is required (the name people see in the Lattice UI)')
    description = entity.get('description')
    if description is not None and (not isinstance(description, str)
                                    or len(description) > C.MAX_DESCRIPTION_LEN):
        add('description must be a text of reasonable length')

    # --- the shape-specific parts ---
    template = _dig(entity, 'ontology', 'template')
    if not _in(template, C.TEMPLATES):
        add(f'ontology.template {str(template)[:40]!r} is not a valid Lattice template')
    if template in (C.TEMPLATE_ASSET, C.TEMPLATE_TRACK):
        if not _dig(entity, 'ontology', 'platformType'):
            add('ontology.platformType is required for assets and tracks')
        _check_position(_dig(entity, 'location', 'position'), 'location.position', add)
        mil = entity.get('milView')
        if not isinstance(mil, dict):
            add('milView is required for assets and tracks')
        else:
            if not _in(mil.get('disposition'), C.DISPOSITIONS):
                add(f'milView.disposition {str(mil.get("disposition"))[:40]!r} is not a valid enum word')
            if not _in(mil.get('environment'), C.ENVIRONMENTS):
                add(f'milView.environment {str(mil.get("environment"))[:40]!r} is not a valid enum word')
    if template == C.TEMPLATE_GEO:
        if not _in(_dig(entity, 'geoDetails', 'type'), C.GEO_TYPES):
            add(f'geoDetails.type {str(_dig(entity, "geoDetails", "type"))[:40]!r} is not a valid Lattice geo type')
        if 'geoShape' not in entity:
            add('geoShape is required for geo-entities')
        else:
            _check_geo_shape(entity['geoShape'], add)
    elif 'geoShape' in entity or 'geoDetails' in entity:
        add('geoShape / geoDetails belong to TEMPLATE_GEO entities only')

    # --- optional components we use ---
    if 'health' in entity:
        _check_health(entity['health'], add)
    quality = _dig(entity, 'tracked', 'trackQualityWrapper')
    if quality is not None and (isinstance(quality, bool) or not isinstance(quality, int)
                                or not 0 <= quality <= C.TRACK_QUALITY_MAX):
        add(f'tracked.trackQualityWrapper must be an integer from 0 to {C.TRACK_QUALITY_MAX}')
    attitude = _dig(entity, 'location', 'attitudeEnu')
    if attitude is not None:
        parts = [attitude.get(k) for k in 'xyzw'] if isinstance(attitude, dict) else []
        if len(parts) != 4 or not all(is_real_number(p) for p in parts):
            add('location.attitudeEnu must have numbers x, y, z, w')
        elif abs(math.sqrt(sum(p * p for p in parts)) - 1.0) > 1e-3:
            add('location.attitudeEnu is not a unit quaternion')
    return problems


def _check_time_text(where: str, value, add) -> Optional[datetime]:
    """Parse a time field; add a problem and return None when it is missing or not UTC text with Z."""
    if not isinstance(value, str):
        add(f'{where} is required and must be UTC text like 2026-10-24T15:30:00.000000Z')
        return None
    try:
        return parse_utc(value)
    except ValueError:
        add(f'{where} {value[:40]!r} is not UTC ISO-8601 text ending in Z')
        return None


def validate_entity(entity, now: datetime) -> None:
    """Raise EntityValidationError (a ValueError) listing every problem, or return None when it is valid."""
    problems = find_problems(entity, now)
    if problems:
        raise EntityValidationError(problems)
