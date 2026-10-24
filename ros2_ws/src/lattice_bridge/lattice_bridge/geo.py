"""
lattice_bridge/geo.py -- PURPOSE: the small amount of geometry the bridge needs, and the number checks every
other module shares: "is this a real, finite number in range?", "is this a believable latitude and
longitude?", distance between two points on the Earth, metres <-> degrees with the SAME flat-earth formula
ArduPilot uses (so the numbers agree with what the autopilot and main/src/target_geolocation.py compute),
the rules for a clean map polygon (closed, counter-clockwise, no self-crossing) and the rotation that turns a
compass heading into the quaternion Lattice expects.

Why checks live here: a NaN or infinity slips through ordinary comparisons (every comparison with NaN is
False, so `if x > 90` does not catch it) and then ends up in a published entity. Every function here uses
math.isfinite first, and `True` / `False` are NOT accepted as numbers.

Pure Python (math, numbers): no ROS, no I/O, no clock.
"""

from __future__ import annotations

import math
import numbers
from typing import List, Optional, Sequence, Tuple

from . import constants as C

LatLon = Tuple[float, float]

#: ArduPilot's LOCATION_SCALING_FACTOR in metres per degree of latitude (same value as target_geolocation.py).
METRES_PER_DEGREE = 0.011131884502145034 * 1.0e7
#: Mean Earth radius for the great-circle distance (metres).
EARTH_RADIUS_M = 6371008.8
#: A position this close to latitude 0 / longitude 0 is the "no GPS fix" value of MAVLink and ROS messages.
_NULL_ISLAND_DEG = 1e-7


def _short(value) -> str:
    """A short, safe text for error messages (a hostile value must not flood a log)."""
    return repr(value)[:40]


def is_real_number(value) -> bool:
    """True for an int/float that is finite. False for NaN, +-infinity, bool, str, None, huge ints."""
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, ValueError, TypeError):
        return False


def require_number(name: str, value, low: Optional[float] = None, high: Optional[float] = None) -> float:
    """Return `value` as a float, or raise ValueError saying which input is wrong and why."""
    if not is_real_number(value):
        raise ValueError(f'{name} must be a finite number, got {_short(value)}')
    number = float(value)
    if low is not None and number < low:
        raise ValueError(f'{name} must be >= {low}, got {number}')
    if high is not None and number > high:
        raise ValueError(f'{name} must be <= {high}, got {number}')
    return number


def require_lat_lon(lat, lon, name: str = 'position') -> LatLon:
    """Return (lat, lon) as floats; ValueError unless both are finite and in range.

    Latitude -90..90, longitude -180..180. A position at exactly (0, 0) is refused as well: it is what
    GLOBAL_POSITION_INT and NavSatFix report when there is no GPS fix, and nothing of ours is ever there.
    """
    latitude = require_number(f'{name} latitude', lat, -90.0, 90.0)
    longitude = require_number(f'{name} longitude', lon, -180.0, 180.0)
    if abs(latitude) < _NULL_ISLAND_DEG and abs(longitude) < _NULL_ISLAND_DEG:
        raise ValueError(f'{name} is (0, 0), which means "no GPS fix", not a place')
    return latitude, longitude


def wrap_lon(lon: float) -> float:
    """Bring a longitude into [-180, 180)."""
    return ((lon + 180.0) % 360.0) - 180.0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres between two points given in degrees."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = phi2 - phi1
    d_lambda = math.radians(lon2 - lon1)
    a = math.sin(d_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def _lon_scale(lat: float) -> float:
    """cos(latitude), never below 0.01 (ArduPilot's own floor, so the formula survives the poles)."""
    return max(math.cos(math.radians(lat)), 0.01)


def offset_latlon(lat: float, lon: float, east_m: float, north_m: float) -> LatLon:
    """The point `east_m` metres east and `north_m` metres north of (lat, lon) (ArduPilot flat-earth formula)."""
    new_lat = lat + north_m / METRES_PER_DEGREE
    new_lon = lon + east_m / (METRES_PER_DEGREE * _lon_scale(lat))
    return new_lat, wrap_lon(new_lon)


def latlon_to_enu(ref_lat: float, ref_lon: float, lat: float, lon: float) -> Tuple[float, float]:
    """(east, north) metres of (lat, lon) seen from (ref_lat, ref_lon); the inverse of offset_latlon."""
    delta_lon = ((lon - ref_lon + 180.0) % 360.0) - 180.0
    return (delta_lon * METRES_PER_DEGREE * _lon_scale(ref_lat), (lat - ref_lat) * METRES_PER_DEGREE)


# ---------------------------------------------------------------------------------------------------
# Heading -> quaternion
# ---------------------------------------------------------------------------------------------------

def heading_to_attitude_enu(heading_deg) -> dict:
    """Compass heading (0 = north, 90 = east, clockwise) -> Lattice `attitudeEnu` quaternion {x, y, z, w}.

    Lattice describes the body's turn in an ENU frame (x = east, y = north, z = up). A compass heading H is
    a turn about "up" of (90 - H) degrees counter-clockwise from east, so z = sin(yaw/2), w = cos(yaw/2).
    The yaw is folded into [-180, 180) first, so w is never negative and 360 gives exactly the same answer as 0.
    Check against the Lattice docs: flying north-east (H = 45) gives z = 0.3827, w = 0.9239, the numbers
    in their drone example. Anything outside 0..360 raises ValueError.
    """
    heading = require_number('heading_deg', heading_deg, 0.0, 360.0)
    yaw_deg = ((90.0 - heading + 180.0) % 360.0) - 180.0
    yaw = math.radians(yaw_deg)
    # "+ 0.0" turns a possible -0.0 into 0.0 so the JSON never contains "-0.0".
    return {'x': 0.0, 'y': 0.0, 'z': round(math.sin(yaw / 2.0), 6) + 0.0,
            'w': round(math.cos(yaw / 2.0), 6) + 0.0}


# ---------------------------------------------------------------------------------------------------
# Polygons
# ---------------------------------------------------------------------------------------------------

def ring_signed_area(points: Sequence[LatLon]) -> float:
    """Signed area of a ring in square degrees (positive = counter-clockwise on a map with north up).

    Only the sign and "is it zero" matter, so plain degrees are good enough; longitude is shrunk by
    cos(latitude) so a long thin ring is not mis-judged.
    """
    if len(points) < 3:
        return 0.0
    scale = _lon_scale(sum(p[0] for p in points) / len(points))
    total = 0.0
    for index, (lat1, lon1) in enumerate(points):
        lat2, lon2 = points[(index + 1) % len(points)]
        total += (lon1 * scale) * lat2 - (lon2 * scale) * lat1
    return total / 2.0


def _orient(a: LatLon, b: LatLon, c: LatLon) -> float:
    """> 0 when a->b->c turns left (counter-clockwise), < 0 right, 0 when the three points are on a line."""
    return (b[1] - a[1]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[1] - a[1])


def _on_segment(a: LatLon, b: LatLon, p: LatLon) -> bool:
    """True when p (known to be on the line a-b) lies between a and b."""
    return (min(a[0], b[0]) <= p[0] <= max(a[0], b[0])) and (min(a[1], b[1]) <= p[1] <= max(a[1], b[1]))


def segments_intersect(a: LatLon, b: LatLon, c: LatLon, d: LatLon) -> bool:
    """True when the segment a-b touches or crosses the segment c-d."""
    o1, o2 = _orient(a, b, c), _orient(a, b, d)
    o3, o4 = _orient(c, d, a), _orient(c, d, b)
    if ((o1 > 0 > o2) or (o1 < 0 < o2)) and ((o3 > 0 > o4) or (o3 < 0 < o4)):
        return True
    return ((o1 == 0 and _on_segment(a, b, c)) or (o2 == 0 and _on_segment(a, b, d))
            or (o3 == 0 and _on_segment(c, d, a)) or (o4 == 0 and _on_segment(c, d, b)))


def is_simple_ring(points: Sequence[LatLon]) -> bool:
    """True when no two edges of the (open) ring cross or touch, except neighbours at their shared corner."""
    count = len(points)
    for i in range(count):
        a, b = points[i], points[(i + 1) % count]
        for j in range(i + 1, count):
            if j == i or (j + 1) % count == i or (i + 1) % count == j:
                continue                      # neighbouring edges share a corner by definition
            c, d = points[j], points[(j + 1) % count]
            if segments_intersect(a, b, c, d):
                return False
    return True


def clean_polygon_ring(points: Sequence[Sequence[float]], name: str = 'polygon') -> List[LatLon]:
    """Turn a list of (lat, lon) corners into the ring Lattice wants: closed, counter-clockwise, checked.

    Accepts the ring open or already closed (first == last). Raises ValueError for fewer than 3 different
    corners, more than C.MAX_POLYGON_VERTICES, a bad number or (0, 0) anywhere, repeated neighbouring
    corners, zero area, a ring that crosses itself or spans 180 degrees of longitude or more.
    Lattice's reference asks for "canonical" polygons and points to RFC 7946, whose rule is that the outer
    ring runs counter-clockwise, so that is the order returned. The first corner is kept as the first.
    """
    try:
        raw = list(points)
    except TypeError:
        raise ValueError(f'{name} must be a list of (lat, lon) corners') from None
    ring: List[LatLon] = []
    for index, corner in enumerate(raw):
        try:
            lat, lon = corner
        except (TypeError, ValueError):
            raise ValueError(f'{name} corner {index} must be a (lat, lon) pair') from None
        ring.append(require_lat_lon(lat, lon, f'{name} corner {index}'))
    if len(ring) >= 2 and ring[0] == ring[-1]:
        ring.pop()                                  # drop the closing corner; it is added back below
    if len(ring) < C.MIN_POLYGON_VERTICES:
        raise ValueError(f'{name} needs at least {C.MIN_POLYGON_VERTICES} different corners, got {len(ring)}')
    if len(ring) > C.MAX_POLYGON_VERTICES:
        raise ValueError(f'{name} has {len(ring)} corners; the limit is {C.MAX_POLYGON_VERTICES}')
    for index, corner in enumerate(ring):
        if corner == ring[(index + 1) % len(ring)]:
            raise ValueError(f'{name} corners {index} and {(index + 1) % len(ring)} are the same point')
    lons = [p[1] for p in ring]
    if max(lons) - min(lons) >= 180.0:
        raise ValueError(f'{name} spans 180 degrees of longitude or more (antimeridian rings are not supported)')
    area = ring_signed_area(ring)
    if area == 0.0:
        raise ValueError(f'{name} has zero area (all corners on one line)')
    if not is_simple_ring(ring):
        raise ValueError(f'{name} crosses itself')
    if area < 0.0:                                  # clockwise: reverse, keeping the first corner first
        ring = [ring[0]] + ring[:0:-1]
    return ring + [ring[0]]
