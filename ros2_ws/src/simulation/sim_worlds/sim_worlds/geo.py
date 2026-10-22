"""
sim_worlds/geo.py -- PURPOSE: convert between GPS coordinates (latitude/longitude in degrees) and the local
metres Gazebo uses, plus a few small polygon helpers.

Local frame (the same in every world of this package):
  ENU = East, North, Up.  x = metres EAST of the world origin, y = metres NORTH, z = metres UP.
  The world origin is the "home" point: the centre of the take-off pad, at ground level (z = 0).

Why there are TWO conventions for "metres -> degrees":
  * 'ardupilot' (the DEFAULT): the exact formulas ArduPilot uses inside SITL (Location::offset and
    Location::get_distance_NE in libraries/AP_Common/Location.cpp: a sphere of radius 6378100 m, longitude
    scaled with the cosine of the mid-point latitude). SITL turns the Gazebo position of the drone into its
    simulated GPS with these formulas, so the drone's GPS, MAVROS /mavros/global_position/global, the photo
    file names "<lat> , <lon>.jpg" and the team's drop waypoints all live in this convention. Missions and
    truth files therefore use it: a waypoint written with it is flown to exactly the Gazebo spot we meant.
  * 'wgs84': the real WGS84 ellipsoid, exactly what a Gazebo NavSat sensor (and a real GPS) reports.
Inside our worlds (less than 1 km from the origin) the two differ by at most about 1 metre.
"""

import math
from dataclasses import dataclass

# --- ArduPilot constants (libraries/AP_Common/Location.cpp, AP_Math/definitions.h) -------------------------
#: metres per 1e-7 degree of latitude (= 2 * pi * 6378100 / 360 / 1e7). ArduPilot's LOCATION_SCALING_FACTOR.
LOCATION_SCALING_FACTOR = 0.011131884502145034
#: metres per degree of latitude in ArduPilot's spherical model.
ARDUPILOT_M_PER_DEG = LOCATION_SCALING_FACTOR * 1.0e7

# --- WGS84 ellipsoid (what Gazebo's <surface_model>EARTH_WGS84</surface_model> uses) ------------------------
WGS84_A = 6378137.0                    # semi-major axis [m]
WGS84_F = 1.0 / 298.257223563          # flattening
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)   # first eccentricity squared


@dataclass(frozen=True)
class Home:
    """The world origin as a GPS position.

    lat, lon: degrees (north and east positive). alt: metres above mean sea level of the ground at the
    origin (z = 0 in Gazebo). heading: degrees, the initial heading SITL is told (0 = north).
    The same four numbers go into SITL's --home argument and into the world's <spherical_coordinates>.
    """

    lat: float
    lon: float
    alt: float
    heading: float = 0.0

    @classmethod
    def parse(cls, text):
        """Read a "lat,lon,alt_msl,heading_deg" string (the SITL --home format). Raises ValueError if bad."""
        parts = [p.strip() for p in str(text).split(',')]
        if len(parts) not in (3, 4):
            raise ValueError(f'home "{text}" must be "lat,lon,alt_msl[,heading_deg]"')
        vals = [float(p) for p in parts]
        lat, lon, alt = vals[:3]
        heading = vals[3] if len(vals) == 4 else 0.0
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            raise ValueError(f'home "{text}": latitude/longitude out of range')
        return cls(lat, lon, alt, heading)

    def sitl_string(self):
        """The string for SITL's --home option, e.g. "36.2143000,-96.0068000,210.0,0"."""
        return f'{self.lat:.7f},{self.lon:.7f},{self.alt:.1f},{self.heading:g}'


def _longitude_scale(lat_deg):
    """ArduPilot's Location::longitude_scale(): cos(latitude), never below 0.01."""
    return max(math.cos(math.radians(lat_deg)), 0.01)


def ardupilot_offset(lat, lon, north_m, east_m):
    """Move a GPS point by (north_m, east_m) metres exactly like ArduPilot's Location::offset().

    Inputs: lat, lon in degrees; north_m, east_m in metres. Output: (lat, lon) in degrees.
    (ArduPilot stores 1e-7 degree integers, so it may differ from this float result by up to ~1 cm.)
    """
    dlat = north_m / ARDUPILOT_M_PER_DEG
    dlon = east_m / ARDUPILOT_M_PER_DEG / _longitude_scale(lat + dlat / 2.0)
    return lat + dlat, lon + dlon


def ardupilot_distance_ne(lat1, lon1, lat2, lon2):
    """North and east metres from point 1 to point 2, exactly like ArduPilot's Location::get_distance_NE()."""
    north = (lat2 - lat1) * ARDUPILOT_M_PER_DEG
    dlon = (lon2 - lon1 + 540.0) % 360.0 - 180.0          # wrap to [-180, 180)
    east = dlon * ARDUPILOT_M_PER_DEG * _longitude_scale((lat1 + lat2) / 2.0)
    return north, east


def _geodetic_to_ecef(lat, lon, alt):
    """WGS84 latitude/longitude (deg) and height (m) -> Earth-centred Earth-fixed X, Y, Z (m)."""
    phi, lam = math.radians(lat), math.radians(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(phi) ** 2)
    x = (n + alt) * math.cos(phi) * math.cos(lam)
    y = (n + alt) * math.cos(phi) * math.sin(lam)
    z = (n * (1.0 - WGS84_E2) + alt) * math.sin(phi)
    return x, y, z


def _ecef_to_geodetic(x, y, z):
    """Earth-centred X, Y, Z (m) -> WGS84 latitude, longitude (deg) and height (m). Iterative, ~1 mm exact."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    alt = 0.0
    for _ in range(8):
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
        alt = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1.0 - WGS84_E2 * n / (n + alt)))
    return math.degrees(lat), math.degrees(lon), alt


def _enu_axes(lat, lon):
    """Unit vectors east, north, up (in ECEF) at a GPS point."""
    phi, lam = math.radians(lat), math.radians(lon)
    e = (-math.sin(lam), math.cos(lam), 0.0)
    n = (-math.sin(phi) * math.cos(lam), -math.sin(phi) * math.sin(lam), math.cos(phi))
    u = (math.cos(phi) * math.cos(lam), math.cos(phi) * math.sin(lam), math.sin(phi))
    return e, n, u


def enu_to_wgs84(x, y, z, home):
    """Local ENU metres -> exact WGS84 (lat deg, lon deg, alt m MSL), like a Gazebo NavSat sensor reports."""
    x0, y0, z0 = _geodetic_to_ecef(home.lat, home.lon, home.alt)
    e, n, u = _enu_axes(home.lat, home.lon)
    X = x0 + x * e[0] + y * n[0] + z * u[0]
    Y = y0 + x * e[1] + y * n[1] + z * u[1]
    Z = z0 + x * e[2] + y * n[2] + z * u[2]
    return _ecef_to_geodetic(X, Y, Z)


def wgs84_to_enu(lat, lon, alt, home):
    """Exact WGS84 (lat deg, lon deg, alt m MSL) -> local ENU metres (x east, y north, z up)."""
    X, Y, Z = _geodetic_to_ecef(lat, lon, alt)
    x0, y0, z0 = _geodetic_to_ecef(home.lat, home.lon, home.alt)
    dx, dy, dz = X - x0, Y - y0, Z - z0
    e, n, u = _enu_axes(home.lat, home.lon)
    return (dx * e[0] + dy * e[1] + dz * e[2],
            dx * n[0] + dy * n[1] + dz * n[2],
            dx * u[0] + dy * u[1] + dz * u[2])


def enu_to_latlon(x, y, home, convention='ardupilot'):
    """Local metres (x east, y north) -> (lat, lon) in degrees.

    convention: 'ardupilot' (default; what the drone's GPS and MAVROS report above that spot) or 'wgs84'
    (what a Gazebo NavSat sensor reports). See the module text for why both exist.
    """
    if convention == 'ardupilot':
        return ardupilot_offset(home.lat, home.lon, y, x)
    if convention == 'wgs84':
        lat, lon, _ = enu_to_wgs84(x, y, 0.0, home)
        return lat, lon
    raise ValueError(f'unknown convention "{convention}" (use "ardupilot" or "wgs84")')


def latlon_to_enu(lat, lon, home, convention='ardupilot'):
    """(lat, lon) in degrees -> local metres (x east, y north). Inverse of enu_to_latlon()."""
    if convention == 'ardupilot':
        north, east = ardupilot_distance_ne(home.lat, home.lon, lat, lon)
        return east, north
    if convention == 'wgs84':
        x, y, _ = wgs84_to_enu(lat, lon, home.alt, home)
        return x, y
    raise ValueError(f'unknown convention "{convention}" (use "ardupilot" or "wgs84")')


def heading_to_yaw(heading_deg):
    """Compass heading (degrees, 0 = north, 90 = east, clockwise) -> ENU yaw (radians, 0 = east, CCW)."""
    return math.radians(90.0 - heading_deg)


def yaw_to_heading(yaw_rad):
    """ENU yaw (radians, 0 = east, counter-clockwise) -> compass heading in degrees [0, 360)."""
    return (90.0 - math.degrees(yaw_rad)) % 360.0


def point_in_polygon(x, y, poly):
    """True when (x, y) is inside the polygon poly = [(x, y), ...] (even-odd ray casting)."""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y):
            x_cross = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < x_cross:
                inside = not inside
        j = i
    return inside


def distance_to_polygon_edge(x, y, poly):
    """Shortest distance (m) from (x, y) to the outline of the polygon poly = [(x, y), ...]."""
    best = float('inf')
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / l2))
        best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best


def inside_with_margin(x, y, poly, margin):
    """True when (x, y) is inside the polygon AND at least `margin` metres away from its outline."""
    return point_in_polygon(x, y, poly) and distance_to_polygon_edge(x, y, poly) >= margin
