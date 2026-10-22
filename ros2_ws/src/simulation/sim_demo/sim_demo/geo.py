"""
sim_demo/geo.py -- PURPOSE: convert between GPS coordinates (latitude/longitude in degrees)
and the Gazebo world's flat map in metres (x = east, y = north, z = up), and measure
distances between GPS points.

WHY: the drone reports where it is in GPS degrees (MAVROS /mavros/global_position/global),
the targets' answer key may be written in either GPS or metres, and RViz draws in metres.
The Gazebo world's <spherical_coordinates> origin is the same point as the SITL --home
(the simulation contract requires it), so "metres east/north of home" ARE Gazebo world
coordinates.

TWO CONVENTIONS (they differ by up to ~1 m inside a SUAS field):
  * 'ardupilot' (DEFAULT): the exact formulas ArduPilot SITL uses to turn the drone's Gazebo
    position into its simulated GPS (AP_Common/Location.cpp offset()/get_distance_NE(): a
    sphere of radius 6378100 m, longitude scaled by the cosine of the mid latitude). MAVROS
    GPS, the photo file names and the team's drop waypoints all live in this convention, and
    sim_worlds writes its missions and truth files with it (sim_worlds/geo.py).
  * 'wgs84': the real ellipsoid (what a Gazebo NavSat sensor reports), as a local tangent
    plane with the two WGS-84 curvature radii (millimetre-accurate over a SUAS field).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# WGS-84 ellipsoid (the GPS Earth model).
WGS84_A = 6378137.0                      # equatorial radius [m]
WGS84_F = 1.0 / 298.257223563            # flattening
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)     # first eccentricity squared
MEAN_EARTH_RADIUS = 6371008.8            # [m], used only by haversine_m
# ArduPilot: metres per 1e-7 degree of latitude (LOCATION_SCALING_FACTOR) -> metres per degree.
ARDUPILOT_M_PER_DEG = 0.011131884502145034 * 1.0e7


def _ap_lon_scale(lat_deg: float) -> float:
    """ArduPilot's Location::longitude_scale(): cos(latitude), never below 0.01."""
    return max(math.cos(math.radians(lat_deg)), 0.01)


def curvature_radii(lat_deg: float) -> tuple[float, float]:
    """Return the Earth's two radii of curvature at a latitude.

    Args:
        lat_deg: latitude in degrees.
    Returns:
        (meridian_radius_m, prime_vertical_radius_m): metres per radian of latitude change
        (north-south) and the radius that, times cos(latitude), gives metres per radian of
        longitude change (east-west).
    """
    s = math.sin(math.radians(lat_deg))
    w = math.sqrt(1.0 - WGS84_E2 * s * s)
    prime_vertical = WGS84_A / w
    meridian = WGS84_A * (1.0 - WGS84_E2) / (w ** 3)
    return meridian, prime_vertical


@dataclass(frozen=True)
class GeoOrigin:
    """The GPS point that is (0, 0, 0) in the Gazebo world (= SITL home).

    Attributes:
        lat: latitude [deg]
        lon: longitude [deg]
        alt: altitude above mean sea level [m] (only used for the z/up conversion)
        convention: 'ardupilot' (default, matches the drone's GPS) or 'wgs84' (module docstring).
    """

    lat: float
    lon: float
    alt: float = 0.0
    convention: str = 'ardupilot'

    def to_enu(self, lat: float, lon: float, alt: float | None = None) -> tuple[float, float, float]:
        """Convert a GPS point to metres east, north and up of this origin.

        Args:
            lat, lon: degrees. alt: metres above sea level (None -> up = 0).
        Returns:
            (east_m, north_m, up_m)
        """
        up = 0.0 if alt is None else float(alt) - self.alt
        if self.convention == 'wgs84':
            meridian, prime = curvature_radii(self.lat)
            north = math.radians(lat - self.lat) * meridian
            east = math.radians(lon - self.lon) * prime * math.cos(math.radians(self.lat))
            return east, north, up
        north = (lat - self.lat) * ARDUPILOT_M_PER_DEG
        dlon = (lon - self.lon + 540.0) % 360.0 - 180.0
        east = dlon * ARDUPILOT_M_PER_DEG * _ap_lon_scale((self.lat + lat) / 2.0)
        return east, north, up

    def to_geo(self, east: float, north: float, up: float = 0.0) -> tuple[float, float, float]:
        """Convert metres east/north/up of this origin back to GPS (inverse of to_enu).

        Args:
            east, north, up: metres.
        Returns:
            (lat_deg, lon_deg, alt_m_msl)
        """
        if self.convention == 'wgs84':
            meridian, prime = curvature_radii(self.lat)
            lat = self.lat + math.degrees(north / meridian)
            lon = self.lon + math.degrees(east / (prime * math.cos(math.radians(self.lat))))
            return lat, lon, self.alt + up
        dlat = north / ARDUPILOT_M_PER_DEG
        lon = self.lon + east / ARDUPILOT_M_PER_DEG / _ap_lon_scale(self.lat + dlat / 2.0)
        return self.lat + dlat, lon, self.alt + up


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance between two GPS points in metres (good to ~0.3 %).

    Args:
        lat1, lon1, lat2, lon2: degrees.
    Returns:
        distance in metres.
    """
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2.0 * MEAN_EARTH_RADIUS * math.asin(min(1.0, math.sqrt(a)))


def local_distance_m(origin: GeoOrigin, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Distance in metres between two GPS points using the flat map around `origin`.

    More precise than haversine for the short distances of a SUAS field because it uses the
    ellipsoid radii at the field's latitude.
    """
    e1, n1, _ = origin.to_enu(lat1, lon1)
    e2, n2, _ = origin.to_enu(lat2, lon2)
    return math.hypot(e2 - e1, n2 - n1)


def parse_home(text: str) -> tuple[float, float, float, float]:
    """Parse the contract's home string "lat,lon,alt_msl,heading_deg" (SITL --home format).

    Missing trailing fields default to 0. Extra spaces are fine.

    Args:
        text: e.g. "38.3154,-76.5490,10.0,0".
    Returns:
        (lat_deg, lon_deg, alt_m, heading_deg)
    Raises:
        ValueError: if lat/lon are missing or not numbers.
    """
    parts = [p.strip() for p in str(text).split(',') if p.strip()]
    if len(parts) < 2:
        raise ValueError(f'home must look like "lat,lon,alt,heading", got {text!r}')
    values = [float(p) for p in parts[:4]]
    while len(values) < 4:
        values.append(0.0)
    return values[0], values[1], values[2], values[3]


def compass_to_enu_yaw_deg(compass_deg: float) -> float:
    """Compass heading (0 = north, clockwise) -> ENU yaw (0 = east, counter-clockwise), deg."""
    return wrap_deg(90.0 - compass_deg)


def enu_yaw_to_compass_deg(enu_yaw_deg: float) -> float:
    """ENU yaw (0 = east, counter-clockwise) -> compass heading (0 = north, clockwise), deg."""
    return wrap_deg_360(90.0 - enu_yaw_deg)


def wrap_deg(angle: float) -> float:
    """Wrap an angle to the range (-180, 180] degrees."""
    a = math.fmod(angle + 180.0, 360.0)
    if a <= 0.0:
        a += 360.0
    return a - 180.0


def wrap_deg_360(angle: float) -> float:
    """Wrap an angle to the range [0, 360) degrees."""
    a = math.fmod(angle, 360.0)
    return a + 360.0 if a < 0 else a


def quaternion_to_yaw_deg(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about +z, counter-clockwise, degrees) of a quaternion (ENU frames)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))
