#!/usr/bin/env python3
"""Pure geometry for GPS direct georeferencing.

Frames used throughout:
  - Geodetic:  latitude / longitude in degrees, altitude in metres AGL.
  - Local ENU: metres east (x) / north (y) of a reference lat/lon. The
    mapping area is small (hundreds of metres), so a flat-earth
    approximation is sub-centimetre accurate for RELATIVE positions.
  - Camera:    standard CV frame — x = image right, y = image down,
    z = optical axis (forward).

Angle conventions (match MAVROS / SIYI):
  - yaw    : compass heading, degrees CLOCKWISE from true north.
  - pitch  : 0 = level forward, -90 = straight down (nadir).
  - roll   : about the forward axis, right-wing-down positive.

No ROS or OpenCV imports here — numpy only, unit-testable anywhere.
"""

import math
from typing import Optional, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Geodetic <-> local ENU
# ---------------------------------------------------------------------------

def meters_per_degree(lat_deg: float) -> Tuple[float, float]:
    """(metres per degree latitude, metres per degree longitude) at lat_deg.

    Standard series expansion of the WGS-84 ellipsoid — accurate to well
    under 0.1% which is far below GPS error.
    """
    lat = math.radians(lat_deg)
    m_per_deg_lat = (111132.954
                     - 559.822 * math.cos(2.0 * lat)
                     + 1.175 * math.cos(4.0 * lat))
    m_per_deg_lon = (111412.84 * math.cos(lat)
                     - 93.5 * math.cos(3.0 * lat)
                     + 0.118 * math.cos(5.0 * lat))
    return m_per_deg_lat, m_per_deg_lon


def latlon_to_enu(lat: float, lon: float,
                  ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """Project (lat, lon) to metres (east, north) of (ref_lat, ref_lon)."""
    m_lat, m_lon = meters_per_degree(ref_lat)
    east = (lon - ref_lon) * m_lon
    north = (lat - ref_lat) * m_lat
    return east, north


def enu_to_latlon(east: float, north: float,
                  ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """Inverse of latlon_to_enu."""
    m_lat, m_lon = meters_per_degree(ref_lat)
    lat = ref_lat + north / m_lat
    lon = ref_lon + east / m_lon
    return lat, lon


def bearing_deg(lat1: float, lon1: float,
                lat2: float, lon2: float) -> float:
    """Compass bearing (deg CW from north, [0, 360)) from point 1 to point 2.

    Flat-earth version — plenty for deriving heading between consecutive
    photos a few metres apart.
    """
    east, north = latlon_to_enu(lat2, lon2, lat1, lon1)
    return math.degrees(math.atan2(east, north)) % 360.0


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Flat-earth distance in metres between two lat/lon points."""
    east, north = latlon_to_enu(lat2, lon2, lat1, lon1)
    return math.hypot(east, north)


# ---------------------------------------------------------------------------
# Camera orientation
# ---------------------------------------------------------------------------

def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, c, -s],
                     [0.0, s, c]])


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s],
                     [0.0, 1.0, 0.0],
                     [-s, 0.0, c]])


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0],
                     [s, c, 0.0],
                     [0.0, 0.0, 1.0]])


# Camera CV axes expressed in aerospace body axes (x fwd, y right, z down):
#   cam x (image right)  = body y
#   cam y (image down)   = body z
#   cam z (optical axis) = body x
_R_BODY_FROM_CAM = np.array([[0.0, 0.0, 1.0],
                             [1.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0]])

# NED (north, east, down) -> ENU (east, north, up)
_R_ENU_FROM_NED = np.array([[0.0, 1.0, 0.0],
                            [1.0, 0.0, 0.0],
                            [0.0, 0.0, -1.0]])


def rotation_enu_from_camera(yaw_deg: float, pitch_deg: float,
                             roll_deg: float) -> np.ndarray:
    """3x3 rotation taking camera-frame vectors into the local ENU frame.

    Sanity anchor: nadir (pitch=-90) at yaw=0 maps image right -> east,
    image down -> south, optical axis -> straight down (image top = north).
    """
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    # Aerospace ZYX: body -> NED. Rz is about the NED down axis, so a
    # positive yaw is clockwise from north — exactly a compass heading.
    r_ned_from_body = _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)
    return _R_ENU_FROM_NED @ r_ned_from_body @ _R_BODY_FROM_CAM


def camera_intrinsics(width: int, height: int,
                      hfov_deg: float) -> Tuple[float, float, float, float]:
    """(fx, fy, cx, cy) in pixels from image size + horizontal FOV.

    Square pixels assumed (fy = fx); VFOV follows from the aspect ratio.
    """
    fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    fy = fx
    cx = (width - 1) / 2.0
    cy = (height - 1) / 2.0
    return fx, fy, cx, cy


# ---------------------------------------------------------------------------
# Ground projection
# ---------------------------------------------------------------------------

def ground_footprint(east: float, north: float, alt_agl: float,
                     yaw_deg: float, pitch_deg: float, roll_deg: float,
                     width: int, height: int, hfov_deg: float,
                     min_down_component: float = 0.17,
                     max_span_factor: float = 8.0) -> Optional[np.ndarray]:
    """Project the 4 image corners onto the ground plane (z = 0).

    Returns a 4x2 array of (east, north) metres, ordered like the image
    corners: top-left, top-right, bottom-right, bottom-left. Returns None
    when the view is too oblique to map (a corner ray points less than
    ~10 deg below the horizon, or the footprint blows up beyond
    alt * max_span_factor).
    """
    if alt_agl <= 0.0:
        return None

    fx, fy, cx, cy = camera_intrinsics(width, height, hfov_deg)
    rot = rotation_enu_from_camera(yaw_deg, pitch_deg, roll_deg)

    corners_px = ((0, 0), (width - 1, 0), (width - 1, height - 1), (0, height - 1))
    points = []
    for u, v in corners_px:
        d_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        d = rot @ d_cam
        d = d / np.linalg.norm(d)
        if d[2] > -min_down_component:
            return None  # ray (nearly) misses the ground plane
        t = -alt_agl / d[2]
        points.append((east + t * d[0], north + t * d[1]))

    pts = np.array(points)
    span = max(pts[:, 0].max() - pts[:, 0].min(),
               pts[:, 1].max() - pts[:, 1].min())
    if span > alt_agl * max_span_factor:
        return None
    return pts


def nadir_gsd_m(alt_agl: float, width: int, hfov_deg: float) -> float:
    """Native ground sample distance (m/px) of a nadir shot at alt_agl."""
    ground_width = 2.0 * alt_agl * math.tan(math.radians(hfov_deg) / 2.0)
    return ground_width / float(width)
