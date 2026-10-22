"""
Work out where on the ground a detected object is, from its box in the photo.

target_geolocation.py -- PURPOSE: turn the pixel position of a detection box into latitude /
longitude. main_controller.py uses it to aim the payload drops.

Why (October 2026): main_controller used to fly to the drone's own GPS position at the moment the
photo was taken. A target near the edge of a photo is then up to half a picture away: in the
simulator the drops missed by 14-25 m. Now the box centre is projected onto the ground.

The idea, for the normal survey case (camera looking straight down):

        top of the photo = the direction the camera faces (the drone's nose in the simulator)
      +-------------------------------------+
      |                                     |
      |               X  box centre (u, v)  |   (du, dv) = pixels from the photo centre
      |                  +  photo centre    |   on flat ground that is
      |                                     |       du * H / f  metres to the photo's right
      +-------------------------------------+       dv * H / f  metres to the photo's bottom
                                                H = camera height above the ground
                                                f = focal length in pixels
                                                  = (photo width / 2) / tan(horizontal FOV / 2)
    then those metres are turned to east/north with the compass heading of the photo's top
    and added to the drone's GPS position. A tilted camera is handled the same way with a 3D ray.

Inputs come from the photo's "sidecar" (a JSON file video_cam writes next to every photo, format
in video_cam/photo_metadata.py): drone position, height above home and heading at the shutter,
camera field of view, zoom, gimbal angles and how the photo is turned relative to the drone.

Conventions (used everywhere in this file):
  * compass headings in degrees: 0 = north, 90 = east, clockwise;
  * camera pitch in the SIYI sign: 0 = horizon, -90 = straight down;
  * gimbal yaw in the SIYI sign: + = the camera turned LEFT of the nose;
  * pixels in OpenCV order: (0, 0) = top-left corner, u to the right, v down;
  * ground offsets in metres east / north of the point below the camera;
  * latitude/longitude <-> metres with ArduPilot's own flat-earth formula (AP_Common
    Location::offset / get_distance_NE), so the result is exactly what the autopilot flies to.

Pure Python (math, json, os, struct only): no ROS. Unit tests: ../test/test_target_geolocation.py
    python3 -m pytest ros2_ws/src/main/test/test_target_geolocation.py -v
"""

from dataclasses import dataclass
import json
import math
import os
import struct
from typing import Optional, Tuple

# ArduPilot LOCATION_SCALING_FACTOR: metres per 1e-7 degree of latitude -> metres per degree.
METRES_PER_DEGREE = 0.011131884502145034 * 1.0e7

# The sidecar format this module understands (written by video_cam/photo_metadata.py).
SIDECAR_FORMAT = 'astra.photo_metadata.v1'
SIDECAR_EXTENSION = '.json'

# A reported gimbal pitch outside this range is not trusted (it would mean the camera looks at
# the sky or the horizon during a survey); the configured default pitch is used instead.
TRUSTED_PITCH_RANGE_DEG = (-120.0, -30.0)


# ---------------------------------------------------------------------------- small helpers
def wrap_360(angle_deg: float) -> float:
    """Wrap an angle to [0, 360) degrees."""
    a = math.fmod(float(angle_deg), 360.0)
    return a + 360.0 if a < 0 else a


def focal_length_px(width_px: float, hfov_deg: float, zoom: float = 1.0) -> float:
    """
    Compute the focal length in pixels of a picture `width_px` wide.

    Zoom narrows the view: tan(hfov_zoomed / 2) = tan(hfov / 2) / zoom (optical or digital).
    """
    z = max(1e-6, float(zoom))
    return (float(width_px) / 2.0) / (math.tan(math.radians(hfov_deg) / 2.0) / z)


def offset_to_latlon(lat: float, lon: float, east_m: float,
                     north_m: float) -> Tuple[float, float]:
    """Move a GPS point by east/north metres (ArduPilot's Location::offset formula)."""
    dlat = north_m / METRES_PER_DEGREE
    mid_lat = lat + dlat / 2.0
    scale = max(math.cos(math.radians(mid_lat)), 0.01)
    return lat + dlat, lon + east_m / (METRES_PER_DEGREE * scale)


def latlon_to_offset(lat0: float, lon0: float, lat: float, lon: float) -> Tuple[float, float]:
    """Return the east/north metres from (lat0, lon0) to (lat, lon)."""
    north = (lat - lat0) * METRES_PER_DEGREE
    dlon = (lon - lon0 + 540.0) % 360.0 - 180.0
    scale = max(math.cos(math.radians((lat0 + lat) / 2.0)), 0.01)
    return dlon * METRES_PER_DEGREE * scale, north


def compass_name(east_m: float, north_m: float) -> str:
    """Name the direction of a ground offset ('north', 'south-east', ...) for log messages."""
    names = ['north', 'north-east', 'east', 'south-east', 'south', 'south-west', 'west',
             'north-west']
    bearing = wrap_360(math.degrees(math.atan2(east_m, north_m)))
    return names[int((bearing + 22.5) // 45.0) % 8]


# ---------------------------------------------------------------------------- the projection
def pixel_to_ground_offset(u: float, v: float, image_width: int, image_height: int,
                           hfov_deg: float, height_m: float, camera_heading_deg: float,
                           camera_pitch_deg: float = -90.0,
                           image_top_direction_deg: float = 0.0, zoom: float = 1.0,
                           max_range_m: float = 1000.0) -> Optional[Tuple[float, float]]:
    """
    Find where on flat ground pixel (u, v) of a photo is, relative to the point below the camera.

    Parameters
    ----------
    u, v
        Pixel position in the saved photo (e.g. a detection box centre).
    image_width, image_height
        Size of the saved photo in pixels.
    hfov_deg
        Horizontal field of view of the camera's natural picture at zoom 1.
    height_m
        Camera height above the ground under the target, metres (> 0).
    camera_heading_deg
        Compass direction the camera faces (for a straight-down camera: where the top of its
        natural picture points; = drone heading when the gimbal yaw is centred).
    camera_pitch_deg
        -90 = straight down, -60 = tilted 30 deg forward, 0 = horizon.
    image_top_direction_deg
        How far the SAVED photo is turned clockwise relative to the camera's natural picture:
        0 (simulator), 90, 180 (e.g. rotate_180) or 270.
    zoom
        Zoom factor at the shutter (1 = widest).
    max_range_m
        Farther answers are refused (a pixel near the horizon).

    Returns
    -------
    tuple or None
        (east_m, north_m), or None if the pixel looks at the sky/horizon, is farther than
        max_range_m, or an input is impossible (zero size, height <= 0, bad field of view).

    """
    if image_width <= 0 or image_height <= 0 or height_m <= 0.0:
        return None
    if not 0.0 < hfov_deg < 180.0:
        return None

    # 1. Turn the pixel offset back into the camera's natural picture (top = where it faces).
    alpha = math.radians(image_top_direction_deg)
    ca, sa = math.cos(alpha), math.sin(alpha)
    du_saved = float(u) - image_width / 2.0
    dv_saved = float(v) - image_height / 2.0
    du = ca * du_saved - sa * dv_saved
    dv = sa * du_saved + ca * dv_saved
    # The natural picture's width is the saved width, or the saved height when turned sideways.
    natural_width = image_width if abs(ca) >= abs(sa) else image_height
    f = focal_length_px(natural_width, hfov_deg, zoom)

    # 2. The camera's axes in east/north/up (no roll: the gimbal keeps the picture level).
    psi = math.radians(camera_heading_deg)
    theta = math.radians(camera_pitch_deg)
    forward = (math.cos(theta) * math.sin(psi), math.cos(theta) * math.cos(psi),
               math.sin(theta))
    image_up = (-math.sin(theta) * math.sin(psi), -math.sin(theta) * math.cos(psi),
                math.cos(theta))
    image_down = (-image_up[0], -image_up[1], -image_up[2])
    # right = down x forward
    right = (image_down[1] * forward[2] - image_down[2] * forward[1],
             image_down[2] * forward[0] - image_down[0] * forward[2],
             image_down[0] * forward[1] - image_down[1] * forward[0])

    # 3. The ray through the pixel, and where it meets the ground (height_m below the camera).
    a, b = du / f, dv / f
    ray = tuple(right[i] * a + image_down[i] * b + forward[i] for i in range(3))
    if ray[2] >= -1e-9:
        return None                                  # sky or horizon
    t = height_m / -ray[2]
    east, north = t * ray[0], t * ray[1]
    if math.hypot(east, north) > max_range_m:
        return None
    return east, north


# ---------------------------------------------------------------------------- files
def jpeg_size(path: str) -> Optional[Tuple[int, int]]:
    """Read (width, height) of a JPEG file from its header (no decoding); None if not a JPEG."""
    try:
        with open(path, 'rb') as f:
            if f.read(2) != b'\xff\xd8':
                return None
            while True:
                byte = f.read(1)
                while byte and byte != b'\xff':
                    byte = f.read(1)
                while byte == b'\xff':               # fill bytes before a marker
                    byte = f.read(1)
                if not byte:
                    return None
                marker = byte[0]
                if marker == 0x01 or 0xD0 <= marker <= 0xD8:
                    continue                         # markers without a length field
                if marker in (0xD9, 0xDA):
                    return None                      # end of image / scan before a frame header
                length_bytes = f.read(2)
                if len(length_bytes) < 2:
                    return None
                length = struct.unpack('>H', length_bytes)[0]
                if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
                    data = f.read(5)                 # precision, height, width
                    if len(data) < 5:
                        return None
                    _precision, height, width = struct.unpack('>BHH', data)
                    return int(width), int(height)
                f.seek(length - 2, 1)
    except OSError:
        return None


def sidecar_path_for(image_path: str) -> str:
    """Return the sidecar path of a photo: same folder and name, extension .json."""
    return os.path.splitext(image_path)[0] + SIDECAR_EXTENSION


def read_photo_sidecar(image_path: str) -> Optional[dict]:
    """Read the sidecar of a photo; None if it is missing, unreadable or another format."""
    try:
        with open(sidecar_path_for(image_path), encoding='utf-8') as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None
    if not isinstance(data, dict) or data.get('format') != SIDECAR_FORMAT:
        return None
    return data


def _num(d: Optional[dict], *keys) -> Optional[float]:
    """Return d[keys[0]][keys[1]]... as a float, or None if missing / not a finite number."""
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return None
        cur = cur[k]
    try:
        x = float(cur)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def photo_position(meta: Optional[dict], name_lat: Optional[float],
                   name_lon: Optional[float]) -> Optional[Tuple[float, float]]:
    """
    Return the drone's position for a photo: the sidecar's exact fix, else the file name's.

    (0, 0) means "no GPS fix" (the FCU reports it before a lock) and is refused, like video_cam
    does.
    """
    lat, lon = _num(meta, 'position', 'lat'), _num(meta, 'position', 'lon')
    if lat is None or lon is None:
        lat, lon = name_lat, name_lon
    if lat is None or lon is None:
        return None
    if abs(lat) < 1e-9 and abs(lon) < 1e-9:
        return None
    return float(lat), float(lon)


# ---------------------------------------------------------------------------- the whole step
@dataclass
class AimResult:
    """
    Where main_controller should fly for one detection.

    lat, lon: the aim point (degrees). method: 'box' (projected from the box centre) or 'photo'
    (the photo's own position). east_m, north_m: aim point minus the photo position, metres
    (0, 0 for 'photo'). note: one short line for the log saying how it was computed or why it
    fell back.
    """

    lat: float
    lon: float
    method: str
    east_m: float = 0.0
    north_m: float = 0.0
    note: str = ''

    @property
    def offset_m(self) -> float:
        """Return the distance between the aim point and the photo position, metres."""
        return math.hypot(self.east_m, self.north_m)


def aim_at_box(meta: Optional[dict], photo_lat: float, photo_lon: float, u: float, v: float,
               image_size: Optional[Tuple[int, int]], default_pitch_deg: float = -90.0,
               default_hfov_deg: float = 81.0, ground_height_m: float = 0.0,
               max_offset_m: float = 150.0, min_height_m: float = 3.0) -> AimResult:
    """
    Compute the aim point for a detection box centred at pixel (u, v) of a photo.

    Falls back to the photo's own position (the old behaviour) when the photo cannot be
    projected: no sidecar, no height or heading recorded, zoom changing at the shutter, unknown
    image size, or an answer that is implausibly far away. The reason is in AimResult.note.

    Parameters
    ----------
    meta
        The photo's sidecar dict (read_photo_sidecar) or None.
    photo_lat, photo_lon
        The drone's position for the photo (photo_position()).
    u, v
        Box centre in pixels of the saved photo.
    image_size
        (width, height) of the saved photo (jpeg_size()) or None.
    default_pitch_deg
        Camera pitch when the camera did not report its attitude.
    default_hfov_deg
        Field of view when the sidecar has none.
    ground_height_m
        Height of the ground under the targets above home (rel_alt is above home).
    max_offset_m
        Answers farther than this from the photo position are not trusted.
    min_height_m
        Below this height above the ground no projection is attempted.

    Returns
    -------
    AimResult
        The aim point and how it was found.

    """
    def fallback(why: str) -> AimResult:
        return AimResult(photo_lat, photo_lon, 'photo', note=f'photo position ({why})')

    if meta is None:
        return fallback('no sidecar for this photo')
    if not image_size:
        return fallback('photo size unknown')
    rel_alt = _num(meta, 'rel_alt_m')
    heading = _num(meta, 'heading_deg')
    if rel_alt is None:
        return fallback('no height recorded at the shutter')
    if heading is None:
        return fallback('no heading recorded at the shutter')
    height = rel_alt - float(ground_height_m)
    if height < min_height_m:
        return fallback(f'only {height:.1f} m above the ground')
    camera = meta.get('camera') if isinstance(meta.get('camera'), dict) else {}
    if camera.get('zoom_changing'):
        return fallback('the zoom was moving at the shutter')
    hfov = _num(camera, 'hfov_deg') or float(default_hfov_deg)
    zoom = _num(camera, 'zoom_x') or 1.0
    top_dir = _num(camera, 'image_top_direction_deg') or 0.0

    pitch, gimbal_yaw, attitude_note = float(default_pitch_deg), 0.0, 'pitch assumed'
    gimbal = meta.get('gimbal') if isinstance(meta.get('gimbal'), dict) else None
    g_pitch, g_yaw = _num(gimbal, 'pitch_deg'), _num(gimbal, 'yaw_deg')
    low, high = TRUSTED_PITCH_RANGE_DEG
    if g_pitch is not None and low <= g_pitch <= high:
        pitch, attitude_note = g_pitch, 'gimbal reported'
        if g_yaw is not None:
            gimbal_yaw = g_yaw
    elif g_pitch is not None:
        attitude_note = f'gimbal said pitch {g_pitch:.0f} deg, not trusted'
    # SIYI yaw + = camera turned LEFT, i.e. counter-clockwise = smaller compass heading.
    camera_heading = wrap_360(heading - gimbal_yaw)

    offset = pixel_to_ground_offset(u, v, image_size[0], image_size[1], hfov, height,
                                    camera_heading, pitch, top_dir, zoom,
                                    max_range_m=max_offset_m)
    if offset is None:
        return fallback('the box does not project onto the ground within '
                        f'{max_offset_m:.0f} m')
    east, north = offset
    lat, lon = offset_to_latlon(photo_lat, photo_lon, east, north)
    note = (f'box centre, {math.hypot(east, north):.1f} m {compass_name(east, north)} of the '
            f'photo centre (height {height:.1f} m, heading {camera_heading:.0f}, pitch '
            f'{pitch:.0f} [{attitude_note}], FOV {hfov:.0f} deg, zoom {zoom:.1f}x, '
            f'photo top +{top_dir:.0f} deg)')
    return AimResult(lat, lon, 'box', east, north, note)
