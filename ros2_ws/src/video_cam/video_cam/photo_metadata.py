"""
The JSON "sidecar" file video_cam writes next to every photo.

video_cam/photo_metadata.py -- PURPOSE: write and read the small JSON file that records what the
drone and the camera were doing at the moment of the shutter:

    mapping_photos/36.214934 , -96.007241.jpg    <- the photo (name as always: "<lat> , <lon>.jpg")
    mapping_photos/36.214934 , -96.007241.json   <- its sidecar (this module)

main_controller reads it (main/src/target_geolocation.py) to turn the pixel position of a detected
target into latitude/longitude, so the payload is dropped on the target and not on the spot where
the drone happened to be when it took the photo (that missed by 14-25 m in the simulator, October
2026). The photo file name is NOT changed, so the detector (detection/geotag.py) and its tests are
not affected. Detectors and other tools that only look for image files ignore the .json files.

Format "astra.photo_metadata.v1" (every number may be null when it was not known):
    {
      "format": "astra.photo_metadata.v1",
      "image_name": "36.214934 , -96.007241.jpg",
      "shutter_unix_s": 1792208863.97,   wall-clock time the shutter command was sent
      "trigger_unix_s": 1792208863.90,   when the /camera/trigger that asked for it arrived
      "position": {"lat": 36.2149341, "lon": -96.0072412, "alt_msl_m": 240.1, "age_s": 0.4},
                                         drone GPS fix used for the name (full precision)
      "rel_alt_m": 30.0, "rel_alt_age_s": 0.4,     height above home (global_position/rel_alt)
      "heading_deg": 90.1, "heading_age_s": 0.4,   compass heading (global_position/compass_hdg)
      "camera": {"model": "ZR10", "hfov_deg": 81.0, "zoom_x": 1.0, "zoom_source": "readback",
                 "zoom_changing": false, "image_top_direction_deg": 0.0, "rotate_180": false},
      "gimbal": {"yaw_deg": 0.0, "pitch_deg": -90.0, "roll_deg": 0.0, "source": "camera 0x0D"}
                                         or null when the camera did not report its attitude
    }
  * image_top_direction_deg: where the TOP of the saved photo points, clockwise from the drone's
    nose, when the gimbal looks straight down with its yaw centred (siyi parameter
    image_top_direction_deg, +180 when rotate_180 is on). 0 = nose (simulator).
  * gimbal angles are the SIYI SDK's: yaw + = camera turned LEFT, pitch -90 = straight down.

Pure Python (json/os only): no ROS. Unit tests: ../test/test_photo_metadata.py
"""

import json
import math
import os
from typing import Optional

SIDECAR_FORMAT = 'astra.photo_metadata.v1'
SIDECAR_EXTENSION = '.json'


def sidecar_path_for(image_path: str) -> str:
    """Return the sidecar path of a photo: same folder and name, extension .json."""
    return os.path.splitext(image_path)[0] + SIDECAR_EXTENSION


def _clean(value, digits: int = 7):
    """Return a finite float rounded to `digits`, or None (NaN, inf, junk become None)."""
    if value is None:
        return None
    try:
        x = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(x):
        return None
    return round(x, digits)


def build_photo_metadata(image_name: Optional[str], shutter_unix_s: float,
                         trigger_unix_s: Optional[float] = None,
                         lat=None, lon=None, alt_msl_m=None, fix_age_s=None,
                         rel_alt_m=None, rel_alt_age_s=None,
                         heading_deg=None, heading_age_s=None,
                         camera_model: str = '', hfov_deg=None, zoom_x=None,
                         zoom_source: str = '', zoom_changing: bool = False,
                         image_top_direction_deg: float = 0.0, rotate_180: bool = False,
                         gimbal: Optional[dict] = None) -> dict:
    """
    Build the sidecar dictionary (see the module docstring for every field).

    `image_name` may be filled in later (with the SD-card name when there was no GPS fix).
    `gimbal` is {'yaw_deg', 'pitch_deg', 'roll_deg'} as the camera reported them, or None.
    Every other argument is the value at the shutter; None = not known.

    Returns
    -------
    dict
        A JSON-ready dict (no NaN or infinity anywhere).

    """
    gimbal_out = None
    if gimbal:
        gimbal_out = {
            'yaw_deg': _clean(gimbal.get('yaw_deg'), 2),
            'pitch_deg': _clean(gimbal.get('pitch_deg'), 2),
            'roll_deg': _clean(gimbal.get('roll_deg'), 2),
            'source': str(gimbal.get('source', 'camera 0x0D')),
        }
    return {
        'format': SIDECAR_FORMAT,
        'image_name': image_name,
        'shutter_unix_s': _clean(shutter_unix_s, 3),
        'trigger_unix_s': _clean(trigger_unix_s, 3),
        'position': {
            'lat': _clean(lat, 9),
            'lon': _clean(lon, 9),
            'alt_msl_m': _clean(alt_msl_m, 2),
            'age_s': _clean(fix_age_s, 3),
        },
        'rel_alt_m': _clean(rel_alt_m, 3),
        'rel_alt_age_s': _clean(rel_alt_age_s, 3),
        'heading_deg': _clean(heading_deg, 2),
        'heading_age_s': _clean(heading_age_s, 3),
        'camera': {
            'model': str(camera_model or ''),
            'hfov_deg': _clean(hfov_deg, 3),
            'zoom_x': _clean(zoom_x, 3),
            'zoom_source': str(zoom_source or ''),
            'zoom_changing': bool(zoom_changing),
            'image_top_direction_deg': _clean(image_top_direction_deg, 2) or 0.0,
            'rotate_180': bool(rotate_180),
        },
        'gimbal': gimbal_out,
    }


def write_sidecar(path: str, metadata: dict) -> bool:
    """
    Write the sidecar atomically (temporary file + rename), so a reader never sees half a file.

    Returns
    -------
    bool
        True on success, False if it could not be written (the photo is still saved;
        main_controller then aims at the photo position for that photo).

    """
    tmp = path + '.tmp'
    try:
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(metadata, f, indent=1)
        os.replace(tmp, path)
        return True
    except (OSError, TypeError, ValueError):
        try:
            if os.path.exists(tmp):
                os.remove(tmp)
        except OSError:
            pass
        return False


def read_sidecar(path: str) -> Optional[dict]:
    """Read a sidecar file; None when it is missing, unreadable or another format."""
    try:
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None
    if not isinstance(data, dict) or data.get('format') != SIDECAR_FORMAT:
        return None
    return data
