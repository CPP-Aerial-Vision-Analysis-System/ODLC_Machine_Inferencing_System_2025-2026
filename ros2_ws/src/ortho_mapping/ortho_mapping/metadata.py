#!/usr/bin/env python3
"""Per-image pose collection for the batch mapper.

Pose sources, best first:
  1. JSON sidecar written by siyi_node at shutter time
     ("<stem>.json" next to the image): lat/lon, rel_alt, compass heading,
     gimbal attitude, timestamp.
  2. Filename "<lat> , <lon>.jpg" (the existing video_cam convention) for
     position; file mtime for ordering; heading derived from the GPS track;
     altitude from MappingConfig.default_altitude_agl_m.

Images with neither a sidecar nor parseable filename coordinates are
skipped (with a reason recorded for the run report).

ROS-free: stdlib only.
"""

import json
import os
import struct
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .config import IMAGE_EXTENSIONS, MIN_TRACK_STEP_M
from .geo_math import bearing_deg, distance_m


@dataclass
class ImagePose:
    """Everything the mapper needs to place one image."""
    path: str
    lat: float
    lon: float
    alt_agl_m: float
    timestamp: float                 # unix seconds (sidecar or file mtime)
    width: int
    height: int
    heading_deg: Optional[float] = None      # compass heading at shutter
    gimbal_yaw_deg: Optional[float] = None   # relative to airframe (follow mode)
    gimbal_pitch_deg: Optional[float] = None  # -90 = nadir
    gimbal_roll_deg: Optional[float] = None
    has_sidecar: bool = False
    heading_source: str = 'none'     # 'sidecar' | 'track' | 'fixed' | 'none'
    notes: List[str] = field(default_factory=list)


def parse_latlon_from_name(filename: str) -> Optional[Tuple[float, float]]:
    """Parse the "<lat> , <lon>" stem convention used by siyi_node.

    The ' , ' (space-comma-space) delimiter is the same one detection/new_od.py
    relies on. Returns None for anything that doesn't parse (e.g. sim_*.jpg
    or the camera's native IMG_xxxx names).
    """
    stem = os.path.splitext(os.path.basename(filename))[0]
    parts = stem.split(' , ')
    if len(parts) != 2:
        return None
    try:
        lat = float(parts[0])
        lon = float(parts[1])
    except ValueError:
        return None
    if lat == 0.0 and lon == 0.0:
        return None  # Null Island = "no GPS fix" in this codebase
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return None
    return lat, lon


def sidecar_path_for(image_path: str) -> str:
    return os.path.splitext(image_path)[0] + '.json'


def load_sidecar(image_path: str) -> Optional[dict]:
    """Load and minimally validate the metadata sidecar, or None."""
    path = sidecar_path_for(image_path)
    if not os.path.isfile(path):
        return None
    try:
        with open(path, 'r', encoding='utf-8') as fh:
            data = json.load(fh)
    except (OSError, json.JSONDecodeError, UnicodeDecodeError):
        return None
    if not isinstance(data, dict):
        return None
    return data


# ---------------------------------------------------------------------------
# Cheap image-size probing (no full JPEG decode; a 4K decode costs ~100 ms
# on the Jetson and we only need dimensions for canvas sizing).
# ---------------------------------------------------------------------------

def read_image_size(path: str) -> Optional[Tuple[int, int]]:
    """(width, height) from JPEG/PNG/BMP headers without decoding pixels."""
    try:
        with open(path, 'rb') as fh:
            head = fh.read(32)
            if head.startswith(b'\xff\xd8'):
                return _jpeg_size(fh, head)
            if head.startswith(b'\x89PNG\r\n\x1a\n') and len(head) >= 24:
                w, h = struct.unpack('>II', head[16:24])
                return int(w), int(h)
            if head.startswith(b'BM') and len(head) >= 26:
                w, h = struct.unpack('<ii', head[18:26])
                return int(w), abs(int(h))
    except OSError:
        return None
    return None


def _jpeg_size(fh, head: bytes) -> Optional[Tuple[int, int]]:
    """Walk JPEG segments to the SOF marker that carries the dimensions."""
    fh.seek(2)
    while True:
        marker = fh.read(2)
        if len(marker) < 2 or marker[0] != 0xFF:
            return None
        code = marker[1]
        # Standalone markers with no length field
        if code in (0xD8, 0x01) or 0xD0 <= code <= 0xD7:
            continue
        length_bytes = fh.read(2)
        if len(length_bytes) < 2:
            return None
        (seg_len,) = struct.unpack('>H', length_bytes)
        # SOF0..SOF15 except DHT(0xC4)/JPG(0xC8)/DAC(0xCC) hold dimensions
        if 0xC0 <= code <= 0xCF and code not in (0xC4, 0xC8, 0xCC):
            body = fh.read(5)
            if len(body) < 5:
                return None
            h, w = struct.unpack('>HH', body[1:5])
            return int(w), int(h)
        fh.seek(seg_len - 2, os.SEEK_CUR)


# ---------------------------------------------------------------------------
# Collection
# ---------------------------------------------------------------------------

def collect_poses(images_dir: str,
                  default_alt_agl_m: float) -> Tuple[List[ImagePose], List[dict]]:
    """Scan images_dir and build an ImagePose per usable image.

    Returns (poses sorted by timestamp, skipped) where skipped is a list of
    {'file': name, 'reason': text} for the run report.
    """
    poses: List[ImagePose] = []
    skipped: List[dict] = []

    try:
        names = sorted(os.listdir(images_dir))
    except OSError as exc:
        raise RuntimeError(f"Cannot list images_dir {images_dir}: {exc}")

    for name in names:
        if not name.lower().endswith(IMAGE_EXTENSIONS):
            continue
        path = os.path.join(images_dir, name)
        if not os.path.isfile(path):
            continue

        sidecar = load_sidecar(path)
        lat = lon = None
        if sidecar is not None:
            lat = _as_float(sidecar.get('latitude'))
            lon = _as_float(sidecar.get('longitude'))
        if lat is None or lon is None:
            parsed = parse_latlon_from_name(name)
            if parsed is not None:
                lat, lon = parsed
        if lat is None or lon is None:
            skipped.append({'file': name, 'reason': 'no GPS (no sidecar, unparseable name)'})
            continue

        size = read_image_size(path)
        if size is None:
            skipped.append({'file': name, 'reason': 'unreadable image header'})
            continue
        width, height = size
        if width < 100 or height < 100:
            skipped.append({'file': name, 'reason': f'implausible size {width}x{height}'})
            continue

        pose = ImagePose(
            path=path, lat=lat, lon=lon,
            alt_agl_m=default_alt_agl_m,
            timestamp=os.path.getmtime(path),
            width=width, height=height,
        )

        if sidecar is not None:
            pose.has_sidecar = True
            ts = _as_float(sidecar.get('timestamp_unix'))
            if ts is not None:
                pose.timestamp = ts
            alt = _as_float(sidecar.get('rel_alt_m'))
            if alt is not None and alt > 0.0:
                pose.alt_agl_m = alt
            else:
                pose.notes.append('sidecar missing/invalid rel_alt; using default')
            hdg = _as_float(sidecar.get('compass_hdg_deg'))
            if hdg is not None:
                pose.heading_deg = hdg % 360.0
            gimbal = sidecar.get('gimbal')
            if isinstance(gimbal, dict):
                pose.gimbal_yaw_deg = _as_float(gimbal.get('yaw_deg'))
                pose.gimbal_pitch_deg = _as_float(gimbal.get('pitch_deg'))
                pose.gimbal_roll_deg = _as_float(gimbal.get('roll_deg'))

        poses.append(pose)

    poses.sort(key=lambda p: p.timestamp)
    return poses, skipped


def assign_headings(poses: List[ImagePose], source: str,
                    fixed_heading_deg: float = 0.0) -> None:
    """Fill pose.heading_deg for every pose, in place.

    source: 'auto' | 'sidecar' | 'track' | 'fixed' (see MappingConfig).
    Track headings use the bearing to the next photo (the aircraft flies
    roughly camera-top-forward on a lawnmower pattern); photos closer than
    MIN_TRACK_STEP_M inherit the previous leg's heading.
    """
    if source == 'fixed':
        for p in poses:
            p.heading_deg = fixed_heading_deg % 360.0
            p.heading_source = 'fixed'
        return

    if source == 'sidecar':
        for p in poses:
            p.heading_source = 'sidecar' if p.heading_deg is not None else 'none'
        return

    track = _track_headings(poses)
    for pose, track_hdg in zip(poses, track):
        if source == 'auto' and pose.heading_deg is not None:
            pose.heading_source = 'sidecar'
            continue
        if track_hdg is not None:
            pose.heading_deg = track_hdg
            pose.heading_source = 'track'
        elif pose.heading_deg is None:
            pose.heading_deg = fixed_heading_deg % 360.0
            pose.heading_source = 'fixed'
            pose.notes.append('no track heading available; used fixed fallback')


def _track_headings(poses: List[ImagePose]) -> List[Optional[float]]:
    """Bearing to the next sufficiently-distant photo, carried forward."""
    n = len(poses)
    headings: List[Optional[float]] = [None] * n
    for i in range(n):
        for j in range(i + 1, n):
            if distance_m(poses[i].lat, poses[i].lon,
                          poses[j].lat, poses[j].lon) >= MIN_TRACK_STEP_M:
                headings[i] = bearing_deg(poses[i].lat, poses[i].lon,
                                          poses[j].lat, poses[j].lon)
                break
    # Photos with no forward leg (hover bursts, the final photo) inherit
    # the last known heading; leading gaps inherit the first known one.
    last = None
    for i in range(n):
        if headings[i] is None:
            headings[i] = last
        else:
            last = headings[i]
    first = next((h for h in headings if h is not None), None)
    for i in range(n):
        if headings[i] is None:
            headings[i] = first
    return headings


def _as_float(value) -> Optional[float]:
    if value is None or isinstance(value, bool):
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    if out != out:  # NaN
        return None
    return out
