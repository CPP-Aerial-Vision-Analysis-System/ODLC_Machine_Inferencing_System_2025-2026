"""
sim_demo/photo_info.py -- PURPOSE: use the camera emulator's exact record of every photo it
took (topic /sim/camera/photo_info, std_msgs/String JSON, one message per photo, published by
siyi_emulator) to know precisely where the camera was and where it pointed for each photo
that video_cam later saves in the team's photo folder.

The emulator's JSON contains (siyi_emulator/emulator.py _photo_metadata + node.py):
    wall_time            UTC ISO time the photo was made, e.g. "2026-10-10T18:40:59.123+00:00"
    image                {width, height}
    zoom                 {total, optical, digital}
    hfov_rad             {lens, effective}
    camera_world         {position_enu_m: [x, y, z], optical_quat_xyzw: [x, y, z, w], ...}
    sd_name              IMG_NNNN.jpg on the emulated SD card
Matching: video_cam chooses the file name from the GPS FIRST, then takes the photo (sometimes
seconds later, when it has to wait for the camera lock), downloads it and saves it as
"<lat> , <lon>.jpg", one photo at a time, in order. Rules, best first:
  1. when we know the moment the drone was at the file name's GPS position (from the MAVROS
     history), the right record is the FIRST not-yet-matched one made after that moment;
  2. otherwise the not-yet-matched record whose camera position is NEAREST to the name's GPS
     (records about equally near, e.g. while hovering, are taken oldest first);
  3. without GPS in the name, the oldest not-yet-matched record.
Older unmatched records are then dropped (their download failed). Pure Python, no ROS.
"""
from __future__ import annotations

import json
import math
import threading
from dataclasses import dataclass
from datetime import datetime

from .camera_model import CameraIntrinsics, CameraPose, axes_from_quaternion
from .geo import GeoOrigin


@dataclass
class EmulatorPhoto:
    """One photo as the emulator recorded it.

    Attributes:
        t: wall-clock time it was made [s, epoch]. sd_name: IMG_NNNN.jpg.
        intr: intrinsics (size, lens field of view, zoom). pose: exact camera pose (world ENU).
        matched: True once it has been paired with a saved file.
    """

    t: float
    sd_name: str
    intr: CameraIntrinsics
    pose: CameraPose
    matched: bool = False


def parse_photo_info(text: str) -> EmulatorPhoto | None:
    """Parse one /sim/camera/photo_info JSON message (None if it lacks the needed fields)."""
    try:
        d = json.loads(text)
        t = datetime.fromisoformat(str(d['wall_time']).replace('Z', '+00:00')).timestamp()
        img = d.get('image') or {}
        width, height = int(img['width']), int(img['height'])
        zoom = float((d.get('zoom') or {}).get('total', 1.0) or 1.0)
        hfov = d.get('hfov_rad') or {}
        lens = float(hfov.get('lens') or 1.4137)
        cam = d['camera_world']
        x, y, z = (float(v) for v in cam['position_enu_m'][:3])
        q = cam.get('optical_quat_xyzw')
        axes = axes_from_quaternion(*[float(v) for v in q]) if q and len(q) == 4 else None
        pose = CameraPose(x, y, z, yaw_deg=float(cam.get('heading_compass_deg', 0.0) or 0.0),
                          pitch_deg=float(cam.get('pitch_deg', -90.0) or -90.0), axes=axes)
        return EmulatorPhoto(t, str(d.get('sd_name', '')), CameraIntrinsics(width, height, lens, zoom),
                             pose)
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None


class PhotoInfoIndex:
    """Pairs saved photo files with the emulator's photo records (see module docstring)."""

    def __init__(self, origin: GeoOrigin, window_s: float = 30.0, max_distance_m: float = 15.0,
                 tie_tolerance_m: float = 2.0):
        """
        Args:
            origin: the world's GPS origin (to compare file-name GPS with emulator metres).
            window_s: an emulator photo must be at most this old when the file appears.
            max_distance_m: file-name GPS and emulator camera position must agree this well.
            tie_tolerance_m: records this much farther than the nearest still count as equally
                near (then the oldest wins).
        """
        self.origin = origin
        self.window_s = window_s
        self.max_distance_m = max_distance_m
        self.tie_tolerance_m = tie_tolerance_m
        self.photos: list[EmulatorPhoto] = []
        self.received = 0
        self.last_miss = ''               # why the last match() returned None (for logs)
        self._lock = threading.Lock()     # add_json and match may run on different threads

    def add_json(self, text: str) -> bool:
        """Add one /sim/camera/photo_info message. Returns False if it could not be parsed."""
        p = parse_photo_info(text)
        if p is None:
            return False
        with self._lock:
            self.received += 1
            self.photos.append(p)
            self.photos.sort(key=lambda e: e.t)
            if len(self.photos) > 2000:
                del self.photos[:500]
        return True

    def match(self, file_time: float, lat: float | None, lon: float | None,
              name_time: float | None = None) -> EmulatorPhoto | None:
        """The emulator photo a newly saved file came from (None if there is no good match).

        Args:
            file_time: modification time of the saved file [s, epoch].
            lat, lon: GPS from the file name (None if the name has none).
            name_time: when the drone was at the file name's GPS position [s, epoch] (from the
                MAVROS history), i.e. when video_cam chose the name. video_cam takes the photo
                AFTER choosing the name (sometimes seconds later, while it waits for the camera
                lock), so the right record is the FIRST one made after name_time. This is the
                most reliable rule; without name_time the nearest record is used.
        """
        target = self.origin.to_enu(lat, lon)[:2] if lat is not None and lon is not None else None
        with self._lock:
            cands = [p for p in self.photos               # oldest first (FIFO, like video_cam)
                     if not p.matched and file_time - self.window_s <= p.t <= file_time + 1.0]
            if name_time is not None:
                after = [p for p in cands if p.t >= name_time - 0.5]
                if after and (target is None or math.hypot(after[0].pose.east - target[0],
                                                           after[0].pose.north - target[1])
                                                <= self.max_distance_m * 4):
                    best = after[0]
                    best.matched = True
                    for older in self.photos:             # anything older was never saved
                        if older.t < best.t and not older.matched:
                            older.matched = True
                    return best
            if target is not None:
                # Photos are ~5 m apart during a survey while the file-name GPS is within ~1-3 m
                # of the camera: the NEAREST record wins; records about as near (hover) go FIFO.
                dist = [(math.hypot(p.pose.east - target[0], p.pose.north - target[1]), p)
                        for p in cands]
                near = [(d, p) for d, p in dist if d <= self.max_distance_m]
                if not near:
                    self.last_miss = (f'{len(cands)} records in the time window, nearest '
                                      f'{min((d for d, _ in dist), default=float("inf")):.1f} m away '
                                      f'(received {self.received})')
                    return None
                dist = near
                d_min = min(d for d, _ in dist)
                best = next(p for d, p in dist if d <= d_min + self.tie_tolerance_m)
            elif cands:
                best = cands[0]
            else:
                self.last_miss = (f'no record in the time window (received {self.received}, '
                                  f'newest {max((p.t for p in self.photos), default=0) - file_time:+.1f} s '
                                  'from the file)')
                return None
            best.matched = True
            for older in self.photos:                     # anything older was never saved
                if older.t < best.t and not older.matched:
                    older.matched = True
            return best
