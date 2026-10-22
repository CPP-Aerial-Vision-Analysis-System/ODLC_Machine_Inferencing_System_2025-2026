"""
example_detector/blob_logic.py -- PURPOSE: the "brain" of the TEMPLATE detector, kept free of ROS so you can
test it with plain pytest and replace it with your own model without touching the ROS part (node.py).

It finds the hiker and the tent by COLOUR: every connected patch of pixels whose colour (in HSV) falls in a
class's range and whose size is plausible becomes one detection. That is deliberately simple: it is a
template that shows the exact inputs and outputs the team's mission code expects. Swap detect() for a YOLO
model, a different colour rule, anything: as long as you return Detection objects in FULL-PHOTO pixels, the
node publishes them correctly.

What the rest of the system needs (the detector contract, docs/simulation/PLUGINS.md):
  * class_id "0" = person (the hiker / mannequin), "1" = tent  (main_controller ignores everything else)
  * score 0..1 (main_controller keeps the highest score per class)
  * the box centre (cx, cy) and size (w, h) in pixels of the FULL-SIZE photo: main_controller projects the
    box centre onto the ground to aim the drop
HSV here is OpenCV's: hue 0..179 (red ~0/179, orange ~10-20, yellow ~30, green ~60, blue ~110),
saturation 0..255 (grey 0, vivid 255), value 0..255 (black 0, bright 255).
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

#: The class ids main_controller understands, with their names.
CLASS_NAMES = {'0': 'person', '1': 'tent'}

#: Default colour ranges (OpenCV HSV, low and high, both included). PLACEHOLDERS: tune them for your targets
#: (ros2 run example_detector example_detector --ros-args -p tent_hsv_low:="[95,120,50]" ...).
DEFAULT_RANGES: Dict[str, Tuple[Tuple[int, int, int], Tuple[int, int, int]]] = {
    '0': ((5, 120, 120), (30, 255, 255)),      # person: orange / yellow (a rescue dummy's colours)
    '1': ((95, 120, 50), (130, 255, 255)),     # tent: blue (the simulator's pop-up tent)
}

#: Photo file endings the team's video_cam writes.
PHOTO_ENDINGS = ('.jpg', '.jpeg', '.png')


@dataclass
class Detection:
    """One found object, in FULL-PHOTO pixels."""
    class_id: str          # "0" person, "1" tent
    score: float           # 0..1
    cx: float              # box centre x (pixels from the left edge)
    cy: float              # box centre y (pixels from the top edge)
    w: float               # box width (pixels)
    h: float               # box height (pixels)

    @property
    def area(self) -> float:
        return self.w * self.h

    @property
    def name(self) -> str:
        return CLASS_NAMES.get(self.class_id, 'object')


def is_photo(file_name: str) -> bool:
    """True for a finished photo file. video_cam writes "<name>.tmp.jpg" first and then renames it, so any
    name with ".tmp." is still being written; sidecars (.json) are not photos."""
    low = file_name.lower()
    return low.endswith(PHOTO_ENDINGS) and '.tmp.' not in low and not low.startswith('.')


def latlon_from_name(file_name: str) -> Tuple[float, float]:
    """The GPS position in a team photo name: "36.215026 , -96.006800.jpg" -> (36.215026, -96.0068).

    video_cam names every photo "<lat> , <lon>.jpg" (space, comma, space) from the drone's GPS at the shutter;
    "IMG_0001.jpg" / "sim_*.jpg" when there was no GPS fix -> (0.0, 0.0), which main_controller ignores.
    """
    stem = os.path.splitext(os.path.basename(file_name))[0]
    parts = stem.split(' , ')
    if len(parts) != 2:
        return 0.0, 0.0
    try:
        lat, lon = float(parts[0]), float(parts[1])
    except ValueError:
        return 0.0, 0.0
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return 0.0, 0.0
    return lat, lon


def find_blobs(bgr, low: Sequence[int], high: Sequence[int], min_area: float, max_area: float
               ) -> List[Tuple[float, float, float, float, float]]:
    """Connected patches whose colour is inside [low, high] (OpenCV HSV).

    bgr: an OpenCV image (numpy array, height x width x 3, blue-green-red). Returns
    [(cx, cy, w, h, fill)] in pixels of THAT image; fill = coloured pixels / box area (0..1).
    """
    import cv2
    import numpy as np
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8))
    # A small "opening" removes single noisy pixels before counting patches.
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = []
    for i in range(1, count):                     # label 0 is the background
        x, y, w, h, pixels = (int(v) for v in stats[i])
        if not (min_area <= pixels <= max_area):
            continue
        out.append((x + w / 2.0, y + h / 2.0, float(w), float(h), pixels / float(max(w * h, 1))))
    return out


def detect(bgr, ranges: Dict[str, Tuple[Sequence[int], Sequence[int]]] = None, min_area_px: float = 150.0,
           max_area_px: float = 200000.0, downscale: int = 4, max_per_class: int = 5) -> List[Detection]:
    """Find persons and tents in one photo. Boxes come back in FULL-PHOTO pixels.

    downscale: work on a 1/N size copy (a 4K photo has 8 million pixels; 1/4 size is 16x faster). The size
    limits min_area_px / max_area_px are in full-photo pixels too.
    """
    import cv2
    ranges = ranges or DEFAULT_RANGES
    n = max(1, int(downscale))
    small = cv2.resize(bgr, (max(1, bgr.shape[1] // n), max(1, bgr.shape[0] // n)),
                       interpolation=cv2.INTER_AREA) if n > 1 else bgr
    scale_x = bgr.shape[1] / float(small.shape[1])
    scale_y = bgr.shape[0] / float(small.shape[0])
    area_scale = scale_x * scale_y
    found: List[Detection] = []
    for class_id, (low, high) in ranges.items():
        blobs = find_blobs(small, low, high, min_area_px / area_scale, max_area_px / area_scale)
        dets = [Detection(class_id=class_id,
                          # Template score: a solid, well-filled patch looks more like a real object.
                          score=round(min(1.0, 0.5 + 0.5 * fill), 3),
                          cx=cx * scale_x, cy=cy * scale_y, w=w * scale_x, h=h * scale_y)
                for cx, cy, w, h, fill in blobs]
        dets.sort(key=lambda d: (d.score, d.area), reverse=True)
        found += dets[:max_per_class]
    return found
