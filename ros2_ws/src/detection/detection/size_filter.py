"""
Physical size filter: drop detections whose real-world size cannot be a target.

PURPOSE: the detector boxes tree crowns as "person" (boxes 8 to 20 m long) and round crowns as "tent". A hiker (the SUAS mannequin) is
about 1 to 2 m long and the pop-up tent about 2.4 m; a box several metres long is not one of them. The size of a pixel on the ground
(GSD) follows from the camera's field of view and the height of the camera above the ground, both saved in each photo's JSON sidecar, so
every box can be turned into metres and judged. The filter is OFF unless the ROS parameter physical_size_filter is true (or the
environment variable ASTRA_PHYSICAL_SIZE_FILTER is 1): it changes which detections the mission acts on, so the team decides.
"""

import json
import math
import os

#: allowed length of the longer box side in metres, per class (a margin around the mannequin and the tent)
DEFAULT_LIMITS_M = {'person': (0.3, 3.0), 'tent': (1.0, 4.5)}


def sidecar_path(image_path):
    """Return the JSON sidecar path that belongs to a photo."""
    return os.path.splitext(image_path)[0] + '.json'


def ground_sample_distance(image_path, image_width_px, ground_height_m=0.0):
    """Return metres per pixel on the ground for a photo, or None when the sidecar cannot say.

    Uses rel_alt_m (height above the take-off point) minus ground_height_m as the camera height, and the horizontal field of view
    (at zoom 1 the photo's width covers 2 * height * tan(hfov / 2); zoom narrows it).
    """
    try:
        with open(sidecar_path(image_path), encoding='utf-8') as f:
            meta = json.load(f)
        height = float(meta['rel_alt_m']) - float(ground_height_m)
        cam = meta.get('camera', {})
        hfov = float(cam.get('hfov_deg', 81.0))
        zoom = float(cam.get('zoom_x', 1.0)) or 1.0
    except (OSError, ValueError, KeyError, TypeError):
        return None
    if height <= 1.0 or image_width_px <= 0:
        return None
    return 2.0 * height * math.tan(math.radians(hfov / 2.0)) / zoom / image_width_px


def apply_size_filter(detections, gsd_m_per_px, limits=None):
    """Return (kept, dropped): detections whose longer box side is a possible size for their class, and the rest.

    Only the classes in limits (person, tent by default) are judged; any other class is kept. gsd_m_per_px None keeps everything.
    """
    if not gsd_m_per_px:
        return list(detections), []
    limits = limits or DEFAULT_LIMITS_M
    kept, dropped = [], []
    for det in detections:
        lim = limits.get(det.get('class'))
        if lim is None:
            kept.append(det)
            continue
        x1, y1, x2, y2 = det['bbox']
        side_m = max(x2 - x1, y2 - y1) * gsd_m_per_px
        (kept if lim[0] <= side_m <= lim[1] else dropped).append(det)
    return kept, dropped
