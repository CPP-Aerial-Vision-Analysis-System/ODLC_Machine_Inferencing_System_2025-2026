"""
sim_worlds/survey.py -- PURPOSE: the camera-footprint maths and the lawn-mower ("survey grid") planner used to
build the search missions.

The idea in plain words:
  * A camera pointing straight down from height h sees a rectangle of ground (its "footprint").
    With a horizontal field of view hfov the footprint is  width = 2 * h * tan(hfov / 2)  metres wide and
    width * image_height / image_width  metres tall (docs/dev/ARCHITECTURE.md section 8).
  * The drone flies straight parallel lines ("lanes") over the search area, nose along the lane. The gimbal
    follows the nose, so the wide side of the photo is ACROSS the lane.
  * Neighbouring lanes must overlap by `sidelap` (0.3 = 30 %) so nothing is missed between them; the lanes
    are spread evenly so the first and last photo strips end exactly at the area's edges.
  * Photos along a lane must overlap by `frontlap`. The team's trigger fires once per second but the real
    pipeline delivers about one photo every 2 s, so the speed is chosen from that photo interval.

Defaults are the team's camera: SIYI A8 mini lens (hfov 81 deg = 1.4137 rad) taking 2560 x 1440 stills (the
ZR10 2K profile last year's code and the detector use).
"""

import math
from dataclasses import dataclass, field

A8_HFOV_RAD = 1.4137      #: SIYI A8 mini horizontal field of view [rad] (81 degrees)
PHOTO_WIDTH_PX = 2560     #: still photo width [px] (contract default camera_model zr10)
PHOTO_HEIGHT_PX = 1440    #: still photo height [px]


@dataclass(frozen=True)
class Footprint:
    """Ground rectangle seen by a straight-down camera.

    width_m: metres across the image width (across the lane). height_m: metres along the image height
    (along the lane). gsd_m: ground sample distance, metres of ground per pixel at the image centre.
    """

    width_m: float
    height_m: float
    gsd_m: float


def footprint(altitude_m, hfov_rad=A8_HFOV_RAD, width_px=PHOTO_WIDTH_PX, height_px=PHOTO_HEIGHT_PX):
    """Footprint of a nadir camera at altitude_m metres above flat ground. Returns a Footprint."""
    if altitude_m <= 0:
        raise ValueError('altitude must be > 0 m')
    w = 2.0 * altitude_m * math.tan(hfov_rad / 2.0)
    return Footprint(width_m=w, height_m=w * height_px / width_px, gsd_m=w / width_px)


@dataclass(frozen=True)
class Rect:
    """A rectangle on the ground (local metres, ENU).

    cx, cy: centre. length: side along `yaw` (the lane direction). width: the other side.
    yaw: direction of the length side in radians (0 = east, pi/2 = north, counter-clockwise).
    """

    cx: float
    cy: float
    length: float
    width: float
    yaw: float = 0.0

    def to_world(self, u, v):
        """Rectangle coordinates (u along the length, v across, from the centre) -> world (x, y)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.cx + u * c - v * s, self.cy + u * s + v * c

    def to_rect(self, x, y):
        """World (x, y) -> rectangle coordinates (u along the length, v across, from the centre)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = x - self.cx, y - self.cy
        return dx * c + dy * s, -dx * s + dy * c

    def corners(self):
        """The four corners in world coordinates, counter-clockwise."""
        hl, hw = self.length / 2.0, self.width / 2.0
        return [self.to_world(u, v) for u, v in ((-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw))]

    def contains(self, x, y, margin=0.0):
        """True when (x, y) is inside the rectangle shrunk by `margin` metres on every side."""
        u, v = self.to_rect(x, y)
        return abs(u) <= self.length / 2.0 - margin and abs(v) <= self.width / 2.0 - margin


@dataclass
class SurveyPlan:
    """Result of plan_lanes().

    waypoints: [(x, y), ...] in flight order (lane 1 start, lane 1 end, lane 2 start, ...), metres ENU.
    lanes: [((x0, y0), (x1, y1)), ...] the same points grouped per lane.
    spacing_m: actual distance between neighbouring lanes. max_spacing_m: the largest spacing the sidelap allows.
    path_length_m: length of the survey path from the first to the last waypoint.
    """

    waypoints: list
    lanes: list
    spacing_m: float
    max_spacing_m: float
    footprint: Footprint
    altitude_m: float
    sidelap: float
    lane_yaw: float
    path_length_m: float
    notes: list = field(default_factory=list)

    @property
    def n_lanes(self):
        """Number of lanes."""
        return len(self.lanes)


def plan_lanes(rect, altitude_m, sidelap=0.3, hfov_rad=A8_HFOV_RAD, width_px=PHOTO_WIDTH_PX,
               height_px=PHOTO_HEIGHT_PX, start_xy=(0.0, 0.0), overshoot_m=0.0):
    """Plan a lawn-mower survey of `rect` (a Rect) at `altitude_m`.

    Lanes run along rect.length (rect.yaw). sidelap: fraction (0..0.9) of the photo width shared by
    neighbouring lanes. start_xy: where the drone comes from (home); the plan starts at the rectangle corner
    nearest to it. overshoot_m: extra metres flown past each end of a lane (0 = lanes end at the edges).
    Returns a SurveyPlan. Raises ValueError for impossible inputs.
    """
    if not 0.0 <= sidelap < 0.95:
        raise ValueError('sidelap must be between 0 and 0.95')
    if rect.length <= 0 or rect.width <= 0:
        raise ValueError('the search rectangle needs a positive length and width')
    fp = footprint(altitude_m, hfov_rad, width_px, height_px)
    max_spacing = fp.width_m * (1.0 - sidelap)
    notes = []
    if rect.width <= fp.width_m:
        n, spacing = 1, 0.0
        vs = [0.0]
        notes.append('one lane is enough: the photo is wider than the search area')
    else:
        n = int(math.ceil((rect.width - fp.width_m) / max_spacing - 1e-9)) + 1
        spacing = (rect.width - fp.width_m) / (n - 1)
        v0 = -rect.width / 2.0 + fp.width_m / 2.0
        vs = [v0 + k * spacing for k in range(n)]
    hl = rect.length / 2.0 + overshoot_m

    # Try the four possible starting corners and keep the one nearest to start_xy.
    best = None
    for reverse_lanes in (False, True):
        order = list(reversed(vs)) if reverse_lanes else list(vs)
        for first_dir in (1.0, -1.0):
            pts = []
            d = first_dir
            for v in order:
                pts.append(rect.to_world(-d * hl, v))
                pts.append(rect.to_world(d * hl, v))
                d = -d
            dist = math.hypot(pts[0][0] - start_xy[0], pts[0][1] - start_xy[1])
            if best is None or dist < best[0] - 1e-9:
                best = (dist, pts)
    pts = best[1]
    lanes = [(pts[2 * i], pts[2 * i + 1]) for i in range(n)]
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:]))
    return SurveyPlan(waypoints=pts, lanes=lanes, spacing_m=spacing, max_spacing_m=max_spacing, footprint=fp,
                      altitude_m=altitude_m, sidelap=sidelap, lane_yaw=rect.yaw, path_length_m=length,
                      notes=notes)


def survey_speed(altitude_m, frontlap=0.6, photo_interval_s=2.0, max_speed=10.0, min_speed=2.0,
                 hfov_rad=A8_HFOV_RAD, width_px=PHOTO_WIDTH_PX, height_px=PHOTO_HEIGHT_PX):
    """Fastest ground speed (m/s, rounded DOWN to 0.5 m/s) that still gives `frontlap` overlap between photos.

    One photo every photo_interval_s seconds; the drone may move at most footprint_height * (1 - frontlap)
    metres between two photos. Clamped to [min_speed, max_speed].
    """
    if photo_interval_s <= 0:
        raise ValueError('photo_interval_s must be > 0')
    fp = footprint(altitude_m, hfov_rad, width_px, height_px)
    v = fp.height_m * (1.0 - frontlap) / photo_interval_s
    v = math.floor(v * 2.0) / 2.0
    return max(min_speed, min(max_speed, v))
