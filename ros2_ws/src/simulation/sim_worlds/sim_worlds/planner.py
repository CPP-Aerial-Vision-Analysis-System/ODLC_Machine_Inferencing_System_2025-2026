"""
sim_worlds/planner.py -- PURPOSE: turn "survey this rectangle from this height" into a complete mission in the
team's shape (sim_worlds.mission), and read the survey settings from a world's config file.

Used by the world generators (scripts/build_*.py) and by scripts/make_mission.py, so a mission can always be
re-made from config/<world>.yaml alone, for example at a different altitude:
    ros2 run sim_worlds make_mission.py runway --altitude 35
"""

import math
from dataclasses import dataclass

from sim_worlds import geo, mission, survey


@dataclass
class SurveySettings:
    """How to fly the survey.

    altitude_m: height above home [m]. sidelap / frontlap: photo overlap across / along the lanes (0..1).
    photo_interval_s: seconds between two photos the pipeline really delivers (the trigger asks every 1 s).
    speed_mps: lane speed [m/s]; None = the fastest speed that keeps the frontlap; 0 = no DO_CHANGE_SPEED item
    (ArduPilot then uses its WPNAV_SPEED parameter). max_speed_mps: upper limit for the automatic speed.
    hfov_rad / width_px / height_px: the photo camera. overshoot_m: metres flown past each lane end.
    lead_in_s: seconds of flight added BEFORE the first lane, on its extension outside the area (0 = the first
    lane starts at the area edge, last year's layout). Why: when the mission's DO_DIGICAM_CONTROL starts the
    photos, the team's video_cam first racks the zoom and focuses (about 15 s; its lens worker drops every
    photo request meanwhile), so without a lead-in the first ~80 m of lane 1 get no photos at all. The first
    lane waypoint moves lead_in_s x lane speed metres back; the mission keeps exactly the same items and indices.
    """

    altitude_m: float
    sidelap: float = 0.3
    frontlap: float = 0.6
    photo_interval_s: float = 2.0
    speed_mps: float = None
    max_speed_mps: float = 10.0
    hfov_rad: float = survey.A8_HFOV_RAD
    width_px: int = survey.PHOTO_WIDTH_PX
    height_px: int = survey.PHOTO_HEIGHT_PX
    overshoot_m: float = 0.0
    lead_in_s: float = 0.0


def lead_in_point(start, nxt, distance_m):
    """The point `distance_m` metres BEFORE `start` on the line from `nxt` through `start` (lane 1 made longer
    at its beginning). Returns `start` unchanged when the distance is 0 or the two points coincide."""
    dx, dy = start[0] - nxt[0], start[1] - nxt[1]
    n = math.hypot(dx, dy)
    if distance_m <= 0 or n < 1e-9:
        return start
    return (start[0] + dx / n * distance_m, start[1] + dy / n * distance_m)


def plan_survey_mission(home, rect, settings, hold_xy=None):
    """Plan the lanes over `rect` (survey.Rect) and build the mission.

    home: geo.Home (world origin; the drone starts there). settings: SurveySettings. hold_xy: where the drone
    waits for detection (default: the centre of the rectangle).
    Returns (items, plan, speed_mps): mission items (list of mission.MissionItem), the survey.SurveyPlan, and
    the lane speed written into the mission (None when no DO_CHANGE_SPEED item was written).
    """
    plan = survey.plan_lanes(rect, settings.altitude_m, settings.sidelap, settings.hfov_rad, settings.width_px,
                             settings.height_px, start_xy=(0.0, 0.0), overshoot_m=settings.overshoot_m)
    if settings.speed_mps is None:
        speed = survey.survey_speed(settings.altitude_m, settings.frontlap, settings.photo_interval_s,
                                    settings.max_speed_mps, hfov_rad=settings.hfov_rad,
                                    width_px=settings.width_px, height_px=settings.height_px)
    elif settings.speed_mps == 0:
        speed = None
    else:
        speed = float(settings.speed_mps)
    waypoints = list(plan.waypoints)
    if settings.lead_in_s > 0 and len(waypoints) >= 2:
        # lane speed, or ArduPilot's own speed (unknown here: assume the upper limit) without DO_CHANGE_SPEED
        waypoints[0] = lead_in_point(waypoints[0], waypoints[1],
                                     settings.lead_in_s * (speed or settings.max_speed_mps))
    latlon = [geo.enu_to_latlon(x, y, home) for x, y in waypoints]
    hx, hy = hold_xy if hold_xy is not None else (rect.cx, rect.cy)
    items = mission.build_survey_mission(home, latlon, settings.altitude_m, geo.enu_to_latlon(hx, hy, home), speed)
    return items, plan, speed


def rect_from_area(area):
    """survey.Rect from a config 'search_areas' entry {center: [x, y], size: [length, width], yaw: rad}."""
    return survey.Rect(float(area['center'][0]), float(area['center'][1]), float(area['size'][0]),
                       float(area['size'][1]), float(area.get('yaw', 0.0)))


def settings_from_config(survey_cfg, **overrides):
    """SurveySettings from a config 'survey' section, with keyword overrides (None values are ignored)."""
    cam = survey_cfg.get('camera', {}) or {}
    s = SurveySettings(altitude_m=float(survey_cfg['altitude_m']),
                       sidelap=float(survey_cfg.get('sidelap', 0.3)),
                       frontlap=float(survey_cfg.get('frontlap', 0.6)),
                       photo_interval_s=float(survey_cfg.get('photo_interval_s', 2.0)),
                       speed_mps=survey_cfg.get('speed_mps'),
                       max_speed_mps=float(survey_cfg.get('max_speed_mps', 10.0)),
                       hfov_rad=float(cam.get('hfov_rad', survey.A8_HFOV_RAD)),
                       width_px=int(cam.get('width_px', survey.PHOTO_WIDTH_PX)),
                       height_px=int(cam.get('height_px', survey.PHOTO_HEIGHT_PX)),
                       overshoot_m=float(survey_cfg.get('overshoot_m', 0.0)),
                       lead_in_s=float(survey_cfg.get('lead_in_s', 0.0) or 0.0))
    for k, v in overrides.items():
        if v is not None:
            setattr(s, k, v)
    return s


def summary(items, plan, speed, home):
    """A dict that describes a planned mission (written into config files and printed by make_mission.py)."""
    idx = mission.team_indices(items)
    fp = plan.footprint
    return {'altitude_m': plan.altitude_m, 'lanes': plan.n_lanes, 'lane_spacing_m': round(plan.spacing_m, 2),
            'max_lane_spacing_m': round(plan.max_spacing_m, 2), 'footprint_m': [round(fp.width_m, 1), round(fp.height_m, 1)],
            'gsd_cm_per_px': round(fp.gsd_m * 100.0, 2), 'speed_mps': speed,
            'survey_path_m': round(plan.path_length_m, 1),
            'total_path_m': round(mission.path_length_m(items, home), 1),
            'items': len(items), 'digicam_index': idx['digicam_indices'][0] if idx['digicam_indices'] else -1,
            'buffer_wp': idx['buffer_wp'], 'hold_wp': idx['last_before_rtl'], 'rtl_index': idx['rtl_index'],
            'lane_yaw_deg': round(math.degrees(plan.lane_yaw), 1)}
