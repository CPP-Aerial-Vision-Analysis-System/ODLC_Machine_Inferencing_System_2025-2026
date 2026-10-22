#!/usr/bin/env python3
"""
sim_worlds/scripts/make_mission.py -- PURPOSE: (re)make or check the search mission of a world, as a QGC WPL 110
file that MAVProxy (`wp load`) and Mission Planner read, in EXACTLY the shape last year's flight software expects
(HOME, TAKEOFF, first lane waypoint, DO_DIGICAM_CONTROL, lanes, buffer_wp = last lane end, hold, RTL;
see sim_worlds/mission.py and the README "Mission shape").

It reads the search area and the survey settings from config/<world>.yaml (written by the world generator) and
plans a lawn-mower survey for a 2560 x 1440 photo camera with the A8 mini's 81 degree lens.

Examples:
  make_mission.py runway                         # rewrite missions/runway_survey.waypoints from the config
  make_mission.py runway --altitude 35 --out /tmp/runway_35m.waypoints
  make_mission.py suas_field --area search_boundary_2 --out /tmp/sb2.waypoints
  make_mission.py --check missions/runway_survey.waypoints     # only print and check an existing file
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if os.path.isdir(os.path.join(os.path.dirname(HERE), 'sim_worlds')):
    sys.path.insert(0, os.path.dirname(HERE))              # running from the source tree

from sim_worlds import mission, planner                  # noqa: E402
from sim_worlds.world_config import load_world_config   # noqa: E402


def check_file(path, alt_min=None, alt_max=None, home=None):
    """Print a mission file as a table plus the team-code indices; return the list of shape problems."""
    items = mission.read_waypoints(path)
    print(mission.describe(items, home))
    idx = mission.team_indices(items)
    print(f'team indices: takeoff {idx["takeoff_index"]}, DigiCamCtrl {idx["digicam_indices"]}, buffer_wp (photos stop) '
          f'{idx["buffer_wp"]}, hold (main_controller GUIDED) {idx["last_before_rtl"]}, RTL {idx["rtl_index"]}, '
          f'targets inserted at {idx["insert_index"]}')
    problems = mission.check_team_shape(items, alt_min, alt_max)
    print('shape check: ' + ('OK' if not problems else 'PROBLEMS:\n  - ' + '\n  - '.join(problems)))
    return problems


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('world', nargs='?', help='world name (runway, suas_field) or a config YAML path')
    ap.add_argument('--check', metavar='FILE', help='only read, print and check this .waypoints file')
    ap.add_argument('--area', help='search area name (default: the config survey area)')
    ap.add_argument('--altitude', type=float, help='survey altitude above home [m]')
    ap.add_argument('--sidelap', type=float, help='photo overlap between lanes (0..0.9)')
    ap.add_argument('--frontlap', type=float, help='photo overlap along a lane (0..0.9)')
    ap.add_argument('--photo-interval', type=float, help='seconds between delivered photos (speed is derived)')
    ap.add_argument('--speed', type=float, help='lane speed [m/s] (0 = no DO_CHANGE_SPEED item)')
    ap.add_argument('--lead-in', type=float, help='seconds flown on lane 1 before the area (team camera set-up time)')
    ap.add_argument('--out', help='output file (default: the config mission path)')
    args = ap.parse_args(argv)

    if args.check:
        home = None
        if args.world:
            home = load_world_config(args.world).home
        return 1 if check_file(args.check, home=home) else 0
    if not args.world:
        ap.error('give a world name (or --check FILE)')
    cfg = load_world_config(args.world)
    survey_cfg = cfg.survey
    if not survey_cfg:
        ap.error(f'{args.world}: the config has no "survey" section')
    area_name = args.area or survey_cfg.get('area')
    areas = {a['name']: a for a in cfg.search_areas}
    if area_name not in areas:
        ap.error(f'unknown area "{area_name}"; the config has: {", ".join(areas)}')
    settings = planner.settings_from_config(survey_cfg, altitude_m=args.altitude, sidelap=args.sidelap,
                                            frontlap=args.frontlap, photo_interval_s=args.photo_interval,
                                            speed_mps=args.speed, lead_in_s=args.lead_in)
    items, plan, speed = planner.plan_survey_mission(cfg.home, planner.rect_from_area(areas[area_name]), settings)
    out = args.out or cfg.mission
    mission.write_waypoints(out, items)
    s = planner.summary(items, plan, speed, cfg.home)
    print(f'wrote {out}: {s["items"]} items, {s["lanes"]} lanes {s["lane_spacing_m"]} m apart at {s["altitude_m"]:g} m '
          f'(footprint {s["footprint_m"][0]} x {s["footprint_m"][1]} m, {s["gsd_cm_per_px"]} cm/px), '
          f'{speed} m/s, survey {s["survey_path_m"]} m, whole flight {s["total_path_m"]} m')
    problems = check_file(out, settings.altitude_m - 1e-6, settings.altitude_m + 1e-6, cfg.home)
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
