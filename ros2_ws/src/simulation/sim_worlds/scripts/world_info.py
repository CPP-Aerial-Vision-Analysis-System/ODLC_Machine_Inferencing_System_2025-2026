#!/usr/bin/env python3
"""
sim_worlds/scripts/world_info.py -- PURPOSE: print facts about a world from its config/<world>.yaml so shell
scripts and launch files never hard-code paths or coordinates.

usage:
  world_info.py --list                     the worlds that exist (one per line)
  world_info.py runway                     everything, as YAML (absolute paths)
  world_info.py runway home                one field: world_file | world_name | home | spawn | spawn_x |
                                           spawn_y | spawn_z | spawn_yaw | mission | truth | share | models
Examples (from a shell):
  HOME_ARG=$(ros2 run sim_worlds world_info.py runway home)        # -> 36.2143000,-96.0068000,210.0,0
  WORLD=$(ros2 run sim_worlds world_info.py suas_field world_file)
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if os.path.isdir(os.path.join(os.path.dirname(HERE), 'sim_worlds')):
    sys.path.insert(0, os.path.dirname(HERE))              # running from the source tree

import yaml                                              # noqa: E402

from sim_worlds.world_config import available_worlds, load_world_config   # noqa: E402


def field_value(cfg, field):
    """The printable value of one field of a WorldConfig (raises KeyError for unknown fields)."""
    sp = cfg.spawn
    values = {'world_file': cfg.world_file, 'world_name': cfg.world_name, 'home': cfg.home.sitl_string(),
              'spawn': f'{sp["x"]} {sp["y"]} {sp["z"]} {sp["yaw"]}', 'spawn_x': sp['x'], 'spawn_y': sp['y'],
              'spawn_z': sp['z'], 'spawn_yaw': sp['yaw'], 'mission': cfg.mission, 'truth': cfg.truth,
              'share': cfg.share, 'models': os.path.join(cfg.share, 'models')}
    return values[field]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('world', nargs='?', help='world name or config YAML path')
    ap.add_argument('field', nargs='?', help='one field to print (see the help text)')
    ap.add_argument('--list', action='store_true', help='list the available worlds')
    args = ap.parse_args(argv)
    if args.list:
        print('\n'.join(available_worlds()))
        return 0
    if not args.world:
        ap.error('give a world name, or --list')
    try:
        cfg = load_world_config(args.world)
    except (FileNotFoundError, ValueError) as exc:
        print(f'world_info: {exc}', file=sys.stderr)
        return 2
    if args.field:
        try:
            print(field_value(cfg, args.field))
        except KeyError:
            print(f'world_info: unknown field "{args.field}"', file=sys.stderr)
            return 2
        return 0
    d = cfg.as_dict()
    d['models'] = os.path.join(cfg.share, 'models')
    d['search_areas'] = cfg.search_areas
    print(yaml.safe_dump(d, sort_keys=False, default_flow_style=None, width=120), end='')
    return 0


if __name__ == '__main__':
    sys.exit(main())
