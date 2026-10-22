#!/usr/bin/env python3
"""
sim_worlds/scripts/build_real_field.py -- PURPOSE: build the REAL-TERRAIN SUAS 2026 field world
"suas_field_real" (the competition site at Skyway36, Tulsa OK) from real, public data: USGS lidar ground and
trees, the USDA NAIP 2023 aerial photo, OpenStreetMap / Microsoft buildings, the handbook's runway and search
boundary coordinates, and CC0 ground-detail pictures. Everything is reproducible; the full story (data,
licences, how to change the targets, troubleshooting) is in docs/simulation/WORLDS.md.

It runs in four stages. You normally need only stage 3 (done for you by the setup build) and stage 4 (only
when you want different targets):

  stage 1  prepare   raw downloads -> prepared grids            needs the GIS Python + fetch_real_field_data.sh
                     (~/.astra_sim/world_data/suas_field_real/raw -> .../prepared), about 10 minutes
  stage 2  pack      prepared grids -> the SITE PACK            needs numpy, Pillow, scipy
                     sim_worlds/data/suas_field_real/ (about 12 MB, COMMITTED to git)
  stage 3  assets    site pack -> Gazebo models                 needs numpy + Pillow only (ROS machine is fine)
                     terrain tiles, ground pictures, trees, buildings -> <out>/suas_field_real_* (NOT in git,
                     about 1 to 2 minutes; the setup build runs it: see "assets --install")
  stage 4  world     site pack -> world + config + truth + missions   needs numpy + PyYAML
                     worlds/suas_field_real.sdf, config/, truth/, missions/ (COMMITTED, small)

  check    offline checks (files, sizes, counts, alignment, missions vs terrain)
  preview  pictures to look at: top-down ground with trees and targets, hillshade, terrain profile

usage examples (from the sim_worlds source folder):
  python3 scripts/build_real_field.py world --seed 7            # different targets, same terrain
  python3 scripts/build_real_field.py assets --install          # (re)build the Gazebo models into the install
  python3 scripts/build_real_field.py assets --out /tmp/rf      # ... or into any folder
  ~/.astra_sim/gis_venv/bin/python scripts/build_real_field.py prepare   # stage 1 (maintainers)
  python3 scripts/build_real_field.py check
"""

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_SRC = os.path.dirname(HERE)
if os.path.isdir(os.path.join(PKG_SRC, 'sim_worlds')):
    sys.path.insert(0, PKG_SRC)                       # running from the source tree


def default_out_dir():
    """The package source folder when run from it (where package.xml is), else the current folder."""
    return PKG_SRC if os.path.isfile(os.path.join(PKG_SRC, 'package.xml')) else os.getcwd()


def log(msg):
    """Print with a time stamp, at once (also when the output goes to a log file)."""
    print(time.strftime('%H:%M:%S ') + msg, flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('prepare', help='stage 1: raw downloads -> prepared grids (GIS Python)')
    p.add_argument('--data', default=None, help='data folder (default $ASTRA_WORLD_DATA or ~/.astra_sim/world_data)')
    p.add_argument('--skip-lidar', action='store_true', help='reuse the canopy model of an earlier run (saves ~5 min)')

    p = sub.add_parser('pack', help='stage 2: prepared grids -> the committed site pack')
    p.add_argument('--data', default=None, help='data folder (default $ASTRA_WORLD_DATA or ~/.astra_sim/world_data)')
    p.add_argument('--out-dir', default=None, help='package folder to write into (default: this source package)')

    p = sub.add_parser('assets', help='stage 3: site pack -> Gazebo models (terrain, ground pictures, trees)')
    g = p.add_mutually_exclusive_group()
    g.add_argument('--out', default=None, help='models folder to write into (default: <package>/models)')
    g.add_argument('--install', action='store_true',
                   help='write into the INSTALLED sim_worlds share/models (found with ament_index), the setup way')
    p.add_argument('--quality', choices=('hd', 'standard'), default='hd',
                   help='hd = 3.2 cm/px ground pictures in the search areas (default); standard = 6.4 cm/px, '
                        'half the graphics memory there')
    p.add_argument('--if-needed', action='store_true', help='do nothing when the models are already up to date')
    p.add_argument('--pack', default=None, help='site pack folder (default: <package>/data/suas_field_real)')

    p = sub.add_parser('world', help='stage 4: site pack -> world SDF, config, truth and missions')
    p.add_argument('--seed', type=int, default=None, help='layout seed of targets and debris (default 2026)')
    p.add_argument('--assets', choices=('fuel', 'primitive'), default='fuel',
                   help='fuel = realistic Gazebo Fuel models (default); primitive = simple shapes, fully offline')
    p.add_argument('--hiker-poses', default=None,
                   help='comma list of 2 poses (SB1,SB2) from lying_face_up, lying_face_down, sitting, standing')
    p.add_argument('--tent-colours', default=None, help='comma list of 2 tent colours (SB1,SB2), see WORLDS.md')
    p.add_argument('--altitude-margin', type=float, default=1.0,
                   help='extra metres above the 150 ft floor over the highest ground of each area (default 1)')
    p.add_argument('--out-dir', default=None, help='package folder to write into (default: this source package)')
    p.add_argument('--pack', default=None, help='site pack folder (default: <package>/data/suas_field_real)')

    p = sub.add_parser('check', help='offline checks (no Gazebo)')
    p.add_argument('--models', default=None, help='models folder to check (default: <package>/models, then the install)')
    p.add_argument('--pack', default=None, help='site pack folder (default: <package>/data/suas_field_real)')

    p = sub.add_parser('preview', help='pictures: top-down ground + trees + targets, hillshade, terrain profile')
    p.add_argument('--out', default='real_field_preview', help='folder for the PNG/JPG files')
    p.add_argument('--models', default=None, help='models folder (to draw the baked ground pictures)')
    p.add_argument('--pack', default=None, help='site pack folder (default: <package>/data/suas_field_real)')

    p = sub.add_parser('all', help='pack + assets + world + check (after prepare)')
    p.add_argument('--data', default=None)
    p.add_argument('--out', default=None, help='models folder for the assets (default: <package>/models)')
    p.add_argument('--seed', type=int, default=None)

    args = ap.parse_args(argv)
    t0 = time.time()
    pkg = default_out_dir()
    if args.cmd == 'prepare':
        from sim_worlds.realfield import prepare
        prepare.run(args.data, log=log, skip_lidar=args.skip_lidar)
    elif args.cmd == 'pack':
        from sim_worlds.realfield import pack
        pack.run(args.data, os.path.abspath(args.out_dir or pkg), log=log)
    elif args.cmd == 'assets':
        from sim_worlds.realfield import assets_build
        out = assets_build.resolve_out(args.out, args.install, pkg)
        assets_build.run(out, pack_dir=args.pack, quality=args.quality, if_needed=args.if_needed, log=log)
    elif args.cmd == 'world':
        from sim_worlds.realfield import world_build
        world_build.run(os.path.abspath(args.out_dir or pkg), pack_dir=args.pack, seed=args.seed,
                        asset_mode=args.assets, hiker_poses=args.hiker_poses, tent_colours=args.tent_colours,
                        altitude_margin=args.altitude_margin, log=log)
    elif args.cmd == 'check':
        from sim_worlds.realfield import checks
        return checks.run(pkg, models=args.models, pack_dir=args.pack, log=log)
    elif args.cmd == 'preview':
        from sim_worlds.realfield import previews
        previews.run(pkg, os.path.abspath(args.out), models=args.models, pack_dir=args.pack, log=log)
    elif args.cmd == 'all':
        from sim_worlds.realfield import assets_build, checks, pack, world_build
        pack.run(args.data, pkg, log=log)
        assets_build.run(assets_build.resolve_out(args.out, False, pkg), log=log)
        world_build.run(pkg, seed=args.seed, log=log)
        checks.run(pkg, models=args.out, log=log)
    log(f'build_real_field {args.cmd}: done in {time.time() - t0:.0f} s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
