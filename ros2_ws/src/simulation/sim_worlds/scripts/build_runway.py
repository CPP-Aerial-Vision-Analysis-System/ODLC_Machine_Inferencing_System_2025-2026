#!/usr/bin/env python3
"""
sim_worlds/scripts/build_runway.py -- PURPOSE: generate the small, fast "runway" test world and its files:
worlds/runway.sdf, config/runway.yaml, truth/runway.yaml and missions/runway_survey.waypoints.

The world, in plain words: a flat pasture near the real SUAS field (home lat 36.2143, lon -96.0068, ground
210 m above sea level). The take-off pad is at the origin, an asphalt runway runs east-west just south of it,
and a small flight line (canopy, table, chairs, the pilot, a pickup) stands south-west of the pad. North of the
pad is a 120 x 100 m SEARCH BOX marked with cones at its corners. Inside it, 60-120 m from the pad: ONE hiker
(the Rescue Randy mannequin, lying on its back by default) and ONE open pop-up tent, plus a few distractors (a
car, debris, bushes). Trees line the far edge; dry/lush grass and dirt patches break up the ground.

The survey mission flies the search box at 30 m: below the SUAS 150 ft floor ON PURPOSE, because this is a
quick test world: the climb is short, the photos overlap a lot, and the hiker is about 90 pixels long in a
2560 x 1440 photo (about 55 pixels from the 50 m competition height).

Everything random comes from --seed, so the same seed always gives the same world.
usage:  build_runway.py [--seed 7] [--assets fuel|primitive] [--hiker-pose lying_face_up] [--textures] [--out-dir DIR]
"""

import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_SRC = os.path.dirname(HERE)
if os.path.isdir(os.path.join(PKG_SRC, 'sim_worlds')):
    sys.path.insert(0, PKG_SRC)                       # running from the source tree

import numpy as np                                    # noqa: E402

from sim_worlds import geo, planner, survey          # noqa: E402
from sim_worlds.worldgen import assets, primitives, world   # noqa: E402
from sim_worlds.worldgen.builder import WorldBuilder, write_shared_models   # noqa: E402
from sim_worlds.worldgen.layout import Placer        # noqa: E402

WORLD = 'runway'
DEFAULT_SEED = 7
HOME = geo.Home(36.2143, -96.0068, 210.0, 0.0)       #: near the real SUAS 2026 field (Skyway Range, Tulsa)
SEARCH_BOX = survey.Rect(25.0, 95.0, 120.0, 100.0, 0.0)   #: centre (m), 120 m east-west x 100 m north-south
TARGET_RANGE_M = (60.0, 120.0)                        #: hiker and tent distance from the pad


def BUSH_SCALE(s):                                    # noqa: N802 (reads like a constant at the call sites)
    """An oak tree scaled by s across and 0.62 * s in height reads as a low, round bush from the air."""
    return (s, s, 0.62 * s)
# lead_in_s: lane 1 starts 16 s of flight before the search box, so the team's ~15 s camera set-up after
# DigiCamCtrl (zoom rack + focus, no photos meanwhile) is over before the drone reaches the box.
SURVEY = planner.SurveySettings(altitude_m=30.0, sidelap=0.3, frontlap=0.6, photo_interval_s=2.0, lead_in_s=16.0)


def default_out_dir():
    """The package source folder when run from it (where package.xml is), else the current folder."""
    return PKG_SRC if os.path.isfile(os.path.join(PKG_SRC, 'package.xml')) else os.getcwd()


def build(seed=DEFAULT_SEED, asset_mode='fuel', hiker_pose='lying_face_up', log=print):
    """Lay out the runway world. Returns a filled WorldBuilder (nothing written yet)."""
    rng = np.random.default_rng(seed)
    b = WorldBuilder(WORLD, HOME, seed, asset_mode,
                     description='small fast test world: pad, runway, a 120 x 100 m search box with one hiker and one tent')
    b.gui_camera = world.look_at((-45.0, -55.0, 48.0), (25.0, 80.0, 0.0))
    b.add_include('ground', 'model://sim_worlds_ground', (0.0, 0.0, 0.0))
    b.add_include('pad', 'model://sim_worlds_pad', (0.0, 0.0, 0.0))
    b.add_include('runway', 'model://sim_worlds_runway', (0.0, -32.0, 0.0))
    b.add_search_area('search_box', SEARCH_BOX, 'the only search area: one hiker + one tent inside')
    fb = np.random.default_rng(seed + 1000)          # randomness for the offline fallbacks only

    # --- flight line (south-west of the pad, outside the search box) ----------------------------------------------
    b.add_asset('canopy', 'flightline_canopy', -16.0, -12.0, 0.0, fallback=primitives.canopy_parts)
    b.add_asset('folding_table', 'flightline_table', -16.0, -12.3, 0.0, fallback=primitives.table_parts)
    for i, (dx, yaw) in enumerate(((-0.6, -math.pi / 2), (0.6, -math.pi / 2))):
        b.add_asset('folding_chair', f'flightline_chair_{i + 1}', -16.0 + dx, -11.2, yaw,
                    fallback=lambda: primitives.folding_chair_parts())
    pilot = assets.PEOPLE[int(rng.integers(len(assets.PEOPLE)))]
    b.add_asset(pilot, 'pilot', -7.5, -6.0, math.radians(35.0), role='distractor', area='flight_line',
                meta={'note': 'the pilot at the flight line, outside the search box'},
                fallback=lambda: primitives.person_parts(fb, *(primitives.pick(fb, p)[1] for p in (
                    primitives.HIVIS_COLOURS, primitives.SHIRT_COLOURS, primitives.PANTS_COLOURS,
                    primitives.SKIN_TONES, primitives.HAIR_COLOURS))))
    b.add_asset('pickup', 'flightline_pickup', -26.0, -16.0, math.radians(80.0), role='distractor',
                area='flight_line', fallback=lambda: primitives.pickup_parts(primitives.CAR_COLOURS['white']))
    for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1))):    # cones at the pad corners
        b.add_asset('cone', f'pad_cone_{i + 1}', sx * 7.6, sy * 7.6, 0.0, fallback=primitives.cone_parts)

    # --- search box corners: cone + striped pole --------------------------------------------------------------------
    for i, (cx, cy) in enumerate(SEARCH_BOX.corners()):
        b.add_asset('cone', f'box_corner_cone_{i + 1}', cx, cy, 0.0, fallback=primitives.cone_parts)
        parts, _ = primitives.corner_pole_parts()
        b.add_object(f'box_corner_pole_{i + 1}', parts, cx + 0.7, cy + 0.7, 0.0, kind='marker')

    # --- the two targets ---------------------------------------------------------------------------------------------
    placer = Placer(rng, SEARCH_BOX, inset=12.0)
    for x, y, r in ((0.0, 0.0, 15.0),):
        placer.reserve(x, y, r)
    lo, hi = TARGET_RANGE_M

    def in_range(x, y):
        return lo <= math.hypot(x, y) <= hi

    tx, ty = placer.place(2.5, gap=4.0, extra_ok=in_range)
    tent_colour = list(primitives.TENT_COLOURS)[int(rng.integers(len(primitives.TENT_COLOURS)))]
    b.add_tent('tent', tx, ty, rng.uniform(-math.pi, math.pi), tent_colour, 'search_box')
    hx, hy = placer.place(1.6, gap=4.0, extra_ok=lambda x, y: in_range(x, y) and math.hypot(x - tx, y - ty) >= 35.0)
    hyaw = rng.uniform(-math.pi, math.pi)
    b.add_hiker(hiker_pose, 'hiker', hx, hy, hyaw, 'search_box', meta={'near': 'small bush'})
    # a small bush right next to the hiker (the rules: "surrounded/covered by bushes/trees/vehicles")
    side = 1.0 if rng.random() < 0.5 else -1.0
    bx, by = hx + 2.2 * math.cos(hyaw + side * math.pi / 2), hy + 2.2 * math.sin(hyaw + side * math.pi / 2)
    b.add_asset('oak_tree', 'bush_by_hiker', bx, by, rng.uniform(0, 6.28), scale=BUSH_SCALE(0.24), role='distractor',
                area='search_box', collision=False, fallback=lambda: primitives.bush_parts(fb, 2.4))
    placer.reserve(bx, by, 1.6)

    # --- distractors inside the box ----------------------------------------------------------------------------------
    vx, vy = placer.place(3.2, gap=3.0)
    b.add_asset('suv', 'box_suv', vx, vy, rng.uniform(-math.pi, math.pi), role='distractor', area='search_box',
                fallback=lambda: primitives.sedan_parts(primitives.CAR_COLOURS['silver']))
    for i in range(3):
        x, y = placer.place(1.6, gap=2.0)
        b.add_asset('oak_tree', f'box_bush_{i + 1}', x, y, rng.uniform(0, 6.28), scale=BUSH_SCALE(rng.uniform(0.2, 0.3)),
                    role='distractor', area='search_box', collision=False, fallback=lambda: primitives.bush_parts(fb))
    debris = [('barrel', None), ('drum', None), ('drum', None), ('carton', lambda: primitives.box_parts(rng)),
              ('tarp', lambda: primitives.tarp_parts(rng, primitives.pick(rng, primitives.TARP_COLOURS)[1])),
              ('log', lambda: primitives.log_parts(rng)), ('log', lambda: primitives.log_parts(rng)),
              ('pallet', lambda: primitives.pallet_parts(rng)), ('tire', lambda: primitives.tire_parts(rng))]
    fallbacks = {'barrel': lambda: primitives.barrel_parts((0.95, 0.40, 0.10), 1.0, 0.33),
                 'drum': lambda: primitives.barrel_parts()}
    for i, (kind, maker) in enumerate(debris):
        if maker is None:                                  # a Fuel prop
            a = assets.asset(kind)
            x, y = placer.place(a.radius() + 0.2, gap=1.5)
            b.add_asset(kind, f'box_{kind}_{i + 1}', x, y, rng.uniform(-math.pi, math.pi), role='distractor',
                        area='search_box', fallback=fallbacks[kind])
        else:
            parts, meta = maker()
            r = 0.5 * math.hypot(meta['size'][0], meta['size'][1])
            x, y = placer.place(r, gap=1.5)
            b.add_object(f'box_{kind}_{i + 1}', parts, x, y, rng.uniform(-math.pi, math.pi), role='distractor',
                         kind='debris', area='search_box', size=meta['size'], description=kind)
    for i in range(2):                                     # bare dirt inside the box
        x, y = placer.place(4.0, gap=1.0)
        b.add_patch('dirt', x, y, rng.uniform(5.0, 9.0), rng.uniform(0, 6.28), i)

    # --- dry and greener grass patches all around (big, non-repeating variation on top of the ground picture) ------
    for i in range(30):
        x, y = rng.uniform(-260, 300), rng.uniform(-200, 330)
        if abs(y + 32.0) < 12.0 and abs(x) < 85.0:         # not on the runway
            continue
        b.add_patch('dry' if i % 3 else 'lush', x, y, rng.uniform(20.0, 45.0), rng.uniform(0, 6.28), i)
    for i in range(5):
        x, y = rng.uniform(-150, 200), rng.uniform(-100, 230)
        if not SEARCH_BOX.contains(x, y, -8.0) and math.hypot(x, y) > 20:
            b.add_patch('dirt', x, y, rng.uniform(4.0, 10.0), rng.uniform(0, 6.28), i + 1)

    # --- trees: a line beyond the north edge of the box, a few scattered ones ---------------------------------------
    x = -90.0
    while x < 150.0:
        key = 'oak_tree' if rng.random() < 0.65 else 'pine_tree'
        scale = rng.uniform(0.9, 1.4) if key == 'oak_tree' else rng.uniform(2.0, 2.8)
        b.add_tree('north_line', key, x + rng.uniform(-3, 3), 178.0 + rng.uniform(-6, 6), rng.uniform(0, 6.28), scale)
        x += rng.uniform(9.0, 15.0)
    for i in range(9):
        for _ in range(50):
            x, y = rng.uniform(-170, 220), rng.uniform(-110, 170)
            if (not SEARCH_BOX.contains(x, y, -15.0) and math.hypot(x, y) > 45 and not (abs(y + 32) < 25 and abs(x) < 100)):
                key = 'oak_tree' if rng.random() < 0.6 else 'pine_tree'
                b.add_tree('scattered', key, x, y, rng.uniform(0, 6.28),
                           rng.uniform(0.8, 1.3) if key == 'oak_tree' else rng.uniform(1.8, 2.6))
                break
    log(f'  hiker ({hiker_pose}) at ({hx:.1f}, {hy:.1f}) {math.hypot(hx, hy):.0f} m from the pad; '
        f'{tent_colour} tent at ({tx:.1f}, {ty:.1f}) {math.hypot(tx, ty):.0f} m; {math.hypot(hx - tx, hy - ty):.0f} m apart')
    return b


PURPOSE = [
    'worlds/runway.sdf - PURPOSE: the small, fast "runway" test world for the SUAS search demo.',
    'A flat pasture near the real SUAS field; take-off pad at the origin (= SITL home), runway south of it,',
    'flight line south-west, and a 120 x 100 m search box to the north with ONE hiker (Rescue Randy',
    'mannequin) and ONE open pop-up tent among a few distractors. The drone is NOT in this file: the bringup',
    'spawns model "iris" at config/runway.yaml spawn. GUI: 3D view + "Drone camera (live)" picture; the',
    'bringup makes the view follow the drone.',
    'GENERATED by sim_worlds/scripts/build_runway.py - edit that script and re-run it (see README.md).',
]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seed', type=int, default=DEFAULT_SEED, help=f'layout seed (default {DEFAULT_SEED})')
    ap.add_argument('--assets', choices=('fuel', 'primitive'), default='fuel',
                    help='fuel = realistic Gazebo Fuel models (default); primitive = simple shapes, fully offline')
    ap.add_argument('--hiker-pose', choices=assets.HIKER_POSES, default='lying_face_up')
    ap.add_argument('--altitude', type=float, default=None, help=f'survey altitude in m (default {SURVEY.altitude_m:g})')
    ap.add_argument('--textures', action='store_true', help='re-paint the shared pictures even if they exist')
    ap.add_argument('--out-dir', default=None, help='package folder to write into (default: this source package)')
    args = ap.parse_args(argv)
    out = os.path.abspath(args.out_dir or default_out_dir())
    print(f'build_runway: seed {args.seed}, assets {args.assets}, out {out}')
    write_shared_models(os.path.join(out, 'models'), force=args.textures)
    b = build(args.seed, args.assets, args.hiker_pose)
    settings = planner.SurveySettings(**{**SURVEY.__dict__, 'altitude_m': args.altitude or SURVEY.altitude_m})
    paths = b.write(out, PURPOSE, [('search_box', '', settings)])
    for k in ('world', 'config', 'truth', 'mission'):
        print(f'  wrote {paths[k]}')
    print(f'  {len(b.objects)} objects, {sum(len(v) for v in b.tree_groups.values())} trees, '
          f'{len(b.patches)} ground patches; Fuel models used: {len(assets.fuel_models_used(b.used_assets))}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
