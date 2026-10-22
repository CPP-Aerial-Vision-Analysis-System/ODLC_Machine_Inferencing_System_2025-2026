#!/usr/bin/env python3
"""
sim_worlds/scripts/build_suas_field.py -- PURPOSE: generate the SUAS 2026 competition field world and its files:
worlds/suas_field.sdf, config/suas_field.yaml, truth/suas_field.yaml, missions/suas_field_survey.waypoints
(Search Boundary 1, the contract mission) and missions/suas_field_sb2_survey.waypoints (Search Boundary 2).

The field copies the RoboNation SUAS 2026 site (Skyway Range near Tulsa, Oklahoma) from the GPS numbers of the
Team Handbook: the flight boundary, Search Boundaries 1 and 2, Runway 1 (asphalt strip 02/20) and Runway 2 (the
VTOL pad = the world origin = SITL home). Each search boundary holds ONE hiker (Rescue Randy mannequin, lying,
sitting or standing; the rules: "any orientation ... surrounded/covered by bushes/trees/vehicles") and ONE open
pop-up tent, hidden among debris, vehicles, bushes and a few standing people. A creek with a tree line runs
along the north, more trees line the west and south-east, and dry-grass and dirt patches break up the pasture.

The survey flies at 50 m (164 ft): inside the SUAS 150-400 ft band, close to the team's 49 m drop altitude,
and the hiker is still about 55 pixels long in a 2560 x 1440 photo.

Salvaged from the first twin prototype (suas_twin/scripts/build_suas_field.py, an unfinished draft): the
geography, the target placement rules and the debris list are kept; the occupancy map, the tour, walker's
tree models and the click-to-fly GUI are gone; models are realistic Gazebo Fuel meshes (see worldgen/assets.py).

Everything random comes from --seed (default 2026): the same seed always gives the same field.
usage:  build_suas_field.py [--seed 2026] [--assets fuel|primitive] [--textures] [--out-dir DIR]
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
from sim_worlds.worldgen import assets, meshes, primitives, sdf, textures, world   # noqa: E402
from sim_worlds.worldgen.builder import WorldBuilder, write_model_files, write_shared_models   # noqa: E402
from sim_worlds.worldgen.layout import Placer        # noqa: E402

WORLD = 'suas_field'
DEFAULT_SEED = 2026

# --- competition geography (RoboNation SUAS 2026 Team Handbook) ---------------------------------------------------
HOME = geo.Home(36.2143, -96.0068, 210.0, 0.0)        #: Runway 2 / VTOL pad centre (estimate, +-30 m), ground MSL
FLIGHT_BOUNDARY_LATLON = [
    (36.2193143935478, -96.00171089172362), (36.2203876816226, -96.0041356086731),
    (36.2208031440207, -96.006817817688), (36.2205954130975, -96.0094571113586),
    (36.2198164172223, -96.01181745529171), (36.2181891563676, -96.0142421722412),
    (36.2149951257415, -96.0109269618988), (36.2128397305797, -96.0107660293579),
    (36.2128051052733, -96.00916743278502), (36.2106063669373, -96.00924253463749),
    (36.2106583063506, -96.00173234939579),
]
SEARCH_BOUNDARIES_LATLON = {
    'search_boundary_1': {'lat_min': 36.211822, 'lat_max': 36.213147, 'lon_min': -96.005545, 'lon_max': -96.002627},
    'search_boundary_2': {'lat_min': 36.213469, 'lat_max': 36.215795, 'lon_min': -96.004286, 'lon_max': -96.002635},
}
RUNWAY1_CENTER_LATLON = (36.2150, -96.0065)           #: 40 x 500 ft paved strip (estimate)
RUNWAY1_HEADING_DEG = 20.0                            #: the strip runs about 020 degrees true
MIN_AGL_M, MAX_AGL_M = 45.7, 121.9                    #: 150 ft .. 400 ft
SURVEY = planner.SurveySettings(altitude_m=50.0, sidelap=0.3, frontlap=0.6, photo_interval_s=2.0)
HIKER_POSE_WEIGHTS = {'lying_face_up': 0.35, 'lying_face_down': 0.25, 'sitting': 0.3, 'standing': 0.1}


def default_out_dir():
    """The package source folder when run from it (where package.xml is), else the current folder."""
    return PKG_SRC if os.path.isfile(os.path.join(PKG_SRC, 'package.xml')) else os.getcwd()


def search_rect(name):
    """A search boundary as a survey.Rect in local metres; lanes run along its LONG side."""
    b = SEARCH_BOUNDARIES_LATLON[name]
    x0, y0 = geo.latlon_to_enu(b['lat_min'], b['lon_min'], HOME)
    x1, y1 = geo.latlon_to_enu(b['lat_max'], b['lon_max'], HOME)
    cx, cy, sx, sy = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
    return survey.Rect(cx, cy, sx, sy, 0.0) if sx >= sy else survey.Rect(cx, cy, sy, sx, math.pi / 2)


def bush_scale(rng):
    """Scale of an oak tree that reads as a 1.5-3 m bush from the air (low and round)."""
    s = rng.uniform(0.18, 0.32)
    return (s, s, 0.62 * s)


def creek_line(poly):
    """Centre line of the creek (visual only) arching along the north, kept 25 m inside the flight boundary."""
    pts = []
    for x in np.arange(-560.0, 440.1, 20.0):
        base = 640.0 - 0.00045 * (x + 60.0) ** 2
        y = base + 18.0 * math.sin(x / 95.0) + 9.0 * math.sin(x / 37.0 + 1.3)
        if geo.inside_with_margin(float(x), y, poly, 25.0):
            pts.append((float(x), float(y)))
    return pts


def write_creek_model(models_dir, line, rng):
    """models/suas_field_creek: the creek ribbon mesh and its water/mud texture (this world only)."""
    d = os.path.join(models_dir, 'suas_field_creek')
    os.makedirs(os.path.join(d, 'meshes'), exist_ok=True)
    os.makedirs(os.path.join(d, 'materials', 'textures'), exist_ok=True)
    meshes.ribbon_mesh(os.path.join(d, 'meshes', 'creek.obj'), line, 9.0, 0.04, 20.0, 'creek strip, 9 m wide')
    tex = os.path.join(d, 'materials', 'textures', 'creek_albedo.jpg')
    if not os.path.isfile(tex):
        textures.save_jpg(textures.creek_texture(rng), tex)
    write_model_files(d, 'suas_field_creek', '''      <visual name="water">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>meshes/creek.obj</uri></mesh></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
          <pbr><metal><albedo_map>materials/textures/creek_albedo.jpg</albedo_map>
            <roughness>0.35</roughness><metalness>0.0</metalness></metal></pbr>
        </material>
      </visual>''', 'the creek along the north of the SUAS field (visual only, 9 m wide water and mud banks).',
                 'SUAS field creek strip (visual only).')


def build(seed=DEFAULT_SEED, asset_mode='fuel', log=print, models_dir=None):
    """Lay out the SUAS field. Returns a filled WorldBuilder (nothing written yet except the creek model)."""
    rng = np.random.default_rng(seed)
    fb = np.random.default_rng(seed + 1000)               # randomness for the offline fallbacks only
    b = WorldBuilder(WORLD, HOME, seed, asset_mode,
                     description='SUAS 2026 field (Skyway Range): 2 search boundaries, each with one hiker and one tent')
    poly = [geo.latlon_to_enu(lat, lon, HOME) for lat, lon in FLIGHT_BOUNDARY_LATLON]
    sbs = {name: search_rect(name) for name in SEARCH_BOUNDARIES_LATLON}
    b.gui_camera = world.look_at((-55.0, 45.0, 42.0), (60.0, -70.0, 0.0))   # pad + flight line, SB1 beyond
    b.add_include('ground', 'model://sim_worlds_ground', (0.0, 0.0, 0.0))
    b.add_include('runway_2_pad', 'model://sim_worlds_pad', (0.0, 0.0, 0.0))
    ryaw = geo.heading_to_yaw(RUNWAY1_HEADING_DEG)
    rcx, rcy = geo.latlon_to_enu(*RUNWAY1_CENTER_LATLON, HOME)
    rcx += 30.0 * math.cos(ryaw)                          # slide 30 m along its axis so the pad is clear of it
    rcy += 30.0 * math.sin(ryaw)
    b.add_include('runway_1', 'model://sim_worlds_runway', (rcx, rcy, 0.0), (0.0, 0.0, ryaw))
    line = creek_line(poly)
    if models_dir:
        write_creek_model(models_dir, line, np.random.default_rng(seed + 5))
    b.add_include('creek', 'model://suas_field_creek', (0.0, 0.0, 0.0))
    for name, rect in sbs.items():
        b.add_search_area(name, rect, 'one hiker + one tent among debris, vehicles, bushes and people')

    def person_fallback():
        return primitives.person_parts(fb, *(primitives.pick(fb, p)[1] for p in (
            primitives.HIVIS_COLOURS, primitives.SHIRT_COLOURS, primitives.PANTS_COLOURS, primitives.SKIN_TONES,
            primitives.HAIR_COLOURS)))

    def vehicle_fallback(kind):
        if kind == 'pickup':
            return lambda: primitives.pickup_parts(primitives.pick(fb, primitives.CAR_COLOURS)[1])
        return lambda: primitives.sedan_parts(primitives.pick(fb, primitives.CAR_COLOURS)[1])

    # --- flight line: canopies, parked cars, judges, cones around the pad ---------------------------------------
    perp = (-math.sin(ryaw), math.cos(ryaw))              # left of the runway direction (north-west side)
    canopies = [(rcx + perp[0] * 22.0, rcy + perp[1] * 22.0), (-18.0, -6.0)]
    for i, (px, py) in enumerate(canopies):
        b.add_asset('canopy', f'flightline_canopy_{i + 1}', px, py, ryaw, fallback=primitives.canopy_parts)
        b.add_asset('folding_table', f'flightline_table_{i + 1}', px, py, ryaw, fallback=primitives.table_parts)
    for i in range(3):
        base = canopies[0] if i < 2 else canopies[1]
        off = 9.0 + 7.0 * (i % 2)
        kind = assets.VEHICLES[int(rng.integers(len(assets.VEHICLES)))]
        b.add_asset(kind, f'flightline_{kind}_{i + 1}', base[0] + perp[0] * off, base[1] + perp[1] * off,
                    ryaw + math.pi / 2 + rng.uniform(-0.1, 0.1), role='distractor', area='flight_line',
                    fallback=vehicle_fallback(kind))
    for i in range(2):
        base = canopies[i]
        key = assets.PEOPLE[int(rng.integers(len(assets.PEOPLE)))]
        b.add_asset(key, f'flightline_person_{i + 1}', base[0] + rng.uniform(-4, 4), base[1] + rng.uniform(-4, 4) - 3.0,
                    rng.uniform(-math.pi, math.pi), role='distractor', area='flight_line',
                    meta={'note': 'judge / team member at the flight line'}, fallback=person_fallback)
    half = 12.2 / 2 + 1.5
    for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1))):
        b.add_asset('cone', f'pad_cone_{i + 1}', sx * half, sy * half, 0.0, fallback=primitives.cone_parts)

    # --- search boundaries ---------------------------------------------------------------------------------------
    used_poses = []
    for sb_i, (sb_name, rect) in enumerate(sbs.items(), start=1):
        tag = f'sb{sb_i}'
        for ci, (cx, cy) in enumerate(rect.corners()):
            b.add_asset('cone', f'{tag}_corner_cone_{ci + 1}', cx, cy, 0.0, fallback=primitives.cone_parts)
            parts, _ = primitives.corner_pole_parts()
            b.add_object(f'{tag}_corner_pole_{ci + 1}', parts, cx + 0.6, cy + 0.6, 0.0, kind='marker')
        placer = Placer(rng, rect, inset=4.0)
        # the tent, then the hiker >= 30 m from it, both >= 10 m inside the boundary
        tx, ty = placer.place(2.5, gap=3.0, inset=10.0)
        colour = list(primitives.TENT_COLOURS)[int(rng.integers(len(primitives.TENT_COLOURS)))]
        b.add_tent(f'{tag}_tent', tx, ty, rng.uniform(-math.pi, math.pi), colour, sb_name)
        # the two hikers never share a pose (SB2 draws from the poses SB1 did not get)
        poses = [p for p in HIKER_POSE_WEIGHTS if p not in used_poses]
        w = np.array([HIKER_POSE_WEIGHTS[p] for p in poses])
        pose = poses[int(rng.choice(len(poses), p=w / w.sum()))]
        used_poses.append(pose)
        mode = ['open', 'near_bush', 'near_vehicle'][int(rng.integers(3))]
        hx, hy = placer.place(1.8, gap=3.0, inset=10.0, extra_ok=lambda x, y: math.hypot(x - tx, y - ty) >= 30.0)
        hyaw = rng.uniform(-math.pi, math.pi)
        b.add_hiker(pose, f'{tag}_hiker', hx, hy, hyaw, sb_name, meta={'near': mode})
        side = 1.0 if rng.random() < 0.5 else -1.0
        perp_h = (math.cos(hyaw + side * math.pi / 2), math.sin(hyaw + side * math.pi / 2))
        if mode == 'near_bush':                           # a bush right beside the body, canopy partly over it
            bx, by = hx + 2.0 * perp_h[0], hy + 2.0 * perp_h[1]   # partly covering, never hiding it
            b.add_asset('oak_tree', f'{tag}_bush_by_hiker', bx, by, rng.uniform(0, 6.28), scale=(0.3, 0.3, 0.19),
                        role='distractor', area=sb_name, collision=False, fallback=lambda: primitives.bush_parts(fb, 3.0))
            placer.reserve(bx, by, 1.6)
        elif mode == 'near_vehicle':                      # a vehicle parked right next to the body
            kind = assets.VEHICLES[int(rng.integers(len(assets.VEHICLES)))]
            hw = assets.asset(kind).size()[1] / 2.0
            vx, vy = hx + (hw + 0.9) * perp_h[0], hy + (hw + 0.9) * perp_h[1]
            b.add_asset(kind, f'{tag}_{kind}_by_hiker', vx, vy, hyaw, role='distractor', area=sb_name,
                        meta={'note': 'parked right next to the hiker'}, fallback=vehicle_fallback(kind))
            placer.reserve(vx, vy, 3.2)
        log(f'  {sb_name}: {colour} tent at ({tx:.1f}, {ty:.1f}); hiker ({pose}, {mode}) at ({hx:.1f}, {hy:.1f}), '
            f'{math.hypot(hx - tx, hy - ty):.0f} m apart')
        # vehicles (1-3), standing people (1-3), bushes (5-8)
        for i in range(int(rng.integers(1, 4))):
            kind = assets.VEHICLES[int(rng.integers(len(assets.VEHICLES)))]
            vx, vy = placer.place(3.2, gap=2.0)
            b.add_asset(kind, f'{tag}_{kind}_{i + 1}', vx, vy, rng.uniform(-math.pi, math.pi), role='distractor',
                        area=sb_name, fallback=vehicle_fallback(kind))
        for i in range(int(rng.integers(1, 4))):
            key = assets.PEOPLE[int(rng.integers(len(assets.PEOPLE)))]
            px, py = placer.place(0.6, gap=1.5)
            b.add_asset(key, f'{tag}_person_{i + 1}', px, py, rng.uniform(-math.pi, math.pi), role='distractor',
                        area=sb_name, meta={'standing': True}, fallback=person_fallback)
        for i in range(int(rng.integers(5, 9))):
            s = bush_scale(rng)
            bx, by = placer.place(4.5 * s[0] + 0.2, gap=1.5)
            b.add_asset('oak_tree', f'{tag}_bush_{i + 1}', bx, by, rng.uniform(0, 6.28), scale=s, role='distractor',
                        area=sb_name, collision=False, fallback=lambda: primitives.bush_parts(fb))
        # debris
        plan = ([('log', lambda: primitives.log_parts(rng))] * int(rng.integers(6, 11))
                + [('plywood', lambda: primitives.sheet_parts(rng, 'plywood'))] * int(rng.integers(1, 3))
                + [('roof_sheet', lambda: primitives.sheet_parts(rng, 'roof'))] * int(rng.integers(1, 3))
                + [('lumber', lambda: primitives.lumber_parts(rng))] * int(rng.integers(2, 4))
                + [('tarp', lambda: primitives.tarp_parts(rng, primitives.pick(rng, primitives.TARP_COLOURS)[1]))]
                * int(rng.integers(1, 3))
                + [('tire', lambda: primitives.tire_parts(rng))] * int(rng.integers(2, 5))
                + [('bin', lambda: primitives.bin_parts(rng, primitives.pick(rng, primitives.BIN_COLOURS)[1]))]
                * int(rng.integers(1, 3))
                + [('cooler', lambda: primitives.cooler_parts(rng, primitives.pick(rng, primitives.COOLER_COLOURS)[1]))]
                * int(rng.integers(1, 3))
                + [('carton', lambda: primitives.box_parts(rng))] * int(rng.integers(2, 5))
                + [('mattress', lambda: primitives.mattress_parts(rng))] * int(rng.integers(0, 2))
                + [('pallet', lambda: primitives.pallet_parts(rng))] * int(rng.integers(1, 3)))
        counts = {}
        for kind, maker in plan:
            counts[kind] = counts.get(kind, 0) + 1
            parts, meta = maker()
            r = 0.5 * math.hypot(meta['size'][0], meta['size'][1])
            dx, dy = placer.place(r, gap=0.8)
            b.add_object(f'{tag}_{kind}_{counts[kind]}', parts, dx, dy, rng.uniform(-math.pi, math.pi),
                         role='distractor', kind='debris', area=sb_name, size=meta['size'], description=kind,
                         meta={'type': kind})
        for key, n in (('folding_chair', int(rng.integers(1, 4))), ('drum', int(rng.integers(1, 3))),
                       ('barrel', int(rng.integers(0, 2)))):
            for i in range(n):
                a = assets.asset(key)
                x, y = placer.place(a.radius() + 0.2, gap=1.0)
                fbk = {'folding_chair': lambda: primitives.folding_chair_parts(),
                       'drum': lambda: primitives.barrel_parts(),
                       'barrel': lambda: primitives.barrel_parts((0.95, 0.40, 0.10), 1.0, 0.33)}[key]
                b.add_asset(key, f'{tag}_{key}_{i + 1}', x, y, rng.uniform(-math.pi, math.pi), role='distractor',
                            area=sb_name, fallback=fbk)
        for i in range(int(rng.integers(2, 4))):
            px, py = placer.place(5.0, gap=2.0)
            b.add_patch('dirt', px, py, rng.uniform(6.0, 12.0), rng.uniform(0, 6.28), i)

    # --- trees: along the creek, two tree lines, a few scattered; bushes around the field --------------------------
    taken = []

    def tree_ok(x, y, r, margin=12.0):
        if not geo.inside_with_margin(x, y, poly, margin):
            return False
        for rect in sbs.values():
            if rect.contains(x, y, -25.0):
                return False
        if math.hypot(x - rcx, y - rcy) < 110 or math.hypot(x, y) < 60:
            return False
        return all(math.hypot(x - tx_, y - ty_) >= r + tr for tx_, ty_, tr in taken)

    def add_tree(group, x, y, key=None):
        key = key or ('oak_tree' if rng.random() < 0.62 else 'pine_tree')
        scale = rng.uniform(0.9, 1.5) if key == 'oak_tree' else rng.uniform(2.0, 3.0)
        r = 4.5 * scale if key == 'oak_tree' else 1.4 * scale
        if not tree_ok(x, y, r):
            return False
        taken.append((x, y, r))
        b.add_tree(group, key, x, y, rng.uniform(0, 6.28), scale)
        return True

    pts = np.array(line)
    for i in range(len(pts) - 1):
        seg = pts[i + 1] - pts[i]
        seg_len = float(np.hypot(*seg))
        d = seg / seg_len
        n = np.array([-d[1], d[0]])
        t = 0.0
        while t < seg_len:
            p = pts[i] + d * t
            for side in (-1, 1):
                if rng.random() < 0.55:
                    off = rng.uniform(10.0, 18.0)
                    add_tree('creek', float(p[0] + side * n[0] * off + rng.uniform(-3, 3)),
                             float(p[1] + side * n[1] * off + rng.uniform(-3, 3)))
            t += rng.uniform(12.0, 17.0)

    def tree_line(p0, p1, inset, spacing, group):
        p0, p1 = np.array(p0, dtype=float), np.array(p1, dtype=float)
        dd = p1 - p0
        length = float(np.hypot(*dd))
        dd /= length
        nn = np.array([-dd[1], dd[0]])
        t = 0.0
        while t < length:
            p = p0 + dd * t + nn * inset
            add_tree(group, float(p[0] + rng.uniform(-4, 4)), float(p[1] + rng.uniform(-4, 4)))
            t += rng.uniform(spacing * 0.8, spacing * 1.25)

    tree_line(poly[6], poly[5], -28.0, 12.0, 'west_line')
    tree_line(poly[7], poly[6], -28.0, 13.0, 'west_line')
    tree_line((120.0, -380.0), (430.0, -378.0), 0.0, 13.0, 'south_line')
    tree_line((432.0, 220.0), (432.0, 520.0), 0.0, 14.0, 'east_line')
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    n_sc = 0
    for _ in range(400):
        if n_sc >= 14:
            break
        if add_tree('scattered', rng.uniform(min(xs), max(xs)), rng.uniform(min(ys), max(ys)), 'oak_tree'):
            n_sc += 1
    n_b = 0
    for _ in range(600):
        if n_b >= 30:
            break
        x, y = rng.uniform(min(xs), max(xs)), rng.uniform(min(ys), max(ys))
        if tree_ok(x, y, 2.5, margin=8.0):
            taken.append((x, y, 2.5))
            s = bush_scale(rng)
            b.add_tree('scattered_bushes', 'oak_tree', x, y, rng.uniform(0, 6.28), s)
            n_b += 1
    # --- ground patches: dry and greener grass all over, dirt here and there ---------------------------------------
    for i in range(110):
        x, y = rng.uniform(min(xs) - 120, max(xs) + 120), rng.uniform(min(ys) - 120, max(ys) + 120)
        if math.hypot(x - rcx, y - rcy) < 90 or math.hypot(x, y) < 20:
            continue
        b.add_patch('dry' if i % 3 else 'lush', x, y, rng.uniform(20.0, 45.0), rng.uniform(0, 6.28), i)
    for i in range(12):
        x, y = rng.uniform(min(xs), max(xs)), rng.uniform(min(ys), max(ys))
        if tree_ok(x, y, 10.0, margin=15.0):
            b.add_patch('dirt', x, y, rng.uniform(5.0, 12.0), rng.uniform(0, 6.28), i)
    b.extra_truth = {
        'flight_boundary': {'latlon': [[lat, lon] for lat, lon in FLIGHT_BOUNDARY_LATLON],
                            'local_xy': [[round(x, 3), round(y, 3)] for x, y in poly]},
        'altitude_rule_agl_m': {'min': MIN_AGL_M, 'max': MAX_AGL_M},
        'runway_1': {'center_xy': [round(rcx, 3), round(rcy, 3)], 'heading_deg_true': RUNWAY1_HEADING_DEG,
                     'yaw_rad': round(ryaw, 4), 'length_m': 152.4, 'width_m': 12.2,
                     'note': 'handbook estimate moved 30 m along its axis so the pad is clear of the threshold'},
        'runway_2_pad': {'center_xy': [0.0, 0.0], 'size_m': 12.2, 'note': 'the world origin = SITL home'},
        'creek_centreline_xy': [[round(x, 1), round(y, 1)] for x, y in line],
    }
    return b


PURPOSE = [
    'worlds/suas_field.sdf - PURPOSE: the SUAS 2026 competition field (Skyway Range near Tulsa, OK) for the',
    'search demo. Local frame ENU, origin = centre of the VTOL pad (Runway 2) = SITL home. Flight boundary,',
    'Runway 1 (asphalt 02/20), two search boundaries (one hiker = Rescue Randy mannequin + one open pop-up tent',
    'each, among debris, vehicles, bushes and standing people), a creek with a tree line in the north.',
    'The drone is NOT in this file: the bringup spawns model "iris" at config/suas_field.yaml spawn.',
    'GUI: 3D view + "Drone camera (live)" picture; the bringup makes the view follow the drone.',
    'GENERATED by sim_worlds/scripts/build_suas_field.py - edit that script and re-run it (see README.md).',
]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seed', type=int, default=DEFAULT_SEED, help=f'layout seed (default {DEFAULT_SEED})')
    ap.add_argument('--assets', choices=('fuel', 'primitive'), default='fuel',
                    help='fuel = realistic Gazebo Fuel models (default); primitive = simple shapes, fully offline')
    ap.add_argument('--altitude', type=float, default=None,
                    help=f'survey altitude in m (default {SURVEY.altitude_m:g}; the rules allow {MIN_AGL_M}-{MAX_AGL_M})')
    ap.add_argument('--textures', action='store_true', help='re-paint the shared pictures even if they exist')
    ap.add_argument('--out-dir', default=None, help='package folder to write into (default: this source package)')
    args = ap.parse_args(argv)
    alt = args.altitude or SURVEY.altitude_m
    if not MIN_AGL_M <= alt <= MAX_AGL_M:
        ap.error(f'--altitude {alt} is outside the SUAS band {MIN_AGL_M}-{MAX_AGL_M} m')
    out = os.path.abspath(args.out_dir or default_out_dir())
    print(f'build_suas_field: seed {args.seed}, assets {args.assets}, out {out}')
    write_shared_models(os.path.join(out, 'models'), force=args.textures)
    b = build(args.seed, args.assets, models_dir=os.path.join(out, 'models'))
    settings = planner.SurveySettings(**{**SURVEY.__dict__, 'altitude_m': alt})
    paths = b.write(out, PURPOSE, [('search_boundary_1', '', settings), ('search_boundary_2', '_sb2', settings)])
    for k in ('world', 'config', 'truth', 'mission', 'mission_sb2'):
        print(f'  wrote {paths[k]}')
    print(f'  {len(b.objects)} objects, {sum(len(v) for v in b.tree_groups.values())} trees/bushes, '
          f'{len(b.patches)} ground patches; Fuel models used: {len(assets.fuel_models_used(b.used_assets))}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
