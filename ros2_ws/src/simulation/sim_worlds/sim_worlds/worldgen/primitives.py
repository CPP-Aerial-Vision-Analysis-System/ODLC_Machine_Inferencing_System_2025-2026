"""
sim_worlds/worldgen/primitives.py -- PURPOSE: objects built from simple shapes (boxes, cylinders, capsules,
spheres, ellipsoids) or our own small meshes, for the things Gazebo Fuel has no good model of, and as offline
fallbacks for the Fuel models (`--assets primitive`).

Every function returns a list of parts (see worldgen.sdf.part) in the object's own frame: x forward, y left,
z up, the object standing on the ground (z = 0). Most return (parts, meta) where meta is a small dict of facts
that go into the truth file (size, colour, ...).

Contents: the pop-up dome tent (the search target), debris (logs, plywood, roof sheets, lumber, tarps, tyres,
bins, coolers, boxes, mattresses, pallets), bushes, corner markers, and fallbacks: mannequin in 5 poses,
standing person, sedan, pickup, canopy, cone, folding chair.
"""

import math

import numpy as np

from sim_worlds.worldgen.sdf import collision_box, part, parts_bbox, rest_on_ground, rpy_to_matrix, transform_parts

# --- colour palettes (r, g, b in 0..1) --------------------------------------------------------------------------
CAR_COLOURS = {'white': (0.92, 0.92, 0.90), 'silver': (0.70, 0.71, 0.73), 'black': (0.06, 0.06, 0.07),
               'red': (0.70, 0.08, 0.08), 'dark_blue': (0.08, 0.14, 0.40), 'dark_green': (0.10, 0.28, 0.14)}
#: tent colour name -> (panel colour, skirt colour); the skirt is the darker strip along the ground
TENT_COLOURS = {'blue': ((0.13, 0.32, 0.66), (0.10, 0.17, 0.30)), 'green': ((0.20, 0.42, 0.22), (0.12, 0.20, 0.12)),
                'orange': ((0.92, 0.45, 0.12), (0.33, 0.30, 0.28)), 'grey': ((0.56, 0.57, 0.59), (0.27, 0.27, 0.29)),
                'red': ((0.72, 0.14, 0.11), (0.28, 0.11, 0.09)), 'yellow': ((0.90, 0.76, 0.18), (0.34, 0.32, 0.29))}
SHIRT_COLOURS = {'red': (0.75, 0.12, 0.12), 'blue': (0.15, 0.30, 0.70), 'white': (0.90, 0.90, 0.88),
                 'black': (0.08, 0.08, 0.09), 'yellow': (0.90, 0.80, 0.15), 'green': (0.15, 0.50, 0.22),
                 'grey': (0.50, 0.50, 0.52), 'purple': (0.40, 0.15, 0.55)}
PANTS_COLOURS = {'jeans': (0.20, 0.28, 0.45), 'khaki': (0.62, 0.55, 0.40), 'black': (0.08, 0.08, 0.09),
                 'grey': (0.40, 0.40, 0.42), 'olive': (0.35, 0.38, 0.22)}
SKIN_TONES = {'light': (0.85, 0.68, 0.55), 'medium': (0.65, 0.48, 0.35), 'dark': (0.38, 0.26, 0.18)}
HAIR_COLOURS = {'black': (0.07, 0.06, 0.05), 'brown': (0.30, 0.20, 0.12), 'blond': (0.75, 0.62, 0.35)}
HIVIS_COLOURS = {'hivis_yellow': (0.80, 0.95, 0.10), 'hivis_orange': (1.00, 0.42, 0.05)}
TARP_COLOURS = {'blue': (0.10, 0.30, 0.75), 'grey': (0.55, 0.56, 0.58), 'brown': (0.45, 0.32, 0.20),
                'green': (0.15, 0.40, 0.20)}
BIN_COLOURS = {'grey': (0.35, 0.36, 0.38), 'blue': (0.10, 0.25, 0.60), 'black': (0.08, 0.08, 0.09),
               'green': (0.10, 0.35, 0.18)}
COOLER_COLOURS = {'white': (0.92, 0.92, 0.90), 'red': (0.75, 0.10, 0.10), 'blue': (0.15, 0.35, 0.70)}
BUSH_GREENS = [(0.16, 0.25, 0.10), (0.20, 0.30, 0.12), (0.24, 0.31, 0.14), (0.13, 0.21, 0.09)]
WOOD = (0.55, 0.42, 0.26)
PLYWOOD = (0.72, 0.60, 0.40)
ROOF_SHEET = (0.62, 0.63, 0.65)
BARK = (0.33, 0.24, 0.15)
BLACK_RUBBER = (0.05, 0.05, 0.05)
CARDBOARD = (0.68, 0.52, 0.32)
MATTRESS = (0.88, 0.86, 0.80)
CONE_ORANGE = (1.00, 0.35, 0.02)
GLASS = (0.10, 0.12, 0.15)

TENT_SIZE = (2.4, 2.2, 1.35)                   #: pop-up dome tent footprint and height [m]
VEHICLE_SIZES = {'sedan': (4.7, 1.8, 1.42), 'pickup': (5.5, 2.0, 1.85)}
MANNEQUIN_POSES = ('lying_face_up', 'lying_face_down', 'lying_on_side', 'sitting', 'standing')


def pick(rng, mapping):
    """A random (key, value) pair from a dict, using the numpy Generator rng."""
    keys = list(mapping.keys())
    k = keys[int(rng.integers(len(keys)))]
    return k, mapping[k]


def _capsule_between(name, a, b, radius, color):
    """A capsule whose axis runs from point a to point b (object frame)."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    d = b - a
    length = float(np.linalg.norm(d))
    c = 0.5 * (a + b)
    dz = d / length
    yaw = math.atan2(dz[1], dz[0])
    pitch = math.atan2(math.hypot(dz[0], dz[1]), dz[2])       # angle away from +z
    return part(name, 'capsule', (radius, max(length - 2 * radius, 0.02)), c, (0.0, pitch, yaw), color)


# ---------------------------------------------------------------------------------------------------------------
# the search target: an open pop-up dome tent
# ---------------------------------------------------------------------------------------------------------------

def tent_parts(mesh_uri, fabric_uri, colour_rgb):
    """Open pop-up dome tent (2.4 x 2.2 x 1.35 m), door at +x: textured dome, dark floor seen through the door,
    the door flap lying on the ground in front, one collision box. Returns (parts, meta)."""
    a, b, c = TENT_SIZE
    dark = tuple(v * 0.6 for v in colour_rgb)
    parts = [
        part('dome', 'mesh', xyz=(0, 0, 0.0), color=(1.0, 1.0, 1.0), uri=mesh_uri, albedo=fabric_uri,
             double_sided=True, roughness=0.7),
        # elliptical dark floor, inside the dome outline (only seen through the open door)
        part('floor', 'ellipsoid', (a / 2 * 0.96, b / 2 * 0.96, 0.012), (0, 0, 0.0), color=(0.15, 0.15, 0.16)),
        part('door_flap', 'box', (0.70, 0.72, 0.015), (a / 2 + 0.33, 0, 0.0075), color=dark),
        collision_box('tent_box', (a, b, c), (0, 0, c / 2)),
    ]
    return parts, {'size': [a + 0.75, b, c], 'size_dome': [a, b, c], 'door': '+x side, open'}


# ---------------------------------------------------------------------------------------------------------------
# fallback mannequin (only used with --assets primitive; the default is the Fuel Rescue Randy)
# ---------------------------------------------------------------------------------------------------------------

def mannequin_parts(rng, pose_name, colours):
    """A clothed dummy about 1.8 m tall. colours = dict(shirt, pants, skin, hair) of RGB tuples.
    Lying poses: body along +x (head at +x) before the final centring. Returns (parts, meta)."""
    shirt, pants, skin, hair = colours['shirt'], colours['pants'], colours['skin'], colours['hair']
    shoes = (0.12, 0.10, 0.09)
    th = 0.20
    z0 = th / 2
    P = [part('torso', 'box', (0.52, 0.36, th), (0.44, 0, z0), color=shirt),
         part('hips', 'box', (0.22, 0.34, th * 0.9), (0.08, 0, z0 * 0.9), color=pants),
         part('head', 'sphere', (0.11,), (0.84, 0, 0.11), color=skin),
         part('hair', 'sphere', (0.105,), (0.82, 0, 0.085), color=hair),
         part('neck', 'cylinder', (0.05, 0.08), (0.72, 0, 0.10), (0, math.pi / 2, 0), color=skin)]
    arm_spread = rng.uniform(0.25, 1.3)
    arm_bend = rng.uniform(0.0, 1.2)
    leg_spread = rng.uniform(0.05, 0.45)
    leg_bend = rng.uniform(0.0, 0.5)
    for side in (-1, 1):
        lr = 'l' if side > 0 else 'r'
        sh = np.array([0.66, side * 0.21, 0.09])
        ua = np.array([-math.cos(arm_spread), side * math.sin(arm_spread), 0.0])
        elbow = sh + 0.30 * ua
        fa_ang = arm_spread + arm_bend * (1 if rng.random() < 0.5 else -1)
        fa = np.array([-math.cos(fa_ang), side * math.sin(fa_ang), 0.0])
        hand = elbow + 0.27 * fa
        P.append(_capsule_between(f'upper_arm_{lr}', sh, elbow, 0.05, shirt))
        P.append(_capsule_between(f'forearm_{lr}', elbow, hand, 0.045, skin))
        hip = np.array([0.02, side * 0.10, 0.09])
        thd = np.array([-math.cos(leg_spread), side * math.sin(leg_spread), 0.0])
        knee = hip + 0.44 * thd
        sh_ang = leg_spread - leg_bend
        shd = np.array([-math.cos(sh_ang), side * math.sin(sh_ang), 0.0])
        ankle = knee + 0.42 * shd
        P.append(_capsule_between(f'thigh_{lr}', hip, knee, 0.085, pants))
        P.append(_capsule_between(f'shin_{lr}', knee, ankle, 0.065, pants))
        P.append(part(f'shoe_{lr}', 'box', (0.27, 0.11, 0.10), ankle + 0.12 * shd,
                      (0, 0, math.atan2(shd[1], shd[0])), color=shoes))
    if pose_name == 'lying_face_up':
        Q = P
    elif pose_name == 'lying_face_down':
        Q = transform_parts(P, rpy_to_matrix(math.pi, 0, 0))
    elif pose_name == 'lying_on_side':
        Q = transform_parts(P, rpy_to_matrix(math.pi / 2 * (1 if rng.random() < 0.5 else -1), 0, 0))
    elif pose_name == 'sitting':
        upper = [p for p in P if p['name'] in ('torso', 'head', 'hair', 'neck') or 'arm' in p['name']]
        lower = [p for p in P if p not in upper]
        up = transform_parts(upper, rpy_to_matrix(0, math.radians(-75.0), 0), (0.10, 0, 0.02))
        Q = lower + up
    elif pose_name == 'standing':
        Q = transform_parts(P, rpy_to_matrix(0, -math.pi / 2, 0))     # head (+x) turns up (+z)
    else:
        raise ValueError(f'unknown mannequin pose {pose_name}')
    Q = rest_on_ground(Q, 0.0)
    lo, hi = parts_bbox(Q)
    centre = 0.5 * (lo + hi)
    Q = transform_parts(Q, None, (-centre[0], -centre[1], 0.0))       # centre it on the origin
    lo, hi = parts_bbox(Q)
    size = [float(v) for v in (hi - lo)]
    Q.append(collision_box('bbox', size, (0.0, 0.0, size[2] / 2)))
    return Q, {'size': [round(v, 3) for v in size]}


# ---------------------------------------------------------------------------------------------------------------
# fallbacks for Fuel people / vehicles / flight-line props
# ---------------------------------------------------------------------------------------------------------------

def person_parts(rng, vest_rgb, shirt_rgb, pants_rgb, skin_rgb, hair_rgb):
    """A standing person (about 1.78 m) with a hi-vis vest. Returns (parts, meta)."""
    P = [part('leg_l', 'cylinder', (0.075, 0.85), (0, 0.11, 0.425), color=pants_rgb),
         part('leg_r', 'cylinder', (0.075, 0.85), (0, -0.11, 0.425), color=pants_rgb),
         part('torso', 'box', (0.26, 0.44, 0.62), (0, 0, 1.16), color=vest_rgb),
         part('stripe', 'box', (0.28, 0.46, 0.06), (0, 0, 1.25), color=(0.80, 0.80, 0.82)),
         part('arm_l', 'cylinder', (0.045, 0.60), (0, 0.27, 1.15), color=shirt_rgb),
         part('arm_r', 'cylinder', (0.045, 0.60), (0, -0.27, 1.15), color=shirt_rgb),
         part('neck', 'cylinder', (0.05, 0.08), (0, 0, 1.50), color=skin_rgb),
         part('head', 'sphere', (0.11,), (0, 0, 1.64), color=skin_rgb),
         part('hair', 'sphere', (0.105,), (-0.025, 0, 1.655), color=hair_rgb),
         collision_box('person_box', (0.4, 0.6, 1.78), (0, 0, 0.89))]
    return P, {'size': [0.4, 0.6, 1.78]}


def sedan_parts(colour_rgb):
    """A sedan car (4.7 x 1.8 x 1.42 m), front at +x. Returns (parts, meta)."""
    L, W, _ = VEHICLE_SIZES['sedan']
    P = [part('body', 'box', (L, W, 0.55), (0, 0, 0.575), color=colour_rgb, roughness=0.35),
         part('hood', 'box', (1.3, W * 0.92, 0.08), (1.35, 0, 0.89), color=colour_rgb, roughness=0.35),
         part('trunk', 'box', (0.9, W * 0.92, 0.08), (-1.65, 0, 0.89), color=colour_rgb, roughness=0.35),
         part('glass', 'box', (2.5, W * 0.92, 0.45), (-0.10, 0, 1.075), color=GLASS, roughness=0.2),
         part('roof', 'box', (1.5, W * 0.90, 0.06), (-0.15, 0, 1.33), color=colour_rgb, roughness=0.35)]
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        P.append(part(f'wheel_{i}', 'cylinder', (0.32, 0.22), (sx * 1.45, sy * 0.80, 0.32), (math.pi / 2, 0, 0),
                      color=BLACK_RUBBER))
    P.append(collision_box('car_box', (L, W, 1.42), (0, 0, 0.71)))
    return P, {'size': [L, W, 1.42]}


def pickup_parts(colour_rgb):
    """A pickup truck (5.5 x 2.0 x 1.85 m), front at +x. Returns (parts, meta)."""
    L, W, _ = VEHICLE_SIZES['pickup']
    P = [part('body', 'box', (L, W, 0.62), (0, 0, 0.69), color=colour_rgb, roughness=0.35),
         part('hood', 'box', (1.5, W * 0.92, 0.10), (1.95, 0, 1.03), color=colour_rgb, roughness=0.35),
         part('glass', 'box', (1.5, W * 0.92, 0.55), (0.55, 0, 1.275), color=GLASS, roughness=0.2),
         part('roof', 'box', (1.2, W * 0.90, 0.06), (0.45, 0, 1.58), color=colour_rgb, roughness=0.35),
         part('bed_floor', 'box', (2.35, W * 0.84, 0.02), (-1.45, 0, 1.01), color=(0.22, 0.22, 0.23))]
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        P.append(part(f'wheel_{i}', 'cylinder', (0.38, 0.26), (sx * 1.70, sy * 0.86, 0.38), (math.pi / 2, 0, 0),
                      color=BLACK_RUBBER))
    P.append(collision_box('truck_box', (L, W, 1.85), (0, 0, 0.925)))
    return P, {'size': [L, W, 1.85]}


def canopy_parts(colour_rgb=(0.16, 0.45, 0.22)):
    """3 x 3 m pop-up canopy: 4 poles and a flat-topped roof (fallback for the Fuel canopy)."""
    P = [part(f'pole_{i}', 'cylinder', (0.025, 2.0), (sx * 1.45, sy * 1.45, 1.0), color=(0.75, 0.75, 0.78))
         for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1)))]
    P.append(part('roof', 'box', (3.0, 3.0, 0.08), (0, 0, 2.05), color=colour_rgb))
    P.append(part('peak', 'box', (1.6, 1.6, 0.25), (0, 0, 2.2), color=colour_rgb))
    P.append(collision_box('canopy_box', (3.0, 3.0, 2.4), (0, 0, 1.2)))
    return P, {'size': [3.0, 3.0, 2.4]}


def cone_parts():
    """Orange traffic cone 0.7 m (fallback): base, cone (cylinder-ish), white band."""
    P = [part('base', 'box', (0.38, 0.38, 0.03), (0, 0, 0.015), color=(0.05, 0.05, 0.05)),
         part('cone', 'cylinder', (0.12, 0.66), (0, 0, 0.36), color=CONE_ORANGE),
         part('band', 'cylinder', (0.125, 0.10), (0, 0, 0.40), color=(0.95, 0.95, 0.95)),
         collision_box('cone_box', (0.38, 0.38, 0.7), (0, 0, 0.35))]
    return P, {'size': [0.38, 0.38, 0.7]}


def folding_chair_parts(colour_rgb=(0.15, 0.30, 0.65)):
    """Folding camp chair (fallback)."""
    P = [part('seat', 'box', (0.48, 0.50, 0.04), (0, 0, 0.45), color=colour_rgb),
         part('back', 'box', (0.04, 0.50, 0.50), (-0.24, 0, 0.72), (0, -0.2, 0), color=colour_rgb)]
    for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1))):
        P.append(part(f'leg_{i}', 'cylinder', (0.015, 0.45), (sx * 0.22, sy * 0.23, 0.225), color=(0.1, 0.1, 0.1)))
    return P, {'size': [0.55, 0.55, 1.0]}


def table_parts():
    """Folding table 1.8 x 0.75 m (fallback for the Fuel folding table)."""
    P = [part('top', 'box', (1.8, 0.75, 0.04), (0, 0, 0.72), color=(0.85, 0.85, 0.82))]
    for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1))):
        P.append(part(f'leg_{i}', 'cylinder', (0.018, 0.70), (sx * 0.82, sy * 0.30, 0.35), color=(0.1, 0.1, 0.1)))
    return P, {'size': [1.8, 0.75, 0.74]}


def barrel_parts(colour_rgb=(0.12, 0.22, 0.45), height=0.9, radius=0.3):
    """A drum / barrel standing upright (fallback for the Fuel drum and construction barrel)."""
    return [part('drum', 'cylinder', (radius, height), (0, 0, height / 2), color=colour_rgb, roughness=0.5),
            part('rim', 'cylinder', (radius * 1.02, 0.03), (0, 0, height * 0.66), color=tuple(v * 0.7 for v in colour_rgb)),
            collision_box('drum_box', (2 * radius, 2 * radius, height), (0, 0, height / 2))], \
        {'size': [2 * radius, 2 * radius, height]}


def corner_pole_parts():
    """2.5 m orange/white striped pole with a flag: marks a search-boundary corner."""
    P = [part(f'seg_{i}', 'cylinder', (0.03, 0.5), (0, 0, 0.25 + 0.5 * i),
              color=CONE_ORANGE if i % 2 == 0 else (0.95, 0.95, 0.95)) for i in range(5)]
    P.append(part('flag', 'box', (0.02, 0.45, 0.30), (0, 0.23, 2.3), color=CONE_ORANGE, double_sided=True))
    return P, {'size': [0.1, 0.5, 2.5]}


# ---------------------------------------------------------------------------------------------------------------
# bushes and debris
# ---------------------------------------------------------------------------------------------------------------

def bush_parts(rng, size_m=None):
    """A bush of 3-6 overlapping dark-green ellipsoids, 1.2-3 m across (fallback for the small-oak bushes)."""
    size = size_m or rng.uniform(1.2, 3.0)
    P = []
    for i in range(int(rng.integers(3, 7))):
        r = size * rng.uniform(0.25, 0.42)
        ox, oy = rng.uniform(-size * 0.25, size * 0.25, 2)
        P.append(part(f'leaf_{i}', 'ellipsoid', (r, r * rng.uniform(0.8, 1.1), r * rng.uniform(0.55, 0.8)),
                      (ox, oy, r * 0.6), color=BUSH_GREENS[int(rng.integers(len(BUSH_GREENS)))], roughness=0.95))
    return P, {'size': [round(size, 2), round(size, 2), round(size * 0.6, 2)]}


def log_parts(rng):
    """A fallen branch or log 2-6 m long, maybe with a stub branch."""
    L = rng.uniform(2.0, 6.0)
    r = rng.uniform(0.07, 0.20)
    P = [part('log', 'cylinder', (r, L), (0, 0, r), (0, math.pi / 2, 0), color=BARK)]
    if rng.random() < 0.7:
        a = rng.uniform(0.4, 1.1) * (1 if rng.random() < 0.5 else -1)
        x = rng.uniform(-L * 0.3, L * 0.3)
        lb = rng.uniform(0.6, 1.6)
        P.append(part('branch', 'cylinder', (r * 0.45, lb), (x + math.cos(a) * lb / 2, math.sin(a) * lb / 2, r * 0.6),
                      (0, math.pi / 2, a), color=BARK))
    return P, {'size': [round(L, 2), round(2 * r + 0.1, 2), round(2 * r, 2)]}


def sheet_parts(rng, kind):
    """A plywood sheet ('plywood') or a corrugated roof sheet ('roof'), 2.4 x 1.2 m, flat or slightly tilted."""
    col = PLYWOOD if kind == 'plywood' else ROOF_SHEET
    tilt = rng.uniform(0.0, 0.25) if rng.random() < 0.4 else 0.0
    z = 0.01 + 0.6 * math.sin(tilt) if tilt else 0.01
    P = [part('sheet', 'box', (2.4, 1.2, 0.02), (0, 0, z), (tilt, 0, 0), color=col,
              roughness=0.9 if kind == 'plywood' else 0.45)]
    if kind == 'roof':
        for i in range(6):
            P.append(part(f'rib_{i}', 'box', (2.4, 0.04, 0.015), (0, -0.5 + i * 0.2, z + 0.016), (tilt, 0, 0),
                          color=(0.50, 0.51, 0.53)))
    return P, {'size': [2.4, 1.2, round(z + 0.05, 2)]}


def lumber_parts(rng):
    """A small pile of 3-6 boards (2x4s), roughly parallel."""
    n = int(rng.integers(3, 7))
    L = rng.uniform(2.4, 3.6)
    P = [part(f'board_{i}', 'box', (L, 0.09, 0.04), (rng.uniform(-0.3, 0.3), -0.3 + i * 0.12, 0.02 + 0.045 * (i % 2)),
              (0, 0, rng.uniform(-0.25, 0.25)), color=WOOD) for i in range(n)]
    return P, {'size': [round(L + 0.6, 2), 0.8, 0.12]}


def tarp_parts(rng, colour_rgb):
    """A 4 x 3 m tarp lying on the ground with a few creases."""
    P = [part('tarp', 'box', (4.0, 3.0, 0.02), (0, 0, 0.01), color=colour_rgb, roughness=0.6)]
    for i in range(3):
        P.append(part(f'crease_{i}', 'box', (rng.uniform(1.0, 3.5), 0.12, 0.05),
                      (rng.uniform(-1.2, 1.2), rng.uniform(-1.0, 1.0), 0.03), (0, 0, rng.uniform(-0.6, 0.6)),
                      color=tuple(v * 0.85 for v in colour_rgb)))
    return P, {'size': [4.0, 3.0, 0.06]}


def tire_parts(rng):
    """A car tyre lying flat (or standing)."""
    standing = rng.random() < 0.25
    if standing:
        P = [part('tyre', 'cylinder', (0.33, 0.21), (0, 0, 0.33), (math.pi / 2, 0, 0), color=BLACK_RUBBER),
             part('hub', 'cylinder', (0.19, 0.23), (0, 0, 0.33), (math.pi / 2, 0, 0), color=(0.30, 0.30, 0.32))]
    else:
        P = [part('tyre', 'cylinder', (0.33, 0.21), (0, 0, 0.105), color=BLACK_RUBBER),
             part('hub', 'cylinder', (0.19, 0.23), (0, 0, 0.115), color=(0.30, 0.30, 0.32))]
    return P, {'size': [0.7, 0.7, 0.66 if standing else 0.22]}


def bin_parts(rng, colour_rgb):
    """A plastic wheelie bin / storage bin."""
    h = rng.uniform(0.6, 1.0)
    P = [part('bin', 'box', (0.55, 0.72, h), (0, 0, h / 2), color=colour_rgb, roughness=0.6),
         part('lid', 'box', (0.58, 0.75, 0.04), (0, 0, h + 0.02), color=tuple(v * 0.8 for v in colour_rgb))]
    return P, {'size': [0.58, 0.75, round(h + 0.04, 2)]}


def cooler_parts(rng, colour_rgb):
    """A picnic cooler box."""
    lid = (0.92, 0.92, 0.90) if colour_rgb != COOLER_COLOURS['white'] else (0.20, 0.30, 0.65)
    return [part('cooler', 'box', (0.65, 0.42, 0.42), (0, 0, 0.21), color=colour_rgb, roughness=0.5),
            part('lid', 'box', (0.67, 0.44, 0.05), (0, 0, 0.445), color=lid)], {'size': [0.67, 0.44, 0.47]}


def box_parts(rng):
    """A cardboard carton (fallback for the Fuel cardboard box)."""
    s = rng.uniform(0.35, 0.65)
    h = rng.uniform(0.3, 0.5)
    w = s * rng.uniform(0.7, 1.0)
    return [part('carton', 'box', (s, w, h), (0, 0, h / 2), color=CARDBOARD, roughness=0.9)], \
        {'size': [round(s, 2), round(w, 2), round(h, 2)]}


def mattress_parts(rng):
    """An old mattress (2.0 x 1.5 m) lying on the ground."""
    return [part('mattress', 'box', (2.0, 1.5, 0.25), (0, 0, 0.125), color=MATTRESS, roughness=0.9),
            part('stripe', 'box', (2.0, 0.08, 0.26), (0, 0.45, 0.125), color=(0.25, 0.35, 0.60)),
            part('stripe2', 'box', (2.0, 0.08, 0.26), (0, -0.45, 0.125), color=(0.25, 0.35, 0.60))], \
        {'size': [2.0, 1.5, 0.26]}


def pallet_parts(rng):
    """A wooden shipping pallet (1.2 x 0.8 m): top boards over three runners."""
    P = [part(f'board_{i}', 'box', (1.2, 0.10, 0.022), (0, -0.35 + i * 0.1167, 0.133), color=WOOD)
         for i in range(7)]
    P += [part(f'runner_{i}', 'box', (1.2, 0.10, 0.10), (0, -0.35 + i * 0.35, 0.061), color=tuple(v * 0.85 for v in WOOD))
          for i in range(3)]
    P += [part(f'bottom_{i}', 'box', (0.10, 0.8, 0.022), (-0.55 + i * 0.55, 0, 0.011), color=WOOD) for i in range(3)]
    return P, {'size': [1.2, 0.8, 0.144]}
