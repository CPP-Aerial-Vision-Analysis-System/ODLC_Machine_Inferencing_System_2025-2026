"""
sim_worlds/realfield/structures.py -- PURPOSE: the man-made things of the real SUAS field, made from the site pack:
the BUILDINGS (OpenStreetMap + Microsoft footprints, raised to the height the 2017 lidar measured on their roofs)
as merged low-poly meshes, and the hand-added structures of site.MANUAL_STRUCTURES (the three wooden poles inside
Search Boundary 1 and the optional, unconfirmed wire between them) as simple SDF shapes. numpy only.

Why this way:
  * buildings: about 400 footprints, most far outside the flight boundary. All of them go into ONE mesh per
    "shadow group" (near the action: casts shadows; far away: does not), with one tiny colour-palette picture, so
    Gazebo draws them in a couple of batches instead of 400 models. Roof colours come from the real photo.
  * heights (research fix 5): the median lidar height inside the footprint ('height_lidar', set by stage 1), else
    the OpenStreetMap height / levels tag, else Microsoft's AI height, else 4 m. Lidar roofs in the flight
    boundary are 3.4-5.6 m; Microsoft said 3.0-3.6 m and had no height for two of the five.
  * poles (research fix 3): a 3 x 3 noise filter in stage 1 deletes thin poles from the lidar canopy model, and no
    map database has them, so they are added back by hand with their source notes (site.py). They cast shadows:
    long thin pole shadows inside a search area matter to the detector.
"""

import math

import numpy as np

from sim_worlds import geo
from sim_worlds.realfield import site
from sim_worlds.worldgen import sdf

DEFAULT_BUILDING_H = 4.0
WALL_RGB = (184, 178, 168)            #: light weathered siding / sheet metal
POLE_RGB = (0.36, 0.30, 0.24)         #: weathered wooden utility pole
WIRE_RGB = (0.10, 0.10, 0.10)
PALETTE_CELLS, PALETTE_CELL_PX = 4, 16   #: 4 x 4 colour cells of 16 px = a 64 x 64 palette picture
NEAR_SHADOW_M = 150.0                 #: buildings within this distance of a search area or the pad cast shadows


def building_height(b):
    """(height in m, where it came from) for one features.json building."""
    if b.get('height_lidar'):
        return float(b['height_lidar']), 'lidar 2017 median'
    for key in ('height',):
        v = b.get(key)
        try:
            if v is not None and float(str(v).split()[0]) > 0:
                return float(str(v).split()[0]), 'Microsoft footprint height' if b.get('source') == 'microsoft' \
                    else 'OpenStreetMap height tag'
        except ValueError:
            pass
    if b.get('levels'):
        try:
            return 3.0 * float(b['levels']) + 1.0, 'OpenStreetMap levels x 3 m + 1 m'
        except ValueError:
            pass
    return DEFAULT_BUILDING_H, 'default 4 m (no height data)'


def _area2(poly):
    """Twice the signed area of a polygon [(x, y), ...] (positive = counter-clockwise)."""
    a = 0.0
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        a += x0 * y1 - x1 * y0
    return a


def triangulate(poly):
    """Ear-clipping triangulation of a simple polygon (counter-clockwise or not). Returns [(i, j, k), ...]
    counter-clockwise seen from above. Falls back to a fan for degenerate rings."""
    n = len(poly)
    if n < 3:
        return []
    idx = list(range(n)) if _area2(poly) > 0 else list(range(n))[::-1]
    tris = []

    def is_ear(i0, i1, i2, rest):
        (ax, ay), (bx, by), (cx, cy) = poly[i0], poly[i1], poly[i2]
        if (bx - ax) * (cy - ay) - (by - ay) * (cx - ax) <= 1e-9:
            return False                                       # reflex or flat corner
        for j in rest:
            px, py = poly[j]
            d1 = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
            d2 = (cx - bx) * (py - by) - (cy - by) * (px - bx)
            d3 = (ax - cx) * (py - cy) - (ay - cy) * (px - cx)
            if d1 >= -1e-9 and d2 >= -1e-9 and d3 >= -1e-9:
                return False
        return True

    guard = 0
    while len(idx) > 3 and guard < 10 * n:
        guard += 1
        m = len(idx)
        for k in range(m):
            i0, i1, i2 = idx[(k - 1) % m], idx[k], idx[(k + 1) % m]
            rest = [j for j in idx if j not in (i0, i1, i2)]
            if is_ear(i0, i1, i2, rest):
                tris.append((i0, i1, i2))
                idx.pop(k)
                break
        else:
            break
    if len(idx) == 3:
        tris.append(tuple(idx))
    elif len(idx) > 3:                                         # self-touching ring: fan (good enough for a roof)
        tris += [(idx[0], idx[k], idx[k + 1]) for k in range(1, len(idx) - 1)]
    return tris


def roof_colour(pack, poly):
    """Mean photo colour (0..255) inside a footprint (the season grade never touches roofs)."""
    xs = np.array([p[0] for p in poly])
    ys = np.array([p[1] for p in poly])
    name = 'colour_near' if (xs.min() >= pack.meta['near_box'][0] and xs.max() <= pack.meta['near_box'][2] and
                             ys.min() >= pack.meta['near_box'][1] and ys.max() <= pack.meta['near_box'][3]) \
        else 'colour_far'
    r = pack.raster(name)
    gx = np.arange(xs.min(), xs.max() + 1e-6, r.res)
    gy = np.arange(ys.min(), ys.max() + 1e-6, r.res)
    if len(gx) < 2 or len(gy) < 2:
        return np.array(r.sample(xs.mean(), ys.mean()), dtype=np.float32)
    X, Y = np.meshgrid(gx, gy)
    inside = np.array([geo.point_in_polygon(x, y, poly) for x, y in zip(X.ravel(), Y.ravel())])
    if not inside.any():
        return np.array(r.sample(xs.mean(), ys.mean()), dtype=np.float32)
    vals = r.sample(X.ravel()[inside], Y.ravel()[inside])
    return np.asarray(vals, dtype=np.float32).reshape(-1, 3).mean(axis=0)


def _palette_uv(cell):
    """(u, v) of the centre of a palette cell (cell 0 = walls, 1..15 = roof colour groups)."""
    cx, cy = cell % PALETTE_CELLS, cell // PALETTE_CELLS
    return ((cx + 0.5) / PALETTE_CELLS, (cy + 0.5) / PALETTE_CELLS)


def palette_image(colours):
    """The 64 x 64 palette picture: cell 0 = wall colour, then the roof group colours (uint8 RGB)."""
    px = PALETTE_CELLS * PALETTE_CELL_PX
    img = np.zeros((px, px, 3), dtype=np.uint8)
    for cell, col in enumerate([WALL_RGB] + [tuple(c) for c in colours]):
        cx, cy = cell % PALETTE_CELLS, cell // PALETTE_CELLS
        img[cy * PALETTE_CELL_PX:(cy + 1) * PALETTE_CELL_PX, cx * PALETTE_CELL_PX:(cx + 1) * PALETTE_CELL_PX] = \
            np.clip(np.round(col), 0, 255).astype(np.uint8)
    return img


def near_action(x, y, home):
    """True when (x, y) is within NEAR_SHADOW_M of a search boundary or of the pad (shadow casting group)."""
    return site.near_zone_contains(x, y, home, NEAR_SHADOW_M)


def build_buildings(pack, log=print):
    """Merged building meshes. Returns (groups, palette RGB, info list) with groups = {'near': prim or None,
    'far': prim or None}; info = one dict per building (id, source, height, how, centre, group)."""
    feats = pack.features()
    blds = []
    x0, y0, x1, y1 = site.EXTENT
    for b in feats.get('buildings', []):
        poly = [tuple(p) for p in b['polygon']]
        if len(poly) < 3 or abs(_area2(poly)) < 4.0:
            continue
        cx = sum(p[0] for p in poly) / len(poly)
        cy = sum(p[1] for p in poly) / len(poly)
        if not (x0 + 2 < cx < x1 - 2 and y0 + 2 < cy < y1 - 2):
            continue
        h, how = building_height(b)
        blds.append({'b': b, 'poly': poly, 'cx': cx, 'cy': cy, 'h': h, 'how': how,
                     'rgb': roof_colour(pack, poly)})
    if not blds:
        return {'near': None, 'far': None}, palette_image([]), []
    # roof colour groups: 15 luminance quantiles, each drawn with its median colour
    lum = np.array([d['rgb'].mean() for d in blds])
    qs = np.percentile(lum, np.linspace(0, 100, PALETTE_CELLS * PALETTE_CELLS))[1:-1]
    group = np.searchsorted(qs, lum)
    colours = []
    for g in range(PALETTE_CELLS * PALETTE_CELLS - 1):
        sel = group == g
        colours.append(np.median(np.array([d['rgb'] for d, s in zip(blds, sel) if s]), axis=0) if sel.any()
                       else np.array([128.0, 128.0, 128.0]))
    prims = {'near': {'P': [], 'N': [], 'U': [], 'I': []}, 'far': {'P': [], 'N': [], 'U': [], 'I': []}}
    info = []
    for d, g in zip(blds, group):
        poly = d['poly'] if _area2(d['poly']) > 0 else d['poly'][::-1]
        xs = np.array([p[0] for p in poly])
        ys = np.array([p[1] for p in poly])
        gz = pack.ground_z(np.append(xs, d['cx']), np.append(ys, d['cy']))
        base = float(gz.min()) - 0.3
        roof = float(np.median(gz)) + d['h']
        key = 'near' if near_action(d['cx'], d['cy'], pack.home) else 'far'
        q = prims[key]
        start = sum(len(p) for p in q['P'])
        uw, vw = _palette_uv(0)
        P, N, U, I = [], [], [], []
        n = len(poly)
        for i in range(n):                                     # walls: one flat quad per edge, outward normal
            (ax, ay), (bx, by) = poly[i], poly[(i + 1) % n]
            ex, ey = bx - ax, by - ay
            length = math.hypot(ex, ey)
            if length < 1e-6:
                continue
            nx, ny = ey / length, -ex / length                 # right of the edge = outside for a CCW ring
            k = start + len(P)
            P += [(ax, ay, base), (bx, by, base), (bx, by, roof), (ax, ay, roof)]
            N += [(nx, ny, 0.0)] * 4
            U += [(uw, vw)] * 4
            I += [k, k + 1, k + 2, k, k + 2, k + 3]
        ur, vr = _palette_uv(int(g) + 1)
        k = start + len(P)
        P += [(x, y, roof) for x, y in poly]
        N += [(0.0, 0.0, 1.0)] * n
        U += [(ur, vr)] * n
        for a, b_, c in triangulate(poly):
            I += [k + a, k + b_, k + c]
        q['P'].append(np.array(P, dtype=np.float32))
        q['N'].append(np.array(N, dtype=np.float32))
        q['U'].append(np.array(U, dtype=np.float32))
        q['I'].append(np.array(I, dtype=np.int64))
        info.append({'id': d['b'].get('id'), 'source': d['b'].get('source'), 'height_m': round(d['h'], 2),
                     'height_how': d['how'], 'centre_xy': [round(d['cx'], 1), round(d['cy'], 1)], 'group': key,
                     'in_flight_boundary': geo.point_in_polygon(d['cx'], d['cy'], site.flight_boundary_xy(pack.home))})
    groups = {}
    for key, q in prims.items():
        groups[key] = None if not q['P'] else {'pos': np.concatenate(q['P']), 'nrm': np.concatenate(q['N']),
                                               'uv': np.concatenate(q['U']), 'idx': np.concatenate(q['I']),
                                               'mat': None}
    log(f'  buildings: {len(info)} ({sum(1 for i in info if i["group"] == "near")} near the action), '
        f'{sum(1 for i in info if i["height_how"].startswith("lidar"))} with a lidar height')
    return groups, palette_image(colours), info


# ------------------------------------------------------------------------------------------------------------
# hand-added structures (world SDF parts, stage 4)
# ------------------------------------------------------------------------------------------------------------

def pole_parts(height_m, diameter_m):
    """SDF parts of one wooden pole standing on z = 0 (sunk 0.5 m into the ground like a real pole), with a
    simple cylinder collision."""
    return [sdf.part('pole', 'cylinder', (diameter_m / 2.0, height_m + 0.5), (0.0, 0.0, (height_m - 0.5) / 2.0),
                     color=POLE_RGB, collision=True, roughness=0.95)]


def wire_parts(p0, p1, sag_m, diameter_m, segments=10, name='wire'):
    """A sagging cable from world point p0 to p1 (x, y, z) as `segments` thin cylinders (parabola sag)."""
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    pts = []
    for k in range(segments + 1):
        t = k / segments
        p = p0 + (p1 - p0) * t
        p[2] -= 4.0 * sag_m * t * (1.0 - t)
        pts.append(p)
    parts = []
    for k in range(segments):
        a, b = pts[k], pts[k + 1]
        d = b - a
        length = float(np.linalg.norm(d))
        yaw = math.atan2(d[1], d[0])
        pitch = math.acos(max(-1.0, min(1.0, d[2] / max(length, 1e-9))))
        mid = (a + b) / 2.0
        parts.append(sdf.part(f'{name}_{k}', 'cylinder', (diameter_m / 2.0, length), tuple(mid), (0.0, pitch, yaw),
                              color=WIRE_RGB, roughness=0.6))
    return parts
