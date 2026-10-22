"""
sim_worlds/realfield/trees.py -- PURPOSE: the REAL trees of the field as Gazebo meshes, from the site pack's
trees.csv (lidar 2017 + 2023 photo, see trees_detect.py), and the closed forest as canopy sheets. numpy + Pillow.

Why it is built like this (measured by the gazebo_engine research on this laptop): 2,000 trees as separate
Gazebo models took 30 s to load and 5.5 GB of memory; the same trees MERGED into a few meshes loaded in 3.5 s.
So:
  * every tree is a copy of a small low-poly shape (deciduous: lumpy ellipsoid crown; eastern red-cedar: narrow
    lumpy cone), stretched to its REAL height and crown diameter, turned randomly, set on the real ground;
  * all trees of one 264 m terrain block are merged into ONE crown mesh (+ one trunk mesh near the action), with
    ONE shared material, so Gazebo has about 2 x 49 tree meshes in total instead of 12,000 models;
  * near the search areas and the pad the shapes have ~100 triangles; far away ~36 and no trunk;
  * crowns get their colour from a small leaf "atlas" picture: 4 tones for deciduous trees and 4 for cedars,
    taken from the real crown colours in the 2023 photo (each tree uses the tone closest to its own crown);
  * no collision: trees reach about 28 m and the drone flies 46-122 m above the ground; payloads fall in the
    open search areas;
  * closed forest (56 ha, mostly the creek valley) is ONE canopy sheet per block: a 3 m grid at the real canopy
    height (lidar), painted with the photo, with walls down to the ground at its edges.
"""

import math

import numpy as np

from sim_worlds.realfield import site, texture

ATLAS_COLS, ATLAS_ROWS, CELL_PX = 4, 2, 256
CANOPY_GRID_M = 3.0
CANOPY_TEX_PX = 512


def _ring_surface(profile_z, profile_r, segs, rng, lump):
    """A closed-at-the-top surface of revolution: rings at heights profile_z with radii profile_r (unit crown),
    lumpy radius noise. Returns pos (N, 3), nrm (N, 3), uv (N, 2) (u around, v = 0 at the top), idx."""
    rings = len(profile_z)
    ph = np.linspace(0, 2 * math.pi, segs + 1)
    noise = 1.0 + lump * (rng.random((rings, segs + 1)) - 0.5)
    noise[:, -1] = noise[:, 0]
    R = np.asarray(profile_r)[:, None] * noise
    Z = np.repeat(np.asarray(profile_z)[:, None], segs + 1, axis=1)
    pos = np.stack([R * np.cos(ph), R * np.sin(ph), Z], -1).reshape(-1, 3)
    # normals: from the surface tangents (finite differences of the grid)
    P = pos.reshape(rings, segs + 1, 3)
    du = np.roll(P, -1, axis=1) - np.roll(P, 1, axis=1)
    du[:, 0] = P[:, 1] - P[:, -2]
    du[:, -1] = du[:, 0]
    dv = np.zeros_like(P)
    dv[1:-1] = P[2:] - P[:-2]
    dv[0] = P[1] - P[0]
    dv[-1] = P[-1] - P[-2]
    n = np.cross(du, dv)
    flip = (n * (P - np.array([0, 0, P[..., 2].mean()]))).sum(-1) < 0
    n[flip] *= -1
    n /= np.linalg.norm(n, axis=-1, keepdims=True) + 1e-9
    tips = np.asarray(profile_r) <= 1e-9                    # a closed tip has no tangent: point straight up/down
    for i in np.nonzero(tips)[0]:
        n[i] = (0.0, 0.0, 1.0 if profile_z[i] >= np.mean(profile_z) else -1.0)
    zn = (Z - Z.min()) / max(Z.max() - Z.min(), 1e-6)
    uv = np.stack([np.repeat(ph[None, :] / (2 * math.pi), rings, axis=0), 1.0 - zn], -1).reshape(-1, 2)
    idx = []
    w = segs + 1
    for i in range(rings - 1):
        for j in range(segs):
            a, b = i * w + j, i * w + j + 1
            c, d = a + w, b + w
            idx += [a, b, d, a, d, c]
    tri = np.array(idx, dtype=np.int64).reshape(-1, 3)
    # make faces point outwards (counter-clockwise seen from outside)
    v0, v1, v2 = pos[tri[:, 0]], pos[tri[:, 1]], pos[tri[:, 2]]
    fn = np.cross(v1 - v0, v2 - v0)
    out = (fn * ((v0 + v1 + v2) / 3 - np.array([0, 0, pos[:, 2].mean()]))).sum(-1) < 0
    tri[out] = tri[out][:, [0, 2, 1]]
    tri = tri[np.linalg.norm(fn, axis=1) > 1e-10]             # drop the zero-area triangles at a closed tip
    return pos.astype(np.float32), n.reshape(-1, 3).astype(np.float32), uv.astype(np.float32), tri.ravel()


def crown_shape(kind, detail, seed):
    """Unit crown (x, y within +-0.5 = crown diameter 1, z 0..1 = tree height 1). detail: 'near' or 'far'."""
    rng = np.random.default_rng(seed)
    if kind == 'cedar':
        rings, segs = (7, 8) if detail == 'near' else (4, 6)
        zn = np.linspace(0.0, 1.0, rings)
        z = 0.06 + 0.94 * zn
        r = 0.5 * np.power(np.clip(1.0 - zn, 0.0, 1.0), 0.85) * (0.88 + 0.12 * np.sin(np.pi * np.clip(zn * 1.6, 0, 1)))
        r[-1] = 0.0
        return _ring_surface(z, r, segs, rng, 0.22)
    rings, segs = (7, 10) if detail == 'near' else (5, 6)
    th = np.linspace(0.0, math.pi, rings)
    z = 0.62 + 0.38 * np.cos(th)                                   # crown from 0.24 to 1.0 of the height
    r = 0.5 * np.sin(th)
    r[0] = r[-1] = 0.0
    return _ring_surface(z, r, segs, rng, 0.30)


def trunk_shape(kind, segs=6):
    """Unit trunk: a 6-sided prism from 0.3 m below ground (z = 0) to inside the crown."""
    top = 0.30 if kind == 'deciduous' else 0.12
    ph = np.linspace(0, 2 * math.pi, segs + 1)
    rr = 0.035
    ring = np.stack([rr * np.cos(ph), rr * np.sin(ph)], 1)
    pos = np.concatenate([np.c_[ring, np.zeros(segs + 1)], np.c_[ring * 0.7, np.full(segs + 1, top)]])
    nrm = np.concatenate([np.c_[np.cos(ph), np.sin(ph), np.zeros(segs + 1)]] * 2)
    idx = []
    for j in range(segs):
        a, b, c, d = j, j + 1, j + segs + 1, j + segs + 2
        idx += [a, b, d, a, d, c]
    return pos.astype(np.float32), nrm.astype(np.float32), np.array(idx, dtype=np.int64)


def _tone(rgb_hex):
    r, g, b = (int(rgb_hex[i:i + 2], 16) for i in (0, 2, 4))
    return np.array([r, g, b], dtype=np.float32)


def leaf_tones(trees):
    """Per kind, 4 tone colours (luminance quartiles of the real crown colours) and a function tree -> cell."""
    tones, bounds = {}, {}
    for row, kind in enumerate(('deciduous', 'cedar')):
        cols = np.array([_tone(t.get('crown_rgb') or '3e573f') for t in trees if t['kind'] == kind] or
                        [[62, 87, 63]], dtype=np.float32)
        lum = cols.mean(axis=1)
        qs = np.percentile(lum, [25, 50, 75])
        bounds[kind] = qs
        cells = []
        for q in range(4):
            lo = -1 if q == 0 else qs[q - 1]
            hi = 1e9 if q == 3 else qs[q]
            sel = (lum > lo) & (lum <= hi)
            cells.append(np.median(cols[sel], axis=0) if sel.any() else cols.mean(axis=0))
        tones[kind] = cells

    def cell(t):
        kind = 'cedar' if t['kind'] == 'cedar' else 'deciduous'
        lum = float(_tone(t.get('crown_rgb') or '3e573f').mean())
        q = int(np.searchsorted(bounds[kind], lum))
        return (0 if kind == 'deciduous' else 1), min(q, 3)
    return tones, cell


def leaf_atlas(tones, seed=2026):
    """The crown picture: 4 x 2 cells of procedural leaf-clump texture (made here, no licence issue)."""
    rng = np.random.default_rng(seed)
    img = np.zeros((ATLAS_ROWS * CELL_PX, ATLAS_COLS * CELL_PX, 3), dtype=np.float32)
    from PIL import Image
    for row, kind in enumerate(('deciduous', 'cedar')):
        for col in range(ATLAS_COLS):
            base = np.clip(tones[kind][col] * (1.18 if kind == 'deciduous' else 1.10), 0, 255)
            tex = np.zeros((CELL_PX, CELL_PX), dtype=np.float32)
            for octave, amp in ((8, 0.55), (24, 0.30), (64, 0.15)):
                small = rng.random((octave, octave)).astype(np.float32)
                tex += amp * np.asarray(Image.fromarray(small).resize((CELL_PX, CELL_PX), Image.BICUBIC))
            tex = (tex - tex.mean()) / (tex.std() + 1e-6)
            gaps = np.clip(-tex - 1.1, 0, None)                       # dark gaps between leaf clumps
            shade = np.clip(1.0 + 0.22 * tex - 0.55 * gaps, 0.35, 1.45)
            img[row * CELL_PX:(row + 1) * CELL_PX, col * CELL_PX:(col + 1) * CELL_PX] = base * shade[..., None]
    return np.clip(np.round(img), 0, 255).astype(np.uint8)


def _place(shape_pos, shape_nrm, d, h, yaw, xyz):
    """Stretch a unit shape to crown diameter d and height h, turn it by yaw, move it to xyz."""
    s = np.array([d, d, h], dtype=np.float32)
    c, si = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -si, 0], [si, c, 0], [0, 0, 1]], dtype=np.float32)
    p = (shape_pos * s) @ R.T + np.asarray(xyz, dtype=np.float32)
    n = (shape_nrm / s) @ R.T
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-9
    return p, n


def tree_base_z(pack, x, y, r):
    """Ground height under a trunk: the lowest of the centre and 4 points 0.5 m around it, sunk 0.25 m (no
    floating on slopes)."""
    pts_x = np.array([x, x + r, x - r, x, x])
    pts_y = np.array([y, y, y, y + r, y - r])
    return float(pack.ground_z(pts_x, pts_y).min()) - 0.25


def block_of(x, y):
    """(i, j) of the 264 m terrain block containing world point (x, y)."""
    return int((x - site.EXTENT[0]) // site.BLOCK_M), int((y - site.EXTENT[1]) // site.BLOCK_M)


def build_tree_blocks(pack, home, near_blocks, seed=2026):
    """Merge the 3D-model trees per block. Returns ({block: {'crowns': prim, 'trunks': prim or None, 'trees': n,
    'triangles': n}}, tones, atlas)."""
    rng = np.random.default_rng(seed)
    trees = [t for t in pack.trees() if t['forest'] == 0]
    x0, y0, x1, y1 = site.EXTENT
    trees = [t for t in trees if x0 + 1 < t['x'] < x1 - 1 and y0 + 1 < t['y'] < y1 - 1]
    tones, cell_of = leaf_tones(trees)
    atlas = leaf_atlas(tones)
    shapes = {(k, dl, v): crown_shape(k, dl, 100 * v + (7 if k == 'cedar' else 3) + (0 if dl == 'near' else 50))
              for k in ('deciduous', 'cedar') for dl in ('near', 'far') for v in range(3)}
    trunks = {k: trunk_shape(k) for k in ('deciduous', 'cedar')}
    groups = {}
    for t in trees:
        groups.setdefault(block_of(t['x'], t['y']), []).append(t)
    out = {}
    for blk, ts in sorted(groups.items()):
        detail = 'near' if blk in near_blocks else 'far'
        CP, CN, CU, CI, TP, TN, TI = [], [], [], [], [], [], []
        cbase = tbase = 0
        for t in ts:
            kind = 'cedar' if t['kind'] == 'cedar' else 'deciduous'
            pos, nrm, uv, idx = shapes[(kind, detail, int(rng.integers(3)))]
            d = float(t['crown_diam_m']) * float(rng.uniform(0.92, 1.08))
            h = max(float(t['height_m']), 2.0)
            yaw = float(rng.uniform(0, 2 * math.pi))
            z = tree_base_z(pack, t['x'], t['y'], 0.5)
            p, n = _place(pos, nrm, d, h, yaw, (t['x'], t['y'], z))
            row, col = cell_of(t)
            u = (col + 0.03 + 0.94 * uv[:, 0]) / ATLAS_COLS
            v = (row + 0.03 + 0.94 * uv[:, 1]) / ATLAS_ROWS
            CP.append(p)
            CN.append(n)
            CU.append(np.stack([u, v], -1))
            CI.append(idx + cbase)
            cbase += len(p)
            if detail == 'near':
                tp, tn, ti = trunks[kind]
                p2, n2 = _place(tp, tn, d, h, yaw, (t['x'], t['y'], z))
                TP.append(p2)
                TN.append(n2)
                TI.append(ti + tbase)
                tbase += len(p2)
        crowns = {'pos': np.concatenate(CP), 'nrm': np.concatenate(CN), 'uv': np.concatenate(CU).astype(np.float32),
                  'idx': np.concatenate(CI), 'mat': None}
        trunk = ({'pos': np.concatenate(TP), 'nrm': np.concatenate(TN), 'uv': None, 'idx': np.concatenate(TI),
                  'mat': None} if TP else None)
        tri = len(crowns['idx']) // 3 + (len(trunk['idx']) // 3 if trunk else 0)
        out[blk] = {'crowns': crowns, 'trunks': trunk, 'trees': len(ts), 'triangles': tri, 'detail': detail}
    return out, tones, atlas


def build_canopy_block(pack, blk):
    """Canopy sheet of the closed forest in one block: (prim, texture RGB) or (None, None) without forest."""
    bx0 = site.EXTENT[0] + blk[0] * site.BLOCK_M
    by0 = site.EXTENT[1] + blk[1] * site.BLOCK_M
    n = int(round(site.BLOCK_M / CANOPY_GRID_M)) + 1
    xs = np.linspace(bx0, bx0 + site.BLOCK_M, n)
    ys = np.linspace(by0 + site.BLOCK_M, by0, n)
    X, Y = np.meshgrid(xs, ys)
    C = pack.raster('canopy').sample(X, Y)
    if (C > 0.5).sum() < 4:
        return None, None
    Z = pack.ground_z(X, Y)
    top = Z + C
    act = (C[:-1, :-1] > 0.5) & (C[1:, :-1] > 0.5) & (C[:-1, 1:] > 0.5) & (C[1:, 1:] > 0.5)
    if not act.any():
        return None, None
    gy, gx = np.gradient(top, -CANOPY_GRID_M, CANOPY_GRID_M)        # rows go south: dy is negative
    nrm = np.stack([-gx, -gy, np.ones_like(top)], -1)
    nrm /= np.linalg.norm(nrm, axis=-1, keepdims=True)
    vid = -np.ones((n, n), dtype=np.int64)
    used = np.zeros((n, n), dtype=bool)
    used[:-1, :-1] |= act
    used[1:, :-1] |= act
    used[:-1, 1:] |= act
    used[1:, 1:] |= act
    vid[used] = np.arange(int(used.sum()))
    P = np.stack([X[used], Y[used], top[used]], -1)
    N = nrm[used]
    uvf = texture_uv(bx0, by0)
    U = np.stack(uvf(P[:, 0], P[:, 1]), -1)
    r, c = np.nonzero(act)
    a, b, cc, d = vid[r, c], vid[r, c + 1], vid[r + 1, c], vid[r + 1, c + 1]
    tris = [np.stack([a, cc, d], 1), np.stack([a, d, b], 1)]           # counter-clockwise seen from above
    # walls: along every edge between an active and an inactive (or outside) cell, down to the ground
    walls_p, walls_n, walls_u, walls_i = [], [], [], []
    base = len(P)
    actp = np.pad(act, 1)
    for dr, dc, (e0, e1) in ((-1, 0, ((0, 0), (0, 1))), (1, 0, ((1, 0), (1, 1))),
                             (0, -1, ((0, 0), (1, 0))), (0, 1, ((0, 1), (1, 1)))):
        edge = act & ~actp[1 + dr:1 + dr + act.shape[0], 1 + dc:1 + dc + act.shape[1]]
        rr, cc2 = np.nonzero(edge)
        for k in range(len(rr)):
            (ra, ca), (rb, cb) = (rr[k] + e0[0], cc2[k] + e0[1]), (rr[k] + e1[0], cc2[k] + e1[1])
            ta = np.array([X[ra, ca], Y[ra, ca], top[ra, ca]])
            tb = np.array([X[rb, cb], Y[rb, cb], top[rb, cb]])
            ga = np.array([X[ra, ca], Y[ra, ca], Z[ra, ca] - 0.3])
            gb = np.array([X[rb, cb], Y[rb, cb], Z[rb, cb] - 0.3])
            walls_p += [ta, tb, gb, ga]
            out_n = np.array([dc, -dr, 0.0])
            walls_n += [out_n] * 4
            ua, va = uvf(ta[0], ta[1])
            ub, vb = uvf(tb[0], tb[1])
            walls_u += [(ua, va), (ub, vb), (ub, vb), (ua, va)]
            q = base + 4 * len(walls_i)
            walls_i.append([q, q + 1, q + 2, q, q + 2, q + 3, q, q + 2, q + 1, q, q + 3, q + 2])  # both sides
    if walls_p:
        P = np.concatenate([P, np.array(walls_p)])
        N = np.concatenate([N, np.array(walls_n)])
        U = np.concatenate([U, np.array(walls_u)])
        tris.append(np.array(walls_i, dtype=np.int64).reshape(-1, 3))
    prim = {'pos': P.astype(np.float32), 'nrm': N.astype(np.float32), 'uv': U.astype(np.float32),
            'idx': np.concatenate(tris).ravel(), 'mat': None}
    tex = texture.sample_colour(pack, bx0, by0, bx0 + site.BLOCK_M, by0 + site.BLOCK_M, CANOPY_TEX_PX)
    return prim, tex


def texture_uv(bx0, by0):
    """UV function of a block picture (first/last pixel centres on the block edges)."""
    from sim_worlds.realfield.terrain import texel_uv
    return texel_uv({'x0': bx0, 'y0': by0, 'x1': bx0 + site.BLOCK_M, 'y1': by0 + site.BLOCK_M}, CANOPY_TEX_PX)
