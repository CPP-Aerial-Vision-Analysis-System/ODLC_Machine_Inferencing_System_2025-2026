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


def _smooth_noise(rng, rows, cols, gr, gc):
    """Smooth random field (rows x cols, values 0..1): a gr x gc grid of random values, linearly interpolated
    (wraps around the columns so the crown has no seam). numpy only."""
    g = rng.random((gr + 1, gc + 1))
    g[:, -1] = g[:, 0]
    ys = np.linspace(0, gr, rows)
    xs = np.linspace(0, gc, cols)
    y0 = np.minimum(ys.astype(int), gr - 1)
    x0 = np.minimum(xs.astype(int), gc - 1)
    fy = (ys - y0)[:, None]
    fx = (xs - x0)[None, :]
    a = g[y0][:, x0] * (1 - fx) + g[y0][:, x0 + 1] * fx
    b = g[y0 + 1][:, x0] * (1 - fx) + g[y0 + 1][:, x0 + 1] * fx
    return a * (1 - fy) + b * fy


def _ring_surface(profile_z, profile_r, segs, rng, lump):
    """A closed-at-the-top surface of revolution: rings at heights profile_z with radii profile_r (unit crown),
    lumpy radius noise. Returns pos (N, 3), nrm (N, 3), uv (N, 2) (u around, v = 0 at the top), idx."""
    rings = len(profile_z)
    ph = np.linspace(0, 2 * math.pi, segs + 1)
    coarse = _smooth_noise(rng, rings, segs + 1, 3, 4)               # big lobes (a branch, a clump)
    fine = rng.random((rings, segs + 1))                              # leaf-cluster bumps
    noise = 1.0 + lump * (0.65 * (coarse - 0.5) + 0.35 * (fine - 0.5)) * 2.0
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
        rings, segs = (9, 12) if detail == 'near' else (4, 6)
        zn = np.linspace(0.0, 1.0, rings)
        z = 0.06 + 0.94 * zn
        r = 0.5 * np.power(np.clip(1.0 - zn, 0.0, 1.0), 0.85) * (0.88 + 0.12 * np.sin(np.pi * np.clip(zn * 1.6, 0, 1)))
        r[-1] = 0.0
        return _ring_surface(z, r, segs, rng, 0.30)
    rings, segs = (11, 16) if detail == 'near' else (5, 6)
    th = np.linspace(0.0, math.pi, rings)
    z = 0.62 + 0.38 * np.cos(th)                                   # crown from 0.24 to 1.0 of the height
    r = 0.5 * np.sin(th)
    r[0] = r[-1] = 0.0
    return _ring_surface(z, r, segs, rng, 0.42)


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


# Leaf tones used when the pack has no per-crown colour (the committed pack has none): late-summer greens of
# the Oklahoma post-oak / elm / hackberry woodland (warm olive to mid green) and the darker blue-green of eastern
# red-cedar. The average sits near the measured mean tree colour of the 2023 photo (62, 87, 63) once the
# shaded side of each crown is counted in.
FALLBACK_TONES = {
    'deciduous': [(82, 108, 60), (70, 96, 54), (58, 80, 50), (94, 110, 62)],
    'cedar': [(42, 66, 52), (36, 58, 46), (48, 72, 56), (32, 50, 42)],
}


def leaf_tones(trees):
    """Per kind, 4 tone colours and a function tree -> (row, col) cell of the atlas. If the trees carry their real
    crown colours (crown_rgb, from the 2023 photo) the tones are the luminance quartiles of those and each tree
    uses the tone closest to its own crown; if not, the fixed palette above is used and each tree picks a tone by
    a stable hash of its id (so a re-build gives the same forest)."""
    have = any(t.get('crown_rgb') for t in trees)
    tones, bounds = {}, {}
    for kind in ('deciduous', 'cedar'):
        if not have:
            tones[kind] = [np.array(c, dtype=np.float32) for c in FALLBACK_TONES[kind]]
            continue
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
        row = 0 if kind == 'deciduous' else 1
        if not have:
            return row, (int(t.get('id', 0)) * 2654435761 >> 7) % 4
        lum = float(_tone(t.get('crown_rgb') or '3e573f').mean())
        return row, min(int(np.searchsorted(bounds[kind], lum)), 3)
    return tones, cell


def leaf_atlas(tones, seed=2026):
    """The crown picture: 4 x 2 cells of procedural foliage texture (made here, no licence issue): leaf-sized
    speckle, a few small dark gaps, lighter on top and darker underneath (the shading a real crown has). The
    pattern repeats seamlessly around the crown (u) so there is no seam line."""
    rng = np.random.default_rng(seed)
    img = np.zeros((ATLAS_ROWS * CELL_PX, ATLAS_COLS * CELL_PX, 3), dtype=np.float32)
    vv = np.linspace(0.0, 1.0, CELL_PX)[:, None]                          # 0 = top of the crown, 1 = bottom
    for row, kind in enumerate(('deciduous', 'cedar')):
        for col in range(ATLAS_COLS):
            base = np.clip(tones[kind][col], 0, 255)
            tex = np.zeros((CELL_PX, CELL_PX), dtype=np.float32)
            for grid, amp in ((6, 0.30), (20, 0.30), (64, 0.28), (160, 0.12)):
                tex += amp * _smooth_noise(rng, CELL_PX, CELL_PX, grid, grid)
            tex = (tex - tex.mean()) / (tex.std() + 1e-6)
            gaps = np.clip(-tex - 1.6, 0, None)                             # a few small shadow holes
            shade = (1.0 + 0.20 * tex - 0.45 * gaps) * (1.14 - 0.40 * vv)
            hue = 1.0 + 0.05 * (_smooth_noise(rng, CELL_PX, CELL_PX, 5, 5) - 0.5)   # a little yellow/green drift
            rgb = base[None, None, :] * np.clip(shade, 0.30, 1.40)[..., None]
            rgb[..., 0] *= hue
            rgb[..., 2] /= hue
            img[row * CELL_PX:(row + 1) * CELL_PX, col * CELL_PX:(col + 1) * CELL_PX] = rgb
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


def _top_down_uv(ts, crown_positions, crown_normals, tops):
    """Texture coordinates that lay each tree's own top-down photo over its crown mesh (planar projection from above),
    and the block's picture: all the block's crown pictures side by side. ts: the block's trees (dicts with 'index',
    'x', 'y', 'crown_diam_m'), crown_positions / crown_normals: world vertex positions / normals of each crown (same
    order). Steep parts of a crown (its sides) would show the photo's stretched edge pixels, so their lookup is pulled
    towards the middle of the crown picture (the crown's average look). Returns
    (uv [N, 2] float32, RGB uint8 picture)."""
    arr, cell, cols, span = tops
    n = len(ts)
    gc = int(math.ceil(math.sqrt(n)))
    gr = int(math.ceil(n / gc))
    pic = np.zeros((gr * cell, gc * cell, 3), dtype=np.uint8)
    UV = []
    for k, (t, p, nrm) in enumerate(zip(ts, crown_positions, crown_normals)):
        i = int(t['index'])
        sr, sc = (i // cols) * cell, (i % cols) * cell
        r, c = (k // gc) * cell, (k % gc) * cell
        pic[r:r + cell, c:c + cell] = arr[sr:sr + cell, sc:sc + cell]
        d = max(float(t['crown_diam_m']), 1.0)
        # offset from the trunk in photo units: 0.5 = centre of the cell; the cell shows span * d metres, north up
        a = 0.5 + (p[:, 0] - t['x']) / (span * d)
        b = 0.5 - (p[:, 1] - t['y']) / (span * d)
        inset = 0.75 / cell                              # keep the lookup inside the cell: no bleeding from neighbours
        pull = np.clip(nrm[:, 2] * 1.6, 0.25, 1.0)          # 1 on the top, down to 0.25 on a vertical side
        a = np.clip(0.5 + (a - 0.5) * pull, inset, 1 - inset)
        b = np.clip(0.5 + (b - 0.5) * pull, inset, 1 - inset)
        UV.append(np.stack([(c + a * cell) / (gc * cell), (r + b * cell) / (gr * cell)], -1))
    return np.concatenate(UV).astype(np.float32), _leaf_detail(pic)


def _leaf_detail(pic, factor=2, amount=0.11, seed=11):
    """The crown pictures are 25 cm per photo pixel, so a crown has only ~30 pixels across: smooth. Double the size and
    multiply in leaf-clump noise (two scales) whose average is 1, so the real colours of each crown stay what they are
    on average and only the fine structure (leaf clumps, small gaps) is added. numpy + Pillow."""
    from PIL import Image
    h, w = pic.shape[:2]
    big = np.asarray(Image.fromarray(pic).resize((w * factor, h * factor), Image.BICUBIC), dtype=np.float32)
    rng = np.random.default_rng(seed)
    n = np.zeros((h * factor, w * factor), dtype=np.float32)
    for grid, amp in ((0.5, 0.6), (0.2, 0.4)):                            # lumps about 2 and 0.8 photo pixels wide
        gh, gw = max(int(h * factor * grid), 4), max(int(w * factor * grid), 4)
        small = rng.random((gh, gw)).astype(np.float32)
        n += amp * np.asarray(Image.fromarray(small).resize((w * factor, h * factor), Image.BICUBIC))
    n = (n - n.mean()) / (n.std() + 1e-6)
    shade = np.clip(1.0 + amount * n - 0.12 * np.clip(-n - 1.3, 0, None), 0.55, 1.45)
    return np.clip(np.round(big * shade[..., None]), 0, 255).astype(np.uint8)


def build_tree_blocks(pack, home, near_blocks, seed=2026):
    """Merge the 3D-model trees per block. Returns ({block: {'crowns': prim, 'trunks': prim or None, 'trees': n,
    'triangles': n}}, tones, atlas)."""
    rng = np.random.default_rng(seed)
    trees = [t for t in pack.trees() if t['forest'] == 0]
    x0, y0, x1, y1 = site.EXTENT
    trees = [t for t in trees if x0 + 1 < t['x'] < x1 - 1 and y0 + 1 < t['y'] < y1 - 1]
    tones, cell_of = leaf_tones(trees)
    atlas = leaf_atlas(tones)
    tops = pack.tree_tops()                               # the real top-down crown pictures, when the pack has them
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
        if tops is not None:
            crowns['uv'], block_atlas = _top_down_uv(ts, CP, CN, tops)
        else:
            block_atlas = None
        trunk = ({'pos': np.concatenate(TP), 'nrm': np.concatenate(TN), 'uv': None, 'idx': np.concatenate(TI),
                  'mat': None} if TP else None)
        tri = len(crowns['idx']) // 3 + (len(trunk['idx']) // 3 if trunk else 0)
        out[blk] = {'crowns': crowns, 'trunks': trunk, 'trees': len(ts), 'triangles': tri, 'detail': detail,
                    'atlas': block_atlas}
    return out, tones, atlas


def _neighbour_max(a):
    """Maximum of the 8 neighbours of every cell (edges repeat). numpy only."""
    p = np.pad(a, 1, mode='edge')
    out = a.copy()
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr or dc:
                out = np.maximum(out, p[1 + dr:1 + dr + a.shape[0], 1 + dc:1 + dc + a.shape[1]])
    return out


def _taper_edges(C, ring_fractions=(0.6, 0.3)):
    """Give the closed forest a SLOPING edge: the cells just outside it get 60 % and then 30 % of the canopy
    height next to them (instead of a vertical wall from the full height to the ground, which rendered as dark
    cracks). A real forest edge is crown after crown stepping down; a two-step skirt is the same at a 3 m grid."""
    C = C.copy()
    inside = C > 0.5
    for frac in ring_fractions:
        nb = _neighbour_max(np.where(inside, C, 0.0))
        ring = (~inside) & (nb > 0.5)
        C = np.where(ring, np.maximum(nb * frac, 0.6), C)
        inside = inside | ring
    return C


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
    C = _taper_edges(C)
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
