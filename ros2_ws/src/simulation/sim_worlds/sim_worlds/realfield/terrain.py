"""
sim_worlds/realfield/terrain.py -- PURPOSE: the ground of the real-terrain world as Gazebo meshes, made from the
site pack heights (USGS lidar): one GLB per terrain tile (fine near the search areas, coarse far away, see
site.tile_layout) plus ONE simple collision mesh. numpy only.

Design (follows the measured gazebo_engine research):
  * Each tile is a 65 x 65 point grid simplified with RTIN to within a height error that depends on how close it
    is to the action (site.TILE_KINDS: 5 cm in the search areas, 10 cm near them, 25 cm far away). Every point on
    a tile's border is kept, so tiles meet edge to edge; where a fine tile meets a coarse one, a short vertical
    "skirt" hangs below the border and hides any hairline crack.
  * Vertices are in world metres (x east, y north, z up, z = 0 at the pad); normals come from the 1 m height
    raster (smooth light across tile borders); texture coordinates put the tile's own picture exactly on it.
  * Physics gets a separate, coarser mesh (site.COLLISION_TOLERANCE) for the tiles inside the flight boundary
    + 150 m: a Gazebo <heightmap> collision misbehaved on this laptop (boxes floated 2.8 m or fell through,
    elevation research), meshes did not.
"""

import numpy as np

from sim_worlds.realfield import rtin, site

SKIRT_M = {'detail': 0.6, 'near66': 0.8, 'near': 1.0, 'far': 1.5}
COLLISION_MARGIN_M = 150.0


def texel_uv(t, n_px):
    """UV mapping helper: (u, v) of world points for a tile picture of n_px pixels whose FIRST and LAST pixel
    centres sit exactly on the tile edges (so two neighbouring tiles show the same colour on their shared edge).
    Returns a function f(x, y) -> (u, v)."""
    size_x, size_y = t['x1'] - t['x0'], t['y1'] - t['y0']

    def f(x, y):
        u = (0.5 + (x - t['x0']) / size_x * (n_px - 1)) / n_px
        v = (0.5 + (t['y1'] - y) / size_y * (n_px - 1)) / n_px
        return u, v
    return f


def tile_grid(pack, t, n=site.GRID_N):
    """(xs west->east, ys north->south, Z [n, n] world z) of a tile's full grid."""
    xs = np.linspace(t['x0'], t['x1'], n)
    ys = np.linspace(t['y1'], t['y0'], n)
    X, Y = np.meshgrid(xs, ys)
    return xs, ys, pack.ground_z(X, Y).astype(np.float64)


def normals(pack, x, y, d=1.0):
    """Unit ground normals at points (x, y) from central differences of the height raster (step d metres)."""
    zx = (pack.ground_z(x + d, y) - pack.ground_z(x - d, y)) / (2 * d)
    zy = (pack.ground_z(x, y + d) - pack.ground_z(x, y - d)) / (2 * d)
    nrm = np.stack([-zx, -zy, np.ones_like(zx)], axis=-1)
    return (nrm / np.linalg.norm(nrm, axis=-1, keepdims=True)).astype(np.float32)


def _ccw(pos, tri):
    """Flip triangles that are clockwise seen from above (so every face looks up)."""
    a, b, c = pos[tri[:, 0]], pos[tri[:, 1]], pos[tri[:, 2]]
    area = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
    tri = tri.copy()
    neg = area < 0
    tri[neg] = tri[neg][:, [0, 2, 1]]
    return tri


def simplify_tile(pack, t, tolerance, n=site.GRID_N):
    """RTIN mesh of one tile: (pos [V, 3] float64, tri [T, 3]) with every border point kept."""
    xs, ys, Z = tile_grid(pack, t, n)
    R = rtin.RTIN.get(n)
    err = R.errors(Z, forced=rtin.border_mask(n))
    verts, tri = R.mesh(err, tolerance)
    rows, cols = verts // n, verts % n
    pos = np.stack([xs[cols], ys[rows], Z[rows, cols]], axis=-1)
    return pos, _ccw(pos, tri)


def _boundary_edges(tri):
    """Edges used by exactly one triangle, in the triangle's own direction (interior on the left)."""
    e = np.concatenate([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]])
    key = np.sort(e, axis=1)
    _, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
    return e[cnt[inv.ravel()] == 1]


def build_tile(pack, t, tex_px):
    """The visual mesh of one tile as a glb primitive dict (pos, nrm, uv, idx) + its triangle count."""
    kind = site.TILE_KINDS[t['kind']]
    pos, tri = simplify_tile(pack, t, kind['tolerance'])
    nrm = normals(pack, pos[:, 0], pos[:, 1])
    uvf = texel_uv(t, tex_px)
    u, v = uvf(pos[:, 0], pos[:, 1])
    uv = np.stack([u, v], axis=-1)
    # skirts: copy every border vertex SKIRT_M lower; two triangles per border edge, both orientations
    edges = _boundary_edges(tri)
    border = np.unique(edges.ravel())
    low = {int(b): len(pos) + i for i, b in enumerate(border)}
    depth = SKIRT_M[t['kind']]
    pos2 = np.concatenate([pos, pos[border] - np.array([0.0, 0.0, depth])])
    nrm2 = np.concatenate([nrm, nrm[border]])
    uv2 = np.concatenate([uv, uv[border]])
    sk = []
    for a, b in edges:
        a2, b2 = low[int(a)], low[int(b)]
        sk += [(a, a2, b2), (a, b2, b), (a, b2, a2), (a, b, b2)]
    idx = np.concatenate([tri, np.array(sk, dtype=np.int64)]) if sk else tri
    surface_tris = len(tri)
    return {'pos': pos2.astype(np.float32), 'nrm': nrm2, 'uv': uv2.astype(np.float32),
            'idx': idx.ravel(), 'mat': None}, surface_tris, len(idx)


def collision_tiles(tiles, home):
    """The tiles that get physics: those within COLLISION_MARGIN_M of the flight boundary's bounding box."""
    fb = site.flight_boundary_xy(home)
    x0 = min(p[0] for p in fb) - COLLISION_MARGIN_M
    x1 = max(p[0] for p in fb) + COLLISION_MARGIN_M
    y0 = min(p[1] for p in fb) - COLLISION_MARGIN_M
    y1 = max(p[1] for p in fb) + COLLISION_MARGIN_M
    return [t for t in tiles if t['x1'] > x0 and t['x0'] < x1 and t['y1'] > y0 and t['y0'] < y1]


def build_collision(pack, tiles, home, log=print):
    """One merged collision primitive (pos, idx) over collision_tiles(), coarser tolerance per tile kind."""
    P, T, base = [], [], 0
    sel = collision_tiles(tiles, home)
    for t in sel:
        pos, tri = simplify_tile(pack, t, site.COLLISION_TOLERANCE[t['kind']])
        P.append(pos)
        T.append(tri + base)
        base += len(pos)
    pos = np.concatenate(P).astype(np.float32)
    tri = np.concatenate(T)
    # merge the duplicated border vertices of neighbouring tiles (same position) so the mesh is one surface
    key = np.round(pos[:, :2] * 1000).astype(np.int64)
    _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    pos_u = pos[first]
    tri_u = inv.ravel()[tri]
    good = (tri_u[:, 0] != tri_u[:, 1]) & (tri_u[:, 1] != tri_u[:, 2]) & (tri_u[:, 0] != tri_u[:, 2])
    tri_u = tri_u[good]
    nrm = normals(pack, pos_u[:, 0], pos_u[:, 1])
    log(f'  collision: {len(sel)} tiles, {len(pos_u):,} vertices, {len(tri_u):,} triangles')
    return {'pos': pos_u, 'nrm': nrm, 'uv': None, 'idx': tri_u.ravel(), 'mat': None}, len(sel), len(tri_u)


def max_slope_deg(pack, x, y, d=1.0):
    """Ground slope (degrees) at points, from the height raster."""
    n = normals(pack, x, y, d)
    return np.degrees(np.arccos(np.clip(n[..., 2], -1.0, 1.0)))
