"""
sim_worlds/realfield/rtin.py -- PURPOSE: simplify a square height grid into a triangle mesh that stays within a
chosen height error ("RTIN", right-triangulated irregular network). numpy only.

Why: a regular 1 m grid over the whole 1.8 km field would be millions of triangles, and the gazebo_engine
research measured that such a mesh runs this laptop out of memory. Flat grass needs few triangles, ditches and
creek banks need many: RTIN keeps detail only where the ground really bends, and guarantees that no point of the
mesh is farther than `max_error` metres (vertically) from the grid.

How (the algorithm of "Martini" by Vladimir Agafonkin / Mapbox, 2019, re-written here with numpy): the grid
(2^k + 1 points per side) is split recursively into right triangles. For every triangle, the error is how far
the grid height at the middle of its long side differs from the straight line between its two corners; each
triangle also inherits the largest error of the smaller triangles inside it. Building the mesh then just walks
down the triangle tree and stops where the error is small enough. The error pass is vectorised level by level.

`forced` points (e.g. every border point of a tile) are given an infinite error, so they are always vertices;
two neighbouring tiles that force the same border points join without gaps.
"""

import numpy as np

_CACHE = {}


class RTIN:
    """Triangle hierarchy of an n x n grid (n = 2^k + 1)."""

    def __init__(self, n):
        t = n - 1
        if n < 3 or t & (t - 1):
            raise ValueError(f'grid size must be 2^k + 1, got {n}')
        self.n, self.t = n, t
        num = 2 * t * t - 2
        ids = np.arange(num, dtype=np.int64) + 2
        odd = (ids & 1) == 1
        ax = np.where(odd, 0, t)
        ay = np.where(odd, 0, t)
        bx = np.where(odd, t, 0)
        by = np.where(odd, t, 0)
        cx = np.where(odd, t, 0)
        cy = np.where(odd, 0, t)
        cur = ids.copy()
        while True:
            cur = cur >> 1
            act = cur > 1
            if not act.any():
                break
            mx, my = (ax + bx) >> 1, (ay + by) >> 1
            left = act & ((cur & 1) == 1)
            right = act & ((cur & 1) == 0)
            nax = np.where(left, cx, np.where(right, bx, ax))
            nay = np.where(left, cy, np.where(right, by, ay))
            nbx = np.where(left, ax, np.where(right, cx, bx))
            nby = np.where(left, ay, np.where(right, cy, by))
            cx = np.where(act, mx, cx)
            cy = np.where(act, my, cy)
            ax, ay, bx, by = nax, nay, nbx, nby
        level = np.floor(np.log2(ids)).astype(np.int64) - 1          # 0 for the two biggest triangles
        self.levels = []
        for lv in range(int(level.max()), -1, -1):                   # deepest (smallest triangles) first
            s = level == lv
            self.levels.append((ax[s], ay[s], bx[s], by[s], lv == int(level.max())))

    @classmethod
    def get(cls, n):
        """Cached hierarchy for grid size n."""
        if n not in _CACHE:
            _CACHE[n] = cls(n)
        return _CACHE[n]

    def errors(self, Z, forced=None):
        """Error of every grid point (array [n*n]) for heights Z [n, n] (row = y index, col = x index)."""
        n = self.n
        z = np.asarray(Z, dtype=np.float64).ravel()
        err = np.zeros(n * n, dtype=np.float64)
        if forced is not None:
            err[np.asarray(forced, dtype=bool).ravel()] = np.inf
        for ax, ay, bx, by, deepest in self.levels:
            mx, my = (ax + bx) >> 1, (ay + by) >> 1
            mid = my * n + mx
            e = np.abs(0.5 * (z[ay * n + ax] + z[by * n + bx]) - z[mid])
            if not deepest:
                cx, cy = mx + my - ay, my + ax - mx
                lc = ((ay + cy) >> 1) * n + ((ax + cx) >> 1)
                rc = ((by + cy) >> 1) * n + ((bx + cx) >> 1)
                e = np.maximum(e, np.maximum(err[lc], err[rc]))
            np.maximum.at(err, mid, e)
        return err

    def mesh(self, err, max_error):
        """(vertex grid indices [V] (= row * n + col), triangles [T, 3] into those vertices) for max_error."""
        n, t = self.n, self.t
        tris = []
        stack = [(0, 0, t, t, t, 0), (t, t, 0, 0, 0, t)]
        while stack:
            ax, ay, bx, by, cx, cy = stack.pop()
            mx, my = (ax + bx) >> 1, (ay + by) >> 1
            if abs(ax - cx) + abs(ay - cy) > 1 and err[my * n + mx] > max_error:
                stack.append((cx, cy, ax, ay, mx, my))
                stack.append((bx, by, cx, cy, mx, my))
            else:
                tris.append((ay * n + ax, by * n + bx, cy * n + cx))
        tri = np.array(tris, dtype=np.int64)
        verts, inv = np.unique(tri.ravel(), return_inverse=True)
        return verts, inv.reshape(-1, 3)


def border_mask(n):
    """bool [n, n]: True on the outer ring of the grid (to force tile borders)."""
    m = np.zeros((n, n), dtype=bool)
    m[0, :] = m[-1, :] = m[:, 0] = m[:, -1] = True
    return m
