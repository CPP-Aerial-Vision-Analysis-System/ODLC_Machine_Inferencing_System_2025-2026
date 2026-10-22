"""
sim_worlds/worldgen/layout.py -- PURPOSE: decide WHERE objects go without overlapping, reproducibly.

The Placer keeps a list of circles (x, y, radius) already taken and finds free random spots inside a rectangle
(optionally with extra rules, e.g. "at least 30 m from the tent"). All randomness comes from the numpy
Generator you pass in, so the same --seed always gives the same layout.
"""

import math


class Placer:
    """Find free spots in a rectangle.

    rng: numpy random Generator. rect: an object with x_min, x_max, y_min, y_max OR a survey.Rect.
    inset: metres kept free along every edge.
    """

    def __init__(self, rng, rect, inset=0.0):
        self.rng = rng
        self.rect = rect
        if hasattr(rect, 'x_min'):
            self.x0, self.x1, self.y0, self.y1 = rect.x_min, rect.x_max, rect.y_min, rect.y_max
            self._inside = None
        else:                                         # rotated survey.Rect
            xs = [c[0] for c in rect.corners()]
            ys = [c[1] for c in rect.corners()]
            self.x0, self.x1, self.y0, self.y1 = min(xs), max(xs), min(ys), max(ys)
            self._inside = rect.contains
        self.inset = inset
        self.taken = []

    def free(self, x, y, r, gap=0.0):
        """True when a circle of radius r at (x, y) keeps `gap` metres from every taken circle."""
        return all(math.hypot(x - tx, y - ty) >= r + tr + gap for tx, ty, tr in self.taken)

    def reserve(self, x, y, r):
        """Mark a circle as taken (for objects placed by hand)."""
        self.taken.append((x, y, r))

    def place(self, r, gap=1.0, inset=0.0, tries=600, extra_ok=None):
        """Return a free (x, y) for a circle of radius r and reserve it.

        gap: metres to keep from other objects. inset: extra metres from the rectangle edges.
        extra_ok(x, y) -> bool: optional additional rule. Raises RuntimeError when no spot is found.
        """
        m = self.inset + inset + r
        for _ in range(tries):
            x = self.rng.uniform(self.x0 + m, self.x1 - m)
            y = self.rng.uniform(self.y0 + m, self.y1 - m)
            if self._inside is not None and not self._inside(x, y, m):
                continue
            if self.free(x, y, r, gap) and (extra_ok is None or extra_ok(x, y)):
                self.reserve(x, y, r)
                return x, y
        raise RuntimeError('no free spot found (the area is too crowded for this many objects)')
