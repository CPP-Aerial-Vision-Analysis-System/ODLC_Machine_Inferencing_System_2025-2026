"""
sim_worlds/realfield/grids.py -- PURPOSE: the small file format that connects the two generator stages.

Stage 1 ("prepare", GIS libraries) turns the raw downloads into north-up rasters on the WORLD grid and saves
each as a plain NumPy .npy file plus one prepared/grid.json that says where every raster sits. Stage 2
("build", numpy + Pillow only) reads them back with this module. Keeping the format this simple is on purpose:
no GDAL is needed to read it, and np.load(..., mmap_mode='r') reads only the part of a big raster a tile needs.

A raster entry in grid.json:
  {"file": "naip_0p25m.npy", "x0": -950.9, "y1": 1175.9, "res": 0.25, "shape": [rows, cols(, bands)],
   "dtype": "uint8", "what": "...", "nodata": null}
x0 = world x of the LEFT edge of column 0, y1 = world y of the TOP edge of row 0 (row 0 is the northern edge),
res = metres per pixel. Pixel (r, c) covers x in [x0 + c*res, x0 + (c+1)*res], y in [y1 - (r+1)*res, y1 - r*res].
"""

import json
import os

import numpy as np


class Raster:
    """One prepared raster: .data (numpy array, possibly memory-mapped) and its placement in the world."""

    def __init__(self, data, x0, y1, res, what='', nodata=None):
        self.data = data
        self.x0 = float(x0)
        self.y1 = float(y1)
        self.res = float(res)
        self.what = what
        self.nodata = nodata

    @property
    def rows(self):
        """Number of rows (north to south)."""
        return self.data.shape[0]

    @property
    def cols(self):
        """Number of columns (west to east)."""
        return self.data.shape[1]

    @property
    def bounds(self):
        """(x_min, y_min, x_max, y_max) in world metres."""
        return (self.x0, self.y1 - self.rows * self.res, self.x0 + self.cols * self.res, self.y1)

    def colrow(self, x, y):
        """Fractional (col, row) of world point(s) (x, y), measured from pixel CENTRES (so integer = centre)."""
        return (np.asarray(x, dtype=np.float64) - self.x0) / self.res - 0.5, \
            (self.y1 - np.asarray(y, dtype=np.float64)) / self.res - 0.5

    def sample(self, x, y, band=None):
        """Bilinear sample at world point(s) (x, y). Points outside are clamped to the edge. band: index for a
        multi-band raster (None = all bands / single band). Returns float64 numpy values."""
        c, r = self.colrow(x, y)
        c = np.clip(c, 0.0, self.cols - 1.000001)
        r = np.clip(r, 0.0, self.rows - 1.000001)
        c0 = np.floor(c).astype(np.int64)
        r0 = np.floor(r).astype(np.int64)
        fc, fr = c - c0, r - r0
        d = self.data if band is None or self.data.ndim == 2 else self.data[..., band]
        a = d[r0, c0].astype(np.float64)
        b = d[r0, c0 + 1].astype(np.float64)
        cc = d[r0 + 1, c0].astype(np.float64)
        dd = d[r0 + 1, c0 + 1].astype(np.float64)
        if a.ndim > np.ndim(fc):                      # multi-band: broadcast the weights over the band axis
            fc, fr = fc[..., None], fr[..., None]
        return (a * (1 - fc) + b * fc) * (1 - fr) + (cc * (1 - fc) + dd * fc) * fr

    def window(self, x0, y0, x1, y1, pad=0):
        """Integer pixel window (r0, r1, c0, c1) covering world box [x0, x1] x [y0, y1] plus `pad` pixels,
        clipped to the raster."""
        c0 = int(np.floor((x0 - self.x0) / self.res)) - pad
        c1 = int(np.ceil((x1 - self.x0) / self.res)) + pad
        r0 = int(np.floor((self.y1 - y1) / self.res)) - pad
        r1 = int(np.ceil((self.y1 - y0) / self.res)) + pad
        return max(r0, 0), min(r1, self.rows), max(c0, 0), min(c1, self.cols)

    def resample_grid(self, xs, ys, band=None):
        """Bilinear samples on the grid xs (1-D, west->east) x ys (1-D, NORTH->south): array [len(ys), len(xs)]."""
        X, Y = np.meshgrid(xs, ys)
        return self.sample(X, Y, band)


class Prepared:
    """All prepared rasters + metadata of one data folder (prepared/grid.json)."""

    def __init__(self, folder):
        self.folder = folder
        path = os.path.join(folder, 'grid.json')
        if not os.path.isfile(path):
            raise FileNotFoundError(f'{path} not found: run "build_real_field.py prepare" first (see WORLDS.md)')
        with open(path, encoding='utf-8') as f:
            self.meta = json.load(f)
        self._cache = {}

    def has(self, name):
        """True when raster `name` was prepared (optional layers may be missing)."""
        return name in self.meta.get('rasters', {}) and \
            os.path.isfile(os.path.join(self.folder, self.meta['rasters'][name]['file']))

    def raster(self, name, mmap=True):
        """Load raster `name` (e.g. 'dtm', 'naip', 'chm') as a Raster (memory-mapped by default)."""
        if name not in self._cache:
            e = self.meta['rasters'][name]
            data = np.load(os.path.join(self.folder, e['file']), mmap_mode='r' if mmap else None)
            self._cache[name] = Raster(data, e['x0'], e['y1'], e['res'], e.get('what', ''), e.get('nodata'))
        return self._cache[name]

    def path(self, name):
        """Absolute path of a non-raster prepared file listed under meta['files'] (trees.csv, features.json)."""
        return os.path.join(self.folder, self.meta['files'][name])


def save_raster(folder, meta, name, array, x0, y1, res, what, nodata=None):
    """Save one raster as <folder>/<name>_<res>m.npy and register it in meta['rasters'] (meta is grid.json)."""
    fname = f'{name}_{str(res).replace(".", "p")}m.npy'
    np.save(os.path.join(folder, fname), np.ascontiguousarray(array))
    meta.setdefault('rasters', {})[name] = {'file': fname, 'x0': float(x0), 'y1': float(y1), 'res': float(res),
                                           'shape': list(array.shape), 'dtype': str(array.dtype), 'what': what,
                                           'nodata': nodata}


def write_meta(folder, meta):
    """Write prepared/grid.json (pretty, sorted keys so diffs stay readable)."""
    with open(os.path.join(folder, 'grid.json'), 'w', encoding='utf-8', newline='\n') as f:
        json.dump(meta, f, indent=1, sort_keys=True)
        f.write('\n')
