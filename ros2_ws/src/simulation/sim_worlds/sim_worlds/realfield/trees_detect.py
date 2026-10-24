"""
sim_worlds/realfield/trees_detect.py -- PURPOSE: find the REAL trees of the SUAS field (part of generator stage
1, needs numpy + scipy) and write prepared/trees.csv: one row per tree with its position, ground height,
height, crown diameter, kind (deciduous / cedar) and where it came from. Also writes the 2023 crown mask
('crown2023', used later to paint the ground under the 3D trees) and the closed-forest mask ('forest') that
stage 3 turns into canopy sheets instead of thousands of single trees.

How, in plain words (the numbers were CALIBRATED on 2026-10-24 against the research estimates of about 130 trees
in Search Boundary 1, 75 in Search Boundary 2 and 3,100 in the flight boundary; see WORLDS.md):
  1. Tree TOPS come from the 2017 lidar canopy height model (CHM, already free of floating "haze", prepare.py).
     The lidar was flown in February, when oak crowns are bare twigs, so the CHM of one crown is full of holes and
     one crown looked like several trees. Like the research's "variant A" we first build a crown ENVELOPE: the
     highest point per 1 m cell, then a 3 x 3 grey closing (fills the holes between twigs), then a 1 m blur. A
     pixel is a top when it is the highest point inside a window that grows with the tree height (2 m + 0.35 x
     height, 3..12 m wide). Tops under 3 m are ignored, tops over MAX_TREE_H (31 m) are noise (the tallest real
     tree here is 24.6-29.4 m) and tops inside building footprints are roofs, not trees.
  2. CROWN size in 2017: from each top, 8 rays walk outwards (0.5 m steps) until the canopy drops below 45 % of
     the top height or starts rising into a neighbour; the crown radius is the median ray length.
  3. Which pixels of the September 2023 photo are TREE CROWNS? Calibrated on this very photo (2026-10-17):
     lidar crowns have NDVI about 0.43 and brightness about 73 (of 255), sunlit grass NDVI about 0.20 (95 % below
     0.34) and brightness about 96, and tree SHADOWS on grass are dark but weaker in near-infrared. Rule on a
     0.75 m smoothed photo: (NDVI >= 0.32 and brightness <= 95 and NIR >= 100) or (NDVI >= 0.38 and brightness
     <= 110), then specks smaller than about 1 m are removed.
  4. Is a lidar tree STILL THERE? Lots of small trees in the south of Search Boundary 1 were cleared after 2017.
     A lidar tree stays when at least 30 % of a disk around its top is crown in the 2023 photo.
  5. NEW trees (mostly eastern red-cedars that grew since 2017, and crowns the leaf-off lidar missed): crown blobs
     of the 2023 photo (on a 0.5 m grid) that no kept lidar tree explains. The first version put one tree in every
     2 m cell of a blob (3 x too many trees, 3-4 m crowns; research fix 2). Now: one tree at the "fattest" point
     of the blob inside a PHOTO_WINDOW_M window, at least PHOTO_SPACING_M apart, at least 2 x
     PHOTO_MIN_HALF_WIDTH_M wide; every blob pixel then belongs to its nearest tree, so the crown diameter is
     the size of the patch it really covers (median about 7 m, like the DeepForest AI crowns on the same photo).
     A new tree must be confirmed: the Meta 2019 canopy map sees >= 2 m there, OR the 2017 lidar already saw a
     young tree of >= YOUNG_TREE_LIDAR_M (Meta misses about half of the small trees in Search Boundary 2).
  6. Tree KIND: eastern red-cedars are evergreen, so in the March 2025 leaf-off photo they are the only dark,
     olive crowns (bare oaks are bright grey-brown). A tree is a cedar when at least 15 % of a disk around it
     (crown + 2 m, because that photo is shifted by 1-2 m) is "dark in March". That photo is used ONLY as this
     yes/no mask (its licence does not allow redistribution); without it, tall narrow crowns are called cedars.
  7. GROWTH to September 2026: heights grow by about 0.3 m a year for small trees and less for big ones (capped
     at +3 m), as the site_facts research suggested. Crowns take the larger of the 2017 lidar crown (+ growth)
     and the 2023 photo crown. These are ESTIMATES and are marked as such in the csv.
"""

import csv
import math
import os

import numpy as np

from sim_worlds import geo
from sim_worlds.realfield import grids, site

MIN_TREE_H = 3.0                 #: lidar tops lower than this are shrubs (research variant A)
MAX_TREE_H = 31.0                #: lidar tops higher than this are noise: tallest real tree 24.6-29.4 m (2017)
YEARS_SINCE_LIDAR = 9.6          #: 2017-02-10 -> 2026-09-15
YEARS_SINCE_META = 6.9           #: 2019-10-18 -> 2026-09-15
STILL_THERE_FRACTION = 0.30      #: share of crown pixels (2023) a lidar tree needs to be kept
CEDAR_DARK_FRACTION = 0.15       #: share of "dark in March 2025" pixels that makes a tree a cedar
NEW_TREE_META_MIN_M = 2.0        #: a photo-only tree is confirmed by this Meta 2019 canopy height under it ...
YOUNG_TREE_LIDAR_M = 1.0         #: ... or by a young tree of this height in the 2017 lidar
PHOTO_RES = 0.5                  #: grid (m) of the photo-only step (the 0.25 m crown mask averaged 2 x 2)
PHOTO_WINDOW_M = 10.0            #: photo-only trees: peak window ...
PHOTO_SPACING_M = 6.0            #: ... and smallest distance between two new trees (calibrated, see module text)
PHOTO_MIN_HALF_WIDTH_M = 1.2     #: a new tree's blob must be at least 2 x this wide at its centre
PHOTO_MIN_AREA_M2 = 4.0          #: ... and its crown patch at least this big
FOREST_MIN_AREA_M2 = 1500.0      #: closed canopy patches smaller than this stay single trees
FOREST_COVER = 0.50              #: share of canopy cells within 20 m that makes a closed forest
ROOF_MARGIN_M = 1.0              #: tops this close to a building footprint are roofs / eaves


def window_m(h):
    """Local-maximum window (m) for a tree of height h (variant A of the vegetation research)."""
    return float(np.clip(2.0 + 0.35 * h, 3.0, 12.0))


def growth_m(h, years):
    """Estimated height growth (m) of a tree of height h over `years` (capped at 3 m). Works on arrays too."""
    h = np.asarray(h, dtype=np.float64)
    rate = np.where(h < 10, 0.30, np.where(h < 16, 0.22, 0.12))
    g = np.minimum(rate * years, 3.0)
    return float(g) if g.ndim == 0 else g


def crown_mask(naip_rgbn, block_rows=1024):
    """2023 tree-crown pixels of the NAIP photo (uint8 R,G,B,NIR array [rows, cols, 4]) -> bool array (rule in
    the module text). Works in blocks of rows (with a 1-row overlap for the smoothing), so the float copy of a
    7520 x 7520 photo never exists all at once."""
    from scipy import ndimage
    rows = naip_rgbn.shape[0]
    out = np.zeros(naip_rgbn.shape[:2], dtype=bool)
    for r0 in range(0, rows, block_rows):
        r1 = min(r0 + block_rows, rows)
        lo, hi = max(r0 - 1, 0), min(r1 + 1, rows)
        a = ndimage.uniform_filter(np.asarray(naip_rgbn[lo:hi], dtype=np.float32), size=(3, 3, 1))
        R, G, B, N = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
        nd = (N - R) / (N + R + 1e-6)
        br = (R + G + B) / 3.0
        m = ((nd >= 0.32) & (br <= 95.0) & (N >= 100.0)) | ((nd >= 0.38) & (br <= 110.0))
        out[r0:r1] = m[r0 - lo:r0 - lo + (r1 - r0)]
    return ndimage.binary_opening(out, iterations=2)


def disk_window(raster, x, y, radius):
    """(window_array, disk_bool) of a Raster around world point (x, y) with the given radius (m); the disk
    marks the pixels whose centres are inside the circle. Empty arrays at the raster edge."""
    c, r = raster.colrow(x, y)
    k = int(math.ceil(radius / raster.res))
    r0, c0 = int(round(float(r))), int(round(float(c)))
    y0, y1 = max(r0 - k, 0), min(r0 + k + 1, raster.rows)
    x0, x1 = max(c0 - k, 0), min(c0 + k + 1, raster.cols)
    if y1 <= y0 or x1 <= x0:
        return np.zeros((0, 0)), np.zeros((0, 0), dtype=bool)
    win = np.asarray(raster.data[y0:y1, x0:x1])
    yy, xx = np.mgrid[y0 - r0:y1 - r0, x0 - c0:x1 - c0]
    return win, (yy ** 2 + xx ** 2) * raster.res ** 2 <= radius ** 2


def disk_fraction(raster, x, y, radius):
    """Share (0..1) of True/1 pixels of a mask Raster inside a disk (0 when the disk is outside the raster)."""
    win, disk = disk_window(raster, x, y, radius)
    return float(win[disk].mean()) if disk.any() else 0.0


def _stamp_disk(mask, raster, x, y, radius):
    """Set the pixels of `mask` (on raster's grid) whose centres are within `radius` of (x, y) to True."""
    win, disk = disk_window(raster, x, y, radius)
    if not disk.any():
        return
    c, r = raster.colrow(x, y)
    k = int(math.ceil(radius / raster.res))
    r0, c0 = int(round(float(r))), int(round(float(c)))
    y0, x0 = max(r0 - k, 0), max(c0 - k, 0)
    mask[y0:y0 + disk.shape[0], x0:x0 + disk.shape[1]] |= disk


def building_mask(buildings, x0, y1, cols, rows, res, grow_m):
    """bool [rows, cols]: pixel centres inside a building footprint grown by grow_m metres."""
    from scipy import ndimage
    m = np.zeros((rows, cols), dtype=bool)
    for b in buildings or []:
        poly = b['polygon']
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        c0 = max(int((min(xs) - x0) / res) - 1, 0)
        c1 = min(int((max(xs) - x0) / res) + 2, cols)
        r0 = max(int((y1 - max(ys)) / res) - 1, 0)
        r1 = min(int((y1 - min(ys)) / res) + 2, rows)
        if c1 <= c0 or r1 <= r0:
            continue
        X, Y = np.meshgrid(x0 + (np.arange(c0, c1) + 0.5) * res, y1 - (np.arange(r0, r1) + 0.5) * res)
        inside = np.zeros(X.shape, dtype=bool)
        n = len(poly)
        for i in range(n):
            xa, ya = poly[i]
            xb, yb = poly[(i + 1) % n]
            cond = (ya > Y) != (yb > Y)
            xi = (xb - xa) * (Y - ya) / ((yb - ya) if yb != ya else 1e-12) + xa
            inside ^= cond & (X < xi)
        m[r0:r1, c0:c1] |= inside
    it = int(round(grow_m / res))
    return ndimage.binary_dilation(m, iterations=it) if it > 0 and m.any() else m


def lidar_tops(chm, log):
    """Variant-A tree tops from the 0.5 m canopy model: (x, y, height_2017, crown_radius, envelope Raster)."""
    from scipy import ndimage
    c = np.asarray(chm.data, dtype=np.float32)
    h, w = c.shape[0] // 2, c.shape[1] // 2
    env = c[:2 * h, :2 * w].reshape(h, 2, w, 2).max(axis=(1, 3))        # highest point per 1 m cell
    env = ndimage.grey_closing(env, size=(3, 3))                        # fill the holes of leaf-off crowns
    cs = ndimage.gaussian_filter(env, 1.0)                              # sigma 1 m
    res = 2.0 * chm.res
    sizes_m = [3, 4, 5, 6, 8, 10, 12]
    cand = cs >= MIN_TREE_H
    is_top = np.zeros_like(cand)
    for i, s in enumerate(sizes_m):
        lo = 0.0 if i == 0 else (sizes_m[i - 1] - 2.0) / 0.35
        hi = (s - 2.0) / 0.35 if i < len(sizes_m) - 1 else 1e9
        band = cand & (cs >= lo) & (cs < hi)
        if band.any():
            k = max(3, int(round(s / res)) | 1)
            is_top |= band & (cs >= ndimage.maximum_filter(cs, size=k) - 1e-4)
    rows, cols = np.nonzero(is_top)
    raw_h = ndimage.maximum_filter(env, size=3)
    xs = chm.x0 + (cols + 0.5) * res
    ys = chm.y1 - (rows + 0.5) * res
    hs = raw_h[rows, cols].astype(np.float64)
    dirs = [(math.cos(a), math.sin(a)) for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)]
    step = 0.5                                                           # metres per ray step
    rads = np.empty(len(rows))
    for i, (r0, c0) in enumerate(zip(rows, cols)):
        top = float(cs[r0, c0])
        rmax = min(1.6 * window_m(top), 9.0)
        lens = []
        for dx, dy in dirs:
            prev, length = top, rmax
            for k in range(1, int(rmax / step) + 1):
                rr = int(round(r0 - dy * k * step / res))
                cc = int(round(c0 + dx * k * step / res))
                if rr < 0 or cc < 0 or rr >= cs.shape[0] or cc >= cs.shape[1]:
                    length = k * step
                    break
                v = cs[rr, cc]
                if v < 0.45 * top or v > prev + 0.5:
                    length = k * step
                    break
                prev = min(prev, v)
            lens.append(length)
        rads[i] = float(np.clip(np.median(lens), 0.75, 9.0))
    log(f'  trees: {len(rows):,} lidar tree tops (>= {MIN_TREE_H:g} m, variant-A crown envelope) before checks')
    return xs, ys, hs, rads, grids.Raster(env, chm.x0, chm.y1, res)


def photo_only_trees(crown, cr, kept, meta_chm, env, roofs05, log):
    """Step 5 of the module text. crown: bool 0.25 m mask (Raster cr). kept: list of kept lidar tree dicts.
    meta_chm: Raster or None. env: 1 m lidar envelope Raster. roofs05: bool mask on the 0.5 m grid.
    Returns (list of new tree dicts, statistics dict)."""
    from scipy import ndimage
    f = int(round(PHOTO_RES / cr.res))
    h2, w2 = crown.shape[0] // f, crown.shape[1] // f
    c05 = crown[:h2 * f, :w2 * f].reshape(h2, f, w2, f).mean(axis=(1, 3)) >= 0.5
    grid = grids.Raster(c05, cr.x0, cr.y1, PHOTO_RES)
    explained = np.zeros(c05.shape, dtype=bool)
    for t in kept:
        _stamp_disk(explained, grid, t['x'], t['y'], max(t['r'], t['r_photo']) + 1.5)
    newmask = ndimage.binary_opening(c05 & ~explained & ~roofs05, iterations=1)
    del explained
    lab, _ = ndimage.label(newmask)
    dist = ndimage.distance_transform_edt(newmask).astype(np.float32) * PHOTO_RES
    k = max(3, int(round(PHOTO_WINDOW_M / PHOTO_RES)) | 1)
    peaks = (dist >= PHOTO_MIN_HALF_WIDTH_M) & (dist >= ndimage.maximum_filter(dist, size=k) - 1e-6)
    pr, pc = np.nonzero(peaks)
    order = np.argsort(-dist[pr, pc], kind='stable')                    # fattest first, reproducible
    pr, pc = pr[order], pc[order]
    taken = np.zeros(c05.shape, dtype=bool)
    sk = int(math.ceil(PHOTO_SPACING_M / PHOTO_RES))
    keep_r, keep_c = [], []
    for r0, c0 in zip(pr, pc):
        if taken[r0, c0]:
            continue
        keep_r.append(r0)
        keep_c.append(c0)
        y0, x0 = max(r0 - sk, 0), max(c0 - sk, 0)
        y1, x1 = min(r0 + sk + 1, taken.shape[0]), min(c0 + sk + 1, taken.shape[1])
        yy, xx = np.mgrid[y0:y1, x0:x1]
        taken[y0:y1, x0:x1] |= ((yy - r0) ** 2 + (xx - c0) ** 2) * PHOTO_RES ** 2 < PHOTO_SPACING_M ** 2
    del taken, peaks
    keep_r = np.array(keep_r, dtype=np.int64)
    keep_c = np.array(keep_c, dtype=np.int64)
    # every blob pixel belongs to its nearest new tree of the SAME blob: that patch is the tree's crown
    markers = np.zeros(c05.shape, dtype=np.int32)
    markers[keep_r, keep_c] = np.arange(1, len(keep_r) + 1, dtype=np.int32)
    idx = ndimage.distance_transform_edt(markers == 0, return_distances=False, return_indices=True)
    owner = markers[idx[0], idx[1]]
    del idx, markers
    marker_lab = np.concatenate([[0], lab[keep_r, keep_c]]) if len(keep_r) else np.zeros(1, dtype=lab.dtype)
    owner[~newmask | (lab != marker_lab[owner])] = 0
    area = np.bincount(owner.ravel(), minlength=len(keep_r) + 1)[1:] * PHOTO_RES ** 2
    del owner, lab, dist, newmask
    diam = 2.0 * np.sqrt(area / math.pi)
    xs = cr.x0 + (keep_c + 0.5) * PHOTO_RES
    ys = cr.y1 - (keep_r + 0.5) * PHOTO_RES
    new, small, unconfirmed, young = [], 0, 0, 0
    for x, y, d, a in zip(xs, ys, diam, area):
        if a < PHOTO_MIN_AREA_M2:
            small += 1
            continue
        rad = float(d / 2.0)
        mh = 0.0
        if meta_chm is not None:
            win, disk = disk_window(meta_chm, x, y, max(rad, 1.0) + 1.0)
            mh = float(win[disk].max()) if disk.any() else 0.0
        ok = mh >= NEW_TREE_META_MIN_M
        if not ok:
            win, disk = disk_window(env, x, y, max(rad, 1.0))
            ok = bool(disk.any()) and float(win[disk].max()) >= YOUNG_TREE_LIDAR_M
            young += int(ok)
        if not ok:
            unconfirmed += 1
            continue
        new.append({'x': float(x), 'y': float(y), 'r': rad, 'r_photo': rad, 'source': 'naip2023', 'leafy2023': 1.0,
                    'h2017': 0.0, 'meta_h': mh})
    stats = {'candidates': int(len(xs)), 'too_small': small, 'not_confirmed': unconfirmed,
             'confirmed_by_young_lidar_tree': young, 'kept': len(new)}
    log(f'    {len(new):,} new trees found only in the 2023 photo ({unconfirmed:,} blobs rejected: no tree in the Meta '
        f'2019 canopy map nor a young tree in the 2017 lidar; {small:,} too small)')
    return new, stats


def area_counts(trees, home):
    """{'flight_boundary' | 'search_boundary_1' | 'search_boundary_2': {'lidar2017': n, 'naip2023': n, 'total': n}}."""
    fb = site.flight_boundary_xy(home)
    boxes = {n: site.search_box_xy(n, home) for n in site.SEARCH_BOUNDARIES_LATLON}
    out = {'flight_boundary': {'lidar2017': 0, 'naip2023': 0}}
    out.update({n: {'lidar2017': 0, 'naip2023': 0} for n in boxes})
    for t in trees:
        if geo.point_in_polygon(t['x'], t['y'], fb):
            out['flight_boundary'][t['source']] += 1
        for n, b in boxes.items():
            if b[0] <= t['x'] <= b[2] and b[1] <= t['y'] <= b[3]:
                out[n][t['source']] += 1
    for v in out.values():
        v['total'] = v['lidar2017'] + v['naip2023']
    return out


def run(out, meta, dtm, chm, naip, leafoff, metachm, home, log, buildings=None):
    """Detect trees, write out/trees.csv and the 'crown2023' and 'forest' rasters, add statistics to
    meta['trees']. buildings: footprint list of features.json (tops / new trees on roofs are dropped)."""
    from scipy import ndimage

    res = chm.res
    c = np.asarray(chm.data, dtype=np.float32)
    # --- 1./2. variant-A tops, crowns by 8 rays; drop haze and roofs -----------------------------------------
    xs, ys, hs, rads, env = lidar_tops(chm, log)
    roofs1 = building_mask(buildings, env.x0, env.y1, env.cols, env.rows, env.res, ROOF_MARGIN_M)
    rc, rr = env.colrow(xs, ys)
    on_roof = roofs1[np.clip(np.round(rr).astype(int), 0, env.rows - 1), np.clip(np.round(rc).astype(int), 0, env.cols - 1)]
    haze = hs > MAX_TREE_H
    trees = [{'x': float(x), 'y': float(y), 'h2017': float(h), 'r': float(r)}
             for x, y, h, r, bad in zip(xs, ys, hs, rads, haze | on_roof) if not bad]
    log(f'    dropped {int(haze.sum())} tops over {MAX_TREE_H:g} m (lidar noise) and {int((on_roof & ~haze).sum())} tops '
        'on building roofs')
    # --- 3./4. 2023 crowns; which lidar trees are still there ----------------------------------------------
    crown = crown_mask(np.asarray(naip.data))
    cr = grids.Raster(crown, naip.x0, naip.y1, naip.res)
    log(f'    2023 crown mask: {100 * crown.mean():.1f} % of the area')
    kept, dropped = [], 0
    for t in trees:
        frac = disk_fraction(cr, t['x'], t['y'], max(0.7 * t['r'], 1.0) + 0.5)
        if frac < STILL_THERE_FRACTION:
            dropped += 1
            continue
        win, disk = disk_window(cr, t['x'], t['y'], min(t['r'] + 2.5, 8.0))
        area = float(win[disk].sum()) * cr.res ** 2 if disk.any() else 0.0
        t.update(leafy2023=round(frac, 2), source='lidar2017', r_photo=math.sqrt(area / math.pi))
        kept.append(t)
    log(f'    {len(kept):,} lidar trees still there in 2023, {dropped:,} gone (cleared since the 2017 lidar)')
    # --- 5. new trees from the photo ------------------------------------------------------------------------
    roofs05 = building_mask(buildings, cr.x0, cr.y1, crown.shape[1] // 2, crown.shape[0] // 2, PHOTO_RES, ROOF_MARGIN_M)
    new, photo_stats = photo_only_trees(crown, cr, kept, metachm, env, roofs05, log)
    del roofs05
    allt = kept + new
    # --- 6./7. kind, height and crown in 2026 --------------------------------------------------------------
    n_cedar = 0
    for t in allt:
        if t['source'] == 'lidar2017':
            t['h'] = t['h2017'] + growth_m(t['h2017'], YEARS_SINCE_LIDAR)
            t['height_how'] = 'lidar 2017 + estimated growth'
        elif t['meta_h'] >= 2.5:
            t['h'] = t['meta_h'] + growth_m(t['meta_h'], YEARS_SINCE_META)
            t['height_how'] = 'Meta canopy height 2019 + estimated growth'
        else:
            t['h'] = None
            t['height_how'] = 'from crown size (no height data)'
        dark = None
        if leafoff is not None:
            dark = disk_fraction(leafoff, t['x'], t['y'], t['r'] + 2.0)
        t['dark_march'] = dark
        d_lidar = 2.0 * t['r'] + (0.08 * YEARS_SINCE_LIDAR if t['source'] == 'lidar2017' else 0.0)
        d_photo = 2.0 * t['r_photo']
        d = max(d_lidar, min(d_photo, d_lidar + 3.0)) if t['source'] == 'lidar2017' else d_photo
        if t['h'] is None:
            ever_guess = dark is not None and dark >= CEDAR_DARK_FRACTION
            t['h'] = float(np.clip(1.7 * d, 2.5, 9.0)) if ever_guess else float(np.clip(1.0 * d + 1.5, 3.0, 14.0))
        ever = (dark >= CEDAR_DARK_FRACTION) if dark is not None else (d / max(t['h'], 0.1) < 0.42 and t['h'] < 12.0)
        t['kind'] = 'cedar' if ever else 'deciduous'
        n_cedar += int(ever)
        t['crown'] = float(np.clip(d, 1.5, 18.0))
        if t['kind'] == 'cedar':
            t['crown'] = min(t['crown'], 0.75 * t['h'])            # red-cedars are narrow cones
    # --- closed forest: dense canopy in both 2017 (lidar) and 2023 (photo) -----------------------------------
    # The 2017 lidar was flown leaf-off, so a 0.5 m canopy cell inside a deciduous forest is often empty: use the
    # highest return within 1.5 m, and the share of 2023 crown pixels, averaged over 20 m.
    cmax = ndimage.maximum_filter(c, size=5)
    crown05 = _resample_mask(crown, cr, chm)
    # crowns OR the dark gaps between them (shadows inside a wood are not "crown" pixels but are not grass either)
    closed = (cmax > 2.5) & (crown05 > 0.25)
    cover = ndimage.uniform_filter(closed.astype(np.float32), size=int(round(20.0 / res)))
    forest = ndimage.binary_opening(cover >= FOREST_COVER, iterations=int(round(3.0 / res)))
    lab, n = ndimage.label(forest)
    if n:
        a = ndimage.sum(np.ones_like(lab), lab, index=np.arange(1, n + 1)) * res * res
        forest = np.isin(lab, np.nonzero(a >= FOREST_MIN_AREA_M2)[0] + 1)
    # never inside the search areas (+30 m): there every tree is its own 3D model (targets hide next to them)
    X, Y = _centres(chm)
    for name in site.SEARCH_BOUNDARIES_LATLON:
        x0, y0, x1, y1 = site.search_box_xy(name, home)
        forest &= ~((X > x0 - 30) & (X < x1 + 30) & (Y > y0 - 30) & (Y < y1 + 30))
    del X, Y, cover, closed, cmax
    grids.save_raster(out, meta, 'forest', forest.astype(np.uint8), chm.x0, chm.y1, res,
                      '1 = closed forest canopy in 2017 lidar AND 2023 photo (drawn as canopy sheets)')
    grids.save_raster(out, meta, 'crown2023', crown.astype(np.uint8), cr.x0, cr.y1, cr.res,
                      '1 = tree crown in the 2023 NAIP photo (calibrated rule, trees_detect.crown_mask)')
    fr = grids.Raster(forest.astype(np.uint8), chm.x0, chm.y1, res)
    in_forest = 0
    for t in allt:
        t['forest'] = int(fr.sample(t['x'], t['y']) > 0.5)
        in_forest += t['forest']
    # --- write ------------------------------------------------------------------------------------------
    allt.sort(key=lambda t: (round(t['y'], 1), round(t['x'], 1)))
    path = os.path.join(out, 'trees.csv')
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['id', 'x', 'y', 'ground_navd88', 'height_m', 'crown_diam_m', 'kind', 'source', 'forest',
                    'lidar_height_2017', 'crown_share_2023', 'dark_march_2025', 'height_how'])
        for i, t in enumerate(allt):
            g = float(dtm.sample(t['x'], t['y']))
            w.writerow([i, f'{t["x"]:.2f}', f'{t["y"]:.2f}', f'{g:.2f}', f'{t["h"]:.2f}', f'{t["crown"]:.2f}',
                        t['kind'], t['source'], t['forest'], f'{t["h2017"]:.2f}', t['leafy2023'],
                        '' if t['dark_march'] is None else f'{t["dark_march"]:.2f}', t['height_how']])
    meta.setdefault('files', {})['trees'] = 'trees.csv'
    hs_all = np.array([t['h'] for t in allt]) if allt else np.zeros(1)
    photo_d = [t['crown'] for t in new]
    meta['trees'] = {'total': len(allt), 'from_lidar_2017': len(kept), 'new_in_photo_2023': len(new),
                     'lidar_trees_gone_by_2023': dropped, 'lidar_tops_dropped_haze': int(haze.sum()),
                     'lidar_tops_dropped_roofs': int((on_roof & ~haze).sum()), 'photo_only': photo_stats,
                     'photo_only_crown_median_m': round(float(np.median(photo_d)), 2) if photo_d else None,
                     'cedars': n_cedar, 'in_closed_forest': in_forest,
                     'forest_area_m2': round(float(forest.sum() * res * res)),
                     'crown_mask_share': round(float(crown.mean()), 4),
                     'height_p50_p95_max': [round(float(np.percentile(hs_all, 50)), 1),
                                            round(float(np.percentile(hs_all, 95)), 1), round(float(hs_all.max()), 1)],
                     'by_area': area_counts(allt, home),
                     'settings': {'min_tree_h': MIN_TREE_H, 'max_tree_h': MAX_TREE_H, 'photo_window_m': PHOTO_WINDOW_M,
                                  'photo_spacing_m': PHOTO_SPACING_M, 'photo_min_half_width_m': PHOTO_MIN_HALF_WIDTH_M,
                                  'young_tree_lidar_m': YOUNG_TREE_LIDAR_M, 'meta_min_m': NEW_TREE_META_MIN_M}}
    ba = meta['trees']['by_area']
    log(f'    {len(allt):,} trees written ({n_cedar:,} cedars); {in_forest:,} inside closed forest '
        f'({meta["trees"]["forest_area_m2"]:,} m2) -> drawn as canopy sheets')
    log(f'    per area (research estimate in brackets): SB1 {ba["search_boundary_1"]["total"]} (~130), SB2 '
        f'{ba["search_boundary_2"]["total"]} (~75), flight boundary {ba["flight_boundary"]["total"]:,} (~3,100)')


def _centres(r):
    """World x, y of the pixel centres of a Raster (2-D arrays)."""
    xs = r.x0 + (np.arange(r.cols) + 0.5) * r.res
    ys = r.y1 - (np.arange(r.rows) + 0.5) * r.res
    return np.meshgrid(xs, ys)


def _resample_mask(mask, src, dst):
    """Fraction of True pixels of `mask` (on Raster src's grid) inside each pixel of Raster dst's grid.
    Both grids share the same corner (prepare.world_grid), and dst.res is a whole multiple of src.res."""
    f = int(round(dst.res / src.res))
    rows, cols = dst.rows, dst.cols
    m = np.zeros((rows * f, cols * f), dtype=np.float32)
    h = min(mask.shape[0], rows * f)
    w = min(mask.shape[1], cols * f)
    m[:h, :w] = mask[:h, :w]
    return m.reshape(rows, f, cols, f).mean(axis=(1, 3))
