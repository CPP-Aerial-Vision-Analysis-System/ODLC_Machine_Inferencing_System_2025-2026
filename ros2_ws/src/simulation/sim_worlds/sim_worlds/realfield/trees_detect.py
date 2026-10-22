"""
sim_worlds/realfield/trees_detect.py -- PURPOSE: find the REAL trees of the SUAS field (part of generator stage
1, needs numpy + scipy) and write prepared/trees.csv: one row per tree with its position, ground height,
height, crown diameter, kind (deciduous / cedar) and where it came from. Also writes the 2023 crown mask
('crown2023', used later to paint the ground under the 3D trees) and the closed-forest mask ('forest') that
stage 3 turns into canopy sheets instead of thousands of single trees.

How, in plain words:
  1. Tree TOPS come from the 2017 lidar canopy height model (CHM): a pixel is a tree top when it is the
     highest point inside a window that grows with the tree height (2 m + 0.35 x height, 3..12 m wide; the
     "variant A" settings of the vegetation research). Trees under 2.5 m are ignored.
  2. CROWN size in 2017: from each top, 8 rays walk outwards until the canopy drops below 45 % of the top height
     or starts rising into a neighbour; the crown radius is the median ray length.
  3. Which pixels of the September 2023 photo are TREE CROWNS? Calibrated on this very photo (2026-10-17, see
     WORLDS.md): lidar crowns have NDVI about 0.43 and brightness about 73 (of 255), sunlit grass NDVI about
     0.20 (95 % below 0.34) and brightness about 96, and tree SHADOWS on grass are dark but weaker in
     near-infrared. Rule on a 0.75 m smoothed photo: (NDVI >= 0.32 and brightness <= 95 and NIR >= 100) or
     (NDVI >= 0.38 and brightness <= 110), then specks smaller than about 1 m are removed. The older rule
     (NDVI >= 0.30 alone) also took green grass for trees and made 28,000 fake trees.
  4. Is a lidar tree STILL THERE? Lots of small trees in the south of Search Boundary 1 were cleared after 2017.
     A lidar tree stays when at least 30 % of a disk around its top is crown in the 2023 photo.
  5. NEW trees (mostly eastern red-cedars that grew since 2017): crown blobs of the 2023 photo that no kept lidar
     tree explains become trees, split into single crowns with a distance transform, but ONLY where the Meta
     2019 canopy height map also sees a tree of 2 m or more (this throws out grass clumps and stray shadows).
     Their height comes from that Meta map (+ growth), or from the crown size when Meta is missing.
  6. Tree KIND: eastern red-cedars are evergreen, so in the March 2025 leaf-off photo they are the only dark,
     olive crowns (bare oaks are bright grey-brown). A tree is a cedar when at least 15 % of a disk around it
     (crown + 2 m, because that photo is shifted by 1-2 m) is "dark in March". Without that photo, tall narrow
     crowns are called cedars.
  7. GROWTH to September 2026: heights grow by about 0.3 m a year for small trees and less for big ones (capped
     at +3 m), as the site_facts research suggested. Crowns take the larger of the 2017 lidar crown (+ growth)
     and the 2023 photo crown. These are ESTIMATES and are marked as such in the csv.
"""

import csv
import math
import os

import numpy as np

from sim_worlds.realfield import grids, site

MIN_TREE_H = 2.5
YEARS_SINCE_LIDAR = 9.6          #: 2017-02-10 -> 2026-09-15
YEARS_SINCE_META = 6.9           #: 2019-10-18 -> 2026-09-15
STILL_THERE_FRACTION = 0.30      #: share of crown pixels (2023) a lidar tree needs to be kept
CEDAR_DARK_FRACTION = 0.15       #: share of "dark in March 2025" pixels that makes a tree a cedar
NEW_TREE_META_MIN_M = 2.0        #: a photo-only tree needs this Meta 2019 canopy height under it
FOREST_MIN_AREA_M2 = 1500.0      #: closed canopy patches smaller than this stay single trees
FOREST_COVER = 0.50              #: share of canopy cells within 20 m that makes a closed forest


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


def run(out, meta, dtm, chm, naip, leafoff, metachm, home, log):
    """Detect trees, write out/trees.csv and the 'crown2023' and 'forest' rasters, add statistics to
    meta['trees']."""
    from scipy import ndimage

    res = chm.res
    c = np.asarray(chm.data, dtype=np.float32)
    cs = ndimage.gaussian_filter(c, 1.0 / res)                      # sigma 1 m
    # --- 1. tops with height-dependent windows -----------------------------------------------------------
    sizes_m = [3, 4, 5, 6, 8, 10, 12]
    cand = cs >= MIN_TREE_H
    is_top = np.zeros_like(cand)
    for i, s in enumerate(sizes_m):
        lo = 0.0 if i == 0 else (sizes_m[i - 1] - 2.0) / 0.35
        hi = (s - 2.0) / 0.35 if i < len(sizes_m) - 1 else 1e9
        band = cand & (cs >= lo) & (cs < hi)
        if band.any():
            is_top |= band & (cs >= ndimage.maximum_filter(cs, size=int(round(s / res)) | 1) - 1e-4)
    rows, cols = np.nonzero(is_top)
    log(f'  trees: {len(rows):,} lidar tree tops (>= {MIN_TREE_H} m) before checks')
    # --- 2. crowns in 2017 by 8 rays ----------------------------------------------------------------------
    dirs = [(math.cos(a), math.sin(a)) for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)]
    raw_h = ndimage.maximum_filter(c, size=3)
    trees = []
    for r0, c0 in zip(rows, cols):
        top = float(cs[r0, c0])
        rmax = min(1.6 * window_m(top), 9.0)
        lens = []
        for dx, dy in dirs:
            prev = top
            length = rmax
            for k in range(1, int(rmax / res) + 1):
                rr = int(round(r0 - dy * k))
                cc = int(round(c0 + dx * k))
                if rr < 0 or cc < 0 or rr >= cs.shape[0] or cc >= cs.shape[1]:
                    length = k * res
                    break
                v = cs[rr, cc]
                if v < 0.45 * top or v > prev + 0.5:
                    length = k * res
                    break
                prev = min(prev, v)
            lens.append(length)
        trees.append({'x': chm.x0 + (c0 + 0.5) * res, 'y': chm.y1 - (r0 + 0.5) * res, 'h2017': float(raw_h[r0, c0]),
                      'r': float(np.clip(np.median(lens), 0.75, 9.0))})
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
    # --- 5. new trees: photo crowns no kept lidar tree explains, confirmed by the Meta 2019 canopy map -----
    explained = np.zeros(crown.shape, dtype=bool)
    for t in kept:
        win, disk = disk_window(cr, t['x'], t['y'], max(t['r'], t['r_photo']) + 1.5)
        if not disk.any():
            continue
        cc, rr = cr.colrow(t['x'], t['y'])
        k = int(math.ceil((max(t['r'], t['r_photo']) + 1.5) / cr.res))
        r0, c0 = int(round(float(rr))), int(round(float(cc)))
        y0, x0 = max(r0 - k, 0), max(c0 - k, 0)
        explained[y0:y0 + disk.shape[0], x0:x0 + disk.shape[1]] |= disk
    newmask = ndimage.binary_opening(crown & ~explained, iterations=3)       # >= ~1.5 m wide blobs only
    del explained
    lab, n = ndimage.label(newmask)
    dist = ndimage.distance_transform_edt(newmask).astype(np.float32)     # float32: half the memory
    dist *= cr.res
    peaks = (dist >= 0.9) & (dist >= ndimage.maximum_filter(dist, size=int(round(3.0 / cr.res)) | 1) - 1e-6)
    pr, pc = np.nonzero(peaks)
    areas = ndimage.sum(np.ones_like(lab), lab, index=np.arange(1, n + 1)) * cr.res ** 2 if n else np.array([])
    del newmask
    new, seen, no_meta = [], set(), 0
    for r0, c0 in zip(pr, pc):
        lb = int(lab[r0, c0])
        if lb == 0 or areas[lb - 1] < 4.0:
            continue
        key = (lb, int(r0 * cr.res / 2.5), int(c0 * cr.res / 2.5))     # at most one tree per 2.5 m cell of a blob
        if key in seen:
            continue
        seen.add(key)
        x, y = cr.x0 + (c0 + 0.5) * cr.res, cr.y1 - (r0 + 0.5) * cr.res
        rad = float(min(dist[r0, c0] * 1.15, 7.0))
        mh = 0.0
        if metachm is not None:
            win, disk = disk_window(metachm, x, y, rad + 1.0)
            mh = float(win[disk].max()) if disk.any() else 0.0
            if mh < NEW_TREE_META_MIN_M:
                no_meta += 1
                continue
        elif areas[lb - 1] < 8.0:
            continue
        new.append({'x': x, 'y': y, 'r': rad, 'r_photo': rad, 'source': 'naip2023', 'leafy2023': 1.0,
                    'h2017': 0.0, 'meta_h': mh})
    del lab, dist, peaks
    log(f'    {len(new):,} new trees found only in the 2023 photo ({no_meta:,} photo blobs rejected: no tree in '
        f'the Meta 2019 canopy map)')
    allt = kept + new
    # --- 6./7. kind, height and crown in 2026 --------------------------------------------------------------
    n_cedar = 0
    for t in allt:
        if t['source'] == 'lidar2017':
            t['h'] = t['h2017'] + growth_m(t['h2017'], YEARS_SINCE_LIDAR)
            t['height_how'] = 'lidar 2017 + estimated growth'
        elif t['meta_h'] >= MIN_TREE_H:
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
    hs = np.array([t['h'] for t in allt]) if allt else np.zeros(1)
    meta['trees'] = {'total': len(allt), 'from_lidar_2017': len(kept), 'new_in_photo_2023': len(new),
                     'lidar_trees_gone_by_2023': dropped, 'photo_blobs_without_meta_tree': no_meta,
                     'cedars': n_cedar, 'in_closed_forest': in_forest,
                     'forest_area_m2': round(float(forest.sum() * res * res)),
                     'crown_mask_share': round(float(crown.mean()), 4),
                     'height_p50_p95_max': [round(float(np.percentile(hs, 50)), 1),
                                            round(float(np.percentile(hs, 95)), 1), round(float(hs.max()), 1)]}
    log(f'    {len(allt):,} trees written ({n_cedar:,} cedars); {in_forest:,} inside closed forest '
        f'({meta["trees"]["forest_area_m2"]:,} m2) -> drawn as canopy sheets')


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
