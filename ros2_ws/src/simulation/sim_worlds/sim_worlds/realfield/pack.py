"""
sim_worlds/realfield/pack.py -- PURPOSE: generator stage 2 ("pack"): turn the prepared full-resolution grids of
stage 1 (hundreds of MB, NOT in git) into the compact SITE PACK that IS committed to git:
sim_worlds/data/suas_field_real/ (about 12 MB; format in sitepack.py). Stage 3 (Gazebo assets) and stage 4
(world, targets, missions) read only this pack, so the team never needs the raw data or GIS libraries.

What happens to the ground picture here (in this order), and why:
  1. Season: the NAIP photo is from 18 Sept 2023; competition week (15-17 Sept 2026) was greener and darker after
     rain. Grass pixels are multiplied by the grade stage 1 MEASURED with Sentinel-2 over pure-grass cells (after
     removing the +1000 storage offset, research fix 4), or by site.GRASS_GRADE_RGB when no Sentinel-2 pair was
     downloaded; trees are left alone.
  2. Changes since 2023: the new wetland west of Search Boundary 1 (bare earthwork in 2023, green in 2026) is
     repainted wet and green; the buried-pipeline lane through SB1 (lighter and less green in Sept 2026
     Sentinel-2) is lightened along the OpenStreetMap line.
  3. Trees painted OUT: every tree that becomes a 3D model in Gazebo would otherwise appear twice (once as paint
     on the ground, once as a model). Its crown pixels are filled with the colour of the nearest open ground
     (tree shadows are left: the world's sun shines from the same direction as on the photo day, see site.py).
  4. Ground classes (grass, dry grass, bare soil, gravel, asphalt, water, canopy, under-tree, wetland) for the
     search areas: stage 3 uses them to add the matching CC0 centimetre detail.
  5. Heights: the 0.5 m lidar ground (with the stage-1 edits) is averaged to 1 m and the pad square is levelled
     to the home height, so the 12.2 m concrete pad model sits flush.
Needs numpy, scipy and Pillow (the GIS venv of stage 1 has them).
"""

import csv
import datetime
import json
import math
import os
import shutil

import numpy as np

from sim_worlds import geo
from sim_worlds.realfield import grids, prepare, site
from sim_worlds.realfield.sitepack import CLASSES

PACK_VERSION = 1
HEIGHT_RES, HEIGHT_OFFSET, HEIGHT_SCALE = 1.0, 150.0, 0.01     #: 16-bit PNG: m NAVD88 = 150 + v * 0.01
FAR_RES, NEAR_RES = 1.0, 0.25
CANOPY_SCALE = 0.125                                            #: canopy PNG: metres = v * 0.125
PACK_MARGIN_M = 4.0                                             #: rasters reach this far past the terrain
PAD_FLAT_FEATHER_M = 2.0
#: CC0 detail pictures (Poly Haven): manifest file name -> (pack name, ground use, real size m, px, amplitude)
DETAIL = {
    'textures/aerial_grass_rock_diff_2k.jpg': ('grass_aerial', 'grass', 15.0, 1024, 0.55),
    'textures/grass_ground_diff_2k.jpg': ('grass_close', 'grass', 2.51, 512, 0.35),
    'textures/dry_ground_01_diff_2k.jpg': ('dry_ground', 'dry_grass', 4.0, 512, 0.50),
    'textures/dirt_aerial_02_diff_2k.jpg': ('dirt_aerial', 'bare_soil', 20.0, 1024, 0.50),
    'textures/gravel_road_diff_2k.jpg': ('gravel', 'gravel', 2.0, 512, 0.45),
    'textures/aerial_asphalt_01_diff_2k.jpg': ('asphalt', 'asphalt', 30.0, 1024, 0.40),
}


def pack_dir(pkg):
    """<pkg>/data/suas_field_real."""
    return os.path.join(pkg, 'data', site.WORLD)


def zone_boxes(home):
    """(near_box, detail_box): world boxes (x0, y0, x1, y1) around all non-far tiles and all detail tiles."""
    tiles = site.tile_layout(home)

    def bbox(ts):
        return (min(t['x0'] for t in ts), min(t['y0'] for t in ts), max(t['x1'] for t in ts), max(t['y1'] for t in ts))
    return bbox([t for t in tiles if t['kind'] != 'far']), bbox([t for t in tiles if t['kind'] == 'detail'])


def _grid(box, res, margin):
    """(x0, y1, cols, rows) of a north-up grid covering box + margin."""
    x0, y0, x1, y1 = box
    return x0 - margin, y1 + margin, int(round((x1 - x0 + 2 * margin) / res)), int(round((y1 - y0 + 2 * margin) / res))


def _crop(r, x0, y1, cols, rows):
    """Integer-aligned crop of Raster r: the block of r.data whose top-left pixel corner is (x0, y1)."""
    c0 = int(round((x0 - r.x0) / r.res))
    r0 = int(round((r.y1 - y1) / r.res))
    if c0 < 0 or r0 < 0 or c0 + cols > r.cols or r0 + rows > r.rows:
        raise ValueError(f'crop outside the prepared raster ({r.what})')
    return np.asarray(r.data[r0:r0 + rows, c0:c0 + cols])


def _block_mean(a, f):
    """Average f x f pixel blocks (a's first two sizes must be multiples of f)."""
    h, w = a.shape[0] // f, a.shape[1] // f
    a = a[:h * f, :w * f].astype(np.float32)
    return a.reshape(h, f, w, f, *a.shape[2:]).mean(axis=(1, 3))


def _centres(x0, y1, cols, rows, res):
    xs = x0 + (np.arange(cols) + 0.5) * res
    ys = y1 - (np.arange(rows) + 0.5) * res
    return np.meshgrid(xs, ys)


def _line_distance(X, Y, line, max_d=50.0):
    """Distance (m) from every point to a polyline [(x, y), ...]; segments farther than max_d from the whole
    window are skipped (their distance stays 'inf'), which keeps 100+ roads fast."""
    d = np.full(X.shape, np.inf, dtype=np.float32)
    wx0, wx1, wy0, wy1 = float(X.min()), float(X.max()), float(Y.min()), float(Y.max())
    for (ax, ay), (bx, by) in zip(line[:-1], line[1:]):
        if (max(ax, bx) < wx0 - max_d or min(ax, bx) > wx1 + max_d or max(ay, by) < wy0 - max_d
                or min(ay, by) > wy1 + max_d):
            continue
        vx, vy = bx - ax, by - ay
        L2 = vx * vx + vy * vy
        if L2 < 1e-9:
            continue
        t = np.clip(((X - ax) * vx + (Y - ay) * vy) / L2, 0.0, 1.0)
        d = np.minimum(d, np.hypot(X - (ax + t * vx), Y - (ay + t * vy)).astype(np.float32))
    return d


def _polygon_mask(X, Y, poly):
    """Even-odd point-in-polygon for arrays X, Y."""
    inside = np.zeros(X.shape, dtype=bool)
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        cond = (y1 > Y) != (y2 > Y)
        xi = (x2 - x1) * (Y - y1) / ((y2 - y1) if y2 != y1 else 1e-12) + x1
        inside ^= cond & (X < xi)
    return inside


def season_grade(prepared_meta):
    """(R, G, B) grass multipliers: the grade stage 1 measured (prepare.prepare_season_grade), else the
    site_facts fallback site.GRASS_GRADE_RGB. Returns (factors, where-from text)."""
    g = (prepared_meta or {}).get('season_grade') or {}
    f = g.get('factors_rgb')
    if f and len(f) == 3 and all(0.6 <= float(v) <= 1.3 for v in f):
        return tuple(float(v) for v in f), g.get('how', 'measured in stage 1')
    return tuple(site.GRASS_GRADE_RGB), 'site.GRASS_GRADE_RGB (site_facts research 6.3; no Sentinel-2 pair measured)'


def process_colour(rgbn, X, Y, crown, paint_out, feats, log, tag, res, grade=None):
    """Steps 1-3 of the module text on one window. rgbn: uint8 [rows, cols, 4]; crown / paint_out: bool masks;
    res: metres per pixel; grade: (R, G, B) grass multipliers (default site.GRASS_GRADE_RGB).
    Returns (rgb uint8, ndvi float32, painted bool)."""
    from scipy import ndimage
    a = rgbn.astype(np.float32)
    R, G, B, N = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    ndvi = (N - R) / (N + R + 1e-6)
    rgb = a[..., :3].copy()
    # 1. season grade on grass (soft weight: NDVI 0.05 -> 0, 0.20 -> 1; never on crowns)
    w = np.clip((ndvi - 0.05) / 0.15, 0.0, 1.0) * (~crown)
    f = np.array(grade if grade is not None else site.GRASS_GRADE_RGB, dtype=np.float32)
    rgb *= (1.0 - w[..., None]) + w[..., None] * f
    # 2a. wetland (stage-1 DTM edit 'new_wetland'): wet, dark green, keeps the photo's brightness texture
    for e in site.DTM_EDITS:
        if e['kind'] != 'lower':
            continue
        q = np.hypot((X - e['center'][0]) / e['radii'][0], (Y - e['center'][1]) / e['radii'][1])
        ww = np.where(q < 1.0, 0.5 * (1.0 + np.cos(np.pi * q)), 0.0).astype(np.float32) * 0.75
        if ww.max() > 0:
            lum = rgb.mean(axis=-1)
            target = np.array([62.0, 88.0, 50.0], dtype=np.float32)
            rel = lum / max(float(np.median(lum[ww > 0.3])) if (ww > 0.3).any() else 1.0, 1.0)
            rgb = rgb * (1 - ww[..., None]) + ww[..., None] * target * np.clip(rel, 0.6, 1.4)[..., None]
    # 2b. pipeline lane (lighter, less green in Sept 2026), 12 m wide with soft edges, grass only
    for p in feats.get('pipelines', []):
        d = _line_distance(X, Y, p['line'])
        ww = np.clip((9.0 - d) / 3.0, 0.0, 1.0) * np.clip((ndvi - 0.05) / 0.15, 0.0, 1.0) * (~crown)
        if ww.max() > 0:
            lum = rgb.mean(axis=-1, keepdims=True)
            lane = lum + (rgb - lum) * 0.75                                # 25 % less saturated
            rgb = rgb * (1 - ww[..., None]) + ww[..., None] * lane * 1.12  # 12 % brighter
    # 3. paint out crowns of trees that become 3D models: nearest open, sunlit ground pixel, then smoothed.
    #    Only where the ground around is mostly open (crown cover within 25 m below 55 %): inside a dense wood
    #    there is no open ground near enough to copy (it gave dark smears), and the 3D trees stand on the
    #    photo's own crowns there anyway.
    cover = ndimage.uniform_filter(crown.astype(np.float32), size=max(3, int(round(25.0 / res))))
    painted = paint_out & crown & (cover < 0.55)
    if painted.any():
        lum = rgb.mean(axis=-1)
        local = ndimage.uniform_filter(lum, size=max(3, int(round(15.0 / res))) | 1)
        source_ok = ~crown & (lum > 0.78 * local)                          # open ground, not a shadow
        idx = ndimage.distance_transform_edt(~source_ok, return_distances=False, return_indices=True)
        filled = rgb[idx[0], idx[1]]
        del idx
        blur = np.stack([ndimage.gaussian_filter(filled[..., k], 2.5) for k in range(3)], axis=-1)
        rgb[painted] = blur[painted]
        log(f'    {tag}: painted out {100 * painted.mean():.2f} % of the pixels (crowns of the 3D trees)')
    return np.clip(np.round(rgb), 0, 255).astype(np.uint8), ndvi.astype(np.float32), painted


def classify(rgb, ndvi, crown, painted, X, Y, feats):
    """Ground class codes (sitepack.CLASSES) for one window."""
    br = rgb.astype(np.float32).mean(axis=-1)
    R, G = rgb[..., 0].astype(np.float32), rgb[..., 1].astype(np.float32)
    cls = np.zeros(rgb.shape[:2], dtype=np.uint8)                          # 0 grass
    cls[(ndvi < 0.16) & (br > 95)] = 1                                     # dry / tan grass, hay
    cls[(ndvi < 0.08) & (br > 115) & (R > G)] = 2                          # bare soil, red clay
    code = {v: k for k, v in CLASSES.items()}
    for kind, width in (('roads', 6.0), ('taxiways', 9.0)):
        for f in feats.get(kind, []):
            if kind == 'roads' and f.get('kind') in ('footway', 'path', 'track', 'cycleway'):
                continue
            d = _line_distance(X, Y, f['line'])
            cls[(d < width / 2) & (ndvi < 0.2)] = code['gravel']
    for f in feats.get('runways', []):
        d = _line_distance(X, Y, f['line'])
        cls[(d < 10.0) & (ndvi < 0.2)] = code['asphalt']
    for f in feats.get('water', []):
        cls[_polygon_mask(X, Y, f['polygon']) & (br < 110)] = code['water']
    cls[crown & ~painted] = code['canopy']
    cls[painted] = code['under_tree']
    for e in site.DTM_EDITS:
        if e['kind'] == 'lower':
            q = np.hypot((X - e['center'][0]) / e['radii'][0], (Y - e['center'][1]) / e['radii'][1])
            cls[(q < 0.8) & ~crown] = code['wetland']
    return cls


def _save_png16(path, a):
    from PIL import Image
    Image.fromarray(np.ascontiguousarray(a.astype(np.uint16))).save(path, optimize=True)


def _save_png8(path, a):
    from PIL import Image
    Image.fromarray(np.ascontiguousarray(a.astype(np.uint8))).save(path, optimize=True)


def _save_jpg(path, rgb, quality):
    """JPEG with the usual 4:2:0 colour subsampling (colour detail finer than 0.5 m is not in NAIP anyway)."""
    from PIL import Image
    Image.fromarray(np.ascontiguousarray(rgb)).save(path, quality=quality, optimize=True, subsampling=2)


def detail_map(path_in, px, amplitude):
    """CC0 colour picture -> seamless 'brightness detail' map (uint8, 128 = unchanged): luminance divided by its
    own blur (wrap-around, so the tiling stays seamless), scaled so +-amplitude maps to +-128."""
    from PIL import Image
    from scipy import ndimage
    with Image.open(path_in) as im:
        g = np.asarray(im.convert('L').resize((px, px), Image.LANCZOS), dtype=np.float32)
    base = ndimage.gaussian_filter(g, px / 16.0, mode='wrap')
    rel = g / np.maximum(base, 1.0) - 1.0
    rel = rel / max(float(np.percentile(np.abs(rel), 99)), 1e-3)          # 99 % of pixels within +-1
    return np.clip(128.0 + 127.0 * np.clip(rel, -1.0, 1.0), 0, 255).astype(np.uint8)


def run(data_root, pkg, log=print):
    """Stage 2: prepared grids -> <pkg>/data/suas_field_real/. Returns the pack folder."""
    from scipy import ndimage
    _, raw, prep_dir = prepare.data_dirs(data_root)
    P = grids.Prepared(prep_dir)
    out = pack_dir(pkg)
    os.makedirs(os.path.join(out, 'detail'), exist_ok=True)
    home = geo.Home(site.PAD_LATLON[0], site.PAD_LATLON[1], round(P.meta['home']['alt_msl'], 1), 0.0)
    log(f'pack: {prep_dir} -> {out}; home altitude {home.alt} m NAVD88 (DTM median over the pad '
        f'{P.meta["home"]["alt_msl"]} m, rounded to 0.1 m like the SITL --home string)')
    with open(P.path('features'), encoding='utf-8') as ff:
        feats = json.load(ff)
    grade, grade_how = season_grade(P.meta)
    log(f'  season grade applied to grass (R, G, B): {grade} ({grade_how})')
    near_box, detail_box = zone_boxes(home)
    meta = {'purpose': 'Compact real-field terrain pack metadata: raster placement, tree estimates and source credits; '
                       'generated by realfield/pack.py.',
            'version': PACK_VERSION, 'world': site.WORLD, 'generated': datetime.date.today().isoformat(),
            'generator': 'sim_worlds/realfield/pack.py (stage 2) from prepare.py (stage 1)',
            'frame': 'x = metres east, y = metres north of the Runway 2 pad centre on the exact WGS84 local tangent '
                     'plane (what Gazebo uses); z = metres NAVD88 minus home altitude. ' + site.local_proj4(),
            'home': {'lat': home.lat, 'lon': home.lon, 'alt_msl': home.alt, 'heading': 0.0,
                     'dtm_median_over_pad': P.meta['home']['alt_msl']},
            'extent': list(site.EXTENT), 'near_box': [round(v, 2) for v in near_box],
            'detail_box': [round(v, 2) for v in detail_box], 'rasters': {}}

    # ---- heights 1 m (+ pad levelled) ----------------------------------------------------------------------
    dtm = P.raster('dtm')
    x0, y1, cols, rows = _grid(site.EXTENT, HEIGHT_RES, PACK_MARGIN_M)
    f = int(round(HEIGHT_RES / dtm.res))
    h = _block_mean(_crop(dtm, x0, y1, cols * f, rows * f), f)
    X, Y = _centres(x0, y1, cols, rows, HEIGHT_RES)
    s = site.PAD_SIZE_M / 2.0 + 0.5
    w = prepare._feather_box(X, Y, (-s, -s, s, s), PAD_FLAT_FEATHER_M)
    h = h * (1 - w) + w * home.alt
    v = np.round((h - HEIGHT_OFFSET) / HEIGHT_SCALE)
    _save_png16(os.path.join(out, 'height_1m.png'), np.clip(v, 0, 65535))
    meta['rasters']['height'] = {'file': 'height_1m.png', 'x0': x0, 'y1': y1, 'res': HEIGHT_RES,
                                 'offset': HEIGHT_OFFSET, 'scale': HEIGHT_SCALE,
                                 'what': 'bare-earth ground height, metres NAVD88 = offset + value * scale '
                                         '(USGS 3DEP lidar DTM 2017 + edits, 1 m means of the 0.5 m DTM)'}
    log(f'  height: {cols} x {rows} px at {HEIGHT_RES} m, {h.min():.2f} .. {h.max():.2f} m NAVD88')

    # ---- trees: which ones become 3D models (all except those inside closed forest) ---------------------------
    trees = []
    with open(P.path('trees'), encoding='utf-8') as fcsv:
        for row in csv.DictReader(fcsv):
            trees.append(row)
    model_trees = [t for t in trees if t['forest'] == '0']
    log(f'  trees: {len(trees):,} in total, {len(model_trees):,} become 3D models, '
        f'{len(trees) - len(model_trees):,} are inside closed forest (canopy sheets)')

    def paint_mask(gx0, gy1, gcols, grows, res):
        """True inside the crown disks (+0.75 m) of the 3D-model trees, on a grid."""
        m = np.zeros((grows, gcols), dtype=bool)
        for t in model_trees:
            x, y, r = float(t['x']), float(t['y']), float(t['crown_diam_m']) / 2.0 + 0.75
            c0 = int((x - r - gx0) / res)
            r0 = int((gy1 - (y + r)) / res)
            c1 = int((x + r - gx0) / res) + 2
            r1 = int((gy1 - (y - r)) / res) + 2
            c0, r0, c1, r1 = max(c0, 0), max(r0, 0), min(c1, gcols), min(r1, grows)
            if c1 <= c0 or r1 <= r0:
                continue
            xs = gx0 + (np.arange(c0, c1) + 0.5) * res
            ys = gy1 - (np.arange(r0, r1) + 0.5) * res
            m[r0:r1, c0:c1] |= ((xs[None, :] - x) ** 2 + (ys[:, None] - y) ** 2) <= r * r
        return m

    naip, crown_r = P.raster('naip'), P.raster('crown2023')
    # ---- colour near (0.25 m) + classes of the detail zone --------------------------------------------------
    nx0, ny1, ncols, nrows = _grid(near_box, NEAR_RES, 2.0)
    rgbn = _crop(naip, nx0, ny1, ncols, nrows)
    crown = _crop(crown_r, nx0, ny1, ncols, nrows).astype(bool)
    X, Y = _centres(nx0, ny1, ncols, nrows, NEAR_RES)
    paint = paint_mask(nx0, ny1, ncols, nrows, NEAR_RES)
    rgb, ndvi, painted = process_colour(rgbn, X, Y, crown, paint, feats, log, 'near', NEAR_RES, grade)
    _save_jpg(os.path.join(out, 'colour_near_0p25m.jpg'), rgb, 90)
    meta['rasters']['colour_near'] = {'file': 'colour_near_0p25m.jpg', 'x0': nx0, 'y1': ny1, 'res': NEAR_RES,
                                      'what': 'ground colour RGB (NAIP 2023-09-18 graded to Sept 2026, 3D-tree '
                                              'crowns painted out) around the search areas and the pad'}
    log(f'  colour near: {ncols} x {nrows} px at {NEAR_RES} m ({os.path.getsize(os.path.join(out, "colour_near_0p25m.jpg")) / 1e6:.1f} MB)')
    dx0, dy1, dcols, drows = _grid(detail_box, NEAR_RES, 2.0)
    c0 = int(round((dx0 - nx0) / NEAR_RES))
    r0 = int(round((ny1 - dy1) / NEAR_RES))
    sl = (slice(r0, r0 + drows), slice(c0, c0 + dcols))
    cls = classify(rgb[sl], ndvi[sl], crown[sl], painted[sl], X[sl], Y[sl], feats)
    _save_png8(os.path.join(out, 'class_detail_0p25m.png'), cls)
    meta['rasters']['class'] = {'file': 'class_detail_0p25m.png', 'x0': dx0, 'y1': dy1, 'res': NEAR_RES,
                                'classes': {str(k): v for k, v in CLASSES.items()},
                                'what': 'ground class per pixel in the search areas and at the pad'}
    counts = np.bincount(cls.ravel(), minlength=len(CLASSES))
    meta['class_share'] = {CLASSES[k]: round(float(counts[k]) / cls.size, 4) for k in CLASSES}
    log(f'  classes: ' + ', '.join(f'{CLASSES[k]} {100 * counts[k] / cls.size:.1f} %' for k in CLASSES))
    del rgbn, X, Y, paint, ndvi

    # ---- colour far (1 m) for the whole terrain: process at 0.5 m in strips, then average ----------------------
    fx0, fy1, fcols, frows = _grid(site.EXTENT, FAR_RES, PACK_MARGIN_M)
    work = 0.5
    k = int(round(work / naip.res))
    strip = 256                                                            # output rows (1 m) per strip
    far = np.zeros((frows, fcols, 3), dtype=np.uint8)
    for s0 in range(0, frows, strip):
        s1 = min(s0 + strip, frows)
        pad = 32                                                           # 32 m overlap for the filters
        a0, a1 = max(s0 - pad, 0), min(s1 + pad, frows)
        wy1 = fy1 - a0 * FAR_RES
        wr = int(round((a1 - a0) * FAR_RES / work))
        wc = int(round(fcols * FAR_RES / work))
        big = _crop(naip, fx0, wy1, wc * k, wr * k)
        rgbn = _block_mean(big, k).astype(np.uint8)
        cr = _block_mean(_crop(crown_r, fx0, wy1, wc * k, wr * k).astype(np.float32), k) > 0.5
        X, Y = _centres(fx0, wy1, wc, wr, work)
        paint = paint_mask(fx0, wy1, wc, wr, work)
        rgb_w, _, _ = process_colour(rgbn, X, Y, cr, paint, feats, lambda m: None, 'far', work, grade)
        rgb_1m = _block_mean(rgb_w, int(round(FAR_RES / work)))
        far[s0:s1] = np.clip(np.round(rgb_1m[(s0 - a0):(s0 - a0) + (s1 - s0)]), 0, 255).astype(np.uint8)
    # the near zone is sharper and more carefully painted: copy its 1 m average into the far picture
    f4 = int(round(FAR_RES / NEAR_RES))
    nb = _block_mean(rgb, f4)
    c0 = int(round((nx0 - fx0) / FAR_RES))
    r0 = int(round((fy1 - ny1) / FAR_RES))
    far[r0:r0 + nb.shape[0], c0:c0 + nb.shape[1]] = np.clip(np.round(nb), 0, 255).astype(np.uint8)
    _save_jpg(os.path.join(out, 'colour_far_1m.jpg'), far, 88)
    meta['rasters']['colour_far'] = {'file': 'colour_far_1m.jpg', 'x0': fx0, 'y1': fy1, 'res': FAR_RES,
                                     'what': 'ground colour RGB of the whole terrain at 1 m (same processing)'}
    log(f'  colour far: {fcols} x {frows} px at {FAR_RES} m ({os.path.getsize(os.path.join(out, "colour_far_1m.jpg")) / 1e6:.1f} MB)')
    del rgb, crown, far

    # ---- closed-forest canopy height (1 m): highest return within 1.5 m, smoothed, + growth to 2026 ------------
    chm, forest = P.raster('chm'), P.raster('forest')
    cx0, cy1, ccols, crows = _grid(site.EXTENT, HEIGHT_RES, PACK_MARGIN_M)
    f2 = int(round(HEIGHT_RES / chm.res))
    c = _crop(chm, cx0, cy1, ccols * f2, crows * f2).astype(np.float32)
    c = ndimage.gaussian_filter(ndimage.maximum_filter(c, size=3), 1.0)
    fo = _crop(forest, cx0, cy1, ccols * f2, crows * f2).astype(np.float32)
    canopy = _block_mean(c, f2)
    fmask = _block_mean(fo, f2) > 0.5
    from sim_worlds.realfield.trees_detect import YEARS_SINCE_LIDAR, growth_m
    canopy = np.where(fmask, np.maximum(canopy, 3.0) + growth_m(canopy, YEARS_SINCE_LIDAR), 0.0)
    _save_png8(os.path.join(out, 'canopy_1m.png'), np.clip(np.round(canopy / CANOPY_SCALE), 0, 255))
    meta['rasters']['canopy'] = {'file': 'canopy_1m.png', 'x0': cx0, 'y1': cy1, 'res': HEIGHT_RES,
                                 'offset': 0.0, 'scale': CANOPY_SCALE,
                                 'what': 'closed-forest canopy height above the ground, m (2017 lidar, highest '
                                         'return within 1.5 m, smoothed, + estimated growth to 2026); 0 = no forest'}
    log(f'  canopy: {100 * fmask.mean():.1f} % of the terrain is closed forest, median height '
        f'{float(np.median(canopy[fmask])) if fmask.any() else 0:.1f} m')

    # ---- trees.csv (rounded) + each crown's mean colour in the 2023 photo, features.json ----------------------------
    #      (the colour picks the leaf tone of the 3D tree; crowns are painted out of the ground picture above)
    keep = ['id', 'x', 'y', 'ground_navd88', 'height_m', 'crown_diam_m', 'kind', 'source', 'forest', 'height_how']
    from sim_worlds.realfield.trees_detect import disk_window
    with open(os.path.join(out, 'trees.csv'), 'w', newline='', encoding='utf-8') as fo_:
        wcsv = csv.writer(fo_)
        wcsv.writerow(keep + ['crown_rgb'])
        for t in trees:
            x, y, r = float(t['x']), float(t['y']), max(float(t['crown_diam_m']) / 2.0, 0.75)
            win, disk = disk_window(naip, x, y, r)
            cwin, _ = disk_window(crown_r, x, y, r)
            sel = disk & (cwin > 0) if disk.any() else disk
            if sel.any() and sel.sum() >= 3:
                col = win[sel][:, :3].mean(axis=0)
            elif disk.any():
                col = win[disk][:, :3].mean(axis=0)
            else:
                col = np.array([62, 87, 63])                               # mean tree colour (site_facts)
            wcsv.writerow([t[k] for k in keep] + ['%02x%02x%02x' % tuple(int(round(v)) for v in col)])
    shutil.copyfile(P.path('features'), os.path.join(out, 'features.json'))
    meta['files'] = {'trees': 'trees.csv', 'features': 'features.json'}
    meta['trees'] = dict(P.meta.get('trees', {}), as_3d_models=len(model_trees))
    meta['manual_structures'] = [{k: s[k] for k in ('name', 'kind', 'latlon', 'height_m', 'diameter_m', 'source')}
                                 for s in site.MANUAL_STRUCTURES]

    # ---- CC0 detail pictures ---------------------------------------------------------------------------------------
    man = prepare.load_manifest()
    by_file = {s['file']: s for s in man['sources']}
    meta['detail_textures'] = []
    for src_file, (name, use, size_m, px, amp) in DETAIL.items():
        p = os.path.join(raw, src_file)
        if not os.path.isfile(p):
            log(f'  detail picture missing (run fetch_real_field_data.sh --only texture): {src_file}')
            continue
        _save_png8(os.path.join(out, 'detail', f'{name}.png'), detail_map(p, px, amp))
        s = by_file.get(src_file, {})
        meta['detail_textures'].append({'name': name, 'file': f'detail/{name}.png', 'use': use, 'real_size_m': size_m,
                                        'px': px, 'amplitude': amp, 'source_url': s.get('url'),
                                        'licence': s.get('licence', 'CC0 1.0'), 'attribution': 'Poly Haven (CC0)'})
    # ---- metadata: edits, sources and licences ----------------------------------------------------------------------
    meta['dtm_edits'] = P.meta.get('dtm_edits', []) + [{
        'name': 'pad_levelled', 'max_lowered_m': None,
        'why': f'the {site.PAD_SIZE_M} m pad square (+0.5 m, {PAD_FLAT_FEATHER_M} m feather) is set to the home height '
               f'so the flat concrete pad model sits flush (the old strip there slopes by '
               f'{P.meta["home"].get("pad_relief_m")} m)'}]
    meta['season_grade'] = {'applied_rgb': list(grade), 'applied_how': grade_how,
                            'fallback_rgb': list(site.GRASS_GRADE_RGB),
                            'computed_in_stage_1': P.meta.get('season_grade')}
    meta['chm_cleaning'] = P.meta.get('chm_cleaning')
    meta['sun_direction_enu'] = list(site.SUN_DIRECTION_ENU)
    meta['sources'] = P.meta.get('sources', [])
    with open(os.path.join(out, 'pack.json'), 'w', encoding='utf-8', newline='\n') as fj:
        json.dump(meta, fj, indent=1)
        fj.write('\n')
    write_licences(out, meta)
    total = sum(os.path.getsize(os.path.join(dp, fn)) for dp, _, fns in os.walk(out) for fn in fns)
    log(f'pack: done, {total / 1e6:.1f} MB in {out}')
    return out


def write_licences(out, meta):
    """LICENSES.md of the pack: what each file is derived from and how to credit it."""
    lines = ['<!-- PURPOSE: credit the source datasets and distinguish their licenses for the generated '
             'real-field site pack. -->', '', '# Licences of the suas_field_real site pack',
             '',
             'PURPOSE: says where every file in this folder comes from and how to credit it. GENERATED by',
             'sim_worlds/realfield/pack.py; the full explanation is docs/simulation/WORLDS.md.',
             '',
             '| file | derived from | licence | credit |',
             '|---|---|---|---|',
             '| height_1m.png, canopy_1m.png, trees.csv (heights) | USGS 3DEP lidar, project '
             'OK_Woodward_FEMA_R6_Lidar_2016_D17 (flown 2017-02-10) | public domain | U.S. Geological Survey, 3D '
             'Elevation Program |',
             '| colour_*.jpg, class_detail_0p25m.png, trees.csv (positions) | USDA NAIP 2023-09-18, 0.3 m | public '
             'domain | USDA Farm Production and Conservation Business Center, NAIP |',
             '| trees.csv (new-tree heights) | Meta / WRI High Resolution Canopy Height Maps v2 | CC-BY 4.0 | Meta and '
             'World Resources Institute |',
             '| trees.csv (cedar / deciduous column only) | OKMaps / Oklahoma OGI Aerial 2025 leaf-off photo, used '
             'only INSIDE the generator as a yes/no "dark crown in March" mask; none of its pixels are in this folder, '
             'in the world or in any picture we publish (it has no written redistribution licence) | state data, no '
             'fee, internal use only | Oklahoma Office of Geographic Information / OKMaps |',
             '| colour_*.jpg (grass colour) | Copernicus Sentinel-2 L2A (season grade numbers only) | free and open | '
             'Contains modified Copernicus Sentinel data 2023, 2026 |',
             '| features.json (OpenStreetMap part: roads, runway, streams, water, pipeline, OSM buildings) | '
             'OpenStreetMap (fixed date 2026-10-10) | **ODbL 1.0**: keep this credit, and share changes to the OSM '
             'data under ODbL | (c) OpenStreetMap contributors, https://www.openstreetmap.org/copyright |',
             '| features.json (buildings with "source": "microsoft") | Microsoft Global ML Building Footprints '
             '(release 2026-02-03) | **CDLA Permissive 2.0** (https://cdla.dev/permissive-2-0/) | Microsoft |',
             '| features.json (building heights "height_lidar") | USGS 3DEP lidar 2017 | public domain | U.S. '
             'Geological Survey |',
             '| detail/*.png | Poly Haven textures (see pack.json detail_textures) | CC0 1.0 | Poly Haven '
             '(polyhaven.com), no credit required |',
             '',
             'Hand-added structures (the three wooden poles in Search Boundary 1) are listed with their sources in',
             'sim_worlds/realfield/site.py (MANUAL_STRUCTURES).',
             '',
             'Google Earth / Google Maps imagery is NOT used anywhere (Google\'s terms do not allow it as a texture).',
             '']
    with open(os.path.join(out, 'LICENSES.md'), 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines))
