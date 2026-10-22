"""
sim_worlds/realfield/texture.py -- PURPOSE: paint the picture of every terrain tile (and of the forest canopy
sheets) from the site pack: the real colour of the place (NAIP 2023, season graded, 3D-tree crowns painted out)
plus, in the search areas, centimetre ground detail from CC0 pictures, so the ground still looks like ground in
a 3-4 cm per pixel drone photo (the NAIP photo alone has 30 cm pixels: a 10x blur at that height).
numpy + Pillow only.

How the detail works (recipe of the imagery research, "camsim"): each ground class (grass, dry grass, bare soil,
gravel, asphalt, canopy, under-tree, wetland) has one or two seamless CC0 pictures turned into pure brightness
detail (1.0 = no change). They are laid over the world at their REAL size (e.g. the aerial grass picture covers
15 x 15 m), the second layer rotated so the repeats never line up, pre-shrunk to the output pixel size (no
shimmer), blended by the class map, and multiplied onto the colour. The colour itself is never changed by the
detail on average, so the field still looks like the photo from high up.

Optional imagery hook: colour_override(t, px) -> RGB array lets a different, licence-compatible picture (for
example the team's own drone orthomosaic) replace the NAIP colour of a tile; see WORLDS.md. Google Earth imagery
must NOT be used here (Google's terms).
"""

import math

import numpy as np

from sim_worlds.realfield.sitepack import CLASSES

#: detail layers per ground class: (detail picture, exponent, rotation in degrees)
RECIPE = {
    'grass': [('grass_aerial', 1.0, 0.0), ('grass_close', 0.6, 37.0)],
    'dry_grass': [('dry_ground', 1.0, 0.0), ('grass_aerial', 0.5, 23.0)],
    'bare_soil': [('dirt_aerial', 1.0, 0.0)],
    'gravel': [('gravel', 1.0, 0.0)],
    'asphalt': [('asphalt', 1.0, 0.0)],
    'water': [],
    'canopy': [('grass_close', 0.9, 11.0), ('grass_aerial', 0.3, 0.0)],
    'under_tree': [('dirt_aerial', 0.7, 0.0), ('dry_ground', 0.7, 53.0)],
    'wetland': [('grass_close', 1.0, 71.0)],
}


def _pil_image(pack, name):
    """Cached PIL image of a pack colour raster ('colour_near' / 'colour_far')."""
    from PIL import Image
    key = 'pil:' + name
    if key not in pack._cache:
        pack._cache[key] = Image.fromarray(np.asarray(pack.raster(name).data))
    return pack._cache[key]


def _box(r, x0, y0, x1, y1, px):
    """Pillow resize box (source pixel coordinates) that puts output pixel centres i = 0..px-1 at world
    x0 + i * (x1 - x0) / (px - 1) (first and last pixel exactly on the edges), rows from y1 down to y0."""
    sx = (x1 - x0) / ((px - 1) * r.res)
    sy = (y1 - y0) / ((px - 1) * r.res)
    left = (x0 - r.x0) / r.res - 0.5 * sx
    top = (r.y1 - y1) / r.res - 0.5 * sy
    return (left, top, left + sx * px, top + sy * px), max(sx, sy)


def _inside(box, outer, tol=1e-6):
    return box[0] >= outer[0] - tol and box[1] >= outer[1] - tol and box[2] <= outer[2] + tol and box[3] <= outer[3] + tol


def sample_colour(pack, x0, y0, x1, y1, px):
    """RGB uint8 [px, px, 3] of the box (x0, y0, x1, y1), first/last pixels on the edges. Uses the 0.25 m near
    picture when the box is inside it, else the 1 m picture of the whole terrain."""
    from PIL import Image
    name = 'colour_near' if _inside((x0, y0, x1, y1), pack.meta['near_box']) else 'colour_far'
    r = pack.raster(name)
    box, scale = _box(r, x0, y0, x1, y1, px)
    im = _pil_image(pack, name).resize((px, px), Image.BICUBIC if scale <= 1.0 else Image.LANCZOS, box=box)
    return np.asarray(im, dtype=np.uint8)


def _class_weights(pack, x0, y0, x1, y1, px):
    """{class name: weight array [px, px]} for the classes present in the box (smooth 25 cm transitions)."""
    from PIL import Image
    r = pack.raster('class')
    box, _ = _box(r, x0, y0, x1, y1, px)
    c0, r0 = int(math.floor(box[0])) - 2, int(math.floor(box[1])) - 2
    c1, r1 = int(math.ceil(box[2])) + 2, int(math.ceil(box[3])) + 2
    c0c, r0c, c1c, r1c = max(c0, 0), max(r0, 0), min(c1, r.cols), min(r1, r.rows)
    if c1c <= c0c or r1c <= r0c:
        return {'grass': np.ones((px, px), dtype=np.float32)}
    sub = np.asarray(r.data[r0c:r1c, c0c:c1c])
    shifted = (box[0] - c0c, box[1] - r0c, box[2] - c0c, box[3] - r0c)
    out = {}
    for code in np.unique(sub):
        name = CLASSES.get(int(code), 'grass')
        m = Image.fromarray(((sub == code) * 255).astype(np.uint8))
        w = np.asarray(m.resize((px, px), Image.BILINEAR, box=shifted), dtype=np.float32) / 255.0
        out[name] = out.get(name, 0.0) + w
    return out


def _detail_lookup(pack, name, X, Y, rot_deg, out_res):
    """Detail picture `name` laid over the world at its real size, rotated by rot_deg, pre-shrunk so one texel
    is about one output pixel, sampled (bilinear, wrapping) at world points X, Y. 1.0 = no change."""
    from PIL import Image
    e = next(d for d in pack.detail_textures() if d['name'] == name)
    size = float(e['real_size_m'])
    key = f'detail_px:{name}:{out_res:.4f}'
    if key not in pack._cache:
        d = pack.detail_image(name)
        p = int(np.clip(round(size / out_res), 8, d.shape[0]))
        if p < d.shape[0]:
            d = np.asarray(Image.fromarray(np.ascontiguousarray(d, dtype=np.float32)).resize((p, p), Image.BOX),
                           dtype=np.float32)
        pack._cache[key] = d
    d = pack._cache[key]
    p = d.shape[0]
    a = math.radians(rot_deg)
    u = (X * math.cos(a) - Y * math.sin(a)) / size * p
    v = (X * math.sin(a) + Y * math.cos(a)) / size * p
    i0, j0 = np.floor(u).astype(np.int64), np.floor(v).astype(np.int64)
    fu, fv = (u - i0).astype(np.float32), (v - j0).astype(np.float32)
    i0 %= p
    j0 %= p
    i1, j1 = (i0 + 1) % p, (j0 + 1) % p
    return ((d[j0, i0] * (1 - fu) + d[j0, i1] * fu) * (1 - fv) + (d[j1, i0] * (1 - fu) + d[j1, i1] * fu) * fv)


def micro_modulation(pack, x0, y0, x1, y1, px):
    """Brightness multiplier [px, px] (about 1.0 on average) for the box, from the class map and RECIPE."""
    out_res = (x1 - x0) / (px - 1)
    xs = x0 + np.arange(px) * out_res
    ys = y1 - np.arange(px) * (y1 - y0) / (px - 1)
    X, Y = np.meshgrid(xs, ys)
    weights = _class_weights(pack, x0, y0, x1, y1, px)
    total = np.zeros((px, px), dtype=np.float32)
    wsum = np.zeros((px, px), dtype=np.float32)
    for name, w in weights.items():
        m = np.ones((px, px), dtype=np.float32)
        for det, expo, rot in RECIPE.get(name, []):
            if any(d['name'] == det for d in pack.detail_textures()):
                m *= np.power(np.maximum(_detail_lookup(pack, det, X, Y, rot, out_res), 0.05), expo)
        total += w * m
        wsum += w
    return np.where(wsum > 1e-3, total / np.maximum(wsum, 1e-3), 1.0).astype(np.float32)


def tile_picture(pack, t, px, micro, colour_override=None):
    """The picture of terrain tile t (dict with x0, y0, x1, y1): RGB uint8 [px, px, 3]."""
    rgb = None
    if colour_override is not None:
        rgb = colour_override(t, px)
    if rgb is None:
        rgb = sample_colour(pack, t['x0'], t['y0'], t['x1'], t['y1'], px)
    if not micro:
        return rgb
    m = micro_modulation(pack, t['x0'], t['y0'], t['x1'], t['y1'], px)
    return np.clip(np.round(rgb.astype(np.float32) * m[..., None]), 0, 255).astype(np.uint8)
