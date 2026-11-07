"""
sim_worlds/realfield/previews.py -- PURPOSE: pictures you can LOOK AT to judge the real-terrain world without starting
Gazebo ("build_real_field.py preview --out DIR"). Everything is drawn from the site pack and the generated truth file:

  overview.jpg        the whole terrain from above (1 pixel = 2 m): ground colour, the flight boundary, both search
                      boundaries, the pad, every real tree as a dot, the four targets
  hillshade.jpg       the ground height as shaded relief (lighting from the north-west), contour lines every 2 m
  search_boundary_1.jpg / search_boundary_2.jpg
                      each search boundary from above at about 6 cm per pixel (the 0.25 m photo magnified), with the
                      real trees (circles), the hiker (red), the tent (yellow), the other props (white dots) and the
                      survey lanes (cyan)
  profile_search_boundary_1.png / _2.png
                      ground height along the middle of the boundary and the flight height of the mission above it
                      (the SUAS rule band: 150 ft above the highest ground, 400 ft above the lowest, is shaded)

Needs numpy, Pillow and PyYAML.
"""

import os

import numpy as np
import yaml
from PIL import Image, ImageDraw

from sim_worlds import geo, mission
from sim_worlds.realfield import site
from sim_worlds.realfield.sitepack import SitePack
from sim_worlds.realfield.world_build import search_rect

RED, YELLOW, CYAN, WHITE = (230, 40, 40), (250, 220, 30), (0, 220, 255), (255, 255, 255)


def _px(raster, x, y):
    """World (x, y) -> float pixel (col, row) of a raster picture."""
    return (x - raster.x0) / raster.res, (raster.y1 - y) / raster.res


def _crop_scaled(raster, box, scale):
    """The colour picture inside world box (x0, y0, x1, y1), magnified by `scale`. Returns (image, world->pixel)."""
    x0, y0, x1, y1 = box
    c0, r0 = _px(raster, x0, y1)
    c1, r1 = _px(raster, x1, y0)
    c0, r0, c1, r1 = int(max(c0, 0)), int(max(r0, 0)), int(min(c1, raster.cols)), int(min(r1, raster.rows))
    im = Image.fromarray(raster.data[r0:r1, c0:c1].astype(np.uint8)).convert('RGB')
    im = im.resize((int(im.width * scale), int(im.height * scale)), Image.BICUBIC)
    ox, oy = raster.x0 + c0 * raster.res, raster.y1 - r0 * raster.res        # world position of the crop's top-left

    def to_px(x, y):
        return (x - ox) / raster.res * scale, (oy - y) / raster.res * scale
    return im, to_px


def _draw_marks(im, to_px, tree_list, truth, lanes=None, rect_pts=None, tree_scale=1.0):
    d = ImageDraw.Draw(im)
    for t in tree_list:
        px, py = to_px(t['x'], t['y'])
        r = max(t['crown_diam_m'] / 2.0 * tree_scale, 1.0)
        if -50 < px < im.width + 50 and -50 < py < im.height + 50:
            d.ellipse([px - r, py - r, px + r, py + r], outline=(20, 90, 20) if t['kind'] != 'cedar' else (10, 60, 90))
    if lanes:
        for a, b in lanes:
            d.line([to_px(*a), to_px(*b)], fill=CYAN, width=2)
    if rect_pts:
        pts = [to_px(*p) for p in rect_pts]
        d.line(pts + [pts[0]], fill=WHITE, width=2)
    for e in truth['distractors']:
        px, py = to_px(e['x'], e['y'])
        d.ellipse([px - 3, py - 3, px + 3, py + 3], fill=WHITE)
    for e in truth['targets']:
        px, py = to_px(e['x'], e['y'])
        col = RED if e['class'] == 'person' else YELLOW
        d.ellipse([px - 14, py - 14, px + 14, py + 14], outline=col, width=3)
        d.text((px + 18, py - 6), e['id'], fill=col)


def run(pkg, out, models=None, pack_dir=None, log=print):
    """Write the preview pictures into folder `out`."""
    os.makedirs(out, exist_ok=True)
    pack = SitePack(pack_dir)
    name = site.WORLD
    truth = yaml.safe_load(open(os.path.join(pkg, 'truth', f'{name}.yaml'), encoding='utf-8'))
    trees = pack.trees()
    # overview ----------------------------------------------------------------------------------------------------
    far = pack.raster('colour_far')
    im, to_px = _crop_scaled(far, site.EXTENT, 0.5)
    d = ImageDraw.Draw(im)
    poly = [to_px(*p) for p in site.flight_boundary_xy(pack.home)]
    d.line(poly + [poly[0]], fill=(255, 140, 0), width=3)
    for sb in site.SEARCH_BOUNDARIES_LATLON:
        x0, y0, x1, y1 = site.search_box_xy(sb, pack.home)
        d.rectangle([to_px(x0, y1), to_px(x1, y0)], outline=WHITE, width=2)
    for t in trees:
        px, py = to_px(t['x'], t['y'])
        d.point((px, py), fill=(15, 70, 15))
    px, py = to_px(0, 0)
    d.ellipse([px - 6, py - 6, px + 6, py + 6], outline=(255, 0, 255), width=3)
    for e in truth['targets']:
        px, py = to_px(e['x'], e['y'])
        d.ellipse([px - 6, py - 6, px + 6, py + 6], outline=RED if e['class'] == 'person' else YELLOW, width=3)
    im.convert('RGB').save(os.path.join(out, 'overview.jpg'), quality=90)
    log(f'  overview.jpg ({im.width} x {im.height})')
    # hillshade -----------------------------------------------------------------------------------------------------
    h = pack.raster('height')
    z = h.data.astype(np.float32) * 0.01 + 150.0 if h.data.dtype != np.float32 else h.data
    step = 2
    zs = z[::step, ::step]
    gy, gx = np.gradient(zs, step * h.res)
    az, el = np.radians(315.0), np.radians(45.0)
    shade = np.clip(np.sin(el) + np.cos(el) * 2.5 * (gx * np.sin(az) * -1 + gy * np.cos(az) * -1) / 1.0, 0.0, 1.0)
    img = (255 * (0.25 + 0.75 * shade)).astype(np.uint8)
    lev = np.floor(zs / 2.0)
    edge = (lev != np.roll(lev, 1, 0)) | (lev != np.roll(lev, 1, 1))
    rgb = np.stack([img] * 3, -1)
    rgb[edge] = (rgb[edge] * 0.55).astype(np.uint8)
    Image.fromarray(rgb).save(os.path.join(out, 'hillshade.jpg'), quality=90)
    log('  hillshade.jpg')
    # search boundaries from above + profile ----------------------------------------------------------------------
    near = pack.raster('colour_near')
    for idx, sb in enumerate(site.SEARCH_BOUNDARIES_LATLON, start=1):
        box = site.search_box_xy(sb, pack.home)
        pad = 8.0
        big, tp = _crop_scaled(near, (box[0] - pad, box[1] - pad, box[2] + pad, box[3] + pad), 4.0)
        rect = search_rect(box)
        suffix = '' if idx == 1 else '_sb2'
        items = mission.read_waypoints(os.path.join(pkg, 'missions', f'{name}{suffix}_survey.waypoints'))
        wp = [geo.latlon_to_enu(i.x, i.y, pack.home) for i in items if i.command == 16 and i.frame == 3 and i.x != 0]
        pts_l = wp[:-1]                                      # the last waypoint is the hold point, not a lane end
        lanes = [(pts_l[i], pts_l[i + 1]) for i in range(0, len(pts_l) - 1, 2)] if len(pts_l) > 1 else None
        _draw_marks(big, tp, [t for t in trees if box[0] - 30 < t['x'] < box[2] + 30 and box[1] - 30 < t['y'] < box[3] + 30],
                    truth, lanes=lanes, rect_pts=rect.corners(), tree_scale=4.0 / near.res)
        big.convert('RGB').save(os.path.join(out, f'search_boundary_{idx}.jpg'), quality=90)
        log(f'  search_boundary_{idx}.jpg ({big.width} x {big.height})')
        # profile along the lane direction through the middle
        n = 300
        us = np.linspace(-rect.length / 2, rect.length / 2, n)
        pts = [rect.to_world(u, 0.0) for u in us]
        zz = pack.ground_z(np.array([p[0] for p in pts]), np.array([p[1] for p in pts]))
        alt = max(i.z for i in items if i.frame == 3 and i.command in (16, 22))
        _profile(os.path.join(out, f'profile_search_boundary_{idx}.png'), us, zz, alt, rect)
        log(f'  profile_search_boundary_{idx}.png')
    return out


def _profile(path, us, zz, alt, rect, w=1200, h=420):
    """Ground height along the boundary's middle line and the flight height above home (a simple PIL line chart)."""
    im = Image.new('RGB', (w, h), (250, 250, 250))
    d = ImageDraw.Draw(im)
    lo, hi = min(float(zz.min()), 0.0) - 2.0, alt + 12.0
    sx = lambda u: 50 + (u - us[0]) / (us[-1] - us[0]) * (w - 70)          # noqa: E731
    sy = lambda v: h - 30 - (v - lo) / (hi - lo) * (h - 60)                  # noqa: E731
    floor = float(zz.max()) + site.RULE_MIN_AGL_M
    ceil = float(zz.min()) + site.RULE_MAX_AGL_M
    d.rectangle([sx(us[0]), sy(min(ceil, hi)), sx(us[-1]), sy(floor)], fill=(215, 240, 215))
    d.line([(sx(u), sy(v)) for u, v in zip(us, zz)], fill=(120, 80, 30), width=3)
    d.line([(sx(us[0]), sy(alt)), (sx(us[-1]), sy(alt))], fill=(200, 30, 30), width=2)
    d.text((55, 6), f'ground (brown) and flight height {alt:.1f} m above the pad (red); green band = legal heights '
                    f'({floor:.1f} .. {ceil:.1f} m)', fill=(0, 0, 0))
    d.text((55, h - 18), f'distance along the boundary, 0 at its centre, {rect.length:.0f} m long', fill=(0, 0, 0))
    im.save(path)
