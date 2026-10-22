"""
sim_demo/mission_map.py -- PURPOSE: draw a top-down picture of one mission for the score report
(mission_map.png): the search areas (yellow), every photo's ground footprint (blue), where
the drone flew (cyan line), where the detector said it saw targets (dots), what
main_controller chose (big rings), where the payloads landed (diamonds) and the REAL targets
(red = hiker, orange = tent) with their names. North is up, a 10 m scale bar is drawn.

Pure Python + Pillow (python3-pil). Input: world-ENU metres (x = east, y = north).
"""
from __future__ import annotations

import math

COLORS = {
    'bg': (22, 27, 34), 'grid': (40, 48, 58), 'area': (240, 200, 40), 'footprint': (70, 150, 230),
    'track': (60, 220, 240), 'person': (255, 77, 77), 'tent': (255, 163, 26), 'text': (230, 237, 243),
    'muted': (139, 152, 165), 'drop': (230, 80, 230),
}


def _bounds(point_lists, margin: float = 15.0):
    xs = [p[0] for pts in point_lists for p in pts]
    ys = [p[1] for pts in point_lists for p in pts]
    if not xs:
        return -50.0, 50.0, -50.0, 50.0
    x0, x1, y0, y1 = min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin
    # Keep the picture from being extremely thin.
    if x1 - x0 < 60:
        c = (x0 + x1) / 2
        x0, x1 = c - 30, c + 30
    if y1 - y0 < 60:
        c = (y0 + y1) / 2
        y0, y1 = c - 30, c + 30
    return x0, x1, y0, y1


def draw_mission_map(path: str, targets, areas, footprints, track, detections, choices, drops,
                     title: str = '', size_px: int = 1100) -> str:
    """Draw and save the map.

    Args:
        path: PNG file to write.
        targets: list of (name, cls, east, north).
        areas: list of (name, [(east, north), ...]).
        footprints: list of [(east, north) x4].
        track: list of (east, north).
        detections: list of (cls, east, north, confidence).
        choices: list of (cls, east, north) - what main_controller flew to.
        drops: list of (cls, east, north, source).
        title: text at the top.
    Returns:
        the path written.
    """
    from PIL import Image, ImageDraw, ImageFont

    pts = [[(t[2], t[3]) for t in targets], [p for _, a in areas for p in a],
           [p for fp in footprints for p in fp], list(track), [(d[1], d[2]) for d in detections]]
    x0, x1, y0, y1 = _bounds(pts)
    scale = (size_px - 40) / max(x1 - x0, y1 - y0)
    w = int((x1 - x0) * scale) + 40
    h = int((y1 - y0) * scale) + 90
    img = Image.new('RGB', (w, h), COLORS['bg'])
    d = ImageDraw.Draw(img, 'RGBA')
    try:
        font = ImageFont.truetype('DejaVuSans.ttf', 15)
        font_b = ImageFont.truetype('DejaVuSans-Bold.ttf', 17)
    except OSError:
        font = font_b = ImageFont.load_default()

    def px(e, n):
        return 20 + (e - x0) * scale, 70 + (y1 - n) * scale

    # 10 m grid.
    for gx in range(int(math.floor(x0 / 10)) * 10, int(x1) + 10, 10):
        d.line([px(gx, y0), px(gx, y1)], fill=COLORS['grid'], width=1)
    for gy in range(int(math.floor(y0 / 10)) * 10, int(y1) + 10, 10):
        d.line([px(x0, gy), px(x1, gy)], fill=COLORS['grid'], width=1)
    for name, poly in areas:
        d.polygon([px(*p) for p in poly], outline=COLORS['area'] + (255,), fill=COLORS['area'] + (18,))
        top = max(poly, key=lambda p: p[1])
        d.text(px(top[0], top[1] + 2), f'search area: {name}', fill=COLORS['area'], font=font)
    for fp in footprints:
        d.polygon([px(*p) for p in fp], outline=COLORS['footprint'] + (140,),
                  fill=COLORS['footprint'] + (28,))
    if len(track) >= 2:
        d.line([px(*p) for p in track], fill=COLORS['track'] + (230,), width=2)
    for cls, e, n, conf in detections:
        x, y = px(e, n)
        col = COLORS.get(cls, COLORS['text'])
        d.ellipse([x - 4, y - 4, x + 4, y + 4], fill=col + (200,))
    for cls, e, n in choices:
        x, y = px(e, n)
        col = COLORS.get(cls, COLORS['text'])
        d.ellipse([x - 12, y - 12, x + 12, y + 12], outline=col + (255,), width=3)
        d.text((x + 14, y - 22), f'{cls} chosen', fill=col, font=font)
    for name, cls, e, n in targets:
        x, y = px(e, n)
        col = COLORS.get(cls, COLORS['text'])
        d.rectangle([x - 7, y - 7, x + 7, y + 7], fill=col + (255,), outline=(0, 0, 0, 255))
        d.text((x + 10, y + 4), f'REAL {"hiker" if cls == "person" else cls} ({name})', fill=col, font=font_b)
    for cls, e, n, source in drops:
        x, y = px(e, n)
        d.polygon([(x, y - 9), (x + 9, y), (x, y + 9), (x - 9, y)], fill=COLORS['drop'] + (255,),
                  outline=(0, 0, 0, 255))
        # Below the point: the "chosen" label sits above it (they are often at the same spot).
        d.text((x + 12, y + 6), f'{cls} payload {source}', fill=COLORS['drop'], font=font)
    # Title, legend, scale bar, north arrow.
    d.text((20, 12), title or 'Mission map', fill=COLORS['text'], font=font_b)
    legend = [('track', 'drone track'), ('footprint', 'photo footprints'), ('area', 'search area'),
              ('drop', 'payload landed / released')]
    lx = 20
    for key, label in legend:
        d.rectangle([lx, 42, lx + 14, 56], fill=COLORS[key] + (255,))
        d.text((lx + 20, 41), label, fill=COLORS['muted'], font=font)
        lx += 40 + int(d.textlength(label, font=font))
    sx, sy = 20, h - 18
    d.line([(sx, sy), (sx + 10 * scale, sy)], fill=COLORS['text'], width=3)
    d.text((sx + 10 * scale + 8, sy - 9), '10 m', fill=COLORS['text'], font=font)
    nx, ny = w - 30, 90
    d.polygon([(nx, ny - 18), (nx - 8, ny + 4), (nx + 8, ny + 4)], fill=COLORS['text'])
    d.text((nx - 5, ny + 6), 'N', fill=COLORS['text'], font=font_b)
    img.save(path)
    return path
