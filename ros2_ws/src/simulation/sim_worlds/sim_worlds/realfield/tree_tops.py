"""
sim_worlds/realfield/tree_tops.py -- PURPOSE: give every 3D tree the REAL look of its crown from above. The ground picture
of the world has the crowns of the 3D trees painted out (the model draws them instead), so without this step a tree seen
from a drone is a generic green blob that does not look like the aerial photo. This maintainer tool cuts a small square
picture of every tree's crown out of the UNPAINTED aerial photo (NAIP 2023, public domain) and packs all of them into one
picture, `tree_tops_32.jpg`, in the site pack. Stage 3 (trees.py) then lays each tree's top-down picture over its crown
mesh, so from above the crown shows the photo's real colours, leaf clumps and gaps.

It also writes the mean crown colour of every tree into trees.csv (column crown_rgb, used to choose leaf tones when no
picture is available).

Layout: cell i = row i of trees.csv (all trees, forest ones too), CELL_PX x CELL_PX pixels, COLS cells per picture row.
A cell shows a square of SPAN x the crown diameter centred on the tree (north up), so the crown disc fills 1/SPAN of it.

Needs numpy + Pillow and the prepared photo (prepared/naip_0p25m.npy from stage 1 `prepare`). Run (maintainers):
  python3 scripts/build_real_field.py treetops --data <world_data folder>
"""

import csv
import json
import math
import os

import numpy as np

CELL_PX = 32
COLS = 128
SPAN = 1.3
FILE = 'tree_tops_32.jpg'


def _naip(prepared_dir):
    """(array [rows, cols, 4] uint8, x0, y1, res) of the prepared NAIP photo."""
    with open(os.path.join(prepared_dir, 'grid.json'), encoding='utf-8') as f:
        g = json.load(f)['rasters']['naip']
    a = np.load(os.path.join(prepared_dir, g['file']), mmap_mode='r')
    return a, float(g['x0']), float(g['y1']), float(g['res'])


def make(pack_dir, prepared_dir, log=print):
    """Cut the crown pictures out of the unpainted photo, write tree_tops_32.jpg, add crown_rgb to trees.csv and record
    the picture in pack.json. Returns the number of trees."""
    from PIL import Image
    naip, x0, y1, res = _naip(prepared_dir)
    with open(os.path.join(pack_dir, 'pack.json'), encoding='utf-8') as f:
        meta = json.load(f)
    csv_path = os.path.join(pack_dir, meta['files']['trees'])
    with open(csv_path, encoding='utf-8', newline='') as f:
        rows = list(csv.DictReader(f))
    n = len(rows)
    nrows = int(math.ceil(n / COLS))
    atlas = np.zeros((nrows * CELL_PX, COLS * CELL_PX, 3), dtype=np.uint8)
    H, W = naip.shape[:2]
    for i, t in enumerate(rows):
        x, y, d = float(t['x']), float(t['y']), float(t['crown_diam_m'])
        half = SPAN * max(d, 1.0) / 2.0 / res                                  # half window in photo pixels
        cc, rr = (x - x0) / res, (y1 - y) / res                                # photo pixel of the tree centre
        c0, r0 = int(round(cc - half)), int(round(rr - half))
        size = max(int(round(2 * half)), 4)
        win = np.zeros((size, size, 3), dtype=np.uint8)
        sc0, sr0 = max(c0, 0), max(r0, 0)
        sc1, sr1 = min(c0 + size, W), min(r0 + size, H)
        if sc1 > sc0 and sr1 > sr0:
            piece = np.asarray(naip[sr0:sr1, sc0:sc1, :3])
            win[sr0 - r0:sr1 - r0, sc0 - c0:sc1 - c0] = piece
            win[:sr0 - r0] = win[sr0 - r0:sr0 - r0 + 1]                        # repeat the edge outside the photo
        im = Image.fromarray(win).resize((CELL_PX, CELL_PX), Image.BILINEAR if size < 2 * CELL_PX else Image.LANCZOS)
        cell = np.asarray(im)
        atlas[(i // COLS) * CELL_PX:(i // COLS + 1) * CELL_PX, (i % COLS) * CELL_PX:(i % COLS + 1) * CELL_PX] = cell
        yy, xx = np.mgrid[0:CELL_PX, 0:CELL_PX]
        disc = (xx - CELL_PX / 2 + 0.5) ** 2 + (yy - CELL_PX / 2 + 0.5) ** 2 <= (CELL_PX / SPAN / 2) ** 2
        col = cell[disc].mean(axis=0)
        t['crown_rgb'] = '%02x%02x%02x' % tuple(int(round(v)) for v in col)
    Image.fromarray(atlas).save(os.path.join(pack_dir, FILE), quality=88)
    fields = [k for k in rows[0].keys()]
    with open(csv_path, 'w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, lineterminator='\n')
        w.writeheader()
        w.writerows(rows)
    meta['files']['tree_tops'] = FILE
    meta['tree_tops'] = {'file': FILE, 'cell_px': CELL_PX, 'cols': COLS, 'span_per_crown_diameter': SPAN, 'trees': n,
                         'source': 'USDA NAIP 2023-09-18 (public domain), unpainted, north up',
                         'what': 'cell i (row-major, COLS per row) = top-down picture of tree i of trees.csv'}
    with open(os.path.join(pack_dir, 'pack.json'), 'w', encoding='utf-8', newline='\n') as f:
        json.dump(meta, f, indent=1)
        f.write('\n')
    log(f'[treetops] {n:,} crown pictures -> {FILE} ({atlas.shape[1]} x {atlas.shape[0]} px, '
        f'{os.path.getsize(os.path.join(pack_dir, FILE)) / 1e6:.1f} MB); crown_rgb added to trees.csv')
    return n
