"""
sim_worlds/realfield/upscale.py -- PURPOSE: the OPTIONAL "sharper ground" layer of the real-terrain world. The aerial photo
has 25-30 cm pixels; the drone camera sees about 2-3 cm per pixel at 47 m, so the plain photo looks like smeared paint.
This module makes the photo sharper in the search areas and around the pad with an AI upscaler (Real-ESRGAN x4plus, BSD-3
licence, runs on the GPU in about a minute) and lets stage 3 (assets_build.py) paint the ground tiles from the result.

What the AI does and does not do: it invents plausible FINE texture (grass blades, cracks in the slab, gravel grains) from
the blurry photo; it does not add or move objects. Positions, colours of larger patches and the real shapes stay as in
the photo. Because invented texture could confuse a detector (false alarms), every use of it must be checked by a flight
that compares the detector's false-alarm count with and without it (docs/simulation/WORLDS.md).

Files (never in git; the upscaled picture is a derived, rebuildable cache):
  <data>/upscaled/colour_detail_x4.jpg   the upscaled colour of the "detail box" (4 x more pixels per side)
  <data>/upscaled/colour_detail_x4.json  where it sits in the world and which model made it
where <data> = $ASTRA_WORLD_DATA or ~/.astra_sim/world_data/suas_field_real. Needs numpy + Pillow to USE the picture, and
torch + realesrgan + basicsr + the weights file (~/.astra_sim/models/RealESRGAN_x4plus.pth) to MAKE it (a separate
virtual environment, see simulation/setup/upscale_env.sh, keeps ROS's Python untouched).
"""

import json
import os
import time

import numpy as np

SCALE = 4
MARGIN_M = 100.0                      #: the picture covers the detail box plus this many metres on every side
FILE = 'colour_detail_x4.jpg'
META = 'colour_detail_x4.json'
WEIGHTS = os.path.join(os.path.expanduser('~'), '.astra_sim', 'models', 'RealESRGAN_x4plus.pth')


def default_dir():
    """Folder of the upscaled picture: $ASTRA_WORLD_DATA/suas_field_real/upscaled or ~/.astra_sim/world_data/..."""
    base = os.environ.get('ASTRA_WORLD_DATA') or os.path.join(os.path.expanduser('~'), '.astra_sim', 'world_data')
    return os.path.join(base, 'suas_field_real', 'upscaled')


def available(folder=None):
    """True when an upscaled picture exists in `folder`."""
    folder = folder or default_dir()
    return os.path.isfile(os.path.join(folder, FILE)) and os.path.isfile(os.path.join(folder, META))


def signature(folder=None):
    """Short text that changes when the upscaled picture changes (goes into the assets fingerprint); '' if none."""
    folder = folder or default_dir()
    if not available(folder):
        return ''
    p = os.path.join(folder, FILE)
    return f'up4:{os.path.getsize(p)}:{int(os.path.getmtime(p))}'


def make(pack, folder=None, log=print, tile=256):
    """Upscale the pack's near-colour picture over its detail box x4 with Real-ESRGAN and save it into `folder`."""
    import torch                                                   # noqa: WPS433 (only the maker needs them)
    from basicsr.archs.rrdbnet_arch import RRDBNet
    from PIL import Image
    from realesrgan import RealESRGANer
    folder = folder or default_dir()
    if not os.path.isfile(WEIGHTS):
        raise SystemExit(f'[upscale] ERROR: weights not found: {WEIGHTS}. Fix: bash simulation/setup/upscale_env.sh')
    near = pack.raster('colour_near')
    nb = pack.meta['near_box']
    x0, y0, x1, y1 = pack.meta['detail_box']
    x0, y0, x1, y1 = (max(x0 - MARGIN_M, nb[0]), max(y0 - MARGIN_M, nb[1]), min(x1 + MARGIN_M, nb[2]),
                      min(y1 + MARGIN_M, nb[3]))                  # the detail box + 100 m: the tiles around it too
    c0, r0 = int((x0 - near.x0) / near.res), int((near.y1 - y1) / near.res)
    c1, r1 = int(np.ceil((x1 - near.x0) / near.res)), int(np.ceil((near.y1 - y0) / near.res))
    src = np.asarray(near.data[r0:r1, c0:c1], dtype=np.uint8)
    log(f'[upscale] {src.shape[1]} x {src.shape[0]} px -> {src.shape[1] * SCALE} x {src.shape[0] * SCALE} px '
        f'(torch {torch.__version__}, GPU {torch.cuda.is_available()})')
    model = RRDBNet(num_in_ch=3, num_out_ch=3, num_feat=64, num_block=23, num_grow_ch=32, scale=SCALE)
    up = RealESRGANer(scale=SCALE, model_path=WEIGHTS, model=model, tile=tile, tile_pad=16, pre_pad=0,
                      half=bool(torch.cuda.is_available()))
    t0 = time.time()
    out, _ = up.enhance(np.ascontiguousarray(src[..., ::-1]), outscale=SCALE)       # the model works in BGR
    out = np.ascontiguousarray(out[..., ::-1])
    os.makedirs(folder, exist_ok=True)
    Image.fromarray(out).save(os.path.join(folder, FILE), quality=95, subsampling=0)
    meta = {'purpose': 'Real-ESRGAN x4plus upscale of the colour_near picture over the detail box (see upscale.py)',
            'model': 'RealESRGAN_x4plus', 'scale': SCALE, 'x0': near.x0 + c0 * near.res, 'y1': near.y1 - r0 * near.res,
            'res': near.res / SCALE, 'width': out.shape[1], 'height': out.shape[0], 'seconds': round(time.time() - t0, 1)}
    with open(os.path.join(folder, META), 'w', encoding='utf-8') as f:
        json.dump(meta, f, indent=1)
    log(f'[upscale] done in {meta["seconds"]} s: {os.path.join(folder, FILE)} ({out.shape[1]} x {out.shape[0]} px, '
        f'{meta["res"] * 100:.2f} cm per pixel)')
    return meta


def colour_override(folder=None):
    """A texture.tile_picture colour_override(t, px) that paints a tile from the upscaled picture when the tile lies
    inside it (returns None otherwise, so the normal photo is used). Returns None when no upscaled picture exists."""
    folder = folder or default_dir()
    if not available(folder):
        return None
    from PIL import Image
    Image.MAX_IMAGE_PIXELS = None
    with open(os.path.join(folder, META), encoding='utf-8') as f:
        meta = json.load(f)
    im = Image.open(os.path.join(folder, FILE)).convert('RGB')
    res = float(meta['res'])
    bx0, by1 = float(meta['x0']), float(meta['y1'])
    bx1, by0 = bx0 + im.width * res, by1 - im.height * res

    def override(t, px):
        x0, y0, x1, y1 = t['x0'], t['y0'], t['x1'], t['y1']
        if x0 < bx0 or y0 < by0 or x1 > bx1 or y1 > by1:
            return None
        sx = (x1 - x0) / ((px - 1) * res)                           # same pixel-centre convention as texture._box
        sy = (y1 - y0) / ((px - 1) * res)
        left = (x0 - bx0) / res - 0.5 * sx
        top = (by1 - y1) / res - 0.5 * sy
        w_box, h_box = sx * px, sy * px
        left = min(max(left, 0.0), im.width - w_box)        # a tile on the picture's edge: shift by under 1 pixel
        top = min(max(top, 0.0), im.height - h_box)
        box = (left, top, left + w_box, top + h_box)
        return np.asarray(im.resize((px, px), Image.BICUBIC if max(sx, sy) <= 1.0 else Image.LANCZOS, box=box),
                          dtype=np.uint8)
    return override
