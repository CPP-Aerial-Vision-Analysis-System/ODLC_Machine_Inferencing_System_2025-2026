"""
sim_worlds/worldgen/textures.py -- PURPOSE: paint every texture of the worlds with numpy + Pillow, so the worlds
can be rebuilt anywhere without photos or downloads (deterministic for a given random seed).

What is painted here:
  grass_quilt()   the ground: ONE seamless 4096 x 4096 pasture picture covering 160 x 160 m (3.9 cm per
                  pixel, about what the drone camera resolves from 30-50 m). It has only WEAK big blotches
                  (they would show as a repeating pattern from high up); the big dry and greener areas are
                  patch pictures laid on top at random places (patch_decal + builder.py).
  patch_decal()   irregular dry-grass / lush-grass / bare-dirt patches (RGBA; transparent outside the patch).
  tent_fabric()   the nylon of the pop-up dome tent: panels, pole seams, a darker skirt.
  pad_texture()   concrete take-off pad with a white H.    runway_texture()  asphalt strip with markings.
  creek_texture() water + muddy banks along the creek of the SUAS field.

Gazebo renders transparent texture pixels as a hard cut-out (no blending), so patch edges are "dithered"
(random on/off pixels) which looks like a soft edge from the air.
"""

import numpy as np
from PIL import Image, ImageDraw

# ---------------------------------------------------------------------------------------------------------------
# noise helpers
# ---------------------------------------------------------------------------------------------------------------


def periodic_noise(rng, n, m=None, beta=1.5, max_wavelength_px=None):
    """Seamless (tileable) fractal noise, float32 array (n, m), mean 0, standard deviation 1.

    Made in the frequency domain: white noise filtered by 1/f^beta; sums of sine waves on an n x m grid repeat
    perfectly at the edges. beta ~0.7 = fine grain, ~1.5 = clouds, ~2.5 = large soft blobs.
    max_wavelength_px: remove every wave longer than this (pixels). Important for a picture that repeats:
    1/f noise keeps most of its strength in the LONGEST waves (picture-sized blotches), and those show as a
    regular pattern when the picture is tiled; cutting them leaves only features up to this size.
    """
    m = m or n
    white = rng.standard_normal((n, m)).astype(np.float32)
    spec = np.fft.rfft2(white)
    fy = np.fft.fftfreq(n)[:, None]
    fx = np.fft.rfftfreq(m)[None, :]
    freq = np.sqrt(fx * fx + fy * fy)
    freq[0, 0] = 1.0
    filt = (1.0 / freq ** beta).astype(np.float32)
    filt[0, 0] = 0.0
    if max_wavelength_px:
        filt[freq < 1.0 / max_wavelength_px] = 0.0
    out = np.fft.irfft2(spec * filt, s=(n, m)).astype(np.float32)
    out -= out.mean()
    out /= out.std() + 1e-9
    return out


def smoothstep(lo, hi, x):
    """0 below lo, 1 above hi, smooth S-curve in between (works with lo > hi for a falling edge)."""
    t = np.clip((x - lo) / (hi - lo), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def to_image(rgb):
    """float array (h, w, 3) in 0..1 -> Pillow RGB image."""
    return Image.fromarray((np.clip(rgb, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8), 'RGB')


def save_jpg(img, path, quality=85):
    """Save a JPEG (no chroma subsampling, so thin lines keep their colour)."""
    img.save(path, 'JPEG', quality=quality, optimize=True, subsampling=0)


def save_png(img, path):
    """Save a PNG (keeps the alpha channel)."""
    img.save(path, 'PNG', optimize=True)


# ---------------------------------------------------------------------------------------------------------------
# grass (the ground)
# ---------------------------------------------------------------------------------------------------------------

GREEN = np.array([0.31, 0.37, 0.17], np.float32)       # September pasture green (olive, not lawn green)
DRY = np.array([0.58, 0.53, 0.36], np.float32)         # straw-coloured dry grass
CLUMP = np.array([0.20, 0.26, 0.12], np.float32)       # dark tufts of taller weeds
DIRT = np.array([0.46, 0.38, 0.28], np.float32)        # Oklahoma brown soil
STALK = np.array([0.70, 0.65, 0.48], np.float32)       # pale dry stalks


def _grass_detail(rng, h, w, max_tuft_wavelength_px=None):
    """Small-scale grass structure shared by the ground and the patches: (grain, tufts, stalks) arrays.
    max_tuft_wavelength_px limits how large an area of denser tufts can be (see periodic_noise)."""
    fine = periodic_noise(rng, h, w, beta=0.55)            # blade-scale grain
    tufts = smoothstep(1.05, 1.9, periodic_noise(rng, h, w, beta=1.05, max_wavelength_px=max_tuft_wavelength_px)
                       + 0.35 * fine)                      # 5-30 cm tufts of darker weeds
    stalks = smoothstep(2.0, 2.8, periodic_noise(rng, h, w, beta=0.35))                  # scattered pale bits
    return fine, tufts, stalks


def grass_quilt(rng, size=4096):
    """The seamless ground picture (Pillow RGB, size x size) for a 160 x 160 m tile (3.9 cm per pixel).

    It repeats every 160 m, so it must NOT contain big blotches: anything larger than ~30 m shows up as a regular
    pattern from high up (tried and seen in test renders). So every noise layer is band-limited
    (periodic_noise max_wavelength_px): dry/green spots of 1-10 m, a weak 8-30 m layer, tuft clusters up to 5 m.
    None of that repeats inside one camera picture; the big dry/green areas come from patch pictures placed at
    random (patch_decal, builder.py).
    """
    px_per_m = size / 160.0                                # the picture covers 160 m
    mid = periodic_noise(rng, size, beta=2.0, max_wavelength_px=30 * px_per_m)       # 8-30 m, weak
    medium = periodic_noise(rng, size, beta=1.6, max_wavelength_px=10 * px_per_m)    # 1-10 m: the main variation
    fine, tufts, stalks = _grass_detail(rng, size, size, max_tuft_wavelength_px=5 * px_per_m)
    dry_mix = smoothstep(-1.6, 1.6, 0.30 * mid + 0.60 * medium)[..., None]
    rgb = GREEN * (1.0 - dry_mix) + DRY * dry_mix
    rgb = rgb * (1.0 - 0.55 * tufts[..., None]) + CLUMP * (0.55 * tufts[..., None])
    rgb = rgb * (1.0 - 0.6 * stalks[..., None]) + STALK * (0.6 * stalks[..., None])
    rgb = rgb * (1.0 + 0.06 * medium + 0.15 * fine)[..., None]
    return to_image(rgb)


def _blob_alpha(rng, px, roundness=0.78, wobble=0.14, rim=0.16):
    """Irregular blob mask: alpha 255 inside, 0 outside, dithered (random) rim. Returns uint8 (px, px)."""
    yy, xx = np.mgrid[0:px, 0:px].astype(np.float32)
    c = px / 2.0
    ang = np.arctan2(yy - c, xx - c)
    tone = periodic_noise(rng, px, beta=2.0)
    edge_r = (roundness + wobble * np.sin(3 * ang + rng.uniform(0, 6.28))
              + 0.6 * wobble * np.sin(5 * ang + rng.uniform(0, 6.28)) + 0.35 * wobble * tone)
    r = np.hypot(xx - c, yy - c) / c
    inside = smoothstep(edge_r, edge_r - rim, r)            # 1 inside, 0 outside
    return (rng.random((px, px)) < inside).astype(np.uint8) * 255


def patch_decal(rng, px, kind):
    """A patch of 'dry' grass, 'lush' grass or bare 'dirt' as an RGBA Pillow image (px x px).

    The inside keeps grass-like fine detail so it matches the ground around it; the edge is dithered.
    """
    fine, tufts, stalks = _grass_detail(rng, px, px)
    tone = periodic_noise(rng, px, beta=2.0)
    if kind == 'dry':
        # same mix as the dry areas of the ground texture (straw with some green), so it blends in
        base = (DRY * 0.74 + GREEN * 0.26) * 0.96 * (1.0 + 0.08 * tone[..., None])
        rgb = base * (1.0 - 0.45 * tufts[..., None]) + CLUMP * (0.45 * tufts[..., None])
        rgb = rgb * (1.0 - 0.4 * stalks[..., None]) + STALK * (0.4 * stalks[..., None])
    elif kind == 'lush':
        base = (GREEN * 0.8 + CLUMP * 0.2) * (1.0 + 0.10 * tone[..., None])
        rgb = base * (1.0 - 0.5 * tufts[..., None]) + CLUMP * (0.5 * tufts[..., None])
    elif kind == 'dirt':
        base = DIRT * (1.0 + 0.12 * tone[..., None])
        sparse = smoothstep(1.2, 1.8, periodic_noise(rng, px, beta=1.0))[..., None]   # a few grass tufts
        rgb = base * (1.0 - sparse) + GREEN * sparse
    else:
        raise ValueError(f'unknown patch kind "{kind}" (dry, lush, dirt)')
    rgb = rgb * (1.0 + 0.14 * fine)[..., None]
    if kind == 'dirt':
        alpha = _blob_alpha(rng, px, roundness=0.74, rim=0.14)
    else:                       # grass patches: very irregular, wide dithered rim = a soft natural transition
        alpha = _blob_alpha(rng, px, roundness=0.68, wobble=0.20, rim=0.38)
    out = np.dstack([(np.clip(rgb, 0, 1) * 255 + 0.5).astype(np.uint8), alpha])
    return Image.fromarray(out, 'RGBA')


# ---------------------------------------------------------------------------------------------------------------
# tent fabric (pop-up dome tent, the search target)
# ---------------------------------------------------------------------------------------------------------------

def tent_fabric(rng, colour, accent, w=1024, h=512):
    """Nylon texture for the dome tent mesh (u = around the tent, v = from the ground (0) to the top (1)).

    colour / accent: (r, g, b) 0..1 for the panels and the skirt + seams. Four pole seams run up the dome at
    45, 135, 225 and 315 degrees and cross at the top, like the ribs of an umbrella seen from above.
    """
    u = np.linspace(0.0, 1.0, w, endpoint=False, dtype=np.float32)[None, :]
    # image row 0 is the TOP of the picture = the apex (v = 1); the last row is the ground (v = 0), which is
    # how OBJ texture coordinates (v pointing up) sample a picture
    v = np.linspace(1.0, 0.0, h, dtype=np.float32)[:, None]
    shade = periodic_noise(rng, h, w, beta=1.3)
    col = np.array(colour, np.float32)
    acc = np.array(accent, np.float32)
    rgb = np.broadcast_to(col, (h, w, 3)).astype(np.float32) * (1.0 + 0.04 * shade[..., None])
    # skirt: the lowest 18 % of the dome in the accent colour
    skirt = smoothstep(0.20, 0.16, v)[..., None]
    rgb = rgb * (1.0 - skirt) + acc * skirt
    # pole seams (dark), at u = 1/8, 3/8, 5/8, 7/8, narrowing near the top where the meridians converge
    seam = np.zeros((h, w), np.float32)
    width = 0.010 / np.maximum(np.cos(v * np.pi / 2.0), 0.08)
    for k in range(4):
        d = np.abs(((u - (2 * k + 1) / 8.0) + 0.5) % 1.0 - 0.5)
        seam = np.maximum(seam, smoothstep(width * 1.6, width * 0.6, d))
    rgb = rgb * (1.0 - 0.6 * seam[..., None]) + (col * 0.55) * (0.6 * seam[..., None])   # pole sleeves
    # a light band (mesh window strip) a little below the top, typical of pop-up tents
    band = smoothstep(0.012, 0.004, np.abs(v - 0.80))[..., None]
    rgb = rgb * (1.0 - 0.35 * band) + np.array([0.85, 0.85, 0.85], np.float32) * (0.35 * band)
    return to_image(rgb)


# ---------------------------------------------------------------------------------------------------------------
# pad, runway, creek (from the first twin prototype, unchanged in spirit)
# ---------------------------------------------------------------------------------------------------------------

def _digit_segments(d):
    table = {'0': 'abcdef', '1': 'bc', '2': 'abdeg', '3': 'abcdg', '4': 'bcfg',
             '5': 'acdfg', '6': 'acdefg', '7': 'abc', '8': 'abcdefg', '9': 'abcdfg'}
    return table[d]


def draw_digit(draw, x0, y0, w, h, digit, fill, stroke):
    """Paint a 7-segment digit in the box (x0, y0, w, h) (pixels)."""
    s = stroke
    segs = {'a': (x0, y0, x0 + w, y0 + s), 'b': (x0 + w - s, y0, x0 + w, y0 + h / 2),
            'c': (x0 + w - s, y0 + h / 2, x0 + w, y0 + h), 'd': (x0, y0 + h - s, x0 + w, y0 + h),
            'e': (x0, y0 + h / 2, x0 + s, y0 + h), 'f': (x0, y0, x0 + s, y0 + h / 2),
            'g': (x0, y0 + h / 2 - s / 2, x0 + w, y0 + h / 2 + s / 2)}
    for k in _digit_segments(digit):
        draw.rectangle(segs[k], fill=fill)


def runway_texture(rng, length_m, width_m, px_per_m=24.0, numbers=('02', '20')):
    """Asphalt strip with edge lines, dashed centre line, threshold bars and runway numbers (x = long axis)."""
    W = int(round(length_m * px_per_m))
    Hh = int(round(width_m * px_per_m))
    grain = periodic_noise(rng, Hh, W, beta=0.8)
    patches = periodic_noise(rng, Hh, W, beta=2.0)
    base = np.array([0.30, 0.30, 0.31], np.float32)
    img = to_image(base[None, None, :] * (1.0 + 0.06 * grain + 0.08 * patches)[..., None])
    draw = ImageDraw.Draw(img)
    white = (232, 232, 226)
    p = px_per_m
    for y in (0.6 * p, Hh - 0.6 * p):
        draw.rectangle((0, y - 0.1 * p, W, y + 0.1 * p), fill=white)
    yc = Hh / 2
    x = 16.0 * p
    while x < W - 16.0 * p:
        draw.rectangle((x, yc - 0.15 * p, min(x + 4.0 * p, W - 16 * p), yc + 0.15 * p), fill=white)
        x += 8.0 * p
    n_keys, key_w, gap = 6, 0.9 * p, 0.8 * p
    y_start = yc - (n_keys * key_w + (n_keys - 1) * gap) / 2
    for end in (0, 1):
        x0 = 2.0 * p if end == 0 else W - 10.0 * p
        for k in range(n_keys):
            y = y_start + k * (key_w + gap)
            draw.rectangle((x0, y, x0 + 8.0 * p, y + key_w), fill=white)
    dw, dh, st = 2.0 * p, 5.0 * p, 0.45 * p
    for end, text in zip((0, 1), numbers):
        x0 = 11.5 * p if end == 0 else W - 11.5 * p - (2 * dw + 0.8 * p)
        for i, ch in enumerate(text):
            draw_digit(draw, x0 + i * (dw + 0.8 * p), yc - dh / 2, dw, dh, ch, white, st)
    return img


def pad_texture(rng, size_m, px=1024):
    """Square concrete pad with a white circle and a big white H (the drone's home)."""
    grain = periodic_noise(rng, px, beta=0.8)
    patches = periodic_noise(rng, px, beta=2.2)
    base = np.array([0.55, 0.55, 0.53], np.float32)
    img = to_image(base[None, None, :] * (1.0 + 0.05 * grain + 0.06 * patches)[..., None])
    draw = ImageDraw.Draw(img)
    p = px / size_m
    c = px / 2
    white = (238, 238, 234)
    r_out = 4.6 * p
    r_in = r_out - 0.4 * p
    draw.ellipse((c - r_out, c - r_out, c + r_out, c + r_out), fill=white)
    draw.ellipse((c - r_in, c - r_in, c + r_in, c + r_in), fill=tuple(int(v * 255) for v in base))
    hh, hw, s = 3.0 * p, 2.4 * p, 0.45 * p
    draw.rectangle((c - hw / 2, c - hh / 2, c - hw / 2 + s, c + hh / 2), fill=white)
    draw.rectangle((c + hw / 2 - s, c - hh / 2, c + hw / 2, c + hh / 2), fill=white)
    draw.rectangle((c - hw / 2, c - s / 2, c + hw / 2, c + s / 2), fill=white)
    draw.rectangle((0, 0, px - 1, px - 1), outline=(70, 70, 68), width=max(1, int(0.15 * p)))
    return img


def creek_texture(rng, w=1024, h=256):
    """Creek strip: u (image x) runs along the creek, v (image y) across: grass, mud bank, water, bank, grass."""
    v = np.linspace(0.0, 1.0, h, dtype=np.float32)[:, None]
    along = periodic_noise(rng, h, w, beta=1.2)
    streaks = periodic_noise(rng, h, w, beta=0.9)
    streaks = (streaks + np.roll(streaks, 3, axis=1) + np.roll(streaks, 6, axis=1)) / 3.0
    water = np.array([0.17, 0.23, 0.21], np.float32)
    mud = np.array([0.46, 0.39, 0.29], np.float32)
    grass = np.array([0.33, 0.38, 0.18], np.float32)
    d = np.abs(v - 0.5) * 2.0 + 0.04 * along
    water_mix = 1.0 - smoothstep(0.50, 0.62, d)
    mud_mix = smoothstep(0.50, 0.62, d) * (1.0 - smoothstep(0.86, 1.0, d))
    grass_mix = smoothstep(0.86, 1.0, d)
    rgb = (water[None, None, :] * water_mix[..., None] + mud[None, None, :] * mud_mix[..., None]
           + grass[None, None, :] * grass_mix[..., None])
    rgb = rgb * (1.0 + 0.10 * streaks * water_mix + 0.08 * along * (1 - water_mix))[..., None]
    return to_image(rgb)
