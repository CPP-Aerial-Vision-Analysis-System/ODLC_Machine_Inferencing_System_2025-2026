"""
sim_worlds/realfield/dds.py -- PURPOSE: save a ground picture as a compressed DDS texture (BC1 / DXT1 with a
full mipmap chain), numpy + Pillow only.

Why: a 2048 x 2048 JPG becomes 22 MB of graphics memory in Gazebo (decoded to 4 bytes per pixel + mipmaps); the
same picture as BC1 stays compressed on the graphics card: 2.8 MB. The ~28 sharp search-area tiles then need
about 80 MB instead of 630 MB on the shared 8 GB RTX 4060 (the YOLO detector needs memory too). OgreNext (the
Gazebo renderer) loads DDS directly and, for colour maps, treats them as sRGB exactly like JPGs (checked in
gz-rendering Ogre2Material.cc). Measured by the gazebo_engine research: a BC1 DDS world loads and renders fine.

The encoder: for every 4 x 4 block the two end colours lie on the block's main colour axis (principal axis), a
little inside its extremes; each pixel picks the nearest of the 4 palette colours. Good for photos of ground.
"""

import os
import struct

import numpy as np


def _to565(c):
    c = np.clip(np.rint(c), 0, 255).astype(np.uint32)
    return ((c[..., 0] >> 3) << 11) | ((c[..., 1] >> 2) << 5) | (c[..., 2] >> 3)


def _from565(v):
    r = ((v >> 11) & 31).astype(np.float32)
    g = ((v >> 5) & 63).astype(np.float32)
    b = (v & 31).astype(np.float32)
    return np.stack([r * 255.0 / 31.0, g * 255.0 / 63.0, b * 255.0 / 31.0], -1)


def bc1_encode(rgb):
    """rgb: (H, W, 3) uint8 with H and W multiples of 4 -> BC1 block bytes (blocks in row-major order)."""
    h, w, _ = rgb.shape
    blk = rgb.reshape(h // 4, 4, w // 4, 4, 3).transpose(0, 2, 1, 3, 4).reshape(h // 4, w // 4, 16, 3)
    blk = blk.astype(np.float32)
    mean = blk.mean(2, keepdims=True)
    d = blk - mean
    cov = np.einsum('...ki,...kj->...ij', d, d)                        # (bh, bw, 3, 3)
    axis = np.ones(cov.shape[:-1], dtype=np.float32)                   # power iteration from (1, 1, 1)
    for _ in range(4):
        axis = np.einsum('...ij,...j->...i', cov, axis)
        axis /= np.linalg.norm(axis, axis=-1, keepdims=True) + 1e-6
    proj = np.einsum('...ki,...i->...k', d, axis)                       # (bh, bw, 16)
    lo, hi = proj.min(-1), proj.max(-1)
    inset = (hi - lo) / 16.0
    m = mean[..., 0, :]
    e0 = m + axis * (hi - inset)[..., None]
    e1 = m + axis * (lo + inset)[..., None]
    c0, c1 = _to565(e0), _to565(e1)
    swap = c0 < c1                                                      # c0 > c1 selects the 4-colour mode
    c0, c1 = np.where(swap, c1, c0), np.where(swap, c0, c1)
    p0, p1 = _from565(c0), _from565(c1)
    pal = np.stack([p0, p1, (2 * p0 + p1) / 3, (p0 + 2 * p1) / 3], 2)  # (bh, bw, 4, 3)
    dist = ((blk[:, :, :, None, :] - pal[:, :, None, :, :]) ** 2).sum(-1)
    idx = dist.argmin(-1).astype(np.uint32)
    idx = np.where((c0 == c1)[..., None], 0, idx)
    bits = (idx << (2 * np.arange(16, dtype=np.uint32))).sum(-1, dtype=np.uint64).astype(np.uint32)
    out = np.empty((h // 4, w // 4), dtype=[('c0', '<u2'), ('c1', '<u2'), ('i', '<u4')])
    out['c0'], out['c1'], out['i'] = c0, c1, bits
    return out.tobytes()


def _header(w, h, mips):
    flags = 0x1 | 0x2 | 0x4 | 0x1000 | 0x20000 | 0x80000       # CAPS HEIGHT WIDTH PIXELFORMAT MIPMAPCOUNT LINEARSIZE
    linear = max(1, (w + 3) // 4) * max(1, (h + 3) // 4) * 8
    hdr = struct.pack('<4s7I', b'DDS ', 124, flags, h, w, linear, 0, mips)
    hdr += struct.pack('<11I', *([0] * 11))
    hdr += struct.pack('<2I4s5I', 32, 0x4, b'DXT1', 0, 0, 0, 0, 0)   # pixel format: FOURCC DXT1
    hdr += struct.pack('<5I', 0x8 | 0x1000 | 0x400000, 0, 0, 0, 0)   # COMPLEX | TEXTURE | MIPMAP
    return hdr


def write_dds_bc1(path, rgb, strip=256):
    """Save rgb (H, W, 3) uint8 (H, W powers of two, at least 4) as BC1 DDS with a full mipmap chain down to 1 x 1.
    Returns the file size in bytes."""
    from PIL import Image
    img = Image.fromarray(np.ascontiguousarray(rgb))
    w, h = img.size
    levels = [img]
    while levels[-1].size != (1, 1):
        lw, lh = levels[-1].size
        levels.append(levels[-1].resize((max(1, lw // 2), max(1, lh // 2)), Image.BOX))
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    tmp = path + '.part'
    with open(tmp, 'wb') as f:
        f.write(_header(w, h, len(levels)))
        for im in levels:
            lw, lh = im.size
            pw, ph = max(4, (lw + 3) // 4 * 4), max(4, (lh + 3) // 4 * 4)
            a = np.asarray(im.resize((pw, ph), Image.NEAREST) if (pw, ph) != (lw, lh) else im, dtype=np.uint8)
            for r0 in range(0, ph, strip):
                f.write(bc1_encode(a[r0:r0 + strip]))
    os.replace(tmp, path)
    return os.path.getsize(path)


def decode_bc1_top(path):
    """Decode the top mipmap level of a BC1 DDS written above -> (H, W, 3) uint8 (used by the checks/previews)."""
    with open(path, 'rb') as f:
        hdr = f.read(128)
        h, w = struct.unpack('<II', hdr[12:20])
        nb = (h // 4) * (w // 4)
        blocks = np.frombuffer(f.read(nb * 8), dtype=[('c0', '<u2'), ('c1', '<u2'), ('i', '<u4')])
    c0, c1 = blocks['c0'].astype(np.uint32), blocks['c1'].astype(np.uint32)
    p0, p1 = _from565(c0), _from565(c1)
    pal = np.stack([p0, p1, (2 * p0 + p1) / 3, (p0 + 2 * p1) / 3], 1)        # (nb, 4, 3)
    idx = (blocks['i'][:, None] >> (2 * np.arange(16, dtype=np.uint32))) & 3   # (nb, 16)
    px = np.take_along_axis(pal, idx[..., None].astype(np.int64), axis=1)      # (nb, 16, 3)
    img = px.reshape(h // 4, w // 4, 4, 4, 3).transpose(0, 2, 1, 3, 4).reshape(h, w, 3)
    return np.clip(np.rint(img), 0, 255).astype(np.uint8)
