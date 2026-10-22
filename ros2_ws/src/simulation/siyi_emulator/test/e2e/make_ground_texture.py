"""
test/e2e/make_ground_texture.py -- PURPOSE: write the grass-like ground texture (ground.png) used by the
emulator's Gazebo test world (test/fixtures/gimbal_stand.sdf). Generated instead of stored so the
repository stays small: multi-scale noise in greens and browns, 1024x1024 pixels.
Usage: python3 make_ground_texture.py <output.png>
"""

import sys

import cv2
import numpy as np


def make_texture(size: int = 1024, seed: int = 11) -> np.ndarray:
    """Return a BGR uint8 grass/dirt texture of size x size pixels."""
    rng = np.random.default_rng(seed)
    acc = np.zeros((size, size), np.float32)
    for scale, weight in ((8, 0.45), (32, 0.25), (128, 0.18), (size, 0.12)):
        small = rng.random((scale, scale)).astype(np.float32)
        acc += weight * cv2.resize(small, (size, size), interpolation=cv2.INTER_CUBIC)
    acc = (acc - acc.min()) / (acc.max() - acc.min() + 1e-9)
    grass = np.array([40, 120, 60], np.float32)      # BGR
    dirt = np.array([60, 100, 140], np.float32)
    mix = np.clip((acc - 0.55) * 4.0, 0.0, 1.0)[..., None]
    img = grass * (1.0 - mix) + dirt * mix
    img *= (0.75 + 0.5 * rng.random((size, size, 1)).astype(np.float32))   # fine grain
    return np.clip(img, 0, 255).astype(np.uint8)


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "ground.png"
    if not cv2.imwrite(out, make_texture()):
        raise SystemExit(f"could not write {out}")
    print(f"wrote {out}")
