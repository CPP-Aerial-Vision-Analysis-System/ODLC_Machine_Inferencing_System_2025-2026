"""
sim_demo/test/ros_e2e/synthetic_world.py -- PURPOSE: a tiny pretend "simulator" for tests: it
paints a top-down picture of a world (grass with mowing stripes, a runway, bushes, and the
tent and mannequin from the world's answer key, at their true positions and sizes) and renders
what a camera at any pose would see, using sim_demo's own camera model. Because the pictures
and the truth_detector use the same geometry, the stand-in's boxes must land exactly on the
painted targets - a visual check of the projection math. numpy + Pillow only.
"""
from __future__ import annotations

import math

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

from sim_demo.camera_model import CameraIntrinsics, CameraPose, camera_axes


class SyntheticGround:
    """A painted ground texture in world metres, and a camera renderer for it."""

    def __init__(self, world, px_per_m: float = 20.0, margin_m: float = 60.0, seed: int = 4):
        """
        Args:
            world: world_info.WorldInfo (targets + search areas decide the painted extent).
            px_per_m: texture resolution. margin_m: extra ground around the targets/areas.
        """
        xs = [t.east for t in world.targets] + [p[0] for _, a in world.search_areas for p in a] + [0.0]
        ys = [t.north for t in world.targets] + [p[1] for _, a in world.search_areas for p in a] + [0.0]
        self.x0, self.x1 = min(xs) - margin_m, max(xs) + margin_m
        self.y0, self.y1 = min(ys) - margin_m, max(ys) + margin_m
        self.s = px_per_m
        w = int((self.x1 - self.x0) * self.s)
        h = int((self.y1 - self.y0) * self.s)
        rng = np.random.default_rng(seed)
        base = np.zeros((h, w, 3), np.float32)
        base[...] = (78, 118, 54)
        coarse = rng.normal(0, 1, (h // 16 + 1, w // 16 + 1, 1)).astype(np.float32)
        base += np.kron(coarse, np.ones((16, 16, 1), np.float32))[:h, :w] * 9
        base += rng.normal(0, 7, (h, w, 1)).astype(np.float32)
        # Mowing stripes every 8 m (east-west), so motion is visible in the live view.
        for k in range(int(self.y0 // 8) - 1, int(self.y1 // 8) + 2, 2):
            ra, rb = sorted((int(self.py(k * 8.0)), int(self.py(k * 8.0 + 8.0))))
            base[max(0, ra):max(0, rb)] += (10.0, 16.0, 6.0)
        img = Image.fromarray(np.clip(base, 0, 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(1.2))
        d = ImageDraw.Draw(img)
        # Runway along y = -10 .. 10 m.
        d.rectangle([0, self.py(10.0), w, self.py(-10.0)], fill=(96, 96, 100))
        for x in np.arange(self.x0, self.x1, 12.0):
            d.rectangle([self.px(x), self.py(0.4), self.px(x + 6.0), self.py(-0.4)], fill=(235, 235, 235))
        for k in range(40):                                    # bushes (distractor texture)
            bx, by = rng.uniform(self.x0, self.x1), rng.uniform(self.y0, self.y1)
            if -14 < by < 14:
                continue
            r = rng.uniform(0.6, 1.6) * self.s
            d.ellipse([self.px(bx) - r, self.py(by) - r, self.px(bx) + r, self.py(by) + r], fill=(40, 72, 34))
        for t in world.targets:
            self._paint_target(d, t)
        self.texture = np.asarray(img)

    # world metres -> texture pixels (north up)
    def px(self, x: float) -> float:
        """Texture column of world x [m]."""
        return (x - self.x0) * self.s

    def py(self, y: float) -> float:
        """Texture row of world y [m] (north is up)."""
        return (self.y1 - y) * self.s

    def _poly(self, cx, cy, length, width, yaw_deg):
        c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        pts = []
        for dx, dy in ((-length / 2, -width / 2), (length / 2, -width / 2), (length / 2, width / 2),
                       (-length / 2, width / 2)):
            pts.append((self.px(cx + c * dx - s * dy), self.py(cy + s * dx + c * dy)))
        return pts

    def _paint_target(self, d, t) -> None:
        L, W, _ = t.size
        if t.cls == 'tent':
            d.polygon(self._poly(t.east, t.north, L, W, t.yaw_deg), fill=(255, 132, 24),
                      outline=(140, 60, 0))
            p = self._poly(t.east, t.north, L, W, t.yaw_deg)
            d.line([p[0], p[2]], fill=(150, 70, 0), width=3)
            d.line([p[1], p[3]], fill=(150, 70, 0), width=3)
        else:
            # Lying mannequin: body along its long side, head at one end.
            long_ax = 0.0 if L >= W else 90.0
            length, width = max(L, W), min(L, W)
            d.polygon(self._poly(t.east, t.north, length * 0.8, width * 0.7, t.yaw_deg + long_ax),
                      fill=(30, 70, 190))
            a = math.radians(t.yaw_deg + long_ax)
            hx, hy = t.east + math.cos(a) * length * 0.45, t.north + math.sin(a) * length * 0.45
            r = width * 0.35 * self.s
            d.ellipse([self.px(hx) - r, self.py(hy) - r, self.px(hx) + r, self.py(hy) + r],
                      fill=(235, 196, 165))

    def render(self, intr: CameraIntrinsics, pose: CameraPose, scale: float = 1.0) -> np.ndarray:
        """What the camera sees (uint8 RGB, height x width), nearest-pixel sampling.

        scale < 1 renders a smaller image with the same field of view (faster previews).
        """
        w, h = int(intr.width * scale), int(intr.height * scale)
        x_r, y_d, z_f = (np.asarray(v, np.float32) for v in camera_axes(pose))
        a = ((np.arange(w, dtype=np.float32) + 0.5) / scale - intr.cx) / intr.fx
        b = ((np.arange(h, dtype=np.float32) + 0.5) / scale - intr.cy) / intr.fy
        ray = (x_r[None, None, :] * a[None, :, None] + y_d[None, None, :] * b[:, None, None]
               + z_f[None, None, :])
        rz = np.minimum(ray[..., 2], -1e-6)
        t = (0.0 - pose.up) / rz
        e = pose.east + t * ray[..., 0]
        n = pose.north + t * ray[..., 1]
        col = np.clip(((e - self.x0) * self.s).astype(np.int32), 0, self.texture.shape[1] - 1)
        row = np.clip(((self.y1 - n) * self.s).astype(np.int32), 0, self.texture.shape[0] - 1)
        out = self.texture[row, col]
        sky = ray[..., 2] > -1e-6
        out[sky] = (150, 190, 235)
        return out
