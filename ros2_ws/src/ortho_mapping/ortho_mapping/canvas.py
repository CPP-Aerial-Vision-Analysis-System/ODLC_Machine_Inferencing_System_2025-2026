#!/usr/bin/env python3
"""Metric mosaic canvas: warp images onto a north-up ground plane and blend.

The canvas is a regular raster over local ENU ground coordinates:
  - column x grows EAST, row y grows SOUTH (north-up image),
  - one pixel = gsd_m metres on the ground.

Blending is an incremental weighted average with feathered (distance-to-
edge) per-image weights, so seams fade instead of showing hard steps and
memory stays at one uint8 canvas + one float32 weight map.
"""

import math
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from .config import (
    REFINE_EPS,
    REFINE_MAX_DIM_PX,
    REFINE_MAX_ITER,
    REFINE_MIN_OVERLAP_FRAC,
)

# Weight floor inside a footprint so even the extreme edge of a lone image
# still paints (weight 0 there would leave pinholes where nothing overlaps).
_MIN_FEATHER_WEIGHT = 0.02


class MosaicCanvas:

    def __init__(self, east_min: float, east_max: float,
                 north_min: float, north_max: float,
                 gsd_m: float, max_pixels: float,
                 margin_m: float = 5.0, logger=print):
        """Allocate a canvas covering the given ENU bounding box.

        If the box at gsd_m would exceed max_pixels, the GSD is coarsened
        (never refined) so the canvas always fits.
        """
        self.log = logger
        east_min -= margin_m
        north_min -= margin_m
        east_max += margin_m
        north_max += margin_m

        span_e = max(east_max - east_min, 1.0)
        span_n = max(north_max - north_min, 1.0)

        needed_px = (span_e / gsd_m) * (span_n / gsd_m)
        if needed_px > max_pixels:
            scale = math.sqrt(needed_px / max_pixels)
            gsd_m *= scale
            self.log(f"[canvas] area needs {needed_px / 1e6:.0f} MP; "
                     f"coarsening GSD to {gsd_m * 100:.1f} cm/px")

        self.gsd_m = gsd_m
        self.east_min = east_min
        self.north_max = north_max
        self.width = max(int(math.ceil(span_e / gsd_m)), 1)
        self.height = max(int(math.ceil(span_n / gsd_m)), 1)

        self.image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self.weight = np.zeros((self.height, self.width), dtype=np.float32)
        self._feather_cache: Dict[Tuple[int, int, float], np.ndarray] = {}

        self.log(f"[canvas] {self.width}x{self.height} px "
                 f"({self.width * self.height / 1e6:.1f} MP) at "
                 f"{self.gsd_m * 100:.1f} cm/px")

    # ------------------------------------------------------------------
    # Coordinates
    # ------------------------------------------------------------------

    def enu_to_px(self, east: float, north: float) -> Tuple[float, float]:
        """ENU metres -> float canvas pixel (x right/east, y down/south)."""
        x = (east - self.east_min) / self.gsd_m
        y = (self.north_max - north) / self.gsd_m
        return x, y

    def px_to_enu(self, x: float, y: float) -> Tuple[float, float]:
        east = self.east_min + x * self.gsd_m
        north = self.north_max - y * self.gsd_m
        return east, north

    # ------------------------------------------------------------------
    # Painting
    # ------------------------------------------------------------------

    def paste(self, img: np.ndarray, corners_enu: np.ndarray,
              feather_frac: float = 0.08,
              refine: bool = False,
              max_refine_shift_px: float = 0.0) -> dict:
        """Warp img so its corners land on corners_enu and blend it in.

        corners_enu: 4x2 (east, north) matching image corner order
        TL, TR, BR, BL. Returns a status dict for the run report.
        """
        status = {'painted': False, 'refined': False, 'shift_px': 0.0}

        dst_quad = np.array([self.enu_to_px(e, n) for e, n in corners_enu],
                            dtype=np.float64)

        # ROI on the canvas that this image can touch
        x0 = int(math.floor(dst_quad[:, 0].min()))
        y0 = int(math.floor(dst_quad[:, 1].min()))
        x1 = int(math.ceil(dst_quad[:, 0].max())) + 1
        y1 = int(math.ceil(dst_quad[:, 1].max())) + 1
        x0c, y0c = max(x0, 0), max(y0, 0)
        x1c, y1c = min(x1, self.width), min(y1, self.height)
        if x1c - x0c < 2 or y1c - y0c < 2:
            status['reason'] = 'footprint outside canvas'
            return status
        roi_w, roi_h = x1c - x0c, y1c - y0c

        # Pre-shrink the source so we never warp 4K pixels into a canvas
        # patch a quarter that size (big speed win on the Jetson).
        img, src_quad = self._preshrink(img, dst_quad)

        dst_local = dst_quad - np.array([x0c, y0c], dtype=np.float64)
        homography = cv2.getPerspectiveTransform(
            src_quad.astype(np.float32), dst_local.astype(np.float32))

        feather = self._feather_mask(img.shape[1], img.shape[0], feather_frac)

        if refine and max_refine_shift_px >= 1.0:
            shift = self._ecc_shift(img, homography, feather,
                                    (x0c, y0c, roi_w, roi_h),
                                    max_refine_shift_px)
            if shift is not None:
                dx, dy = shift
                translate = np.array([[1.0, 0.0, dx],
                                      [0.0, 1.0, dy],
                                      [0.0, 0.0, 1.0]])
                homography = translate @ homography
                status['refined'] = True
                status['shift_px'] = float(math.hypot(dx, dy))

        warped = cv2.warpPerspective(
            img, homography, (roi_w, roi_h),
            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        warped_w = cv2.warpPerspective(
            feather, homography, (roi_w, roi_h),
            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        self._blend(warped, warped_w, x0c, y0c)
        status['painted'] = True
        return status

    def _preshrink(self, img: np.ndarray,
                   dst_quad: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Downscale img toward the on-canvas size it will occupy."""
        h, w = img.shape[:2]
        src_quad = np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]],
                            dtype=np.float64)
        dst_edges = np.linalg.norm(np.roll(dst_quad, -1, axis=0) - dst_quad, axis=1)
        src_edges = np.linalg.norm(np.roll(src_quad, -1, axis=0) - src_quad, axis=1)
        # Keep ~25% headroom above the largest edge magnification.
        scale = 1.25 * float(np.max(dst_edges / np.maximum(src_edges, 1.0)))
        if scale < 0.95:
            new_w = max(int(round(w * scale)), 16)
            new_h = max(int(round(h * scale)), 16)
            img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
            src_quad = np.array([[0, 0], [new_w - 1, 0],
                                 [new_w - 1, new_h - 1], [0, new_h - 1]],
                                dtype=np.float64)
        return img, src_quad

    def _feather_mask(self, width: int, height: int,
                      feather_frac: float) -> np.ndarray:
        """Distance-to-edge weight in [_MIN_FEATHER_WEIGHT, 1]."""
        key = (width, height, round(feather_frac, 4))
        cached = self._feather_cache.get(key)
        if cached is not None:
            return cached
        interior = np.zeros((height, width), dtype=np.uint8)
        interior[1:-1, 1:-1] = 255
        dist = cv2.distanceTransform(interior, cv2.DIST_L2, 3)
        ramp = max(feather_frac * min(width, height), 1.0)
        mask = np.clip(dist / ramp, _MIN_FEATHER_WEIGHT, 1.0).astype(np.float32)
        if len(self._feather_cache) > 8:
            self._feather_cache.clear()
        self._feather_cache[key] = mask
        return mask

    def _blend(self, warped: np.ndarray, warped_w: np.ndarray,
               x0: int, y0: int) -> None:
        """Incremental weighted average of warped into the canvas ROI."""
        roi_h, roi_w = warped_w.shape
        img_roi = self.image[y0:y0 + roi_h, x0:x0 + roi_w]
        wgt_roi = self.weight[y0:y0 + roi_h, x0:x0 + roi_w]

        new_w = warped_w
        total = wgt_roi + new_w
        with np.errstate(divide='ignore', invalid='ignore'):
            alpha = np.where(total > 0.0, new_w / total, 0.0).astype(np.float32)

        blended = (img_roi.astype(np.float32) * (1.0 - alpha)[..., None]
                   + warped.astype(np.float32) * alpha[..., None])
        np.copyto(img_roi, np.clip(blended + 0.5, 0, 255).astype(np.uint8))
        np.copyto(wgt_roi, total)

    def _ecc_shift(self, img: np.ndarray, homography: np.ndarray,
                   feather: np.ndarray, roi: Tuple[int, int, int, int],
                   max_shift_px: float) -> Optional[Tuple[float, float]]:
        """Translation-only ECC of the incoming image against painted canvas.

        Returns (dx, dy) in canvas pixels, or None when the overlap is too
        small, ECC fails to converge, or the shift exceeds the GPS bound.
        """
        x0, y0, roi_w, roi_h = roi
        wgt_roi = self.weight[y0:y0 + roi_h, x0:x0 + roi_w]
        if float(wgt_roi.max()) <= 0.0:
            return None  # first image in this area — nothing to align to

        # Work at reduced resolution to keep ECC cheap.
        scale = min(1.0, REFINE_MAX_DIM_PX / float(max(roi_w, roi_h)))
        small_w = max(int(roi_w * scale), 8)
        small_h = max(int(roi_h * scale), 8)
        scale_mat = np.array([[scale, 0.0, 0.0],
                              [0.0, scale, 0.0],
                              [0.0, 0.0, 1.0]])
        h_small = scale_mat @ homography

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        moving = cv2.warpPerspective(gray, h_small, (small_w, small_h),
                                     flags=cv2.INTER_LINEAR)
        moving_mask = cv2.warpPerspective(
            (feather > 0).astype(np.uint8) * 255, h_small,
            (small_w, small_h), flags=cv2.INTER_NEAREST)

        canvas_roi = self.image[y0:y0 + roi_h, x0:x0 + roi_w]
        template = cv2.cvtColor(canvas_roi, cv2.COLOR_BGR2GRAY)
        template = cv2.resize(template, (small_w, small_h),
                              interpolation=cv2.INTER_AREA)
        painted = cv2.resize((wgt_roi > 0).astype(np.uint8) * 255,
                             (small_w, small_h),
                             interpolation=cv2.INTER_NEAREST)

        overlap = cv2.bitwise_and(moving_mask, painted)
        moving_area = float(np.count_nonzero(moving_mask))
        if moving_area <= 0.0:
            return None
        if float(np.count_nonzero(overlap)) / moving_area < REFINE_MIN_OVERLAP_FRAC:
            return None

        warp = np.eye(2, 3, dtype=np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                    REFINE_MAX_ITER, REFINE_EPS)
        try:
            _, warp = cv2.findTransformECC(
                templateImage=template.astype(np.float32),
                inputImage=moving.astype(np.float32),
                warpMatrix=warp,
                motionType=cv2.MOTION_TRANSLATION,
                criteria=criteria,
                inputMask=overlap,
                gaussFiltSize=5)
        except cv2.error:
            return None

        dx = float(warp[0, 2]) / scale
        dy = float(warp[1, 2]) / scale
        if math.hypot(dx, dy) > max_shift_px:
            return None  # beyond what GPS error can explain — distrust it
        return dx, dy

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------

    def coverage_fraction(self) -> float:
        """Painted fraction of the painted-region bounding box."""
        painted = self.weight > 0.0
        if not painted.any():
            return 0.0
        ys, xs = np.nonzero(painted.any(axis=1)), np.nonzero(painted.any(axis=0))
        y0, y1 = int(ys[0][0]), int(ys[0][-1]) + 1
        x0, x1 = int(xs[0][0]), int(xs[0][-1]) + 1
        box = painted[y0:y1, x0:x1]
        return float(np.count_nonzero(box)) / float(box.size)

    def finalize(self) -> Optional[dict]:
        """Crop to the painted bounding box.

        Returns {'image', 'top_left_east', 'top_left_north', 'gsd_m',
        'coverage'} or None if nothing was painted. top_left_* is the ENU
        position of the CENTre of the cropped image's top-left pixel
        (world-file convention).
        """
        painted = self.weight > 0.0
        if not painted.any():
            return None
        rows = np.nonzero(painted.any(axis=1))[0]
        cols = np.nonzero(painted.any(axis=0))[0]
        y0, y1 = int(rows[0]), int(rows[-1]) + 1
        x0, x1 = int(cols[0]), int(cols[-1]) + 1

        cropped = self.image[y0:y1, x0:x1].copy()
        east, north = self.px_to_enu(x0 + 0.5, y0 + 0.5)
        return {
            'image': cropped,
            'top_left_east': east,
            'top_left_north': north,
            'gsd_m': self.gsd_m,
            'coverage': self.coverage_fraction(),
        }


def world_file_content(top_left_lat: float, top_left_lon: float,
                       gsd_m: float, ref_lat: float) -> str:
    """ESRI world file (.jgw) for a north-up mosaic, in EPSG:4326 degrees.

    Line order: x-scale, y-rotation, x-rotation, y-scale (negative =
    north-up), x of top-left pixel centre, y of top-left pixel centre.
    """
    from .geo_math import meters_per_degree
    m_lat, m_lon = meters_per_degree(ref_lat)
    return '\n'.join([
        f"{gsd_m / m_lon:.12f}",
        "0.0",
        "0.0",
        f"{-gsd_m / m_lat:.12f}",
        f"{top_left_lon:.10f}",
        f"{top_left_lat:.10f}",
    ]) + '\n'
