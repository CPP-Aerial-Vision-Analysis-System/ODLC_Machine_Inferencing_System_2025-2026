"""
sim_demo/truth_model.py -- PURPOSE: the brain of the truth_detector stand-in. Given one photo's
camera pose and the world's answer key, decide which targets the photo shows, where their
boxes are in the picture, and how confident a real detector would plausibly be.

WHY a stand-in: the team's YOLO model was trained on real photos (COCO); simulated mannequins
and tents may look different enough that it misses them. The stand-in lets the REST of the
mission (main_controller choosing targets, flying there, dropping) be demonstrated anyway.
It is clearly labelled everywhere as "not the AI model".

Realism rules (all tunable through TruthModelConfig):
  * a target is reported only if at least `min_visible_fraction` of its box is inside the
    photo and it is at least `min_pixels` long in the picture (too small = not detectable);
  * detection probability `detect_probability` (a deterministic pseudo-random draw per photo
    and target, so the same run gives the same result);
  * confidence = class base value x size factor (small in the picture = less sure) x edge
    factor (cut off by the photo border = less sure) x centre factor (lens edges = slightly
    less sure) + small noise, clipped to [min_confidence, max_confidence];
  * box centre and size get a few percent of noise, like a real detector's jitter.
This module has no ROS imports (pure Python; unit-tested).
"""
from __future__ import annotations

import math
import random
import zlib
from dataclasses import dataclass

from .camera_model import CameraIntrinsics, CameraPose, box_corners_3d, project_box
from .detections import Box
from .world_info import TruthTarget


@dataclass
class TruthModelConfig:
    """Knobs of the stand-in (defaults chosen to look like a decent real detector).

    Attributes:
        min_visible_fraction: 0..1 of the box that must be inside the photo.
        min_pixels: smallest detectable object size (longest side) in pixels.
        good_pixels: size at which the detector is fully confident.
        detect_probability: chance a visible, big-enough target is reported.
        base_confidence: per-class confidence for a big, centred, fully visible target.
        noise_sigma: standard deviation of the confidence noise.
        min_confidence, max_confidence: output clip range (new_od drops < 0.25).
        box_jitter: box centre/size noise as a fraction of the box size.
        seed: changes all random draws (same seed + same photos = same output).
    """

    min_visible_fraction: float = 0.5
    min_pixels: float = 12.0
    good_pixels: float = 60.0
    detect_probability: float = 0.97
    base_confidence: tuple = (('person', 0.82), ('tent', 0.88))
    noise_sigma: float = 0.03
    min_confidence: float = 0.30
    max_confidence: float = 0.97
    box_jitter: float = 0.04
    seed: int = 0

    def base_for(self, cls: str) -> float:
        """Base confidence of a class (0.8 for an unknown class)."""
        return dict(self.base_confidence).get(cls, 0.8)


@dataclass
class TruthHit:
    """One target the stand-in reports in a photo (box + score + which truth target)."""

    target: TruthTarget
    box: Box
    visible_fraction: float
    pixels: float


def _rng_for(image_name: str, target_name: str, seed: int) -> random.Random:
    """A pseudo-random generator that gives the same numbers for the same photo + target."""
    key = f'{image_name}|{target_name}|{seed}'.encode()
    return random.Random(zlib.crc32(key))


def confidence_for(cfg: TruthModelConfig, cls: str, pixels: float, visible_fraction: float,
                   center_norm: float, noise: float = 0.0) -> float:
    """The stand-in's confidence model (pure function; see the module docstring).

    Args:
        cls: 'person' or 'tent'. pixels: longest side of the object in the photo [px].
        visible_fraction: 0..1 inside the photo. center_norm: 0 = image centre, 1 = corner.
        noise: added at the end (already drawn by the caller).
    Returns:
        confidence in [cfg.min_confidence, cfg.max_confidence].
    """
    span = max(1e-6, cfg.good_pixels - cfg.min_pixels)
    size_f = 0.6 + 0.4 * min(1.0, max(0.0, (pixels - cfg.min_pixels) / span))
    edge_f = math.sqrt(max(0.0, min(1.0, visible_fraction)))
    center_f = 1.0 - 0.15 * min(1.0, center_norm) ** 2
    c = cfg.base_for(cls) * size_f * edge_f * center_f + noise
    return max(cfg.min_confidence, min(cfg.max_confidence, c))


def detect_targets(intr: CameraIntrinsics, pose: CameraPose, targets: list[TruthTarget],
                   image_name: str, cfg: TruthModelConfig | None = None) -> list[TruthHit]:
    """Which targets a photo shows, with boxes and confidences.

    Args:
        intr: photo intrinsics (size, field of view, zoom at capture).
        pose: camera pose at capture (world ENU metres; see camera_model).
        targets: the world's answer key.
        image_name: the photo's file name (seeds the deterministic randomness).
        cfg: stand-in knobs.
    Returns:
        list of TruthHit (empty if no target is visible).
    """
    cfg = cfg or TruthModelConfig()
    hits = []
    for tg in targets:
        corners = box_corners_3d(tg.east, tg.north, tg.up, tg.size[0], tg.size[1], tg.size[2],
                                 tg.yaw_deg)
        pb = project_box(intr, pose, corners)
        if pb is None:
            continue
        if pb.visible_fraction < cfg.min_visible_fraction or pb.longest_side_px < cfg.min_pixels:
            continue
        rng = _rng_for(image_name, tg.name, cfg.seed)
        if rng.random() > cfg.detect_probability:
            continue
        conf = confidence_for(cfg, tg.cls, pb.longest_side_px, pb.visible_fraction,
                              pb.center_norm, rng.gauss(0.0, cfg.noise_sigma))
        j = cfg.box_jitter
        w = max(2.0, pb.width * (1.0 + rng.gauss(0.0, j)))
        h = max(2.0, pb.height * (1.0 + rng.gauss(0.0, j)))
        cx = min(max(pb.cx + rng.gauss(0.0, j) * pb.width, 0.0), float(intr.width))
        cy = min(max(pb.cy + rng.gauss(0.0, j) * pb.height, 0.0), float(intr.height))
        hits.append(TruthHit(tg, Box(tg.cls, round(conf, 3), cx, cy, w, h),
                             pb.visible_fraction, pb.longest_side_px))
    return hits


def pose_from_sample(origin, sample, gimbal_pitch_deg: float = -90.0,
                     gimbal_yaw_offset_deg: float = 0.0, ground_up: float = 0.0,
                     default_height: float = 40.0) -> CameraPose:
    """Build the camera pose of a photo from a recorded drone state.

    Args:
        origin: geo.GeoOrigin of the world.
        sample: pose_history.PoseSample at capture time.
        gimbal_pitch_deg: camera tilt (-90 = straight down; the emulator stabilises it).
        gimbal_yaw_offset_deg: camera yaw relative to the nose, compass sense (+ = right).
        ground_up: height of the ground in the world [m] (flat field: 0).
        default_height: used when the height above home is unknown.
    Returns:
        CameraPose in world ENU metres.
    """
    east, north, _ = origin.to_enu(sample.lat, sample.lon)
    rel = sample.rel_alt if not math.isnan(sample.rel_alt) else default_height
    heading = sample.heading_deg if not math.isnan(sample.heading_deg) else 0.0
    return CameraPose(east=east, north=north, up=ground_up + rel,
                      yaw_deg=(heading + gimbal_yaw_offset_deg) % 360.0,
                      pitch_deg=gimbal_pitch_deg, roll_deg=0.0)
