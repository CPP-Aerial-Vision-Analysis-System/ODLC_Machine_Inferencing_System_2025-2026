#!/usr/bin/env python3
"""Batch orthomosaic pipeline: folder of geotagged photos -> one map.

Run AFTER the flight, once no more images will be produced. Steps:

  1. Scan images_dir; build an ImagePose per usable image (sidecar JSON
     preferred, "<lat> , <lon>.jpg" filename + defaults as fallback).
  2. Resolve a heading for every image (sidecar compass or GPS track).
  3. Project every footprint onto a local ENU ground plane; size a canvas.
  4. Warp + feather-blend each image (chronological order), optionally
     ECC-refining each placement against the already-painted canvas.
  5. Write <name>.jpg + <name>.jgw (world file) + <name>_report.json.

ROS-free — used by both the ROS node and the offline CLI.
"""

import json
import math
import os
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import cv2
import numpy as np

from .canvas import MosaicCanvas, world_file_content
from .config import MappingConfig
from .geo_math import enu_to_latlon, ground_footprint, latlon_to_enu
from .metadata import ImagePose, assign_headings, collect_poses

LogFn = Callable[[str], None]


@dataclass
class MapResult:
    ok: bool
    message: str
    image_path: Optional[str] = None
    world_file_path: Optional[str] = None
    report_path: Optional[str] = None
    total_images: int = 0
    used_images: int = 0
    coverage: float = 0.0
    gsd_m: float = 0.0
    canvas_size: tuple = (0, 0)
    elapsed_s: float = 0.0
    skipped: List[dict] = field(default_factory=list)


def build_map(cfg: MappingConfig, log: LogFn = print,
              progress: Optional[Callable[[int, int], None]] = None) -> MapResult:
    """Run the full batch pipeline. Never raises for per-image problems —
    bad images are skipped and listed in the report."""
    started = time.time()

    # ------------------------------------------------ 1. collect poses
    if not os.path.isdir(cfg.images_dir):
        return MapResult(False, f"images_dir does not exist: {cfg.images_dir}")

    try:
        poses, skipped = collect_poses(cfg.images_dir, cfg.default_altitude_agl_m)
    except RuntimeError as exc:
        return MapResult(False, str(exc))

    total = len(poses) + len(skipped)
    log(f"[mapper] {len(poses)} usable image(s), {len(skipped)} skipped, "
        f"in {cfg.images_dir}")
    if not poses:
        return MapResult(False, "no usable geotagged images found",
                         total_images=total, skipped=skipped)

    # ------------------------------------------------ 2. headings
    assign_headings(poses, cfg.heading_source, cfg.fixed_heading_deg)
    if cfg.heading_source == 'sidecar':
        dropped = [p for p in poses if p.heading_deg is None]
        for p in dropped:
            skipped.append({'file': os.path.basename(p.path),
                            'reason': "heading_source='sidecar' but no sidecar heading"})
        poses = [p for p in poses if p.heading_deg is not None]
        if not poses:
            return MapResult(False, "no images have sidecar headings",
                             total_images=total, skipped=skipped)

    # ------------------------------------------------ 3. footprints + canvas
    ref_lat = float(np.mean([p.lat for p in poses]))
    ref_lon = float(np.mean([p.lon for p in poses]))

    placements = []
    for pose in poses:
        east, north = latlon_to_enu(pose.lat, pose.lon, ref_lat, ref_lon)
        yaw, pitch, roll = _camera_angles(pose, cfg)
        footprint = ground_footprint(
            east, north, pose.alt_agl_m, yaw, pitch, roll,
            pose.width, pose.height, cfg.hfov_deg,
            min_down_component=cfg.min_down_component,
            max_span_factor=cfg.max_span_factor)
        if footprint is None:
            skipped.append({'file': os.path.basename(pose.path),
                            'reason': f'unmappable view (pitch {pitch:.0f} deg, '
                                      f'alt {pose.alt_agl_m:.0f} m)'})
            continue
        placements.append((pose, footprint))

    if not placements:
        return MapResult(False, "no image produced a valid ground footprint",
                         total_images=total, skipped=skipped)

    all_pts = np.vstack([fp for _, fp in placements])
    canvas = MosaicCanvas(
        east_min=float(all_pts[:, 0].min()), east_max=float(all_pts[:, 0].max()),
        north_min=float(all_pts[:, 1].min()), north_max=float(all_pts[:, 1].max()),
        gsd_m=cfg.gsd_m, max_pixels=cfg.max_canvas_mp * 1e6, logger=log)

    # ------------------------------------------------ 4. paint
    used = 0
    per_image = []
    max_shift_px = cfg.max_refine_shift_m / canvas.gsd_m
    for idx, (pose, footprint) in enumerate(placements):
        name = os.path.basename(pose.path)
        img = cv2.imread(pose.path, cv2.IMREAD_COLOR)
        if img is None:
            skipped.append({'file': name, 'reason': 'decode failed'})
            continue
        try:
            status = canvas.paste(
                img, footprint,
                feather_frac=cfg.feather_frac,
                refine=cfg.refine,
                max_refine_shift_px=max_shift_px)
        except cv2.error as exc:
            skipped.append({'file': name, 'reason': f'warp/blend error: {exc}'})
            continue

        if status.get('painted'):
            used += 1
            per_image.append({
                'file': name,
                'heading_deg': round(pose.heading_deg or 0.0, 1),
                'heading_source': pose.heading_source,
                'alt_agl_m': round(pose.alt_agl_m, 1),
                'sidecar': pose.has_sidecar,
                'refined': status.get('refined', False),
                'shift_px': round(status.get('shift_px', 0.0), 2),
                'notes': pose.notes,
            })
        else:
            skipped.append({'file': name,
                            'reason': status.get('reason', 'not painted')})

        if progress is not None:
            progress(idx + 1, len(placements))

    if used == 0:
        return MapResult(False, "no image could be painted onto the canvas",
                         total_images=total, skipped=skipped)

    # ------------------------------------------------ 5. output
    final = canvas.finalize()
    if final is None:
        return MapResult(False, "canvas empty after painting",
                         total_images=total, skipped=skipped)

    os.makedirs(cfg.output_dir, exist_ok=True)
    basename = cfg.output_basename or time.strftime('mapped_%Y%m%d-%H%M%S')
    image_path = os.path.join(cfg.output_dir, basename + '.jpg')
    world_path = os.path.join(cfg.output_dir, basename + '.jgw')
    report_path = os.path.join(cfg.output_dir, basename + '_report.json')

    if not cv2.imwrite(image_path, final['image'],
                       [cv2.IMWRITE_JPEG_QUALITY, int(cfg.jpeg_quality)]):
        return MapResult(False, f"failed to write {image_path}",
                         total_images=total, skipped=skipped)

    tl_lat, tl_lon = enu_to_latlon(final['top_left_east'],
                                   final['top_left_north'], ref_lat, ref_lon)
    with open(world_path, 'w', encoding='utf-8') as fh:
        fh.write(world_file_content(tl_lat, tl_lon, final['gsd_m'], ref_lat))

    h, w = final['image'].shape[:2]
    elapsed = time.time() - started
    report = {
        'generated_unix': time.time(),
        'images_dir': cfg.images_dir,
        'output_image': image_path,
        'total_images_seen': total,
        'images_used': used,
        'images_skipped': len(skipped),
        'coverage_of_bounding_box': round(final['coverage'], 4),
        'gsd_m_per_px': round(final['gsd_m'], 4),
        'canvas_px': [w, h],
        'reference_latlon': [ref_lat, ref_lon],
        'top_left_pixel_latlon': [tl_lat, tl_lon],
        'heading_source_setting': cfg.heading_source,
        'refine_enabled': cfg.refine,
        'elapsed_s': round(elapsed, 1),
        'skipped': skipped,
        'per_image': per_image,
    }
    with open(report_path, 'w', encoding='utf-8') as fh:
        json.dump(report, fh, indent=2)

    log(f"[mapper] DONE: {used}/{total} images -> {w}x{h} px, "
        f"coverage {final['coverage'] * 100:.0f}%, {elapsed:.1f}s")
    log(f"[mapper] map:    {image_path}")
    log(f"[mapper] world:  {world_path}")
    log(f"[mapper] report: {report_path}")

    return MapResult(
        ok=True,
        message=f"mapped {used}/{total} images, coverage "
                f"{final['coverage'] * 100:.0f}%",
        image_path=image_path,
        world_file_path=world_path,
        report_path=report_path,
        total_images=total,
        used_images=used,
        coverage=final['coverage'],
        gsd_m=final['gsd_m'],
        canvas_size=(w, h),
        elapsed_s=elapsed,
        skipped=skipped,
    )


def _camera_angles(pose: ImagePose, cfg: MappingConfig) -> tuple:
    """(yaw, pitch, roll) of the CAMERA in world terms for one image.

    Gimbal yaw from the SIYI is relative to the airframe (follow mode), so
    camera yaw = aircraft heading + gimbal yaw. Without gimbal data we
    assume a locked nadir camera, which is how the mapping mission flies.
    """
    heading = pose.heading_deg if pose.heading_deg is not None else 0.0
    yaw = heading + cfg.yaw_offset_deg
    pitch = -90.0
    roll = 0.0
    if cfg.use_gimbal_attitude:
        if pose.gimbal_yaw_deg is not None:
            yaw += pose.gimbal_yaw_deg
        if pose.gimbal_pitch_deg is not None:
            pitch = pose.gimbal_pitch_deg
        if pose.gimbal_roll_deg is not None:
            roll = pose.gimbal_roll_deg
    return yaw % 360.0, pitch, roll


def estimate_native_gsd(poses: List[ImagePose], hfov_deg: float) -> float:
    """Median native GSD across poses — a sane lower bound for cfg.gsd_m."""
    values = [2.0 * p.alt_agl_m * math.tan(math.radians(hfov_deg) / 2.0) / p.width
              for p in poses if p.width > 0]
    return float(np.median(values)) if values else 0.0
