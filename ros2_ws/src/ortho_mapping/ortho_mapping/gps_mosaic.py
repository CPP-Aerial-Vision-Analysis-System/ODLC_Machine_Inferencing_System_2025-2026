#!/usr/bin/env python3
"""GPS-only orthomosaic builder for photos that have NO metadata sidecar.

Use this on OLD mission folders whose photos are named
"<lat> , <lon>.jpg" but were captured before siyi_node started writing
the JSON pose sidecars. The only information available is each photo's
GPS position (from its filename) -- there is no altitude, heading, or
gimbal angle. This tool reconstructs the two missing pieces (scale and
heading) directly from the imagery + GPS, so you usually need to pass
nothing but the folder.

    python3 gps_mosaic.py --dir /path/to/mappingImages

How it works
------------
1. Parse (lat, lon) from every "<lat> , <lon>.jpg" filename.
2. CHAIN-CALIBRATE ("snake" mode, the default): reconstruct the flight
   order from GPS (nearest-neighbour chain), then feature-match each
   photo against the previous one in the chain, recovering the FULL
   similarity transform (rotation + scale + translation) per link — so
   the 180-degree turns of a lawnmower pattern are handled. Composing
   the links lays every photo out in one pixel frame; a least-squares
   fit of that layout onto the GPS positions then recovers:
       - scale   = metres per pixel   (no flight altitude needed)
       - heading = each photo's OWN rotation vs. true north
   Self-guards (link scale sanity, GPS-residual check) make it fall
   back to --altitude scale + north-up when matching is untrustworthy.
3. Place every photo on a north-up metre-grid canvas at its GPS position,
   rotated by its own heading, and feather-blend the overlaps.
4. Save <out>.jpg + <out>.jgw world file + <out>_report.json.

This is deliberately a single standalone file (only needs numpy +
opencv-python) so it can run on any laptop without building the ROS
workspace. The live pipeline uses the richer ortho_mapping package.
"""

import argparse
import json
import math
import os
import sys
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np

IMAGE_EXTS = ('.jpg', '.jpeg', '.png', '.bmp')

# SIYI A8 mini stills: 3840x2160, 81 deg horizontal FOV. Only used by the
# --altitude fallback when auto-calibration cannot find enough matches.
DEFAULT_HFOV_DEG = 81.0


# ---------------------------------------------------------------------------
# Geodesy (flat-earth; sub-cm relative error over a competition-size area)
# ---------------------------------------------------------------------------

def meters_per_degree(lat_deg: float) -> Tuple[float, float]:
    lat = math.radians(lat_deg)
    m_lat = 111132.954 - 559.822 * math.cos(2 * lat) + 1.175 * math.cos(4 * lat)
    m_lon = (111412.84 * math.cos(lat) - 93.5 * math.cos(3 * lat)
             + 0.118 * math.cos(5 * lat))
    return m_lat, m_lon


def latlon_to_enu(lat, lon, ref_lat, ref_lon) -> Tuple[float, float]:
    """(east, north) metres of (lat, lon) relative to a reference point."""
    m_lat, m_lon = meters_per_degree(ref_lat)
    return (lon - ref_lon) * m_lon, (lat - ref_lat) * m_lat


# ---------------------------------------------------------------------------
# Filename parsing
# ---------------------------------------------------------------------------

def parse_latlon(filename: str) -> Optional[Tuple[float, float]]:
    """Parse the "<lat> , <lon>" stem (space-comma-space, as siyi_node writes)."""
    stem = os.path.splitext(os.path.basename(filename))[0]
    parts = stem.split(' , ')
    if len(parts) != 2:
        return None
    try:
        lat, lon = float(parts[0]), float(parts[1])
    except ValueError:
        return None
    if lat == 0.0 and lon == 0.0:
        return None
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        return None
    return lat, lon


class Photo:
    """One source photo.

    heading_deg / gsd_m are per-photo calibration results filled in by
    chain_calibrate(); None means "use the global Calibration values".
    heading_deg is in the mosaic's own rotation convention (the angle
    build_mosaic applies), not a compass bearing.
    """
    __slots__ = ('path', 'name', 'lat', 'lon', 'east', 'north',
                 'heading_deg', 'gsd_m')

    def __init__(self, path, lat, lon):
        self.path = path
        self.name = os.path.basename(path)
        self.lat = lat
        self.lon = lon
        self.east = 0.0
        self.north = 0.0
        self.heading_deg = None
        self.gsd_m = None


def load_photos(images_dir: str, ref_lat=None, ref_lon=None) -> List[Photo]:
    photos = []
    for name in sorted(os.listdir(images_dir)):
        if not name.lower().endswith(IMAGE_EXTS):
            continue
        parsed = parse_latlon(name)
        if parsed is None:
            continue
        photos.append(Photo(os.path.join(images_dir, name), *parsed))
    if not photos:
        return photos
    if ref_lat is None:
        ref_lat = sum(p.lat for p in photos) / len(photos)
        ref_lon = sum(p.lon for p in photos) / len(photos)
    for p in photos:
        p.east, p.north = latlon_to_enu(p.lat, p.lon, ref_lat, ref_lon)
    return photos


# ---------------------------------------------------------------------------
# Chain ("snake") calibration: recover scale + per-photo heading from the
# flight path itself. Works for any flight pattern — the order is rebuilt
# from GPS, and every link measures full rotation+scale, so the 180-degree
# row turns of a lawnmower/snake survey are handled.
# ---------------------------------------------------------------------------

class Calibration:
    def __init__(self, gsd_m: float, heading_deg: float, method: str,
                 pairs_used: int = 0):
        self.gsd_m = gsd_m            # ground metres per source pixel
        self.heading_deg = heading_deg  # mosaic rotation for uncalibrated photos
        self.method = method
        self.pairs_used = pairs_used
        self.residual_rms_m = None    # GPS-vs-vision fit residual (chain mode)
        self.segments = None          # number of independent chain segments


def order_by_gps(photos: List[Photo]) -> List[int]:
    """Reconstruct the flight order: greedy nearest-neighbour chain,
    starting from the photo farthest from the survey centroid (an end of
    the snake, wherever the pattern actually started)."""
    n = len(photos)
    ce = sum(p.east for p in photos) / n
    cn = sum(p.north for p in photos) / n
    start = max(range(n),
                key=lambda i: math.hypot(photos[i].east - ce,
                                         photos[i].north - cn))
    order = [start]
    todo = set(range(n)) - {start}
    while todo:
        last = photos[order[-1]]
        nxt = min(todo, key=lambda j: math.hypot(photos[j].east - last.east,
                                                 photos[j].north - last.north))
        order.append(nxt)
        todo.discard(nxt)
    return order


def _make_feature_engine():
    """SIFT if this OpenCV has it (much better on bland dirt), else ORB."""
    try:
        return cv2.SIFT_create(nfeatures=4000), cv2.NORM_L2
    except AttributeError:
        return cv2.ORB_create(nfeatures=6000, fastThreshold=8), cv2.NORM_HAMMING


def _match_pair_similarity(feat_a, feat_b, norm, min_matches=12
                           ) -> Optional[np.ndarray]:
    """RANSAC similarity (rotation+scale+translation) taking B pixels onto
    A pixels. feat_* are (keypoints, descriptors). Returns 2x3 or None."""
    (ka, da), (kb, db) = feat_a, feat_b
    if da is None or db is None or len(ka) < min_matches or len(kb) < min_matches:
        return None
    matcher = cv2.BFMatcher(norm)
    raw = matcher.knnMatch(db, da, k=2)
    good = [m for pair in raw if len(pair) == 2
            for m, n_ in [pair] if m.distance < 0.75 * n_.distance]
    if len(good) < min_matches:
        return None
    pts_b = np.float32([kb[m.queryIdx].pt for m in good])
    pts_a = np.float32([ka[m.trainIdx].pt for m in good])
    matrix, inliers = cv2.estimateAffinePartial2D(
        pts_b, pts_a, method=cv2.RANSAC, ransacReprojThreshold=4.0)
    if matrix is None or inliers is None or int(inliers.sum()) < min_matches:
        return None
    return matrix


def _homog(m2x3: np.ndarray) -> np.ndarray:
    out = np.eye(3)
    out[:2, :] = m2x3
    return out


def chain_calibrate(photos: List[Photo], work_px: int = 1400,
                    log=print) -> Optional[Calibration]:
    """Snake calibration. Fills photos[i].heading_deg / .gsd_m in place and
    returns the global Calibration, or None when matching is untrustworthy.

    1. Order photos into the flight chain (GPS nearest-neighbour).
    2. Feature-match consecutive photos -> full similarity per link.
       Composing links places every photo in the first photo's pixel frame.
    3. Least-squares similarity fit (2-D Procrustes) of those pixel-frame
       centres onto the GPS (east, north) positions. The fitted scale is
       metres-per-pixel; the fitted rotation plus each photo's own chain
       rotation is its individual mosaic heading. GPS residuals of the fit
       are the self-guard: a bad chain cannot secretly agree with GPS.
    """
    n = len(photos)
    if n < 3:
        return None

    order = order_by_gps(photos)
    engine, norm_type = _make_feature_engine()

    gray_cache = {}

    def gray(idx):
        """(downscaled gray, work scale, full W, full H) for photo idx."""
        if idx not in gray_cache:
            img = cv2.imread(photos[idx].path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                gray_cache[idx] = (None, 1.0, 0, 0)
            else:
                h, w = img.shape
                s = min(work_px / float(max(h, w)), 1.0)
                if s < 1.0:
                    img = cv2.resize(img, (int(w * s), int(h * s)),
                                     interpolation=cv2.INTER_AREA)
                gray_cache[idx] = (img, s, w, h)
        return gray_cache[idx]

    feat_cache = {}

    def features(idx):
        if idx not in feat_cache:
            img = gray(idx)[0]
            feat_cache[idx] = engine.detectAndCompute(img, None) \
                if img is not None else (None, None)
        return feat_cache[idx]

    # --- 2. chain matching -> transform of each photo in its segment frame
    # Photos are visited in flight order, but each one may register against
    # any of its nearest already-placed photos — so the adjacent snake row
    # can stitch a photo even when its own row link fails, and segments
    # merge instead of fragmenting.
    transforms = {}          # idx -> 3x3, pixels of idx -> segment-frame pixels
    seg_of = {}              # idx -> segment list it belongs to
    segments: List[List[int]] = []
    matched_links = 0
    for k, idx in enumerate(order):
        if gray(idx)[0] is None:
            continue
        candidates = sorted(
            (ref for ref in order[:k] if ref in transforms),
            key=lambda r: math.hypot(photos[r].east - photos[idx].east,
                                     photos[r].north - photos[idx].north))[:4]
        placed = False
        for ref in candidates:
            m = _match_pair_similarity(features(ref), features(idx), norm_type)
            if m is None:
                continue
            # Full-res translation (rotation/scale are scale-invariant).
            s_ref = gray(ref)[1]
            m = m.copy()
            m[:, 2] /= s_ref
            link_scale = math.hypot(m[0, 0], m[1, 0])
            if not (0.6 <= link_scale <= 1.6):     # same flight, similar alt
                continue
            transforms[idx] = transforms[ref] @ _homog(m)
            seg_of[idx] = seg_of[ref]
            seg_of[idx].append(idx)
            matched_links += 1
            placed = True
            break
        if not placed:
            seg = [idx]
            segments.append(seg)
            seg_of[idx] = seg
            transforms[idx] = np.eye(3)

    # Merge pass: a segment that failed to join when it was started may
    # still overlap photos placed later. Re-root whole segments onto each
    # other while cross-segment matches keep succeeding.
    merged = True
    while merged and len(segments) > 1:
        merged = False
        for seg in sorted(segments, key=len):
            others = [j for j in transforms if seg_of[j] is not seg]
            cross = sorted(
                ((math.hypot(photos[i].east - photos[j].east,
                             photos[i].north - photos[j].north), i, j)
                 for i in seg for j in others))[:6]
            for _, i, j in cross:
                m = _match_pair_similarity(features(j), features(i), norm_type)
                if m is None:
                    continue
                m = m.copy()
                m[:, 2] /= gray(j)[1]
                if not (0.6 <= math.hypot(m[0, 0], m[1, 0]) <= 1.6):
                    continue
                # Map seg's frame onto j's frame via the i->j link.
                bridge = transforms[j] @ _homog(m) @ np.linalg.inv(transforms[i])
                target = seg_of[j]
                for x in seg:
                    transforms[x] = bridge @ transforms[x]
                    seg_of[x] = target
                target.extend(seg)
                segments.remove(seg)
                matched_links += 1
                merged = True
                break
            if merged:
                break

    if matched_links < 0.5 * (n - 1):
        log(f"[snake] only {matched_links}/{n - 1} chain links matched -- "
            "not enough to trust vision calibration; falling back")
        return None

    # --- 3. anchor each segment to GPS with a similarity fit
    fits = []      # (seg, theta, scale, rms, span)
    for seg in segments:
        if len(seg) < 2:
            continue
        pix = []
        enu = []
        for idx in seg:
            _, _, w, h = gray(idx)
            c = transforms[idx] @ np.array([w / 2.0, h / 2.0, 1.0])
            pix.append((c[0], -c[1]))            # y-up pixel frame
            enu.append((photos[idx].east, photos[idx].north))
        p = np.array(pix)
        q = np.array(enu)
        pc = p - p.mean(axis=0)
        qc = q - q.mean(axis=0)
        denom = float((pc ** 2).sum())
        span = float(max(np.ptp(qc[:, 0]), np.ptp(qc[:, 1])))
        if denom < 1e-6 or span < 2.0:           # nothing to anchor scale to
            continue
        a = float((pc * qc).sum())
        b = float((pc[:, 0] * qc[:, 1] - pc[:, 1] * qc[:, 0]).sum())
        theta = math.atan2(b, a)                 # segment frame -> ENU
        scale = math.hypot(a, b) / denom         # metres per segment-frame px
        if not (0.001 <= scale <= 0.2):          # 0.1..20 cm/px sanity window
            continue
        rot = np.array([[math.cos(theta), -math.sin(theta)],
                        [math.sin(theta), math.cos(theta)]])
        res = scale * (pc @ rot.T) - qc
        rms = float(np.sqrt((res ** 2).sum(axis=1).mean()))
        log(f"[snake] segment: {len(seg)} photo(s), span {span:.0f} m, "
            f"GSD {scale * 100:.2f} cm/px, GPS residual {rms:.1f} m")
        fits.append((seg, theta, scale, rms, span, res))

    if not fits:
        log("[snake] no segment could be anchored to GPS; falling back")
        return None

    # Consensus guards. Absolute GPS residuals of a few metres are normal
    # consumer-GPS noise, so they cannot condemn a segment on their own.
    # What bad vision CANNOT fake is independent segments agreeing on the
    # scale, so: (a) drop segments whose GSD strays far from the
    # photo-weighted median, (b) drop truly wild GPS disagreement.
    med_scale = float(np.median(
        [s for seg, _, s, _, _, _ in fits for _ in seg]))
    gsd_values = []
    residuals = []
    fitted = 0
    for seg, theta, scale, rms, span, res in fits:
        if abs(scale - med_scale) > 0.3 * med_scale:
            log(f"[snake] segment of {len(seg)} rejected: GSD "
                f"{scale * 100:.2f} cm/px vs consensus {med_scale * 100:.2f}")
            continue
        if rms > max(6.0, 0.35 * span):
            log(f"[snake] segment of {len(seg)} rejected: GPS residual "
                f"{rms:.1f} m over {span:.0f} m")
            continue
        residuals.append(rms)
        for i, idx in enumerate(seg):
            t = transforms[idx]
            alpha = math.atan2(t[1, 0], t[0, 0])     # chain rotation, y-down
            sigma = math.hypot(t[0, 0], t[1, 0])     # chain scale drift
            photos[idx].heading_deg = math.degrees(theta - alpha)
            photos[idx].gsd_m = scale * sigma
            gsd_values.append(scale * sigma)
            # Snap the photo to the vision-consistent position (the GPS-
            # anchored chain layout). Raw per-photo GPS keeps its 2-5 m
            # noise; the chain is locally centimetre-consistent, so this
            # is what makes neighbouring tiles actually line up.
            photos[idx].east += float(res[i, 0])
            photos[idx].north += float(res[i, 1])
        fitted += len(seg)

    if fitted < 0.6 * n:
        log(f"[snake] only {fitted}/{n} photo(s) survived the GPS "
            "consistency fit; falling back")
        return None

    # Photos the chain could not place borrow the heading of their nearest
    # calibrated GPS neighbour (adjacent snake rows share orientation far
    # more often than not); scale falls back to the global median.
    calibrated = [p for p in photos if p.heading_deg is not None]
    for p in photos:
        if p.heading_deg is None:
            near = min(calibrated,
                       key=lambda c: math.hypot(c.east - p.east,
                                                c.north - p.north))
            p.heading_deg = near.heading_deg

    med_gsd = float(np.median(gsd_values))
    headings = [p.heading_deg for p in calibrated]
    calib = Calibration(med_gsd, _circular_median(headings), 'snake',
                        matched_links)
    calib.residual_rms_m = max(residuals) if residuals else None
    calib.segments = len(segments)
    log(f"[snake] calibrated {fitted}/{n} photo(s) over "
        f"{len(segments)} segment(s), {matched_links} link(s): "
        f"GSD={med_gsd * 100:.2f} cm/px, "
        f"heading spread {_circular_std(headings):.0f} deg, "
        f"GPS residual {calib.residual_rms_m:.1f} m")
    return calib


def _circular_median(angles_deg: List[float]) -> float:
    """Median of angles that wraps correctly around +/-180."""
    s = float(np.median([math.sin(math.radians(a)) for a in angles_deg]))
    c = float(np.median([math.cos(math.radians(a)) for a in angles_deg]))
    return math.degrees(math.atan2(s, c))


def _circular_std(angles_deg: List[float]) -> float:
    """Circular standard deviation in degrees (0 = perfect agreement)."""
    s = float(np.mean([math.sin(math.radians(a)) for a in angles_deg]))
    c = float(np.mean([math.cos(math.radians(a)) for a in angles_deg]))
    r = math.hypot(s, c)
    if r >= 1.0:
        return 0.0
    return math.degrees(math.sqrt(-2.0 * math.log(r)))


def altitude_calibration(altitude_m: float, hfov_deg: float,
                         image_width: int) -> Calibration:
    """Fallback GSD from a known flight altitude; heading assumed north-up."""
    ground_width = 2 * altitude_m * math.tan(math.radians(hfov_deg) / 2)
    return Calibration(ground_width / image_width, 0.0, 'altitude')


# ---------------------------------------------------------------------------
# Mosaic rendering
# ---------------------------------------------------------------------------

def _feather(width: int, height: int, frac: float) -> np.ndarray:
    interior = np.zeros((height, width), np.uint8)
    interior[1:-1, 1:-1] = 255
    dist = cv2.distanceTransform(interior, cv2.DIST_L2, 3)
    ramp = max(frac * min(width, height), 1.0)
    return np.clip(dist / ramp, 0.02, 1.0).astype(np.float32)


def build_mosaic(photos: List[Photo], calib: Calibration,
                 out_gsd_m: float, feather_frac: float = 0.08,
                 max_canvas_mp: float = 200.0, blend: str = 'center',
                 log=print) -> Optional[dict]:
    """Warp every photo onto a north-up canvas and combine the overlaps.

    blend='center' (default): each output pixel is taken from the SINGLE
        photo that covers it most centrally (winner-take-all by feather
        weight). Crisp -- best when GPS scatter means overlaps don't
        register perfectly, which is the usual GPS-only case.
    blend='feather': weighted average of all overlapping photos. Smooth
        seams, but blurs where misaligned frames disagree.
    """
    # Footprint half-extents in metres (from source size * source GSD).
    first = cv2.imread(photos[0].path, cv2.IMREAD_COLOR)
    if first is None:
        log("[mosaic] cannot read first image")
        return None
    src_h, src_w = first.shape[:2]
    margin_gsd = max(p.gsd_m or calib.gsd_m for p in photos)
    half_w_m = 0.5 * src_w * margin_gsd
    half_h_m = 0.5 * src_h * margin_gsd
    diag = math.hypot(half_w_m, half_h_m)   # rotation-safe margin

    east_min = min(p.east for p in photos) - diag
    east_max = max(p.east for p in photos) + diag
    north_min = min(p.north for p in photos) - diag
    north_max = max(p.north for p in photos) + diag

    span_e, span_n = east_max - east_min, north_max - north_min
    need_mp = (span_e / out_gsd_m) * (span_n / out_gsd_m) / 1e6
    if need_mp > max_canvas_mp:
        out_gsd_m *= math.sqrt(need_mp / max_canvas_mp)
        log(f"[mosaic] area needs {need_mp:.0f} MP; "
            f"coarsening output to {out_gsd_m * 100:.1f} cm/px")

    cw = max(int(math.ceil(span_e / out_gsd_m)), 1)
    ch = max(int(math.ceil(span_n / out_gsd_m)), 1)
    log(f"[mosaic] canvas {cw}x{ch} px ({cw * ch / 1e6:.1f} MP) at "
        f"{out_gsd_m * 100:.1f} cm/px")

    # 'feather' accumulates a weighted sum; 'center' keeps a winner-take-all
    # best-weight buffer. Both track `painted` (wsum>0) for cropping.
    accum = np.zeros((ch, cw, 3), np.float32) if blend == 'feather' else None
    wsum = np.zeros((ch, cw), np.float32)
    out = np.zeros((ch, cw, 3), np.uint8) if blend == 'center' else None
    best = np.zeros((ch, cw), np.float32) if blend == 'center' else None

    placed = 0
    for idx, p in enumerate(photos):
        img = cv2.imread(p.path, cv2.IMREAD_COLOR)
        if img is None:
            continue
        h, w = img.shape[:2]

        # Per-photo calibration when chain_calibrate provided it; otherwise
        # the single global scale + rotation.
        scale = (p.gsd_m or calib.gsd_m) / out_gsd_m
        theta = math.radians(p.heading_deg if p.heading_deg is not None
                             else calib.heading_deg)

        # Canvas pixel of this photo's centre (x east, y south).
        cx = (p.east - east_min) / out_gsd_m
        cy = (north_max - p.north) / out_gsd_m

        # Affine: scale, rotate (heading), translate so image centre -> (cx, cy).
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        a = scale * cos_t
        b = scale * sin_t
        m = np.array([
            [a, b, cx - a * (w / 2) - b * (h / 2)],
            [-b, a, cy + b * (w / 2) - a * (h / 2)],
        ], np.float32)

        warped = cv2.warpAffine(img, m, (cw, ch), flags=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_CONSTANT)
        wmask = cv2.warpAffine(_feather(w, h, feather_frac), m, (cw, ch),
                               flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_CONSTANT)
        if blend == 'feather':
            accum += warped.astype(np.float32) * wmask[..., None]
            wsum += wmask
        else:
            win = wmask > best
            out[win] = warped[win]
            best[win] = wmask[win]
            wsum = np.maximum(wsum, wmask)
        placed += 1
        if (idx + 1) % 10 == 0 or idx + 1 == len(photos):
            log(f"[mosaic] placed {idx + 1}/{len(photos)}")

    if placed == 0:
        return None

    if blend == 'feather':
        with np.errstate(divide='ignore', invalid='ignore'):
            out = np.where(wsum[..., None] > 0, accum / wsum[..., None], 0.0)
        out = np.clip(out + 0.5, 0, 255).astype(np.uint8)

    # Crop to painted region.
    painted = wsum > 0
    rows = np.nonzero(painted.any(axis=1))[0]
    cols = np.nonzero(painted.any(axis=0))[0]
    y0, y1 = int(rows[0]), int(rows[-1]) + 1
    x0, x1 = int(cols[0]), int(cols[-1]) + 1
    cropped = out[y0:y1, x0:x1]
    coverage = float(np.count_nonzero(painted[y0:y1, x0:x1])) / painted[y0:y1, x0:x1].size

    tl_east = east_min + (x0 + 0.5) * out_gsd_m
    tl_north = north_max - (y0 + 0.5) * out_gsd_m
    return {
        'image': cropped,
        'placed': placed,
        'out_gsd_m': out_gsd_m,
        'coverage': coverage,
        'tl_east': tl_east,
        'tl_north': tl_north,
    }


def world_file(tl_lat, tl_lon, gsd_m, ref_lat) -> str:
    m_lat, m_lon = meters_per_degree(ref_lat)
    return '\n'.join([
        f"{gsd_m / m_lon:.12f}", "0.0", "0.0",
        f"{-gsd_m / m_lat:.12f}", f"{tl_lon:.10f}", f"{tl_lat:.10f}",
    ]) + '\n'


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def run(args) -> int:
    log = print
    images_dir = os.path.abspath(os.path.expanduser(args.dir))
    if not os.path.isdir(images_dir):
        log(f"ERROR: not a directory: {images_dir}")
        return 1

    photos = load_photos(images_dir)
    if len(photos) < 1:
        log(f"ERROR: no GPS-named photos ('<lat> , <lon>.jpg') in {images_dir}")
        return 1
    ref_lat = sum(p.lat for p in photos) / len(photos)
    ref_lon = sum(p.lon for p in photos) / len(photos)
    log(f"[load] {len(photos)} GPS-tagged photo(s)")

    # --- calibrate scale + heading ---
    # Snake/chain calibration is the default: it reconstructs the flight
    # order, feature-matches along it, and cross-checks against GPS. It
    # self-guards and falls back to the altitude-based scale (north-up)
    # when the imagery cannot be trusted.
    calib = None
    if args.auto:
        calib = chain_calibrate(photos, log=log)
    if calib is None:
        probe = cv2.imread(photos[0].path, cv2.IMREAD_GRAYSCALE)
        width = probe.shape[1] if probe is not None else 3840
        calib = altitude_calibration(args.altitude, args.hfov, width)
        log(f"[fallback] altitude {args.altitude} m -> "
            f"GSD {calib.gsd_m * 100:.2f} cm/px, heading north-up")
    if args.heading is not None:
        calib.heading_deg = args.heading
        for p in photos:
            p.heading_deg = None
        log(f"[override] heading forced to {args.heading:.1f} deg")
    if args.gsd_cm is not None:
        calib.gsd_m = args.gsd_cm / 100.0
        for p in photos:
            p.gsd_m = None
        log(f"[override] source GSD forced to {args.gsd_cm:.2f} cm/px")

    out_gsd = args.out_gsd_cm / 100.0 if args.out_gsd_cm else calib.gsd_m

    result = build_mosaic(photos, calib, out_gsd,
                          feather_frac=args.feather, blend=args.blend,
                          max_canvas_mp=args.max_mp, log=log)
    if result is None:
        log("ERROR: nothing could be placed")
        return 1

    out_dir = os.path.abspath(os.path.expanduser(args.out)) if args.out \
        else os.path.join(os.path.dirname(images_dir), 'mapping_output')
    os.makedirs(out_dir, exist_ok=True)
    base = args.name or time.strftime('gps_map_%Y%m%d-%H%M%S')
    img_path = os.path.join(out_dir, base + '.jpg')
    jgw_path = os.path.join(out_dir, base + '.jgw')
    rep_path = os.path.join(out_dir, base + '_report.json')

    if not cv2.imwrite(img_path, result['image'],
                       [cv2.IMWRITE_JPEG_QUALITY, args.quality]):
        log(f"ERROR: failed to write {img_path}")
        return 1

    m_lat, m_lon = meters_per_degree(ref_lat)
    tl_lat = ref_lat + result['tl_north'] / m_lat
    tl_lon = ref_lon + result['tl_east'] / m_lon
    with open(jgw_path, 'w', encoding='utf-8') as fh:
        fh.write(world_file(tl_lat, tl_lon, result['out_gsd_m'], ref_lat))

    h, w = result['image'].shape[:2]
    with open(rep_path, 'w', encoding='utf-8') as fh:
        json.dump({
            'generated_unix': time.time(),
            'images_dir': images_dir,
            'photos_placed': result['placed'],
            'photos_total': len(photos),
            'calibration_method': calib.method,
            'calibration_links_matched': calib.pairs_used,
            'calibration_segments': calib.segments,
            'calibration_gps_residual_m': (
                round(calib.residual_rms_m, 2)
                if calib.residual_rms_m is not None else None),
            'source_gsd_cm_per_px': round(calib.gsd_m * 100, 3),
            'heading_deg': round(calib.heading_deg, 2),
            'per_photo_heading_deg': {
                p.name: round(p.heading_deg, 1) for p in photos
                if p.heading_deg is not None},
            'output_gsd_cm_per_px': round(result['out_gsd_m'] * 100, 3),
            'canvas_px': [w, h],
            'coverage_of_bounding_box': round(result['coverage'], 4),
            'reference_latlon': [ref_lat, ref_lon],
            'top_left_pixel_latlon': [tl_lat, tl_lon],
        }, fh, indent=2)

    log(f"[done] {result['placed']}/{len(photos)} photos -> {w}x{h} px, "
        f"coverage {result['coverage'] * 100:.0f}%")
    log(f"[done] map:    {img_path}")
    log(f"[done] world:  {jgw_path}")
    log(f"[done] report: {rep_path}")
    return 0


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Build one georeferenced map from GPS-named photos that "
                    "have no metadata sidecar (old-mission fallback).")
    p.add_argument('--dir', required=True, help='folder of "<lat> , <lon>.jpg"')
    p.add_argument('--out', default=None,
                   help='output folder (default: <dir>/../mapping_output)')
    p.add_argument('--name', default=None, help='output basename')
    p.add_argument('--auto', action='store_true', default=True,
                   help='recover scale + per-photo heading by chaining '
                        'feature matches along the flight path (default ON; '
                        'self-guards and falls back to --altitude)')
    p.add_argument('--no-auto', dest='auto', action='store_false',
                   help='skip vision calibration; use --altitude scale, north-up')
    p.add_argument('--altitude', type=float, default=30.0,
                   help='flight AGL (m) -> sets the scale (default 30)')
    p.add_argument('--hfov', type=float, default=DEFAULT_HFOV_DEG,
                   help='camera horizontal FOV deg for fallback (default 81)')
    p.add_argument('--gsd-cm', type=float, default=None,
                   help='force SOURCE ground sample distance (cm/px)')
    p.add_argument('--heading', type=float, default=None,
                   help='force camera heading (deg CW from north); '
                        '0 = images already north-up')
    p.add_argument('--out-gsd-cm', type=float, default=None,
                   help='output resolution cm/px (default: source GSD)')
    p.add_argument('--blend', choices=['center', 'feather'], default='center',
                   help="'center' = sharp winner-take-all seams (default); "
                        "'feather' = smooth averaged overlaps (can blur)")
    p.add_argument('--feather', type=float, default=0.08,
                   help='feather width as fraction of image (default 0.08)')
    p.add_argument('--max-mp', type=float, default=200.0,
                   help='max canvas megapixels before coarsening (default 200)')
    p.add_argument('--quality', type=int, default=92, help='JPEG quality')
    return p.parse_args(argv)


def main(argv=None) -> int:
    return run(parse_args(argv))


if __name__ == '__main__':
    sys.exit(main())
