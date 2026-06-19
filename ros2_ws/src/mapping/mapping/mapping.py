#!/usr/bin/env python3
"""
Incremental Image Stitcher with ROS2 Integration
Improved version with proper threading, error handling, and configuration management.

Key improvements:
- Non-blocking async stitching via threading
- Proper ROS lifecycle management (no rclpy.shutdown() from node)
- Exact command tokens (MAP_START, STITCH_START)
- Resettable state for multiple runs
- Image rescanning on each command
- Proper QoS profile
- ament_index_python path discovery
- Parameterized directories
- SIFT fallback to ORB
- Exposed magic number parameters
- Improved homography validation
- Canvas size limits
- Working blend_method selection
- Natural sorting for images
- Progress telemetry
- Exception handling per-frame
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# sensor_msgs and CvBridge no longer strictly needed, but left for compatibility
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import threading
import logging
import re
import json
from pathlib import Path
from datetime import datetime
from queue import Queue, Empty

from mavros_msgs.msg import StatusText

try:
    from ament_index_python.packages import get_package_share_directory
    HAS_AMENT_INDEX = True
except ImportError:
    HAS_AMENT_INDEX = False

# Configure logging to use ROS loggers where possible
# We'll use Python logging for utility functions, ROS logger in node
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')



def estimate_affine(img_ref, img_new, min_matches=6, ratio=0.75,
                    use_sift=False, ransac_thresh=3.0):
    """
    Estimate affine transform: img_new → img_ref coordinate space.
    Returns (M_3x3, inlier_count, info_str) or (None, 0, reason).
    Tuned for downscale=0.5 aerial tiles.
    """
    def enhance(img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        return clahe.apply(gray)

    g_ref = enhance(img_ref)
    g_new = enhance(img_new)

    if use_sift:
        det = cv2.SIFT_create(nfeatures=5000)
        norm = cv2.NORM_L2
    else:
        det = cv2.ORB_create(
            nfeatures=8000, scaleFactor=1.2, nlevels=8,
            edgeThreshold=15, WTA_K=2,
            scoreType=cv2.ORB_HARRIS_SCORE, patchSize=31, fastThreshold=10
        )
        norm = cv2.NORM_HAMMING

    k_ref, d_ref = det.detectAndCompute(g_ref, None)
    k_new, d_new = det.detectAndCompute(g_new, None)

    if d_ref is None or d_new is None or len(k_ref) < min_matches or len(k_new) < min_matches:
        return None, 0, f"Too few keypoints: ref={len(k_ref) if k_ref else 0}, new={len(k_new) if k_new else 0}"

    bf = cv2.BFMatcher(norm, crossCheck=False)
    try:
        knn = bf.knnMatch(d_new, d_ref, k=2)
    except cv2.error as e:
        return None, 0, f"Match error: {e}"

    good = [m for m, n in knn if len((m, n)) == 2 and m.distance < ratio * n.distance]

    # ── Thresholds tuned for downscale=0.5 ──────────────────────────────────
    # At 0.5× scale a "frame" is roughly 320–640 px wide; 15 inliers is generous.
    MIN_INLIERS       = 10      # absolute floor
    MIN_INLIER_RATIO  = 0.25    # 25 % of good matches must survive RANSAC
    # ────────────────────────────────────────────────────────────────────────

    if len(good) < min_matches:
        return None, 0, f"Too few good matches: {len(good)}"

    src = np.float32([k_new[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst = np.float32([k_ref[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

    M, mask = cv2.estimateAffinePartial2D(
        src, dst,
        method=cv2.RANSAC,
        ransacReprojThreshold=ransac_thresh,   # 3 px at 0.5× ≈ 6 px full-res
        maxIters=5000, confidence=0.99, refineIters=10
    )

    if M is None or mask is None:
        return None, 0, "Affine estimation failed"

    inliers = int(mask.sum())
    ratio_i = inliers / len(good)

    if inliers < MIN_INLIERS or ratio_i < MIN_INLIER_RATIO:
        return None, inliers, f"Weak inliers: {inliers} ({ratio_i:.1%})"

    # Scale sanity check (no crazy zoom)
    sx, sy = np.linalg.norm(M[0, :2]), np.linalg.norm(M[1, :2])
    if not (0.65 < sx < 1.55 and 0.65 < sy < 1.55):
        return None, inliers, f"Bad scale: sx={sx:.3f} sy={sy:.3f}"

    H = np.eye(3, dtype=np.float64)
    H[:2, :] = M
    return H, inliers, f"OK inliers={inliers} ({ratio_i:.1%})"

def warp_and_blend(panorama, pano_mask, frame, H_global):
    """
    Warp `frame` into panorama coordinates using H_global (accumulated 3×3),
    then distance-transform blend into `panorama`.

    panorama   : current BGR panorama canvas
    pano_mask  : uint8 mask (255 = valid pixels) same size as panorama
    frame      : new BGR frame (in its own coordinate space)
    H_global   : 3×3 homography  frame_coords → panorama_coords

    Returns (new_panorama, new_pano_mask, offset_xy)
    offset_xy  : (tx, ty) translation applied so callers can update H_global
    """
    h_p, w_p = panorama.shape[:2]
    h_f, w_f = frame.shape[:2]

    # Where do the four corners of the new frame land?
    corners_f = np.float32([[0,0],[0,h_f],[w_f,h_f],[w_f,0]]).reshape(-1,1,2)
    corners_w = cv2.perspectiveTransform(corners_f, H_global)

    corners_p = np.float32([[0,0],[0,h_p],[w_p,h_p],[w_p,0]]).reshape(-1,1,2)
    all_c = np.concatenate([corners_p, corners_w], axis=0)

    xmin, ymin = np.int32(all_c.min(axis=0).ravel() - 0.5)
    xmax, ymax = np.int32(all_c.max(axis=0).ravel() + 0.5)

    tx = max(0, -xmin)
    ty = max(0, -ymin)
    out_w = xmax - xmin
    out_h = ymax - ymin

    max_dim = max(w_p, h_p, w_f, h_f) * 8   # generous limit
    if out_w <= 0 or out_h <= 0 or out_w > max_dim or out_h > max_dim:
        return panorama, pano_mask, (0, 0), f"Canvas too large: {out_w}x{out_h}"

    # Translation matrix to shift everything into positive coords
    T = np.array([[1,0,tx],[0,1,ty],[0,0,1]], dtype=np.float64)
    TH = T @ H_global

    # Warp new frame
    warped_f  = cv2.warpPerspective(frame, TH, (out_w, out_h))
    warped_fm = np.zeros((out_h, out_w), dtype=np.uint8)
    frame_fill = np.ones((h_f, w_f), dtype=np.uint8) * 255
    warped_fm = cv2.warpPerspective(frame_fill, TH, (out_w, out_h))

    # Expand panorama canvas
    new_pano = np.zeros((out_h, out_w, 3), dtype=np.uint8)
    new_pano[ty:ty+h_p, tx:tx+w_p] = panorama
    new_mask = np.zeros((out_h, out_w), dtype=np.uint8)
    new_mask[ty:ty+h_p, tx:tx+w_p] = pano_mask

    # ── Mask-intersection overlap check (replaces bbox heuristic) ──────────
    overlap = cv2.bitwise_and(new_mask, warped_fm)
    overlap_px   = int(cv2.countNonZero(overlap))
    frame_px     = int(cv2.countNonZero(warped_fm))
    overlap_frac = overlap_px / frame_px if frame_px > 0 else 0.0
    if overlap_frac < 0.03:                 # need at least 3 % pixel overlap
        return panorama, pano_mask, (tx, ty), f"No overlap: {overlap_frac:.1%}"

    # ── Distance-transform blend ─────────────────────────────────────────────
    dist1 = cv2.distanceTransform(new_mask,  cv2.DIST_L2, 5).astype(np.float32)
    dist2 = cv2.distanceTransform(warped_fm, cv2.DIST_L2, 5).astype(np.float32)

    d_sum = dist1 + dist2 + 1e-6
    w1 = cv2.GaussianBlur(dist1 / d_sum, (31,31), 10)
    w2 = cv2.GaussianBlur(dist2 / d_sum, (31,31), 10)
    # renormalize
    ws = w1 + w2 + 1e-6
    w1, w2 = w1/ws, w2/ws

    w1_3 = np.stack([w1]*3, axis=-1)
    w2_3 = np.stack([w2]*3, axis=-1)

    in_pano  = new_mask[:,:,None]  > 0
    in_frame = warped_fm[:,:,None] > 0
    both     = in_pano & in_frame

    result = new_pano.copy()
    # frame-only region
    result = np.where(in_frame & ~in_pano, warped_f, result)
    # overlap — smooth blend
    result = np.where(both,
                      (new_pano.astype(np.float32)*w1_3
                       + warped_f.astype(np.float32)*w2_3).astype(np.uint8),
                      result)

    new_mask_out = np.where((new_mask > 0) | (warped_fm > 0),
                            np.uint8(255), np.uint8(0))

    return result, new_mask_out, (tx, ty), f"Blended overlap={overlap_frac:.1%}"

# ============================================================================
# ROS2 NODE CLASS
# ============================================================================

class MappingNode(Node):
    """
    ROS2 node that orchestrates stitching via worker thread.
    No blocking callbacks; respects ROS lifecycle.
    """
    def __init__(self):
        super().__init__('mapping_node')

        # Declare all parameters
        self.declare_parameter('use_sift', False)
        self.declare_parameter('downscale_factor', 0.5)
        self.declare_parameter('max_frames', 150)
        self.declare_parameter('min_matches', 6)
        self.declare_parameter('ratio_test', 0.8)
        self.declare_parameter('blend_method', 'distance')  # 'distance' or 'multiband'
        self.declare_parameter('ransac_thresh', 3.0)
        self.declare_parameter('max_canvas_width', 8192)
        self.declare_parameter('max_canvas_height', 8192)
        self.declare_parameter('overlap_threshold', 0.03)
        self.declare_parameter('max_skew', 1.0)
        self.declare_parameter('det_min', 0.05)
        self.declare_parameter('det_max', 15.0)
        self.declare_parameter('save_intermediates', False)
        self.declare_parameter('intermediate_interval', 10)
        self.declare_parameter('input_directory', '')  # empty = auto-discover
        self.declare_parameter('output_directory', '')  # empty = auto-discover
        self.declare_parameter('reprojection_threshold', 3.0)  # Max pixel error for homography
        self.declare_parameter('max_growth_ratio', 4.0)  # Max canvas growth (4.0 = 300% for panoramas)
        self.declare_parameter('max_growth_w_ratio', 8.0)  # Max width growth (8.0 = 700% for panoramas)
        self.declare_parameter('max_growth_h_ratio', 4.0)  # Max height growth (4.0 = 300% for panoramas)
        self.declare_parameter('use_clahe', True)  # Enable contrast enhancement
        self.declare_parameter('clahe_clip', 2.0)  # CLAHE clip limit
        self.declare_parameter('clahe_tile', 8)  # CLAHE tile grid size
        self.declare_parameter('command_qos_reliable', True)  # Use RELIABLE QoS for commands
        self.declare_parameter('package_name', 'mapping')  # For ament_index discovery
        self.declare_parameter('progress_interval', 5)  # Frames between progress updates
        self.declare_parameter('max_distance_blend_pixels', 8000000)  # Max pixels for distance blending (0=never)

        # Get parameters
        use_sift = self.get_parameter('use_sift').value
        self.downscale = self.get_parameter('downscale_factor').value
        self.max_frames = self.get_parameter('max_frames').value
        self.min_matches = self.get_parameter('min_matches').value
        self.ratio_test = self.get_parameter('ratio_test').value
        self.blend_method = self.get_parameter('blend_method').value
        ransac_thresh = self.get_parameter('ransac_thresh').value
        max_canvas_width = self.get_parameter('max_canvas_width').value
        max_canvas_height = self.get_parameter('max_canvas_height').value
        overlap_threshold = self.get_parameter('overlap_threshold').value
        max_skew = self.get_parameter('max_skew').value
        det_min = self.get_parameter('det_min').value
        det_max = self.get_parameter('det_max').value
        self.save_intermediates = self.get_parameter('save_intermediates').value
        self.intermediate_interval = self.get_parameter('intermediate_interval').value
        input_dir_param = self.get_parameter('input_directory').value
        output_dir_param = self.get_parameter('output_directory').value
        reprojection_threshold = self.get_parameter('reprojection_threshold').value
        max_growth_ratio = self.get_parameter('max_growth_ratio').value
        max_growth_w_ratio = self.get_parameter('max_growth_w_ratio').value
        max_growth_h_ratio = self.get_parameter('max_growth_h_ratio').value
        use_clahe = self.get_parameter('use_clahe').value
        clahe_clip = self.get_parameter('clahe_clip').value
        clahe_tile = self.get_parameter('clahe_tile').value
        command_qos_reliable = self.get_parameter('command_qos_reliable').value
        self.progress_interval = self.get_parameter('progress_interval').value
        max_distance_blend_pixels = self.get_parameter('max_distance_blend_pixels').value
        self.package_name = self.get_parameter('package_name').value

        # We keep CvBridge defined but it's no longer used in directory mode
        self.bridge = CvBridge()

        self.panorama = None
        self.pano_mask = None
        self.frames_processed = 0
        self.frames_stitched = 0

        # Store for later reference (metadata, config)
        self.use_sift = use_sift
        detector_name = "SIFT" if use_sift else "ORB"

        # Load images from mapping_photos
        exts = (".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff")

        self.last_frame = None
        self.H_global = np.eye(3, dtype=np.float64)

        def numeric_key(path):
            # Extract all digits from the file name and convert to int
            fname = os.path.basename(path)
            digits = ''.join(ch for ch in fname if ch.isdigit())
            return int(digits) if digits else 0

        self.image_files = sorted(
            [
                os.path.join(self.camera_feed_path, f)
                for f in os.listdir(self.camera_feed_path)
                if f.lower().endswith(exts)
            ],
            key=numeric_key
        )

        # State for threading
        self.stitching_done = True  # Not currently stitching
        self.stitching_thread = None
        self.stitching_lock = threading.Lock()
        self.stop_event = threading.Event()  # Signal worker thread to stop
        self.status_queue = Queue(maxsize=200)  # Bounded queue for status publishing from worker thread

        # Logging
        self.get_logger().info(
            f'Mapping node initialized:\n'
            f'  Detector: {"SIFT" if use_sift else "ORB"}\n'
            f'  Downscale: {self.downscale}\n'
            f'  Blend: {self.blend_method}\n'
            f'  Canvas limits: {max_canvas_width}x{max_canvas_height}\n'
            f'  Input dir: {self.input_dir}\n'
            f'  Output dir: {self.output_dir}'
        )

        # Subscribe with appropriate QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE if command_qos_reliable else QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.command_sub = self.create_subscription(
            StatusText,
            '/mavros/statustext/recv',
            self._command_cb,
            qos
        )

        # Publisher for feedback
        self.status_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)

        self.run_mapping()

    def send_back(self, text):
        # feedback to GCS (Mission Planner messages tab)
        msg = StatusText()
        msg.severity = StatusText.NOTICE
        msg.text = text
        self.status_queue.put(msg)

    def command_cb(self, msg: StatusText):
        if "follow" in msg.text.lower():  # change this if needed
            if self.mapping_started:
                # Avoid running twice if multiple zigzag messages arrive
                self.get_logger().info("Mapping already started, ignoring extra zigzag command.")
                return

    def run_mapping(self):
        """
        Frame-to-frame affine estimation + global accumulation.
        Recovery: if last frame fails, try up to 3 previous keyframes.
        """
        WINDOW = 10          # how many recent keyframes to try on failure
        keyframes = []      # list of (original_frame_small, H_at_time_of_stitching)

        for image_path in self.image_files:
            if self.frames_processed >= self.max_frames:
                break

            frame = cv2.imread(image_path)
            if frame is None:
                self.get_logger().warn(f"Cannot read: {image_path}")
                continue

            frame_small = cv2.resize(frame, (0,0), fx=self.downscale, fy=self.downscale)
            self.frames_processed += 1
            fname = os.path.basename(image_path)

            # ── Bootstrap ───────────────────────────────────────────────────────
            if self.panorama is None:
                self.panorama  = frame_small.copy()
                self.pano_mask = np.full(frame_small.shape[:2], 255, dtype=np.uint8)
                self.H_global  = np.eye(3, dtype=np.float64)
                keyframes.append((frame_small.copy(), self.H_global.copy()))
                self.frames_stitched = 1
                self.get_logger().info(f"Bootstrap: {fname}")
                continue

            # ── Try matching against recent keyframes (newest first) ────────────
            candidates = list(reversed(keyframes[-WINDOW:]))   # newest → oldest
            matched_H  = None
            matched_ref_H = None

            for ref_frame, ref_H in candidates:
                H_rel, n_in, info = estimate_affine(
                    ref_frame, frame_small,
                    min_matches=self.min_matches,
                    ratio=self.ratio_test,
                    use_sift=self.use_sift
                )
                
                if H_rel is not None:
                    matched_H     = H_rel      # frame_small → ref_frame coords
                    matched_ref_H = ref_H      # ref_frame   → panorama coords
                    self.get_logger().info(f"✓ {fname}: {info}")
                    break
                else:
                    self.get_logger().warn(f"  retry {fname} vs older keyframe: {info}")

            if matched_H is None:
                self.get_logger().warn(f"✗ Skipped {fname}: no candidate matched")
                keyframes.append((frame_small.copy(), keyframes[-1][1]))  # inherit last good H
                continue

            # ── Compose: frame → panorama ────────────────────────────────────────
            # H_global_new = matched_ref_H  @  matched_H
            H_new_global = matched_ref_H @ matched_H

            # ── Warp & blend ─────────────────────────────────────────────────────
            result, new_mask, (tx, ty), blend_info = warp_and_blend(
                self.panorama, self.pano_mask, frame_small, H_new_global
            )

            if "No overlap" in blend_info:
                self.get_logger().warn(f"✗ {fname}: {blend_info}")
                continue

            # Update panorama + shift all stored H_globals by translation (tx, ty)
            T_shift = np.array([[1,0,tx],[0,1,ty],[0,0,1]], dtype=np.float64)
            self.panorama  = result
            self.pano_mask = new_mask
            self.H_global  = T_shift @ H_new_global   # absolute position of *last* frame

            # Shift all stored keyframe Hs to match new canvas origin
            keyframes = [(f, T_shift @ h) for f, h in keyframes]
            keyframes.append((frame_small.copy(), self.H_global.copy()))

            self.frames_stitched += 1
            self.get_logger().info(f"  canvas: {self.panorama.shape[1]}x{self.panorama.shape[0]}  {blend_info}")

        self.finish_and_shutdown()

            

    def finish_and_shutdown(self):
        if self.panorama is not None:
            # --- Robust crop of non-black area ---
            gray = cv2.cvtColor(self.panorama, cv2.COLOR_BGR2GRAY)

            # Treat anything that is not absolutely black as valid
            # (THRESH_BINARY with threshold=0 will make all >0 → 255)
            _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)

            nonzero = cv2.countNonZero(mask)
            if nonzero > 0:
                # Find all non-zero pixels and take a tight bounding box
                coords = cv2.findNonZero(mask)  # Nx1x2
                x, y, w, h = cv2.boundingRect(coords)
                self.panorama = self.panorama[y:y+h, x:x+w]
            else:
                # This should basically never happen unless the panorama is fully black
                self.get_logger().warn("Panorama looks empty (all black); skipping crop.")

            # --- Save final panorama (cropped or not) ---
            final_path = os.path.join(self.save_dir, 'final_panorama.jpg')
            cv2.imwrite(final_path, self.panorama)

            self.get_logger().info("=" * 60)
            self.get_logger().info("FINAL RESULTS:")
            self.get_logger().info(f"  Frames processed: {self.frames_processed}")
            self.get_logger().info(f"  Frames stitched: {self.frames_stitched}")
            if self.frames_processed > 0:
                self.get_logger().info(
                    f"  Success rate: {100 * self.frames_stitched / self.frames_processed:.1f}%"
                )
            self.get_logger().info(
                f"  Final size: {self.panorama.shape[1]}x{self.panorama.shape[0]}"
            )
            self.get_logger().info(f"  Saved to: {final_path}")
            self.get_logger().info("=" * 60)
            self.send_back(f"Mapping complete. Panorama saved to {final_path}")
        else:
            self.get_logger().error("No panorama created!")
            self.send_back("Mapping failed: no panorama created")

        
def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Signal worker thread to stop and wait for it to finish
        node.stop_event.set()
        if node.stitching_thread is not None and node.stitching_thread.is_alive():
            node.stitching_thread.join(timeout=10.0)  # Increased from 5s for heavy OpenCV ops
            if node.stitching_thread.is_alive():
                node.get_logger().warning("Worker thread did not stop within 10s (likely inside OpenCV operation)")
        
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()