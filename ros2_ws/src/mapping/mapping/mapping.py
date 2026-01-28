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


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def natural_sort(paths):
    """Natural sort to handle 1.jpg before 10.jpg (unlike lexicographic sort)."""
    def natural_key(text):
        return [int(c) if c.isdigit() else c.lower() for c in re.split(r'(\d+)', text)]
    return sorted(paths, key=lambda x: natural_key(os.path.basename(x)))


def try_sift_create(nfeatures=5000):
    """Try to create SIFT detector; return (detector, success) or (None, False)."""
    try:
        return cv2.SIFT_create(nfeatures=nfeatures), True
    except (AttributeError, cv2.error):
        logging.warning("SIFT not available, will fall back to ORB")
        return None, False


def get_project_directory(package_name: str = 'mapping'):
    """
    Locate the ros2_ws directory using ament_index if available,
    otherwise walk up from current file.
    Args:
        package_name: Name of ROS package for ament_index discovery
    """
    if HAS_AMENT_INDEX:
        try:
            share_dir = get_package_share_directory(package_name)
            parts = Path(share_dir).parts
            if 'install' in parts:
                install_idx = parts.index('install')
                ros2_ws = os.path.sep.join(parts[:install_idx])
                return ros2_ws
        except Exception as e:
            logging.debug(f"ament_index lookup failed for '{package_name}': {e}")
    
    # Fallback: walk up from current file
    current_file = os.path.abspath(__file__)
    current_dir = os.path.dirname(current_file)
    search_dir = current_dir

    for _ in range(10):
        if os.path.exists(os.path.join(search_dir, "install")) and os.path.exists(os.path.join(search_dir, "src")):
            return search_dir
        search_dir = os.path.dirname(search_dir)
        if search_dir == "/":
            break

    return None


# ============================================================================
# CORE STITCHER CLASS (Pure, testable image processing)
# ============================================================================

class PanoramaStitcher:
    """
    Pure image stitching logic, independent of ROS.
    All CV operations are here; ROS node just orchestrates.
    """
    def __init__(self, use_sift=False, min_matches=8, ratio_test=0.75, 
                 ransac_thresh=3.0, max_canvas_width=8192, max_canvas_height=8192,
                 blend_method='distance', overlap_threshold=0.03, 
                 max_skew=1.0, det_min=0.05, det_max=15.0, 
                 reprojection_threshold=3.0, max_growth_ratio=4.0,
                 max_growth_w_ratio=8.0, max_growth_h_ratio=4.0,
                 use_clahe=True, clahe_clip=2.0, clahe_tile=8, 
                 max_distance_blend_pixels=8000000, debug=False):
        """
        Args:
            use_sift: Whether to use SIFT (else ORB)
            min_matches: Minimum required good matches
            ratio_test: Lowe's ratio test threshold
            ransac_thresh: RANSAC threshold for homography
            max_canvas_width/height: Maximum panorama dimensions
            blend_method: 'distance' or 'multiband'
            overlap_threshold: Minimum required overlap ratio
            max_skew: Maximum allowed skew in homography
            det_min/det_max: Bounds for determinant check
            reprojection_threshold: Max pixel error for valid homography
            max_growth_ratio: Max area growth (e.g., 4.0 = 300% area increase max for panoramas)
            max_growth_w_ratio: Max width growth (e.g., 8.0 = 700% width increase max for panoramas)
            max_growth_h_ratio: Max height growth (e.g., 4.0 = 300% height increase max for panoramas)
            use_clahe: Enable CLAHE contrast enhancement
            clahe_clip: CLAHE clip limit
            clahe_tile: CLAHE tile grid size
            max_distance_blend_pixels: Max canvas pixels for distance blending (0=never, >8M=always)
            debug: Enable debug logging
        """
        self.use_sift = use_sift
        self.min_matches = min_matches
        self.ratio_test = ratio_test
        self.ransac_thresh = ransac_thresh
        self.max_canvas_width = max_canvas_width
        self.max_canvas_height = max_canvas_height
        self.blend_method = blend_method
        self.overlap_threshold = overlap_threshold
        self.max_skew = max_skew
        self.det_min = det_min
        self.det_max = det_max
        self.reprojection_threshold = reprojection_threshold
        self.max_growth_ratio = max_growth_ratio
        self.max_growth_w_ratio = max_growth_w_ratio
        self.max_growth_h_ratio = max_growth_h_ratio
        self.use_clahe = use_clahe
        self.clahe_clip = clahe_clip
        self.clahe_tile = clahe_tile
        self.max_distance_blend_pixels = max_distance_blend_pixels
        self.debug = debug
        
        self.detector = None
        self.detector_available = False
        self._init_detector()
        
    def _init_detector(self):
        """Initialize feature detector with proper error handling."""
        if self.use_sift:
            self.detector, self.detector_available = try_sift_create(nfeatures=5000)
            if not self.detector_available:
                logging.info("SIFT unavailable, using ORB instead")
                self.use_sift = False
        
        if not self.use_sift:
            self.detector = cv2.ORB_create(
                nfeatures=8000, scaleFactor=1.2, nlevels=8,
                edgeThreshold=15, firstLevel=0, WTA_K=2,
                scoreType=cv2.ORB_HARRIS_SCORE, patchSize=31, fastThreshold=10
            )
            self.detector_available = True

    def stitch_pair(self, img1, img2, old_size=None):
        """
        Stitch two images together.
        Args:
            img1: Reference image (panorama)
            img2: New frame to add
            old_size: (width, height) tuple of current panorama for growth checking
        Returns:
            (result_image, info_dict) tuple
            info_dict always contains 'success' key; other keys vary
        """
        try:
            return self._stitch_pair_impl(img1, img2, old_size)
        except Exception as e:
            error_info = {
                'success': False,
                'error': str(e),
                'exception_type': type(e).__name__
            }
            logging.error(f"Stitching exception: {e}", exc_info=self.debug)
            return None, error_info

    def _stitch_pair_impl(self, img1, img2, old_size=None):
        """Internal stitching implementation with full error handling."""
        # Feature detection with contrast enhancement
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY) if len(img1.shape) == 3 else img1
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY) if len(img2.shape) == 3 else img2

        if self.use_clahe:
            clahe = cv2.createCLAHE(clipLimit=self.clahe_clip, tileGridSize=(self.clahe_tile, self.clahe_tile))
            gray1 = clahe.apply(gray1)
            gray2 = clahe.apply(gray2)

        # Detect and compute features
        k1, d1 = self.detector.detectAndCompute(gray1, None)
        k2, d2 = self.detector.detectAndCompute(gray2, None)

        error_info = {
            'success': False,
            'k1_count': len(k1) if k1 else 0,
            'k2_count': len(k2) if k2 else 0,
        }

        if d1 is None or d2 is None:
            error_info['error'] = f"No descriptors found"
            return None, error_info

        if len(k1) < self.min_matches or len(k2) < self.min_matches:
            error_info['error'] = f"Insufficient keypoints"
            return None, error_info

        # Matching with ratio test
        norm_type = cv2.NORM_L2 if self.use_sift else cv2.NORM_HAMMING
        bf = cv2.BFMatcher(norm_type, crossCheck=False)
        
        try:
            knn = bf.knnMatch(d2, d1, k=2)
        except cv2.error as e:
            error_info['error'] = f"Matching failed: {e}"
            return None, error_info

        good = []
        for match_pair in knn:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < self.ratio_test * n.distance:
                    good.append(m)

        if len(good) < self.min_matches:
            error_info['error'] = f"Insufficient good matches: {len(good)}/{self.min_matches}"
            return None, error_info

        # Extract matched points
        src_pts = np.float32([k2[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([k1[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        # Homography estimation
        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, self.ransac_thresh, maxIters=5000)

        if H is None:
            error_info['error'] = "Homography estimation failed"
            return None, error_info

        matches_mask = mask.ravel().tolist()
        inlier_count = sum(matches_mask)
        inlier_ratio = inlier_count / len(good)
        min_inliers = max(6, self.min_matches // 2)

        if inlier_count < min_inliers:
            error_info['error'] = f"Too few inliers: {inlier_count}/{min_inliers}"
            return None, error_info

        # Validate homography (improved)
        try:
            # Check scale
            det = abs(np.linalg.det(H[:2, :2]))
            if det < self.det_min or det > self.det_max:
                error_info['error'] = f"Invalid scale: det={det:.3f}"
                return None, error_info

            # Check skew (ignores reflection via abs)
            if abs(H[0, 1]) > self.max_skew or abs(H[1, 0]) > self.max_skew:
                error_info['error'] = f"Excessive skew"
                return None, error_info
            
            # Check condition number (homography stability)
            cond_number = np.linalg.cond(H)
            if cond_number > 100:
                logging.warning(f"High condition number: {cond_number}")
            
            # Compute reprojection error on inliers (most meaningful metric)
            inlier_pts_src = src_pts[mask.ravel() == 1]
            inlier_pts_dst = dst_pts[mask.ravel() == 1]
            
            if len(inlier_pts_src) > 0:
                # Project source points through homography
                proj_pts = cv2.perspectiveTransform(inlier_pts_src, H)
                # Compute euclidean distances
                errors = np.sqrt(np.sum((proj_pts - inlier_pts_dst) ** 2, axis=2)).ravel()
                median_error = np.median(errors)
                mean_error = np.mean(errors)
                
                if median_error > self.reprojection_threshold:
                    error_info['error'] = f"High reprojection error: median={median_error:.2f}px, mean={mean_error:.2f}px"
                    return None, error_info
                
                # Check inlier spread: must cover at least 10% of image width/height
                # (prevents clusters where all inliers are in a tiny corner)
                h2_tmp, w2_tmp = img2.shape[:2]  # Get image dimensions from img2
                inlier_x = inlier_pts_src[:, 0, 0]
                inlier_y = inlier_pts_src[:, 0, 1]
                x_spread = np.max(inlier_x) - np.min(inlier_x)
                y_spread = np.max(inlier_y) - np.min(inlier_y)
                min_x_spread = w2_tmp * 0.10  # 10% of image width
                min_y_spread = h2_tmp * 0.10  # 10% of image height
                
                if x_spread < min_x_spread or y_spread < min_y_spread:
                    error_info['error'] = f"Inliers too clustered: x_spread={x_spread:.0f}px (need {min_x_spread:.0f}), y_spread={y_spread:.0f}px (need {min_y_spread:.0f})"
                    return None, error_info
                
        except Exception as e:
            error_info['error'] = f"Homography validation error: {e}"
            return None, error_info

        # Canvas computation
        h1, w1 = img1.shape[:2]
        h2, w2 = img2.shape[:2]

        corners_img2 = np.float32([[0, 0], [0, h2], [w2, h2], [w2, 0]]).reshape(-1, 1, 2)
        corners_img2_transformed = cv2.perspectiveTransform(corners_img2, H)
        corners_img1 = np.float32([[0, 0], [0, h1], [w1, h1], [w1, 0]]).reshape(-1, 1, 2)

        all_corners = np.concatenate([corners_img1, corners_img2_transformed], axis=0)

        [xmin, ymin] = np.int32(all_corners.min(axis=0).ravel() - 0.5)
        [xmax, ymax] = np.int32(all_corners.max(axis=0).ravel() + 0.5)

        tx = -xmin if xmin < 0 else 0
        ty = -ymin if ymin < 0 else 0

        T = np.array([[1, 0, tx], [0, 1, ty], [0, 0, 1]], dtype=np.float64)

        out_w = xmax - xmin
        out_h = ymax - ymin

        if out_w <= 0 or out_h <= 0 or out_w > self.max_canvas_width or out_h > self.max_canvas_height:
            error_info['error'] = f"Invalid output size: {out_w}x{out_h}"
            return None, error_info

        # Check canvas growth ratio (img1 is the current panorama)
        if old_size is not None:
            old_w, old_h = old_size
            old_area = old_w * old_h
            new_area = out_w * out_h
            growth_ratio = new_area / (old_area + 1e-6)
            w_ratio = out_w / (old_w + 1e-6)
            h_ratio = out_h / (old_h + 1e-6)
            
            if growth_ratio > self.max_growth_ratio:
                error_info['error'] = f"Canvas area growth too large: {growth_ratio:.2f}x (limit: {self.max_growth_ratio}x)"
                return None, error_info
            if w_ratio > self.max_growth_w_ratio:
                error_info['error'] = f"Canvas width growth too large: {w_ratio:.2f}x (limit: {self.max_growth_w_ratio}x)"
                return None, error_info
            if h_ratio > self.max_growth_h_ratio:
                error_info['error'] = f"Canvas height growth too large: {h_ratio:.2f}x (limit: {self.max_growth_h_ratio}x)"
                return None, error_info
        elif img1 is not None:
            # Fallback: use img1 as the current panorama size
            old_w, old_h = img1.shape[1], img1.shape[0]
            old_area = old_w * old_h
            new_area = out_w * out_h
            growth_ratio = new_area / (old_area + 1e-6)
            w_ratio = out_w / (old_w + 1e-6)
            h_ratio = out_h / (old_h + 1e-6)
            
            if growth_ratio > self.max_growth_ratio:
                error_info['error'] = f"Canvas area growth too large: {growth_ratio:.2f}x (limit: {self.max_growth_ratio}x)"
                return None, error_info
            if w_ratio > self.max_growth_w_ratio:
                error_info['error'] = f"Canvas width growth too large: {w_ratio:.2f}x (limit: {self.max_growth_w_ratio}x)"
                return None, error_info
            if h_ratio > self.max_growth_h_ratio:
                error_info['error'] = f"Canvas height growth too large: {h_ratio:.2f}x (limit: {self.max_growth_h_ratio}x)"
                return None, error_info

        # Warping
        warped_img2 = cv2.warpPerspective(img2, T @ H, (out_w, out_h))

        result = np.zeros((out_h, out_w, 3), dtype=np.uint8)
        result[ty:ty + h1, tx:tx + w1] = img1

        # Masks - use warped mask for img2 (more accurate than fillConvexPoly)
        mask_img1 = np.zeros((out_h, out_w), dtype=np.uint8)
        mask_img1[ty:ty + h1, tx:tx + w1] = 255

        # Create binary mask of img2 and warp it (more accurate for geometry)
        mask_img2_src = np.ones((h2, w2), dtype=np.uint8) * 255
        mask_img2 = cv2.warpPerspective(mask_img2_src, T @ H, (out_w, out_h))
        # Ensure mask is strictly binary (warp can create gray values from interpolation)
        mask_img2 = (mask_img2 > 0).astype(np.uint8) * 255

        # Overlap check using mask-based pixel counts (more accurate than bounding box)
        overlap_mask = cv2.bitwise_and(mask_img1, mask_img2)
        overlap_pixels = cv2.countNonZero(overlap_mask)
        img2_pixels = cv2.countNonZero(mask_img2)

        if img2_pixels == 0:
            error_info['error'] = "No valid img2 pixels after warping"
            return None, error_info

        overlap_ratio = overlap_pixels / img2_pixels if img2_pixels > 0 else 0

        if overlap_ratio < self.overlap_threshold:
            error_info['error'] = f"Insufficient overlap: {overlap_ratio:.1%}"
            return None, error_info

        # Blending
        overlap_mask = cv2.bitwise_and(mask_img1, mask_img2)

        if cv2.countNonZero(overlap_mask) > 0:
            if self.blend_method == 'multiband':
                result = self._multiband_blend(result, warped_img2, mask_img1, mask_img2)
            else:
                result = self._distance_blend(result, warped_img2, mask_img1, mask_img2, overlap_mask)
        else:
            result = np.where(mask_img2[:, :, np.newaxis] > 0, warped_img2, result)

        # Crop black borders using mask-based approach
        result = self._crop_to_valid_region(result, mask_img1, mask_img2)

        success_info = {
            'success': True,
            'inliers': inlier_count,
            'inlier_ratio': inlier_ratio,
            'overlap': overlap_ratio,
            'output_size': (out_w, out_h),
            'final_size': (result.shape[1], result.shape[0])
        }
        
        return result, success_info

    def _distance_blend(self, result, warped_img2, mask_img1, mask_img2, overlap_mask):
        """Distance transform blending for seamless seams.
        Skip blending if panorama exceeds max_distance_blend_pixels threshold (expensive operation).
        """
        # Check if blending should be skipped for performance
        canvas_pixels = result.shape[0] * result.shape[1]
        if self.max_distance_blend_pixels > 0 and canvas_pixels > self.max_distance_blend_pixels:
            # Fall back to simple overlay (faster)
            result = np.where(
                np.stack([overlap_mask] * 3, axis=-1) > 0,
                result,  # Keep existing content
                np.where(mask_img2[:, :, np.newaxis] > 0, warped_img2, result)
            )
            return result
        
        dist1 = cv2.distanceTransform(mask_img1, cv2.DIST_L2, 5)
        dist2 = cv2.distanceTransform(mask_img2, cv2.DIST_L2, 5)

        dist1_norm = dist1 / (dist1 + dist2 + 1e-6)
        dist2_norm = dist2 / (dist1 + dist2 + 1e-6)

        dist1_norm = cv2.GaussianBlur(dist1_norm, (31, 31), 10)
        dist2_norm = cv2.GaussianBlur(dist2_norm, (31, 31), 10)

        dist_sum = dist1_norm + dist2_norm
        dist_sum = np.maximum(dist_sum, 1e-6)
        dist1_norm = dist1_norm / dist_sum
        dist2_norm = dist2_norm / dist_sum

        dist1_3c = np.stack([dist1_norm] * 3, axis=-1)
        dist2_3c = np.stack([dist2_norm] * 3, axis=-1)

        overlap_3c = np.stack([overlap_mask] * 3, axis=-1) > 0
        result = np.where(
            overlap_3c,
            (result.astype(float) * dist1_3c + warped_img2.astype(float) * dist2_3c).astype(np.uint8),
            np.where(mask_img2[:, :, np.newaxis] > 0, warped_img2, result)
        )
        
        return result

    def _multiband_blend(self, img1, img2, mask1, mask2, levels=3):
        """Multi-band blending (simplified for performance)."""
        if img1.shape[0] > 4000 or img1.shape[1] > 4000:
            levels = 2
        
        G1, G2, GM1, GM2 = img1.copy(), img2.copy(), mask1.copy(), mask2.copy()
        gp1, gp2, gpm1, gpm2 = [G1], [G2], [GM1], [GM2]

        for _ in range(levels):
            G1 = cv2.pyrDown(G1)
            G2 = cv2.pyrDown(G2)
            GM1 = cv2.pyrDown(GM1)
            GM2 = cv2.pyrDown(GM2)
            gp1.append(G1)
            gp2.append(G2)
            gpm1.append(GM1)
            gpm2.append(GM2)

        lp1 = [gp1[levels - 1]]
        lp2 = [gp2[levels - 1]]

        for i in range(levels - 1, 0, -1):
            size = (gp1[i - 1].shape[1], gp1[i - 1].shape[0])
            L1 = cv2.subtract(gp1[i - 1], cv2.pyrUp(gp1[i], dstsize=size))
            L2 = cv2.subtract(gp2[i - 1], cv2.pyrUp(gp2[i], dstsize=size))
            lp1.append(L1)
            lp2.append(L2)

        LS = []
        for l1, l2, m1, m2 in zip(lp1, lp2, gpm1[::-1], gpm2[::-1]):
            m1 = m1.astype(float) / 255.0
            m2 = m2.astype(float) / 255.0
            if len(m1.shape) == 2:
                m1 = np.stack([m1] * 3, axis=-1)
                m2 = np.stack([m2] * 3, axis=-1)
            ls = l1.astype(float) * m1 + l2.astype(float) * m2
            LS.append(ls)

        result = LS[0]
        for i in range(1, levels):
            size = (LS[i].shape[1], LS[i].shape[0])
            result = cv2.add(cv2.pyrUp(result, dstsize=size), LS[i])

        return np.clip(result, 0, 255).astype(np.uint8)

    def _crop_to_valid_region(self, image, mask_img1, mask_img2):
        """Crop based on valid regions from masks rather than pixel intensity."""
        combined_mask = cv2.bitwise_or(mask_img1, mask_img2)
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return image
        
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)

        margin = 2
        x = max(0, x + margin)
        y = max(0, y + margin)
        w = min(w - 2 * margin, image.shape[1] - x)
        h = min(h - 2 * margin, image.shape[0] - y)

        if w > 0 and h > 0:
            return image[y:y + h, x:x + w]
        return image


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
        self.declare_parameter('min_matches', 8)
        self.declare_parameter('ratio_test', 0.75)
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

        # Setup directories
        self._setup_directories(input_dir_param, output_dir_param, self.package_name)

        # Store for later reference (metadata, config)
        self.use_sift = use_sift
        
        # Initialize stitcher with all parameters
        self.stitcher = PanoramaStitcher(
            use_sift=use_sift,
            min_matches=self.min_matches,
            ratio_test=self.ratio_test,
            ransac_thresh=ransac_thresh,
            max_canvas_width=max_canvas_width,
            max_canvas_height=max_canvas_height,
            blend_method=self.blend_method,
            overlap_threshold=overlap_threshold,
            max_skew=max_skew,
            det_min=det_min,
            det_max=det_max,
            reprojection_threshold=reprojection_threshold,
            max_growth_ratio=max_growth_ratio,
            max_growth_w_ratio=max_growth_w_ratio,
            max_growth_h_ratio=max_growth_h_ratio,
            use_clahe=use_clahe,
            clahe_clip=clahe_clip,
            clahe_tile=clahe_tile,
            max_distance_blend_pixels=max_distance_blend_pixels
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

        # Timer for safe publishing from worker thread via queue
        self.create_timer(0.1, self._publish_from_queue)
        
        # Auto-start mapping if images exist in input directory
        self._auto_start_mapping()
    
    def _publish_from_queue(self):
        """Timer callback: publishes queued status messages (runs in ROS executor thread)."""
        try:
            # Publish up to 10 messages per tick to avoid starving other callbacks
            for _ in range(10):
                try:
                    msg = self.status_queue.get_nowait()
                    self.status_pub.publish(msg)
                except Empty:
                    break  # Queue is empty, done
        except Exception as e:
            self.get_logger().error(f"Error publishing from queue: {e}")

    def _auto_start_mapping(self):
        """Auto-start mapping if images exist in input directory."""
        try:
            if os.path.exists(self.input_dir):
                exts = (".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff")
                images = [f for f in os.listdir(self.input_dir) 
                         if os.path.isfile(os.path.join(self.input_dir, f)) 
                         and f.lower().endswith(exts)]
                
                if images:
                    self.get_logger().info(f"Found {len(images)} images in input directory. Auto-starting mapping...")
                    self._send_status(f"Auto-starting mapping with {len(images)} images")
                    
                    # Start stitching in worker thread
                    with self.stitching_lock:
                        self.stitching_done = False
                        self.stop_event.clear()
                        self.stitching_thread = threading.Thread(target=self._do_stitching)
                        self.stitching_thread.daemon = False
                        self.stitching_thread.start()
                else:
                    self.get_logger().info(f"No images found in {self.input_dir}. Waiting for MAP_START command...")
            else:
                self.get_logger().warning(f"Input directory does not exist: {self.input_dir}")
        except Exception as e:
            self.get_logger().warning(f"Error in auto-start check: {e}")

    def _setup_directories(self, input_dir_param, output_dir_param, package_name):
        """Setup input/output directories with fallback discovery."""
        ros2_ws = get_project_directory(package_name)

        if not input_dir_param:
            if ros2_ws:
                self.input_dir = os.path.join(ros2_ws, "src", "video_cam", "mapping_photos")
            else:
                self.input_dir = "/astra/ros2_ws/src/video_cam/mapping_photos"
        else:
            self.input_dir = input_dir_param

        if not output_dir_param:
            if ros2_ws:
                self.output_dir = os.path.join(ros2_ws, "src", "video_cam", "mapping_results")
            else:
                self.output_dir = "/astra/ros2_ws/src/video_cam/mapping_results"
        else:
            self.output_dir = output_dir_param

        os.makedirs(self.input_dir, exist_ok=True)
        os.makedirs(self.output_dir, exist_ok=True)

    def _send_status(self, text, severity=6):
        """Queue status message for thread-safe publishing."""
        msg = StatusText()
        msg.severity = severity
        msg.text = text
        self.status_queue.put(msg)

    def _command_cb(self, msg: StatusText):
        """
        Callback for incoming status messages.
        Non-blocking: starts a worker thread if command is recognized.
        """
        text = msg.text.strip().upper()

        # Exact command matching (not fuzzy substring matching)
        if text == "MAP_START" or text == "STITCH_START":
            with self.stitching_lock:
                if self.stitching_done:
                    self.stitching_done = False
                    self.stop_event.clear()  # Reset stop signal for new run
                    self.get_logger().info(f"Command received: {text}")
                    self._send_status(f"Mapping started ({text})")
                    
                    # Start stitching in worker thread
                    self.stitching_thread = threading.Thread(target=self._do_stitching)
                    self.stitching_thread.daemon = False
                    self.stitching_thread.start()
                else:
                    self.get_logger().info("Stitching already in progress, ignoring command")
                    self._send_status("Stitching in progress, command ignored")
        
        elif text == "MAP_STOP" or text == "STITCH_STOP":
            with self.stitching_lock:
                if not self.stitching_done:
                    self.get_logger().info(f"Command received: {text}")
                    self._send_status(f"Stopping mapping ({text})...")
                    self.stop_event.set()  # Signal worker thread to stop
                else:
                    self.get_logger().info("No stitching in progress, ignoring stop command")
        
        else:
            self.get_logger().debug(f"Unknown command (expected MAP_START, STITCH_START, MAP_STOP, STITCH_STOP): {text}")

    def _do_stitching(self):
        """
        Worker thread: perform stitching without blocking ROS spin.
        """
        try:
            self.get_logger().info("Stitching thread started")

            # Rescan image directory (allows new images between runs)
            exts = (".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff")
            all_images = [
                os.path.join(self.input_dir, f)
                for f in os.listdir(self.input_dir)
                if f.lower().endswith(exts)
            ]
            all_images = natural_sort(all_images)

            if not all_images:
                self.get_logger().error("No images found in input directory")
                self._send_status("Error: No images found")
                with self.stitching_lock:
                    self.stitching_done = True
                return

            self.get_logger().info(f"Found {len(all_images)} images to stitch")

            # Reset panorama for fresh run
            panorama = None
            frames_processed = 0
            frames_stitched = 0

            for idx, image_path in enumerate(all_images):
                # Check for shutdown signal
                if self.stop_event.is_set():
                    self.get_logger().info("Shutdown signal received, stopping stitching")
                    break

                if frames_processed >= self.max_frames:
                    self.get_logger().info(f"Reached max frames ({self.max_frames})")
                    break

                try:
                    frame = cv2.imread(image_path)
                    if frame is None:
                        self.get_logger().warning(f"Failed to read: {os.path.basename(image_path)}")
                        frames_processed += 1
                        continue

                    frame_small = cv2.resize(frame, (0, 0), fx=self.downscale, fy=self.downscale, interpolation=cv2.INTER_AREA)

                    if panorama is None:
                        panorama = frame_small
                        frames_stitched = 1
                        self.get_logger().info("Initialized panorama with first frame")
                    else:
                        # Pass old panorama size for growth ratio validation
                        old_size = (panorama.shape[1], panorama.shape[0])
                        result, info = self.stitcher.stitch_pair(panorama, frame_small, old_size=old_size)

                        if result is not None:
                            panorama = result
                            frames_stitched += 1
                            inlier_ratio = info.get('inlier_ratio', 0)
                            overlap = info.get('overlap', 0)
                            self.get_logger().info(
                                f"✓ Frame {frames_processed + 1}: inliers={inlier_ratio:.1%}, overlap={overlap:.1%}"
                            )
                            
                            # Periodic progress report
                            if frames_stitched % self.progress_interval == 0:
                                self._send_status(f"Stitching: {frames_stitched} frames done")
                            
                            # Save intermediate panorama
                            if self.save_intermediates and frames_stitched % self.intermediate_interval == 0:
                                inter_path = os.path.join(self.output_dir, f'intermediate_{frames_stitched:03d}.jpg')
                                cv2.imwrite(inter_path, panorama)
                                self.get_logger().info(f"Saved intermediate: {inter_path}")
                        else:
                            error = info.get('error', 'Unknown')
                            self.get_logger().warning(f"✗ Frame {frames_processed + 1}: {error}")

                    frames_processed += 1

                except Exception as e:
                    self.get_logger().error(f"Exception processing frame {idx}: {e}", exc_info=True)
                    frames_processed += 1
                    continue

            # Save final panorama (or partial if stopped early)
            if panorama is not None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                was_stopped = self.stop_event.is_set()
                prefix = 'panorama_stopped' if was_stopped else 'panorama'
                final_path = os.path.join(self.output_dir, f'{prefix}_{timestamp}.jpg')
                cv2.imwrite(final_path, panorama)

                # Save metadata as JSON
                success_rate = (100 * frames_stitched / frames_processed) if frames_processed > 0 else 0
                metadata = {
                    'timestamp': timestamp,
                    'panorama_file': os.path.basename(final_path),
                    'was_stopped_early': was_stopped,
                    'panorama_size': {
                        'width': panorama.shape[1],
                        'height': panorama.shape[0]
                    },
                    'frames_processed': frames_processed,
                    'frames_stitched': frames_stitched,
                    'success_rate': f"{success_rate:.1f}%",
                    'detector': "SIFT" if self.use_sift else "ORB",
                    'blend_method': self.blend_method,
                    'downscale_factor': self.downscale
                }
                metadata_path = os.path.join(self.output_dir, f'{prefix}_{timestamp}.json')
                with open(metadata_path, 'w') as f:
                    json.dump(metadata, f, indent=2)

                self.get_logger().info("=" * 60)
                if was_stopped:
                    self.get_logger().info("MAPPING STOPPED EARLY")
                    self._send_status(f"Mapping stopped. Partial panorama saved: {frames_stitched}/{frames_processed} frames.")
                else:
                    self.get_logger().info("MAPPING COMPLETE")
                    self._send_status(f"Mapping complete. {frames_stitched}/{frames_processed} frames stitched.")
                self.get_logger().info(f"  Frames processed: {frames_processed}")
                self.get_logger().info(f"  Frames stitched: {frames_stitched}")
                if frames_processed > 0:
                    self.get_logger().info(f"  Success rate: {success_rate:.1f}%")
                self.get_logger().info(f"  Final size: {panorama.shape[1]}x{panorama.shape[0]}")
                self.get_logger().info(f"  Saved to: {final_path}")
                self.get_logger().info(f"  Metadata: {metadata_path}")
                self.get_logger().info("=" * 60)
            else:
                self.get_logger().error("No panorama created!")
                self._send_status("Error: No panorama created")

        except Exception as e:
            self.get_logger().error(f"Stitching thread exception: {e}", exc_info=True)
            self._send_status(f"Mapping error: {e}")
        
        finally:
            with self.stitching_lock:
                self.stitching_done = True
            self.get_logger().info("Stitching thread finished")

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
