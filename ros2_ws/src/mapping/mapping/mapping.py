#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

def stitch_pair(img1, img2, min_matches=8, ratio=0.75, use_sift=False, debug=False):
    """
    Stitch two images with multi-band blending for seamless results.
    img1: reference image (panorama)
    img2: new frame to add
    """
    # --- Feature detection with contrast enhancement ---
    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY) if len(img1.shape) == 3 else img1
    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY) if len(img2.shape) == 3 else img2
    
    # Apply CLAHE for better feature detection
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray1 = clahe.apply(gray1)
    gray2 = clahe.apply(gray2)
    
    if use_sift:
        detector = cv2.SIFT_create(nfeatures=5000)
        norm_type = cv2.NORM_L2
    else:
        detector = cv2.ORB_create(
            nfeatures=8000,
            scaleFactor=1.2,
            nlevels=8,
            edgeThreshold=15,
            firstLevel=0,
            WTA_K=2,
            scoreType=cv2.ORB_HARRIS_SCORE,
            patchSize=31,
            fastThreshold=10
        )
        norm_type = cv2.NORM_HAMMING
    
    k1, d1 = detector.detectAndCompute(gray1, None)
    k2, d2 = detector.detectAndCompute(gray2, None)
    
    if d1 is None or d2 is None:
        return None, f"No descriptors (k1={len(k1) if k1 else 0}, k2={len(k2) if k2 else 0})"
    
    if len(k1) < min_matches or len(k2) < min_matches:
        return None, f"Insufficient keypoints: k1={len(k1)}, k2={len(k2)}"

    # --- Matching with ratio test ---
    bf = cv2.BFMatcher(norm_type, crossCheck=False)
    try:
        knn = bf.knnMatch(d2, d1, k=2)
    except cv2.error as e:
        return None, f"Matching error: {e}"
    
    good = []
    for match_pair in knn:
        if len(match_pair) == 2:
            m, n = match_pair
            if m.distance < ratio * n.distance:
                good.append(m)
    
    if len(good) < min_matches:
        return None, f"Insufficient good matches: {len(good)}/{min_matches}"

    # Extract matched points
    src_pts = np.float32([k2[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
    dst_pts = np.float32([k1[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

    # --- Homography estimation ---
    ransac_thresh = 3.0
    H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, ransac_thresh, maxIters=5000)
    
    if H is None:
        return None, "Homography estimation failed"
    
    matches_mask = mask.ravel().tolist()
    inlier_count = sum(matches_mask)
    inlier_ratio = inlier_count / len(good)
    
    min_inliers = max(6, min_matches // 2)
    
    if inlier_count < min_inliers:
        return None, f"Too few inliers: {inlier_count}/{min_inliers} ({inlier_ratio:.1%})"

    # Validate homography
    try:
        det = abs(np.linalg.det(H[:2, :2]))
        if det < 0.05 or det > 15:
            return None, f"Invalid scale: det={det:.3f}"
        
        if abs(H[0, 1]) > 0.7 or abs(H[1, 0]) > 0.7:
            return None, f"Excessive skew: H01={H[0,1]:.2f}, H10={H[1,0]:.2f}"
    except Exception as e:
        return None, f"Homography validation error: {e}"

    # --- Compute canvas size ---
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
    
    T = np.array([[1, 0, tx],
                  [0, 1, ty],
                  [0, 0, 1]], dtype=np.float64)

    out_w = xmax - xmin
    out_h = ymax - ymin
    
    max_dimension = max(w1, h1, w2, h2) * 4
    if out_w <= 0 or out_h <= 0 or out_w > max_dimension or out_h > max_dimension:
        return None, f"Invalid output size: {out_w}x{out_h} (max: {max_dimension})"

    # --- Check overlap ---
    img1_box = [tx, ty, tx + w1, ty + h1]
    corners_t = cv2.perspectiveTransform(corners_img2, T @ H)
    x2min = corners_t[:, :, 0].min()
    y2min = corners_t[:, :, 1].min()
    x2max = corners_t[:, :, 0].max()
    y2max = corners_t[:, :, 1].max()
    img2_box = [x2min, y2min, x2max, y2max]

    ix1 = max(img1_box[0], img2_box[0])
    iy1 = max(img1_box[1], img2_box[1])
    ix2 = min(img1_box[2], img2_box[2])
    iy2 = min(img1_box[3], img2_box[3])
    
    if ix2 <= ix1 or iy2 <= iy1:
        return None, "No spatial overlap"
    
    inter_area = (ix2 - ix1) * (iy2 - iy1)
    img2_area = (img2_box[2] - img2_box[0]) * (img2_box[3] - img2_box[1])
    overlap_ratio = inter_area / img2_area if img2_area > 0 else 0
    
    if overlap_ratio < 0.03:
        return None, f"Insufficient overlap: {overlap_ratio:.1%}"

    # --- Warp images ---
    warped_img2 = cv2.warpPerspective(img2, T @ H, (out_w, out_h))
    
    # Create base canvas with img1
    result = np.zeros((out_h, out_w, 3), dtype=np.uint8)
    result[ty:ty + h1, tx:tx + w1] = img1

    # --- Create precise masks ---
    mask_img1 = np.zeros((out_h, out_w), dtype=np.uint8)
    mask_img1[ty:ty + h1, tx:tx + w1] = 255
    
    mask_img2 = np.zeros((out_h, out_w), dtype=np.uint8)
    cv2.fillConvexPoly(mask_img2, np.int32(corners_t), 255)
    
    # --- Distance transform blending for seamless seams ---
    # Find overlap region
    overlap_mask = cv2.bitwise_and(mask_img1, mask_img2)
    
    if cv2.countNonZero(overlap_mask) > 0:
        # Distance transform creates smooth gradients
        dist1 = cv2.distanceTransform(mask_img1, cv2.DIST_L2, 5)
        dist2 = cv2.distanceTransform(mask_img2, cv2.DIST_L2, 5)
        
        # Normalize distances
        dist1_norm = dist1 / (dist1 + dist2 + 1e-6)
        dist2_norm = dist2 / (dist1 + dist2 + 1e-6)
        
        # Apply additional smoothing for extra seamlessness
        dist1_norm = cv2.GaussianBlur(dist1_norm, (31, 31), 10)
        dist2_norm = cv2.GaussianBlur(dist2_norm, (31, 31), 10)
        
        # Renormalize after blur
        dist_sum = dist1_norm + dist2_norm
        dist_sum = np.maximum(dist_sum, 1e-6)
        dist1_norm = dist1_norm / dist_sum
        dist2_norm = dist2_norm / dist_sum
        
        # Convert to 3-channel for RGB blending
        dist1_3c = np.stack([dist1_norm] * 3, axis=-1)
        dist2_3c = np.stack([dist2_norm] * 3, axis=-1)
        
        # Blend in overlap region
        overlap_3c = np.stack([overlap_mask] * 3, axis=-1) > 0
        result = np.where(
            overlap_3c,
            (result.astype(float) * dist1_3c + warped_img2.astype(float) * dist2_3c).astype(np.uint8),
            np.where(mask_img2[:,:,np.newaxis] > 0, warped_img2, result)
        )
    else:
        # No overlap, just place img2
        result = np.where(mask_img2[:,:,np.newaxis] > 0, warped_img2, result)
    
    # --- Crop black borders ---
    # Find the largest rectangular region without black pixels
    gray_result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray_result, 1, 255, cv2.THRESH_BINARY)
    
    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # Get bounding box of largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        
        # Add small margin to avoid edge artifacts
        margin = 2
        x = max(0, x + margin)
        y = max(0, y + margin)
        w = min(w - 2*margin, result.shape[1] - x)
        h = min(h - 2*margin, result.shape[0] - y)
        
        # Crop to content
        result = result[y:y+h, x:x+w]
    
    info = f"Success: {inlier_count} inliers ({inlier_ratio:.1%}), overlap={overlap_ratio:.1%}"
    return result, info


def multiband_blend(img1, img2, mask1, mask2, levels=4):
    """
    Multi-band blending for professional seamless results.
    This is what commercial panorama software uses.
    """
    # Build Gaussian pyramids
    G1 = img1.copy()
    G2 = img2.copy()
    GM1 = mask1.copy()
    GM2 = mask2.copy()
    
    gp1 = [G1]
    gp2 = [G2]
    gpm1 = [GM1]
    gpm2 = [GM2]
    
    for i in range(levels):
        G1 = cv2.pyrDown(G1)
        G2 = cv2.pyrDown(G2)
        GM1 = cv2.pyrDown(GM1)
        GM2 = cv2.pyrDown(GM2)
        gp1.append(G1)
        gp2.append(G2)
        gpm1.append(GM1)
        gpm2.append(GM2)
    
    # Build Laplacian pyramids
    lp1 = [gp1[levels - 1]]
    lp2 = [gp2[levels - 1]]
    
    for i in range(levels - 1, 0, -1):
        size = (gp1[i - 1].shape[1], gp1[i - 1].shape[0])
        L1 = cv2.subtract(gp1[i - 1], cv2.pyrUp(gp1[i], dstsize=size))
        L2 = cv2.subtract(gp2[i - 1], cv2.pyrUp(gp2[i], dstsize=size))
        lp1.append(L1)
        lp2.append(L2)
    
    # Blend each level
    LS = []
    for l1, l2, m1, m2 in zip(lp1, lp2, gpm1[::-1], gpm2[::-1]):
        # Normalize masks
        m1 = m1.astype(float) / 255.0
        m2 = m2.astype(float) / 255.0
        if len(m1.shape) == 2:
            m1 = np.stack([m1] * 3, axis=-1)
            m2 = np.stack([m2] * 3, axis=-1)
        
        ls = l1.astype(float) * m1 + l2.astype(float) * m2
        LS.append(ls)
    
    # Reconstruct
    result = LS[0]
    for i in range(1, levels):
        size = (LS[i].shape[1], LS[i].shape[0])
        result = cv2.add(cv2.pyrUp(result, dstsize=size), LS[i])
    
    return np.clip(result, 0, 255).astype(np.uint8)


class IncrementalStitcher(Node):
    def __init__(self):
        super().__init__('incremental_stitcher')
        
        # Parameters
        self.declare_parameter('use_sift', False)
        self.declare_parameter('downscale_factor', 0.5)
        self.declare_parameter('max_frames', 10)
        self.declare_parameter('save_dir', '/workspace/ros2_ws/panoramas')
        self.declare_parameter('min_matches', 8)
        self.declare_parameter('ratio_test', 0.75)
        self.declare_parameter('blend_method', 'distance')  # 'distance' or 'multiband'
        
        use_sift = self.get_parameter('use_sift').value
        self.downscale = self.get_parameter('downscale_factor').value
        self.max_frames = self.get_parameter('max_frames').value
        self.save_dir = self.get_parameter('save_dir').value
        self.min_matches = self.get_parameter('min_matches').value
        self.ratio_test = self.get_parameter('ratio_test').value
        self.blend_method = self.get_parameter('blend_method').value
        
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.sub = self.create_subscription(Image, 'camera/image_raw', self.cb, 10)
        self.bridge = CvBridge()
        self.panorama = None
        self.frames_processed = 0
        self.frames_stitched = 0
        
        self.use_sift = use_sift
        detector_name = "SIFT" if use_sift else "ORB"
        self.get_logger().info(
            f'Stitcher ready:\n'
            f'  Detector: {detector_name}\n'
            f'  Downscale: {self.downscale}\n'
            f'  Min matches: {self.min_matches}\n'
            f'  Ratio test: {self.ratio_test}\n'
            f'  Blend method: {self.blend_method}'
        )

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_small = cv2.resize(frame, (0, 0), fx=self.downscale, fy=self.downscale)
        
        self.get_logger().info(f"Received frame {self.frames_processed + 1}")
        
        if self.panorama is None:
            self.panorama = frame_small
            self.get_logger().info("Initialized panorama with first frame")
            self.frames_stitched = 1
        else:
            result, msg_str = stitch_pair(
                self.panorama, 
                frame_small, 
                min_matches=self.min_matches,
                ratio=self.ratio_test,
                use_sift=self.use_sift,
                debug=False
            )
            
            if result is not None:
                self.panorama = result
                self.frames_stitched += 1
                self.get_logger().info(
                    f"✓ Stitched frame {self.frames_processed + 1} - {msg_str}"
                )
            else:
                self.get_logger().warn(
                    f"✗ Failed frame {self.frames_processed + 1}: {msg_str}"
                )
        
        self.frames_processed += 1
        
        if self.frames_processed % 5 == 0 and self.panorama is not None:
            interim_path = os.path.join(self.save_dir, f'interim_{self.frames_processed}.jpg')
            cv2.imwrite(interim_path, self.panorama)
            self.get_logger().info(f"Saved interim: {interim_path}")
        
        if self.frames_processed >= self.max_frames:
            if self.panorama is not None:
                final_path = os.path.join(self.save_dir, 'final_panorama.jpg')
                cv2.imwrite(final_path, self.panorama)
                
                self.get_logger().info("=" * 60)
                self.get_logger().info(f"FINAL RESULTS:")
                self.get_logger().info(f"  Frames processed: {self.frames_processed}")
                self.get_logger().info(f"  Frames stitched: {self.frames_stitched}")
                self.get_logger().info(f"  Success rate: {100*self.frames_stitched/self.frames_processed:.1f}%")
                self.get_logger().info(f"  Final size: {self.panorama.shape[1]}x{self.panorama.shape[0]}")
                self.get_logger().info(f"  Saved to: {final_path}")
                self.get_logger().info("=" * 60)
            else:
                self.get_logger().error("No panorama created!")
            
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = IncrementalStitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()