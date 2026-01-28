#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# sensor_msgs and CvBridge no longer strictly needed, but left for compatibility
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

from mavros_msgs.msg import StatusText
from rclpy.qos import qos_profile_sensor_data


def stitch_pair(img1, img2, min_matches=6, ratio=0.8, use_sift=False, debug=False):
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

    # --- Affine estimation (better for mapping, less bending) ---
    ransac_thresh = 3.0
    M, mask = cv2.estimateAffinePartial2D(
        src_pts,
        dst_pts,
        method=cv2.RANSAC,
        ransacReprojThreshold=ransac_thresh,
        maxIters=5000,
        confidence=0.99,
        refineIters=10
    )

    if M is None or mask is None:
        return None, "Affine estimation failed"

    matches_mask = mask.ravel().tolist()
    inlier_count = int(np.sum(matches_mask))
    inlier_ratio = inlier_count / len(good)

    min_inliers = max(1, min_matches // 6)
    if inlier_count < min_inliers:
        return None, f"Too few inliers: {inlier_count}/{min_inliers} ({inlier_ratio:.1%})"

    # Convert 2×3 affine matrix to 3×3 homography for the rest of the code
    H = np.eye(3, dtype=np.float64)
    H[:2, :] = M

    # --- Simple sanity check on scale (no crazy zooming) ---
    sx = np.linalg.norm(H[0, :2])
    sy = np.linalg.norm(H[1, :2])
    if not (0.7 < sx < 1.5 and 0.7 < sy < 1.5):
        return None, f"Unreasonable scale: sx={sx:.3f}, sy={sy:.3f}"

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
            np.where(mask_img2[:, :, np.newaxis] > 0, warped_img2, result)
        )
    else:
        # No overlap, just place img2
        result = np.where(mask_img2[:, :, np.newaxis] > 0, warped_img2, result)

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
        self.declare_parameter('max_frames', 150)
        self.declare_parameter('min_matches', 6)
        self.declare_parameter('ratio_test', 0.8)
        self.declare_parameter('blend_method', 'distance')  # 'distance' or 'multiband'

        current_file = os.path.abspath(__file__)
        current_dir = os.path.dirname(current_file)

        # Navigate up to find ros2_ws (look for install/ or src/ directories)
        search_dir = current_dir
        ros2_ws_dir = None

        for _ in range(10):  # Limit search depth
            if os.path.exists(os.path.join(search_dir, "install")) or os.path.exists(os.path.join(search_dir, "src")):
                if os.path.exists(os.path.join(search_dir, "install")) and os.path.exists(os.path.join(search_dir, "src")):
                    ros2_ws_dir = search_dir
                    break
                parent = os.path.dirname(search_dir)
                if os.path.exists(os.path.join(parent, "install")) and os.path.exists(os.path.join(parent, "src")):
                    ros2_ws_dir = parent
                    break
            search_dir = os.path.dirname(search_dir)
            if search_dir == "/":
                break

        if ros2_ws_dir and os.path.exists(os.path.join(ros2_ws_dir, "src")):
            ros2_ws_dir = os.path.join(ros2_ws_dir, "src")

        # Fallback: construct path directly
        if ros2_ws_dir is None:
            ros2_ws_dir = "/astra/ros2_ws/src"

        video_cam_dir = os.path.join(ros2_ws_dir, "video_cam")
        os.makedirs(video_cam_dir, exist_ok=True)

        # Directory for getting camera images
        self.camera_feed_path = os.path.join(video_cam_dir, "mapping_photos")
        if not os.path.exists(self.camera_feed_path):
            os.makedirs(self.camera_feed_path)

        # Directory for saving mapping results
        self.mapping_results_path = os.path.join(video_cam_dir, "mapping_results")
        if not os.path.exists(self.mapping_results_path):
            os.makedirs(self.mapping_results_path)

        use_sift = self.get_parameter('use_sift').value
        self.downscale = self.get_parameter('downscale_factor').value
        self.max_frames = self.get_parameter('max_frames').value
        self.save_dir = self.mapping_results_path
        self.min_matches = self.get_parameter('min_matches').value
        self.ratio_test = self.get_parameter('ratio_test').value
        self.blend_method = self.get_parameter('blend_method').value

        # We keep CvBridge defined but it's no longer used in directory mode
        self.bridge = CvBridge()

        self.panorama = None
        self.frames_processed = 0
        self.frames_stitched = 0

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
        self.current_index = 0
        self.mapping_started = False   # guard so we only run once

        self.get_logger().info(
            f'Stitcher ready (directory mode):\n'
            f'  Detector: {detector_name}\n'
            f'  Downscale: {self.downscale}\n'
            f'  Min matches: {self.min_matches}\n'
            f'  Ratio test: {self.ratio_test}\n'
            f'  Blend method: {self.blend_method}\n'
            f'  Input dir: {self.camera_feed_path}\n'
            f'  Output dir: {self.save_dir}\n'
            f'  Found {len(self.image_files)} images'
        )

        if not self.image_files:
            self.get_logger().warn("No images found in mapping_photos directory.")

        # Initialize subscription to listen for mapping commands
        self.command_listener = self.create_subscription(
            StatusText,
            '/mavros/statustext/recv',
            self.command_cb,
            qos_profile_sensor_data
        )
        self.command_listener  # prevent unused variable warning

        # Publisher to send feedback
        self.message_sender = self.create_publisher(StatusText, '/mavros/statustext/send', 10)

    def send_back(self, text):
        # feedback to GCS (Mission Planner messages tab)
        msg = StatusText()
        msg.severity = 6  # INFO/notice
        msg.text = text
        self.message_sender.publish(msg)

    def command_cb(self, msg: StatusText):
        if "follow" in msg.text.lower():  # change this if needed
            if self.mapping_started:
                # Avoid running twice if multiple zigzag messages arrive
                self.get_logger().info("Mapping already started, ignoring extra zigzag command.")
                return

            self.mapping_started = True
            self.get_logger().info("Received mapping command. Starting mapping...")
            self.send_back("Mapping command received. Starting mapping...")
            self.run_mapping()

    def run_mapping(self):
        """
        Process images in mapping_photos once, then finish.
        """
        while self.current_index < len(self.image_files) and self.frames_processed < self.max_frames:
            image_path = self.image_files[self.current_index]
            self.current_index += 1

            frame = cv2.imread(image_path)
            if frame is None:
                self.get_logger().warn(f"Failed to read image: {image_path}")
                continue

            frame_small = cv2.resize(frame, (0, 0), fx=self.downscale, fy=self.downscale)

            self.get_logger().info(
                f"Processing frame {self.frames_processed + 1} from {os.path.basename(image_path)}"
            )

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

        if rclpy.ok():
            rclpy.shutdown()





def main(args=None):
    rclpy.init(args=args)
    node = IncrementalStitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
