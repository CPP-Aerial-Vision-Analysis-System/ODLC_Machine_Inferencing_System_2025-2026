#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def stitch_pair(img1, img2, min_matches=10):
    # ORB + BFMatcher + findHomography + warpPerspective
    orb = cv2.ORB_create(2000)
    k1, d1 = orb.detectAndCompute(img1, None)
    k2, d2 = orb.detectAndCompute(img2, None)
    if d1 is None or d2 is None:
        return None
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(d1, d2)
    matches = sorted(matches, key=lambda x: x.distance)
    if len(matches) < min_matches:
        return None
    src_pts = np.float32([k1[m.queryIdx].pt for m in matches]).reshape(-1,1,2)
    dst_pts = np.float32([k2[m.trainIdx].pt for m in matches]).reshape(-1,1,2)
    H, mask = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)
    if H is None:
        return None
    # warp img2 onto img1 coordinate space and blend
    h1, w1 = img1.shape[:2]
    h2, w2 = img2.shape[:2]
    # corners of img2 in img1 coords:
    corners = np.float32([[0,0],[0,h2],[w2,h2],[w2,0]]).reshape(-1,1,2)
    warped_corners = cv2.perspectiveTransform(corners, H)
    all_corners = np.concatenate((np.float32([[0,0],[0,h1],[w1,h1],[w1,0]]).reshape(-1,1,2), warped_corners), axis=0)
    [xmin, ymin] = np.int32(all_corners.min(axis=0).ravel() - 0.5)
    [xmax, ymax] = np.int32(all_corners.max(axis=0).ravel() + 0.5)
    t = [-xmin, -ymin]
    H_translation = np.array([[1,0,t[0]],[0,1,t[1]],[0,0,1]])
    result = cv2.warpPerspective(img2, H_translation.dot(H), (xmax - xmin, ymax - ymin))
    result[t[1]:h1+t[1], t[0]:w1+t[0]] = img1
    # simple seam: could do multiband blending here
    return result

class IncrementalStitcher(Node):
    def __init__(self):
        super().__init__('incremental_stitcher')
        self.sub = self.create_subscription(Image, 'camera/image_raw', self.cb, 10)
        self.pub = self.create_publisher(Image, 'panorama/image_raw', 10)
        self.bridge = CvBridge()
        self.panorama = None
        self.get_logger().info('Stitcher ready')

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # downscale optionally for speed
        frame_small = cv2.resize(frame, (0,0), fx=0.6, fy=0.6)
        if self.panorama is None:
            self.panorama = frame_small
            out_msg = self.bridge.cv2_to_imgmsg(self.panorama, encoding='bgr8')
            out_msg.header = msg.header
            self.pub.publish(out_msg)
            return
        stitched = stitch_pair(self.panorama, frame_small)
        if stitched is not None:
            self.panorama = stitched
            out_msg = self.bridge.cv2_to_imgmsg(self.panorama, encoding='bgr8')
            out_msg.header = msg.header
            self.pub.publish(out_msg)
        else:
            # failed to stitch: publish current panorama or ignore
            pass

def main(args=None):
    rclpy.init(args=args)
    node = IncrementalStitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
