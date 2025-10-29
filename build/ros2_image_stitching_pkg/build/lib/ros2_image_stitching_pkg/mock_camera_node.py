#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class MockCameraNode(Node):
    def __init__(self):
        super().__init__('mock_camera_node')

        # Declare parameter for image directory
        self.declare_parameter('image_dir', '/workspace/ros2_ws/SUAS2025CameraFeed')
        self.image_dir = self.get_parameter('image_dir').value

        # Publisher for camera images
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        # Load all image file paths
        self.image_files = sorted([
            os.path.join(self.image_dir, f) 
            for f in os.listdir(self.image_dir) 
            if f.lower().endswith(('.jpg', '.jpeg', '.png'))
        ])

        if not self.image_files:
            self.get_logger().error(f"No images found in {self.image_dir}")
        else:
            self.get_logger().info(f"Found {len(self.image_files)} images. Starting publishing...")

        # Publish images once
        self.publish_images()

    def publish_images(self):
        for idx, img_file in enumerate(self.image_files):
            frame = cv2.imread(img_file)
            if frame is None:
                self.get_logger().warn(f"Failed to read {img_file}")
                continue

            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
            self.get_logger().info(f"Published image {idx+1}/{len(self.image_files)}: {os.path.basename(img_file)}")

            time.sleep(0.5)  # small delay between frames

        self.get_logger().info("Finished publishing all test images. Node will now exit.")
        self.finished = True

def main(args=None):
    rclpy.init(args=args)
    node = MockCameraNode()
    rclpy.spin(node)  # spin in case you want other callbacks, but it will exit after shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
