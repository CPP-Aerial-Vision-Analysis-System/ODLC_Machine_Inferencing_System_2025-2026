import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class SiyiCamNode(Node):
    def __init__(self):
        super().__init__('siyi_cam_node')
        self.declare_parameter('source', 0) #'rtsp://192.168.1.100:8554/main.264' --> this is for rtsp
        src = self.get_parameter('source').get_parameter_value().string_value
        self.cap = cv2.VideoCapture(src)
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.caminfo_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 1)
        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)
        self.get_logger().info(f"Streaming from {src}")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        self.pub.publish(msg)

        ci = CameraInfo()
        ci.header = msg.header
        ci.width = frame.shape[1]
        ci.height = frame.shape[0]
        self.caminfo_pub.publish(ci)


def main(args=None):
    rclpy.init(args=args)
    node = SiyiCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
