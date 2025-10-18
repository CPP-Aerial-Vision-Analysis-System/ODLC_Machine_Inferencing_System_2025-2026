import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64
from cv_bridge import CvBridge
from mavros_msgs.msg import StatusText
from ament_index_python.packages import get_package_share_directory
import cv2, os, time

class SiyiA8Publisher(Node):
    def __init__(self):
        super().__init__('siyi_a8_publisher')

        # Publishers
        self.publisher = self.create_publisher(Image, 'image_raw', 10)
        self.status_publisher = self.create_publisher(StatusText, '/mavros/statustext/send', 10)

        # Subscribers
        self.create_subscription(Bool, '/camera/trigger', self.camera_trigger_callback, 10)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.check_altitude, 10)

        self.bridge = CvBridge()

        # Directory for saving images 
        self.photo_path = os.path.join(get_package_share_directory("video_cam"),"camera_feed")
        if not os.path.exists(self.photo_path):
            os.makedirs(self.photo_path)
        
        # Directory for saving mapping images
        self.mapping_photo_path = os.path.join(get_package_share_directory("video_cam"),"mapping_photos")
        if not os.path.exists(self.mapping_photo_path):
            os.makedirs(self.mapping_photo_path)
        
        # Real camera flag
        self.use_real_camera = True
        self.get_logger().info(f"Using real camera: {self.use_real_camera}")

        # Altitude threshold flag
        self.camera_enabled = False
        self.ALT_THRESHOLD = 13.716

        # Save photo flag
        self.capture_photo = False

        # Camera setup
        self.lastest_image_msg = None
        if self.use_real_camera:
            gst_pipeline = (
            'rtspsrc location=rtsp://192.168.144.25:8554/main.264 latency=0 ! '
            'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink'
            )
            self.capture = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            text = "Real camera initialized"
            self.send_ack(text)
            self.get_logger().info("Using real camera with GStreamer pipeline")
            if not self.capture.isOpened():
                self.get_logger().error('Could not open video stream')
                return
        else:
            text = "Simulation camera initialized"
            self.send_ack(text)
            self.get_logger().info("Using simulation camera")
            self.capture = cv2.VideoCapture(0)

            def sim_image_callback(self, msg):
                self.lastest_image_msg = msg
            self.create_subscription(Image, '/webcam/image_raw', self.sim_image_callback, 1)
        
        self.timer = self.create_timer(0.1, self.camera_loop)
    
    def send_ack(self, text):
        msg = StatusText()
        msg.severity = 6  # INFO
        msg.text = text
        self.status_publisher.publish(msg)
        self.get_logger().info(f"Status: {text}")
    
    def camera_trigger_callback(self, msg):
        if msg.data:
            self.get_logger().info("Canmera trigger received")
            self.capture_photo = True
    
    def check_altitude(self, msg):
        current_alt = msg.data
        if current_alt >= self.ALT_THRESHOLD:
            if not self.camera_enabled:
                text = "Min Altitude reached. Enabling detection"
                self.get_logger().info(text)
                self.send_ack(text)
            self.camera_enabled = True
        else:
            if self.camera_enabled:
                text = "Ideal Altitude not reached. Disabling detection"
                self.get_logger().info(text)
                self.send_ack(text)
            self.camera_enabled = False

    def camera_loop(self):
        if self.camera_enabled:
            if self.use_real_camera:
                returnValue, capturedFrame = self.capture.read()
                if returnValue == True:
                    self.get_logger().info("Begun Camera Frame Publishing")
                    imageToTransmit = self.bridge.cv2_to_imgmsg(capturedFrame, encoding='bgr8')
                    self.publisher.publish(imageToTransmit)
                    timestamp = time.strftime("%Y%m%d-%H%M%S")
                    filename = os.path.join(self.photo_path, f"photo_{timestamp}.jpg")
                    cv2.imwrite(filename, capturedFrame)

                    if self.capture_photo:
                        self.get_logger().info("Capturing photo...")
                        timestamp = time.strftime("%Y%m%d-%H%M%S")
                        mapping_filename = os.path.join(self.mapping_photo_path, f"mapping_photo_{timestamp}.jpg")
                        cv2.imwrite(mapping_filename, capturedFrame)
                        self.get_logger().info(f"Photo saved to {mapping_filename}")
                        self.capture_photo = False
            else:
                if self.lastest_image_msg is not None:
                    self.get_logger().info("Begun Camera Frame Republishing")
                    self.publisher.publish(self.lastest_image_msg)

                    # Save image
                    cv_image = self.bridge.imgmsg_to_cv2(self.lastest_image_msg, desired_encoding='bgr8')
                    timestamp = time.strftime("%Y%m%d-%H%M%S")
                    filename = os.path.join(self.photo_path, f"photo_{timestamp}.jpg")
                    cv2.imwrite(filename, cv_image) 

                    if self.capture_photo:
                        self.get_logger().info("Capturing photo...")
                        timestamp = time.strftime("%Y%m%d-%H%M%S")
                        mapping_filename = os.path.join(self.mapping_photo_path, f"mapping_photo_{timestamp}.jpg")
                        cv2.imwrite(mapping_filename, cv_image)
                        self.get_logger().info(f"Photo saved to {mapping_filename}")
                        self.capture_photo = False

def main(args=None):
    rclpy.init(args=args)
    siyi_a8_publisher = SiyiA8Publisher()
    rclpy.spin(siyi_a8_publisher)
    siyi_a8_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()