# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64
# Lazy import cv_bridge to avoid initialization errors
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except Exception as e:
    print(f"Warning: cv_bridge import failed: {e}")
    print("Will attempt to use alternative image conversion methods")
    CV_BRIDGE_AVAILABLE = False
    CvBridge = None
from mavros_msgs.msg import StatusText
from ament_index_python.packages import get_package_share_directory
import cv2, os, time
import subprocess
import numpy as np

class SiyiA8Publisher(Node):
    def __init__(self):
        super().__init__('siyi_a8_publisher')

        # Publishers
        self.publisher = self.create_publisher(Image, 'image_raw', 10)
        self.status_publisher = self.create_publisher(StatusText, '/mavros/statustext/send', 10)

        # Subscribers
        self.create_subscription(Bool, '/camera/trigger', self.camera_trigger_callback, 10)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.check_altitude, 10)
        self.create_subscription(Image, '/camera/image', self.sim_image_callback, 1)

        # Initialize cv_bridge if available
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warn("cv_bridge not available, using alternative conversion")

        # Directory for saving images 
        self.photo_path = os.path.join(get_package_share_directory("video_cam"),"camera_feed")
        if not os.path.exists(self.photo_path):
            os.makedirs(self.photo_path)
        
        # Directory for saving mapping images
        self.mapping_photo_path = os.path.join(get_package_share_directory("video_cam"),"mapping_photos")
        if not os.path.exists(self.mapping_photo_path):
            os.makedirs(self.mapping_photo_path)
        
        # Real camera flag
        self.use_real_camera = False
        self.get_logger().info(f"Using real camera: {self.use_real_camera}")

        # Altitude threshold flag
        self.camera_enabled = True  # Enable camera for simulation
        self.ALT_THRESHOLD = -13.716

        # Save photo flag
        self.capture_photo = False

        self.timer = self.create_timer(0.1, self.camera_loop)

        # Camera setup
        self.latest_image_msg = None
        self.gstreamer_process = None
        
        if self.use_real_camera:
            rtsp_url = 'rtsp://192.168.144.25:8554/main.264'
            
            # Try multiple methods to open the camera
            self.capture = None
            
            # Method 1: Try with FFmpeg backend (most compatible)
            self.get_logger().info(f"Attempting to connect to camera at {rtsp_url} using FFmpeg...")
            self.capture = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
            
            if not self.capture.isOpened():
                self.get_logger().warn('FFmpeg method failed, trying GStreamer pipeline...')
                # Method 2: Try with GStreamer pipeline
                gst_pipeline = (
                    'rtspsrc location=rtsp://192.168.144.25:8554/main.264 latency=0 ! '
                    'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink'
                )
                self.capture = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            
            if not self.capture.isOpened():
                self.get_logger().warn('GStreamer method failed, trying default backend...')
                # Method 3: Try with default backend
                self.capture = cv2.VideoCapture(rtsp_url)
            
            if not self.capture.isOpened():
                self.get_logger().error('All OpenCV methods failed. Camera may not be accessible.')
                text = "Camera connection failed"
                self.send_ack(text)
                return
            else:
                text = "Real camera initialized"
                self.send_ack(text)
                self.get_logger().info("Successfully connected to camera!")
        else:
            text = "Simulation camera initialized"
            self.send_ack(text)
            self.get_logger().info("Using simulation camera - no physical camera needed")
            self.capture = None  # No physical camera needed for simulation

    def sim_image_callback(self, msg):
        self.latest_image_msg = msg
    
    def init_gstreamer_subprocess(self):
        """Initialize GStreamer subprocess as fallback when OpenCV doesn't have GStreamer support"""
        try:
            # First, get video dimensions from the stream
            probe_command = [
                'gst-launch-1.0', '-q',
                'rtspsrc', 'location=rtsp://192.168.144.25:8554/main.264', 'latency=0', '!',
                'rtph264depay', '!', 'h264parse', '!', 'avdec_h264', '!',
                'videoconvert', '!', 'video/x-raw,format=BGR', '!',
                'fakesink', 'num-buffers=1'
            ]
            
            # Try to probe stream - if this fails, camera isn't available
            probe = subprocess.run(probe_command, capture_output=True, timeout=5)
            if probe.returncode != 0:
                self.get_logger().error('Cannot connect to camera stream')
                return False
            
            # Assume common resolution - you may need to adjust this
            # Common SIYI A8 resolutions: 1920x1080, 1280x720
            self.frame_width = 1920
            self.frame_height = 1080
            self.frame_channels = 3
            self.frame_size = self.frame_width * self.frame_height * self.frame_channels
            
            # GStreamer pipeline that outputs raw BGR frames to stdout
            gst_command = [
                'gst-launch-1.0', '-q',
                'rtspsrc', 'location=rtsp://192.168.144.25:8554/main.264', 'latency=0', '!',
                'rtph264depay', '!', 'h264parse', '!', 'avdec_h264', '!',
                'videoscale', '!', f'video/x-raw,format=BGR,width={self.frame_width},height={self.frame_height}', '!',
                'fdsink'
            ]
            
            self.gstreamer_process = subprocess.Popen(
                gst_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=self.frame_size * 2
            )
            
            # Wait a bit for the stream to start
            time.sleep(1)
            
            if self.gstreamer_process.poll() is not None:
                self.get_logger().error('GStreamer subprocess failed to start')
                return False
            
            self.get_logger().info(f'GStreamer subprocess started successfully (resolution: {self.frame_width}x{self.frame_height})')
            return True
        except subprocess.TimeoutExpired:
            self.get_logger().error('Timeout connecting to camera')
            return False
        except Exception as e:
            self.get_logger().error(f'Failed to start GStreamer subprocess: {e}')
            return False
    
    def read_frame_from_gstreamer(self):
        """Read a frame from the GStreamer subprocess"""
        try:
            if self.gstreamer_process is None or self.gstreamer_process.poll() is not None:
                return False, None
            
            # Read raw frame data
            raw_frame = self.gstreamer_process.stdout.read(self.frame_size)
            
            if len(raw_frame) != self.frame_size:
                return False, None
            
            # Convert to numpy array and reshape
            frame = np.frombuffer(raw_frame, dtype=np.uint8)
            frame = frame.reshape((self.frame_height, self.frame_width, self.frame_channels))
            
            return True, frame
        except Exception as e:
            self.get_logger().error(f'Error reading frame from GStreamer: {e}')
            return False, None
    
    def cv2_to_imgmsg_manual(self, cv_image, encoding='bgr8'):
        """Convert OpenCV image to ROS Image message without cv_bridge"""
        msg = Image()
        msg.height = cv_image.shape[0]
        msg.width = cv_image.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = cv_image.shape[1] * cv_image.shape[2]
        msg.data = cv_image.tobytes()
        return msg
    
    def imgmsg_to_cv2_manual(self, img_msg, desired_encoding='bgr8'):
        """Convert ROS Image message to OpenCV image without cv_bridge"""
        if img_msg.encoding != desired_encoding:
            self.get_logger().warn(f'Image encoding mismatch: {img_msg.encoding} vs {desired_encoding}')
        
        dtype = np.uint8
        n_channels = 3 if desired_encoding == 'bgr8' else 1
        
        img_buf = np.asarray(img_msg.data, dtype=dtype)
        cv_image = img_buf.reshape(img_msg.height, img_msg.width, n_channels)
        
        return cv_image

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
                if self.capture is None:
                    self.get_logger().warn("Camera not initialized", throttle_duration_sec=10.0)
                    return
                
                returnValue, capturedFrame = self.capture.read()
                
                if returnValue == True and capturedFrame is not None:
                    self.get_logger().info("Camera Frame Publishing", throttle_duration_sec=5.0)
                    
                    # Convert to ROS message
                    if self.bridge is not None:
                        imageToTransmit = self.bridge.cv2_to_imgmsg(capturedFrame, encoding='bgr8')
                    else:
                        imageToTransmit = self.cv2_to_imgmsg_manual(capturedFrame, encoding='bgr8')
                    
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
                    self.get_logger().warn("Failed to read frame from camera", throttle_duration_sec=10.0)
            else:
                if self.latest_image_msg is not None:
                    self.get_logger().info("Begun Camera Frame Republishing", throttle_duration_sec=5.0)
                    self.publisher.publish(self.latest_image_msg)

                    # Save image
                    if self.bridge is not None:
                        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='bgr8')
                    else:
                        cv_image = self.imgmsg_to_cv2_manual(self.latest_image_msg, desired_encoding='bgr8')
                    
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
                else:
                    self.get_logger().warn("No image received from /webcam/image_raw yet", throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    siyi_a8_publisher = SiyiA8Publisher()
    rclpy.spin(siyi_a8_publisher)
    siyi_a8_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()