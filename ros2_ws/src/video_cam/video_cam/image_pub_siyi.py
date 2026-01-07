# -*- coding: utf-8 -*-
"""
SIYI A8 mini Camera ROS2 Node
Captures 4K photos using SDK commands and HTTP media server
Based on SIYI A8 mini User Manual v1.6 and v1.8
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64
from mavros_msgs.msg import StatusText
import cv2
import os
import time
import socket
import requests
import numpy as np
from threading import Lock

# Lazy import cv_bridge to avoid initialization errors
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except Exception as e:
    print(f"Warning: cv_bridge import failed: {e}")
    print("Will attempt to use alternative image conversion methods")
    CV_BRIDGE_AVAILABLE = False
    CvBridge = None


class SiyiA8Publisher(Node):
    # SIYI A8 mini camera configuration
    CAM_IP = "192.168.144.25"
    CTRL_PORT = 37260
    MEDIA_PORT = 82
    MEDIA_URL = f"http://{CAM_IP}:{MEDIA_PORT}/cgi-bin/media.cgi"
    
    # SIYI SDK command: Take Picture (CMD_ID 0x0C, func_type 0)
    # Based on A8 mini User Manual v1.6
    TAKE_PIC_PKT = bytes.fromhex("55 66 01 01 00 00 00 0c 00 34 ce")
    
    def __init__(self):
        super().__init__('siyi_a8_publisher')

        # Publishers
        self.publisher = self.create_publisher(Image, 'image_raw', 10)
        self.status_publisher = self.create_publisher(StatusText, '/mavros/statustext/send', 10)

        # Subscribers
        self.create_subscription(Bool, '/camera/trigger', self.camera_trigger_callback, 10)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.check_altitude, 10)
        self.create_subscription(Image, '/webcam/image_raw', self.sim_image_callback, 1)

        # Initialize cv_bridge if available
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warn("cv_bridge not available, using alternative conversion")

        # Setup photo storage directories
        self.setup_directories()
        
        # UDP socket for SDK commands
        self.sdk_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sdk_socket.settimeout(2.0)
        
        # Camera state
        self.camera_available = False
        self.current_photo_dir = None
        self.last_photo_count = 0
        self.camera_enabled = True
        self.capture_photo = False
        self.latest_image_msg = None
        self.photo_lock = Lock()
        
        # Altitude threshold
        self.ALT_THRESHOLD = 13.716
        
        # Initialize camera connection
        self.initialize_camera()
        
        # Timer for periodic image capture (5 seconds)
        self.timer = self.create_timer(5.0, self.camera_loop)
        
        self.get_logger().info("SIYI A8 Publisher initialized")
    
    def setup_directories(self):
        """Setup directory structure for photo storage"""
        # Find ros2_ws directory
        current_file = os.path.abspath(__file__)
        current_dir = os.path.dirname(current_file)
        
        search_dir = current_dir
        ros2_ws_dir = None
        
        for _ in range(10):
            if os.path.exists(os.path.join(search_dir, "install")) and os.path.exists(os.path.join(search_dir, "src")):
                ros2_ws_dir = search_dir
                break
            search_dir = os.path.dirname(search_dir)
            if search_dir == "/":
                break
        
        # Fallback
        if ros2_ws_dir is None:
            ros2_ws_dir = "/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws"
        
        video_cam_dir = os.path.join(ros2_ws_dir, "video_cam")
        os.makedirs(video_cam_dir, exist_ok=True)
        
        # Directory for saving images 
        self.photo_path = os.path.join(video_cam_dir, "camera_feed")
        os.makedirs(self.photo_path, exist_ok=True)
        
        # Directory for saving mapping images
        self.mapping_photo_path = os.path.join(video_cam_dir, "mapping_photos")
        os.makedirs(self.mapping_photo_path, exist_ok=True)
        
        self.get_logger().info(f"Photo storage: {self.photo_path}")
        self.get_logger().info(f"Mapping photos: {self.mapping_photo_path}")



    def initialize_camera(self):
        """Initialize SIYI A8 mini camera via HTTP media server"""
        self.get_logger().info("Initializing SIYI A8 mini camera...")
        
        try:
            # Test connection to media server
            response = requests.get(
                self.MEDIA_URL,
                params={"cmd": "getdirectories"},
                json={},
                timeout=3
            )
            
            if response.status_code == 200:
                data = response.json()
                self.get_logger().info(f"Camera connected successfully: {data}")
                
                # Get photo directories
                if "photo" in data and len(data["photo"]) > 0:
                    self.current_photo_dir = data["photo"][-1]  # Most recent directory
                    self.get_logger().info(f"Current photo directory: {self.current_photo_dir}")
                    
                    # Get initial photo count
                    count_resp = self.media_command("getmediacount", {
                        "media_type": 0,
                        "path": self.current_photo_dir
                    })
                    
                    if count_resp and "count" in count_resp:
                        self.last_photo_count = count_resp["count"]
                        self.get_logger().info(f"Initial photo count: {self.last_photo_count}")
                else:
                    self.get_logger().warn("No photo directories found. SD card may not be inserted.")
                    self.current_photo_dir = None
                
                self.camera_available = True
                self.send_ack("SIYI A8 mini camera initialized successfully")
                
            else:
                self.get_logger().error(f"Failed to connect to camera: HTTP {response.status_code}")
                self.camera_available = False
                self.send_ack("SIYI camera not available - check network connection")
                
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Camera connection error: {e}")
            self.get_logger().warn("Camera not available. Will use simulation mode if /webcam/image_raw is available")
            self.camera_available = False
            self.send_ack("SIYI camera not available - using simulation mode")
        except Exception as e:
            self.get_logger().error(f"Unexpected error during camera initialization: {e}")
            self.camera_available = False
    
    def media_command(self, cmd, payload):
        """Send command to camera's HTTP media server"""
        try:
            response = requests.post(
                self.MEDIA_URL,
                params={"cmd": cmd},
                json=payload,
                timeout=3
            )
            
            if response.status_code == 200:
                return response.json()
            else:
                self.get_logger().error(f"Media command '{cmd}' failed: HTTP {response.status_code}")
                return None
                
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Media command '{cmd}' error: {e}")
            return None
    
    def take_picture(self):
        """Trigger camera to take a photo using SDK command"""
        try:
            self.sdk_socket.sendto(self.TAKE_PIC_PKT, (self.CAM_IP, self.CTRL_PORT))
            self.get_logger().info("Photo trigger sent to camera")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to send photo trigger: {e}")
            return False
    
    def wait_for_new_photo(self, timeout_s=10):
        """Wait for a new photo to appear on camera's SD card"""
        if not self.current_photo_dir:
            self.get_logger().error("No photo directory available")
            return None
        
        start_time = time.time()
        
        while (time.time() - start_time) < timeout_s:
            # Check photo count
            count_resp = self.media_command("getmediacount", {
                "media_type": 0,
                "path": self.current_photo_dir
            })
            
            if count_resp and "count" in count_resp:
                current_count = count_resp["count"]
                
                if current_count > self.last_photo_count:
                    # New photo detected, get the latest photo URL
                    list_resp = self.media_command("getmedialist", {
                        "media_type": 0,
                        "path": self.current_photo_dir,
                        "start": current_count - 1,  # Get last photo
                        "count": 1
                    })
                    
                    if list_resp and "data" in list_resp and len(list_resp["data"]) > 0:
                        photo_url = list_resp["data"][0]["url"]
                        self.last_photo_count = current_count
                        self.get_logger().info(f"New photo available: {photo_url}")
                        return photo_url
            
            time.sleep(0.3)  # Poll interval
        
        self.get_logger().warn(f"Timeout waiting for new photo after {timeout_s}s")
        return None
    
    def download_photo(self, photo_url):
        """Download photo from camera to Jetson"""
        try:
            response = requests.get(photo_url, timeout=10)
            
            if response.status_code == 200:
                # Convert bytes to numpy array
                img_array = np.frombuffer(response.content, dtype=np.uint8)
                # Decode image
                img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                
                if img is not None:
                    self.get_logger().info(f"Photo downloaded successfully, size: {img.shape}")
                    return img
                else:
                    self.get_logger().error("Failed to decode downloaded image")
                    return None
            else:
                self.get_logger().error(f"Failed to download photo: HTTP {response.status_code}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error downloading photo: {e}")
            return None
    
    def capture_and_download_photo(self):
        """Complete workflow: trigger photo, wait for it, and download it"""
        with self.photo_lock:
            # Step 1: Trigger photo
            if not self.take_picture():
                return None
            
            # Step 2: Wait for photo to appear on SD card
            photo_url = self.wait_for_new_photo(timeout_s=10)
            if not photo_url:
                return None
            
            # Step 3: Download photo
            img = self.download_photo(photo_url)
            return img



    def sim_image_callback(self, msg):
        """Callback for simulation images from /webcam/image_raw topic"""
        self.latest_image_msg = msg
    
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
        """Send status message to MAVLink"""
        msg = StatusText()
        msg.severity = 6  # INFO
        msg.text = text
        self.status_publisher.publish(msg)
        self.get_logger().info(f"Status: {text}")
    
    def camera_trigger_callback(self, msg):
        """Callback for manual camera trigger"""
        if msg.data:
            self.get_logger().info("Camera trigger received")
            self.capture_photo = True
    
    def check_altitude(self, msg):
        """Enable or disable camera based on altitude threshold"""
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
        """Main camera loop that captures and publishes images"""
        if not self.camera_enabled:
            return
        
        # Use SIYI camera if available
        if self.camera_available and self.current_photo_dir:
            try:
                self.get_logger().info("Capturing photo from SIYI camera...", throttle_duration_sec=5.0)
                
                # Capture and download photo
                img = self.capture_and_download_photo()
                
                if img is not None:
                    # Save to camera feed directory
                    timestamp = time.strftime("%Y%m%d-%H%M%S")
                    filename = os.path.join(self.photo_path, f"photo_{timestamp}.jpg")
                    cv2.imwrite(filename, img)
                    self.get_logger().info(f"Photo saved: {filename}")
                    
                    # Convert to ROS message and publish
                    if self.bridge is not None:
                        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                    else:
                        img_msg = self.cv2_to_imgmsg_manual(img, encoding='bgr8')
                    
                    self.publisher.publish(img_msg)
                    self.get_logger().info("Image published to /image_raw", throttle_duration_sec=5.0)
                    
                    # Save to mapping directory if triggered
                    if self.capture_photo:
                        mapping_filename = os.path.join(self.mapping_photo_path, f"mapping_photo_{timestamp}.jpg")
                        cv2.imwrite(mapping_filename, img)
                        self.get_logger().info(f"Mapping photo saved: {mapping_filename}")
                        self.capture_photo = False
                        
                else:
                    self.get_logger().warn("Failed to capture photo from SIYI camera", throttle_duration_sec=10.0)
                    
            except Exception as e:
                self.get_logger().error(f"Error in camera loop: {e}", throttle_duration_sec=10.0)
        
        # Fallback to simulation mode
        elif self.latest_image_msg is not None:
            self.get_logger().info("Using simulation image from /webcam/image_raw", throttle_duration_sec=5.0)
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
                mapping_filename = os.path.join(self.mapping_photo_path, f"mapping_photo_{timestamp}.jpg")
                cv2.imwrite(mapping_filename, cv_image)
                self.get_logger().info(f"Mapping photo saved: {mapping_filename}")
                self.capture_photo = False
        
        else:
            self.get_logger().warn("No camera available and no simulation image received", throttle_duration_sec=10.0)
    
    def __del__(self):
        """Cleanup when node is destroyed"""
        if hasattr(self, 'sdk_socket'):
            self.sdk_socket.close()

def main(args=None):
    rclpy.init(args=args)
    siyi_a8_publisher = SiyiA8Publisher()
    
    try:
        rclpy.spin(siyi_a8_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        siyi_a8_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
