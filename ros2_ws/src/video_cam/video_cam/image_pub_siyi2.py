#!/usr/bin/env python3

"""
SIYI A8 Camera Integration with SD Card Storage and Ethernet Download
Combines image_pub_siyi.py with siyi.py download functionality

This node:
1. Triggers the SIYI A8 camera to capture 4K images
2. Verifies images are saved to the camera's SD card
3. Downloads images from SD card via Ethernet
4. Saves images locally on Jetson
5. Publishes images to ROS2 topics for processing

AP_FLAKE8_CLEAN
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, String
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except Exception as e:
    print(f"Warning: cv_bridge import failed: {e}")
    CV_BRIDGE_AVAILABLE = False
    CvBridge = None
from mavros_msgs.msg import StatusText
import cv2
import os
import time
import socket
import struct
import requests
import json
from threading import Lock
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from urllib.parse import urlencode
from urllib.error import URLError, HTTPError
import numpy as np


class SiyiA8CombinedPublisher(Node):
    """
    Combined SIYI A8 camera node with SD card storage and download capability
    """
    
    CAM_IP = "192.168.144.25"
    CTRL_PORT = 37260  # UDP port for SDK commands
    MEDIA_PORT = 82    # HTTP port for media server
    
    # API endpoints (based on working siyi.py implementation)
    BASE_URL = f"http://{CAM_IP}:{MEDIA_PORT}/cgi-bin/media.cgi/api/v1"
    
    # SIYI SDK commands (A8 mini User Manual)
    TAKE_PHOTO_4K = bytes.fromhex("55 66 01 01 00 00 00 0c 00 34 ce")
    
    # Photo resolution modes
    PHOTO_RESOLUTIONS = {
        '4K': 0x00,      # 3840x2160 (default)
        '2.7K': 0x01,    # 2704x1520
        '1080P': 0x02    # 1920x1080
    }
    
    # Verification constants
    MIN_4K_WIDTH = 3000
    MIN_4K_HEIGHT = 1600
    MIN_FILE_SIZE = 50000  # 50KB minimum
    MAX_CAPTURE_RETRIES = 3
    PHOTO_WAIT_TIMEOUT = 15  # seconds to wait for photo on SD
    
    def __init__(self):
        super().__init__('siyi_a8_combined_publisher')
        
        # ====================================================================
        # ROS2 PUBLISHERS
        # ====================================================================
        self.publisher = self.create_publisher(Image, 'image_raw', 10)
        self.status_publisher = self.create_publisher(
            StatusText, '/mavros/statustext/send', 10)
        self.camera_status_pub = self.create_publisher(
            String, '/camera/status', 10)
        
        # ====================================================================
        # ROS2 SUBSCRIBERS
        # ====================================================================
        self.create_subscription(
            Bool, '/camera/trigger', self.camera_trigger_callback, 10)
        self.create_subscription(
            String, '/camera/set_resolution', self.set_resolution_callback, 10)
        self.create_subscription(
            Float64, '/mavros/global_position/rel_alt', 
            self.check_altitude, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/camera/image', self.sim_image_callback, 1)
        
        # ====================================================================
        # INITIALIZE CV BRIDGE
        # ====================================================================
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warn(
                "cv_bridge not available, using alternative conversion")
        
        # ====================================================================
        # UDP SOCKET FOR CAMERA COMMANDS
        # ====================================================================
        self.sdk_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sdk_socket.settimeout(2.0)
        
        # ====================================================================
        # CAMERA STATE VARIABLES
        # ====================================================================
        self.current_resolution = '4K'
        self.save_to_sd_card = True
        self.photo_count = 0
        self.current_photo_dir = None
        self.last_photo_count = 0
        self.photo_lock = Lock()
        
        # Camera mode
        self.use_real_camera = True
        self.camera_enabled = True
        self.ALT_THRESHOLD = -13.716
        self.capture_photo = False
        
        # Simulation mode
        self.latest_image_msg = None
        self.capture = None
        
        # ====================================================================
        # SETUP LOCAL STORAGE DIRECTORIES
        # ====================================================================
        self._setup_local_directories()
        
        # ====================================================================
        # INITIALIZE SD CARD CONNECTION
        # ====================================================================
        self.initialize_sd_card()
        
        # ====================================================================
        # INITIALIZE CAMERA CONNECTION
        # ====================================================================
        self._initialize_camera()
        
        # ====================================================================
        # START MAIN CAMERA LOOP
        # ====================================================================
        self.timer = self.create_timer(0.1, self.camera_loop)
        
        # ====================================================================
        # LOG INITIALIZATION COMPLETE
        # ====================================================================
        self.get_logger().info("=" * 80)
        self.get_logger().info(" SIYI A8 COMBINED NODE INITIALIZED")
        self.get_logger().info("=" * 80)
        self.get_logger().info(f" Camera IP: {self.CAM_IP}")
        self.get_logger().info(f" Resolution: {self.current_resolution}")
        self.get_logger().info(f" SD Card Directory: {self.current_photo_dir}")
        self.get_logger().info(f" Local camera_feed: {self.photo_path}")
        self.get_logger().info(f" Local mapping_photos: {self.mapping_photo_path}")
        self.get_logger().info("=" * 80)
        self.get_logger().info(" To capture a 4K image:")
        self.get_logger().info("   ros2 topic pub --once /camera/trigger std_msgs/msg/Bool \"data: true\"")
        self.get_logger().info("=" * 80)
    
    def _setup_local_directories(self):
        """Setup local storage directories on Jetson"""
        current_file = os.path.abspath(__file__)
        current_dir = os.path.dirname(current_file)
        
        # Find ros2_ws directory
        search_dir = current_dir
        ros2_ws_dir = None
        
        for _ in range(10):
            if (os.path.exists(os.path.join(search_dir, "install")) and 
                os.path.exists(os.path.join(search_dir, "src"))):
                ros2_ws_dir = search_dir
                break
            search_dir = os.path.dirname(search_dir)
            if search_dir == "/":
                break
        
        if ros2_ws_dir is None:
            ros2_ws_dir = "/astra/ros2_ws"
            self.get_logger().warn(f"Using fallback path: {ros2_ws_dir}")
        
        # Create directories
        video_cam_dir = os.path.join(ros2_ws_dir, "video_cam_data")
        os.makedirs(video_cam_dir, exist_ok=True)
        
        self.photo_path = os.path.join(video_cam_dir, "camera_feed")
        os.makedirs(self.photo_path, exist_ok=True)
        
        self.mapping_photo_path = os.path.join(video_cam_dir, "mapping_photos")
        os.makedirs(self.mapping_photo_path, exist_ok=True)
        
        self.downloaded_photos_path = os.path.join(video_cam_dir, "downloaded_from_sd")
        os.makedirs(self.downloaded_photos_path, exist_ok=True)
        
        self.get_logger().info(f"Local storage setup complete: {video_cam_dir}")
    
    # ========================================================================
    # SD CARD INITIALIZATION AND MANAGEMENT
    # ========================================================================
    
    def initialize_sd_card(self):
        """
        Initialize connection to camera's SD card via HTTP API
        Uses the working API format from siyi.py
        """
        self.current_photo_dir = None
        self.last_photo_count = 0
        
        try:
            self.get_logger().info("Connecting to camera HTTP API...")
            
            url = f"{self.BASE_URL}/getdirectories?media_type=0"
            self.get_logger().info(f"API URL: {url}")
            
            response = requests.get(url, timeout=5)
            
            if response.status_code == 200:
                data = response.json()
                
                if data.get('success', False):
                    if 'data' in data and 'directories' in data['data']:
                        directories = data['data']['directories']
                        self.get_logger().info(
                            f"✓ Found {len(directories)} directories on SD card")
                        
                        if len(directories) > 0:
                            # Use most recent directory
                            self.current_photo_dir = directories[-1]['path']
                            self.get_logger().info(
                                f"✓ Using directory: {self.current_photo_dir}")
                            
                            # Get initial photo count
                            count = self.get_photo_count(self.current_photo_dir)
                            if count is not None:
                                self.last_photo_count = count
                                self.get_logger().info(
                                    f"✓ Initial photo count: {self.last_photo_count}")
                        else:
                            self.current_photo_dir = "DCIM/100MEDIA"
                            self.get_logger().warn("No directories found, using default")
                    else:
                        self.current_photo_dir = "DCIM/100MEDIA"
                else:
                    self.current_photo_dir = "DCIM/100MEDIA"
            else:
                self.current_photo_dir = "DCIM/100MEDIA"
                self.get_logger().warn(
                    f"HTTP {response.status_code}, using default directory")
                
        except requests.exceptions.ConnectionError:
            self.get_logger().error(
                "✗ Cannot connect to camera - check Ethernet connection!")
            self.get_logger().error(
                "  Run: ping 192.168.144.25")
            self.current_photo_dir = "DCIM/100MEDIA"
        except Exception as e:
            self.get_logger().error(f"SD card init error: {e}")
            self.current_photo_dir = "DCIM/100MEDIA"
        
        self.get_logger().info(
            f"SD card ready. Directory: {self.current_photo_dir}")
    
    def get_photo_count(self, dir_path):
        """Get count of photos in directory"""
        try:
            url = f"{self.BASE_URL}/getmedialist"
            params = {
                'media_type': '0',
                'path': dir_path,
                'start': 0,
                'count': 1
            }
            
            response = requests.get(url, params=params, timeout=5)
            
            if response.status_code == 200:
                data = response.json()
                if data.get('success', False) and 'data' in data:
                    return data['data'].get('total', 0)
            return None
        except Exception as e:
            self.get_logger().warn(f"Could not get photo count: {e}")
            return None
    
    # ========================================================================
    # PHOTO CAPTURE WORKFLOW
    # ========================================================================
    
    def trigger_photo_on_sd_card(self):
        """Send SDK command to camera to capture photo to SD card"""
        try:
            self.sdk_socket.sendto(
                self.TAKE_PHOTO_4K, (self.CAM_IP, self.CTRL_PORT))
            self.photo_count += 1
            self.get_logger().info(
                f"✓ Photo trigger sent to camera (photo #{self.photo_count})")
            return True
        except Exception as e:
            self.get_logger().error(f"✗ Failed to trigger photo: {e}")
            return False
    
    def wait_for_new_photo_on_sd(self, timeout_s=15):
        """
        Wait for new photo to appear on SD card
        Returns the download URL if successful, None otherwise
        """
        if not self.current_photo_dir:
            self.get_logger().error("No photo directory available")
            return None
        
        start_time = time.time()
        poll_interval = 0.5
        
        self.get_logger().info(
            f"Polling SD card for new photo (timeout: {timeout_s}s)...")
        
        while (time.time() - start_time) < timeout_s:
            try:
                url = f"{self.BASE_URL}/getmedialist"
                params = {
                    'media_type': '0',
                    'path': self.current_photo_dir,
                    'start': 0,
                    'count': 9999
                }
                
                response = requests.get(url, params=params, timeout=5)
                
                if response.status_code == 200:
                    data = response.json()
                    
                    if data.get('success', False) and 'data' in data:
                        total = data['data'].get('total', 0)
                        
                        # Check if new photo appeared
                        if total > self.last_photo_count:
                            self.get_logger().info(
                                f"✓ SD VERIFIED: Photo count {self.last_photo_count} → {total}")
                            
                            if 'list' in data['data']:
                                file_list = data['data']['list']
                                
                                if len(file_list) > 0:
                                    # Get most recent photo
                                    latest_photo = file_list[-1]
                                    photo_url = latest_photo.get('url', '')
                                    photo_name = latest_photo.get('name', 'unknown')
                                    
                                    # Fix IP address in URL
                                    photo_url = photo_url.replace(
                                        "192.168.144.25", self.CAM_IP)
                                    
                                    self.last_photo_count = total
                                    self.get_logger().info(
                                        f"✓ Found on SD: {photo_name}")
                                    return photo_url
                        else:
                            elapsed = time.time() - start_time
                            self.get_logger().info(
                                f"  Waiting... ({elapsed:.1f}s)", 
                                throttle_duration_sec=2.0)
                
            except Exception as e:
                self.get_logger().warn(f"Poll error: {e}")
            
            time.sleep(poll_interval)
        
        self.get_logger().error(
            f"✗ SD VERIFICATION TIMEOUT after {timeout_s}s")
        return None
    
    def download_photo_from_sd(self, photo_url):
        """Download photo from camera's SD card over Ethernet"""
        try:
            self.get_logger().info(f"⬇ Downloading from SD: {photo_url}")
            response = requests.get(photo_url, timeout=15)
            
            if response.status_code == 200:
                # Convert to numpy array and decode
                img_array = np.frombuffer(response.content, dtype=np.uint8)
                img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                
                if img is not None:
                    h, w = img.shape[:2]
                    size_kb = len(response.content) / 1024
                    self.get_logger().info(
                        f"✓ Downloaded: {w}x{h} ({size_kb:.1f}KB)")
                    
                    # Verify dimensions
                    if w >= self.MIN_4K_WIDTH and h >= self.MIN_4K_HEIGHT:
                        return img
                    else:
                        self.get_logger().warn(
                            f"⚠ Image dimensions may not be 4K: {w}x{h}")
                        return img  # Still return it
                else:
                    self.get_logger().error("✗ Failed to decode image")
                    return None
            else:
                self.get_logger().error(
                    f"✗ Download failed: HTTP {response.status_code}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"✗ Download error: {e}")
            return None
    
    def capture_and_download_4k_photo(self):
        """
        Complete workflow: trigger → wait for SD → download → verify
        Returns (success, image) tuple
        """
        with self.photo_lock:
            for attempt in range(self.MAX_CAPTURE_RETRIES):
                if attempt > 0:
                    self.get_logger().warn(
                        f"Retry {attempt + 1}/{self.MAX_CAPTURE_RETRIES}")
                
                # Step 1: Trigger photo
                self.get_logger().info(
                    f"[1/3] Triggering {self.current_resolution} capture...")
                if not self.trigger_photo_on_sd_card():
                    if attempt < self.MAX_CAPTURE_RETRIES - 1:
                        time.sleep(1)
                        continue
                    return False, None
                
                # Step 2: Wait for SD card
                self.get_logger().info("[2/3] Waiting for photo on SD card...")
                photo_url = self.wait_for_new_photo_on_sd(
                    timeout_s=self.PHOTO_WAIT_TIMEOUT)
                if not photo_url:
                    if attempt < self.MAX_CAPTURE_RETRIES - 1:
                        time.sleep(1)
                        continue
                    return False, None
                
                # Step 3: Download from SD
                self.get_logger().info("[3/3] Downloading from SD card...")
                img = self.download_photo_from_sd(photo_url)
                
                if img is not None:
                    self.get_logger().info("✅ Capture successful!")
                    return True, img
                else:
                    if attempt < self.MAX_CAPTURE_RETRIES - 1:
                        time.sleep(1)
                        continue
            
            self.get_logger().error(
                f"✗ All {self.MAX_CAPTURE_RETRIES} attempts failed")
            return False, None
    
    # ========================================================================
    # FILE OPERATIONS WITH VERIFICATION
    # ========================================================================
    
    def save_photo_atomic(self, filepath, img):
        """
        Atomic file write with verification
        Returns True if successful
        """
        try:
            tmp_path = filepath + ".tmp"
            
            # Write to temp file
            success = cv2.imwrite(tmp_path, img)
            if not success:
                self.get_logger().error(f"cv2.imwrite failed: {tmp_path}")
                return False
            
            # Verify temp file
            if not os.path.exists(tmp_path):
                self.get_logger().error(f"Temp file not created: {tmp_path}")
                return False
            
            file_size = os.path.getsize(tmp_path)
            if file_size < self.MIN_FILE_SIZE:
                self.get_logger().error(
                    f"File too small: {file_size} bytes")
                os.remove(tmp_path)
                return False
            
            # Atomic rename
            os.replace(tmp_path, filepath)
            self.get_logger().info(
                f"✓ Saved: {filepath} ({file_size/1024:.1f}KB)")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Save error: {e}")
            try:
                if os.path.exists(tmp_path):
                    os.remove(tmp_path)
            except:
                pass
            return False
    
    # ========================================================================
    # BATCH DOWNLOAD FROM SD CARD (like siyi.py)
    # ========================================================================
    
    def download_all_from_sd_card(self, dest_dir=None):
        """
        Download all photos and videos from SD card
        This is the functionality from siyi.py
        """
        if dest_dir is None:
            dest_dir = self.downloaded_photos_path
        
        self.get_logger().info("=" * 80)
        self.get_logger().info(" BATCH DOWNLOAD FROM SD CARD")
        self.get_logger().info("=" * 80)
        
        # Download both images (0) and videos (1)
        for media_type in [0, 1]:
            media_type_str = "images" if media_type == 0 else "videos"
            self.get_logger().info(f"Downloading {media_type_str}...")
            
            try:
                # Get directories
                dir_url = f"{self.BASE_URL}/getdirectories?media_type={media_type}"
                response = requests.get(dir_url, timeout=5)
                
                if response.status_code != 200:
                    self.get_logger().error(
                        f"Failed to get directories: HTTP {response.status_code}")
                    continue
                
                dir_data = response.json()
                
                if not dir_data.get('success', False):
                    self.get_logger().error("API returned success=false")
                    continue
                
                directories = dir_data.get('data', {}).get('directories', [])
                self.get_logger().info(f"Found {len(directories)} directories")
                
                # Get files from each directory
                for directory in directories:
                    dir_path = directory.get('path', '')
                    if not dir_path:
                        continue
                    
                    self.get_logger().info(f"  Directory: {dir_path}")
                    
                    # Get file list
                    file_url = f"{self.BASE_URL}/getmedialist"
                    params = {
                        'media_type': str(media_type),
                        'path': dir_path,
                        'start': 0,
                        'count': 9999
                    }
                    
                    file_response = requests.get(
                        file_url, params=params, timeout=5)
                    
                    if file_response.status_code != 200:
                        continue
                    
                    file_data = file_response.json()
                    
                    if not file_data.get('success', False):
                        continue
                    
                    file_list = file_data.get('data', {}).get('list', [])
                    self.get_logger().info(f"    {len(file_list)} files")
                    
                    # Download each file
                    for fileinfo in file_list:
                        filename = fileinfo.get('name', '')
                        file_url = fileinfo.get('url', '')
                        
                        if not filename or not file_url:
                            continue
                        
                        # Fix IP address
                        file_url = file_url.replace("192.168.144.25", self.CAM_IP)
                        
                        dest_file = os.path.join(dest_dir, filename)
                        
                        try:
                            self.get_logger().info(f"      Downloading {filename}...")
                            
                            response = requests.get(file_url, timeout=30)
                            if response.status_code == 200:
                                with open(dest_file, 'wb') as f:
                                    f.write(response.content)
                                self.get_logger().info(
                                    f"      ✓ Saved to {dest_file}")
                            else:
                                self.get_logger().error(
                                    f"      ✗ HTTP {response.status_code}")
                        except Exception as e:
                            self.get_logger().error(f"      ✗ Error: {e}")
                
            except Exception as e:
                self.get_logger().error(f"Error downloading {media_type_str}: {e}")
        
        self.get_logger().info("=" * 80)
        self.get_logger().info(" BATCH DOWNLOAD COMPLETE")
        self.get_logger().info("=" * 80)
    
    # ========================================================================
    # CAMERA INITIALIZATION
    # ========================================================================
    
    def _initialize_camera(self):
        """Initialize RTSP camera stream for live view"""
        if not self.use_real_camera:
            self.get_logger().info("Simulation mode - no camera stream needed")
            self.capture = None
            return
        
        rtsp_url = f'rtsp://{self.CAM_IP}:8554/main.264'
        
        self.get_logger().info(f"Connecting to camera stream: {rtsp_url}")
        
        # Try FFmpeg first
        self.capture = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        
        if not self.capture.isOpened():
            self.get_logger().warn("FFmpeg failed, trying GStreamer...")
            gst_pipeline = (
                f'rtspsrc location={rtsp_url} latency=0 ! '
                'rtph264depay ! h264parse ! avdec_h264 ! '
                'videoconvert ! appsink'
            )
            self.capture = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.capture.isOpened():
            self.get_logger().warn("GStreamer failed, trying default...")
            self.capture = cv2.VideoCapture(rtsp_url)
        
        if self.capture.isOpened():
            self.get_logger().info("✓ Camera stream connected")
            text = "SIYI camera stream initialized"
            self.send_ack(text)
        else:
            self.get_logger().error("✗ Failed to connect to camera stream")
            text = "Camera stream connection failed"
            self.send_ack(text)
            self.capture = None
    
    # ========================================================================
    # ROS2 CALLBACKS
    # ========================================================================
    
    def camera_trigger_callback(self, msg):
        """Handle camera trigger requests"""
        if msg.data:
            self.get_logger().info("=" * 80)
            self.get_logger().info(" CAMERA TRIGGER RECEIVED")
            self.get_logger().info("=" * 80)
            self.capture_photo = True
    
    def set_resolution_callback(self, msg):
        """Handle resolution change requests"""
        resolution = msg.data.upper()
        if resolution in self.PHOTO_RESOLUTIONS:
            self.current_resolution = resolution
            self.get_logger().info(f"Resolution changed to: {resolution}")
        else:
            self.get_logger().warn(
                f"Invalid resolution: {resolution}. Valid: {list(self.PHOTO_RESOLUTIONS.keys())}")
    
    def check_altitude(self, msg):
        """Enable/disable camera based on altitude"""
        current_alt = msg.data
        if current_alt >= self.ALT_THRESHOLD:
            if not self.camera_enabled:
                self.camera_enabled = True
                self.get_logger().info("Altitude OK - camera enabled")
        else:
            if self.camera_enabled:
                self.camera_enabled = False
                self.get_logger().info("Below altitude threshold - camera disabled")
    
    def sim_image_callback(self, msg):
        """Handle simulation images"""
        self.latest_image_msg = msg
    
    def send_ack(self, text):
        """Send status message"""
        msg = StatusText()
        msg.severity = 6  # INFO
        msg.text = text
        self.status_publisher.publish(msg)
    
    # ========================================================================
    # MAIN CAMERA LOOP
    # ========================================================================
    
    def camera_loop(self):
        """Main camera loop - publishes stream and handles photo capture"""
        if not self.camera_enabled:
            return
        
        if self.use_real_camera and self.capture is not None:
            # Read frame from stream
            ret, frame = self.capture.read()
            
            if ret and frame is not None:
                # Publish live stream
                if self.bridge is not None:
                    img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                else:
                    img_msg = self.cv2_to_imgmsg_manual(frame, encoding='bgr8')
                
                self.publisher.publish(img_msg)
                
                # Handle photo capture trigger
                if self.capture_photo:
                    timestamp = time.strftime("%Y%m%d-%H%M%S")
                    
                    self.get_logger().info("=" * 80)
                    self.get_logger().info(" CAPTURING 4K PHOTO FROM SD CARD")
                    self.get_logger().info("=" * 80)
                    
                    # Capture and download 4K photo
                    success, img_4k = self.capture_and_download_4k_photo()
                    
                    if success and img_4k is not None:
                        # Save to both directories
                        filename_feed = os.path.join(
                            self.photo_path, f"photo_4K_{timestamp}.jpg")
                        filename_mapping = os.path.join(
                            self.mapping_photo_path, f"mapping_{timestamp}.jpg")
                        
                        ok1 = self.save_photo_atomic(filename_feed, img_4k)
                        ok2 = self.save_photo_atomic(filename_mapping, img_4k)
                        
                        if ok1 and ok2:
                            self.get_logger().info("=" * 80)
                            self.get_logger().info(" ✅ SUCCESS - PHOTO SAVED")
                            self.get_logger().info("=" * 80)
                            self.get_logger().info(f"  camera_feed: {filename_feed}")
                            self.get_logger().info(f"  mapping: {filename_mapping}")
                            
                            status_msg = String()
                            status_msg.data = f"SUCCESS: 4K photo saved ({self.current_resolution})"
                            self.camera_status_pub.publish(status_msg)
                            self.send_ack(status_msg.data)
                        else:
                            self.get_logger().error("✗ Local save verification failed")
                    else:
                        self.get_logger().error("✗ Failed to capture from SD card")
                        
                        # Fallback: save stream frame
                        fallback_file = os.path.join(
                            self.photo_path, f"fallback_stream_{timestamp}.jpg")
                        if cv2.imwrite(fallback_file, frame):
                            self.get_logger().warn(f"⚠ Saved stream fallback: {fallback_file}")
                    
                    self.get_logger().info("=" * 80)
                    self.capture_photo = False
            else:
                self.get_logger().warn(
                    "Failed to read camera frame", throttle_duration_sec=10.0)
        
        elif self.latest_image_msg is not None:
            # Simulation mode
            self.publisher.publish(self.latest_image_msg)
    
    # ========================================================================
    # UTILITY METHODS
    # ========================================================================
    
    def cv2_to_imgmsg_manual(self, cv_image, encoding='bgr8'):
        """Convert OpenCV image to ROS Image without cv_bridge"""
        msg = Image()
        msg.height = cv_image.shape[0]
        msg.width = cv_image.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = cv_image.shape[1] * cv_image.shape[2]
        msg.data = cv_image.tobytes()
        return msg
    
    def __del__(self):
        """Cleanup"""
        if hasattr(self, 'sdk_socket'):
            self.sdk_socket.close()
        if hasattr(self, 'capture') and self.capture is not None:
            self.capture.release()


def main(args=None):
    rclpy.init(args=args)
    node = SiyiA8CombinedPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
