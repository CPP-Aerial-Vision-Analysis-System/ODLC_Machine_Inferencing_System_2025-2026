#!/usr/bin/env python3

"""
Full Workflow: SIYI A8 Camera Complete Image Capture and Download
Combines image_pub_siyi.py trigger capability with siyi.py download functionality

This node:
1. Connects to SIYI A8 camera via RTSP for live video streaming
2. Listens for ROS2 trigger to capture 4K images
3. Sends UDP command to camera to save photo to SD card
4. Waits for photo to appear on SD card (verification)
5. Downloads ALL images from SD card to Jetson (like siyi.py batch download)
6. Publishes video stream and status updates via ROS2

Complete workflow: Trigger → Capture to SD → Verify → Download All → Save to Jetson
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
from threading import Lock, Thread
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from urllib.parse import urlencode
from urllib.error import URLError, HTTPError
import numpy as np


class FullWorkflowSiyiPublisher(Node):
    """
    Complete SIYI A8 workflow: Capture to SD + Download All to Jetson
    """
    
    # Camera network configuration
    CAM_IP = "192.168.144.25"
    CTRL_PORT = 37260  # UDP port for SDK commands
    MEDIA_PORT = 82    # HTTP port for media API
    
    # API endpoints (from working siyi.py)
    BASE_URL = f"http://{CAM_IP}:{MEDIA_PORT}/cgi-bin/media.cgi/api/v1"
    
    # SIYI SDK commands
    TAKE_PHOTO_4K = bytes.fromhex("55 66 01 01 00 00 00 0c 00 34 ce")
    
    # Photo settings
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
    PHOTO_WAIT_TIMEOUT = 10  # seconds to wait for photo on SD
    
    def __init__(self):
        super().__init__('full_workflow_siyi_publisher')
        
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
        self.get_logger().info(" FULL WORKFLOW SIYI NODE INITIALIZED")
        self.get_logger().info("=" * 80)
        self.get_logger().info(f" Camera IP: {self.CAM_IP}")
        self.get_logger().info(f" Resolution: {self.current_resolution}")
        self.get_logger().info(f" SD Card Directory: {self.current_photo_dir}")
        self.get_logger().info(f" Local download directory: {self.download_path}")
        self.get_logger().info(f" Local camera_feed: {self.photo_path}")
        self.get_logger().info(f" Local mapping_photos: {self.mapping_photo_path}")
        self.get_logger().info("=" * 80)
        self.get_logger().info(" WORKFLOW:")
        self.get_logger().info("   1. Trigger: ros2 topic pub --once /camera/trigger std_msgs/msg/Bool \"data: true\"")
        self.get_logger().info("   2. Camera captures 4K photo to SD card")
        self.get_logger().info("   3. Node verifies photo on SD card")
        self.get_logger().info("   4. Node downloads ALL images from SD to Jetson")
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
            ros2_ws_dir = "/home/astra-dev/astra/ros2_ws"
            self.get_logger().warn(f"Using fallback path: {ros2_ws_dir}")
        
        # Create main video_cam_data directory
        video_cam_dir = os.path.join(ros2_ws_dir, "video_cam_data")
        os.makedirs(video_cam_dir, exist_ok=True)
        
        # Directory for downloaded images from SD (like siyi.py destination)
        self.download_path = os.path.join(video_cam_dir, "downloaded_from_sd")
        os.makedirs(self.download_path, exist_ok=True)
        
        # Directory for camera feed (for streaming)
        self.photo_path = os.path.join(video_cam_dir, "camera_feed")
        os.makedirs(self.photo_path, exist_ok=True)
        
        # Directory for mapping photos
        self.mapping_photo_path = os.path.join(video_cam_dir, "mapping_photos")
        os.makedirs(self.mapping_photo_path, exist_ok=True)
        
        self.get_logger().info(f"✓ Local storage setup: {video_cam_dir}")
    
    # ========================================================================
    # SD CARD INITIALIZATION AND MANAGEMENT (from siyi.py)
    # ========================================================================
    
    def initialize_sd_card(self):
        """
        Initialize connection to camera's SD card via HTTP API
        Uses the working API format from siyi.py
        """
        self.current_photo_dir = None
        self.last_photo_count = 0
        
        try:
            self.get_logger().info("Connecting to camera SD card HTTP API...")
            
            # Get directories for images (media_type=0)
            url = f"{self.BASE_URL}/getdirectories?media_type=0"
            
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
            self.get_logger().error("  Run: ping 192.168.144.25")
            self.current_photo_dir = "DCIM/100MEDIA"
        except Exception as e:
            self.get_logger().error(f"SD card init error: {e}")
            self.current_photo_dir = "DCIM/100MEDIA"
        
        self.get_logger().info(
            f"✓ SD card initialized. Directory: {self.current_photo_dir}")
    
    def get_photo_count(self, dir_path):
        """Get count of photos in directory"""
        try:
            url = f"{self.BASE_URL}/getmedialist"
            params = {
                'media_type': '0',
                'path': dir_path,
                'start': 0,
                'count': 9999  # Get all files to count accurately
            }
            
            response = requests.get(url, params=params, timeout=5)
            
            if response.status_code == 200:
                data = response.json()
                if data.get('success', False) and 'data' in data:
                    # First try to get 'total' field
                    total = data['data'].get('total', None)
                    if total is not None:
                        return total
                    # If no 'total', count the list items
                    file_list = data['data'].get('list', [])
                    return len(file_list)
            return None
        except Exception as e:
            self.get_logger().warn(f"Could not get photo count: {e}")
            return None
    
    # ========================================================================
    # PHOTO CAPTURE WORKFLOW (from image_pub_siyi.py)
    # ========================================================================
    
    def trigger_photo_on_sd_card(self):
        """Send UDP command to camera to capture photo to SD card"""
        try:
            self.sdk_socket.sendto(
                self.TAKE_PHOTO_4K, (self.CAM_IP, self.CTRL_PORT))
            self.photo_count += 1
            self.get_logger().info(
                f"✓ Photo trigger #{self.photo_count} sent to camera SD card")
            return True
        except Exception as e:
            self.get_logger().error(f"✗ Failed to trigger photo: {e}")
            return False
    
    def wait_for_new_photo_on_sd(self, timeout_s=10):
        """
        Wait for new photo to appear on SD card
        Returns True if new photo detected, False otherwise
        """
        if not self.current_photo_dir:
            self.get_logger().error("No photo directory available")
            return False
        
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
                            self.last_photo_count = total
                            return True
                        else:
                            elapsed = time.time() - start_time
                            self.get_logger().info(
                                f"  Waiting... ({elapsed:.1f}s, count={total})", 
                                throttle_duration_sec=2.0)
                
            except Exception as e:
                self.get_logger().warn(f"Poll error: {e}")
            
            time.sleep(poll_interval)
        
        self.get_logger().error(
            f"✗ SD VERIFICATION TIMEOUT after {timeout_s}s")
        return False
    
    # ========================================================================
    # BATCH DOWNLOAD FROM SD CARD (from siyi.py)
    # ========================================================================
    
    def download_all_from_sd_card(self):
        """
        Download ALL photos and videos from SD card to Jetson
        This is the complete siyi.py download functionality
        """
        self.get_logger().info("=" * 80)
        self.get_logger().info(" BATCH DOWNLOAD: All media from SD card to Jetson")
        self.get_logger().info("=" * 80)
        
        total_downloaded = 0
        
        # Download both images (0) and videos (1)
        for media_type in [0, 1]:
            media_type_str = "images" if media_type == 0 else "videos"
            self.get_logger().info(f"📥 Downloading {media_type_str}...")
            
            try:
                # Get list of directories
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
                self.get_logger().info(f"  Found {len(directories)} directories")
                
                # Process each directory
                for directory in directories:
                    dir_path = directory.get('path', '')
                    if not dir_path:
                        continue
                    
                    self.get_logger().info(f"  📁 Directory: {dir_path}")
                    
                    # Get file list from directory
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
                        self.get_logger().warn(
                            f"Failed to get file list: HTTP {file_response.status_code}")
                        continue
                    
                    file_data = file_response.json()
                    
                    if not file_data.get('success', False):
                        self.get_logger().warn("File list API returned success=false")
                        continue
                    
                    file_list = file_data.get('data', {}).get('list', [])
                    self.get_logger().info(f"    📄 {len(file_list)} files to download")
                    
                    # Download each file
                    for fileinfo in file_list:
                        filename = fileinfo.get('name', '')
                        file_url = fileinfo.get('url', '')
                        
                        if not filename or not file_url:
                            continue
                        
                        # Fix IP address in URL (camera returns default IP)
                        file_url = file_url.replace("192.168.144.25", self.CAM_IP)
                        
                        dest_file = os.path.join(self.download_path, filename)
                        
                        # Skip if already exists
                        if os.path.exists(dest_file):
                            self.get_logger().info(
                                f"      ⏭️  Skip (exists): {filename}",
                                throttle_duration_sec=0.5)
                            continue
                        
                        try:
                            self.get_logger().info(f"      ⬇️  Downloading: {filename}")
                            
                            response = requests.get(file_url, timeout=30)
                            if response.status_code == 200:
                                # Write file
                                with open(dest_file, 'wb') as f:
                                    f.write(response.content)
                                
                                size_kb = len(response.content) / 1024
                                self.get_logger().info(
                                    f"      ✅ Saved: {filename} ({size_kb:.1f}KB)")
                                total_downloaded += 1
                            else:
                                self.get_logger().error(
                                    f"      ✗ HTTP {response.status_code}: {filename}")
                        except Exception as e:
                            self.get_logger().error(f"      ✗ Error: {e}")
                
            except Exception as e:
                self.get_logger().error(f"Error downloading {media_type_str}: {e}")
        
        self.get_logger().info("=" * 80)
        self.get_logger().info(f" ✅ BATCH DOWNLOAD COMPLETE: {total_downloaded} files downloaded")
        self.get_logger().info(f"    Destination: {self.download_path}")
        self.get_logger().info("=" * 80)
        
        return total_downloaded
    
    # ========================================================================
    # COMPLETE WORKFLOW: CAPTURE + DOWNLOAD
    # ========================================================================
    
    def full_capture_and_download_workflow(self):
        """
        Complete workflow combining image_pub_siyi.py and siyi.py:
        1. Trigger photo capture to SD
        2. Wait for verification
        3. Download ALL images from SD to Jetson
        """
        with self.photo_lock:
            self.get_logger().info("=" * 80)
            self.get_logger().info(" STARTING FULL WORKFLOW")
            self.get_logger().info("=" * 80)
            
            # STEP 1: Trigger photo capture
            self.get_logger().info("[1/3] Triggering 4K photo capture on camera...")
            if not self.trigger_photo_on_sd_card():
                self.get_logger().error("✗ Failed to trigger photo")
                return False
            
            # STEP 2: Wait for photo to appear on SD card
            self.get_logger().info("[2/3] Waiting for photo to appear on SD card...")
            if not self.wait_for_new_photo_on_sd(timeout_s=self.PHOTO_WAIT_TIMEOUT):
                self.get_logger().error("✗ Photo did not appear on SD card")
                return False
            
            # STEP 3: Download ALL images from SD card
            self.get_logger().info("[3/3] Downloading ALL images from SD card to Jetson...")
            downloaded_count = self.download_all_from_sd_card()
            
            if downloaded_count > 0:
                self.get_logger().info("=" * 80)
                self.get_logger().info(" ✅ FULL WORKFLOW COMPLETE")
                self.get_logger().info("=" * 80)
                
                # Publish success status
                status_msg = String()
                status_msg.data = f"SUCCESS: Photo captured to SD and {downloaded_count} files downloaded to Jetson"
                self.camera_status_pub.publish(status_msg)
                self.send_ack(status_msg.data)
                return True
            else:
                self.get_logger().warn("⚠️ Workflow completed but no new files downloaded")
                return True
    
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
        """Handle camera trigger requests - starts full workflow"""
        if msg.data:
            self.get_logger().info("=" * 80)
            self.get_logger().info(" 📸 CAMERA TRIGGER RECEIVED")
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
                self.get_logger().info("✓ Altitude OK - camera enabled")
        else:
            if self.camera_enabled:
                self.camera_enabled = False
                self.get_logger().info("⚠️ Below altitude threshold - camera disabled")
    
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
                # Publish live stream to ROS2
                if self.bridge is not None:
                    img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                else:
                    img_msg = self.cv2_to_imgmsg_manual(frame, encoding='bgr8')
                
                self.publisher.publish(img_msg)
                
                # Handle photo capture trigger - RUNS FULL WORKFLOW
                if self.capture_photo:
                    # Run full workflow in separate thread to not block loop
                    workflow_thread = Thread(target=self.full_capture_and_download_workflow)
                    workflow_thread.start()
                    
                    self.capture_photo = False
            else:
                self.get_logger().warn(
                    "Failed to read camera frame", throttle_duration_sec=10.0)
        
        elif self.latest_image_msg is not None:
            # Simulation mode
            self.publisher.publish(self.latest_image_msg)
            
            if self.capture_photo:
                self.get_logger().info("⚠️ Simulation mode - no real camera workflow")
                self.capture_photo = False
    
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
    node = FullWorkflowSiyiPublisher()
    
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
