#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, String
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
import socket
import struct
import requests
from threading import Lock
from rclpy.qos import QoSProfile, qos_profile_sensor_data

class SiyiA8Publisher(Node):
    # SIYI A8 mini camera configuration
    CAM_IP = "192.168.144.25"
    CTRL_PORT = 37260
    MEDIA_PORT = 82
    
    # Correct API URLs based on siyi.py working implementation
    BASE_URL = f"http://{CAM_IP}:{MEDIA_PORT}/cgi-bin/media.cgi/api/v1"
    
    # SIYI SDK commands (Based on A8 mini User Manual v1.6)
    # Take Picture command: CMD_ID 0x0C, func_type 0 for photo
    TAKE_PHOTO_4K = bytes.fromhex("55 66 01 01 00 00 00 0c 00 34 ce")
    
    # Photo resolution modes for A8 mini
    PHOTO_RESOLUTIONS = {
        '4K': 0x00,      # 3840x2160 (default)
        '2.7K': 0x01,    # 2704x1520
        '1080P': 0x02    # 1920x1080
    }
    
    def __init__(self):
        super().__init__('siyi_a8_publisher')

        # Publishers
        self.publisher = self.create_publisher(Image, 'image_raw', 10)
        self.status_publisher = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
        self.camera_status_pub = self.create_publisher(String, '/camera/status', 10)

        # Subscribers
        self.create_subscription(Bool, '/camera/trigger', self.camera_trigger_callback, 10)
        self.create_subscription(String, '/camera/set_resolution', self.set_resolution_callback, 10)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.check_altitude, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image', self.sim_image_callback, 1)

        # Initialize cv_bridge if available
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warn("cv_bridge not available, using alternative conversion")
        
        # UDP socket for SDK commands to camera
        self.sdk_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sdk_socket.settimeout(2.0)
        
        # Camera state for SD card operations
        self.current_resolution = '4K'
        self.save_to_sd_card = True
        self.photo_count = 0
        self.current_photo_dir = None
        self.last_photo_count = 0
        self.photo_lock = Lock()
        
        # Initialize SD card directory
        self.initialize_sd_card()

        # Initialize cv_bridge if available
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warn("cv_bridge not available, using alternative conversion")

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
            

        self.get_logger().info(f"Determined ros2_ws directory: {ros2_ws_dir}")
        # Fallback: construct path directly
        if ros2_ws_dir is None:
            ros2_ws_dir = "/astra/ros2_ws/src"
        
        video_cam_dir = os.path.join(ros2_ws_dir, "video_cam")
        os.makedirs(video_cam_dir, exist_ok=True)
        
        # Directory for saving images 
        self.photo_path = os.path.join(video_cam_dir, "camera_feed")
        if not os.path.exists(self.photo_path):
            os.makedirs(self.photo_path)
        
        # Directory for saving mapping images
        self.mapping_photo_path = os.path.join(video_cam_dir, "mapping_photos")
        if not os.path.exists(self.mapping_photo_path):
            os.makedirs(self.mapping_photo_path)
        
        # Real camera flag
        self.use_real_camera = True
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
                self.get_logger().info(f"SD Card mode: {self.save_to_sd_card}, Resolution: {self.current_resolution}")
                self.get_logger().info("=" * 70)
                self.get_logger().info("📡 SUBSCRIPTIONS ACTIVE:")
                self.get_logger().info("   - /camera/trigger (Bool) -> camera_trigger_callback")
                self.get_logger().info("   - /camera/set_resolution (String) -> set_resolution_callback")
                self.get_logger().info("=" * 70)
                self.get_logger().info("To take a photo, publish: ros2 topic pub --once /camera/trigger std_msgs/msg/Bool \"data: true\"")
                self.get_logger().info("To change resolution: ros2 topic pub --once /camera/set_resolution std_msgs/msg/String \"data: '4K'\"")
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

    def calculate_crc16(self, data):
        """Calculate CRC16 for SIYI protocol"""
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
            crc &= 0xFFFF
        return crc
    
    def initialize_sd_card(self):
        """Initialize connection to camera's SD card via HTTP - Using correct API v1 format"""
        # Set defaults first
        self.current_photo_dir = None
        self.last_photo_count = 0
        
        try:
            self.get_logger().info("Connecting to camera HTTP API...")
            
            # Use correct API format: /api/v1/getdirectories with media_type parameter
            url = f"{self.BASE_URL}/getdirectories?media_type=0"
            self.get_logger().info(f"API URL: {url}")
            
            response = requests.get(url, timeout=5)
            
            if response.status_code == 200:
                data = response.json()
                self.get_logger().info(f"✓ API Response: {data}")
                
                # Check if request was successful
                if data.get('success', False):
                    # Get directories from response
                    if 'data' in data and 'directories' in data['data']:
                        directories = data['data']['directories']
                        self.get_logger().info(f"✓ Found {len(directories)} directories")
                        
                        if len(directories) > 0:
                            # Use the most recent directory
                            self.current_photo_dir = directories[-1]['path']
                            self.get_logger().info(f"✓ Using photo directory: {self.current_photo_dir}")
                            
                            # Try to get initial photo count
                            try:
                                count = self.get_photo_count(self.current_photo_dir)
                                if count is not None:
                                    self.last_photo_count = count
                                    self.get_logger().info(f"✓ Initial photo count: {self.last_photo_count}")
                            except Exception as e:
                                self.get_logger().warn(f"Could not get photo count: {e}")
                        else:
                            self.get_logger().warn("⚠ No directories found, using default")
                            self.current_photo_dir = "A:/DCIM/100MEDIA"
                    else:
                        self.get_logger().warn("⚠ No 'data' or 'directories' in response")
                        self.current_photo_dir = "A:/DCIM/100MEDIA"
                else:
                    error_msg = data.get('message', 'Unknown error')
                    self.get_logger().warn(f"⚠ API returned success=False: {error_msg}")
                    self.current_photo_dir = "A:/DCIM/100MEDIA"
            else:
                self.get_logger().warn(f"⚠ HTTP {response.status_code}, using default directory")
                self.current_photo_dir = "A:/DCIM/100MEDIA"
                
        except requests.exceptions.ConnectionError as e:
            self.get_logger().warn("⚠ Cannot connect to camera HTTP server - photos will be triggered but not downloaded")
            self.current_photo_dir = "A:/DCIM/100MEDIA"
        except Exception as e:
            self.get_logger().warn(f"⚠ SD card init error: {type(e).__name__}: {e}")
            self.current_photo_dir = "A:/DCIM/100MEDIA"
        
        self.get_logger().info(f"✓ SD card ready. Directory: {self.current_photo_dir}")
    
    def get_photo_count(self, dir_path):
        """Get count of photos in directory using correct API format"""
        try:
            url = f"{self.BASE_URL}/getmedialist"
            params = {
                'media_type': '0',
                'path': dir_path,
                'start': 0,
                'count': 1  # Just get count, not actual list
            }
            
            response = requests.get(url, params=params, timeout=5)
            
            if response.status_code == 200:
                data = response.json()
                if data.get('success', False) and 'data' in data:
                    total = data['data'].get('total', 0)
                    self.get_logger().info(f"Photo count in {dir_path}: {total}")
                    return total
            return None
        except Exception as e:
            self.get_logger().warn(f"Could not get photo count: {e}")
            return None
    
    def media_command(self, cmd, payload):
        """Send HTTP command to camera's media server"""
        try:
            response = requests.post(
                self.MEDIA_URL,
                params={"cmd": cmd},
                json=payload,
                timeout=5
            )
            
            if response.status_code == 200:
                return response.json()
            else:
                self.get_logger().error(f"Media command '{cmd}' failed: HTTP {response.status_code}")
                return None
        except Exception as e:
            self.get_logger().error(f"Media command '{cmd}' error: {e}")
            return None
    
    def wait_for_new_photo_on_sd(self, timeout_s=15):
        """Wait for new photo to appear on SD card and return its URL - Using correct API"""
        if not self.current_photo_dir:
            self.get_logger().error("No photo directory available")
            return None
        
        start_time = time.time()
        poll_interval = 0.5
        
        self.get_logger().info(f"Polling for new photo (timeout: {timeout_s}s)...")
        
        while (time.time() - start_time) < timeout_s:
            try:
                # Get photo list using correct API format
                url = f"{self.BASE_URL}/getmedialist"
                params = {
                    'media_type': '0',
                    'path': self.current_photo_dir,
                    'start': 0,
                    'count': 9999  # Get all photos
                }
                
                response = requests.get(url, params=params, timeout=5)
                
                if response.status_code == 200:
                    data = response.json()
                    
                    if data.get('success', False) and 'data' in data:
                        total = data['data'].get('total', 0)
                        
                        # Check if new photo appeared
                        if total > self.last_photo_count:
                            self.get_logger().info(f"✓ New photo detected! Count: {self.last_photo_count} -> {total}")
                            
                            # Get the list of files
                            if 'list' in data['data']:
                                file_list = data['data']['list']
                                
                                if len(file_list) > 0:
                                    # Get the most recent photo (last in list)
                                    latest_photo = file_list[-1]
                                    photo_url = latest_photo.get('url', '')
                                    photo_name = latest_photo.get('name', 'unknown')
                                    
                                    # Fix IP address in URL if needed
                                    photo_url = photo_url.replace("192.168.144.25", self.CAM_IP)
                                    
                                    self.last_photo_count = total
                                    self.get_logger().info(f"✓ Photo found: {photo_name}")
                                    self.get_logger().info(f"✓ URL: {photo_url}")
                                    return photo_url
                        else:
                            elapsed = time.time() - start_time
                            self.get_logger().info(f"  Waiting... ({elapsed:.1f}s, count still {total})", throttle_duration_sec=2.0)
                
            except Exception as e:
                self.get_logger().warn(f"Poll error: {e}")
            
            time.sleep(poll_interval)
        
        self.get_logger().warn(f"⚠ Timeout after {timeout_s}s (final count: {self.last_photo_count})")
        return None
    
    def download_photo_from_sd(self, photo_url):
        """Download 4K photo from camera's SD card"""
        try:
            self.get_logger().info(f"Downloading from SD card: {photo_url}")
            response = requests.get(photo_url, timeout=15)
            
            if response.status_code == 200:
                # Convert bytes to numpy array
                img_array = np.frombuffer(response.content, dtype=np.uint8)
                # Decode image
                img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                
                if img is not None:
                    self.get_logger().info(f"✓ Downloaded 4K photo: {img.shape} ({len(response.content)/1024:.1f}KB)")
                    return img
                else:
                    self.get_logger().error("Failed to decode downloaded image")
                    return None
            else:
                self.get_logger().error(f"Download failed: HTTP {response.status_code}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error downloading photo: {e}")
            return None
    
    def capture_and_download_4k_photo(self):
        """Complete workflow: trigger 4K photo, wait, and download from SD"""
        with self.photo_lock:
            # Step 1: Trigger photo on camera
            self.get_logger().info("📸 Step 1: Triggering 4K photo on camera...")
            if not self.trigger_photo_on_sd_card():
                return None
            
            # Step 2: Wait for photo to appear on SD card
            self.get_logger().info("⏳ Step 2: Waiting for photo on SD card...")
            photo_url = self.wait_for_new_photo_on_sd(timeout_s=15)
            if not photo_url:
                self.get_logger().error(" Photo not found on SD card")
                return None
            
            # Step 3: Download photo from SD card
            self.get_logger().info("⬇  Step 3: Downloading 4K photo from SD card...")
            img = self.download_photo_from_sd(photo_url)
            
            if img is not None:
                self.get_logger().info(" Complete: 4K photo captured and downloaded!")
            
            return img
    
    def trigger_photo_on_sd_card(self):
        """Trigger camera to take 4K photo and save to its SD card"""
        try:
            # Send UDP command to camera to take photo
            self.sdk_socket.sendto(self.TAKE_PHOTO_4K, (self.CAM_IP, self.CTRL_PORT))
            self.photo_count += 1
            self.get_logger().info(f"Photo trigger sent to camera SD card (photo #{self.photo_count})")
            
            # Publish status
            status_msg = String()
            status_msg.data = f"Photo captured on SD card in {self.current_resolution} resolution"
            self.camera_status_pub.publish(status_msg)
            
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to trigger photo on SD card: {e}")
            return False
    
    def set_photo_resolution(self, resolution):
        """Set photo resolution mode on camera"""
        if resolution not in self.PHOTO_RESOLUTIONS:
            self.get_logger().warn(f"Invalid resolution: {resolution}. Using 4K.")
            resolution = '4K'
        
        try:
            # Build resolution command packet
            # CMD_ID 0x23 for photo settings
            cmd_id = 0x23
            seq = 0x01
            ctrl = 0x01
            res_value = self.PHOTO_RESOLUTIONS[resolution]
            
            # Packet structure: Header(2) + Ctrl(1) + Seq(1) + DataLen(2) + CMD_ID(1) + Data + CRC(2)
            packet = bytearray([0x55, 0x66, ctrl, seq, 0x01, 0x00, cmd_id, res_value])
            
            # Calculate and append CRC
            crc = self.calculate_crc16(packet)
            packet.extend(struct.pack('<H', crc))
            
            self.sdk_socket.sendto(bytes(packet), (self.CAM_IP, self.CTRL_PORT))
            self.current_resolution = resolution
            self.get_logger().info(f"Photo resolution set to: {resolution}")
            
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to set resolution: {e}")
            return False
    
    def set_resolution_callback(self, msg):
        """Callback for resolution change requests"""
        resolution = msg.data.upper()
        self.set_photo_resolution(resolution)

    def send_ack(self, text):
        msg = StatusText()
        msg.severity = 6  # INFO
        msg.text = text
        self.status_publisher.publish(msg)
        self.get_logger().info(f"Status: {text}")
    
    def camera_trigger_callback(self, msg):
        self.get_logger().info(f"🔔 TRIGGER CALLBACK INVOKED! msg.data={msg.data}")
        if msg.data:
            self.get_logger().info("=" * 70)
            self.get_logger().info("📸 CAMERA TRIGGER RECEIVED - CAPTURING PHOTO")
            self.get_logger().info("=" * 70)
            # Set flag to save on next frame
            self.capture_photo = True
            self.get_logger().info(f"✓ capture_photo flag set to: {self.capture_photo}")
        else:
            self.get_logger().info("⚠ Trigger received but data=False, ignoring")
    
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
                    self.get_logger().info("Camera streaming (waiting for trigger)", throttle_duration_sec=10.0)
                    
                    # Convert to ROS message and publish for live view
                    if self.bridge is not None:
                        imageToTransmit = self.bridge.cv2_to_imgmsg(capturedFrame, encoding='bgr8')
                    else:
                        imageToTransmit = self.cv2_to_imgmsg_manual(capturedFrame, encoding='bgr8')
                    
                    self.publisher.publish(imageToTransmit)
                    
                    # Only save when triggered
                    if self.capture_photo:
                        timestamp = time.strftime("%Y%m%d-%H%M%S")
                        
                        # Capture 4K photo from SD card and download
                        self.get_logger().info("=" * 60)
                        self.get_logger().info(" Starting 4K photo capture from SD card...")
                        
                        img_4k = self.capture_and_download_4k_photo()
                        
                        if img_4k is not None:
                            # Save the downloaded 4K photo to Jetson
                            filename_4k = os.path.join(self.photo_path, f"photo_4K_{timestamp}.jpg")
                            result = cv2.imwrite(filename_4k, img_4k)
                            self.get_logger().info(f" Saved 4K photo to Jetson: {filename_4k}")
                            self.get_logger().info(f"   Size: {os.path.getsize(filename_4k)/1024:.1f}KB, Resolution: {img_4k.shape}")
                            
                            # Also save to mapping folder for detection
                            mapping_filename = os.path.join(self.mapping_photo_path, f"mapping_photo_{timestamp}.jpg")
                            cv2.imwrite(mapping_filename, img_4k)
                            self.get_logger().info(f" Saved to mapping folder: {mapping_filename}")
                        else:
                            self.get_logger().error(" Failed to capture 4K photo from SD card")
                            # Fallback: save RTSP frame
                            filename_rtsp = os.path.join(self.photo_path, f"photo_rtsp_fallback_{timestamp}.jpg")
                            cv2.imwrite(filename_rtsp, capturedFrame)
                            self.get_logger().warn(f"⚠ Saved RTSP fallback frame: {filename_rtsp}")
                        
                        self.get_logger().info("=" * 60)
                        self.capture_photo = False
                else:
                    self.get_logger().warn("Failed to read frame from camera", throttle_duration_sec=10.0)
            else:
                if self.latest_image_msg is not None:
                    self.get_logger().info("Camera Frame Publishing", throttle_duration_sec=10000.0)
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
                        # self.get_logger().info("Capturing photo...")
                        timestamp = time.strftime("%Y%m%d-%H%M%S")
                        mapping_filename = os.path.join(self.mapping_photo_path, f"mapping_photo_{timestamp}.jpg")
                        cv2.imwrite(mapping_filename, cv_image)
                        # self.get_logger().info(f"Photo saved to {mapping_filename}")
                        self.capture_photo = False
                else:
                    self.get_logger().warn("No image received from /webcam/image_raw yet", throttle_duration_sec=5.0)
    
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()