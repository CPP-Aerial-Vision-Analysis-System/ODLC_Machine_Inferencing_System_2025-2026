#!/usr/bin/env python3
#This code is going to run on ubuntu 22.04 and ros2 humble
"""
SIYI A8 Mini Unified Image Capture Pipeline

Single-Node Architecture for deterministic image capture, retrieval, and publication.

This node is the single source of truth for the image pipeline:
- Phase 1: Capture Control (Camera → SD)
- Phase 2: SD Card Indexing (Metadata Only)
- Phase 3: Incremental Download (SD → Jetson)
- Phase 4: ROS Publication (Jetson → ROS Graph)

Design Principles:
- Single owner of the pipeline
- Pull-based, deterministic behavior
- No duplicated work
- Stateless camera, stateful Jetson

AP_FLAKE8_CLEAN
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, String
from mavros_msgs.msg import StatusText
from rclpy.qos import qos_profile_sensor_data

import cv2
import os
import time
import json
import socket
import struct
import requests
import numpy as np
import shutil
import hashlib
from threading import Lock, Event, Thread
from concurrent.futures import ThreadPoolExecutor
from enum import Enum
from typing import Optional, Tuple, List, Dict, Set
from PIL import Image as PILImage

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except Exception as e:
    print(f"Warning: cv_bridge import failed: {e}")
    print("Will attempt to use alternative image conversion methods")
    CV_BRIDGE_AVAILABLE = False
    CvBridge = None


class MediaTypes(Enum):
    """Camera media types"""
    IMAGE = 0
    VIDEO = 1


class CaptureState(Enum):
    """Pipeline execution states"""
    IDLE = "idle"
    CAPTURING = "capturing"
    INDEXING = "indexing"
    DOWNLOADING = "downloading"
    PUBLISHING = "publishing"
    FAILED = "failed"


class SIYIUnifiedPipeline(Node):
    """
    Single-node pipeline for SIYI A8 Mini camera control.
    
    Responsibilities:
    1. Command capture on camera
    2. Query SD card index
    3. Download new images incrementally
    4. Publish to ROS graph
    """
    
    # ========================================================================
    # CAMERA CONFIGURATION
    # ========================================================================
    CAM_IP = "192.168.144.25"
    CTRL_PORT = 37260
    MEDIA_PORT = 82
    
    BASE_URL = f"http://{CAM_IP}:{MEDIA_PORT}/cgi-bin/media.cgi/api/v1"
    
    # SIYI SDK commands (Based on A8 mini User Manual v1.6)
    TAKE_PHOTO_4K = bytes.fromhex("55 66 01 01 00 00 00 0c 00 34 ce")
    
    # Photo resolution modes with capture commands
    PHOTO_RESOLUTIONS = {
        '4K': 0x00,      # 3840x2160 (default)
        '2.7K': 0x01,    # 2704x1520
        '1080P': 0x02    # 1920x1080
    }
    
    # Resolution-specific capture commands
    CAPTURE_COMMANDS = {
        '4K': bytes.fromhex("55 66 01 01 00 00 00 0c 00 34 ce"),
        '2.7K': bytes.fromhex("55 66 01 01 00 00 00 0c 01 35 ce"),  # UNVERIFIED - checksum may be incorrect
        '1080P': bytes.fromhex("55 66 01 01 00 00 00 0c 02 36 ce"),  # UNVERIFIED - checksum may be incorrect
    }
    
    # Resolution verification - only 4K is hardware-verified
    VERIFIED_RESOLUTIONS = {'4K'}  # Only 4K command confirmed working
    USE_UNVERIFIED_RESOLUTIONS = False  # Safety flag - disable unverified resolutions
    
    # Resolution verification specs with file size requirements
    RESOLUTION_SPECS = {
        '4K': {'min_width': 3000, 'min_height': 1600, 'min_file_size': 50000},  # 50KB
        '2.7K': {'min_width': 2000, 'min_height': 1200, 'min_file_size': 30000},  # 30KB
        '1080P': {'min_width': 1800, 'min_height': 900, 'min_file_size': 20000},  # 20KB
    }
    
    # Verification constants
    # 4K is 3840x2160, but allow tolerance for compression artifacts
    MIN_4K_WIDTH = 3000
    MIN_4K_HEIGHT = 1600
    # Minimum JPEG file size - below this indicates corruption
    # Based on testing: smallest valid 4K JPEG observed was ~75KB
    MIN_FILE_SIZE = 50000  # 50KB minimum
    MAX_CAPTURE_RETRIES = 3
    CAPTURE_TIMEOUT = 15.0  # seconds to wait for SD card write
    SD_POLL_INTERVAL = 0.5  # seconds between SD card polls (initial)
    MAX_POLL_INTERVAL = 2.0  # Maximum poll interval for exponential backoff
    MAX_PIPELINE_DURATION = 60.0  # Maximum total pipeline execution time
    
    # Loop timing
    # Video stream rate - balance between latency and bandwidth
    # 10Hz provides smooth preview without overwhelming network
    STREAM_RATE = 10.0  # Hz for video stream publishing
    
    # Disk space monitoring
    MIN_FREE_SPACE_MB = 50  # Minimum free disk space required
    
    def __init__(self):
        super().__init__('siyi_unified_pipeline')
        
        # ====================================================================
        # STATE MANAGEMENT (Jetson is stateful)
        # ====================================================================
        self.pipeline_state = CaptureState.IDLE
        self.state_lock = Lock()  # Lock for pipeline state transitions
        self.download_lock = Lock()  # Lock for downloaded_files tracking
        self.config_lock = Lock()  # Lock for camera_enabled and configuration
        
        # Downloaded file tracking (prevents duplicate downloads)
        # Track (filename, size, timestamp) tuples for better collision detection
        self.downloaded_files: Set[Tuple] = set()
        self.last_download_time: Optional[float] = None
        self.last_publish_time: Optional[float] = None
        
        # SD card state
        self.current_photo_dir: Optional[str] = None
        self.last_photo_count: int = 0
        self.photo_count: int = 0
        
        # Camera settings
        self.current_resolution = '4K'
        self.camera_enabled = True
        self.capture_requested = Event()
        
        # Codec capabilities (determined at startup)
        self.codec_capabilities = {
            'opencv_jpeg': False,
            'pil_available': False
        }
        
        # Worker thread pool for non-blocking pipeline execution
        self.pipeline_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pipeline")
        self.pipeline_future = None
        
        # ====================================================================
        # ROS PARAMETERS
        # ====================================================================
        # Camera enable altitude (meters AGL - Above Ground Level)
        # Negative value indicates below ground level for testing purposes
        # In production, set to positive value (e.g., 10.0 for 10m AGL)
        # Camera disabled below this altitude to prevent ground captures
        self.declare_parameter('min_altitude_agl', -13.716)
        self.declare_parameter('camera_ip', '192.168.144.25')
        self.declare_parameter('ctrl_port', 37260)
        self.declare_parameter('media_port', 82)
        self.declare_parameter('rtsp_port', 8554)
        self.declare_parameter('http_timeout_sec', 10.0)
        self.declare_parameter('capture_timeout_sec', 15.0)
        self.declare_parameter('min_free_space_mb', 50.0)
        
        # Get parameters
        self.altitude_threshold = self.get_parameter('min_altitude_agl').value
        self.CAM_IP = self.get_parameter('camera_ip').value
        self.CTRL_PORT = self.get_parameter('ctrl_port').value
        self.MEDIA_PORT = self.get_parameter('media_port').value
        self.http_timeout = self.get_parameter('http_timeout_sec').value
        self.CAPTURE_TIMEOUT = self.get_parameter('capture_timeout_sec').value
        self.min_free_space_mb = self.get_parameter('min_free_space_mb').value
        
        # Update BASE_URL with configured parameters
        self.BASE_URL = f"http://{self.CAM_IP}:{self.MEDIA_PORT}/cgi-bin/media.cgi/api/v1"
        
        # ====================================================================
        # ROS INTERFACE
        # ====================================================================
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.status_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
        self.camera_status_pub = self.create_publisher(String, '/camera/status', 10)
        self.disk_status_pub = self.create_publisher(Float64, '/camera/disk_free_mb', 10)
        
        # Subscribers
        self.create_subscription(Bool, '/camera/trigger', 
                                self.camera_trigger_callback, 10)
        self.create_subscription(String, '/camera/set_resolution', 
                                self.set_resolution_callback, 10)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', 
                                self.altitude_callback, qos_profile_sensor_data)
        
        # ====================================================================
        # HARDWARE INTERFACE
        # ====================================================================
        
        # SDK socket for camera control
        self.sdk_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sdk_socket.settimeout(2.0)
        
        # HTTP session with connection pooling
        self.http_session = requests.Session()
        self.http_session.headers.update({
            'User-Agent': 'SIYI-ROS-Client/1.0',
            'Connection': 'keep-alive'
        })
        adapter = requests.adapters.HTTPAdapter(
            pool_connections=1,  # Only 1 host (camera)
            pool_maxsize=3,      # Keep 3 connections alive
            max_retries=0        # We'll handle retries ourselves
        )
        self.http_session.mount('http://', adapter)
        
        # Video stream capture (for live preview)
        self.video_capture: Optional[cv2.VideoCapture] = None
        
        # CV Bridge
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warn("cv_bridge not available, using alternative conversion")
        
        # ====================================================================
        # LOCAL STORAGE
        # ====================================================================
        
        # Determine workspace root
        ros2_ws_dir = self._find_ros2_workspace()
        video_cam_dir = os.path.join(ros2_ws_dir, "video_cam")
        os.makedirs(video_cam_dir, exist_ok=True)
        
        # Storage directories
        self.download_dir = os.path.join(video_cam_dir, "downloaded_images")
        self.camera_feed_dir = os.path.join(video_cam_dir, "camera_feed")
        self.mapping_dir = os.path.join(video_cam_dir, "mapping_photos")
        
        for directory in [self.download_dir, self.camera_feed_dir, self.mapping_dir]:
            os.makedirs(directory, exist_ok=True)
        
        # Persistent tracking file for downloaded files
        self.tracking_file = os.path.join(self.download_dir, '.tracking_state.json')
        
        self.get_logger().info("=" * 70)
        self.get_logger().info(" SIYI UNIFIED PIPELINE INITIALIZED")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f" Storage paths:")
        self.get_logger().info(f"   Downloads:      {self.download_dir}")
        self.get_logger().info(f"   Camera feed:    {self.camera_feed_dir}")
        self.get_logger().info(f"   Mapping photos: {self.mapping_dir}")
        self.get_logger().info("=" * 70)
        
        # ====================================================================
        # INITIALIZATION
        # ====================================================================
        
        # Initialize camera connection
        self._initialize_camera()
        
        # Check OpenCV JPEG support
        self._check_image_codec_support()
        
        # Load persistent tracking state
        self._load_tracking_state()
        
        # Initialize SD card indexing
        self._initialize_sd_card()
        
        # Start main pipeline loop
        stream_period = 1.0 / self.STREAM_RATE
        self.pipeline_timer = self.create_timer(stream_period, self._pipeline_loop)
        
        self.get_logger().info(" Pipeline ready. Waiting for triggers...")
        self.get_logger().info("=" * 70)
    
    # ========================================================================
    # INITIALIZATION METHODS
    # ========================================================================
    
    def _find_ros2_workspace(self) -> str:
        """Locate the ROS2 workspace root directory"""
        current_file = os.path.abspath(__file__)
        search_dir = os.path.dirname(current_file)
        
        for _ in range(10):  # Limit search depth
            if (os.path.exists(os.path.join(search_dir, "install")) and 
                os.path.exists(os.path.join(search_dir, "src"))):
                return os.path.join(search_dir, "src")
            
            search_dir = os.path.dirname(search_dir)
            if search_dir == "/":
                break
        
        # Fallback
        fallback = "/astra/ros2_ws/src"
        self.get_logger().warn(f"Could not determine ros2_ws, using fallback: {fallback}")
        return fallback
    
    def _initialize_camera(self):
        """Initialize RTSP video stream connection"""
        rtsp_port = self.get_parameter('rtsp_port').value
        rtsp_url = f'rtsp://{self.CAM_IP}:{rtsp_port}/main.264'
        
        self.get_logger().info(f" Connecting to camera at {rtsp_url}...")
        
        # Try FFmpeg backend first
        self.video_capture = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        
        # Configure timeouts
        if self.video_capture.isOpened():
            self.video_capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            # Try to set timeouts (may not work on all backends)
            try:
                timeout_ms = int(self.http_timeout * 1000)  # Convert to milliseconds
                self.video_capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, timeout_ms)
                self.video_capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, timeout_ms)
                self.get_logger().info(f" ✓ Video capture timeouts set to {timeout_ms}ms")
            except (AttributeError, Exception) as e:
                # Not all backends support timeout properties
                self.get_logger().debug(f" Video capture timeout not supported by backend: {e}")
                self.get_logger().warn(" ⚠ Video stream timeouts not available - stream may hang on network issues")
        
        if not self.video_capture.isOpened():
            self.get_logger().warn(" FFmpeg failed, trying GStreamer...")
            gst_pipeline = (
                f'rtspsrc location={rtsp_url} latency=0 ! '
                'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink'
            )
            self.video_capture = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.video_capture.isOpened():
            self.get_logger().warn(" GStreamer failed, trying default backend...")
            self.video_capture = cv2.VideoCapture(rtsp_url)
        
        if self.video_capture.isOpened():
            self.get_logger().info(" ✓ Camera video stream connected")
            self._send_status("Camera video stream initialized")
        else:
            self.get_logger().error(" ✗ Failed to connect to camera video stream")
            self._send_status("WARNING: Camera video stream unavailable")
    
    def _check_image_codec_support(self):
        """Check if OpenCV can write JPEG files, store capabilities"""
        try:
            # Create a small test image
            test_img = np.zeros((10, 10, 3), dtype=np.uint8)
            
            # Test in the actual download directory
            test_path = os.path.join(self.download_dir, "opencv_test.jpg")
            
            # Test OpenCV JPEG support
            try:
                success = cv2.imwrite(test_path, test_img)
                
                if success and os.path.exists(test_path):
                    self.codec_capabilities['opencv_jpeg'] = True
                    self.get_logger().info(" ✓ OpenCV JPEG support: OK")
                    os.remove(test_path)
                else:
                    self.codec_capabilities['opencv_jpeg'] = False
                    self.get_logger().warn(" ⚠ OpenCV JPEG support: WRITE FAILED")
            except Exception as cv_error:
                self.codec_capabilities['opencv_jpeg'] = False
                self.get_logger().warn(f" ⚠ OpenCV JPEG support: EXCEPTION - {cv_error}")
            
            # Test PIL availability
            try:
                from PIL import Image as PILImage
                self.codec_capabilities['pil_available'] = True
                self.get_logger().info(" ✓ PIL/Pillow support: OK")
            except ImportError:
                self.codec_capabilities['pil_available'] = False
                self.get_logger().warn(" ⚠ PIL/Pillow not available")
            
            # Log summary
            self.get_logger().info(f" Codec capabilities: {self.codec_capabilities}")
            
            # Fail fast if no JPEG codecs available
            if not any(self.codec_capabilities.values()):
                raise RuntimeError("No JPEG codec available - cannot save images!")
            
            # Warn if fallback required
            if not self.codec_capabilities['opencv_jpeg'] and self.codec_capabilities['pil_available']:
                self.get_logger().warn(" → Will use PIL/Pillow fallback for JPEG files")
                
        except RuntimeError:
            raise
        except Exception as e:
            self.get_logger().warn(f" ⚠ Codec check failed: {e}")
            # Assume OpenCV works as fallback
            self.codec_capabilities['opencv_jpeg'] = True
    
    def _initialize_sd_card(self):
        """
        Phase 2: SD Card Indexing (Metadata Only)
        
        Query the camera's SD card to establish initial state.
        This is read-only - we never modify SD card contents.
        """
        self.get_logger().info(" Initializing SD card index...")
        
        try:
            # Query directory list
            url = f"{self.BASE_URL}/getdirectories?media_type={MediaTypes.IMAGE.value}"
            response = self.http_session.get(url, timeout=self.http_timeout)
            
            if response.status_code == 200:
                data = response.json()
                
                if data.get('success', False):
                    directories = data.get('data', {}).get('directories', [])
                    
                    if len(directories) > 0:
                        # Use most recent directory
                        self.current_photo_dir = directories[-1]['path']
                        self.get_logger().info(f" Photo directory: {self.current_photo_dir}")
                        
                        # Get initial file count ONLY if no tracking state exists
                        # If tracking state exists, we already have the correct last_photo_count
                        if not os.path.exists(self.tracking_file):
                            count = self._get_sd_photo_count(self.current_photo_dir)
                            if count is not None:
                                self.last_photo_count = count
                                self.get_logger().info(f" Initial photo count: {count}")
                            
                            # First run: mark existing SD files as seen
                            self._load_existing_sd_files()
                            self.get_logger().info(" First run: marked existing SD files as seen")
                        else:
                            # Tracking state loaded, preserve its last_photo_count
                            self.get_logger().info(
                                f" Using tracking state photo count: {self.last_photo_count}"
                            )
                            self.get_logger().info(" New SD files will be processed on next capture")
                    else:
                        self.get_logger().warn(" No directories found, using default")
                        self.current_photo_dir = "A:/DCIM/100MEDIA"
                else:
                    self.get_logger().warn(f" API error: {data.get('message', 'Unknown')}")
                    self.current_photo_dir = "A:/DCIM/100MEDIA"
            else:
                self.get_logger().warn(f" HTTP {response.status_code}, using default")
                self.current_photo_dir = "A:/DCIM/100MEDIA"
        
        except requests.exceptions.ConnectionError:
            self.get_logger().warn(" Cannot connect to camera HTTP API")
            self.current_photo_dir = "A:/DCIM/100MEDIA"
        except Exception as e:
            self.get_logger().warn(f" SD card init error: {e}")
            self.current_photo_dir = "A:/DCIM/100MEDIA"
        
        self.get_logger().info(" SD card indexing initialized")
    
    def _load_existing_sd_files(self):
        """Load existing SD card files into tracking set to avoid reprocessing"""
        try:
            files = self._query_sd_card_index()
            with self.download_lock:
                for file_info in files:
                    filename = file_info.get('name', '')
                    size = file_info.get('size', 0)
                    timestamp = file_info.get('create_time', 0)
                    if filename:
                        # Track using composite key (filename, size, timestamp)
                        file_key = (filename, size, timestamp)
                        self.downloaded_files.add(file_key)
            
            self.get_logger().info(f" ✓ Loaded {len(self.downloaded_files)} existing files into tracking")
        except Exception as e:
            self.get_logger().warn(f" Could not load existing files: {e}")
    
    def _load_tracking_state(self):
        """Load persistent tracking state from disk to survive node restarts"""
        try:
            if os.path.exists(self.tracking_file):
                with open(self.tracking_file, 'r') as f:
                    data = json.load(f)
                
                # Restore downloaded files set (stored as list of tuples)
                with self.download_lock:
                    self.downloaded_files = set(
                        tuple(item) for item in data.get('downloaded_files', [])
                    )
                
                # Restore last photo count
                self.last_photo_count = data.get('last_photo_count', 0)
                
                last_operation = data.get('last_operation_time', 'unknown')
                self.get_logger().info(
                    f" ✓ Loaded tracking state: {len(self.downloaded_files)} files, "
                    f"last count: {self.last_photo_count}, last operation: {last_operation}"
                )
            else:
                self.get_logger().info(" No tracking state file found - starting fresh")
        except Exception as e:
            self.get_logger().warn(f" Could not load tracking state: {e} - starting fresh")
    
    def _save_tracking_state(self):
        """Save persistent tracking state to disk with pruning (atomic write)"""
        try:
            # Prune old entries - keep only last 500 captures to prevent unbounded growth
            MAX_TRACKED_FILES = 500
            
            with self.download_lock:
                if len(self.downloaded_files) > MAX_TRACKED_FILES:
                    # Convert to sorted list by timestamp (index 2 of tuple)
                    sorted_files = sorted(
                        self.downloaded_files,
                        key=lambda x: x[2] if len(x) > 2 else 0,
                        reverse=True  # Newest first
                    )
                    # Keep only newest entries
                    old_count = len(self.downloaded_files)
                    self.downloaded_files = set(sorted_files[:MAX_TRACKED_FILES])
                    
                    self.get_logger().info(
                        f" Pruned tracking state: {old_count} → {MAX_TRACKED_FILES} files"
                    )
            
            data = {
                'downloaded_files': [list(item) for item in self.downloaded_files],
                'last_photo_count': self.last_photo_count,
                'last_operation_time': time.strftime("%Y-%m-%d %H:%M:%S"),
                'photo_count': self.photo_count
            }
            
            # Atomic write: write to temp file, then rename
            tmp_file = self.tracking_file + '.tmp'
            with open(tmp_file, 'w') as f:
                json.dump(data, f, indent=2)
            
            os.replace(tmp_file, self.tracking_file)
            self.get_logger().debug(f" ✓ Saved tracking state: {len(self.downloaded_files)} files")
        except Exception as e:
            self.get_logger().warn(f" Could not save tracking state: {e}")
    
    # ========================================================================
    # PHASE 1: CAPTURE CONTROL (Camera → SD)
    # ========================================================================
    
    def _trigger_capture(self) -> bool:
        """
        Command the camera to capture and save to SD card with ACK validation.
        
        Returns:
            True if command sent successfully and ACK received
        """
        try:
            # Resolution verification safety check
            if self.current_resolution not in self.VERIFIED_RESOLUTIONS:
                if not self.USE_UNVERIFIED_RESOLUTIONS:
                    self.get_logger().warn(
                        f" Resolution {self.current_resolution} not verified - using 4K for safety"
                    )
                    capture_command = self.CAPTURE_COMMANDS['4K']
                    effective_resolution = '4K'
                else:
                    self.get_logger().warn(
                        f" ⚠ Using UNVERIFIED resolution command: {self.current_resolution}"
                    )
                    capture_command = self.CAPTURE_COMMANDS.get(
                        self.current_resolution, 
                        self.TAKE_PHOTO_4K
                    )
                    effective_resolution = self.current_resolution
            else:
                capture_command = self.CAPTURE_COMMANDS.get(
                    self.current_resolution,
                    self.TAKE_PHOTO_4K
                )
                effective_resolution = self.current_resolution
            
            # Send capture command via UDP
            self.sdk_socket.sendto(capture_command, (self.CAM_IP, self.CTRL_PORT))
            
            # Wait for ACK (non-blocking with timeout)
            self.sdk_socket.settimeout(2.0)  # 2 second timeout for ACK
            
            try:
                response, addr = self.sdk_socket.recvfrom(1024)
                
                # Parse response (SIYI protocol: header + cmd_id + status + checksum)
                if len(response) >= 10:
                    # Check if it's an ACK for photo capture
                    # SIYI protocol typically: header (2 bytes) + cmd_id (1 byte) + ... + status
                    cmd_id = response[2] if len(response) > 2 else 0
                    status = response[6] if len(response) > 6 else 0
                    
                    if cmd_id == 0x0c and status == 0x00:  # Success
                        self.get_logger().info(" ✓ Camera ACK: Capture confirmed")
                    else:
                        self.get_logger().warn(
                            f" ⚠ Camera response: cmd_id={cmd_id:02x}, status={status:02x}"
                        )
                else:
                    self.get_logger().warn(" ⚠ Invalid ACK length, assuming success")
                    
            except socket.timeout:
                self.get_logger().warn(
                    " ⚠ No ACK received from camera (timeout), assuming success and continuing"
                )
                # Still continue - camera might not always send ACK
            
            with self.state_lock:
                self.photo_count += 1
                current_count = self.photo_count
            
            self.get_logger().info(
                f" [Phase 1] Capture command sent: {effective_resolution} (photo #{current_count})"
            )
            return True
            
        except Exception as e:
            self.get_logger().error(f" [Phase 1] Capture command failed: {e}")
            return False
    
    # ========================================================================
    # PHASE 2: SD CARD INDEXING (Metadata Only)
    # ========================================================================
    
    def _query_sd_card_index(self) -> List[Dict]:
        """
        Query SD card for current file list.
        
        Returns:
            List of file info dictionaries with 'name', 'url', 'size', 'create_time', etc.
        """
        if not self.current_photo_dir:
            return []
        
        try:
            url = f"{self.BASE_URL}/getmedialist"
            params = {
                'media_type': str(MediaTypes.IMAGE.value),
                'path': self.current_photo_dir,
                'start': 0,
                'count': 9999
            }
            
            response = self.http_session.get(url, params=params, timeout=self.http_timeout)
            
            if response.status_code == 200:
                data = response.json()
                if data.get('success', False):
                    file_list = data.get('data', {}).get('list', [])
                    return file_list
            
            return []
            
        except Exception as e:
            self.get_logger().warn(f" SD index query error: {e}")
            return []
    
    def _get_sd_photo_count(self, dir_path: str) -> Optional[int]:
        """Get total count of photos in directory"""
        try:
            url = f"{self.BASE_URL}/getmedialist"
            params = {
                'media_type': str(MediaTypes.IMAGE.value),
                'path': dir_path,
                'start': 0,
                'count': 1
            }
            
            response = self.http_session.get(url, params=params, timeout=self.http_timeout)
            
            if response.status_code == 200:
                data = response.json()
                if data.get('success', False):
                    return data.get('data', {}).get('total', 0)
            
            return None
            
        except Exception as e:
            self.get_logger().warn(f" Photo count query error: {e}")
            return None
    
    def _wait_for_new_image_on_sd(self, timeout: float) -> Optional[Dict]:
        """
        Poll SD card until a new image appears (with exponential backoff and directory rollover detection).
        
        Returns:
            File info dict for new image, or None if timeout
        """
        start_time = time.time()
        poll_interval = self.SD_POLL_INTERVAL  # Start with initial interval
        consecutive_empty_polls = 0  # Track consecutive polls with no new images
        
        self.get_logger().info(f" [Phase 2] Polling SD card for new image (timeout: {timeout}s)...")
        
        while (time.time() - start_time) < timeout:
            try:
                file_list = self._query_sd_card_index()
                current_count = len(file_list)
                
                # DIRECTORY ROLLOVER DETECTION
                # If we've polled several times and count hasn't increased,
                # camera might have created a new directory (100MEDIA → 101MEDIA)
                if current_count == self.last_photo_count:
                    consecutive_empty_polls += 1
                    
                    # After 5 failed polls, check for new directory
                    if consecutive_empty_polls >= 5:
                        self.get_logger().info(
                            " No new images in current directory, checking for directory rollover..."
                        )
                        
                        if self._check_and_update_directory():
                            # Directory changed, reset poll counter and re-query
                            consecutive_empty_polls = 0
                            continue
                else:
                    # Count changed, reset empty poll counter
                    consecutive_empty_polls = 0
                
                # Check if count increased
                if current_count > self.last_photo_count:
                    self.get_logger().info(
                        f" ✓ New image detected on SD! Count: {self.last_photo_count} → {current_count}"
                    )
                    
                    # Find the new file(s) using composite key
                    with self.download_lock:
                        for file_info in reversed(file_list):  # Start from most recent
                            filename = file_info.get('name', '')
                            size = file_info.get('size', 0)
                            timestamp = file_info.get('create_time', 0)
                            file_key = (filename, size, timestamp)
                            
                            if filename and file_key not in self.downloaded_files:
                                self.last_photo_count = current_count
                                return file_info
                
                # Log progress
                elapsed = time.time() - start_time
                self.get_logger().info(
                    f"  Waiting... ({elapsed:.1f}s, count: {current_count})",
                    throttle_duration_sec=2.0
                )
                
            except Exception as e:
                self.get_logger().warn(f" SD poll error: {e}")
            
            time.sleep(poll_interval)
            # Exponential backoff - gradually slow down polling
            poll_interval = min(poll_interval * 1.5, self.MAX_POLL_INTERVAL)
        
        self.get_logger().error(f" ✗ Timeout: No new image after {timeout}s")
        return None
    
    def _check_and_update_directory(self) -> bool:
        """
        Check if camera created new directory and update if so.
        
        Handles SD card directory rollover (100MEDIA → 101MEDIA after ~999 files).
        
        Returns:
            True if directory changed, False otherwise
        """
        try:
            url = f"{self.BASE_URL}/getdirectories?media_type={MediaTypes.IMAGE.value}"
            response = self.http_session.get(url, timeout=self.http_timeout)
            
            if response.status_code == 200:
                data = response.json()
                if data.get('success', False):
                    directories = data.get('data', {}).get('directories', [])
                    
                    if len(directories) > 0:
                        latest_dir = directories[-1]['path']
                        
                        if latest_dir != self.current_photo_dir:
                            old_dir = self.current_photo_dir
                            self.current_photo_dir = latest_dir
                            
                            # Reset photo count for new directory
                            count = self._get_sd_photo_count(self.current_photo_dir)
                            if count is not None:
                                self.last_photo_count = count
                            
                            self.get_logger().info(
                                f" ✓ Directory rollover detected: {old_dir} → {latest_dir}"
                            )
                            self.get_logger().info(f" New directory photo count: {count}")
                            
                            return True  # Directory changed
            
            return False  # No change
            
        except Exception as e:
            self.get_logger().warn(f" Directory rollover check failed: {e}")
            return False
    
    # ========================================================================
    # PHASE 3: INCREMENTAL DOWNLOAD (SD → Jetson)
    # ========================================================================
    
    def _check_disk_space(self, required_mb: float = 10.0) -> bool:
        """Check if sufficient disk space is available"""
        try:
            stat = shutil.disk_usage(self.download_dir)
            free_mb = stat.free / (1024 * 1024)
            
            # Publish disk status
            msg = Float64()
            msg.data = free_mb
            self.disk_status_pub.publish(msg)
            
            if free_mb < required_mb:
                self.get_logger().error(
                    f" ✗ Disk space critical: {free_mb:.1f}MB free (need {required_mb:.1f}MB)"
                )
                return False
            
            return True
        except Exception as e:
            self.get_logger().warn(f" Could not check disk space: {e}")
            return True  # Assume OK if check fails
    
    def _download_image_from_sd(self, file_info: Dict) -> Optional[Tuple[str, np.ndarray]]:
        """
        Download image from SD card via HTTP.
        
        Args:
            file_info: File metadata from SD card index
        
        Returns:
            Tuple of (filename, image_array) or None if failed
        """
        try:
            filename = file_info.get('name', '')
            file_url = file_info.get('url', '')
            file_size = file_info.get('size', 0)
            
            if not filename or not file_url:
                self.get_logger().error(
                    f" ✗ [Phase 3] Invalid file info (missing name or URL)"
                )
                return None
            
            # Check disk space before downloading (need at least 2x file size)
            required_mb = max(self.min_free_space_mb, (file_size * 2) / (1024 * 1024))
            if not self._check_disk_space(required_mb):
                raise Exception(f"Insufficient disk space (need {required_mb:.1f}MB)")
            
            # Fix IP address in URL if needed
            file_url = file_url.replace("192.168.144.25", self.CAM_IP)
            
            self.get_logger().info(f" [Phase 3] Downloading: {filename}")
            self.get_logger().info(f"   URL: {file_url}")
            self.get_logger().info(f"   Size: {file_size / 1024:.1f}KB")
            
            # Download image (use configured timeout)
            response = self.http_session.get(file_url, timeout=self.http_timeout)
            
            if response.status_code != 200:
                self.get_logger().error(
                    f" ✗ [Phase 3] Download failed: {filename} from {file_url}\n"
                    f"    HTTP {response.status_code}: {response.reason}"
                )
                return None
            
            # Decode image
            img_array = np.frombuffer(response.content, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            
            if img is None:
                self.get_logger().error(f" ✗ [Phase 3] Failed to decode image: {filename}")
                return None
            
            # Verify dimensions
            h, w = img.shape[:2]
            size_kb = len(response.content) / 1024
            
            self.get_logger().info(f" ✓ Downloaded: {w}x{h}, {size_kb:.1f}KB")
            
            if not self._verify_image_dimensions(img):
                self.get_logger().warn(
                    f" ⚠ Image dimensions {w}x{h} below expected threshold for {self.current_resolution}"
                )
            
            # Verify image integrity
            if not self._verify_image_integrity(img):
                self.get_logger().warn(f" ⚠ Image integrity check failed: {filename}")
            
            return (filename, img)
            
        except Exception as e:
            self.get_logger().error(f" ✗ [Phase 3] Download error for {filename}: {e}")
            return None
    
    def _verify_image_dimensions(self, img: np.ndarray) -> bool:
        """Verify image meets minimum dimension requirements for current resolution"""
        if img is None:
            return False
        
        try:
            h, w = img.shape[:2]
            
            # Get specs for current resolution
            specs = self.RESOLUTION_SPECS.get(self.current_resolution)
            if specs:
                if w >= specs['min_width'] and h >= specs['min_height']:
                    return True
                else:
                    self.get_logger().warn(
                        f" Dimensions {w}x{h} below {self.current_resolution} threshold "
                        f"({specs['min_width']}x{specs['min_height']})"
                    )
                    return False
            
            # Fallback: just check that it's not too small
            return w > 640 and h > 480
            
        except Exception:
            return False
    
    def _verify_image_integrity(self, img: np.ndarray) -> bool:
        """
        Verify image is not corrupted or blank.
        
        Uses warnings instead of hard failures for brightness extremes
        (legitimate night shots or snow scenes can trigger false positives).
        """
        try:
            if img is None:
                return False
            
            h, w = img.shape[:2]
            
            # Sanity check dimensions (HARD FAILURE)
            if h < 100 or w < 100:
                self.get_logger().error(f" ✗ Image too small: {w}x{h}")
                return False
            
            # Check for all black or all white (WARNING ONLY - not failure)
            # These could be legitimate images (night shots, snow, clouds)
            mean_val = np.mean(img)
            if mean_val < 5:
                self.get_logger().warn(
                    f" ⚠ Image appears very dark (mean: {mean_val:.1f}) - "
                    "could be legitimate night shot or error"
                )
                # Don't fail - just warn
            elif mean_val > 250:
                self.get_logger().warn(
                    f" ⚠ Image appears very bright (mean: {mean_val:.1f}) - "
                    "could be snow/clouds or overexposure"
                )
                # Don't fail - just warn
            
            return True  # Always pass unless dimension check fails
            
        except Exception as e:
            self.get_logger().warn(f" Integrity check error: {e}")
            return True  # Don't fail on check errors
    
    def _save_image_locally(self, filename: str, img: np.ndarray) -> bool:
        """
        Save downloaded image to local storage using symbolic links.
        
        Saves once to download_dir (master copy), then creates symlinks in:
        - camera_feed_dir: For general processing
        - mapping_dir: For mapping-specific processing
        
        This avoids storing the same image 3 times, saving disk space.
        
        Returns:
            True if save and symlink creation successful
        """
        try:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            base_name = f"photo_{timestamp}_{filename}"
            
            # Master copy path (single storage location)
            master_path = os.path.join(self.download_dir, filename)
            
            # Symlink paths
            camera_feed_link = os.path.join(self.camera_feed_dir, base_name)
            mapping_link = os.path.join(self.mapping_dir, f"mapping_{base_name}")
            
            self.get_logger().info(f" [Phase 3] Saving to local storage...")
            
            # Write master copy atomically
            if not self._atomic_write(master_path, img):
                self.get_logger().error(f" ✗ Failed to write master copy: {master_path}")
                return False
            
            # Verify master file
            if not self._verify_local_file(master_path):
                self.get_logger().error(f" ✗ Master file verification failed: {master_path}")
                return False
            
            # Create symbolic links
            try:
                # Remove existing links if present
                for link_path in [camera_feed_link, mapping_link]:
                    if os.path.exists(link_path) or os.path.islink(link_path):
                        os.remove(link_path)
                
                # Create new symlinks
                os.symlink(master_path, camera_feed_link)
                os.symlink(master_path, mapping_link)
                
                self.get_logger().info(
                    f" ✓ Saved master copy and created symlinks\n"
                    f"   Master: {master_path}\n"
                    f"   Symlinks: {camera_feed_link}, {mapping_link}"
                )
            except Exception as link_error:
                self.get_logger().warn(
                    f" ⚠ Symlink creation failed: {link_error}\n"
                    f"   Master copy saved successfully at: {master_path}"
                )
                # Don't fail the whole operation if symlinks fail
            
            self.last_download_time = time.time()
            return True
            
        except Exception as e:
            self.get_logger().error(f" ✗ [Phase 3] Save error: {e}")
            return False
    
    def _atomic_write(self, filepath: str, img: np.ndarray) -> bool:
        """Write image file atomically (temp file + rename)"""
        tmp_path = None
        try:
            tmp_path = filepath + ".tmp"
            
            # Check if this is a JPEG file
            file_ext = os.path.splitext(filepath)[1].lower()
            is_jpeg = file_ext in ['.jpg', '.jpeg']
            
            success = False
            
            # Choose codec based on capabilities
            if is_jpeg:
                # Use best available JPEG codec
                if self.codec_capabilities.get('opencv_jpeg', False):
                    # OpenCV JPEG support available
                    success = cv2.imwrite(tmp_path, img)
                    if not success:
                        self.get_logger().debug(f" cv2.imwrite returned False for {tmp_path}")
                elif self.codec_capabilities.get('pil_available', False):
                    # Use PIL directly (don't try OpenCV first)
                    self.get_logger().debug(f" Using PIL for JPEG write: {filepath}")
                    try:
                        # Convert BGR (OpenCV) to RGB (PIL)
                        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        pil_img = PILImage.fromarray(img_rgb)
                        pil_img.save(tmp_path, 'JPEG', quality=95)
                        success = True
                    except Exception as pil_error:
                        self.get_logger().error(f" PIL write failed: {pil_error}")
                        return False
                else:
                    # No JPEG codec available
                    raise RuntimeError("No JPEG codec available")
            else:
                # Non-JPEG file, use OpenCV
                success = cv2.imwrite(tmp_path, img)
            
            if not success:
                self.get_logger().error(f" Image write failed for {tmp_path}")
                return False
            
            # Verify temp file before committing
            if not self._verify_local_file(tmp_path):
                try:
                    os.remove(tmp_path)
                except:
                    pass
                return False
            
            # Atomic rename
            os.replace(tmp_path, filepath)
            return True
            
        except Exception as e:
            self.get_logger().error(f" Atomic write error for {filepath}: {e}")
            try:
                tmp_path = filepath + ".tmp"
                if os.path.exists(tmp_path):
                    os.remove(tmp_path)
            except:
                pass
            return False
    
    def _verify_local_file(self, path: str, min_bytes: Optional[int] = None) -> bool:
        """Verify local file exists and has valid size (resolution-dependent)"""
        if min_bytes is None:
            # Use resolution-specific minimum file size
            specs = self.RESOLUTION_SPECS.get(self.current_resolution, {})
            min_bytes = specs.get('min_file_size', self.MIN_FILE_SIZE)
        
        try:
            if not os.path.exists(path):
                return False
            
            size = os.path.getsize(path)
            if size < min_bytes:
                self.get_logger().warn(
                    f" File size {size} bytes below minimum {min_bytes} bytes for {self.current_resolution}"
                )
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().warn(f" File verification error: {e}")
            return False
    
    # ========================================================================
    # PHASE 4: ROS PUBLICATION (Jetson → ROS Graph)
    # ========================================================================
    
    def _publish_image_to_ros(self, filename: str, file_info: Dict, img: np.ndarray):
        """
        Publish image to ROS graph with proper metadata.
        
        Args:
            filename: Original filename for tracking
            file_info: File metadata dict with size and timestamp
            img: Image array to publish
        """
        try:
            self.get_logger().info(f" [Phase 4] Publishing to ROS: {filename}")
            
            # Convert to ROS message
            if self.bridge is not None:
                msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            else:
                msg = self._cv2_to_imgmsg_manual(img, encoding='bgr8')
            
            # Set timestamp
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            
            # Publish
            self.image_pub.publish(msg)
            self.last_publish_time = time.time()
            
            self.get_logger().info(f" ✓ Published: {img.shape[1]}x{img.shape[0]}")
            
            # Mark as processed using composite key
            with self.download_lock:
                size = file_info.get('size', 0)
                timestamp = file_info.get('create_time', 0)
                file_key = (filename, size, timestamp)
                self.downloaded_files.add(file_key)
            
        except Exception as e:
            self.get_logger().error(f" ✗ [Phase 4] Publish error for {filename}: {e}")
    
    def _cv2_to_imgmsg_manual(self, cv_image: np.ndarray, encoding: str = 'bgr8') -> Image:
        """Convert OpenCV image to ROS message without cv_bridge"""
        msg = Image()
        msg.height = cv_image.shape[0]
        msg.width = cv_image.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = cv_image.shape[1] * cv_image.shape[2]
        msg.data = cv_image.tobytes()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        return msg
    
    # ========================================================================
    # COMPLETE PIPELINE EXECUTION
    # ========================================================================
    
    def _execute_capture_pipeline(self) -> bool:
        """
        Execute complete 4-phase capture pipeline with proper state management.
        
        Returns:
            True if entire pipeline succeeded
        """
        # Atomic state check and transition - keep lock held
        with self.state_lock:
            if self.pipeline_state != CaptureState.IDLE:
                self.get_logger().warn(" Pipeline already running, skipping request")
                return False
            # Set state while holding lock - prevents race condition
            self.pipeline_state = CaptureState.CAPTURING
        
        # Track pipeline start time for maximum timeout
        pipeline_start = time.time()
        file_info = None  # Track for error recovery
        saved_last_count = self.last_photo_count  # For rollback
        
        try:
            self.get_logger().info("=" * 70)
            self.get_logger().info(" STARTING IMAGE CAPTURE PIPELINE")
            self.get_logger().info("=" * 70)
            
            # Check maximum pipeline time before each phase
            def check_timeout():
                elapsed = time.time() - pipeline_start
                if elapsed > self.MAX_PIPELINE_DURATION:
                    raise Exception(f"Pipeline timeout: exceeded {self.MAX_PIPELINE_DURATION}s")
            
            # Phase 1: Trigger capture
            check_timeout()
            if not self._trigger_capture():
                raise Exception("Phase 1 failed: Could not trigger capture")
            
            # Brief wait for camera to write to SD
            time.sleep(0.5)
            
            # Phase 2: Poll SD card for new image
            check_timeout()
            with self.state_lock:
                self.pipeline_state = CaptureState.INDEXING
            
            file_info = self._wait_for_new_image_on_sd(timeout=self.CAPTURE_TIMEOUT)
            if file_info is None:
                # Rollback: Phase 2 failed, don't update last_photo_count
                self.last_photo_count = saved_last_count
                raise Exception("Phase 2 failed: Image not found on SD card")
            
            # Phase 3: Download image
            check_timeout()
            with self.state_lock:
                self.pipeline_state = CaptureState.DOWNLOADING
            
            result = self._download_image_from_sd(file_info)
            if result is None:
                # Rollback: Phase 3 download failed, don't mark as downloaded
                self.last_photo_count = saved_last_count
                raise Exception("Phase 3 failed: Could not download image")
            
            filename, img = result
            
            # Save to local storage
            if not self._save_image_locally(filename, img):
                # Rollback: Phase 3 save failed, don't mark as downloaded
                self.last_photo_count = saved_last_count
                raise Exception("Phase 3 failed: Could not save image locally")
            
            # Phase 4: Publish to ROS
            check_timeout()
            with self.state_lock:
                self.pipeline_state = CaptureState.PUBLISHING
            
            self._publish_image_to_ros(filename, file_info, img)
            
            # Success! Reset state to idle
            with self.state_lock:
                self.pipeline_state = CaptureState.IDLE
            
            elapsed_time = time.time() - pipeline_start
            self.get_logger().info("=" * 70)
            self.get_logger().info(f" ✓ PIPELINE COMPLETED SUCCESSFULLY in {elapsed_time:.1f}s")
            self.get_logger().info("=" * 70)
            
            self._send_status(f"SUCCESS: Captured {filename} in {elapsed_time:.1f}s")
            self._publish_camera_status(f"SUCCESS: {self.current_resolution} image captured")
            
            # Save tracking state to survive restarts
            self._save_tracking_state()
            
            return True
            
        except Exception as e:
            # Error recovery: update state and rollback
            with self.state_lock:
                self.pipeline_state = CaptureState.FAILED
            
            elapsed_time = time.time() - pipeline_start
            self.get_logger().error("=" * 70)
            self.get_logger().error(f" ✗ PIPELINE FAILED after {elapsed_time:.1f}s: {e}")
            self.get_logger().error("=" * 70)
            
            self._send_status(f"FAILED: {e}")
            self._publish_camera_status(f"FAILURE: {e}")
            
            # Reset to idle after brief delay
            time.sleep(1.0)
            with self.state_lock:
                self.pipeline_state = CaptureState.IDLE
            
            return False
    
    # ========================================================================
    # MAIN LOOP
    # ========================================================================
    
    def _pipeline_loop(self):
        """
        Main execution loop (non-blocking).
        
        Responsibilities:
        1. Publish live video stream (always fast)
        2. Submit capture pipeline to worker thread when triggered (non-blocking)
        """
        # Check if camera is enabled (altitude check) with lock
        with self.config_lock:
            camera_enabled = self.camera_enabled
        
        if not camera_enabled:
            return
        
        # Check if capture was requested and submit to worker thread (non-blocking)
        if self.capture_requested.is_set():
            self.capture_requested.clear()
            
            # Check if previous pipeline is still running
            if self.pipeline_future is not None and not self.pipeline_future.done():
                self.get_logger().warn(" Previous pipeline still running, skipping new request")
            else:
                # Submit pipeline execution to worker thread (non-blocking)
                self.get_logger().info(" Submitting capture pipeline to worker thread...")
                self.pipeline_future = self.pipeline_executor.submit(self._execute_capture_pipeline)
        
        # Always publish live video stream (fast operation)
        if self.video_capture is not None and self.video_capture.isOpened():
            ret, frame = self.video_capture.read()
            
            if ret and frame is not None:
                # Publish live stream
                if self.bridge is not None:
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                else:
                    msg = self._cv2_to_imgmsg_manual(frame, encoding='bgr8')
                
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_link"
                
                self.image_pub.publish(msg)
    
    # ========================================================================
    # ROS CALLBACKS
    # ========================================================================
    
    def camera_trigger_callback(self, msg: Bool):
        """Handle capture trigger requests"""
        if msg.data:
            self.get_logger().info(" Capture trigger received!")
            self.capture_requested.set()
        else:
            self.get_logger().debug(" Trigger received with data=False, ignoring")
    
    def set_resolution_callback(self, msg: String):
        """Handle resolution change requests"""
        resolution = msg.data.upper()
        
        if resolution in self.PHOTO_RESOLUTIONS:
            self.current_resolution = resolution
            self.get_logger().info(f" Resolution set to: {resolution}")
            self._send_status(f"Resolution set to {resolution}")
        else:
            self.get_logger().warn(f" Invalid resolution: {resolution}")
    
    def altitude_callback(self, msg: Float64):
        """Handle altitude updates for camera enable/disable (thread-safe)"""
        current_alt = msg.data
        
        # Use parameter value for threshold
        with self.config_lock:
            was_enabled = self.camera_enabled
            
            if current_alt >= self.altitude_threshold:
                self.camera_enabled = True
                if not was_enabled:
                    self.get_logger().info(
                        f" Altitude {current_alt:.2f}m >= {self.altitude_threshold:.2f}m - Camera ENABLED"
                    )
                    self._send_status("Altitude threshold reached - Camera enabled")
            else:
                self.camera_enabled = False
                if was_enabled:
                    self.get_logger().info(
                        f" Altitude {current_alt:.2f}m < {self.altitude_threshold:.2f}m - Camera DISABLED"
                    )
                    self._send_status("Below altitude threshold - Camera disabled")
    
    # ========================================================================
    # UTILITY METHODS
    # ========================================================================
    
    def _send_status(self, text: str):
        """Send status message to MAVROS"""
        msg = StatusText()
        msg.severity = 6  # INFO
        msg.text = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status: {text}")
    
    def _publish_camera_status(self, text: str):
        """Publish camera-specific status"""
        msg = String()
        msg.data = text
        self.camera_status_pub.publish(msg)
    
    def shutdown_callback(self):
        """Proper ROS shutdown handler (called before node destruction)"""
        self.get_logger().info(" Shutting down SIYI pipeline...")
        
        # Shutdown worker thread pool
        if hasattr(self, 'pipeline_executor'):
            self.get_logger().info(" Stopping worker threads...")
            self.pipeline_executor.shutdown(wait=True)
        
        # Close network connections
        if hasattr(self, 'http_session'):
            self.get_logger().info(" Closing HTTP session...")
            self.http_session.close()
        
        if hasattr(self, 'sdk_socket'):
            self.get_logger().info(" Closing SDK socket...")
            self.sdk_socket.close()
        
        # Release video capture
        if hasattr(self, 'video_capture') and self.video_capture is not None:
            self.get_logger().info(" Releasing video capture...")
            self.video_capture.release()
        
        self.get_logger().info(" ✓ Shutdown complete")
    
    def __del__(self):
        """Backup cleanup (may not be called reliably in ROS)"""
        # shutdown_callback handles everything - this is just a safety net
        pass


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = SIYIUnifiedPipeline()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Explicit cleanup before destroying node
        node.get_logger().info("Initiating shutdown...")
        
        # Call shutdown callback for proper cleanup
        node.shutdown_callback()
        
        # Now destroy node
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
