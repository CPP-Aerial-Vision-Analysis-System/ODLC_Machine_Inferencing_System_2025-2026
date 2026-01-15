#!/usr/bin/env python3

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
from threading import Lock, Event
from urllib.request import urlretrieve
from urllib.error import URLError, HTTPError
from enum import Enum
from typing import Optional, Tuple, List, Dict, Set

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
    CAPTURE_TIMEOUT = 15.0  # seconds to wait for SD card write
    SD_POLL_INTERVAL = 0.5  # seconds between SD card polls
    
    # Loop timing
    STREAM_RATE = 10.0  # Hz for video stream publishing
    
    def __init__(self):
        super().__init__('siyi_unified_pipeline')
        
        # ====================================================================
        # STATE MANAGEMENT (Jetson is stateful)
        # ====================================================================
        self.pipeline_state = CaptureState.IDLE
        self.state_lock = Lock()
        
        # Downloaded file tracking (prevents duplicate downloads)
        self.downloaded_files: Set[str] = set()
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
        
        # ====================================================================
        # ROS INTERFACE
        # ====================================================================
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.status_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
        self.camera_status_pub = self.create_publisher(String, '/camera/status', 10)
        
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
        rtsp_url = f'rtsp://{self.CAM_IP}:8554/main.264'
        
        self.get_logger().info(f" Connecting to camera at {rtsp_url}...")
        
        # Try FFmpeg backend first
        self.video_capture = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        
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
            response = requests.get(url, timeout=5)
            
            if response.status_code == 200:
                data = response.json()
                
                if data.get('success', False):
                    directories = data.get('data', {}).get('directories', [])
                    
                    if len(directories) > 0:
                        # Use most recent directory
                        self.current_photo_dir = directories[-1]['path']
                        self.get_logger().info(f" Photo directory: {self.current_photo_dir}")
                        
                        # Get initial file count
                        count = self._get_sd_photo_count(self.current_photo_dir)
                        if count is not None:
                            self.last_photo_count = count
                            self.get_logger().info(f" Initial photo count: {count}")
                        
                        # Load existing files into tracking set
                        self._load_existing_sd_files()
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
            for file_info in files:
                filename = file_info.get('name', '')
                if filename:
                    self.downloaded_files.add(filename)
            
            self.get_logger().info(f" ✓ Loaded {len(self.downloaded_files)} existing files into tracking")
        except Exception as e:
            self.get_logger().warn(f" Could not load existing files: {e}")
    
    # ========================================================================
    # PHASE 1: CAPTURE CONTROL (Camera → SD)
    # ========================================================================
    
    def _trigger_capture(self) -> bool:
        """
        Command the camera to capture and save to SD card.
        
        Returns:
            True if command sent successfully
        """
        try:
            # self.get_logger().info(f" [Phase 1] Triggering {self.current_resolution} capture...")
            
            # Send capture command via UDP
            self.sdk_socket.sendto(self.TAKE_PHOTO_4K, (self.CAM_IP, self.CTRL_PORT))
            self.photo_count += 1
            
            self.get_logger().info(f" Capture command sent (photo #{self.photo_count})")
            return True
            
        except Exception as e:
            self.get_logger().error(f" Capture command failed: {e}")
            return False
    
    # ========================================================================
    # PHASE 2: SD CARD INDEXING (Metadata Only)
    # ========================================================================
    
    def _query_sd_card_index(self) -> List[Dict]:
        """
        Query SD card for current file list.
        
        Returns:
            List of file info dictionaries with 'name', 'url', etc.
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
            
            response = requests.get(url, params=params, timeout=5)
            
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
            
            response = requests.get(url, params=params, timeout=5)
            
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
        Poll SD card until a new image appears.
        
        Returns:
            File info dict for new image, or None if timeout
        """
        start_time = time.time()
        
        self.get_logger().info(f" [Phase 2] Polling SD card for new image (timeout: {timeout}s)...")
        
        while (time.time() - start_time) < timeout:
            try:
                file_list = self._query_sd_card_index()
                current_count = len(file_list)
                
                # Check if count increased
                if current_count > self.last_photo_count:
                    self.get_logger().info(
                        f" New image detected on SD! Count: {self.last_photo_count} → {current_count}"
                    )
                    
                    # Find the new file(s)
                    for file_info in reversed(file_list):  # Start from most recent
                        filename = file_info.get('name', '')
                        if filename and filename not in self.downloaded_files:
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
            
            time.sleep(self.SD_POLL_INTERVAL)
        
        self.get_logger().error(f" Timeout: No new image after {timeout}s")
        return None
    
    # ========================================================================
    # PHASE 3: INCREMENTAL DOWNLOAD (SD → Jetson)
    # ========================================================================
    
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
            
            if not filename or not file_url:
                self.get_logger().error(" ✗ Invalid file info (missing name or URL)")
                return None
            
            # Fix IP address in URL if needed
            file_url = file_url.replace("192.168.144.25", self.CAM_IP)
            
            self.get_logger().info(f" [Phase 3] Downloading: {filename}")
            self.get_logger().info(f"   URL: {file_url}")
            
            # Download image
            response = requests.get(file_url, timeout=15)
            
            if response.status_code != 200:
                self.get_logger().error(f" ✗ Download failed: HTTP {response.status_code}")
                return None
            
            # Decode image
            img_array = np.frombuffer(response.content, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            
            if img is None:
                self.get_logger().error(" ✗ Failed to decode image")
                return None
            
            # Verify dimensions
            h, w = img.shape[:2]
            size_kb = len(response.content) / 1024
            
            self.get_logger().info(f" ✓ Downloaded: {w}x{h}, {size_kb:.1f}KB")
            
            if not self._verify_image_dimensions(img):
                self.get_logger().warn(" ⚠ Image dimensions below expected threshold")
            
            return (filename, img)
            
        except Exception as e:
            self.get_logger().error(f" ✗ Download error: {e}")
            return None
    
    def _verify_image_dimensions(self, img: np.ndarray) -> bool:
        """Verify image meets minimum dimension requirements"""
        if img is None:
            return False
        
        try:
            h, w = img.shape[:2]
            
            if self.current_resolution == '4K':
                if w >= self.MIN_4K_WIDTH and h >= self.MIN_4K_HEIGHT:
                    return True
                else:
                    self.get_logger().warn(f" Dimensions {w}x{h} below 4K threshold")
                    return False
            
            # For other resolutions, just check that it's not too small
            return w > 640 and h > 480
            
        except Exception:
            return False
    
    def _save_image_locally(self, filename: str, img: np.ndarray) -> bool:
        """
        Save downloaded image to local storage with atomic write.
        
        Saves to multiple locations:
        - download_dir: Original downloaded file
        - camera_feed_dir: For general processing
        - mapping_dir: For mapping-specific processing
        
        Returns:
            True if all saves successful
        """
        try:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            base_name = f"photo_{timestamp}_{filename}"
            
            # Define paths
            paths = {
                'download': os.path.join(self.download_dir, filename),
                'camera_feed': os.path.join(self.camera_feed_dir, base_name),
                'mapping': os.path.join(self.mapping_dir, f"mapping_{base_name}")
            }
            
            self.get_logger().info(f" [Phase 3] Saving to local storage...")
            
            # Atomic write to all locations
            all_success = True
            for location, path in paths.items():
                if not self._atomic_write(path, img):
                    self.get_logger().error(f" ✗ Failed to write to {location}: {path}")
                    all_success = False
                else:
                    # Verify written file
                    if not self._verify_local_file(path):
                        self.get_logger().error(f" ✗ Verification failed for {location}: {path}")
                        all_success = False
            
            if all_success:
                self.get_logger().info(" ✓ Saved to all locations successfully")
                self.last_download_time = time.time()
            
            return all_success
            
        except Exception as e:
            self.get_logger().error(f" ✗ Save error: {e}")
            return False
    
    def _atomic_write(self, filepath: str, img: np.ndarray) -> bool:
        """Write image file atomically (temp file + rename)"""
        try:
            tmp_path = filepath + ".tmp"
            
            # Write to temp file
            success = cv2.imwrite(tmp_path, img)
            if not success:
                self.get_logger().error(f" cv2.imwrite failed for {tmp_path}")
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
        """Verify local file exists and has valid size"""
        if min_bytes is None:
            min_bytes = self.MIN_FILE_SIZE
        
        try:
            if not os.path.exists(path):
                return False
            
            size = os.path.getsize(path)
            return size >= min_bytes
            
        except Exception:
            return False
    
    # ========================================================================
    # PHASE 4: ROS PUBLICATION (Jetson → ROS Graph)
    # ========================================================================
    
    def _publish_image_to_ros(self, filename: str, img: np.ndarray):
        """
        Publish image to ROS graph with proper metadata.
        
        Args:
            filename: Original filename for tracking
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
            
            # Mark as processed
            self.downloaded_files.add(filename)
            
        except Exception as e:
            self.get_logger().error(f" ✗ Publish error: {e}")
    
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
        Execute complete 4-phase capture pipeline.
        
        Returns:
            True if entire pipeline succeeded
        """
        with self.state_lock:
            if self.pipeline_state != CaptureState.IDLE:
                self.get_logger().warn(" Pipeline already running, skipping request")
                return False
            
            self.pipeline_state = CaptureState.CAPTURING
        
        try:
            self.get_logger().info("=" * 70)
            self.get_logger().info(" STARTING IMAGE CAPTURE PIPELINE")
            self.get_logger().info("=" * 70)
            
            # Phase 1: Trigger capture
            with self.state_lock:
                self.pipeline_state = CaptureState.CAPTURING
            
            if not self._trigger_capture():
                raise Exception("Phase 1 failed: Could not trigger capture")
            
            # Brief wait for camera to write to SD
            time.sleep(0.5)
            
            # Phase 2: Poll SD card for new image
            with self.state_lock:
                self.pipeline_state = CaptureState.INDEXING
            
            file_info = self._wait_for_new_image_on_sd(timeout=self.CAPTURE_TIMEOUT)
            if file_info is None:
                raise Exception("Phase 2 failed: Image not found on SD card")
            
            # Phase 3: Download image
            with self.state_lock:
                self.pipeline_state = CaptureState.DOWNLOADING
            
            result = self._download_image_from_sd(file_info)
            if result is None:
                raise Exception("Phase 3 failed: Could not download image")
            
            filename, img = result
            
            # Save to local storage
            if not self._save_image_locally(filename, img):
                raise Exception("Phase 3 failed: Could not save image locally")
            
            # Phase 4: Publish to ROS
            with self.state_lock:
                self.pipeline_state = CaptureState.PUBLISHING
            
            self._publish_image_to_ros(filename, img)
            
            # Success!
            with self.state_lock:
                self.pipeline_state = CaptureState.IDLE
            
            self.get_logger().info("=" * 70)
            self.get_logger().info(" ✓ PIPELINE COMPLETED SUCCESSFULLY")
            self.get_logger().info("=" * 70)
            
            self._send_status(f"SUCCESS: Captured and published {filename}")
            self._publish_camera_status(f"SUCCESS: {self.current_resolution} image captured")
            
            return True
            
        except Exception as e:
            with self.state_lock:
                self.pipeline_state = CaptureState.FAILED
            
            self.get_logger().error("=" * 70)
            self.get_logger().error(f" ✗ PIPELINE FAILED: {e}")
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
        Main execution loop.
        
        Responsibilities:
        1. Publish live video stream
        2. Execute capture pipeline when triggered
        """
        # Check if camera is enabled (altitude check)
        if not self.camera_enabled:
            return
        
        # Check if capture was requested
        if self.capture_requested.is_set():
            self.capture_requested.clear()
            self._execute_capture_pipeline()
            return
        
        # Otherwise, publish live video stream
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
        """Handle altitude updates for camera enable/disable"""
        ALT_THRESHOLD = -13.716
        current_alt = msg.data
        
        if current_alt >= ALT_THRESHOLD:
            if not self.camera_enabled:
                self.camera_enabled = True
                self._send_status("Altitude threshold reached - Camera enabled")
        else:
            if self.camera_enabled:
                self.camera_enabled = False
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
    
    def __del__(self):
        """Cleanup resources"""
        if hasattr(self, 'sdk_socket'):
            self.sdk_socket.close()
        if hasattr(self, 'video_capture') and self.video_capture is not None:
            self.video_capture.release()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = SIYIUnifiedPipeline()
    
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
