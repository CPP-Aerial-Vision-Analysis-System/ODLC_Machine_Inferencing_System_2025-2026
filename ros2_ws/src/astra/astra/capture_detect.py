#!/usr/bin/env python3

"""
Combined Capture and Detection Node

Merges functionality from:
- siyiUnifiedWorking.py: Camera capture, SD card management, image storage
- new_od.py: Object detection using SAHI+YOLO

This node performs:
1. Image capture from SIYI A8 Mini camera
2. Storage to SD card and local Jetson directories
3. Automatic object detection on captured images
4. Publishing of both raw images and detection results

Architecture:
- Main thread: Camera capture and image management
- Worker thread: Object detection processing
- Timer callbacks: Pipeline loop, detection monitoring
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, String
from mavros_msgs.msg import StatusText, WaypointReached
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from interfaces.msg import ImageResult
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy

import cv2
import os
import time
import json
import socket
import struct
import requests
import numpy as np
from threading import Lock, Event, Thread
from urllib.request import urlretrieve
from urllib.error import URLError, HTTPError
from enum import Enum
from typing import Optional, Tuple, List, Dict, Set
from PIL import Image as PILImage
import queue
import traceback
import gc
import statistics
import platform
from datetime import datetime
from pathlib import Path

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except Exception as e:
    print(f"Warning: cv_bridge import failed: {e}")
    CV_BRIDGE_AVAILABLE = False
    CvBridge = None

# SAHI and YOLO imports
try:
    from sahi import AutoDetectionModel
    from sahi.predict import get_sliced_prediction
    from sahi.utils.cv import read_image
    SAHI_AVAILABLE = True
except ImportError:
    SAHI_AVAILABLE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

try:
    from tqdm import tqdm
    TQDM_AVAILABLE = True
except ImportError:
    TQDM_AVAILABLE = False


# ============================================================================
# CONSTANTS
# ============================================================================

# Camera configuration
CAM_IP = "192.168.144.25"
CTRL_PORT = 37260
MEDIA_PORT = 82
BASE_URL = f"http://{CAM_IP}:{MEDIA_PORT}/cgi-bin/media.cgi/api/v1"
TAKE_PHOTO_4K = bytes.fromhex("55 66 01 01 00 00 00 0c 00 34 ce")

# Detection configuration
DEFAULT_CONFIDENCE_THRESHOLD = 0.25
DEFAULT_SLICE_SIZE = 640
DEFAULT_OVERLAP = 0.25
DEFAULT_CHECK_INTERVAL = 2.0

# Model format constants
MODEL_FORMAT_PYTORCH = 'pytorch'
MODEL_FORMAT_TENSORRT = 'tensorrt'
MODEL_FORMAT_AUTO = 'auto'

# Class ID mapping
CLASS_ID = {"person": "0", "tent": "1", "object": "2"}

# Camera verification constants
MIN_4K_WIDTH = 3000
MIN_4K_HEIGHT = 1600
MIN_FILE_SIZE = 50000
MAX_CAPTURE_RETRIES = 3
CAPTURE_TIMEOUT = 15.0
SD_POLL_INTERVAL = 0.5


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
    DETECTING = "detecting"
    FAILED = "failed"


class CaptureDetectNode(Node):
    """
    Combined node for image capture and object detection.
    
    Integrates:
    - SIYI camera control and image capture
    - Local storage management
    - SAHI-based object detection
    - ROS2 publishing for both images and detections
    """
    
    def __init__(self):
        super().__init__('capture_detect_node')
        
        self.get_logger().info("=" * 70)
        self.get_logger().info(" INITIALIZING CAPTURE & DETECT NODE")
        self.get_logger().info("=" * 70)
        
        # ====================================================================
        # PARAMETERS
        # ====================================================================
        self.declare_parameter('model_path', 'yolo26x.pt')
        self.declare_parameter('confidence_threshold', DEFAULT_CONFIDENCE_THRESHOLD)
        self.declare_parameter('slice_height', DEFAULT_SLICE_SIZE)
        self.declare_parameter('slice_width', DEFAULT_SLICE_SIZE)
        self.declare_parameter('overlap_height_ratio', DEFAULT_OVERLAP)
        self.declare_parameter('overlap_width_ratio', DEFAULT_OVERLAP)
        self.declare_parameter('detection_check_interval', DEFAULT_CHECK_INTERVAL)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('stream_rate', 10.0)  # Hz for video stream
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.slice_height = self.get_parameter('slice_height').value
        self.slice_width = self.get_parameter('slice_width').value
        self.overlap_height_ratio = self.get_parameter('overlap_height_ratio').value
        self.overlap_width_ratio = self.get_parameter('overlap_width_ratio').value
        self.detection_check_interval = self.get_parameter('detection_check_interval').value
        self.stream_rate = self.get_parameter('stream_rate').value
        
        # ====================================================================
        # STATE MANAGEMENT
        # ====================================================================
        self.pipeline_state = CaptureState.IDLE
        self.state_lock = Lock()
        
        # Image tracking
        self.downloaded_files: Set[str] = set()
        self.processed_images: Dict[str, float] = {}  # For detection tracking
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
        
        # Detection state
        self.detection_model = None
        self.device = None
        self.model_format_detected = None
        
        # Worker thread for detection
        self.work_q = queue.Queue(maxsize=50)
        self.worker_thread = None
        self.worker_stop = Event()
        
        # Statistics
        self.stats = {
            'total_images_captured': 0,
            'total_images_processed': 0,
            'total_detections': 0,
            'total_tents': 0,
            'total_people': 0,
            'total_objects': 0,
            'avg_processing_time': 0.0,
            'last_processing_time': 0.0,
            'node_start_time': time.time(),
            'errors': 0
        }
        
        # Health monitoring
        self.health_status = {
            'is_healthy': True,
            'last_successful_detection': None,
            'consecutive_errors': 0
        }
        
        # ====================================================================
        # DIRECTORIES
        # ====================================================================
        ros2_ws_dir = self._find_ros2_workspace()
        video_cam_dir = os.path.join(ros2_ws_dir, "video_cam")
        os.makedirs(video_cam_dir, exist_ok=True)
        
        # Storage directories (same as siyiUnifiedWorking)
        self.download_dir = os.path.join(video_cam_dir, "downloaded_images")
        self.camera_feed_dir = os.path.join(video_cam_dir, "camera_feed")
        self.mapping_dir = os.path.join(video_cam_dir, "mapping_photos")
        
        # Detection results directory
        detection_pkg_dir = os.path.join(ros2_ws_dir, "src", "detection")
        self.detection_results_path = os.path.join(detection_pkg_dir, "detection_results_sahi")
        
        for directory in [self.download_dir, self.camera_feed_dir, 
                         self.mapping_dir, self.detection_results_path]:
            os.makedirs(directory, exist_ok=True)
        
        self.get_logger().info("Storage paths:")
        self.get_logger().info(f"  Downloads:      {self.download_dir}")
        self.get_logger().info(f"  Camera feed:    {self.camera_feed_dir}")
        self.get_logger().info(f"  Mapping photos: {self.mapping_dir}")
        self.get_logger().info(f"  Detection results: {self.detection_results_path}")
        
        # ====================================================================
        # ROS INTERFACE
        # ====================================================================
        
        # CV Bridge
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warn("cv_bridge not available, using alternative conversion")
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.status_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
        self.camera_status_pub = self.create_publisher(String, '/camera/status', 10)
        
        # Detection publishers
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.detection_pub = self.create_publisher(Detection2DArray, 'detections', qos_profile)
        self.detection_publisher = self.create_publisher(ImageResult, 'image_detection', qos_profile)
        
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
        
        # Video stream capture
        self.video_capture: Optional[cv2.VideoCapture] = None
        
        # ====================================================================
        # INITIALIZATION
        # ====================================================================
        
        # Initialize camera
        self._initialize_camera()
        
        # Initialize SD card
        self._initialize_sd_card()
        
        # Initialize detection model
        self._initialize_detection_model()
        
        # Start worker thread for detection
        self.worker_thread = Thread(target=self._detection_worker_loop, daemon=True)
        self.worker_thread.start()
        self.get_logger().info("✓ Detection worker thread started")
        
        # Start main pipeline loop
        stream_period = 1.0 / self.stream_rate
        self.pipeline_timer = self.create_timer(stream_period, self._pipeline_loop)
        
        # Start detection check timer
        self.detection_timer = self.create_timer(
            self.detection_check_interval, 
            self._check_for_new_images_to_detect
        )
        
        self.get_logger().info("=" * 70)
        self.get_logger().info(" ✓ CAPTURE & DETECT NODE READY")
        self.get_logger().info("=" * 70)
    
    # ========================================================================
    # INITIALIZATION METHODS
    # ========================================================================
    
    def _find_ros2_workspace(self) -> str:
        """Locate the ROS2 workspace root directory"""
        current_file = os.path.abspath(__file__)
        search_dir = os.path.dirname(current_file)
        
        for _ in range(10):
            if (os.path.exists(os.path.join(search_dir, "install")) and 
                os.path.exists(os.path.join(search_dir, "src"))):
                return search_dir
            
            search_dir = os.path.dirname(search_dir)
            if search_dir == "/":
                break
        
        fallback = os.path.expanduser("~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws")
        self.get_logger().warn(f"Could not determine ros2_ws, using fallback: {fallback}")
        return fallback
    
    def _initialize_camera(self):
        """Initialize RTSP video stream connection"""
        rtsp_url = f'rtsp://{CAM_IP}:8554/main.264'
        
        self.get_logger().info(f"Connecting to camera at {rtsp_url}...")
        
        # Try FFmpeg backend first
        self.video_capture = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
        
        if not self.video_capture.isOpened():
            self.get_logger().warn("FFmpeg failed, trying GStreamer...")
            gst_pipeline = (
                f'rtspsrc location={rtsp_url} latency=0 ! '
                'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink'
            )
            self.video_capture = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.video_capture.isOpened():
            self.get_logger().warn("GStreamer failed, trying default backend...")
            self.video_capture = cv2.VideoCapture(rtsp_url)
        
        if self.video_capture.isOpened():
            self.get_logger().info("✓ Camera video stream connected")
        else:
            self.get_logger().error("✗ Failed to connect to camera video stream")
    
    def _initialize_sd_card(self):
        """Initialize SD card state"""
        try:
            response = requests.get(f"{BASE_URL}/files", timeout=5)
            if response.status_code == 200:
                data = response.json()
                if 'files' in data and len(data['files']) > 0:
                    self.current_photo_dir = data['files'][0].get('name', 'SIYI')
                    self.get_logger().info(f"✓ SD card initialized: {self.current_photo_dir}")
        except Exception as e:
            self.get_logger().warn(f"Could not initialize SD card state: {e}")
    
    def _initialize_detection_model(self):
        """Initialize SAHI detection model"""
        if not SAHI_AVAILABLE:
            self.get_logger().error("SAHI not available! Detection will not work.")
            return
        
        if not YOLO_AVAILABLE:
            self.get_logger().error("YOLO not available! Detection will not work.")
            return
        
        self.get_logger().info("Loading detection model...")
        self.get_logger().info(f"  Model: {self.model_path}")
        self.get_logger().info(f"  Confidence: {self.confidence_threshold}")
        self.get_logger().info(f"  Slice size: {self.slice_height}x{self.slice_width}")
        self.get_logger().info(f"  Overlap: {self.overlap_height_ratio}")
        
        # Determine device
        device_param = self.get_parameter('device').value
        if device_param == 'auto':
            if TORCH_AVAILABLE and torch.cuda.is_available():
                self.device = 'cuda:0'
            else:
                self.device = 'cpu'
        else:
            self.device = device_param
        
        self.get_logger().info(f"  Device: {self.device}")
        
        try:
            # Find model path
            model_full_path = self._find_model_path(self.model_path)
            
            # Load SAHI model
            self.detection_model = AutoDetectionModel.from_pretrained(
                model_type='yolov8',
                model_path=model_full_path,
                confidence_threshold=self.confidence_threshold,
                device=self.device
            )
            
            self.model_format_detected = MODEL_FORMAT_PYTORCH
            self.get_logger().info("✓ Detection model loaded successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load detection model: {e}")
            self.get_logger().error(traceback.format_exc())
    
    def _find_model_path(self, model_name: str) -> str:
        """Find model file in ros2_ws directory"""
        # Check in ros2_ws root
        ros2_ws = self._find_ros2_workspace()
        model_path = os.path.join(ros2_ws, model_name)
        
        if os.path.exists(model_path):
            return model_path
        
        # Check relative to current file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, model_name)
        
        if os.path.exists(model_path):
            return model_path
        
        # Return as-is and let it fail
        return model_name
    
    # ========================================================================
    # MAIN PIPELINE LOOP (From siyiUnifiedWorking)
    # ========================================================================
    
    def _pipeline_loop(self):
        """
        Main execution loop for camera capture and streaming.
        """
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
                if self.bridge is not None:
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                else:
                    msg = self._cv2_to_imgmsg_manual(frame, encoding='bgr8')
                
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_link"
                
                self.image_pub.publish(msg)
    
    def _execute_capture_pipeline(self) -> bool:
        """Execute complete image capture pipeline"""
        with self.state_lock:
            if self.pipeline_state != CaptureState.IDLE:
                self.get_logger().warn("Pipeline already running, skipping request")
                return False
            self.pipeline_state = CaptureState.CAPTURING
        
        try:
            self.get_logger().info("=" * 70)
            self.get_logger().info(" STARTING IMAGE CAPTURE PIPELINE")
            self.get_logger().info("=" * 70)
            
            # Phase 1: Trigger capture
            if not self._trigger_capture():
                raise Exception("Phase 1 failed: Could not trigger capture")
            
            time.sleep(0.5)
            
            # Phase 2: Poll SD card for new image
            with self.state_lock:
                self.pipeline_state = CaptureState.INDEXING
            
            file_info = self._wait_for_new_image_on_sd(timeout=CAPTURE_TIMEOUT)
            if file_info is None:
                raise Exception("Phase 2 failed: Image not found on SD card")
            
            # Phase 3: Download image
            with self.state_lock:
                self.pipeline_state = CaptureState.DOWNLOADING
            
            result = self._download_image_from_sd(file_info)
            if result is None:
                raise Exception("Phase 3 failed: Could not download image")
            
            filename, img = result
            
            # Save to local storage (both directories for compatibility)
            if not self._save_image_locally(filename, img):
                raise Exception("Phase 3 failed: Could not save image locally")
            
            # Phase 4: Publish to ROS
            with self.state_lock:
                self.pipeline_state = CaptureState.PUBLISHING
            
            self._publish_image_to_ros(filename, img)
            
            # Phase 5: Queue for detection
            with self.state_lock:
                self.pipeline_state = CaptureState.DETECTING
            
            self._queue_image_for_detection(filename)
            
            # Success!
            with self.state_lock:
                self.pipeline_state = CaptureState.IDLE
            
            self.stats['total_images_captured'] += 1
            
            self.get_logger().info("=" * 70)
            self.get_logger().info(" ✓ PIPELINE COMPLETED SUCCESSFULLY")
            self.get_logger().info("=" * 70)
            
            return True
            
        except Exception as e:
            with self.state_lock:
                self.pipeline_state = CaptureState.FAILED
            
            self.get_logger().error("=" * 70)
            self.get_logger().error(f" ✗ PIPELINE FAILED: {e}")
            self.get_logger().error("=" * 70)
            
            time.sleep(1.0)
            with self.state_lock:
                self.pipeline_state = CaptureState.IDLE
            
            return False
    
    # ========================================================================
    # CAMERA CAPTURE METHODS (From siyiUnifiedWorking)
    # ========================================================================
    
    def _trigger_capture(self) -> bool:
        """Send capture command to camera"""
        try:
            self.sdk_socket.sendto(TAKE_PHOTO_4K, (CAM_IP, CTRL_PORT))
            self.get_logger().info("✓ Capture command sent to camera")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to send capture command: {e}")
            return False
    
    def _wait_for_new_image_on_sd(self, timeout: float) -> Optional[Dict]:
        """Poll SD card for new image"""
        start_time = time.time()
        initial_count = self._get_sd_image_count()
        
        self.get_logger().info(f"Waiting for new image on SD (current count: {initial_count})...")
        
        while (time.time() - start_time) < timeout:
            time.sleep(SD_POLL_INTERVAL)
            
            try:
                # Get latest images from SD
                response = requests.get(
                    f"{BASE_URL}/files/{self.current_photo_dir}/PHOTO",
                    timeout=2
                )
                
                if response.status_code == 200:
                    data = response.json()
                    files = data.get('files', [])
                    
                    if len(files) > initial_count:
                        # Found new image - return the latest one
                        latest = files[-1]
                        self.get_logger().info(f"✓ New image found: {latest['name']}")
                        return latest
                        
            except Exception as e:
                self.get_logger().debug(f"SD poll error: {e}")
        
        self.get_logger().error("Timeout waiting for image on SD card")
        return None
    
    def _get_sd_image_count(self) -> int:
        """Get current count of images on SD card"""
        try:
            response = requests.get(
                f"{BASE_URL}/files/{self.current_photo_dir}/PHOTO",
                timeout=2
            )
            if response.status_code == 200:
                data = response.json()
                return len(data.get('files', []))
        except:
            pass
        return 0
    
    def _download_image_from_sd(self, file_info: Dict) -> Optional[Tuple[str, np.ndarray]]:
        """Download image from SD card"""
        filename = file_info['name']
        file_url = f"{BASE_URL}/files/{self.current_photo_dir}/PHOTO/{filename}"
        
        self.get_logger().info(f"Downloading {filename}...")
        
        try:
            response = requests.get(file_url, timeout=10)
            if response.status_code == 200:
                # Decode image from bytes
                img_array = np.frombuffer(response.content, dtype=np.uint8)
                img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                
                if img is not None:
                    self.get_logger().info(f"✓ Downloaded: {filename} ({img.shape[1]}x{img.shape[0]})")
                    return (filename, img)
                else:
                    self.get_logger().error(f"Failed to decode image: {filename}")
                    return None
        except Exception as e:
            self.get_logger().error(f"Download failed: {e}")
            return None
    
    def _save_image_locally(self, filename: str, img: np.ndarray) -> bool:
        """Save image to local directories"""
        try:
            # Save to camera_feed (for detection)
            camera_feed_path = os.path.join(self.camera_feed_dir, filename)
            cv2.imwrite(camera_feed_path, img)
            
            # Save to downloaded_images (for archival)
            download_path = os.path.join(self.download_dir, filename)
            cv2.imwrite(download_path, img)
            
            # Save to mapping_photos (for mapping)
            mapping_path = os.path.join(self.mapping_dir, filename)
            cv2.imwrite(mapping_path, img)
            
            self.get_logger().info(f"✓ Saved to local directories: {filename}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to save image locally: {e}")
            return False
    
    def _publish_image_to_ros(self, filename: str, img: np.ndarray):
        """Publish image to ROS topic"""
        try:
            if self.bridge is not None:
                msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            else:
                msg = self._cv2_to_imgmsg_manual(img, encoding='bgr8')
            
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            
            self.image_pub.publish(msg)
            self.get_logger().info(f"✓ Published image to ROS: {filename}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")
    
    def _cv2_to_imgmsg_manual(self, cv_image: np.ndarray, encoding: str = "bgr8") -> Image:
        """Manual conversion from cv2 to ROS Image message"""
        img_msg = Image()
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = encoding
        img_msg.is_bigendian = 0
        img_msg.step = cv_image.shape[1] * cv_image.shape[2]
        img_msg.data = cv_image.tobytes()
        return img_msg
    
    # ========================================================================
    # DETECTION METHODS (From new_od.py)
    # ========================================================================
    
    def _queue_image_for_detection(self, filename: str):
        """Queue newly captured image for detection"""
        try:
            image_path = os.path.join(self.camera_feed_dir, filename)
            if os.path.exists(image_path):
                self.work_q.put_nowait(image_path)
                self.processed_images[filename] = time.time()
                self.get_logger().info(f"✓ Queued for detection: {filename}")
        except queue.Full:
            self.get_logger().warn(f"Detection queue full, skipping: {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to queue image: {e}")
    
    def _check_for_new_images_to_detect(self):
        """Check camera_feed directory for any unprocessed images"""
        if self.detection_model is None:
            return
        
        try:
            # Get all images in camera_feed
            image_files = []
            for file in os.listdir(self.camera_feed_dir):
                if file.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')):
                    if file not in self.processed_images:
                        file_path = os.path.join(self.camera_feed_dir, file)
                        image_files.append((file, file_path))
            
            # Queue new images
            for filename, filepath in image_files[:5]:  # Limit to 5 per cycle
                try:
                    self.work_q.put_nowait(filepath)
                    self.processed_images[filename] = time.time()
                    self.get_logger().info(f"Detected unprocessed image: {filename}")
                except queue.Full:
                    break
                    
        except Exception as e:
            self.get_logger().error(f"Error checking for new images: {e}")
    
    def _detection_worker_loop(self):
        """Worker thread for processing images asynchronously"""
        while not self.worker_stop.is_set():
            try:
                image_path = self.work_q.get(timeout=0.2)
            except queue.Empty:
                continue
            
            try:
                self._process_image_detection(image_path)
                self.health_status['last_successful_detection'] = time.time()
                self.health_status['consecutive_errors'] = 0
                self.health_status['is_healthy'] = True
            except Exception as e:
                self.get_logger().error(f"Detection error: {e}")
                self.get_logger().error(traceback.format_exc())
                self.stats['errors'] += 1
                self.health_status['consecutive_errors'] += 1
            finally:
                self.work_q.task_done()
    
    def _process_image_detection(self, image_path: str):
        """Process a single image for object detection"""
        if self.detection_model is None:
            return
        
        start_time = time.time()
        
        # Load image
        frame = cv2.imread(image_path)
        if frame is None:
            self.get_logger().warn(f"Could not load image: {image_path}")
            return
        
        filename = os.path.basename(image_path)
        height, width = frame.shape[:2]
        self.get_logger().info(f"📸 Processing: {filename} ({width}x{height})")
        
        # Run SAHI detection
        detections = self._detect_objects_sahi(frame)
        
        processing_time = time.time() - start_time
        self.stats['last_processing_time'] = processing_time
        
        # Create annotated frame
        frame_orig = frame.copy()
        annotated_frame = self._annotate_frame(frame, detections, processing_time)
        
        # Save results
        self._save_detection_results(annotated_frame, filename)
        
        # Publish detection results
        self._publish_detection_results(detections, filename)
        
        # Update statistics
        self.stats['total_images_processed'] += 1
        self.stats['total_detections'] += len(detections)
        self.stats['total_tents'] += sum(1 for d in detections if d['class'] == 'tent')
        self.stats['total_people'] += sum(1 for d in detections if d['class'] == 'person')
        self.stats['total_objects'] += sum(1 for d in detections if d['class'] == 'object')
        
        # Update average
        n = self.stats['total_images_processed']
        old_avg = self.stats['avg_processing_time']
        self.stats['avg_processing_time'] = (old_avg * (n - 1) + processing_time) / n
        
        self.get_logger().info(f"✓ Detection complete: {len(detections)} objects in {processing_time:.1f}s")
    
    def _detect_objects_sahi(self, frame: np.ndarray) -> List[Dict]:
        """Run SAHI sliced inference on image"""
        detections = []
        
        try:
            result = get_sliced_prediction(
                frame,
                self.detection_model,
                slice_height=self.slice_height,
                slice_width=self.slice_width,
                overlap_height_ratio=self.overlap_height_ratio,
                overlap_width_ratio=self.overlap_width_ratio,
                verbose=0
            )
            
            # Process detections
            for obj in result.object_prediction_list:
                bbox = obj.bbox.to_xyxy()
                class_name = obj.category.name.lower()
                confidence = obj.score.value
                
                # Map to our classes
                if 'person' in class_name or 'people' in class_name:
                    mapped_class = 'person'
                elif 'tent' in class_name:
                    mapped_class = 'tent'
                else:
                    mapped_class = 'object'
                
                detections.append({
                    'class': mapped_class,
                    'confidence': confidence,
                    'bbox': [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])],
                    'yolo_class': class_name
                })
                
        except Exception as e:
            self.get_logger().error(f"SAHI detection failed: {e}")
        
        return detections
    
    def _annotate_frame(self, frame: np.ndarray, detections: List[Dict], 
                       processing_time: float) -> np.ndarray:
        """Annotate frame with detection results"""
        annotated_frame = frame.copy()
        height, width = frame.shape[:2]
        
        # Colors (BGR)
        COLOR_TENT = (0, 200, 255)
        COLOR_PERSON = (0, 200, 0)
        COLOR_OBJECT = (255, 100, 0)
        COLOR_WHITE = (255, 255, 255)
        COLOR_BLACK = (0, 0, 0)
        
        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = 0.45
        font_thickness = 1
        
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            class_name = detection['class']
            confidence = detection['confidence']
            
            if class_name == 'person':
                box_color = COLOR_PERSON
                label = f"PERSON ({confidence:.0%})"
                align_right = True
            elif class_name == 'tent':
                box_color = COLOR_TENT
                label = f"TENT ({confidence:.0%})"
                align_right = False
            else:
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), COLOR_OBJECT, 1)
                continue
            
            # Draw box
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), box_color, 1)
            
            # Calculate label position
            (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, font_thickness)
            padding = 4
            label_h = text_h + padding * 2
            
            if y1 - label_h >= 0:
                label_y1 = y1 - label_h
                label_y2 = y1
            else:
                label_y1 = y1
                label_y2 = y1 + label_h
            
            # Position horizontally
            if align_right:
                label_x1 = max(0, x2 - text_w - padding * 2)
                label_x2 = x2
            else:
                label_x1 = x1
                label_x2 = x1 + text_w + padding * 2
            
            # Draw label background
            cv2.rectangle(annotated_frame, (label_x1, label_y1), 
                         (label_x2, label_y2), box_color, -1)
            
            # Draw text
            text_x = label_x1 + padding
            text_y = label_y2 - padding
            cv2.putText(annotated_frame, label, (text_x, text_y),
                       font, font_scale, COLOR_BLACK, font_thickness + 1, cv2.LINE_AA)
            cv2.putText(annotated_frame, label, (text_x, text_y),
                       font, font_scale, COLOR_WHITE, font_thickness, cv2.LINE_AA)
        
        # Add header
        num_people = sum(1 for d in detections if d['class'] == 'person')
        num_tents = sum(1 for d in detections if d['class'] == 'tent')
        
        header_text = f"SAHI+YOLO | {num_people} people, {num_tents} tents"
        
        overlay = annotated_frame.copy()
        cv2.rectangle(overlay, (0, 0), (width, 40), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, annotated_frame, 0.4, 0, annotated_frame)
        
        cv2.putText(annotated_frame, header_text, (10, 25),
                   font, 0.6, COLOR_WHITE, 1, cv2.LINE_AA)
        
        return annotated_frame
    
    def _save_detection_results(self, annotated_frame: np.ndarray, filename: str):
        """Save annotated image to detection results directory"""
        try:
            output_path = os.path.join(self.detection_results_path, f"det_{filename}")
            cv2.imwrite(output_path, annotated_frame)
            self.get_logger().debug(f"Saved detection result: {output_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save detection result: {e}")
    
    def _publish_detection_results(self, detections: List[Dict], filename: str):
        """Publish detection results to ROS topics"""
        try:
            # Publish Detection2DArray
            det_array = Detection2DArray()
            det_array.header.stamp = self.get_clock().now().to_msg()
            det_array.header.frame_id = "camera_link"
            
            for detection in detections:
                det_msg = Detection2D()
                det_msg.bbox.center.x = float((detection['bbox'][0] + detection['bbox'][2]) / 2)
                det_msg.bbox.center.y = float((detection['bbox'][1] + detection['bbox'][3]) / 2)
                det_msg.bbox.size_x = float(detection['bbox'][2] - detection['bbox'][0])
                det_msg.bbox.size_y = float(detection['bbox'][3] - detection['bbox'][1])
                
                hyp = ObjectHypothesisWithPose()
                hyp.id = CLASS_ID.get(detection['class'], "2")
                hyp.score = detection['confidence']
                det_msg.results.append(hyp)
                
                det_array.detections.append(det_msg)
            
            self.detection_pub.publish(det_array)
            
            # Publish ImageResult (custom message)
            result_msg = ImageResult()
            result_msg.header.stamp = self.get_clock().now().to_msg()
            result_msg.header.frame_id = "camera_link"
            result_msg.image_name = filename
            result_msg.num_detections = len(detections)
            result_msg.num_people = sum(1 for d in detections if d['class'] == 'person')
            result_msg.num_tents = sum(1 for d in detections if d['class'] == 'tent')
            
            self.detection_publisher.publish(result_msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish detection results: {e}")
    
    # ========================================================================
    # ROS CALLBACKS
    # ========================================================================
    
    def camera_trigger_callback(self, msg: Bool):
        """Handle capture trigger requests"""
        if msg.data:
            self.get_logger().info("Capture trigger received!")
            self.capture_requested.set()
    
    def set_resolution_callback(self, msg: String):
        """Handle resolution change requests"""
        resolution = msg.data.upper()
        if resolution in ['4K', '2.7K', '1080P']:
            self.current_resolution = resolution
            self.get_logger().info(f"Resolution set to: {resolution}")
    
    def altitude_callback(self, msg: Float64):
        """Handle altitude updates for camera enable/disable"""
        ALT_THRESHOLD = -13.716
        current_alt = msg.data
        
        if current_alt >= ALT_THRESHOLD:
            if not self.camera_enabled:
                self.camera_enabled = True
                self.get_logger().info("Camera enabled (altitude threshold reached)")
        else:
            if self.camera_enabled:
                self.camera_enabled = False
                self.get_logger().info("Camera disabled (below altitude threshold)")
    
    # ========================================================================
    # CLEANUP
    # ========================================================================
    
    def destroy_node(self):
        """Cleanup on node shutdown"""
        self.get_logger().info("Shutting down Capture & Detect Node...")
        
        # Stop worker thread
        if self.worker_thread is not None:
            self.worker_stop.set()
            self.worker_thread.join(timeout=5.0)
        
        # Close video capture
        if self.video_capture is not None:
            self.video_capture.release()
        
        # Close socket
        if hasattr(self, 'sdk_socket'):
            self.sdk_socket.close()
        
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = CaptureDetectNode()
    
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
