#!/usr/bin/env python3

"""
SAHI Object Detection Node with TensorRT Support
Identical to object_detection_sahi.py but uses TensorRT instead of PyTorch

Current Architecture:
- SAHI: Slices images and manages detection pipeline
- YOLO26/TensorRT: Performs actual object detection on each slice

Detection Pipeline:
1. SAHI slices the image into overlapping patches
2. YOLO26/TensorRT detects objects in each slice
3. Results are merged and filtered (NMS)
4. Detections are annotated with YOLO26 results

Configuration:
- Slice size: 256x256 (optimized for small object detection)
- Overlap: 45% (ensures objects at boundaries are detected)
- Balance: Accuracy over speed for critical small object detection
"""
# ros2 imports
import rclpy # define ros2 nodes
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn # define ros2 lifecycle nodes
from rclpy.executors import MultiThreadedExecutor 
# Removed get_package_share_directory - now using ~/detection directory instead
from cv_bridge import CvBridge # converts between ros image messages and opencv(cv2) images
from sensor_msgs.msg import Image # ros2 message type for sending images
from interfaces.msg import ImageResult # custom message for detection results
from mavros_msgs.msg import WaypointReached # waypoint reached message
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose  # vision_msgs for detection results

# non-ros2 imports
import cv2 # opencv computer vision library
from std_msgs.msg import String # string message type
import numpy as np # numerical arrays
import time # timing
from datetime import datetime # timing
import os # path
from pathlib import Path # path
from typing import List, Dict, Optional, Tuple # type hints
import gc # garbage collection
import platform # system info
from rclpy.parameter import Parameter # for parameter callbacks
from rclpy.qos import QoSProfile, ReliabilityPolicy # QoS settings
# This is just to make sure that we have all the dependencies

# Progress bar
try:
    from tqdm import tqdm
    TQDM_AVAILABLE = True
except ImportError:
    TQDM_AVAILABLE = False

#SAHI
try:
    from sahi import AutoDetectionModel
    from sahi.predict import get_sliced_prediction
    from sahi.utils.cv import read_image
    SAHI_AVAILABLE = True
except ImportError:
    SAHI_AVAILABLE = False
#PyTorch (still needed for some operations)
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
#YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
#TensorRT
try:
    import tensorrt as trt
    TENSORRT_AVAILABLE = True
except ImportError:
    TENSORRT_AVAILABLE = False

# Constants
# TUNED FOR MAXIMUM DETECTION - lower thresholds, smaller slices, more overlap
DEFAULT_CONFIDENCE_THRESHOLD = 0.05  # Very low to catch everything (was 0.10)
DEFAULT_SLICE_SIZE = 256  # Smaller slices for better small object detection (was 512)
DEFAULT_OVERLAP = 0.45  # Higher overlap to catch objects at boundaries (was 0.3)
DEFAULT_CHECK_INTERVAL = 2.0
MAX_SEARCH_DEPTH = 10

# Model format constants
MODEL_FORMAT_PYTORCH = 'pytorch'
MODEL_FORMAT_TENSORRT = 'tensorrt'
MODEL_FORMAT_AUTO = 'auto'

# Find ros2_ws directory without creating extra folders
def get_ros2_ws_directory() -> str:
    """Find the ros2_ws root directory by searching up from current file location."""
    # Try to find ros2_ws directory by looking for install/ or src/ directories
    current_file = os.path.abspath(__file__)
    current_dir = os.path.dirname(current_file)
    
    # Navigate up to find ros2_ws (look for install/ or src/ directories)
    search_dir = current_dir
    ros2_ws_dir = None
    
    for _ in range(MAX_SEARCH_DEPTH):  # Limit search depth
        if os.path.exists(os.path.join(search_dir, "install")) or os.path.exists(os.path.join(search_dir, "src")):
            # Check if this looks like ros2_ws (has both install and src, or just install)
            if os.path.exists(os.path.join(search_dir, "install")) and os.path.exists(os.path.join(search_dir, "src")):
                ros2_ws_dir = search_dir
                break
            # Also accept if we're in install/... and find the parent
            parent = os.path.dirname(search_dir)
            if os.path.exists(os.path.join(parent, "install")) and os.path.exists(os.path.join(parent, "src")):
                ros2_ws_dir = parent
                break
        search_dir = os.path.dirname(search_dir)
        if search_dir == "/":  # Reached root
            break
    
    # Fallback: use environment variable or default location
    if ros2_ws_dir is None:
        ros2_ws_dir = os.getenv('ROS2_WS_PATH') or os.path.expanduser('~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws')
    
    return ros2_ws_dir

class SAHIObjectDetectionNode(LifecycleNode):

    def __init__(self):
        # ROS2 node name - matches launch file
        super().__init__('new_od')
        
        # Lifecycle state flag
        self.shutdown_requested = False
        
        # Declare parameters with defaults
        self.declare_parameter('model_path', 'yolo26m.pt')
        self.declare_parameter('model_format', MODEL_FORMAT_AUTO)  # NEW: 'pytorch', 'tensorrt', or 'auto'
        self.declare_parameter('auto_convert_tensorrt', True)  # NEW: Auto-convert .pt to .engine if missing
        self.declare_parameter('tensorrt_workspace', 4)  # NEW: GPU memory for TensorRT (GB)
        self.declare_parameter('confidence_threshold', DEFAULT_CONFIDENCE_THRESHOLD)
        self.declare_parameter('slice_height', DEFAULT_SLICE_SIZE)
        self.declare_parameter('slice_width', DEFAULT_SLICE_SIZE)
        self.declare_parameter('overlap_height_ratio', DEFAULT_OVERLAP)
        self.declare_parameter('overlap_width_ratio', DEFAULT_OVERLAP)
        self.declare_parameter('check_interval', DEFAULT_CHECK_INTERVAL)
        self.declare_parameter('device', 'auto')
        # New parameters for improvements
        self.declare_parameter('max_images_per_cycle', 5)  # Batch processing
        self.declare_parameter('max_camera_feed_images', 100)  # Image cleanup
        self.declare_parameter('min_detection_area', 25)  # Filter small detections
        self.declare_parameter('max_detection_area', 1000000)  # Filter large detections
        self.declare_parameter('min_aspect_ratio', 0.1)  # Aspect ratio filtering
        self.declare_parameter('max_aspect_ratio', 10.0)  # Aspect ratio filtering
        self.declare_parameter('enable_gpu_memory_cleanup', True)  # GPU memory management
        
        # Initialize variables (will be set in lifecycle callbacks)
        self.bridge = None
        self.publisher = None
        self.detection_publisher = None
        self.detection_pub = None
        self.timer = None
        self.gpu_cleanup_timer = None
        self.stats_service = None
        self.health_service = None
        self.waypoint_subscription = None
        self.detection_model = None
        
        # Processing state
        self.processed_images: Dict[str, float] = {}  # Track processed images with timestamps
        self.is_processing = False  # Flag to prevent overlapping processing
        
        # Statistics
        self.stats = {
            'total_images_processed': 0,
            'total_detections': 0,
            'total_tents': 0,
            'total_people': 0,
            'total_objects': 0,  # Other detected objects
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
        
        # Waypoint tracking
        self.waypoint_reached = 0
        
        # Directory paths (will be set in on_configure)
        self.camera_feed_path = None
        self.detection_results_path = None
        self.device = None
        self.model_format_detected = None
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the node - setup parameters and directories"""
        self.get_logger().info("Configuring SAHI Object Detection Node...")
        
        try:
            # Get and validate parameters
            self._load_and_validate_parameters()
            
            # Setup directories - read images from src/video_cam/mapping_photos, save results to src/detection/detection_results_sahi
            ros2_ws_dir = get_ros2_ws_directory()
            
            # Camera feed path: ros2_ws/src/video_cam/mapping_photos
            self.camera_feed_path = os.path.join(ros2_ws_dir, "src", "video_cam", "mapping_photos")
            
            # Detection results path: ros2_ws/src/detection/detection_results_sahi
            self.detection_results_path = os.path.join(ros2_ws_dir, "src", "detection", "detection_results_sahi")
            
            # Create results directory (camera_feed should already exist with images)
            os.makedirs(self.detection_results_path, exist_ok=True)
            
            # Check if camera_feed exists and has images
            if not os.path.exists(self.camera_feed_path):
                self.get_logger().warn(f"Camera feed directory does not exist: {self.camera_feed_path}")
                self.get_logger().warn("Creating it, but you should place images there for detection")
                os.makedirs(self.camera_feed_path, exist_ok=True)
            
            self.get_logger().info(f"Detection results will be saved to: {self.detection_results_path}")
            
            # Auto-detect device (GPU, MPS, or CPU)
            if self.device == 'auto':
                self.device = self._get_device()
            self.get_logger().info(f"Using device: {self.device}")
            
            # Model format detection
            self.model_format_detected = self._detect_model_format()
            
            # Setup ROS2 bridge
            self.bridge = CvBridge()
            
            # Parameter callback for dynamic reconfiguration
            self.add_on_set_parameters_callback(self._parameter_callback)
            
            self.get_logger().info("Configuration complete")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate the node - initialize model, create publishers, timers, services"""
        self.get_logger().info("Activating SAHI Object Detection Node...")
        
        try:
            # Initialize SAHI model
            if not self.initialize_sahi_model():
                self.get_logger().error("Failed to initialize SAHI model")
                return TransitionCallbackReturn.FAILURE
            
            # QoS profile for reliable messaging
            qos_profile = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE
            )
            
            # Create publishers
            self.publisher = self.create_publisher(Image, '/sahi_detection_results', qos_profile)
            self.detection_publisher = self.create_publisher(String, '/sahi_detection_info', qos_profile)
            self.detection_pub = self.create_publisher(ImageResult, '/image_detections', qos_profile)
            
            # Create services
            from std_srvs.srv import Trigger
            self.stats_service = self.create_service(
                Trigger,
                'sahi/get_statistics',
                self._get_statistics_service
            )
            self.health_service = self.create_service(
                Trigger,
                'sahi/get_health',
                self._get_health_service
            )
            
            # Create subscriber
            self.waypoint_subscription = self.create_subscription(
                WaypointReached, 
                "/mavros/mission/reached", 
                self.waypoint_reached_cb, 
                10
            )
            
            # Create timer to check for new images
            self.timer = self.create_timer(self.check_interval, self.check_for_new_images)
            
            # GPU memory cleanup timer (if enabled)
            if self.enable_gpu_memory_cleanup and self.device.startswith('cuda'):
                self.gpu_cleanup_timer = self.create_timer(30.0, self._periodic_gpu_cleanup)
            
            # Reset stats timing
            self.stats['node_start_time'] = time.time()
            
            self.get_logger().info("SAHI Object Detection Node activated and ready")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Activation failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.FAILURE
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the node - stop processing"""
        self.get_logger().info("Deactivating SAHI Object Detection Node...")
        
        # Stop timers
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        
        if self.gpu_cleanup_timer is not None:
            self.gpu_cleanup_timer.cancel()
            self.gpu_cleanup_timer = None
        
        # Stop processing
        self.is_processing = False
        
        self.get_logger().info("Node deactivated")
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Cleanup the node - destroy publishers, services, subscribers"""
        self.get_logger().info("Cleaning up SAHI Object Detection Node...")
        
        # Destroy timers
        if self.timer is not None:
            self.timer.destroy()
            self.timer = None
        
        if self.gpu_cleanup_timer is not None:
            self.gpu_cleanup_timer.destroy()
            self.gpu_cleanup_timer = None
        
        # Destroy publishers
        if self.publisher is not None:
            self.publisher.destroy()
            self.publisher = None
        
        if self.detection_publisher is not None:
            self.detection_publisher.destroy()
            self.detection_publisher = None
        
        if self.detection_pub is not None:
            self.detection_pub.destroy()
            self.detection_pub = None
        
        # Destroy services
        if self.stats_service is not None:
            self.stats_service.destroy()
            self.stats_service = None
        
        if self.health_service is not None:
            self.health_service.destroy()
            self.health_service = None
        
        # Destroy subscriber
        if self.waypoint_subscription is not None:
            self.waypoint_subscription.destroy()
            self.waypoint_subscription = None
        
        # Clear model from memory (but don't delete it, will reload on activate)
        if self.detection_model is not None:
            del self.detection_model
            self.detection_model = None
        
        # Cleanup GPU memory
        if self.device and self.device.startswith('cuda') and TORCH_AVAILABLE:
            try:
                gc.collect()
                torch.cuda.empty_cache()
            except Exception as e:
                self.get_logger().debug(f"GPU cleanup error: {e}")
        
        self.bridge = None
        
        self.get_logger().info("Cleanup complete")
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown the node - final cleanup"""
        self.get_logger().info("Lifecycle shutdown")
        self.shutdown_requested = True
        
        # Final cleanup
        try:
            # Stop timers
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
            
            if self.gpu_cleanup_timer is not None:
                self.gpu_cleanup_timer.cancel()
                self.gpu_cleanup_timer = None
            
            # Destroy all resources
            if self.publisher is not None:
                self.publisher.destroy()
            if self.detection_publisher is not None:
                self.detection_publisher.destroy()
            if self.detection_pub is not None:
                self.detection_pub.destroy()
            if self.stats_service is not None:
                self.stats_service.destroy()
            if self.health_service is not None:
                self.health_service.destroy()
            if self.waypoint_subscription is not None:
                self.waypoint_subscription.destroy()
            
            # Cleanup model and GPU memory
            if self.detection_model is not None:
                del self.detection_model
                self.detection_model = None
            
            # Final GPU memory cleanup
            if TORCH_AVAILABLE:
                try:
                    gc.collect()
                    if torch.cuda.is_available():
                        torch.cuda.empty_cache()
                        torch.cuda.synchronize()
                except Exception as e:
                    self.get_logger().debug(f"Final GPU cleanup error: {e}")
            
            # Print final statistics
            self.get_logger().info("="*60)
            self.get_logger().info("Final Statistics:")
            self.get_logger().info(f"  Total Images Processed: {self.stats['total_images_processed']}")
            self.get_logger().info(f"  Total Detections: {self.stats['total_detections']}")
            self.get_logger().info(f"  Total People: {self.stats['total_people']}")
            self.get_logger().info(f"  Total Tents: {self.stats['total_tents']}")
            self.get_logger().info(f"  Avg Processing Time: {self.stats['avg_processing_time']:.2f}s")
            self.get_logger().info(f"  Errors: {self.stats['errors']}")
            uptime = time.time() - self.stats['node_start_time']
            self.get_logger().info(f"  Uptime: {uptime:.1f}s")
            self.get_logger().info("="*60)
            
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")
        
        return TransitionCallbackReturn.SUCCESS
    
    def _load_and_validate_parameters(self) -> None:
        """Load and validate all parameters"""
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.model_format = self.get_parameter('model_format').value.lower() if self.has_parameter('model_format') else MODEL_FORMAT_AUTO
        self.auto_convert_tensorrt = self.get_parameter('auto_convert_tensorrt').value if self.has_parameter('auto_convert_tensorrt') else True
        self.tensorrt_workspace = self.get_parameter('tensorrt_workspace').value if self.has_parameter('tensorrt_workspace') else 4
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.slice_height = self.get_parameter('slice_height').value
        self.slice_width = self.get_parameter('slice_width').value
        self.overlap_height_ratio = self.get_parameter('overlap_height_ratio').value
        self.overlap_width_ratio = self.get_parameter('overlap_width_ratio').value
        self.check_interval = self.get_parameter('check_interval').value
        self.device = self.get_parameter('device').value
        self.max_images_per_cycle = self.get_parameter('max_images_per_cycle').value
        self.max_camera_feed_images = self.get_parameter('max_camera_feed_images').value
        self.min_detection_area = self.get_parameter('min_detection_area').value
        self.max_detection_area = self.get_parameter('max_detection_area').value
        self.min_aspect_ratio = self.get_parameter('min_aspect_ratio').value
        self.max_aspect_ratio = self.get_parameter('max_aspect_ratio').value
        self.enable_gpu_memory_cleanup = self.get_parameter('enable_gpu_memory_cleanup').value
        
        # Validate model_format
        if self.model_format not in [MODEL_FORMAT_PYTORCH, MODEL_FORMAT_TENSORRT, MODEL_FORMAT_AUTO]:
            self.get_logger().warn(f"Invalid model_format '{self.model_format}', using 'auto'")
            self.model_format = MODEL_FORMAT_AUTO
        
        # Validate parameters
        if not 0 < self.confidence_threshold <= 1.0:
            self.get_logger().warn(f"Invalid confidence_threshold: {self.confidence_threshold}, using default: {DEFAULT_CONFIDENCE_THRESHOLD}")
            self.confidence_threshold = DEFAULT_CONFIDENCE_THRESHOLD
        
        if self.slice_height < 64 or self.slice_width < 64:
            self.get_logger().warn(f"Slice size too small: {self.slice_height}x{self.slice_width}, minimum is 64x64")
            self.slice_height = max(64, self.slice_height)
            self.slice_width = max(64, self.slice_width)
        
        if not 0 <= self.overlap_height_ratio < 1.0 or not 0 <= self.overlap_width_ratio < 1.0:
            self.get_logger().warn(f"Invalid overlap ratio, using default: {DEFAULT_OVERLAP}")
            self.overlap_height_ratio = DEFAULT_OVERLAP
            self.overlap_width_ratio = DEFAULT_OVERLAP
        
        if self.check_interval < 0.1:
            self.get_logger().warn(f"Check interval too small: {self.check_interval}, using default: {DEFAULT_CHECK_INTERVAL}")
            self.check_interval = DEFAULT_CHECK_INTERVAL
        
        if self.max_images_per_cycle < 1:
            self.get_logger().warn(f"max_images_per_cycle must be >= 1, using 1")
            self.max_images_per_cycle = 1

    def _detect_model_format(self) -> str:
        """Detect model format from file extension."""
        if self.model_path.endswith('.engine'):
            return MODEL_FORMAT_TENSORRT
        elif self.model_path.endswith('.pt'):
            return MODEL_FORMAT_PYTORCH
        else:
            self.get_logger().warn(f"Unknown model format for {self.model_path}. Assuming PyTorch.")
            return MODEL_FORMAT_PYTORCH
    
    def _convert_pytorch_to_tensorrt(self, pt_path: str, engine_path: str) -> bool:
        """
        Convert PyTorch model to TensorRT engine.
        
        Args:
            pt_path: Path to PyTorch .pt model
            engine_path: Path to save TensorRT .engine file
            
        Returns:
            True if conversion successful, False otherwise
        """
        if not YOLO_AVAILABLE:
            self.get_logger().error("Ultralytics YOLO not available for conversion")
            return False
        
        if not TENSORRT_AVAILABLE:
            self.get_logger().error("TensorRT not available. Install: pip install nvidia-tensorrt")
            return False
        
        try:
            self.get_logger().info(f"Converting {pt_path} to TensorRT format...")
            self.get_logger().info("This may take several minutes on first run...")
            
            # Load PyTorch model
            model = YOLO(pt_path)
            
            # Export to TensorRT
            model.export(
                format='engine',
                device=0 if self.device.startswith('cuda') else 'cpu',
                workspace=self.tensorrt_workspace,
                simplify=True,
                verbose=False
            )
            
            # Find the exported engine file
            base_name = os.path.splitext(pt_path)[0]
            exported_engine = f"{base_name}.engine"
            
            if os.path.exists(exported_engine):
                # Move to desired location if different
                if exported_engine != engine_path:
                    import shutil
                    shutil.move(exported_engine, engine_path)
                self.get_logger().info(f"✓ TensorRT conversion successful: {engine_path}")
                return True
            else:
                self.get_logger().error(f"TensorRT engine file not found at {exported_engine}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"TensorRT conversion failed: {str(e)}")
            return False
    
    def _resolve_model_path(self) -> Tuple[str, str]:
        """
        Resolve model path and determine final format.
        
        Returns:
            Tuple of (resolved_model_path, final_format)
        """
        model_path = self.model_path
        ros2_ws_dir = get_ros2_ws_directory()
        
        # Resolve relative paths
        if not os.path.isabs(model_path):
            # Try ros2_ws directory
            ros2_ws_model_path = os.path.join(ros2_ws_dir, model_path)
            if os.path.exists(ros2_ws_model_path):
                model_path = ros2_ws_model_path
            # Try package directory
            elif os.path.exists(os.path.join(os.path.dirname(__file__), model_path)):
                model_path = os.path.join(os.path.dirname(__file__), model_path)
            # Try current directory
            elif not os.path.exists(model_path):
                self.get_logger().warn(
                    f"Model file not found at {self.model_path}, "
                    f"will download from Ultralytics if needed"
                )
        
        # Determine format based on parameter and file extension
        if self.model_format == MODEL_FORMAT_AUTO:
            # Auto-detect: prefer TensorRT if available, fallback to PyTorch
            base_name = os.path.splitext(model_path)[0]
            engine_path = f"{base_name}.engine"
            
            if os.path.exists(engine_path):
                self.get_logger().info(f"Found TensorRT engine: {engine_path}")
                return engine_path, MODEL_FORMAT_TENSORRT
            elif model_path.endswith('.pt') and self.auto_convert_tensorrt and TENSORRT_AVAILABLE:
                # Try to convert PyTorch to TensorRT
                if self._convert_pytorch_to_tensorrt(model_path, engine_path):
                    return engine_path, MODEL_FORMAT_TENSORRT
                else:
                    self.get_logger().warn("TensorRT conversion failed, using PyTorch")
                    return model_path, MODEL_FORMAT_PYTORCH
            else:
                return model_path, self.model_format_detected
        elif self.model_format == MODEL_FORMAT_TENSORRT:
            # Force TensorRT - check if .engine exists, convert if needed
            if model_path.endswith('.engine'):
                if os.path.exists(model_path):
                    return model_path, MODEL_FORMAT_TENSORRT
            else:
                # Convert .pt to .engine
                base_name = os.path.splitext(model_path)[0]
                engine_path = f"{base_name}.engine"
                if os.path.exists(engine_path):
                    return engine_path, MODEL_FORMAT_TENSORRT
                elif model_path.endswith('.pt') and os.path.exists(model_path):
                    if self._convert_pytorch_to_tensorrt(model_path, engine_path):
                        return engine_path, MODEL_FORMAT_TENSORRT
                    else:
                        self.get_logger().error("TensorRT conversion required but failed")
                        return None, None
        else:
            # PyTorch format
            return model_path, MODEL_FORMAT_PYTORCH

    def waypoint_reached_cb(self, msg: WaypointReached) -> None:
        """Callback for waypoint reached messages"""
        self.waypoint_reached = msg.wp_seq
    
    def _parameter_callback(self, params: List[Parameter]) -> rclpy.node.SetParametersResult:
        """
        Handle parameter changes at runtime
        
        Args:
            params: List of parameters being set
        
        Returns:
            SetParametersResult indicating success or failure
        """
        from rclpy.node import SetParametersResult
        for param in params:
            try:
                if param.name == 'confidence_threshold':
                    if 0 < param.value <= 1.0:
                        self.confidence_threshold = param.value
                        self.get_logger().info(f"Updated confidence_threshold to {param.value}")
                    else:
                        return SetParametersResult(successful=False, reason="confidence_threshold must be between 0 and 1")
                elif param.name == 'check_interval':
                    if param.value >= 0.1:
                        self.check_interval = param.value
                        # Only recreate timer if it exists (node is active)
                        if self.timer is not None:
                            self.timer.cancel()
                            self.timer.destroy()
                            self.timer = self.create_timer(self.check_interval, self.check_for_new_images)
                        self.get_logger().info(f"Updated check_interval to {param.value}")
                    else:
                        return SetParametersResult(successful=False, reason="check_interval must be >= 0.1")
                elif param.name == 'max_images_per_cycle':
                    if param.value >= 1:
                        self.max_images_per_cycle = param.value
                        self.get_logger().info(f"Updated max_images_per_cycle to {param.value}")
                    else:
                        return SetParametersResult(successful=False, reason="max_images_per_cycle must be >= 1")
            except Exception as e:
                self.get_logger().error(f"Error updating parameter {param.name}: {e}")
                return SetParametersResult(successful=False, reason=str(e))
        
        return SetParametersResult(successful=True)
    
    def _get_statistics_service(self, request, response):
        """Service callback to get detection statistics"""
        from std_srvs.srv import Trigger
        stats_str = (
            f"SAHI Detection Statistics:\n"
            f"  Total Images Processed: {self.stats['total_images_processed']}\n"
            f"  Total Detections: {self.stats['total_detections']}\n"
            f"  Total Tents: {self.stats['total_tents']}\n"
            f"  Total People: {self.stats['total_people']}\n"
            f"  Avg Processing Time: {self.stats['avg_processing_time']:.2f}s\n"
            f"  Last Processing Time: {self.stats['last_processing_time']:.2f}s\n"
            f"  Errors: {self.stats['errors']}\n"
            f"  Uptime: {time.time() - self.stats['node_start_time']:.1f}s"
        )
        self.get_logger().info(f"Statistics requested:\n{stats_str}")
        response.success = True
        response.message = stats_str
        return response
    
    def _get_health_service(self, request, response):
        """Service callback to get node health status"""
        from std_srvs.srv import Trigger
        health_str = (
            f"Node Health Status:\n"
            f"  Is Healthy: {self.health_status['is_healthy']}\n"
            f"  Consecutive Errors: {self.health_status['consecutive_errors']}\n"
            f"  Last Successful Detection: {self.health_status['last_successful_detection']}\n"
            f"  Model Initialized: {self.detection_model is not None}\n"
            f"  Model Format: {self.model_format_detected}"
        )
        self.get_logger().info(f"Health check requested:\n{health_str}")
        response.success = self.health_status['is_healthy']
        response.message = health_str
        return response
    
    def _periodic_gpu_cleanup(self) -> None:
        """Periodically clean up GPU memory"""
        if self.device.startswith('cuda') and TORCH_AVAILABLE:
            try:
                gc.collect()
                torch.cuda.empty_cache()
                if TORCH_AVAILABLE:
                    allocated = torch.cuda.memory_allocated(0) / 1024**3
                    reserved = torch.cuda.memory_reserved(0) / 1024**3
                    self.get_logger().debug(f"GPU Memory: {allocated:.2f}GB allocated, {reserved:.2f}GB reserved")
            except Exception as e:
                self.get_logger().debug(f"GPU cleanup error: {e}")
    
    def _get_device(self) -> str:
        """
        Auto-detect the best available device (CUDA, MPS, or CPU)
        
        Priority order:
        1. NVIDIA GPU (CUDA) - for Linux, Windows, Jetson
        2. Apple Silicon GPU (MPS) - for Mac M1/M2/M3
        3. CPU - fallback
        
        Returns:
            str: Device string ('cuda:0', 'mps', or 'cpu')
        """
        if not TORCH_AVAILABLE:
            self.get_logger().warn("PyTorch not available, falling back to CPU")
            return "cpu"
        
        # Try NVIDIA CUDA (works on Linux, Windows, Jetson)
        if torch.cuda.is_available():
            device_name = torch.cuda.get_device_name(0)
            device_count = torch.cuda.device_count()
            compute_capability = torch.cuda.get_device_capability(0)
            
            self.get_logger().info(f" CUDA GPU Detected!")
            # self.get_logger().info(f"   Device: {device_name}")
            # self.get_logger().info(f"   GPU Count: {device_count}")
            # self.get_logger().info(f"   Compute Capability: {compute_capability[0]}.{compute_capability[1]}")
            # self.get_logger().info(f"   CUDA Version: {torch.version.cuda}")
            
            # Check if this is a Jetson device
            try:
                with open('/proc/device-tree/model', 'r') as f:
                    model = f.read()
                    if 'jetson' in model.lower():
                        self.get_logger().info(f"   Platform: NVIDIA Jetson ({model.strip()})")
            except (OSError, IOError, FileNotFoundError):
                pass  # Not a Jetson device or can't read device tree
            
            return "cuda:0"
        
        # Try Apple Metal Performance Shaders (MPS) for Mac
        elif hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
            self.get_logger().info(" Apple Silicon GPU (MPS) Detected!")
            # self.get_logger().info("   Using Metal Performance Shaders for acceleration")
            
            # Check if MPS is actually built
            if not torch.backends.mps.is_built():
                self.get_logger().warn("   MPS is available but not built, falling back to CPU")
                return "cpu"
            
            return "mps"
        
        # Fallback to CPU
        else:
            import platform
            self.get_logger().warn("  No GPU detected, using CPU")
            self.get_logger().info(f"   System: {platform.system()} {platform.machine()}")
            self.get_logger().info(f"   CPU Count: {os.cpu_count()}")
            
            # Check if we're on Jetson but GPU not available
            try:
                with open('/proc/device-tree/model', 'r') as f:
                    model = f.read()
                    if 'jetson' in model.lower():
                        self.get_logger().error("     JETSON DEVICE DETECTED BUT GPU NOT AVAILABLE!")
                        self.get_logger().error("   This Jetson has a GPU but PyTorch cannot access it.")
                        self.get_logger().error("   Possible reasons:")
                        self.get_logger().error("   1. PyTorch installed without CUDA support (CPU-only version)")
                        self.get_logger().error("   2. CUDA drivers not properly installed")
                        self.get_logger().error("   3. Incompatible PyTorch/CUDA version")
                        self.get_logger().error("")
                        self.get_logger().error("   To fix:")
                        self.get_logger().error("   1. Run: python3 check_gpu.py (in this directory)")
                        self.get_logger().error("   2. Install PyTorch with CUDA for Jetson:")
                        self.get_logger().error("      Visit: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048")
                        self.get_logger().error("   3. Or run: bash install_pytorch_cuda.sh")
                        self.get_logger().error("")
                        if TORCH_AVAILABLE:
                            self.get_logger().error(f"   Current PyTorch version: {torch.__version__}")
                        self.get_logger().error("   Expected: PyTorch with CUDA support (not CPU-only)")
            except (OSError, IOError, FileNotFoundError):
                pass  # Not a Jetson device or can't read device tree
            
            self.get_logger().info("   Consider using a GPU for better performance!")
            
            return "cpu"
    
    def initialize_sahi_model(self):
        """Initialize SAHI detection model with YOLO26/TensorRT"""
        try:
            if not SAHI_AVAILABLE:
                self.get_logger().error("SAHI is not available. Please install: pip install sahi")
                return False
            
            if not YOLO_AVAILABLE:
                self.get_logger().error("Ultralytics YOLO is not available. Please install: pip install ultralytics")
                return False
            
            # Resolve model path and format
            resolved_path, final_format = self._resolve_model_path()
            if resolved_path is None:
                self.get_logger().error("Failed to resolve model path")
                return False
            
            self.get_logger().info(f"Loading model: {resolved_path} (format: {final_format})")
            
            # GPU memory optimization for CUDA
            if self.device.startswith('cuda') and TORCH_AVAILABLE:
                try:
                    # Aggressive memory cleanup before loading model
                    import gc
                    gc.collect()
                    torch.cuda.empty_cache()
                    torch.cuda.synchronize()
                    
                    # Set memory allocator settings for Jetson
                    # Use more aggressive garbage collection
                    torch.cuda.set_per_process_memory_fraction(0.8, 0)  # Use max 80% of GPU memory
                    
                    # Enable cuDNN benchmarking for consistent input sizes
                    torch.backends.cudnn.benchmark = True
                    
                    # Disable debug mode for better performance
                    torch.backends.cudnn.enabled = True
                    
                    # Get GPU memory info
                    gpu_mem_total = torch.cuda.get_device_properties(0).total_memory / 1024**3  # GB
                    gpu_mem_reserved = torch.cuda.memory_reserved(0) / 1024**3
                    gpu_mem_allocated = torch.cuda.memory_allocated(0) / 1024**3
                    gpu_mem_free = gpu_mem_total - gpu_mem_allocated
                    
                    # self.get_logger().info(f"   GPU Memory: {gpu_mem_free:.2f}GB free / {gpu_mem_total:.2f}GB total")
                    
                    # Warn if low memory
                    if gpu_mem_free < 2.0:
                        self.get_logger().warn(f"   ⚠️  Low GPU memory ({gpu_mem_free:.2f}GB)")
                        self.get_logger().warn(f"   Consider using a smaller model or reducing slice size")
                        self.get_logger().warn(f"   Run: bash fix_gpu_memory.sh")
                except Exception as e:
                    self.get_logger().debug(f"Could not get GPU memory info: {e}")
            
            # Initialize SAHI AutoDetectionModel
            # Note: SAHI uses 'yolo26' as the model_type identifier for YOLO26 models
            # For TensorRT, we still use the same model_type, but with .engine file
            if final_format == MODEL_FORMAT_TENSORRT:
                # TensorRT models - try loading with Ultralytics first
                try:
                    # Load TensorRT engine with Ultralytics YOLO26
                    # SAHI should handle this, but may need model_type adjustment
                    self.detection_model = AutoDetectionModel.from_pretrained(
                        model_type='yolo26',  # SAHI model type identifier for YOLO26
                        model_path=resolved_path,  # TensorRT .engine file
                        confidence_threshold=self.confidence_threshold,
                        device=self.device,
                    )
                    self.get_logger().info("✓ TensorRT model loaded via SAHI")
                except Exception as e:
                    self.get_logger().error(f"Failed to load TensorRT model: {e}")
                    self.get_logger().error("Falling back to PyTorch if available...")
                    # Try fallback to PyTorch
                    base_name = os.path.splitext(resolved_path)[0]
                    pt_path = f"{base_name}.pt"
                    if os.path.exists(pt_path):
                        self.get_logger().warn(f"Using PyTorch fallback: {pt_path}")
                        self.detection_model = AutoDetectionModel.from_pretrained(
                            model_type='yolo26',
                            model_path=pt_path,
                            confidence_threshold=self.confidence_threshold,
                            device=self.device,
                        )
                        self.model_format_detected = MODEL_FORMAT_PYTORCH
                    else:
                        return False
            else:
                # PyTorch model - standard SAHI loading
                self.detection_model = AutoDetectionModel.from_pretrained(
                    model_type='yolo26',  # SAHI model type identifier for YOLO26
                    model_path=resolved_path,  # Actual model: yolo11s.pt
                    confidence_threshold=self.confidence_threshold,
                    device=self.device,
                )
                self.get_logger().info("✓ PyTorch model loaded via SAHI")
            
            # self.get_logger().info(" SAHI YOLOv11s model loaded successfully!")
            
            # Verify model is on correct device
            if TORCH_AVAILABLE and hasattr(self.detection_model, 'model'):
                try:
                    model_device = next(self.detection_model.model.model.parameters()).device
                    # self.get_logger().info(f" Model confirmed on device: {model_device}")
                except (AttributeError, StopIteration, RuntimeError) as e:
                    self.get_logger().debug(f"Could not verify model device: {e}")
            
            return True
            
        except (ImportError, FileNotFoundError, RuntimeError, OSError) as e:
            self.get_logger().error(f"Failed to initialize SAHI model: {e}")
            self.get_logger().error("Make sure you have installed: pip install sahi ultralytics torch")
            if final_format == MODEL_FORMAT_TENSORRT:
                self.get_logger().error("For TensorRT: pip install nvidia-tensorrt")
            
            # If GPU failed, suggest fallback to CPU
            if self.device != 'cpu':
                self.get_logger().error(f"Try running with CPU instead: --ros-args -p device:=cpu")
            
            self.stats['errors'] += 1
            self.health_status['consecutive_errors'] += 1
            self.health_status['is_healthy'] = False
            return False
    
    def check_for_new_images(self) -> None:
        """
        Check for new images in camera_feed folder and process them one by one.
        Also performs image cleanup if configured.
        """
        try:
            # Only process if node is active
            if self.get_current_state().id != State.PRIMARY_STATE_ACTIVE:
                return
            
            if self.shutdown_requested:
                return
            
            if not os.path.exists(self.camera_feed_path):
                self.get_logger().warn(f"Camera feed path does not exist: {self.camera_feed_path}")
                return
            
            if self.detection_model is None:
                self.get_logger().warn("SAHI model not initialized, skipping detection")
                return
            
            # Skip if already processing
            if self.is_processing:
                return
            
            # Clean up old images if needed
            self._cleanup_old_images()
            
            # Get all image files with timestamps
            image_files_with_time: List[Tuple[str, float]] = []
            for file in os.listdir(self.camera_feed_path):
                if file.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')):
                    file_path = os.path.join(self.camera_feed_path, file)
                    try:
                        # Use modification time as creation time proxy
                        mtime = os.path.getmtime(file_path)
                        image_files_with_time.append((file, mtime))
                    except OSError:
                        continue
            
            # Sort by timestamp (oldest first)
            image_files_with_time.sort(key=lambda x: x[1])
            
            # Find first unprocessed image
            for fname, mtime in image_files_with_time:
                if fname not in self.processed_images:
                    # Process this single image
                    image_path = os.path.join(self.camera_feed_path, fname)
                    self.is_processing = True
                    
                    try:
                        self.process_image(image_path)
                        # Update health on success
                        self.health_status['last_successful_detection'] = time.time()
                        self.health_status['consecutive_errors'] = 0
                        self.health_status['is_healthy'] = True
                    except Exception as e:
                        self.get_logger().error(f"Error processing image {image_path}: {e}")
                        import traceback
                        self.get_logger().error(traceback.format_exc())
                        self.stats['errors'] += 1
                        self.health_status['consecutive_errors'] += 1
                    finally:
                        # Mark as processed regardless of success/failure
                        self.processed_images[fname] = mtime
                        self.is_processing = False
                    
                    # Only process one image per timer cycle
                    break
            
        except (OSError, IOError) as e:
            self.get_logger().error(f"Error checking for new images: {e}")
            self.stats['errors'] += 1
            self.health_status['consecutive_errors'] += 1
        except Exception as e:
            self.get_logger().error(f"Unexpected error checking for new images: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.stats['errors'] += 1
            self.health_status['consecutive_errors'] += 1
    
    def _cleanup_old_images(self) -> None:
        """Remove old images from camera_feed if limit is exceeded"""
        if self.max_camera_feed_images <= 0:
            return  # Disabled
        
        try:
            # Get all image files with timestamps
            image_files_with_time: List[Tuple[str, float]] = []
            for file in os.listdir(self.camera_feed_path):
                if file.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')):
                    file_path = os.path.join(self.camera_feed_path, file)
                    try:
                        mtime = os.path.getmtime(file_path)
                        image_files_with_time.append((file, mtime))
                    except OSError:
                        continue
            
            # Sort by timestamp (oldest first)
            image_files_with_time.sort(key=lambda x: x[1])
            
            # Remove oldest images if over limit
            if len(image_files_with_time) > self.max_camera_feed_images:
                to_remove = len(image_files_with_time) - self.max_camera_feed_images
                removed = 0
                for file, _ in image_files_with_time[:to_remove]:
                    file_path = os.path.join(self.camera_feed_path, file)
                    try:
                        os.remove(file_path)
                        # Remove from processed set if it was there
                        self.processed_images.pop(file, None)
                        removed += 1
                    except OSError as e:
                        self.get_logger().debug(f"Could not remove {file}: {e}")
                
                if removed > 0:
                    self.get_logger().info(f"Cleaned up {removed} old images from camera_feed")
        except (OSError, IOError) as e:
            self.get_logger().debug(f"Error during image cleanup: {e}")
    
    def process_image(self, image_path: str) -> None:
        """
        Process a single image using SAHI for small object detection.
        
        Args:
            image_path: Path to the image file to process
        """
        try:
            start_time = time.time()
            
            # Load image
            frame = cv2.imread(image_path)
            if frame is None:
                self.get_logger().warn(f"Could not load image: {image_path}")
                return
            
            height, width = frame.shape[:2]
            # self.get_logger().info(f"Processing image: {os.path.basename(image_path)} ({width}x{height})")
            
            # Run SAHI prediction
            detections = self.detect_objects_sahi(frame)
            
            processing_time = time.time() - start_time
            self.stats['last_processing_time'] = processing_time
            
            # Create annotated frame
            annotated_frame = self.annotate_frame(frame, detections, processing_time)
            
            self.get_logger().info(f"{self.detection_pub.publish(ImageResult())}")
            
            # Publish results
            self.publish_results(annotated_frame, detections, image_path)
            
            # Update statistics
            self.stats['total_images_processed'] += 1
            self.stats['total_detections'] += len(detections)
            self.stats['total_tents'] += sum(1 for d in detections if d['class'] == 'tent')
            self.stats['total_people'] += sum(1 for d in detections if d['class'] == 'person')
            self.stats['total_objects'] += sum(1 for d in detections if d['class'] == 'object')
            
            # Update average processing time
            n = self.stats['total_images_processed']
            self.stats['avg_processing_time'] = (
                (self.stats['avg_processing_time'] * (n - 1) + processing_time) / n
            )
            
            # Count by class for logging
            num_people = sum(1 for d in detections if d['class'] == 'person')
            num_tents = sum(1 for d in detections if d['class'] == 'tent')
            num_objects = sum(1 for d in detections if d['class'] == 'object')
            
            self.get_logger().info(
                f" Found {len(detections)} objects in {processing_time:.2f}s: "
                f"{num_people} people, {num_tents} tents, {num_objects} other objects"
            )
                
        except (cv2.error, OSError, IOError) as e:
            self.get_logger().error(f"Error processing image {image_path}: {e}")
            self.stats['errors'] += 1
        except Exception as e:
            self.get_logger().error(f"Unexpected error processing image {image_path}: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.stats['errors'] += 1
    
    def detect_objects_sahi(self, frame: np.ndarray) -> List[Dict]:
        """
        Detect objects using SAHI (Slicing Aided Hyper Inference)
        
        SAHI slices the image into smaller patches with overlap, runs detection
        on each patch, then merges the results. This is highly effective for
        detecting small objects in large images (e.g., tents in aerial photos).
        
        Args:
            frame: Input image as numpy array (BGR format)
            
        Returns:
            List of detection dictionaries
        """
        detections: List[Dict] = []
        
        try:
            # Convert BGR to RGB for SAHI (openCV loads in BGR and pytorch wants in RGB)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Calculate estimated number of slices for progress indication
            h, w = frame_rgb.shape[:2]
            stride_h = int(self.slice_height * (1 - self.overlap_height_ratio))
            stride_w = int(self.slice_width * (1 - self.overlap_width_ratio))
            num_slices_h = max(1, (h - self.slice_height) // stride_h + 1) if stride_h > 0 else 1
            num_slices_w = max(1, (w - self.slice_width) // stride_w + 1) if stride_w > 0 else 1
            total_slices = num_slices_h * num_slices_w
            
            # self.get_logger().info(
            #     f"Running SAHI prediction with {self.slice_height}x{self.slice_width} slices, "
            #     f"{self.overlap_height_ratio:.1%}x{self.overlap_width_ratio:.1%} overlap..."
            # )
            # self.get_logger().info(f"  Estimated slices: {total_slices} ({num_slices_h}x{num_slices_w} grid)")
            
            # Show progress bar in terminal
            if TQDM_AVAILABLE:
                print(f"\n Processing {w}x{h} image with ~{total_slices} slices...")
                pbar = tqdm(total=100, desc="SAHI Detection", unit="%", ncols=80,
                           bar_format='{l_bar}{bar}| {n_fmt}/{total_fmt} [{elapsed}<{remaining}]')
            
            start_time = time.time()
            
            # Run SAHI sliced prediction
            result = get_sliced_prediction(
                frame_rgb,
                self.detection_model,
                slice_height=self.slice_height,
                slice_width=self.slice_width,
                overlap_height_ratio=self.overlap_height_ratio,
                overlap_width_ratio=self.overlap_width_ratio,
                postprocess_type="NMS",  # Non-Maximum Suppression 'NMS' (when multiple boxes detect the same object, only the highest confidence one is staying)
                postprocess_match_metric="IOS",  # Intersection Over Smaller area (decides which one of boxes that detect the object should represent the object)
                postprocess_match_threshold=0.5,  # Threshold for merging detections( if overlap is >50% they are duplicates and merged post-processing)
                postprocess_class_agnostic=False,  # Class-aware NMS (makes sure that person and tent boxes dont merge)
                verbose=0
            )
            
            # Close progress bar
            if TQDM_AVAILABLE:
                pbar.update(100)  # Complete the progress bar
                pbar.close()
                elapsed = time.time() - start_time
                print(f" Detection complete in {elapsed:.1f}s - found {len(result.object_prediction_list)} raw detections\n")
            
            # self.get_logger().info(f"SAHI found {len(result.object_prediction_list)} raw detections")
            
            # Convert SAHI results to our format
            for object_prediction in result.object_prediction_list:
                # Get bounding box
                bbox = object_prediction.bbox
                x1, y1, x2, y2 = int(bbox.minx), int(bbox.miny), int(bbox.maxx), int(bbox.maxy)
                
                # Get class and confidence
                class_name = object_prediction.category.name
                confidence = float(object_prediction.score.value)
                
                # Filter and categorize detections
                detection = self._categorize_detection(
                    class_name, confidence, [x1, y1, x2, y2], frame
                )
                
                if detection:
                    detections.append(detection)
            
            # Apply additional filtering
            detections = self._filter_detections(detections)
            
            # Clean up GPU memory after detection (important for Jetson)
            if self.enable_gpu_memory_cleanup and self.device.startswith('cuda') and TORCH_AVAILABLE:
                try:
                    torch.cuda.empty_cache()
                except (RuntimeError, AttributeError):
                    pass
            
        except (RuntimeError, AttributeError, ImportError) as e:
            self.get_logger().error(f"Error in SAHI detection: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.stats['errors'] += 1
        
        return detections
    
    def _categorize_detection(self, class_name: str, confidence: float, bbox: List[int], frame) -> Optional[Dict]:
        """
        Categorize ALL YOLO detections and classify them into target categories.
        
        Strategy: Detect EVERYTHING, then categorize:
        - 'person': Direct person detections + mannequin-like objects
        - 'tent': Objects that look like tents/tarps/shelters
        - 'object': Everything else (still detected and shown!)
        
        This ensures we never miss detections - we see everything YOLO finds.
        
        Args:
            class_name: YOLO class name
            confidence: Detection confidence
            bbox: Bounding box [x1, y1, x2, y2]
            frame: Original image frame (unused but kept for compatibility)
            
        Returns:
            Detection dict or None if filtered out by area/aspect ratio
        """
        x1, y1, x2, y2 = bbox
        width = x2 - x1
        height = y2 - y1
        area = width * height
        aspect_ratio = width / height if height > 0 else 0
        
        # Apply area filtering (still filter tiny noise and huge false positives)
        if area < self.min_detection_area or area > self.max_detection_area:
            return None
        
        # Apply aspect ratio filtering
        if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
            return None
        
        # Minimum confidence for any detection (very low to catch everything)
        if confidence < 0.05:
            return None
        
        # === CLASSIFICATION LOGIC ===
        
        # PERSON: Direct person detection
        if class_name == 'person':
            return {
                'class': 'person',
                'yolo_class': class_name,
                'confidence': confidence,
                'bbox': bbox,
                'description': 'person',
                'method': 'sahi+yolo11s+tensorrt' if self.model_format_detected == MODEL_FORMAT_TENSORRT else 'sahi+yolo11s',
                'area': area,
                'is_target': True  # Flag as primary target
            }
        
        # PERSON-LIKE: Objects that could be people/mannequins from aerial view
        person_like_classes = ['doll', 'teddy bear']
        if class_name in person_like_classes:
            return {
                'class': 'person',
                'yolo_class': class_name,
                'confidence': confidence,
                'bbox': bbox,
                'description': f'person-like ({class_name})',
                'method': 'sahi+yolo11s+tensorrt' if self.model_format_detected == MODEL_FORMAT_TENSORRT else 'sahi+yolo11s',
                'area': area,
                'is_target': True
            }
        
        # TENT-LIKE: Objects that commonly represent tents/tarps/shelters
        tent_like_classes = [
            'umbrella', 'kite', 'bed', 'couch', 'boat', 
            'backpack', 'suitcase', 'handbag', 'surfboard', 'bench',
            'airplane', 'truck', 'car', 'bus', 'frisbee'
        ]
        if class_name in tent_like_classes:
            return {
                'class': 'tent',
                'yolo_class': class_name,
                'confidence': confidence,
                'bbox': bbox,
                'description': f'tent-like ({class_name})',
                'method': 'sahi+yolo11s+tensorrt' if self.model_format_detected == MODEL_FORMAT_TENSORRT else 'sahi+yolo11s',
                'area': area,
                'is_target': True
            }
        
        # EVERYTHING ELSE: Still detect it! Just classify as 'object'
        # This ensures we never miss anything - user can see what YOLO found
        return {
            'class': 'object',
            'yolo_class': class_name,
            'confidence': confidence,
            'bbox': bbox,
            'description': f'detected: {class_name}',
            'method': 'sahi+yolo11s+tensorrt' if self.model_format_detected == MODEL_FORMAT_TENSORRT else 'sahi+yolo11s',
            'area': area,
            'is_target': False  # Not a primary target, but still shown
        }
    
    def _filter_detections(self, detections: List[Dict]) -> List[Dict]:
        """
        Filter detections to remove duplicates and low-quality detections
        
        Args:
            detections: List of detection dicts
            
        Returns:
            Filtered list of detections
        """
        if len(detections) == 0:
            return detections
        
        # Sort by confidence (highest first)
        detections = sorted(detections, key=lambda x: x['confidence'], reverse=True)
        
        # Apply NMS within each class (person, tent, and object)
        filtered = []
        for target_class in ['person', 'tent', 'object']:
            class_detections = [d for d in detections if d['class'] == target_class]
            
            if len(class_detections) > 0:
                # Apply NMS
                keep = self._apply_nms(class_detections, overlap_threshold=0.3)
                filtered.extend(keep)
        
        return filtered
    
    def _apply_nms(self, detections: List[Dict], overlap_threshold: float = 0.3) -> List[Dict]:
        """Apply Non-Maximum Suppression to remove overlapping detections"""
        if len(detections) == 0:
            return detections
        
        # Already sorted by confidence
        keep = []
        remaining = detections.copy()
        
        while remaining:
            # Take the detection with highest confidence
            current = remaining.pop(0)
            keep.append(current)
            
            # Remove detections that overlap significantly with current
            new_remaining = []
            for detection in remaining:
                iou = self._calculate_iou(current['bbox'], detection['bbox'])
                if iou < overlap_threshold:
                    new_remaining.append(detection)
                else:
                    # Log filtered detection
                    self.get_logger().debug(
                        f"Filtered overlapping {detection['class']} "
                        f"(IoU={iou:.2f}, conf={detection['confidence']:.2f})"
                    )
            remaining = new_remaining
        
        return keep
    
    def _calculate_iou(self, bbox1: List[int], bbox2: List[int]) -> float:
        """Calculate Intersection over Union (IoU) of two bounding boxes"""
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        
        # Calculate intersection
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i <= x1_i or y2_i <= y1_i:
            return 0.0
        
        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        
        # Calculate union
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0
    
    def annotate_frame(self, frame: np.ndarray, detections: List[Dict], processing_time: float) -> np.ndarray:
        """
        Annotate frame with SAHI detection results in the style of the reference images
        
        Args:
            frame: Original image as numpy array
            detections: List of detection dicts
            processing_time: Time taken for detection
            
        Returns:
            Annotated frame as numpy array
        """
        annotated_frame = frame.copy()
        height, width = frame.shape[:2]
        
        # Define colors (BGR format)
        COLOR_TENT = (0, 200, 255)    # Orange-yellow for tents (more visible)
        COLOR_PERSON = (0, 200, 0)    # Green for people
        COLOR_OBJECT = (255, 100, 0)  # Blue for other objects (no labels)
        COLOR_WHITE = (255, 255, 255) # White text
        COLOR_BLACK = (0, 0, 0)       # Black for outlines/shadows
        
        # Use a cleaner font with better scaling
        # FONT_HERSHEY_DUPLEX has better quality than SIMPLEX
        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = 0.45  # Slightly smaller for cleaner look
        font_thickness = 1  # Thin strokes for clarity
        
        # Annotate each detection
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            class_name = detection['class']
            confidence = detection['confidence']
            yolo_class = detection.get('yolo_class', class_name)
            
            # Choose color based on class
            if class_name == 'person':
                box_color = COLOR_PERSON
                label = f"PERSON ({confidence:.0%})"
            elif class_name == 'tent':
                box_color = COLOR_TENT
                label = f"TENT ({confidence:.0%})"
            else:
                # For 'object' class: just draw blue box, no labels
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), COLOR_OBJECT, 1)
                continue  # Skip label drawing for non-target objects
            
            # Draw bounding box with 1px for slimmer look
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), box_color, 1)
            
            # Calculate text size
            (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, font_thickness)
            
            # Position label above the box (or inside if no room above)
            padding = 4
            label_h = text_h + padding * 2
            
            if y1 - label_h >= 0:
                # Draw above the box
                label_y1 = y1 - label_h
                label_y2 = y1
            else:
                # Draw inside the box at top
                label_y1 = y1
                label_y2 = y1 + label_h
            
            # Draw filled background rectangle for label
            cv2.rectangle(
                annotated_frame,
                (x1, label_y1),
                (x1 + text_w + padding * 2, label_y2),
                box_color,
                -1  # Filled
            )
            
            # Draw text with black outline for better readability
            text_x = x1 + padding
            text_y = label_y2 - padding
            
            # Draw black outline (shadow effect for readability)
            cv2.putText(annotated_frame, label, (text_x, text_y),
                       font, font_scale, COLOR_BLACK, font_thickness + 1, cv2.LINE_AA)
            # Draw white text on top
            cv2.putText(annotated_frame, label, (text_x, text_y),
                       font, font_scale, COLOR_WHITE, font_thickness, cv2.LINE_AA)
        
        # Count targets
        num_people = sum(1 for d in detections if d['class'] == 'person')
        num_tents = sum(1 for d in detections if d['class'] == 'tent')
        num_other = len(detections) - num_people - num_tents
        
        # Add header with semi-transparent background
        header_font_scale = 0.6
        method_str = 'TensorRT' if self.model_format_detected == MODEL_FORMAT_TENSORRT else 'PyTorch'
        header_text = f"SAHI+YOLO ({method_str}) | {num_people} people, {num_tents} tents, {num_other} other"
        time_text = f"Time: {processing_time:.1f}s | Slices: {self.slice_height}x{self.slice_width}"
        
        # Draw semi-transparent header background
        overlay = annotated_frame.copy()
        cv2.rectangle(overlay, (0, 0), (width, 50), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, annotated_frame, 0.4, 0, annotated_frame)
        
        # Draw header text with anti-aliasing
        cv2.putText(annotated_frame, header_text, (10, 20),
                   font, header_font_scale, COLOR_WHITE, 1, cv2.LINE_AA)
        cv2.putText(annotated_frame, time_text, (10, 42),
                   font, header_font_scale * 0.8, (200, 200, 200), 1, cv2.LINE_AA)
        
        return annotated_frame
    
    def save_top_matches_crop(self, frame: np.ndarray, detections: List[Dict], image_path: str) -> Optional[str]:
        """
        Find the highest confidence person and tent, crop them, 
        combine side-by-side, and save as TM_<imagename>.
        
        Args:
            frame: Original image (not annotated)
            detections: List of detection dictionaries
            image_path: Path to the original image file
            
        Returns:
            Path to saved crop image, or None if no targets found
        """
        # Find highest confidence person
        persons = [d for d in detections if d['class'] == 'person']
        best_person = max(persons, key=lambda x: x['confidence']) if persons else None
        
        # Find highest confidence tent
        tents = [d for d in detections if d['class'] == 'tent']
        best_tent = max(tents, key=lambda x: x['confidence']) if tents else None
        
        if not best_person and not best_tent:
            return None  # No targets to crop
        
        crops = []
        labels = []
        height, width = frame.shape[:2]
        
        # Crop best person
        if best_person:
            x1, y1, x2, y2 = best_person['bbox']
            # Clamp to image bounds
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(width, x2), min(height, y2)
            if x2 > x1 and y2 > y1:
                person_crop = frame[y1:y2, x1:x2].copy()
                crops.append(person_crop)
                labels.append(f"PERSON {best_person['confidence']:.0%}")
        
        # Crop best tent
        if best_tent:
            x1, y1, x2, y2 = best_tent['bbox']
            # Clamp to image bounds
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(width, x2), min(height, y2)
            if x2 > x1 and y2 > y1:
                tent_crop = frame[y1:y2, x1:x2].copy()
                crops.append(tent_crop)
                labels.append(f"TENT {best_tent['confidence']:.0%}")
        
        if not crops:
            return None
        
        # Resize crops to same height for side-by-side display
        target_height = 200
        resized_crops = []
        
        for crop, label in zip(crops, labels):
            h, w = crop.shape[:2]
            if h > 0:
                scale = target_height / h
                new_w = int(w * scale)
                resized = cv2.resize(crop, (new_w, target_height), interpolation=cv2.INTER_AREA)
                
                # Add label at bottom
                font = cv2.FONT_HERSHEY_DUPLEX
                font_scale = 0.5
                (tw, th), _ = cv2.getTextSize(label, font, font_scale, 1)
                
                # Add padding at bottom for label
                label_height = th + 10
                padded = np.zeros((target_height + label_height, new_w, 3), dtype=np.uint8)
                padded[:target_height, :] = resized
                
                # Draw label background and text
                cv2.rectangle(padded, (0, target_height), (new_w, target_height + label_height), (40, 40, 40), -1)
                text_x = (new_w - tw) // 2
                cv2.putText(padded, label, (text_x, target_height + th + 3),
                           font, font_scale, (255, 255, 255), 1, cv2.LINE_AA)
                
                resized_crops.append(padded)
        
        if not resized_crops:
            return None
        
        # Combine crops horizontally with a separator
        separator_width = 5
        total_width = sum(c.shape[1] for c in resized_crops) + separator_width * (len(resized_crops) - 1)
        combined_height = resized_crops[0].shape[0]
        
        combined = np.zeros((combined_height, total_width, 3), dtype=np.uint8)
        x_offset = 0
        
        for i, crop in enumerate(resized_crops):
            if i > 0:
                # Draw white separator
                combined[:, x_offset:x_offset + separator_width] = (80, 80, 80)
                x_offset += separator_width
            combined[:, x_offset:x_offset + crop.shape[1]] = crop
            x_offset += crop.shape[1]
        
        # Save the combined crop
        original_filename = os.path.basename(image_path)
        name, ext = os.path.splitext(original_filename)
        output_filename = f"TM_{name}{ext}"
        output_path = os.path.join(self.detection_results_path, output_filename)
        
        cv2.imwrite(output_path, combined)
        # self.get_logger().info(f"  Saved top matches crop: {output_filename}")
        
        return output_path
    
    def publish_results(self, annotated_frame: np.ndarray, detections: List[Dict], image_path: str) -> None:
        """
        Publish annotated image and detection info, and save to disk
        
        Args:
            annotated_frame: Annotated image as numpy array
            detections: List of detection dictionaries
            image_path: Path to the original image file
        """
        try:
            # Check if publishers are available
            if self.publisher is None or self.detection_pub is None or self.detection_publisher is None:
                self.get_logger().warn("Publishers not available, skipping publish")
                return
            
            if self.bridge is None:
                self.get_logger().warn("CvBridge not available, skipping publish")
                return
            
            # Convert to ROS2 Image message
            image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera'
            
            # Publish image
            self.publisher.publish(image_msg)
            
            # Save annotated image to detection_results_sahi folder
            original_filename = os.path.basename(image_path)
            name, ext = os.path.splitext(original_filename)
            output_filename = f"sahi_detected_{name}{ext}"
            output_path = os.path.join(self.detection_results_path, output_filename)
            cv2.imwrite(output_path, annotated_frame)
            
            # Save top matches crop (best person + best tent)
            # Need to read original image for clean crops
            original_frame = cv2.imread(image_path)
            if original_frame is not None:
                self.save_top_matches_crop(original_frame, detections, image_path)
            
            # Create detection info message
            method = 'sahi+yolo11s+tensorrt' if self.model_format_detected == MODEL_FORMAT_TENSORRT else 'sahi+yolo11s'
            
            detection_info = {
                'image': os.path.basename(image_path),
                'timestamp': datetime.now().isoformat(),
                'detections': len(detections),
                'saved_to': output_path,
                'method': method,
                'slice_size': f"{self.slice_height}x{self.slice_width}",
                'overlap': f"{self.overlap_height_ratio}x{self.overlap_width_ratio}",
                'objects': [
                    {
                        'class': d['class'],
                        'yolo_class': d.get('yolo_class', d['class']),
                        'confidence': d['confidence'],
                        'bbox': d['bbox'],
                        'description': d.get('description', d['class']),
                        'area': d.get('area', 0)
                    }
                    for d in detections
                ]
            }

            # create ImageResult message
            image_result_msg = ImageResult()
            image_result_msg.header = image_msg.header
            image_result_msg.image_name = original_filename
            image_result_msg.timestamp = datetime.now().isoformat()
            image_result_msg.num_detections = len(detections)
            image_result_msg.saved_to = output_path
            image_result_msg.method = method
            image_result_msg.slice_size = f"{self.slice_height}x{self.slice_width}"
            image_result_msg.overlap = f"{self.overlap_height_ratio}x{self.overlap_width_ratio}"
            image_result_msg.waypoint_index = self.waypoint_reached             # include latest wp in message (ASSUMES INSTANT DETECTION)

            # Prepare Detection2DArray
            det_array = Detection2DArray()
            det_array.header = image_msg.header

            classes = []
            confidences = []
            areas = []
            descriptions = []
            masks = []  # Empty because you aren't generating segmentation masks

            for det in detections:
                # Create individual Detection2D
                d2d = Detection2D()
                d2d.header = image_msg.header

                # Create bounding box
                x1, y1, x2, y2 = det['bbox']
                # BoundingBox2D center is Pose2D with position attribute
                d2d.bbox.center.position.x = float(x1 + x2) / 2.0
                d2d.bbox.center.position.y = float(y1 + y2) / 2.0
                d2d.bbox.size_x = float(x2 - x1)
                d2d.bbox.size_y = float(y2 - y1)

                # Add hypothesis (class + confidence)
                hypo = ObjectHypothesisWithPose()
                # Map class names to integer IDs (0=person, 1=tent)
                class_id = 0 if det['class'] == 'person' else 1
                hypo.hypothesis.class_id = str(class_id)  # class_id is a string
                hypo.hypothesis.score = float(det['confidence'])
                d2d.results.append(hypo)

                det_array.detections.append(d2d)

                # Collect object summary data
                classes.append(det['class'])
                confidences.append(float(det['confidence']))
                areas.append(float(det.get('area', 0)))
                descriptions.append(det.get('description', det['class']))

            # Assign to ImageResult
            image_result_msg.detections = det_array
            image_result_msg.masks = masks
            image_result_msg.classes = classes
            image_result_msg.confidences = confidences
            image_result_msg.areas = areas
            image_result_msg.descriptions = descriptions


            # Publish ImageResult message
            self.detection_pub.publish(image_result_msg)
            
            # Publish detection info
            info_msg = String()
            info_msg.data = str(detection_info)
            self.detection_publisher.publish(info_msg)
            
            # self.get_logger().info(
            #     f" Published and saved results -> {output_filename}"
            # )
            
        except (cv2.error, OSError, IOError) as e:
            self.get_logger().error(f"Error publishing/saving results: {e}")
            self.stats['errors'] += 1
        except Exception as e:
            self.get_logger().error(f"Unexpected error publishing/saving results: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.stats['errors'] += 1
            
def main(args=None):
    rclpy.init(args=args)
    
    # Check dependencies
    if not SAHI_AVAILABLE or not YOLO_AVAILABLE:
        print("\n" + "="*80)
        print("ERROR: Missing required dependencies!")
        print("="*80)
        if not SAHI_AVAILABLE:
            print("SAHI not found. Install with:")
            print("  pip install sahi")
        if not YOLO_AVAILABLE:
            print("Ultralytics YOLO not found. Install with:")
            print("  pip install ultralytics")
        print("="*80 + "\n")
        return
    
    node = SAHIObjectDetectionNode()
    
    try:
        # Configure the node (transition from unconfigured to inactive)
        if node.trigger_configure() != TransitionCallbackReturn.SUCCESS:
            node.get_logger().error("Failed to configure node")
            node.destroy_node()
            rclpy.shutdown()
            return
        
        # Activate the node (transition from inactive to active)
        if node.trigger_activate() != TransitionCallbackReturn.SUCCESS:
            node.get_logger().error("Failed to activate node")
            node.trigger_cleanup()
            node.destroy_node()
            rclpy.shutdown()
            return
        
        # Use multithreaded executor for better responsiveness
        # Allows service callbacks to respond immediately even during image processing
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        
        node.get_logger().info("Node is active. Use 'ros2 lifecycle set /new_od deactivate' to pause or 'shutdown' to stop.")
        
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        # Graceful shutdown via lifecycle transitions
        if not node.shutdown_requested:
            node.get_logger().info("Initiating lifecycle shutdown...")
            try:
                # Deactivate first
                if node.trigger_deactivate() != TransitionCallbackReturn.SUCCESS:
                    node.get_logger().warn("Failed to deactivate node")
                
                # Cleanup
                if node.trigger_cleanup() != TransitionCallbackReturn.SUCCESS:
                    node.get_logger().warn("Failed to cleanup node")
                
                # Shutdown
                if node.trigger_shutdown() != TransitionCallbackReturn.SUCCESS:
                    node.get_logger().warn("Failed to shutdown node")
            except Exception as e:
                node.get_logger().error(f"Error during lifecycle shutdown: {e}")
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
