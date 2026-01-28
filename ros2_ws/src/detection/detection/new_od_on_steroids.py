#!/usr/bin/env python3

"""
SAHI Object Detection Node with TensorRT Support - STEROIDS VERSION
Enhanced version that:
1. Saves ALL detected boxes as individual cropped images
2. Stitches them together side by side
3. Enhances the stitched image using AI enhancement
"""

# ros2 imports
import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from interfaces.msg import ImageResult
from mavros_msgs.msg import WaypointReached
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

# non-ros2 imports
import cv2
from std_msgs.msg import String
import numpy as np
import time
from datetime import datetime
import os
from pathlib import Path
from typing import List, Dict, Optional, Tuple
import gc
import platform
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
import json
import threading
import queue
import statistics

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

#PyTorch
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

# AI Enhancement - Real-ESRGAN for super-resolution
try:
    from basicsr.archs.rrdbnet_arch import RRDBNet
    from realesrgan import RealESRGANer
    REALESRGAN_AVAILABLE = True
except ImportError:
    REALESRGAN_AVAILABLE = False

# Fallback: OpenCV's DNN Super-Resolution
try:
    from cv2 import dnn_superres
    AI_ENHANCEMENT_AVAILABLE = True
except (ImportError, AttributeError):
    AI_ENHANCEMENT_AVAILABLE = False

# Constants
DEFAULT_CONFIDENCE_THRESHOLD = 0.25
DEFAULT_SLICE_SIZE = 640
DEFAULT_OVERLAP = 0.25
DEFAULT_CHECK_INTERVAL = 2.0
MAX_SEARCH_DEPTH = 10

# Model format constants
MODEL_FORMAT_PYTORCH = 'pytorch'
MODEL_FORMAT_TENSORRT = 'tensorrt'
MODEL_FORMAT_AUTO = 'auto'

# Class ID mapping
CLASS_ID = {"person": "0", "tent": "1", "object": "2"}


class PerformanceMonitor:
    """Track and report performance metrics"""
    
    def __init__(self):
        self.detection_times = []
        self.slice_counts = []
        self.memory_snapshots = []
    
    def record_detection(self, time_val: float, num_slices: int, memory_used: float):
        self.detection_times.append(time_val)
        self.slice_counts.append(num_slices)
        self.memory_snapshots.append(memory_used)
        
        if len(self.detection_times) > 100:
            self.detection_times.pop(0)
            self.slice_counts.pop(0)
            self.memory_snapshots.pop(0)
    
    def get_stats(self) -> dict:
        if not self.detection_times:
            return {}
        
        return {
            'avg_time': statistics.mean(self.detection_times),
            'median_time': statistics.median(self.detection_times),
            'p95_time': statistics.quantiles(self.detection_times, n=20)[18] if len(self.detection_times) > 20 else max(self.detection_times),
            'avg_slices': statistics.mean(self.slice_counts),
            'avg_memory': statistics.mean(self.memory_snapshots),
        }


def get_ros2_ws_directory() -> str:
    """Find the ros2_ws root directory by searching up from current file location."""
    current_file = os.path.abspath(__file__)
    current_dir = os.path.dirname(current_file)
    
    search_dir = current_dir
    ros2_ws_dir = None
    
    for _ in range(MAX_SEARCH_DEPTH):
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
    
    if ros2_ws_dir is None:
        ros2_ws_dir = os.getenv('ROS2_WS_PATH') or os.path.expanduser('~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws')
    
    return ros2_ws_dir


class ImageEnhancer:
    """AI-based image enhancement using Real-ESRGAN or OpenCV Super Resolution"""
    
    def __init__(self, logger):
        self.logger = logger
        self.sr_model = None
        self.upsampler = None  # Real-ESRGAN upsampler
        self.enhancement_method = 'fallback'  # 'realesrgan', 'opencv', or 'fallback'
        self.initialized = False
        
    def initialize(self):
        """Initialize AI enhancement (Real-ESRGAN preferred, with fallbacks)"""
        
        # Try Real-ESRGAN first (best quality)
        if REALESRGAN_AVAILABLE:
            try:
                model_path = self._find_realesrgan_model()
                if model_path:
                    from basicsr.archs.rrdbnet_arch import RRDBNet
                    from realesrgan import RealESRGANer
                    
                    # Initialize Real-ESRGAN with x2 upscaling
                    model = RRDBNet(num_in_ch=3, num_out_ch=3, num_feat=64, num_block=23, num_grow_ch=32, scale=2)
                    self.upsampler = RealESRGANer(
                        scale=2,
                        model_path=model_path,
                        model=model,
                        tile=256,  # Use tiling for large images
                        tile_pad=10,
                        pre_pad=0,
                        half=False  # Use FP32 for CPU
                    )
                    self.enhancement_method = 'realesrgan'
                    self.initialized = True
                    self.logger.info("✓ Real-ESRGAN x2 initialized (best quality)")
                    return True
            except Exception as e:
                self.logger.warn(f"Failed to initialize Real-ESRGAN: {e}")
        
        # Try OpenCV DNN Super-Resolution
        if AI_ENHANCEMENT_AVAILABLE:
            try:
                self.sr_model = cv2.dnn_superres.DnnSuperResImpl_create()
                model_path = self._find_sr_model()
                
                if model_path and os.path.exists(model_path):
                    self.sr_model.readModel(model_path)
                    self.sr_model.setModel("espcn", 2)  # 2x upscaling
                    self.enhancement_method = 'opencv'
                    self.initialized = True
                    self.logger.info("✓ OpenCV ESPCN x2 initialized")
                    return True
            except Exception as e:
                self.logger.warn(f"Failed to initialize OpenCV SR: {e}")
        
        # Fallback to traditional methods
        self.logger.info("Using fallback enhancement (denoise + CLAHE + sharpen)")
        self.enhancement_method = 'fallback'
        return False
    
    def _find_realesrgan_model(self) -> Optional[str]:
        """Try to find Real-ESRGAN model file"""
        possible_paths = [
            "/usr/local/share/realesrgan/RealESRGAN_x2plus.pth",
            "/usr/share/realesrgan/RealESRGAN_x2plus.pth",
            os.path.expanduser("~/models/RealESRGAN_x2plus.pth"),
            "./RealESRGAN_x2plus.pth",
            os.path.expanduser("~/.cache/realesrgan/RealESRGAN_x2plus.pth"),
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                return path
        return None
    
    def _find_sr_model(self) -> Optional[str]:
        """Try to find OpenCV super resolution model file"""
        possible_paths = [
            "/usr/local/share/opencv4/dnn_superres/ESPCN_x2.pb",
            "/usr/share/opencv4/dnn_superres/ESPCN_x2.pb",
            os.path.expanduser("~/models/ESPCN_x2.pb"),
            "./ESPCN_x2.pb",
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                return path
        return None
    
    def enhance(self, image: np.ndarray) -> np.ndarray:
        """
        Enhance image using Real-ESRGAN x2 (preferred) or fallback methods
        
        Enhancement strategy:
        - Best: Real-ESRGAN x2 (recovers detail, reduces pixelation)
        - Good: OpenCV ESPCN x2
        - Fallback: Denoise + CLAHE + Sharpen
        
        Args:
            image: Input image as numpy array
            
        Returns:
            Enhanced image (2x resolution if using SR, same size if fallback)
        """
        if image is None or image.size == 0:
            return image
        
        try:
            # Real-ESRGAN x2 (best quality, but slower)
            if self.enhancement_method == 'realesrgan' and self.upsampler is not None:
                try:
                    output, _ = self.upsampler.enhance(image, outscale=2)
                    return output
                except Exception as e:
                    self.logger.warn(f"Real-ESRGAN failed: {e}, using fallback")
                    return self._enhance_fallback(image)
            
            # OpenCV Super-Resolution x2
            elif self.enhancement_method == 'opencv' and self.sr_model is not None:
                try:
                    return self.sr_model.upsample(image)
                except Exception as e:
                    self.logger.warn(f"OpenCV SR failed: {e}, using fallback")
                    return self._enhance_fallback(image)
            
            # Fallback: traditional methods (no upscaling)
            else:
                return self._enhance_fallback(image)
                
        except Exception as e:
            self.logger.debug(f"Enhancement error: {e}, using original")
            return image
    
    def _enhance_fallback(self, image: np.ndarray) -> np.ndarray:
        """
        Fallback enhancement using traditional CV methods
        
        Enhancement Pipeline:
        1. Bilateral Filter - Reduces noise while preserving edges
        2. Gentle CLAHE - Enhances local contrast (clipLimit=1.5 to avoid noise amplification)
        3. Subtle Sharpening - Mild unsharp mask (strength=1.2 instead of 1.5)
        4. Gamma Correction - Brightens slightly for better visibility
        """
        try:
            # Step 1: Denoise first with bilateral filter (preserves edges better than Gaussian)
            denoised = cv2.bilateralFilter(image, 9, 75, 75)
            
            # Step 2: Gentle CLAHE for contrast enhancement (reduced clipLimit to avoid noise)
            lab = cv2.cvtColor(denoised, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            
            # Gentler CLAHE settings to reduce noise amplification
            clahe = cv2.createCLAHE(clipLimit=1.5, tileGridSize=(8, 8))
            l = clahe.apply(l)
            
            enhanced = cv2.merge([l, a, b])
            enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
            
            # Step 3: Subtle sharpening (reduced strength to minimize noise)
            gaussian = cv2.GaussianBlur(enhanced, (0, 0), 1.0)
            enhanced = cv2.addWeighted(enhanced, 1.2, gaussian, -0.2, 0)
            
            # Step 4: Gamma correction for slight brightening
            gamma = 1.1
            inv_gamma = 1.0 / gamma
            table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
            enhanced = cv2.LUT(enhanced, table)
            
            return enhanced
            
        except Exception as e:
            self.logger.debug(f"Fallback enhancement error: {e}")
            return image


class SAHIObjectDetectionNodeSteroids(LifecycleNode):
    """Enhanced SAHI Object Detection Node with image stitching and AI enhancement"""

    def __init__(self):
        super().__init__('steroids_od')
        
        # Lifecycle state flags
        self.shutdown_requested = False
        self._active = False
        
        # Declare all parameters from original node
        self.declare_parameter('model_path', 'yolo26x.pt')
        self.declare_parameter('model_format', MODEL_FORMAT_AUTO)
        self.declare_parameter('auto_convert_tensorrt', True)
        self.declare_parameter('tensorrt_workspace', 4)
        self.declare_parameter('confidence_threshold', DEFAULT_CONFIDENCE_THRESHOLD)
        self.declare_parameter('slice_height', DEFAULT_SLICE_SIZE)
        self.declare_parameter('slice_width', DEFAULT_SLICE_SIZE)
        self.declare_parameter('overlap_height_ratio', DEFAULT_OVERLAP)
        self.declare_parameter('overlap_width_ratio', DEFAULT_OVERLAP)
        self.declare_parameter('check_interval', DEFAULT_CHECK_INTERVAL)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('max_images_per_cycle', 5)
        self.declare_parameter('max_camera_feed_images', 100)
        self.declare_parameter('min_detection_area', 25)
        self.declare_parameter('max_detection_area', 1000000)
        self.declare_parameter('min_aspect_ratio', 0.1)
        self.declare_parameter('max_aspect_ratio', 10.0)
        self.declare_parameter('enable_gpu_memory_cleanup', True)
        self.declare_parameter('camera_feed_path', '')
        self.declare_parameter('detection_results_path', '')
        self.declare_parameter('log_level', 'INFO')
        self.declare_parameter('detection_watchdog_timeout', 60.0)
        
        # NEW STEROIDS PARAMETERS
        self.declare_parameter('crop_output_path', '')  # Where to save individual crops
        self.declare_parameter('stitch_output_path', '')  # Where to save stitched images
        self.declare_parameter('enhanced_output_path', '')  # Where to save enhanced images
        self.declare_parameter('enable_ai_enhancement', True)  # Enable/disable AI enhancement
        self.declare_parameter('crop_padding', 10)  # Padding around crops
        self.declare_parameter('stitch_spacing', 5)  # Space between stitched images
        
        # Initialize variables
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
        
        # Worker thread pattern
        self.work_q = queue.Queue(maxsize=50)
        self.worker_thread = None
        self.worker_stop = threading.Event()
        
        # Processing state
        self.processed_images: Dict[str, float] = {}
        self.is_processing = False
        
        # Performance monitoring
        self.perf_monitor = PerformanceMonitor()
        
        # Statistics
        self.stats = {
            'total_images_processed': 0,
            'total_detections': 0,
            'total_tents': 0,
            'total_people': 0,
            'total_objects': 0,
            'avg_processing_time': 0.0,
            'last_processing_time': 0.0,
            'node_start_time': time.time(),
            'errors': 0,
            'model_reloads': 0,
            'total_crops_saved': 0,  # NEW
            'total_stitched_images': 0,  # NEW
            'total_enhanced_images': 0,  # NEW
        }
        
        # Health monitoring
        self.health_status = {
            'is_healthy': True,
            'last_successful_detection': None,
            'consecutive_errors': 0
        }
        
        # Waypoint tracking
        self.waypoint_reached = 0
        
        # Directory paths
        self.camera_feed_path = None
        self.detection_results_path = None
        self.crop_output_path = None
        self.stitch_output_path = None
        self.enhanced_output_path = None
        self.device = None
        self.model_format_detected = None
        
        # Image enhancer
        self.enhancer = ImageEnhancer(self.get_logger())
    
    # Copy essential helper methods from original node
    def _get_device(self) -> str:
        """Auto-detect best available device"""
        if TORCH_AVAILABLE:
            if torch.cuda.is_available():
                return 'cuda:0'
            elif hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
                return 'mps'
        return 'cpu'
    
    def _optimize_gpu_memory(self):
        """Optimize GPU memory settings"""
        if TORCH_AVAILABLE and torch.cuda.is_available():
            try:
                torch.cuda.empty_cache()
                gc.collect()
                torch.cuda.set_per_process_memory_fraction(0.8, 0)
                torch.backends.cudnn.benchmark = True
                torch.backends.cudnn.enabled = True
            except Exception as e:
                self.get_logger().debug(f"GPU optimization error: {e}")
    
    def _validate_sahi_config(self) -> List[str]:
        """Validate SAHI configuration and return warnings"""
        warnings = []
        
        if self.slice_height < 320 or self.slice_width < 320:
            warnings.append(f"Small slice size ({self.slice_height}x{self.slice_width}) may reduce detection quality")
        
        if self.overlap_height_ratio < 0.1 or self.overlap_width_ratio < 0.1:
            warnings.append("Low overlap ratio may miss objects at slice boundaries")
        
        return warnings
    
    def _warmup_model(self):
        """Warm up model with dummy inference"""
        try:
            self.get_logger().info("Warming up model...")
            dummy_image = np.zeros((self.slice_height, self.slice_width, 3), dtype=np.uint8)
            _ = self.detect_objects_sahi(dummy_image)
            self.get_logger().info("✓ Model warmup complete")
        except Exception as e:
            self.get_logger().warn(f"Model warmup failed: {e}")
    
    def _is_file_ready(self, filepath: str, stable_time: float = 0.5) -> bool:
        """Check if file is fully written and stable"""
        try:
            initial_size = os.path.getsize(filepath)
            time.sleep(stable_time)
            current_size = os.path.getsize(filepath)
            return initial_size == current_size
        except (OSError, FileNotFoundError):
            return False
    
    def _prune_processed(self):
        """Prune old entries from processed_images dict"""
        if len(self.processed_images) > 1000:
            cutoff_time = time.time() - 3600  # Keep last hour
            self.processed_images = {
                k: v for k, v in self.processed_images.items()
                if v > cutoff_time
            }
    
    def _periodic_gpu_cleanup(self):
        """Periodic GPU memory cleanup"""
        if TORCH_AVAILABLE and self.device.startswith('cuda'):
            try:
                gc.collect()
                torch.cuda.empty_cache()
            except Exception as e:
                self.get_logger().debug(f"GPU cleanup error: {e}")
    
    def _worker_loop(self):
        """Worker thread for processing images"""
        self.get_logger().info("Worker thread started")
        
        while not self.worker_stop.is_set():
            try:
                # Get image from queue with timeout
                image_path = self.work_q.get(timeout=1.0)
                
                if image_path is None:
                    continue
                
                # Process the image
                self.process_image(image_path)
                
                self.work_q.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Worker thread error: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
        
        self.get_logger().info("Worker thread stopped")
    
    def waypoint_reached_cb(self, msg: WaypointReached):
        """Callback for waypoint reached messages"""
        self.waypoint_reached = msg.wp_seq
        self.get_logger().info(f"Waypoint {self.waypoint_reached} reached")
    
    def _parameter_callback(self, params):
        """Handle parameter changes"""
        for param in params:
            if param.name == 'confidence_threshold':
                self.confidence_threshold = param.value
                if self.detection_model:
                    self.detection_model.confidence_threshold = param.value
        return rclpy.parameter.SetParametersResult(successful=True)
    
    def _get_statistics_service(self, request, response):
        """Service to get node statistics"""
        stats_copy = self.stats.copy()
        stats_copy['uptime'] = time.time() - stats_copy['node_start_time']
        perf_stats = self.perf_monitor.get_stats()
        stats_copy.update(perf_stats)
        
        response.success = True
        response.message = json.dumps(stats_copy, indent=2)
        return response
    
    def _get_health_service(self, request, response):
        """Service to get node health status"""
        response.success = self.health_status['is_healthy']
        response.message = json.dumps(self.health_status, indent=2, default=str)
        return response
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the node"""
        self.get_logger().info("Configuring SAHI Object Detection Node (STEROIDS VERSION)...")
        
        try:
            self._load_and_validate_parameters()
            
            # Setup directories
            ros2_ws_dir = get_ros2_ws_directory()
            
            cam = self.get_parameter('camera_feed_path').value
            out = self.get_parameter('detection_results_path').value
            crop = self.get_parameter('crop_output_path').value
            stitch = self.get_parameter('stitch_output_path').value
            enhanced = self.get_parameter('enhanced_output_path').value
            
            # Set camera feed path
            if not cam:
                self.camera_feed_path = os.path.join(ros2_ws_dir, "src", "video_cam", "mapping_photos")
            else:
                self.camera_feed_path = cam
            
            # Set detection results path
            if not out:
                self.detection_results_path = os.path.join(ros2_ws_dir, "src", "detection", "detection_results_steroids")
            else:
                self.detection_results_path = out
            
            # Set crop output path (NEW)
            if not crop:
                self.crop_output_path = os.path.join(self.detection_results_path, "crops")
            else:
                self.crop_output_path = crop
            
            # Set stitch output path (NEW)
            if not stitch:
                self.stitch_output_path = os.path.join(self.detection_results_path, "stitched")
            else:
                self.stitch_output_path = stitch
            
            # Set enhanced output path (NEW)
            if not enhanced:
                self.enhanced_output_path = os.path.join(self.detection_results_path, "enhanced")
            else:
                self.enhanced_output_path = enhanced
            
            # Create all directories
            os.makedirs(self.detection_results_path, exist_ok=True)
            os.makedirs(self.crop_output_path, exist_ok=True)
            os.makedirs(self.stitch_output_path, exist_ok=True)
            os.makedirs(self.enhanced_output_path, exist_ok=True)
            
            if not os.path.exists(self.camera_feed_path):
                self.get_logger().warn(f"Camera feed directory does not exist: {self.camera_feed_path}")
                os.makedirs(self.camera_feed_path, exist_ok=True)
            
            self.get_logger().info(f"Detection results: {self.detection_results_path}")
            self.get_logger().info(f"Crop output: {self.crop_output_path}")
            self.get_logger().info(f"Stitch output: {self.stitch_output_path}")
            self.get_logger().info(f"Enhanced output: {self.enhanced_output_path}")
            
            # OpenCV optimizations
            cv2.setNumThreads(0)
            cv2.ocl.setUseOpenCL(False)
            
            # Auto-detect device
            if self.device == 'auto':
                self.device = self._get_device()
            self.get_logger().info(f"Using device: {self.device}")
            
            # Optimize GPU memory
            if self.device.startswith('cuda'):
                self._optimize_gpu_memory()
            
            # Model format detection
            self.model_format_detected = self._detect_model_format()
            
            # Validate SAHI configuration
            warnings = self._validate_sahi_config()
            for warning in warnings:
                self.get_logger().warn(warning)
            
            # Initialize image enhancer
            if self.enable_ai_enhancement:
                self.enhancer.initialize()
            
            # Setup ROS2 bridge
            self.bridge = CvBridge()
            
            # Parameter callback
            self.add_on_set_parameters_callback(self._parameter_callback)
            
            self.get_logger().info("✓ Configuration complete")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate the node"""
        self.get_logger().info("Activating SAHI Object Detection Node (STEROIDS)...")
        
        try:
            self._active = True
            
            # Initialize SAHI model
            if not self.initialize_sahi_model():
                self.get_logger().error("Failed to initialize SAHI model")
                return TransitionCallbackReturn.FAILURE
            
            # Warmup model
            self._warmup_model()
            
            # QoS profile
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
            
            # Start worker thread
            self.worker_stop.clear()
            self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
            self.worker_thread.start()
            
            # Create timer
            self.timer = self.create_timer(self.check_interval, self.check_for_new_images)
            
            # GPU cleanup timer
            if self.enable_gpu_memory_cleanup and self.device.startswith('cuda'):
                self.gpu_cleanup_timer = self.create_timer(30.0, self._periodic_gpu_cleanup)
            
            # Reset stats
            self.stats['node_start_time'] = time.time()
            
            self.get_logger().info("✓ SAHI Object Detection Node (STEROIDS) activated and ready!")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Activation failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.FAILURE
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the node"""
        self.get_logger().info("Deactivating SAHI Object Detection Node (STEROIDS)...")
        
        self._active = False
        
        # Stop worker thread
        self.worker_stop.set()
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=5.0)
        
        # Stop timers
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        
        if self.gpu_cleanup_timer is not None:
            self.gpu_cleanup_timer.cancel()
            self.gpu_cleanup_timer = None
        
        self.is_processing = False
        
        self.get_logger().info("Node deactivated")
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Cleanup the node"""
        self.get_logger().info("Cleaning up SAHI Object Detection Node (STEROIDS)...")
        
        # Stop worker thread
        self.worker_stop.set()
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=5.0)
        
        # Destroy timers
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
        
        if self.gpu_cleanup_timer is not None:
            self.destroy_timer(self.gpu_cleanup_timer)
            self.gpu_cleanup_timer = None
        
        # Destroy publishers
        if self.publisher is not None:
            self.destroy_publisher(self.publisher)
            self.publisher = None
        
        if self.detection_publisher is not None:
            self.destroy_publisher(self.detection_publisher)
            self.detection_publisher = None
        
        if self.detection_pub is not None:
            self.destroy_publisher(self.detection_pub)
            self.detection_pub = None
        
        # Destroy services
        if self.stats_service is not None:
            self.destroy_service(self.stats_service)
            self.stats_service = None
        
        if self.health_service is not None:
            self.destroy_service(self.health_service)
            self.health_service = None
        
        # Destroy subscriber
        if self.waypoint_subscription is not None:
            self.destroy_subscription(self.waypoint_subscription)
            self.waypoint_subscription = None
        
        # Clear model
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
        """Shutdown the node"""
        self.get_logger().info("Lifecycle shutdown")
        self.shutdown_requested = True
        
        # Final cleanup
        try:
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
            
            if self.gpu_cleanup_timer is not None:
                self.gpu_cleanup_timer.cancel()
                self.gpu_cleanup_timer = None
            
            if self.detection_model is not None:
                del self.detection_model
                self.detection_model = None
            
            if self.device and self.device.startswith('cuda') and TORCH_AVAILABLE:
                gc.collect()
                torch.cuda.empty_cache()
        except Exception as e:
            self.get_logger().error(f"Shutdown cleanup error: {e}")
        
        return TransitionCallbackReturn.SUCCESS
    
    def _load_and_validate_parameters(self) -> None:
        """Load and validate all parameters"""
        # Original parameters
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
        
        # NEW STEROIDS parameters
        self.enable_ai_enhancement = self.get_parameter('enable_ai_enhancement').value
        self.crop_padding = self.get_parameter('crop_padding').value
        self.stitch_spacing = self.get_parameter('stitch_spacing').value
        
        # Validate model_format
        if self.model_format not in [MODEL_FORMAT_PYTORCH, MODEL_FORMAT_TENSORRT, MODEL_FORMAT_AUTO]:
            self.get_logger().warn(f"Invalid model_format '{self.model_format}', using 'auto'")
            self.model_format = MODEL_FORMAT_AUTO
        
        # Validate parameters
        if not 0 < self.confidence_threshold <= 1.0:
            self.get_logger().warn(f"Invalid confidence_threshold: {self.confidence_threshold}, using default")
            self.confidence_threshold = DEFAULT_CONFIDENCE_THRESHOLD
        
        if self.slice_height < 64 or self.slice_width < 64:
            self.get_logger().warn(f"Slice size too small, adjusting to minimum 64x64")
            self.slice_height = max(64, self.slice_height)
            self.slice_width = max(64, self.slice_width)
        
        if self.check_interval < 0.1:
            self.get_logger().warn(f"Check interval too small, using default")
            self.check_interval = DEFAULT_CHECK_INTERVAL
        
        if self.max_images_per_cycle < 1:
            self.get_logger().warn(f"max_images_per_cycle must be >= 1")
            self.max_images_per_cycle = 1
    
    def _detect_model_format(self) -> str:
        """Detect model format from file extension"""
        if self.model_path.endswith('.engine'):
            return MODEL_FORMAT_TENSORRT
        elif self.model_path.endswith('.pt'):
            return MODEL_FORMAT_PYTORCH
        else:
            self.get_logger().warn(f"Unknown model format for {self.model_path}. Assuming PyTorch.")
            return MODEL_FORMAT_PYTORCH
    
    def _convert_pytorch_to_tensorrt(self, pt_path: str, engine_path: str) -> bool:
        """Convert PyTorch model to TensorRT engine"""
        if not YOLO_AVAILABLE or not TENSORRT_AVAILABLE:
            return False
        
        try:
            self.get_logger().info(f"Converting {pt_path} to TensorRT...")
            
            model = YOLO(pt_path)
            model.export(
                format='engine',
                device=0 if self.device.startswith('cuda') else 'cpu',
                imgsz=(self.slice_height, self.slice_width),
                half=True,
                workspace=self.tensorrt_workspace,
                simplify=True,
                verbose=False
            )
            
            base_name = os.path.splitext(pt_path)[0]
            exported_engine = f"{base_name}.engine"
            
            if os.path.exists(exported_engine):
                if exported_engine != engine_path:
                    import shutil
                    shutil.move(exported_engine, engine_path)
                self.get_logger().info(f"✓ TensorRT conversion successful")
                return True
            else:
                self.get_logger().error(f"TensorRT engine file not found")
                return False
                
        except Exception as e:
            self.get_logger().error(f"TensorRT conversion failed: {str(e)}")
            return False
    
    def _resolve_model_path(self) -> Tuple[Optional[str], str]:
        """Resolve model path and determine final format"""
        model_path = self.model_path
        ros2_ws_dir = get_ros2_ws_directory()
        
        # Resolve relative paths
        if not os.path.isabs(model_path):
            ros2_ws_model_path = os.path.join(ros2_ws_dir, model_path)
            if os.path.exists(ros2_ws_model_path):
                model_path = ros2_ws_model_path
            elif os.path.exists(os.path.join(os.path.dirname(__file__), model_path)):
                model_path = os.path.join(os.path.dirname(__file__), model_path)
            elif not os.path.exists(model_path):
                self.get_logger().warn(
                    f"Model file not found at {self.model_path}, "
                    f"will download from Ultralytics if needed"
                )
        
        # Determine format
        if self.model_format == MODEL_FORMAT_AUTO:
            base_name = os.path.splitext(model_path)[0]
            engine_path = f"{base_name}.engine"
            
            if os.path.exists(engine_path):
                self.get_logger().info(f"Found TensorRT engine: {engine_path}")
                return engine_path, MODEL_FORMAT_TENSORRT
            elif model_path.endswith('.pt') and self.auto_convert_tensorrt and TENSORRT_AVAILABLE:
                if self._convert_pytorch_to_tensorrt(model_path, engine_path):
                    return engine_path, MODEL_FORMAT_TENSORRT
                else:
                    self.get_logger().warn("TensorRT conversion failed, using PyTorch")
                    return model_path, MODEL_FORMAT_PYTORCH
            else:
                return model_path, self.model_format_detected
        elif self.model_format == MODEL_FORMAT_TENSORRT:
            if model_path.endswith('.engine'):
                return model_path, MODEL_FORMAT_TENSORRT
            else:
                base_name = os.path.splitext(model_path)[0]
                engine_path = f"{base_name}.engine"
                if os.path.exists(engine_path):
                    return engine_path, MODEL_FORMAT_TENSORRT
                elif self.auto_convert_tensorrt:
                    if self._convert_pytorch_to_tensorrt(model_path, engine_path):
                        return engine_path, MODEL_FORMAT_TENSORRT
                return None, MODEL_FORMAT_TENSORRT
        else:
            return model_path, MODEL_FORMAT_PYTORCH
    
    def initialize_sahi_model(self):
        """Initialize SAHI detection model"""
        try:
            if not SAHI_AVAILABLE:
                self.get_logger().error("SAHI not available. Install: pip install sahi")
                return False
            
            if not YOLO_AVAILABLE:
                self.get_logger().error("Ultralytics YOLO not available. Install: pip install ultralytics")
                return False
            
            # Resolve model path
            resolved_path, final_format = self._resolve_model_path()
            if resolved_path is None:
                self.get_logger().error("Failed to resolve model path")
                return False
            
            self.get_logger().info(f"Loading model: {resolved_path} (format: {final_format})")
            
            # GPU memory optimization
            if self.device.startswith('cuda') and TORCH_AVAILABLE:
                try:
                    import gc
                    gc.collect()
                    torch.cuda.empty_cache()
                    torch.cuda.synchronize()
                    torch.cuda.set_per_process_memory_fraction(0.8, 0)
                    torch.backends.cudnn.benchmark = True
                    torch.backends.cudnn.enabled = True
                except Exception as e:
                    self.get_logger().debug(f"GPU optimization error: {e}")
            
            # Initialize SAHI model
            if final_format == MODEL_FORMAT_TENSORRT:
                try:
                    base_model = YOLO(resolved_path)
                    from sahi.models.yolov8 import Yolov8DetectionModel
                    self.detection_model = Yolov8DetectionModel(
                        model=base_model,
                        confidence_threshold=self.confidence_threshold,
                        device=self.device,
                    )
                    self.get_logger().info("✓ TensorRT model loaded")
                except Exception as e:
                    self.get_logger().error(f"Failed to load TensorRT model: {e}")
                    # Fallback to PyTorch
                    base_name = os.path.splitext(resolved_path)[0]
                    pt_path = f"{base_name}.pt"
                    if os.path.exists(pt_path):
                        self.get_logger().warn(f"Using PyTorch fallback: {pt_path}")
                        self.detection_model = AutoDetectionModel.from_pretrained(
                            model_type='yolov8',
                            model_path=pt_path,
                            confidence_threshold=self.confidence_threshold,
                            device=self.device,
                        )
                        self.model_format_detected = MODEL_FORMAT_PYTORCH
                    else:
                        return False
            else:
                self.detection_model = AutoDetectionModel.from_pretrained(
                    model_type='yolov8',
                    model_path=resolved_path,
                    confidence_threshold=self.confidence_threshold,
                    device=self.device,
                )
                self.get_logger().info("✓ PyTorch model loaded")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize SAHI model: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.stats['errors'] += 1
            self.health_status['consecutive_errors'] += 1
            self.health_status['is_healthy'] = False
            return False
    
    def check_for_new_images(self) -> None:
        """Check for new images and enqueue for processing"""
        try:
            if not self._active or self.shutdown_requested:
                return
            
            if not os.path.exists(self.camera_feed_path):
                return
            
            if self.detection_model is None:
                return
            
            # Clean up old images
            self._cleanup_old_images()
            self._prune_processed()
            
            # Get image files
            image_files_with_time: List[Tuple[str, float]] = []
            for file in os.listdir(self.camera_feed_path):
                if file.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')):
                    file_path = os.path.join(self.camera_feed_path, file)
                    try:
                        mtime = os.path.getmtime(file_path)
                        image_files_with_time.append((file, mtime))
                    except OSError:
                        continue
            
            # Sort by timestamp
            image_files_with_time.sort(key=lambda x: x[1])
            
            # Enqueue new images
            enqueued = 0
            for fname, mtime in image_files_with_time:
                if enqueued >= self.max_images_per_cycle:
                    break
                if fname in self.processed_images:
                    continue
                
                image_path = os.path.join(self.camera_feed_path, fname)
                
                if not self._is_file_ready(image_path):
                    continue
                
                try:
                    self.work_q.put_nowait(image_path)
                    self.processed_images[fname] = time.time()
                    enqueued += 1
                    self.get_logger().info(f"✓ Enqueued NEW image: {fname}")
                except queue.Full:
                    break
            
        except Exception as e:
            self.get_logger().error(f"Error checking for new images: {e}")
            self.stats['errors'] += 1
    
    def _cleanup_old_images(self) -> None:
        """Remove old images from camera_feed if limit exceeded"""
        if self.max_camera_feed_images <= 0:
            return
        
        try:
            image_files_with_time: List[Tuple[str, float]] = []
            for file in os.listdir(self.camera_feed_path):
                if file.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')):
                    file_path = os.path.join(self.camera_feed_path, file)
                    try:
                        mtime = os.path.getmtime(file_path)
                        image_files_with_time.append((file, mtime))
                    except OSError:
                        continue
            
            image_files_with_time.sort(key=lambda x: x[1])
            
            if len(image_files_with_time) > self.max_camera_feed_images:
                to_remove = len(image_files_with_time) - self.max_camera_feed_images
                removed = 0
                for file, _ in image_files_with_time[:to_remove]:
                    file_path = os.path.join(self.camera_feed_path, file)
                    try:
                        os.remove(file_path)
                        self.processed_images.pop(file, None)
                        removed += 1
                    except OSError as e:
                        self.get_logger().debug(f"Could not remove {file}: {e}")
                
                if removed > 0:
                    self.get_logger().info(f"Cleaned up {removed} old images")
        except Exception as e:
            self.get_logger().debug(f"Error during image cleanup: {e}")
    
    def process_image(self, image_path: str) -> None:
        """
        Process image with STEROIDS features:
        1. Detect objects
        2. Save individual crops
        3. Stitch crops together
        4. Enhance stitched image with AI
        """
        try:
            start_time = time.time()
            
            # Load image
            frame = cv2.imread(image_path)
            if frame is None:
                self.get_logger().warn(f"Could not load image: {image_path}")
                return
            
            height, width = frame.shape[:2]
            self.get_logger().info(f"Processing: {os.path.basename(image_path)} ({width}x{height})")
            
            # Keep original for crops
            frame_orig = frame.copy()
            
            # Run detection
            detections = self.detect_objects_sahi(frame)
            
            processing_time = time.time() - start_time
            self.stats['last_processing_time'] = processing_time
            
            # STEROIDS PIPELINE START
            if detections:
                # 1. Save individual crops with metadata
                crop_paths, crop_metadata = self.save_individual_crops(frame_orig, detections, image_path)
                
                # 2. Stitch crops together with placement tracking
                stitched_image, stitched_path, placements = self.stitch_crops(crop_paths, crop_metadata, image_path)
                
                # 3. Re-run detection on stitched image
                if stitched_image is not None:
                    stitched_detections = self.detect_objects_sahi(stitched_image)
                    self.get_logger().info(f"Stitched detections: {len(stitched_detections)} found")
                    
                    # Add labels with stitched detections
                    if stitched_path:
                        stitched_labeled = self.add_detection_labels(stitched_image, stitched_detections, "STITCHED")
                        cv2.imwrite(stitched_path, stitched_labeled)
                
                # 4. AI enhance the stitched image
                if stitched_image is not None and self.enable_ai_enhancement:
                    enhanced_image = self.enhancer.enhance(stitched_image)
                    
                    # 5. Re-run detection on enhanced image
                    enhanced_detections = self.detect_objects_sahi(enhanced_image)
                    self.get_logger().info(f"Enhanced detections: {len(enhanced_detections)} found")
                    
                    # Add labels with enhanced detections
                    enhanced_labeled = self.add_detection_labels(enhanced_image, enhanced_detections, "ENHANCED")
                    enhanced_path = self.save_enhanced_image(enhanced_labeled, image_path)
                    
                    if enhanced_path:
                        self.stats['total_enhanced_images'] += 1
                        self.get_logger().info(f"✓ Enhanced: {os.path.basename(enhanced_path)}")
                    
                    # TODO: Map enhanced_detections back to original image using placements metadata
                    # This would involve: for each detection in enhanced_detections,
                    # determine which crop it came from using placements[i]['x_start/x_end'],
                    # reverse the scale transformation, and map back to original coords
            
            # Create annotated frame
            annotated_frame = self.annotate_frame(frame, detections, processing_time)
            
            # Publish results
            self.publish_results(annotated_frame, detections, image_path)
            
            # Update statistics
            self.stats['total_images_processed'] += 1
            self.stats['total_detections'] += len(detections)
            self.stats['total_tents'] += sum(1 for d in detections if d['class'] == 'tent')
            self.stats['total_people'] += sum(1 for d in detections if d['class'] == 'person')
            self.stats['total_objects'] += sum(1 for d in detections if d['class'] == 'object')
            
            # Update health
            self.health_status['last_successful_detection'] = datetime.now()
            self.health_status['consecutive_errors'] = 0
            self.health_status['is_healthy'] = True
            
            self.get_logger().info(
                f"✓ Complete: {len(detections)} detections in {processing_time:.2f}s"
            )
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.stats['errors'] += 1
            self.health_status['consecutive_errors'] += 1
    
    def detect_objects_sahi(self, frame: np.ndarray) -> List[Dict]:
        """Run SAHI detection on image"""
        try:
            if self.detection_model is None:
                return []
            
            # Run SAHI sliced prediction
            result = get_sliced_prediction(
                frame,
                self.detection_model,
                slice_height=self.slice_height,
                slice_width=self.slice_width,
                overlap_height_ratio=self.overlap_height_ratio,
                overlap_width_ratio=self.overlap_width_ratio,
                verbose=0
            )
            
            # Convert to our detection format
            detections = []
            for obj in result.object_prediction_list:
                bbox = obj.bbox.to_xyxy()
                x1, y1, x2, y2 = map(int, bbox)
                
                area = (x2 - x1) * (y2 - y1)
                
                # Filter by area
                if area < self.min_detection_area or area > self.max_detection_area:
                    continue
                
                # Filter by aspect ratio
                aspect_ratio = (x2 - x1) / max((y2 - y1), 1)
                if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
                    continue
                
                class_name = obj.category.name
                confidence = obj.score.value
                
                # Map to our classes
                detection = self._map_detection_to_class(class_name, confidence, [x1, y1, x2, y2], area)
                detections.append(detection)
            
            # Filter detections
            detections = self._filter_detections(detections)
            
            return detections
            
        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")
            return []
    
    def _map_detection_to_class(self, class_name: str, confidence: float, bbox: List[int], area: float) -> Dict:
        """Map YOLO class to our target classes"""
        # Direct match
        if class_name == 'person':
            return {
                'class': 'person',
                'yolo_class': class_name,
                'confidence': confidence,
                'bbox': bbox,
                'description': 'person',
                'method': 'sahi+yolo+tensorrt' if self.model_format_detected == MODEL_FORMAT_TENSORRT else 'sahi+yolo',
                'area': area,
                'is_target': True
            }
        
        # Person-like
        person_like = ['doll', 'teddy bear']
        if class_name in person_like:
            return {
                'class': 'person',
                'yolo_class': class_name,
                'confidence': confidence,
                'bbox': bbox,
                'description': f'person-like ({class_name})',
                'method': 'sahi+yolo+tensorrt' if self.model_format_detected == MODEL_FORMAT_TENSORRT else 'sahi+yolo',
                'area': area,
                'is_target': True
            }
        
        # Tent-like
        tent_like = [
            'umbrella', 'kite', 'bed', 'couch', 'boat',
            'backpack', 'suitcase', 'handbag', 'surfboard', 'bench',
            'airplane', 'truck', 'car', 'bus', 'frisbee'
        ]
        if class_name in tent_like:
            return {
                'class': 'tent',
                'yolo_class': class_name,
                'confidence': confidence,
                'bbox': bbox,
                'description': f'tent-like ({class_name})',
                'method': 'sahi+yolo+tensorrt' if self.model_format_detected == MODEL_FORMAT_TENSORRT else 'sahi+yolo',
                'area': area,
                'is_target': True
            }
        
        # Everything else
        return {
            'class': 'object',
            'yolo_class': class_name,
            'confidence': confidence,
            'bbox': bbox,
            'description': f'detected: {class_name}',
            'method': 'sahi+yolo+tensorrt' if self.model_format_detected == MODEL_FORMAT_TENSORRT else 'sahi+yolo',
            'area': area,
            'is_target': False
        }
    
    def _filter_detections(self, detections: List[Dict]) -> List[Dict]:
        """Filter overlapping detections"""
        if len(detections) == 0:
            return detections
        
        detections = sorted(detections, key=lambda x: x['confidence'], reverse=True)
        
        filtered = []
        for target_class in ['person', 'tent', 'object']:
            class_detections = [d for d in detections if d['class'] == target_class]
            
            if len(class_detections) > 0:
                keep = self._apply_nms(class_detections, overlap_threshold=0.3)
                filtered.extend(keep)
        
        return filtered
    
    def _apply_nms(self, detections: List[Dict], overlap_threshold: float = 0.3) -> List[Dict]:
        """Apply Non-Maximum Suppression"""
        if len(detections) == 0:
            return detections
        
        keep = []
        remaining = detections.copy()
        
        while remaining:
            current = remaining.pop(0)
            keep.append(current)
            
            new_remaining = []
            for detection in remaining:
                iou = self._calculate_iou(current['bbox'], detection['bbox'])
                if iou < overlap_threshold:
                    new_remaining.append(detection)
            remaining = new_remaining
        
        return keep
    
    def _calculate_iou(self, bbox1: List[int], bbox2: List[int]) -> float:
        """Calculate Intersection over Union"""
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i <= x1_i or y2_i <= y1_i:
            return 0.0
        
        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0
    
    def save_individual_crops(self, frame: np.ndarray, detections: List[Dict], image_path: str) -> Tuple[List[str], List[Dict]]:
        """
        STEROIDS FEATURE 1: Save all detected boxes as individual crop images
        
        Returns:
            Tuple of (crop_paths, crop_metadata)
            crop_metadata contains: {'orig_bbox': [x1,y1,x2,y2], 'padded_bbox': [x1,y1,x2,y2], 'detection': detection_dict}
        """
        crop_paths = []
        crop_metadata = []
        
        if not detections:
            return crop_paths
        
        height, width = frame.shape[:2]
        original_filename = os.path.basename(image_path)
        name, ext = os.path.splitext(original_filename)
        
        for idx, detection in enumerate(detections):
            try:
                x1, y1, x2, y2 = detection['bbox']
                
                # Add padding
                pad = self.crop_padding
                x1 = max(0, x1 - pad)
                y1 = max(0, y1 - pad)
                x2 = min(width, x2 + pad)
                y2 = min(height, y2 + pad)
                
                # Crop image
                if x2 > x1 and y2 > y1:
                    crop = frame[y1:y2, x1:x2].copy()
                    
                    # Create filename: <imagename>_crop_<idx>_<class>_<confidence>.jpg
                    class_name = detection['class']
                    conf = int(detection['confidence'] * 100)
                    crop_filename = f"{name}_crop_{idx:02d}_{class_name}_{conf:02d}{ext}"
                    crop_path = os.path.join(self.crop_output_path, crop_filename)
                    
                    # Save crop
                    cv2.imwrite(crop_path, crop)
                    crop_paths.append(crop_path)
                    
                    # Store metadata for mapping stitched detections back
                    crop_metadata.append({
                        'orig_bbox': detection['bbox'],  # Original detection bbox
                        'padded_bbox': [x1, y1, x2, y2],  # After padding
                        'detection': detection  # Full detection dict
                    })
                    
                    self.stats['total_crops_saved'] += 1
                    
            except Exception as e:
                self.get_logger().debug(f"Error saving crop {idx}: {e}")
        
        if crop_paths:
            self.get_logger().info(f"✓ Saved {len(crop_paths)} individual crops")
        
        return crop_paths
    
    def add_detection_labels(self, image: np.ndarray, detections: List[Dict], label_prefix: str = "") -> np.ndarray:
        """
        Add text labels to image based on existing detections (NO re-detection)
        
        Args:
            image: Input image
            detections: Already computed detections from original image
            label_prefix: Prefix for labels (e.g., "STITCHED:" or "ENHANCED:")
            
        Returns:
            Image with detection labels added at bottom
        """
        try:
            
            # Create canvas with space for labels at bottom
            h, w = image.shape[:2]
            label_height = 60  # Space for labels
            canvas = np.zeros((h + label_height, w, 3), dtype=np.uint8)
            canvas[:h, :] = image
            
            # Dark background for labels
            canvas[h:, :] = (40, 40, 40)
            
            if detections:
                # Create label text
                person_count = sum(1 for d in detections if d['class'] == 'person')
                tent_count = sum(1 for d in detections if d['class'] == 'tent')
                other_count = sum(1 for d in detections if d['class'] == 'object')
                
                # Get highest confidence for each class
                person_conf = max([d['confidence'] for d in detections if d['class'] == 'person'], default=0)
                tent_conf = max([d['confidence'] for d in detections if d['class'] == 'tent'], default=0)
                other_conf = max([d['confidence'] for d in detections if d['class'] == 'object'], default=0)
                
                # Build label text
                labels = []
                if person_count > 0:
                    labels.append(f"PERSON: {person_count}x ({person_conf:.0%})")
                if tent_count > 0:
                    labels.append(f"TENT: {tent_count}x ({tent_conf:.0%})")
                if other_count > 0:
                    labels.append(f"OBJECT: {other_count}x ({other_conf:.0%})")
                
                label_text = label_prefix + " | " + " | ".join(labels) if labels else label_prefix + " | No detections"
            else:
                label_text = label_prefix + " | No detections"
            
            # Draw label text
            font = cv2.FONT_HERSHEY_DUPLEX
            font_scale = 0.6
            thickness = 1
            
            # Calculate text size and position
            (text_w, text_h), _ = cv2.getTextSize(label_text, font, font_scale, thickness)
            text_x = (w - text_w) // 2
            text_y = h + (label_height + text_h) // 2
            
            # Draw text with outline for better visibility
            cv2.putText(canvas, label_text, (text_x, text_y),
                       font, font_scale, (0, 0, 0), thickness + 1, cv2.LINE_AA)
            cv2.putText(canvas, label_text, (text_x, text_y),
                       font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA)
            
            return canvas
            
        except Exception as e:
            self.get_logger().error(f"Error in detect_and_label_image: {e}")
            return image
    
    def stitch_crops(self, crop_paths: List[str], crop_metadata: List[Dict], image_path: str) -> Tuple[Optional[np.ndarray], Optional[str], List[Dict]]:
        """
        STEROIDS FEATURE 2: Stitch all crops together side by side
        
        Returns:
            Tuple of (stitched_image, stitched_path, placements)
            placements contains: {'x_start': int, 'x_end': int, 'scale': float, 'metadata': crop_metadata_dict}
        """
        if not crop_paths:
            return None, None, []
        
        try:
            # Load all crops
            crops = []
            for crop_path in crop_paths:
                crop = cv2.imread(crop_path)
                if crop is not None:
                    crops.append(crop)
            
            if not crops:
                return None, None, []
            
            # Resize all crops to same height (640px to preserve detail for enhancement)
            # Don't shrink to 200px - that destroys detail that AI can't recover!
            target_height = 640
            resized_crops = []
            placements = []  # Track placement info for mapping back
            
            for idx, crop in enumerate(crops):
                h, w = crop.shape[:2]
                if h > 0:
                    scale = target_height / h
                    new_w = int(w * scale)
                    # Use INTER_AREA for downscaling, INTER_CUBIC for upscaling
                    interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_CUBIC
                    resized = cv2.resize(crop, (new_w, target_height), interpolation=interp)
                    resized_crops.append(resized)
                    
                    # Store placement info (will be updated with x_start/x_end later)
                    placements.append({
                        'scale': scale,
                        'orig_size': (w, h),
                        'new_size': (new_w, target_height),
                        'metadata': crop_metadata[idx] if idx < len(crop_metadata) else None
                    })
            
            if not resized_crops:
                return None, None, []
            
            # Stitch horizontally with spacing
            spacing = self.stitch_spacing
            total_width = sum(c.shape[1] for c in resized_crops) + spacing * (len(resized_crops) - 1)
            
            # Create canvas
            stitched = np.zeros((target_height, total_width, 3), dtype=np.uint8)
            
            # Copy crops with spacing and record positions
            x_offset = 0
            for i, crop in enumerate(resized_crops):
                if i > 0:
                    # Add spacing
                    stitched[:, x_offset:x_offset + spacing] = (50, 50, 50)
                    x_offset += spacing
                
                # Record position in stitched image
                placements[i]['x_start'] = x_offset
                placements[i]['x_end'] = x_offset + crop.shape[1]
                
                stitched[:, x_offset:x_offset + crop.shape[1]] = crop
                x_offset += crop.shape[1]
            
            # Return stitched image without labels (will be added by caller with existing detections)
            # stitched_labeled = stitched  # Labels added by caller
            
            # Save stitched image with labels
            original_filename = os.path.basename(image_path)
            name, ext = os.path.splitext(original_filename)
            stitched_filename = f"{name}_stitched{ext}"
            stitched_path = os.path.join(self.stitch_output_path, stitched_filename)
            
            cv2.imwrite(stitched_path, stitched)
            
            self.stats['total_stitched_images'] += 1
            self.get_logger().info(f"✓ Stitched {len(resized_crops)} crops: {stitched_filename}")
            
            return stitched, stitched_path, placements
            
        except Exception as e:
            self.get_logger().error(f"Error stitching crops: {e}")
            return None, None, []
    
    def save_enhanced_image(self, enhanced_image: np.ndarray, image_path: str) -> Optional[str]:
        """
        STEROIDS FEATURE 3: Save AI-enhanced image (already labeled)
        
        Returns:
            Path to saved enhanced image
        """
        try:
            # Enhanced image is already labeled by caller
            original_filename = os.path.basename(image_path)
            name, ext = os.path.splitext(original_filename)
            enhanced_filename = f"{name}_enhanced{ext}"
            enhanced_path = os.path.join(self.enhanced_output_path, enhanced_filename)
            
            cv2.imwrite(enhanced_path, enhanced_image)
            
            return enhanced_path
            
        except Exception as e:
            self.get_logger().error(f"Error saving enhanced image: {e}")
            return None
    
    def annotate_frame(self, frame: np.ndarray, detections: List[Dict], processing_time: float) -> np.ndarray:
        """Annotate frame with detection results"""
        annotated_frame = frame.copy()
        height, width = frame.shape[:2]
        
        # Colors
        COLOR_TENT = (0, 200, 255)
        COLOR_PERSON = (0, 200, 0)
        COLOR_OBJECT = (255, 100, 0)
        COLOR_WHITE = (255, 255, 255)
        COLOR_BLACK = (0, 0, 0)
        
        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = 0.45
        font_thickness = 1
        
        # Draw detections
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
            
            # Draw label
            (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, font_thickness)
            padding = 4
            label_h = text_h + padding * 2
            
            if y1 - label_h >= 0:
                label_y1 = y1 - label_h
                label_y2 = y1
            else:
                label_y1 = y1
                label_y2 = y1 + label_h
            
            if align_right:
                label_x1 = max(0, x2 - text_w - padding * 2)
                label_x2 = x2
            else:
                label_x1 = x1
                label_x2 = x1 + text_w + padding * 2
            
            cv2.rectangle(annotated_frame, (label_x1, label_y1), (label_x2, label_y2), box_color, -1)
            
            text_x = label_x1 + padding
            text_y = label_y2 - padding
            
            cv2.putText(annotated_frame, label, (text_x, text_y),
                       font, font_scale, COLOR_BLACK, font_thickness + 1, cv2.LINE_AA)
            cv2.putText(annotated_frame, label, (text_x, text_y),
                       font, font_scale, COLOR_WHITE, font_thickness, cv2.LINE_AA)
        
        # Header
        num_people = sum(1 for d in detections if d['class'] == 'person')
        num_tents = sum(1 for d in detections if d['class'] == 'tent')
        num_other = len(detections) - num_people - num_tents
        
        header_font_scale = 0.6
        method_str = 'TensorRT' if self.model_format_detected == MODEL_FORMAT_TENSORRT else 'PyTorch'
        header_text = f"STEROIDS MODE | {num_people} people, {num_tents} tents, {num_other} other"
        time_text = f"Time: {processing_time:.1f}s | Crops: {self.stats['total_crops_saved']}"
        
        overlay = annotated_frame.copy()
        cv2.rectangle(overlay, (0, 0), (width, 50), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, annotated_frame, 0.4, 0, annotated_frame)
        
        cv2.putText(annotated_frame, header_text, (10, 20),
                   font, header_font_scale, COLOR_WHITE, 1, cv2.LINE_AA)
        cv2.putText(annotated_frame, time_text, (10, 42),
                   font, header_font_scale * 0.8, (200, 200, 200), 1, cv2.LINE_AA)
        
        return annotated_frame
    
    def publish_results(self, annotated_frame: np.ndarray, detections: List[Dict], image_path: str) -> None:
        """Publish detection results"""
        try:
            if self.publisher is None or self.detection_pub is None or self.detection_publisher is None:
                return
            
            if self.bridge is None:
                return
            
            # Convert to ROS2 Image
            image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera'
            
            # Publish image
            self.publisher.publish(image_msg)
            
            # Save annotated image
            original_filename = os.path.basename(image_path)
            name, ext = os.path.splitext(original_filename)
            output_filename = f"steroids_detected_{name}{ext}"
            output_path = os.path.join(self.detection_results_path, output_filename)
            cv2.imwrite(output_path, annotated_frame)
            
            # Create detection info
            method = 'sahi+yolo+tensorrt+STEROIDS' if self.model_format_detected == MODEL_FORMAT_TENSORRT else 'sahi+yolo+STEROIDS'
            
            num_people = sum(1 for d in detections if d['class'] == 'person')
            num_tents = sum(1 for d in detections if d['class'] == 'tent')
            num_objects = sum(1 for d in detections if d['class'] == 'object')
            
            detection_info = {
                'image': os.path.basename(image_path),
                'timestamp': datetime.now().isoformat(),
                'num_people': num_people,
                'num_tents': num_tents,
                'num_objects': num_objects,
                'total_detections': len(detections),
                'method': method,
                'slice_size': f"{self.slice_height}x{self.slice_width}",
                'overlap': f"{self.overlap_height_ratio}x{self.overlap_width_ratio}",
                'crops_saved': self.stats['total_crops_saved'],
                'stitched_images': self.stats['total_stitched_images'],
                'enhanced_images': self.stats['total_enhanced_images'],
            }
            
            # Create ImageResult message
            image_result_msg = ImageResult()
            image_result_msg.header = image_msg.header
            image_result_msg.image_name = original_filename
            image_result_msg.timestamp = datetime.now().isoformat()
            image_result_msg.num_detections = len(detections)
            image_result_msg.saved_to = output_path
            image_result_msg.method = method
            image_result_msg.slice_size = f"{self.slice_height}x{self.slice_width}"
            image_result_msg.overlap = f"{self.overlap_height_ratio}x{self.overlap_width_ratio}"
            image_result_msg.waypoint_index = self.waypoint_reached
            
            # Prepare Detection2DArray
            det_array = Detection2DArray()
            det_array.header = image_msg.header
            
            classes = []
            confidences = []
            areas = []
            descriptions = []
            masks = []
            
            for det in detections:
                # Create Detection2D
                d2d = Detection2D()
                d2d.header = image_msg.header
                
                x1, y1, x2, y2 = det['bbox']
                d2d.bbox.center.position.x = float(x1 + x2) / 2.0
                d2d.bbox.center.position.y = float(y1 + y2) / 2.0
                d2d.bbox.size_x = float(x2 - x1)
                d2d.bbox.size_y = float(y2 - y1)
                
                hypo = ObjectHypothesisWithPose()
                hypo.hypothesis.class_id = CLASS_ID.get(det['class'], "2")
                hypo.hypothesis.score = float(det['confidence'])
                d2d.results.append(hypo)
                
                det_array.detections.append(d2d)
                
                classes.append(det['class'])
                confidences.append(float(det['confidence']))
                areas.append(float(det.get('area', 0)))
                descriptions.append(det.get('description', det['class']))
            
            image_result_msg.detections = det_array
            image_result_msg.masks = masks
            image_result_msg.classes = classes
            image_result_msg.confidences = confidences
            image_result_msg.areas = areas
            image_result_msg.descriptions = descriptions
            
            # Publish
            self.detection_pub.publish(image_result_msg)
            
            info_msg = String()
            info_msg.data = json.dumps(detection_info, separators=(",", ":"))
            self.detection_publisher.publish(info_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing results: {e}")
            self.stats['errors'] += 1


def main(args=None):
    rclpy.init(args=args)
    
    # Check dependencies
    if not SAHI_AVAILABLE or not YOLO_AVAILABLE:
        print("\n" + "="*80)
        print("ERROR: Missing required dependencies!")
        print("="*80)
        if not SAHI_AVAILABLE:
            print("SAHI not found. Install: pip install sahi")
        if not YOLO_AVAILABLE:
            print("Ultralytics YOLO not found. Install: pip install ultralytics")
        print("="*80 + "\n")
        return
    
    node = SAHIObjectDetectionNodeSteroids()
    
    try:
        # Configure
        if node.trigger_configure() != TransitionCallbackReturn.SUCCESS:
            node.get_logger().error("Failed to configure node")
            node.destroy_node()
            rclpy.shutdown()
            return
        
        # Activate
        if node.trigger_activate() != TransitionCallbackReturn.SUCCESS:
            node.get_logger().error("Failed to activate node")
            node.trigger_cleanup()
            node.destroy_node()
            rclpy.shutdown()
            return
        
        # Use multithreaded executor
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        
        node.get_logger().info("STEROIDS NODE IS ACTIVE! 🚀")
        
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        if not node.shutdown_requested:
            node.get_logger().info("Initiating lifecycle shutdown...")
            try:
                node.trigger_deactivate()
                node.trigger_cleanup()
                node.trigger_shutdown()
            except Exception as e:
                node.get_logger().error(f"Error during lifecycle shutdown: {e}")
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
