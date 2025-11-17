#!/usr/bin/env python3

"""
SAHI Object Detection Node using YOLO and MobileNetV3 for Small Object Detection
Specifically optimized for detecting small tents and people in aerial imagery
Uses Slicing Aided Hyper Inference (SAHI) for improved small object detection

Current Architecture:
- SAHI: Slices images and manages detection pipeline
- YOLO: Performs actual object detection on each slice
- MobileNetV3: Validates detections by classifying cropped regions (PyTorch torchvision)

Detection Pipeline:
1. SAHI slices the image into overlapping patches
2. YOLO detects objects in each slice
3. Results are merged and filtered (NMS)
4. MobileNetV3 validates each detection by classifying the cropped region
5. Detections are annotated with both YOLO and MobileNet results

Configuration:
- Slice size: 512x512 (optimized for small object detection)
- Overlap: 30% (ensures objects at boundaries are detected)
- Balance: Accuracy over speed for critical small object detection
- MobileNet validation: Enabled by default, can be disabled via parameter
"""
# ros2 imports
import rclpy # define ros2 nodes
from rclpy.node import Node # define ros2 nodes 
# Removed get_package_share_directory - now using ~/video_cam directory instead
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
# This is just to make sure that we have all the dependencies
#SAHI
try:
    from sahi import AutoDetectionModel
    from sahi.predict import get_sliced_prediction
    from sahi.utils.cv import read_image
    SAHI_AVAILABLE = True
except ImportError:
    SAHI_AVAILABLE = False
    print("ERROR: sahi not available. Please install with: pip install sahi")
#PyTorch
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    print("WARNING: torch not available, will use CPU only")
#YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("ERROR: ultralytics not available. Please install with: pip install ultralytics")
#MobileNetV3
try:
    import torchvision
    from torchvision import transforms
    from torchvision.models import mobilenet_v3_large, MobileNet_V3_Large_Weights
    MOBILENET_AVAILABLE = True
except ImportError:
    MOBILENET_AVAILABLE = False
    print("WARNING: torchvision not available. MobileNetV3 validation will be disabled. Install with: pip install torchvision")

# Create video_cam directory in ros2_ws
def get_video_cam_directory():
    """Returns the path to video_cam directory in ros2_ws"""
    # Try to find ros2_ws directory by looking for install/ or src/ directories
    current_file = os.path.abspath(__file__)
    current_dir = os.path.dirname(current_file)
    
    # Navigate up to find ros2_ws (look for install/ or src/ directories)
    search_dir = current_dir
    ros2_ws_dir = None
    
    for _ in range(10):  # Limit search depth
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
    
    # Fallback: construct path directly
    if ros2_ws_dir is None:
        # Default to expected location
        ros2_ws_dir = "/home/aro/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws"
    
    video_cam_dir = os.path.join(ros2_ws_dir, "video_cam")
    os.makedirs(video_cam_dir, exist_ok=True)
    return video_cam_dir

class SAHIObjectDetectionNode(Node):
    def __init__(self):
        # Ros2 name. Used so that other nodes can discover its topics, parameters and services
        super().__init__('sahi_object_detection_node_mobilenet')
        
        # Params
        self.declare_parameter('model_path', 'yolo11s.pt')
        self.declare_parameter('confidence_threshold', 0.15)
        self.declare_parameter('slice_height', 512)  # Reverted to original size for better accuracy
        self.declare_parameter('slice_width', 512)   # Reverted to original size for better accuracy
        self.declare_parameter('overlap_height_ratio', 0.3)  # Reverted to original overlap
        self.declare_parameter('overlap_width_ratio', 0.3)   # Reverted to original overlap
        self.declare_parameter('check_interval', 2.0)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('use_mobilenet_validation', True)  # Enable/disable MobileNet validation
        self.declare_parameter('mobilenet_confidence_threshold', 0.3)  # Minimum confidence for MobileNet validation
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.slice_height = self.get_parameter('slice_height').value
        self.slice_width = self.get_parameter('slice_width').value
        self.overlap_height_ratio = self.get_parameter('overlap_height_ratio').value
        self.overlap_width_ratio = self.get_parameter('overlap_width_ratio').value
        self.check_interval = self.get_parameter('check_interval').value
        self.device = self.get_parameter('device').value
        self.use_mobilenet_validation = self.get_parameter('use_mobilenet_validation').value
        self.mobilenet_confidence_threshold = self.get_parameter('mobilenet_confidence_threshold').value
        
        # ROS2 setup
        # Converts betwen ros iamge to opencv image
        self.bridge = CvBridge()
        
        # Topics (todo)
        self.publisher = self.create_publisher(Image, '/sahi_detection_results', 10)
        self.detection_publisher = self.create_publisher(String, '/sahi_detection_info', 10)
        self.detection_pub = self.create_publisher(ImageResult, '/image_detections', 10)
        
        # Use single video_cam directory in home directory
        video_cam_dir = get_video_cam_directory()
        
        # Get camera_feed directory path
        self.camera_feed_path = os.path.join(
            video_cam_dir, 
            "camera_feed"
        )
        
        # Get detection_results directory path
        self.detection_results_path = os.path.join(
            video_cam_dir, 
            "detection_results_sahi"
        )
        
        # Create directories if they don't exist
        os.makedirs(self.camera_feed_path, exist_ok=True)
        os.makedirs(self.detection_results_path, exist_ok=True)
        
        self.get_logger().info(f"SAHI Object Detection Node - Monitoring: {self.camera_feed_path}")
        self.get_logger().info(f"Detection results will be saved to: {self.detection_results_path}")
        
        # Auto-detect device(gpu, mps or cpu)
        if self.device == 'auto':
            self.device = self._get_device()
        self.get_logger().info(f"Using device: {self.device}")
        
        # Initialize SAHI model
        self.detection_model = None
        self.initialize_sahi_model()
        
        # Initialize MobileNetV3 for validation
        self.mobilenet_model = None
        self.mobilenet_transforms = None
        self.mobilenet_class_names = None
        if self.use_mobilenet_validation:
            self.initialize_mobilenet_model()
        
        # Processing state
        self.processed_images = set()  # Track processed images
        
        # Statistics
        self.stats = {
            'total_images_processed': 0,
            'total_detections': 0,
            'total_tents': 0,
            'total_people': 0,
            'avg_processing_time': 0.0
        }
        
        # Timer to check for new images
        self.timer = self.create_timer(self.check_interval, self.check_for_new_images)
        
        self.get_logger().info("="*80)
        self.get_logger().info("SAHI Object Detection Node Initialized")
        self.get_logger().info(f"Model: {self.model_path}")
        self.get_logger().info(f"Confidence Threshold: {self.confidence_threshold}")
        self.get_logger().info(f"Slice Size: {self.slice_height}x{self.slice_width}")
        self.get_logger().info(f"Overlap Ratio: {self.overlap_height_ratio}x{self.overlap_width_ratio}")
        self.get_logger().info(f"Device: {self.device}")
        self.get_logger().info(f"MobileNet Validation: {'Enabled' if (self.use_mobilenet_validation and self.mobilenet_model is not None) else 'Disabled'}")
        if self.use_mobilenet_validation and self.mobilenet_model is not None:
            self.get_logger().info(f"MobileNet Confidence Threshold: {self.mobilenet_confidence_threshold}")
        self.get_logger().info("="*80)

        # wp subscriber
        self.waypoint_reached = 0
        self.create_subscription(WaypointReached, "/mavros/mission/reached", self.waypoint_reached_cb, 10)

    def waypoint_reached_cb(self, msg):
        #updates with current waypoint index
        self.waypoint_reached = msg.wp_seq
    
    def _get_device(self):
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
            self.get_logger().info(f"   Device: {device_name}")
            self.get_logger().info(f"   GPU Count: {device_count}")
            self.get_logger().info(f"   Compute Capability: {compute_capability[0]}.{compute_capability[1]}")
            self.get_logger().info(f"   CUDA Version: {torch.version.cuda}")
            
            # Check if this is a Jetson device
            try:
                with open('/proc/device-tree/model', 'r') as f:
                    model = f.read()
                    if 'jetson' in model.lower():
                        self.get_logger().info(f"   Platform: NVIDIA Jetson ({model.strip()})")
            except:
                pass
            
            return "cuda:0"
        
        # Try Apple Metal Performance Shaders (MPS) for Mac
        elif hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
            self.get_logger().info(" Apple Silicon GPU (MPS) Detected!")
            self.get_logger().info("   Using Metal Performance Shaders for acceleration")
            
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
                        self.get_logger().error("   Current PyTorch version: {torch.__version__}")
                        self.get_logger().error("   Expected: PyTorch with CUDA support (not CPU-only)")
            except:
                pass
            
            self.get_logger().info("   Consider using a GPU for better performance!")
            
            return "cpu"
    
    def initialize_sahi_model(self):
        """Initialize SAHI detection model with YOLO"""
        try:
            if not SAHI_AVAILABLE:
                self.get_logger().error("SAHI is not available. Please install: pip install sahi")
                return False
            
            if not YOLO_AVAILABLE:
                self.get_logger().error("Ultralytics YOLO is not available. Please install: pip install ultralytics")
                return False
            
            # Resolve model path - check multiple locations
            model_path = self.model_path
            
            # If path is relative, try different locations
            if not os.path.isabs(model_path):
                # 1. Try current directory (video_cam directory)
                video_cam_dir = get_video_cam_directory()
                video_cam_model_path = os.path.join(video_cam_dir, model_path)
                if os.path.exists(video_cam_model_path):
                    model_path = video_cam_model_path
                    self.get_logger().info(f"Using model from video_cam directory: {model_path}")
                # 2. Try source package directory (where yolo11s.pt might be)
                elif os.path.exists(os.path.join(os.path.dirname(__file__), model_path)):
                    model_path = os.path.join(os.path.dirname(__file__), model_path)
                    self.get_logger().info(f"Using model from package directory: {model_path}")
                # 3. Check if it exists as-is (current working directory)
                elif os.path.exists(model_path):
                    self.get_logger().info(f"Using model from current directory: {model_path}")
                else:
                    self.get_logger().warn(f"Model file not found at {self.model_path}, will download from Ultralytics if needed")
            elif not os.path.exists(model_path):
                self.get_logger().warn(f"Model file not found at {model_path}, will download from Ultralytics if needed")
            
            self.get_logger().info("Loading SAHI YOLOv11s model for small object detection...")
            self.get_logger().info(f"Target device: {self.device}")
            
            # GPU memory optimization for CUDA
            if self.device.startswith('cuda') and TORCH_AVAILABLE:
                try:
                    # Clear GPU cache before loading model
                    torch.cuda.empty_cache()
                    
                    # Get GPU memory info
                    gpu_mem_total = torch.cuda.get_device_properties(0).total_memory / 1024**3  # GB
                    gpu_mem_reserved = torch.cuda.memory_reserved(0) / 1024**3
                    gpu_mem_allocated = torch.cuda.memory_allocated(0) / 1024**3
                    gpu_mem_free = gpu_mem_total - gpu_mem_allocated
                    
                    self.get_logger().info(f"   GPU Memory: {gpu_mem_free:.2f}GB free / {gpu_mem_total:.2f}GB total")
                    
                    # Warn if low memory
                    if gpu_mem_free < 1.0:
                        self.get_logger().warn(f"   Low GPU memory ({gpu_mem_free:.2f}GB), consider using a smaller model")
                except Exception as e:
                    self.get_logger().debug(f"Could not get GPU memory info: {e}")
            
            # Initialize SAHI AutoDetectionModel
            # Note: SAHI uses 'yolov8' as the model_type identifier for YOLO v8+ models (including YOLO11)
            # The actual model file is yolo11s.pt, specified in model_path
            self.detection_model = AutoDetectionModel.from_pretrained(
                model_type='yolov8',  # SAHI model type identifier (works for YOLO v8, v9, v10, v11)
                model_path=model_path,  # Actual model: yolo11s.pt
                confidence_threshold=self.confidence_threshold,
                device=self.device,
            )
            
            self.get_logger().info(" SAHI YOLOv11s model loaded successfully!")
            
            # Verify model is on correct device
            if TORCH_AVAILABLE and hasattr(self.detection_model, 'model'):
                try:
                    model_device = next(self.detection_model.model.model.parameters()).device
                    self.get_logger().info(f" Model confirmed on device: {model_device}")
                except:
                    pass
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize SAHI model: {e}")
            self.get_logger().error("Make sure you have installed: pip install sahi ultralytics torch")
            
            # If GPU failed, suggest fallback to CPU
            if self.device != 'cpu':
                self.get_logger().error(f"Try running with CPU instead: --ros-args -p device:=cpu")
            
            return False
    
    def initialize_mobilenet_model(self):
        """Initialize MobileNetV3 model for validation"""
        try:
            if not MOBILENET_AVAILABLE:
                self.get_logger().warn("MobileNetV3 is not available. Install with: pip install torchvision")
                return False
            
            if not TORCH_AVAILABLE:
                self.get_logger().warn("PyTorch not available, MobileNetV3 cannot be loaded")
                return False
            
            self.get_logger().info("Loading MobileNetV3Large for validation...")
            
            # Load pretrained MobileNetV3Large with ImageNet weights
            weights = MobileNet_V3_Large_Weights.IMAGENET1K_V1
            self.mobilenet_model = mobilenet_v3_large(weights=weights)
            self.mobilenet_model.eval()  # Set to evaluation mode
            
            # Move to appropriate device
            if self.device.startswith('cuda'):
                self.mobilenet_model = self.mobilenet_model.to('cuda')
            elif self.device == 'mps':
                self.mobilenet_model = self.mobilenet_model.to('mps')
            else:
                self.mobilenet_model = self.mobilenet_model.to('cpu')
            
            # Get ImageNet preprocessing transforms
            self.mobilenet_transforms = weights.transforms()
            
            # Load ImageNet class names for mapping
            # ImageNet class indices can be mapped to names
            # We'll create a mapping for relevant classes
            self._setup_imagenet_class_mapping()
            
            self.get_logger().info(f"MobileNetV3Large loaded successfully on {self.device}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MobileNetV3: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.mobilenet_model = None
            return False
    
    def _setup_imagenet_class_mapping(self):
        """Setup ImageNet class name mapping for person and tent-like objects"""
        # ImageNet class indices for relevant classes
        # These are approximate - ImageNet doesn't have exact "tent" class
        # but has related objects that might be detected
        
        # Person-related classes (ImageNet has "person" at index around 345-365)
        # Common ImageNet classes that might indicate person or tent
        self.imagenet_person_indices = [
            345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359, 360,
            361, 362, 363, 364, 365, 366, 367, 368, 369, 370, 371, 372, 373, 374, 375, 376
        ]  # Various person classes
        
        # Tent-like object classes (backpack, sleeping bag, etc.)
        # These are approximate indices - we'll use top-k predictions instead
        self.imagenet_tent_like_keywords = [
            'backpack', 'pack', 'rucksack', 'knapsack',
            'sleeping_bag', 'sleeping', 'bag',
            'military_uniform', 'uniform',
            'umbrella', 'parachute',
            'tarp', 'canvas', 'awning',
            'suitcase', 'luggage', 'baggage'
        ]
        
        # Load full ImageNet class names if available
        self.imagenet_class_names = None
        try:
            # Try multiple methods to load ImageNet class names
            import urllib.request
            import tempfile
            
            # Method 1: Try to download from PyTorch hub
            url = "https://raw.githubusercontent.com/pytorch/hub/master/imagenet_classes.txt"
            try:
                with tempfile.NamedTemporaryFile(mode='w+', delete=False, suffix='.txt') as tmp_file:
                    tmp_path = tmp_file.name
                
                urllib.request.urlretrieve(url, tmp_path, timeout=5)
                with open(tmp_path, "r") as f:
                    self.imagenet_class_names = [line.strip() for line in f.readlines()]
                
                # Clean up
                os.unlink(tmp_path)
                
                if self.imagenet_class_names and len(self.imagenet_class_names) > 0:
                    self.get_logger().info(f"Loaded {len(self.imagenet_class_names)} ImageNet class names from PyTorch hub")
                else:
                    self.imagenet_class_names = None
            except Exception as e:
                self.get_logger().debug(f"Could not download ImageNet class names: {e}")
                self.imagenet_class_names = None
            
            # If download failed, we'll use keyword matching based on class indices
            # This is still functional, just less descriptive
            if self.imagenet_class_names is None:
                self.get_logger().info("Using keyword-based class matching (ImageNet class names not loaded)")
        except Exception as e:
            self.get_logger().debug(f"Error setting up ImageNet class names: {e}")
            self.imagenet_class_names = None
    
    def check_for_new_images(self):
        """Check for new images in camera_feed folder"""
        try:
            if not os.path.exists(self.camera_feed_path):
                self.get_logger().warn(f"Camera feed path does not exist: {self.camera_feed_path}")
                return
            
            if self.detection_model is None:
                self.get_logger().warn("SAHI model not initialized, skipping detection")
                return
            
            # Get all image files and save em in a list
            image_files = []
            for file in os.listdir(self.camera_feed_path):
                if file.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')):
                    image_files.append(file)
            
            # Log status periodically
            if len(image_files) > 0:
                self.get_logger().info(
                    f"Found {len(image_files)} total images, "
                    f"{len(self.processed_images)} already processed"
                )
            
            # Process new images
            new_images_processed = 0
            for image_file in image_files:
                if image_file not in self.processed_images:
                    self.get_logger().info(f"Processing new image: {image_file}")
                    image_path = os.path.join(self.camera_feed_path, image_file)
                    self.process_image(image_path)
                    self.processed_images.add(image_file)
                    new_images_processed += 1
            
            if new_images_processed > 0:
                self.get_logger().info(f"Processed {new_images_processed} new images")
                self._log_statistics()
                    
        except Exception as e:
            self.get_logger().error(f"Error checking for new images: {e}")
    
    def process_image(self, image_path):
        """Process a single image using SAHI for small object detection"""
        """In here we call detect_object_sahi, annotated_frame and publish_result methods"""
        try:
            start_time = time.time()
            
            # Load image
            frame = cv2.imread(image_path)
            if frame is None:
                self.get_logger().warn(f"Could not load image: {image_path}")
                return
            
            height, width = frame.shape[:2]
            self.get_logger().info(f"Processing image: {os.path.basename(image_path)} ({width}x{height})")
            
            # Run SAHI prediction (another function)
            detections = self.detect_objects_sahi(frame)
            
            processing_time = time.time() - start_time
            
            # Create annotated frame
            annotated_frame = self.annotate_frame(frame, detections, processing_time)
            
            # Publish results
            self.publish_results(annotated_frame, detections, image_path)
            
            # Update statistics
            self.stats['total_images_processed'] += 1
            self.stats['total_detections'] += len(detections)
            self.stats['total_tents'] += sum(1 for d in detections if d['class'] == 'tent')
            self.stats['total_people'] += sum(1 for d in detections if d['class'] == 'person')
            
            # Update average processing time
            n = self.stats['total_images_processed']
            self.stats['avg_processing_time'] = (
                (self.stats['avg_processing_time'] * (n - 1) + processing_time) / n
            )
            
            self.get_logger().info(
                f" Found {len(detections)} objects in {processing_time:.2f}s: "
                f"{sum(1 for d in detections if d['class'] == 'person')} people, "
                f"{sum(1 for d in detections if d['class'] == 'tent')} tents"
            )
                
        except Exception as e:
            self.get_logger().error(f"Error processing image {image_path}: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def detect_objects_sahi(self, frame):
        """
        Detect objects using SAHI (Slicing Aided Hyper Inference)
        
        SAHI slices the image into smaller patches with overlap, runs detection
        on each patch, then merges the results. This is highly effective for
        detecting small objects in large images (e.g., tents in aerial photos).
        """
        detections = []
        
        try:
            # Convert BGR to RGB for SAHI (openCV loads in BGR and pytorch wants in RGB)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            self.get_logger().info(
                f"Running SAHI prediction with {self.slice_height}x{self.slice_width} slices, "
                f"{self.overlap_height_ratio:.1%}x{self.overlap_width_ratio:.1%} overlap..."
            )
            
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
            
            self.get_logger().info(f"SAHI found {len(result.object_prediction_list)} raw detections")
            
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
            
            # Validate with MobileNetV3 if enabled
            if self.use_mobilenet_validation and self.mobilenet_model is not None:
                detections = self.validate_with_mobilenet(frame, detections)
            
        except Exception as e:
            self.get_logger().error(f"Error in SAHI detection: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        
        return detections
    
    def _categorize_detection(self, class_name, confidence, bbox, frame):
        """
        Categorize YOLO detections into our target classes (person/mannequin, tent)
        
        Args:
            class_name: YOLO class name
            confidence: Detection confidence
            bbox: Bounding box [x1, y1, x2, y2]
            frame: Original image frame
            
        Returns:
            Detection dict or None if not a target class
        """
        x1, y1, x2, y2 = bbox
        
        # Person/Mannequin detection - direct person detection
        if class_name == 'person':
            if confidence > 0.25:  # Lower threshold for SAHI to catch more small people
                return {
                    'class': 'person',
                    'yolo_class': class_name,
                    'confidence': confidence,
                    'bbox': bbox,
                    'description': 'person',
                    'method': 'sahi+yolo11s',
                    'area': (x2 - x1) * (y2 - y1)
                }
        
        # Mannequin detection - map various YOLO classes that could be mannequins
        # In aerial/drone imagery, mannequins might be detected as various objects
        mannequin_like_classes = {
            'doll': 0.20,           # Mannequins often detected as dolls
        }
        
        if class_name in mannequin_like_classes:
            threshold = mannequin_like_classes[class_name]
            if confidence > threshold:
                # Additional validation: check size and aspect ratio
                width = x2 - x1
                height = y2 - y1
                aspect_ratio = width / height if height > 0 else 0
                area = width * height
                
                # Mannequins should have reasonable size and aspect ratio
                # Typically more vertical/humanoid than tents
                if area > 300 and 0.3 < aspect_ratio < 3.0:
                    return {
                        'class': 'person',
                        'yolo_class': class_name,
                        'confidence': confidence,
                        'bbox': bbox,
                        'description': f'mannequin-like ({class_name})',
                        'method': 'sahi+yolo11s',
                        'area': area
                    }
        
        # Tent detection - only kite and umbrella
        tent_like_classes = {
            'kite': 0.20,          # Tent fabric might look like kites
            'umbrella': 0.20,      # Tent canopies might look like umbrellas
        }
        
        if class_name in tent_like_classes:
            threshold = tent_like_classes[class_name]
            if confidence > threshold:
                # Additional validation: check size and aspect ratio
                width = x2 - x1
                height = y2 - y1
                aspect_ratio = width / height if height > 0 else 0
                area = width * height
                
                # Tents should have reasonable size and aspect ratio
                if area > 500 and 0.2 < aspect_ratio < 5.0:
                    return {
                        'class': 'tent',
                        'yolo_class': class_name,
                        'confidence': confidence,
                        'bbox': bbox,
                        'description': f'tent-like ({class_name})',
                        'method': 'sahi+yolo11s',
                        'area': area
                    }
        
        return None
    
    def _filter_detections(self, detections):
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
        
        # Apply NMS within each class
        filtered = []
        for target_class in ['person', 'tent']:
            class_detections = [d for d in detections if d['class'] == target_class]
            
            if len(class_detections) > 0:
                # Apply NMS
                keep = self._apply_nms(class_detections, overlap_threshold=0.3)
                filtered.extend(keep)
        
        return filtered
    
    def _apply_nms(self, detections, overlap_threshold=0.3):
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
    
    def _calculate_iou(self, bbox1, bbox2):
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
    
    def validate_with_mobilenet(self, frame, detections):
        """
        Validate YOLO+SAHI detections using MobileNetV3 classification
        
        For each detection, crops the region and classifies it with MobileNetV3.
        Updates detection with MobileNet classification results.
        
        Args:
            frame: Original image frame (BGR format)
            detections: List of detection dicts from YOLO+SAHI
            
        Returns:
            List of validated detections with MobileNet info
        """
        if self.mobilenet_model is None or len(detections) == 0:
            return detections
        
        validated_detections = []
        
        try:
            with torch.no_grad():  # Disable gradient computation for inference
                for detection in detections:
                    x1, y1, x2, y2 = detection['bbox']
                    original_class = detection['class']
                    
                    # Add padding around bounding box (10% padding)
                    height, width = frame.shape[:2]
                    padding_w = int((x2 - x1) * 0.1)
                    padding_h = int((y2 - y1) * 0.1)
                    
                    # Ensure coordinates are within image bounds
                    x1_padded = max(0, x1 - padding_w)
                    y1_padded = max(0, y1 - padding_h)
                    x2_padded = min(width, x2 + padding_w)
                    y2_padded = min(height, y2 + padding_h)
                    
                    # Extract ROI
                    roi = frame[y1_padded:y2_padded, x1_padded:x2_padded]
                    
                    if roi.size == 0:
                        # Keep detection but mark as not validated
                        detection['mobilenet_validated'] = False
                        detection['mobilenet_class'] = 'unknown'
                        detection['mobilenet_confidence'] = 0.0
                        validated_detections.append(detection)
                        continue
                    
                    # Convert BGR to RGB
                    roi_rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
                    
                    # Convert to PIL Image for transforms
                    try:
                        from PIL import Image
                        roi_pil = Image.fromarray(roi_rgb)
                    except ImportError:
                        self.get_logger().error("PIL (Pillow) not available. Install with: pip install Pillow")
                        detection['mobilenet_validated'] = False
                        detection['mobilenet_class'] = 'error'
                        detection['mobilenet_confidence'] = 0.0
                        validated_detections.append(detection)
                        continue
                    
                    # Apply MobileNet preprocessing transforms
                    roi_tensor = self.mobilenet_transforms(roi_pil).unsqueeze(0)
                    
                    # Move to device
                    if self.device.startswith('cuda'):
                        roi_tensor = roi_tensor.to('cuda')
                    elif self.device == 'mps':
                        roi_tensor = roi_tensor.to('mps')
                    
                    # Run inference
                    outputs = self.mobilenet_model(roi_tensor)
                    
                    # Get top-k predictions (top 5)
                    probabilities = torch.nn.functional.softmax(outputs[0], dim=0)
                    top_k = 5
                    top_probs, top_indices = torch.topk(probabilities, top_k)
                    
                    # Convert to CPU numpy
                    top_probs = top_probs.cpu().numpy()
                    top_indices = top_indices.cpu().numpy()
                    
                    # Get class names
                    mobilenet_class, mobilenet_confidence = self._interpret_mobilenet_predictions(
                        top_indices, top_probs, original_class
                    )
                    
                    # Update detection with MobileNet results
                    detection['mobilenet_validated'] = True
                    detection['mobilenet_class'] = mobilenet_class
                    detection['mobilenet_confidence'] = float(mobilenet_confidence)
                    detection['mobilenet_top_predictions'] = [
                        {
                            'class': self._get_class_name(idx),
                            'confidence': float(prob)
                        }
                        for idx, prob in zip(top_indices, top_probs)
                    ]
                    
                    # Decide if we should keep this detection based on MobileNet validation
                    # If MobileNet strongly disagrees, we might want to filter it out
                    # For now, we keep all detections but mark them with MobileNet results
                    keep_detection = True
                    
                    # If original class is person, check if MobileNet also detects person-like
                    if original_class == 'person':
                        if mobilenet_class == 'person' and mobilenet_confidence > self.mobilenet_confidence_threshold:
                            # Strong agreement
                            detection['validation_status'] = 'confirmed'
                        elif mobilenet_confidence > 0.1:
                            # Some confidence, but might be different class
                            detection['validation_status'] = 'partial'
                        else:
                            detection['validation_status'] = 'disagreement'
                    
                    # If original class is tent, check if MobileNet detects tent-like objects
                    elif original_class == 'tent':
                        if mobilenet_class in ['tent', 'tent_like'] and mobilenet_confidence > self.mobilenet_confidence_threshold:
                            detection['validation_status'] = 'confirmed'
                        elif mobilenet_class in ['tent_like'] and mobilenet_confidence > 0.1:
                            detection['validation_status'] = 'partial'
                        else:
                            detection['validation_status'] = 'disagreement'
                    
                    if keep_detection:
                        validated_detections.append(detection)
                    else:
                        self.get_logger().debug(
                            f"Filtered detection: {original_class} (MobileNet: {mobilenet_class}, "
                            f"conf: {mobilenet_confidence:.2f})"
                        )
        
        except Exception as e:
            self.get_logger().error(f"Error in MobileNet validation: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            # Return original detections if validation fails
            return detections
        
        self.get_logger().info(
            f"MobileNet validated {len(validated_detections)}/{len(detections)} detections"
        )
        
        return validated_detections
    
    def _interpret_mobilenet_predictions(self, top_indices, top_probs, original_class):
        """
        Interpret MobileNet predictions and map to our target classes (person, tent)
        
        Args:
            top_indices: Top-k class indices from MobileNet
            top_probs: Top-k probabilities from MobileNet
            original_class: Original class from YOLO ('person' or 'tent')
            
        Returns:
            tuple: (mapped_class, confidence)
        """
        # Get class names for top predictions
        class_names = []
        for idx in top_indices:
            class_name = self._get_class_name(idx)
            class_names.append(class_name.lower())
        
        # Check for person-related classes
        person_keywords = ['person', 'man', 'woman', 'girl', 'boy', 'child', 'adult', 
                          'human', 'people', 'pedestrian', 'walker']
        
        # Check for tent-like classes
        tent_keywords = ['backpack', 'pack', 'rucksack', 'knapsack', 'sleeping', 'bag',
                        'suitcase', 'luggage', 'baggage', 'umbrella', 'parachute',
                        'tarp', 'canvas', 'awning', 'tent', 'camping']
        
        # Find best matching class
        best_class = 'unknown'
        best_confidence = 0.0
        
        # Check each top prediction
        for idx, prob, class_name in zip(top_indices, top_probs, class_names):
            # Check if it matches person
            if any(keyword in class_name for keyword in person_keywords):
                if prob > best_confidence:
                    best_class = 'person'
                    best_confidence = float(prob)
            
            # Check if it matches tent
            elif any(keyword in class_name for keyword in tent_keywords):
                if prob > best_confidence:
                    best_class = 'tent_like'
                    best_confidence = float(prob)
        
        # If no clear match, use the top prediction
        if best_class == 'unknown' and len(top_probs) > 0:
            best_class = class_names[0]
            best_confidence = float(top_probs[0])
        
        # Map to our target classes
        if original_class == 'person' and best_class == 'person':
            return 'person', best_confidence
        elif original_class == 'tent' and best_class == 'tent_like':
            return 'tent_like', best_confidence
        elif best_class == 'person':
            return 'person', best_confidence
        elif best_class == 'tent_like':
            return 'tent_like', best_confidence
        else:
            return best_class, best_confidence
    
    def _get_class_name(self, class_idx):
        """Get ImageNet class name from index"""
        if self.imagenet_class_names and 0 <= class_idx < len(self.imagenet_class_names):
            return self.imagenet_class_names[class_idx]
        else:
            return f"class_{class_idx}"
    
    def annotate_frame(self, frame, detections, processing_time):
        """
        Annotate frame with SAHI detection results in the style of the reference images
        
        Args:
            frame: Original image
            detections: List of detection dicts
            processing_time: Time taken for detection
            
        Returns:
            Annotated frame
        """
        annotated_frame = frame.copy()
        height, width = frame.shape[:2]
        
        # Define colors (BGR format)
        COLOR_TENT = (0, 255, 255)  # Yellow for tents
        COLOR_PERSON = (0, 255, 0)  # Green for people
        COLOR_TEXT_BG = (0, 0, 0)   # Black background
        COLOR_TEXT = (255, 255, 255)  # White text
        
        # Annotate each detection
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            class_name = detection['class']
            confidence = detection['confidence']
            yolo_class = detection.get('yolo_class', class_name)
            
            # Choose color based on class
            if class_name == 'person':
                box_color = COLOR_PERSON
                target_label = "TARGET: PERSON"
            elif class_name == 'tent':
                box_color = COLOR_TENT
                target_label = "TARGET: TENT"
            else:
                box_color = (128, 128, 128)  # Grey
                target_label = f"OBJECT: {class_name.upper()}"
            
            # Draw bounding box (thinner for cleaner look)
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), box_color, 2)
            
            # Prepare text labels
            yolo_label = f"YOLO: {yolo_class} ({confidence:.2f})"
            
            # Get MobileNet validation results if available
            mobilenet_validated = detection.get('mobilenet_validated', False)
            if mobilenet_validated:
                mobilenet_class = detection.get('mobilenet_class', 'unknown')
                mobilenet_confidence = detection.get('mobilenet_confidence', 0.0)
                validation_status = detection.get('validation_status', 'unknown')
                
                # Create MobileNet label with validation status indicator
                status_icon = "✓" if validation_status == 'confirmed' else "?" if validation_status == 'partial' else "!"
                mobilenet_label = f"MobileNet: {mobilenet_class} ({mobilenet_confidence:.2f}) {status_icon}"
            else:
                mobilenet_label = "MobileNet: Not validated"
            # Calculate text size for background
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            thickness = 1
            
            (w1, h1), _ = cv2.getTextSize(target_label, font, font_scale, thickness)
            (w2, h2), _ = cv2.getTextSize(yolo_label, font, font_scale, thickness)
            (w3, h3), _ = cv2.getTextSize(mobilenet_label, font, font_scale, thickness)

            # Calculate background rectangle size
            max_width = max(w1, w2, w3) + 10
            total_height = h1 + h2 + h3 + 10
            
            # Draw text background (yellow for tent, green for person)
            text_y_start = max(y1 - total_height, 0)
            cv2.rectangle(
                annotated_frame,
                (x1, text_y_start),
                (x1 + max_width, y1),
                box_color,
                -1  # Filled
            )
            
            # Draw text labels
            text_y = text_y_start + h1 + 5
            cv2.putText(annotated_frame, target_label, (x1 + 5, text_y),
                       font, font_scale, COLOR_TEXT_BG, 2)
            
            text_y += h2 + 5
            cv2.putText(annotated_frame, yolo_label, (x1 + 5, text_y),
                       font, font_scale, COLOR_TEXT_BG, 2)
            
            # Add MobileNet validation label
            text_y += h3 + 5
            cv2.putText(annotated_frame, mobilenet_label, (x1 + 5, text_y),
                       font, font_scale, COLOR_TEXT_BG, 2)
        
        # Add header with detection info
        detection_method = "SAHI + YOLO"
        if self.use_mobilenet_validation and self.mobilenet_model is not None:
            detection_method += " + MobileNetV3"
        header_text = f"{detection_method} - {len(detections)} objects detected"
        cv2.putText(annotated_frame, header_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, COLOR_TEXT, 2)
        
        # Add processing time
        time_text = f"Processing Time: {processing_time:.2f}s"
        cv2.putText(annotated_frame, time_text, (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_TEXT, 2)
        
        # Add SAHI mode indicator at bottom
        sahi_text = f"SAHI Mode: {self.slice_height}x{self.slice_width} slices, {self.overlap_height_ratio:.0%} overlap"
        if self.use_mobilenet_validation and self.mobilenet_model is not None:
            sahi_text += " | MobileNetV3 Validation: ON"
        cv2.putText(annotated_frame, sahi_text, (10, height - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_TEXT, 2)
        
        return annotated_frame
    
    def publish_results(self, annotated_frame, detections, image_path):
        """Publish annotated image and detection info, and save to disk"""
        try:
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
            
            # Create detection info message
            method = 'sahi+yolo11s'
            if self.use_mobilenet_validation and self.mobilenet_model is not None:
                method += '+mobilenetv3'
            
            detection_info = {
                'image': os.path.basename(image_path),
                'timestamp': datetime.now().isoformat(),
                'detections': len(detections),
                'saved_to': output_path,
                'method': method,
                'slice_size': f"{self.slice_height}x{self.slice_width}",
                'overlap': f"{self.overlap_height_ratio}x{self.overlap_width_ratio}",
                'mobilenet_validation': self.use_mobilenet_validation and self.mobilenet_model is not None,
                'objects': [
                    {
                        'class': d['class'],
                        'yolo_class': d.get('yolo_class', d['class']),
                        'confidence': d['confidence'],
                        'bbox': d['bbox'],
                        'description': d.get('description', d['class']),
                        'area': d.get('area', 0),
                        'mobilenet_validated': d.get('mobilenet_validated', False),
                        'mobilenet_class': d.get('mobilenet_class', None),
                        'mobilenet_confidence': d.get('mobilenet_confidence', None),
                        'validation_status': d.get('validation_status', None)
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
            masks = []  # Empty because you aren’t generating segmentation masks

            for det in detections:
                # Create individual Detection2D
                d2d = Detection2D()
                d2d.header = image_msg.header

                # Create bounding box
                x1, y1, x2, y2 = det['bbox']
                d2d.bbox.center.x = (x1 + x2) / 2.0
                d2d.bbox.center.y = (y1 + y2) / 2.0
                d2d.bbox.size_x = x2 - x1
                d2d.bbox.size_y = y2 - y1

                # Add hypothesis (class + confidence)
                hypo = ObjectHypothesisWithPose()
                hypo.hypothesis.class_id = det['class']
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
            
            self.get_logger().info(
                f" Published and saved results -> {output_filename}"
            )
            
        except Exception as e:
            self.get_logger().error(f"Error publishing/saving results: {e}")
    
    def _log_statistics(self):
        """Log detection statistics"""
        self.get_logger().info("="*80)
        self.get_logger().info("SAHI Detection Statistics:")
        self.get_logger().info(f"  Total Images Processed: {self.stats['total_images_processed']}")
        self.get_logger().info(f"  Total Detections: {self.stats['total_detections']}")
        self.get_logger().info(f"  Total Tents: {self.stats['total_tents']}")
        self.get_logger().info(f"  Total People: {self.stats['total_people']}")
        self.get_logger().info(f"  Avg Processing Time: {self.stats['avg_processing_time']:.2f}s")
        self.get_logger().info("="*80)


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
    
    # Warn about optional dependencies
    if not MOBILENET_AVAILABLE:
        print("\n" + "="*80)
        print("WARNING: MobileNetV3 validation will be disabled!")
        print("="*80)
        print("torchvision not found. Install with:")
        print("  pip install torchvision")
        print("="*80 + "\n")
    elif not TORCH_AVAILABLE:
        print("\n" + "="*80)
        print("WARNING: MobileNetV3 validation will be disabled!")
        print("="*80)
        print("PyTorch not found. Install with:")
        print("  pip install torch")
        print("="*80 + "\n")
    
    node = SAHIObjectDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down SAHI Object Detection Node...")
    finally:
        # Log final statistics
        node._log_statistics()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()