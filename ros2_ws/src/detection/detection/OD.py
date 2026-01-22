#!/usr/bin/env python3

"""
Standalone SAHI Object Detection Script (without ROS2)
Based on new_od.py but modified to work as a standalone Python script

This script:
- Takes images from ros2_ws/src/video_cam/mapping_photos (same as new_od.py)
- Saves results to ros2_ws/src/detection/detection_results_sahi (same as new_od.py)
- Uses SAHI for sliced object detection
- Works on any OS, CUDA version, GPU/CPU
- Monitors the folder for new images and processes them automatically
"""

# Standard library imports
import cv2
import numpy as np
import time
from datetime import datetime
import os
from pathlib import Path
from typing import List, Dict, Optional, Tuple
import gc
import platform
import json
import statistics
import signal
import sys

# Progress bar
try:
    from tqdm import tqdm
    TQDM_AVAILABLE = True
except ImportError:
    TQDM_AVAILABLE = False

# SAHI
try:
    from sahi import AutoDetectionModel
    from sahi.predict import get_sliced_prediction
    from sahi.utils.cv import read_image
    SAHI_AVAILABLE = True
except ImportError:
    SAHI_AVAILABLE = False

# PyTorch
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

# YOLO
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

# TensorRT
try:
    import tensorrt as trt
    TENSORRT_AVAILABLE = True
except ImportError:
    TENSORRT_AVAILABLE = False


# Constants
DEFAULT_CONFIDENCE_THRESHOLD = 0.05
DEFAULT_SLICE_SIZE = 640
DEFAULT_OVERLAP = 0.15
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
        
        # Keep only last 100 records
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
    
    # Navigate up to find ros2_ws
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
        if search_dir == "/" or (platform.system() == "Windows" and len(search_dir) <= 3):
            break
    
    # Fallback
    if ros2_ws_dir is None:
        ros2_ws_dir = os.getenv('ROS2_WS_PATH') or os.path.expanduser('~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws')
    
    return ros2_ws_dir


class ObjectDetector:
    """Standalone Object Detector using SAHI"""
    
    def __init__(self, 
                 model_path='yolo26m.pt',
                 model_format=MODEL_FORMAT_AUTO,
                 confidence_threshold=DEFAULT_CONFIDENCE_THRESHOLD,
                 slice_height=DEFAULT_SLICE_SIZE,
                 slice_width=DEFAULT_SLICE_SIZE,
                 overlap_height_ratio=DEFAULT_OVERLAP,
                 overlap_width_ratio=DEFAULT_OVERLAP,
                 check_interval=DEFAULT_CHECK_INTERVAL,
                 device='auto',
                 max_images_per_cycle=5,
                 max_camera_feed_images=100,
                 min_detection_area=25,
                 max_detection_area=1000000,
                 min_aspect_ratio=0.1,
                 max_aspect_ratio=10.0,
                 enable_gpu_memory_cleanup=True,
                 camera_feed_path='',
                 detection_results_path='',
                 auto_convert_tensorrt=True,
                 tensorrt_workspace=4):
        
        # Configuration
        self.model_path = model_path
        self.model_format = model_format.lower() if model_format else MODEL_FORMAT_AUTO
        self.auto_convert_tensorrt = auto_convert_tensorrt
        self.tensorrt_workspace = tensorrt_workspace
        self.confidence_threshold = confidence_threshold
        self.slice_height = slice_height
        self.slice_width = slice_width
        self.overlap_height_ratio = overlap_height_ratio
        self.overlap_width_ratio = overlap_width_ratio
        self.check_interval = check_interval
        self.device = device
        self.max_images_per_cycle = max_images_per_cycle
        self.max_camera_feed_images = max_camera_feed_images
        self.min_detection_area = min_detection_area
        self.max_detection_area = max_detection_area
        self.min_aspect_ratio = min_aspect_ratio
        self.max_aspect_ratio = max_aspect_ratio
        self.enable_gpu_memory_cleanup = enable_gpu_memory_cleanup
        
        # Setup directories
        ros2_ws_dir = get_ros2_ws_directory()
        
        if not camera_feed_path:
            self.camera_feed_path = os.path.join(ros2_ws_dir, "src", "video_cam", "mapping_photos")
        else:
            self.camera_feed_path = camera_feed_path
        
        if not detection_results_path:
            self.detection_results_path = os.path.join(ros2_ws_dir, "src", "detection", "detection_results_sahi")
        else:
            self.detection_results_path = detection_results_path
        
        # Create results directory
        os.makedirs(self.detection_results_path, exist_ok=True)
        
        # Check if camera_feed exists
        if not os.path.exists(self.camera_feed_path):
            print(f"WARNING: Camera feed directory does not exist: {self.camera_feed_path}")
            print("Creating it, but you should place images there for detection")
            os.makedirs(self.camera_feed_path, exist_ok=True)
        
        print(f"Camera feed path: {self.camera_feed_path}")
        print(f"Detection results path: {self.detection_results_path}")
        
        # OpenCV optimizations
        cv2.setNumThreads(0)
        cv2.ocl.setUseOpenCL(False)
        
        # Auto-detect device
        if self.device == 'auto':
            self.device = self._get_device()
        print(f"Using device: {self.device}")
        
        # Optimize GPU memory
        if self.device.startswith('cuda'):
            self._optimize_gpu_memory()
        
        # Model format detection
        self.model_format_detected = self._detect_model_format()
        
        # Processing state
        self.processed_images: Dict[str, float] = {}
        self.is_processing = False
        self.running = True
        
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
            'model_reloads': 0
        }
        
        # Health monitoring
        self.health_status = {
            'is_healthy': True,
            'last_successful_detection': None,
            'consecutive_errors': 0
        }
        
        # Model
        self.detection_model = None
        
        # Initialize signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, sig, frame):
        """Handle shutdown signals gracefully"""
        print("\n\nShutdown signal received. Cleaning up...")
        self.running = False
        self._print_statistics()
        sys.exit(0)
    
    def _get_device(self) -> str:
        """Auto-detect the best available device"""
        if not TORCH_AVAILABLE:
            print("WARNING: PyTorch not available, falling back to CPU")
            return "cpu"
        
        # Try NVIDIA CUDA
        if torch.cuda.is_available():
            device_name = torch.cuda.get_device_name(0)
            print(f"✓ CUDA GPU Detected: {device_name}")
            return "cuda:0"
        
        # Try Apple MPS
        elif hasattr(torch.backends, 'mps') and torch.backends.mps.is_available():
            print("✓ Apple Silicon GPU (MPS) Detected!")
            if not torch.backends.mps.is_built():
                print("  WARNING: MPS is available but not built, falling back to CPU")
                return "cpu"
            return "mps"
        
        # Fallback to CPU
        else:
            print(f"⚠ No GPU detected, using CPU")
            print(f"  System: {platform.system()} {platform.machine()}")
            print(f"  CPU Count: {os.cpu_count()}")
            return "cpu"
    
    def _optimize_gpu_memory(self):
        """Optimize GPU memory settings"""
        if not self.device.startswith('cuda') or not TORCH_AVAILABLE:
            return
        
        try:
            total_memory = torch.cuda.get_device_properties(0).total_memory / 1024**3
            
            if total_memory < 4:
                recommended_slice = 640
                recommended_overlap = 0.10
                max_fraction = 0.6
                print("Low GPU memory detected: Using large slices + low overlap")
            elif total_memory < 8:
                recommended_slice = 512
                recommended_overlap = 0.15
                max_fraction = 0.7
                print("Medium GPU memory detected: Using balanced settings")
            else:
                recommended_slice = 416
                recommended_overlap = 0.20
                max_fraction = 0.75
                print("High GPU memory detected: Using optimal settings")
            
            torch.cuda.set_per_process_memory_fraction(max_fraction, 0)
            
            split_size = 256 if recommended_slice >= 512 else 128
            os.environ['PYTORCH_CUDA_ALLOC_CONF'] = f'max_split_size_mb:{split_size},expandable_segments:True'
            
            print(f"GPU Memory Config: {total_memory:.1f}GB total, using {max_fraction*100:.0f}% max")
            
        except Exception as e:
            print(f"WARNING: GPU memory optimization failed: {e}")
    
    def _detect_model_format(self) -> str:
        """Detect model format from file extension"""
        if self.model_path.endswith('.engine'):
            return MODEL_FORMAT_TENSORRT
        elif self.model_path.endswith('.pt'):
            return MODEL_FORMAT_PYTORCH
        else:
            print(f"WARNING: Unknown model format for {self.model_path}. Assuming PyTorch.")
            return MODEL_FORMAT_PYTORCH
    
    def _convert_pytorch_to_tensorrt(self, pt_path: str, engine_path: str) -> bool:
        """Convert PyTorch model to TensorRT engine"""
        if not YOLO_AVAILABLE:
            print("ERROR: Ultralytics YOLO not available for conversion")
            return False
        
        if not TENSORRT_AVAILABLE:
            print("ERROR: TensorRT not available. Install: pip install nvidia-tensorrt")
            return False
        
        try:
            print(f"Converting {pt_path} to TensorRT format...")
            print("This may take several minutes on first run...")
            
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
                print(f"✓ TensorRT conversion successful: {engine_path}")
                return True
            else:
                print(f"ERROR: TensorRT engine file not found at {exported_engine}")
                return False
                
        except Exception as e:
            print(f"ERROR: TensorRT conversion failed: {str(e)}")
            return False
    
    def _resolve_model_path(self) -> Tuple[str, str]:
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
                print(f"WARNING: Model file not found at {self.model_path}")
        
        # Determine format
        if self.model_format == MODEL_FORMAT_AUTO:
            base_name = os.path.splitext(model_path)[0]
            engine_path = f"{base_name}.engine"
            
            if os.path.exists(engine_path):
                print(f"Found TensorRT engine: {engine_path}")
                return engine_path, MODEL_FORMAT_TENSORRT
            elif model_path.endswith('.pt') and self.auto_convert_tensorrt and TENSORRT_AVAILABLE:
                if self._convert_pytorch_to_tensorrt(model_path, engine_path):
                    return engine_path, MODEL_FORMAT_TENSORRT
                else:
                    print("TensorRT conversion failed, using PyTorch")
                    return model_path, MODEL_FORMAT_PYTORCH
            else:
                return model_path, self.model_format_detected
        elif self.model_format == MODEL_FORMAT_TENSORRT:
            if model_path.endswith('.engine'):
                if os.path.exists(model_path):
                    return model_path, MODEL_FORMAT_TENSORRT
            else:
                base_name = os.path.splitext(model_path)[0]
                engine_path = f"{base_name}.engine"
                if os.path.exists(engine_path):
                    return engine_path, MODEL_FORMAT_TENSORRT
                elif model_path.endswith('.pt') and os.path.exists(model_path):
                    if self._convert_pytorch_to_tensorrt(model_path, engine_path):
                        return engine_path, MODEL_FORMAT_TENSORRT
                    else:
                        print("ERROR: TensorRT conversion required but failed")
                        return None, None
        else:
            return model_path, MODEL_FORMAT_PYTORCH
        
        return None, None
    
    def initialize_model(self) -> bool:
        """Initialize SAHI detection model"""
        try:
            if not SAHI_AVAILABLE:
                print("ERROR: SAHI is not available. Install: pip install sahi")
                return False
            
            if not YOLO_AVAILABLE:
                print("ERROR: Ultralytics YOLO is not available. Install: pip install ultralytics")
                return False
            
            resolved_path, final_format = self._resolve_model_path()
            if resolved_path is None:
                print("ERROR: Failed to resolve model path")
                return False
            
            print(f"Loading model: {resolved_path} (format: {final_format})")
            
            # GPU memory optimization for CUDA
            if self.device.startswith('cuda') and TORCH_AVAILABLE:
                try:
                    gc.collect()
                    torch.cuda.empty_cache()
                    torch.cuda.synchronize()
                    
                    torch.cuda.set_per_process_memory_fraction(0.8, 0)
                    torch.backends.cudnn.benchmark = True
                    torch.backends.cudnn.enabled = True
                    
                    gpu_mem_total = torch.cuda.get_device_properties(0).total_memory / 1024**3
                    gpu_mem_allocated = torch.cuda.memory_allocated(0) / 1024**3
                    gpu_mem_free = gpu_mem_total - gpu_mem_allocated
                    
                    print(f"GPU Memory: {gpu_mem_free:.2f}GB free / {gpu_mem_total:.2f}GB total")
                    
                    if gpu_mem_free < 2.0:
                        print(f"WARNING: Low GPU memory ({gpu_mem_free:.2f}GB)")
                except Exception as e:
                    print(f"DEBUG: Could not get GPU memory info: {e}")
            
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
                    print("✓ TensorRT model loaded via Ultralytics+SAHI wrapper")
                except Exception as e:
                    print(f"ERROR: Failed to load TensorRT model: {e}")
                    base_name = os.path.splitext(resolved_path)[0]
                    pt_path = f"{base_name}.pt"
                    if os.path.exists(pt_path):
                        print(f"Using PyTorch fallback: {pt_path}")
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
                print("✓ PyTorch model loaded via SAHI")
            
            # Warmup model
            self._warmup_model()
            
            return True
            
        except Exception as e:
            print(f"ERROR: Failed to initialize SAHI model: {e}")
            import traceback
            traceback.print_exc()
            self.stats['errors'] += 1
            self.health_status['consecutive_errors'] += 1
            self.health_status['is_healthy'] = False
            return False
    
    def _warmup_model(self):
        """Warmup model with dummy inference"""
        if self.detection_model is None:
            return
        
        print("Warming up model...")
        
        try:
            dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
            _ = get_sliced_prediction(
                dummy_img,
                self.detection_model,
                slice_height=self.slice_height,
                slice_width=self.slice_width,
                overlap_height_ratio=self.overlap_height_ratio,
                overlap_width_ratio=self.overlap_width_ratio,
                verbose=0
            )
            print("Model warmup complete")
        except Exception as e:
            print(f"WARNING: Model warmup failed: {e}")
    
    def _is_file_ready(self, path: str, min_age_s: float = 0.2) -> bool:
        """Check if file is fully written and stable"""
        try:
            st = os.stat(path)
            if (time.time() - st.st_mtime) < min_age_s:
                return False
            
            size1 = st.st_size
            if size1 == 0:
                return False
            
            time.sleep(0.05)
            size2 = os.stat(path).st_size
            return size1 == size2
        except OSError:
            return False
    
    def _prune_processed(self, max_age_s: float = 3600.0, max_entries: int = 2000):
        """Prevent unbounded memory growth"""
        now = time.time()
        
        old = [k for k, v in self.processed_images.items() if (now - v) > max_age_s]
        for k in old:
            self.processed_images.pop(k, None)
        
        if len(self.processed_images) > max_entries:
            items = sorted(self.processed_images.items(), key=lambda kv: kv[1])
            for k, _ in items[:len(self.processed_images) - max_entries]:
                self.processed_images.pop(k, None)
    
    def _cleanup_old_images(self):
        """Remove old images from camera_feed if limit is exceeded"""
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
                    except OSError:
                        pass
                
                if removed > 0:
                    print(f"Cleaned up {removed} old images from camera_feed")
        except Exception as e:
            print(f"DEBUG: Error during image cleanup: {e}")
    
    def check_for_new_images(self):
        """Check for new images and process them"""
        try:
            if not os.path.exists(self.camera_feed_path):
                print(f"WARNING: Camera feed path does not exist: {self.camera_feed_path}")
                return
            
            if self.detection_model is None:
                print("WARNING: Model not initialized, skipping detection")
                return
            
            # Clean up old images
            self._cleanup_old_images()
            
            # Prune processed dict
            self._prune_processed()
            
            # Get all image files
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
            
            # Process new images
            processed_count = 0
            for fname, mtime in image_files_with_time:
                if processed_count >= self.max_images_per_cycle:
                    break
                
                if fname in self.processed_images:
                    continue
                
                image_path = os.path.join(self.camera_feed_path, fname)
                
                if not self._is_file_ready(image_path):
                    continue
                
                print(f"\n✅ Processing NEW image: {fname}")
                self.process_image(image_path)
                self.processed_images[fname] = mtime
                processed_count += 1
                
                self.health_status['last_successful_detection'] = time.time()
                self.health_status['consecutive_errors'] = 0
                self.health_status['is_healthy'] = True
            
        except Exception as e:
            print(f"ERROR: Error checking for new images: {e}")
            import traceback
            traceback.print_exc()
            self.stats['errors'] += 1
            self.health_status['consecutive_errors'] += 1
    
    def process_image(self, image_path: str):
        """Process a single image"""
        try:
            start_time = time.time()
            
            frame = cv2.imread(image_path)
            if frame is None:
                print(f"WARNING: Could not load image: {image_path}")
                return
            
            height, width = frame.shape[:2]
            print(f"📸 Image size: {width}x{height}")
            
            frame_orig = frame.copy()
            
            # Run detection
            detections = self.detect_objects_sahi(frame)
            
            processing_time = time.time() - start_time
            self.stats['last_processing_time'] = processing_time
            
            # Create annotated frame
            annotated_frame = self.annotate_frame(frame, detections, processing_time)
            
            # Save crops
            self.save_top_matches_crop(frame_orig, detections, image_path)
            
            # Save results
            self.save_results(annotated_frame, detections, image_path)
            
            # Update statistics
            self.stats['total_images_processed'] += 1
            self.stats['total_detections'] += len(detections)
            self.stats['total_tents'] += sum(1 for d in detections if d['class'] == 'tent')
            self.stats['total_people'] += sum(1 for d in detections if d['class'] == 'person')
            self.stats['total_objects'] += sum(1 for d in detections if d['class'] == 'object')
            
            n = self.stats['total_images_processed']
            self.stats['avg_processing_time'] = (
                (self.stats['avg_processing_time'] * (n - 1) + processing_time) / n
            )
            
            # Record performance
            if TORCH_AVAILABLE and self.device.startswith('cuda'):
                try:
                    mem_used = torch.cuda.memory_allocated(0) / 1024**3
                except:
                    mem_used = 0.0
            else:
                mem_used = 0.0
            
            self.perf_monitor.record_detection(processing_time, len(detections), mem_used)
            
            num_people = sum(1 for d in detections if d['class'] == 'person')
            num_tents = sum(1 for d in detections if d['class'] == 'tent')
            num_objects = sum(1 for d in detections if d['class'] == 'object')
            
            print(
                f"✓ Found {len(detections)} objects in {processing_time:.2f}s: "
                f"{num_people} people, {num_tents} tents, {num_objects} other objects"
            )
                
        except Exception as e:
            print(f"ERROR: Error processing image {image_path}: {e}")
            import traceback
            traceback.print_exc()
            self.stats['errors'] += 1
    
    def detect_objects_sahi(self, frame: np.ndarray) -> List[Dict]:
        """Detect objects using SAHI"""
        detections: List[Dict] = []
        
        try:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            h, w = frame_rgb.shape[:2]
            stride_h = int(self.slice_height * (1 - self.overlap_height_ratio))
            stride_w = int(self.slice_width * (1 - self.overlap_width_ratio))
            num_slices_h = max(1, (h - self.slice_height) // stride_h + 1) if stride_h > 0 else 1
            num_slices_w = max(1, (w - self.slice_width) // stride_w + 1) if stride_w > 0 else 1
            total_slices = num_slices_h * num_slices_w
            
            if TQDM_AVAILABLE:
                print(f"Processing with ~{total_slices} slices...")
                pbar = tqdm(total=100, desc="SAHI Detection", unit="%", ncols=80)
            
            start_time = time.time()
            
            result = get_sliced_prediction(
                frame_rgb,
                self.detection_model,
                slice_height=self.slice_height,
                slice_width=self.slice_width,
                overlap_height_ratio=self.overlap_height_ratio,
                overlap_width_ratio=self.overlap_width_ratio,
                postprocess_type="NMS",
                postprocess_match_metric="IOS",
                postprocess_match_threshold=0.5,
                postprocess_class_agnostic=False,
                verbose=0
            )
            
            if TQDM_AVAILABLE:
                pbar.update(100)
                pbar.close()
                elapsed = time.time() - start_time
                print(f"Detection complete in {elapsed:.1f}s - found {len(result.object_prediction_list)} raw detections")
            
            for object_prediction in result.object_prediction_list:
                bbox = object_prediction.bbox
                x1, y1, x2, y2 = int(bbox.minx), int(bbox.miny), int(bbox.maxx), int(bbox.maxy)
                
                class_name = object_prediction.category.name
                confidence = float(object_prediction.score.value)
                
                detection = self._categorize_detection(
                    class_name, confidence, [x1, y1, x2, y2], frame
                )
                
                if detection:
                    detections.append(detection)
            
            detections = self._filter_detections(detections)
            
            if self.enable_gpu_memory_cleanup and self.device.startswith('cuda') and TORCH_AVAILABLE:
                if (self.stats["total_images_processed"] % 10) == 0:
                    try:
                        gc.collect()
                        torch.cuda.empty_cache()
                    except:
                        pass
            
        except Exception as e:
            print(f"ERROR: Error in SAHI detection: {e}")
            import traceback
            traceback.print_exc()
            self.stats['errors'] += 1
        
        return detections
    
    def _categorize_detection(self, class_name: str, confidence: float, bbox: List[int], frame) -> Optional[Dict]:
        """Categorize detections into person, tent, or object"""
        x1, y1, x2, y2 = bbox
        width = x2 - x1
        height = y2 - y1
        area = width * height
        aspect_ratio = width / height if height > 0 else 0
        
        if area < self.min_detection_area or area > self.max_detection_area:
            return None
        
        if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
            return None
        
        if confidence < self.confidence_threshold:
            return None
        
        # Person detection
        if class_name == 'person':
            return {
                'class': 'person',
                'yolo_class': class_name,
                'confidence': confidence,
                'bbox': bbox,
                'description': 'person',
                'method': f'sahi+yolo+{self.model_format_detected}',
                'area': area,
                'is_target': True
            }
        
        # Person-like objects
        person_like_classes = ['doll', 'teddy bear']
        if class_name in person_like_classes:
            return {
                'class': 'person',
                'yolo_class': class_name,
                'confidence': confidence,
                'bbox': bbox,
                'description': f'person-like ({class_name})',
                'method': f'sahi+yolo+{self.model_format_detected}',
                'area': area,
                'is_target': True
            }
        
        # Tent-like objects
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
                'method': f'sahi+yolo+{self.model_format_detected}',
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
            'method': f'sahi+yolo+{self.model_format_detected}',
            'area': area,
            'is_target': False
        }
    
    def _filter_detections(self, detections: List[Dict]) -> List[Dict]:
        """Filter detections"""
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
        """Apply NMS"""
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
        """Calculate IoU"""
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
    
    def annotate_frame(self, frame: np.ndarray, detections: List[Dict], processing_time: float) -> np.ndarray:
        """Annotate frame with detection results"""
        annotated_frame = frame.copy()
        height, width = frame.shape[:2]
        
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
            elif class_name == 'tent':
                box_color = COLOR_TENT
                label = f"TENT ({confidence:.0%})"
            else:
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), COLOR_OBJECT, 1)
                continue
            
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), box_color, 1)
            
            (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, font_thickness)
            
            padding = 4
            label_h = text_h + padding * 2
            
            if y1 - label_h >= 0:
                label_y1 = y1 - label_h
                label_y2 = y1
            else:
                label_y1 = y1
                label_y2 = y1 + label_h
            
            cv2.rectangle(
                annotated_frame,
                (x1, label_y1),
                (x1 + text_w + padding * 2, label_y2),
                box_color,
                -1
            )
            
            text_x = x1 + padding
            text_y = label_y2 - padding
            
            cv2.putText(annotated_frame, label, (text_x, text_y),
                       font, font_scale, COLOR_BLACK, font_thickness + 1, cv2.LINE_AA)
            cv2.putText(annotated_frame, label, (text_x, text_y),
                       font, font_scale, COLOR_WHITE, font_thickness, cv2.LINE_AA)
        
        num_people = sum(1 for d in detections if d['class'] == 'person')
        num_tents = sum(1 for d in detections if d['class'] == 'tent')
        num_other = len(detections) - num_people - num_tents
        
        header_font_scale = 0.6
        method_str = self.model_format_detected.upper()
        header_text = f"SAHI+YOLO ({method_str}) | {num_people} people, {num_tents} tents, {num_other} other"
        time_text = f"Time: {processing_time:.1f}s | Slices: {self.slice_height}x{self.slice_width}"
        
        overlay = annotated_frame.copy()
        cv2.rectangle(overlay, (0, 0), (width, 50), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, annotated_frame, 0.4, 0, annotated_frame)
        
        cv2.putText(annotated_frame, header_text, (10, 20),
                   font, header_font_scale, COLOR_WHITE, 1, cv2.LINE_AA)
        cv2.putText(annotated_frame, time_text, (10, 42),
                   font, header_font_scale * 0.8, (200, 200, 200), 1, cv2.LINE_AA)
        
        return annotated_frame
    
    def save_top_matches_crop(self, frame: np.ndarray, detections: List[Dict], image_path: str) -> Optional[str]:
        """Save crops of best person and tent"""
        persons = [d for d in detections if d['class'] == 'person']
        best_person = max(persons, key=lambda x: x['confidence']) if persons else None
        
        tents = [d for d in detections if d['class'] == 'tent']
        best_tent = max(tents, key=lambda x: x['confidence']) if tents else None
        
        if not best_person and not best_tent:
            return None
        
        crops = []
        labels = []
        height, width = frame.shape[:2]
        
        if best_person:
            x1, y1, x2, y2 = best_person['bbox']
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(width, x2), min(height, y2)
            if x2 > x1 and y2 > y1:
                person_crop = frame[y1:y2, x1:x2].copy()
                crops.append(person_crop)
                labels.append(f"PERSON {best_person['confidence']:.0%}")
        
        if best_tent:
            x1, y1, x2, y2 = best_tent['bbox']
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(width, x2), min(height, y2)
            if x2 > x1 and y2 > y1:
                tent_crop = frame[y1:y2, x1:x2].copy()
                crops.append(tent_crop)
                labels.append(f"TENT {best_tent['confidence']:.0%}")
        
        if not crops:
            return None
        
        target_height = 200
        resized_crops = []
        
        for crop, label in zip(crops, labels):
            h, w = crop.shape[:2]
            if h > 0:
                scale = target_height / h
                new_w = int(w * scale)
                resized = cv2.resize(crop, (new_w, target_height), interpolation=cv2.INTER_AREA)
                
                font = cv2.FONT_HERSHEY_DUPLEX
                font_scale = 0.5
                (tw, th), _ = cv2.getTextSize(label, font, font_scale, 1)
                
                label_height = th + 10
                padded = np.zeros((target_height + label_height, new_w, 3), dtype=np.uint8)
                padded[:target_height, :] = resized
                
                cv2.rectangle(padded, (0, target_height), (new_w, target_height + label_height), (40, 40, 40), -1)
                text_x = (new_w - tw) // 2
                cv2.putText(padded, label, (text_x, target_height + th + 3),
                           font, font_scale, (255, 255, 255), 1, cv2.LINE_AA)
                
                resized_crops.append(padded)
        
        if not resized_crops:
            return None
        
        separator_width = 5
        total_width = sum(c.shape[1] for c in resized_crops) + separator_width * (len(resized_crops) - 1)
        combined_height = resized_crops[0].shape[0]
        
        combined = np.zeros((combined_height, total_width, 3), dtype=np.uint8)
        x_offset = 0
        
        for i, crop in enumerate(resized_crops):
            if i > 0:
                combined[:, x_offset:x_offset + separator_width] = (80, 80, 80)
                x_offset += separator_width
            combined[:, x_offset:x_offset + crop.shape[1]] = crop
            x_offset += crop.shape[1]
        
        original_filename = os.path.basename(image_path)
        name, ext = os.path.splitext(original_filename)
        output_filename = f"TM_{name}{ext}"
        output_path = os.path.join(self.detection_results_path, output_filename)
        
        cv2.imwrite(output_path, combined)
        
        return output_path
    
    def save_results(self, annotated_frame: np.ndarray, detections: List[Dict], image_path: str):
        """Save annotated image and detection info"""
        try:
            original_filename = os.path.basename(image_path)
            name, ext = os.path.splitext(original_filename)
            output_filename = f"sahi_detected_{name}{ext}"
            output_path = os.path.join(self.detection_results_path, output_filename)
            cv2.imwrite(output_path, annotated_frame)
            
            # Save detection info as JSON
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
                'method': f'sahi+yolo+{self.model_format_detected}',
                'slice_size': f"{self.slice_height}x{self.slice_width}",
                'overlap': f"{self.overlap_height_ratio}x{self.overlap_width_ratio}",
                'detections': [
                    {
                        'class': d['class'],
                        'confidence': d['confidence'],
                        'bbox': d['bbox'],
                        'description': d['description']
                    }
                    for d in detections
                ]
            }
            
            json_filename = f"sahi_detected_{name}.json"
            json_path = os.path.join(self.detection_results_path, json_filename)
            with open(json_path, 'w') as f:
                json.dump(detection_info, f, indent=2)
            
            print(f"  Saved results -> {output_filename}")
            
        except Exception as e:
            print(f"ERROR: Error saving results: {e}")
            self.stats['errors'] += 1
    
    def _print_statistics(self):
        """Print final statistics"""
        print("\n" + "="*60)
        print("Final Statistics:")
        print(f"  Total Images Processed: {self.stats['total_images_processed']}")
        print(f"  Total Detections: {self.stats['total_detections']}")
        print(f"  Total People: {self.stats['total_people']}")
        print(f"  Total Tents: {self.stats['total_tents']}")
        print(f"  Avg Processing Time: {self.stats['avg_processing_time']:.2f}s")
        print(f"  Errors: {self.stats['errors']}")
        uptime = time.time() - self.stats['node_start_time']
        print(f"  Uptime: {uptime:.1f}s")
        print("="*60)
    
    def run(self):
        """Main run loop"""
        print("\n" + "="*60)
        print("Standalone SAHI Object Detection")
        print("="*60)
        print(f"Camera feed: {self.camera_feed_path}")
        print(f"Results: {self.detection_results_path}")
        print(f"Check interval: {self.check_interval}s")
        print("Press Ctrl+C to stop")
        print("="*60 + "\n")
        
        # Initialize model
        if not self.initialize_model():
            print("ERROR: Failed to initialize model. Exiting.")
            return
        
        print("\n✓ Ready! Monitoring for new images...\n")
        
        # Main loop
        while self.running:
            try:
                self.check_for_new_images()
                time.sleep(self.check_interval)
            except KeyboardInterrupt:
                print("\n\nKeyboard interrupt received. Shutting down...")
                break
            except Exception as e:
                print(f"ERROR: Unexpected error in main loop: {e}")
                import traceback
                traceback.print_exc()
                self.stats['errors'] += 1
        
        self._print_statistics()


def main():
    """Main entry point"""
    # Check dependencies
    if not SAHI_AVAILABLE or not YOLO_AVAILABLE:
        print("\n" + "="*80)
        print("ERROR: Missing required dependencies!")
        print("="*80)
        if not SAHI_AVAILABLE:
            print("SAHI not found. Install with: pip install sahi")
        if not YOLO_AVAILABLE:
            print("Ultralytics YOLO not found. Install with: pip install ultralytics")
        if not TORCH_AVAILABLE:
            print("PyTorch not found. Install with: pip install torch")
        print("="*80 + "\n")
        return
    
    # Create detector
    detector = ObjectDetector(
        model_path='yolo26m.pt',  # Will look in ros2_ws directory
        model_format=MODEL_FORMAT_AUTO,
        confidence_threshold=DEFAULT_CONFIDENCE_THRESHOLD,
        slice_height=DEFAULT_SLICE_SIZE,
        slice_width=DEFAULT_SLICE_SIZE,
        overlap_height_ratio=DEFAULT_OVERLAP,
        overlap_width_ratio=DEFAULT_OVERLAP,
        check_interval=DEFAULT_CHECK_INTERVAL,
        device='auto',
        max_images_per_cycle=5,
        max_camera_feed_images=100,
        enable_gpu_memory_cleanup=True,
    )
    
    # Run
    detector.run()


if __name__ == '__main__':
    main()
