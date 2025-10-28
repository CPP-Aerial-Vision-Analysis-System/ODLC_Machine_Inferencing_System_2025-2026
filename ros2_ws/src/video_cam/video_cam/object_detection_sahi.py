#!/usr/bin/env python3

"""
SAHI Object Detection Node using YOLO for Small Object Detection
Specifically optimized for detecting small tents and people in aerial imagery
Uses Slicing Aided Hyper Inference (SAHI) for improved small object detection

Current Architecture:
- SAHI: Slices images and manages detection pipeline
- YOLO: Performs actual object detection on each slice
- MobileNet: NOT IMPLEMENTED (commented out for future use)

Configuration:
- Slice size: 512x512 (optimized for small object detection)
- Overlap: 30% (ensures objects at boundaries are detected)
- Balance: Accuracy over speed for critical small object detection

TODO: Implement MobileNet validation for additional accuracy
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from datetime import datetime
import os
import platform
from ament_index_python.packages import get_package_share_directory

# Import SAHI and YOLO dependencies
try:
    from sahi import AutoDetectionModel
    from sahi.predict import get_sliced_prediction
    from sahi.utils.cv import read_image
    SAHI_AVAILABLE = True
except ImportError:
    SAHI_AVAILABLE = False
    print("ERROR: sahi not available. Please install with: pip install sahi")

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    print("WARNING: torch not available, will use CPU only")

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("ERROR: ultralytics not available. Please install with: pip install ultralytics")


class SAHIObjectDetectionNode(Node):
    """
    ROS2 Node for SAHI-based object detection optimized for small objects.
    
    This node monitors a camera_feed directory for new images and processes them using
    YOLO with SAHI (Slicing Aided Hyper Inference) to detect small objects like tents
    and people in aerial/drone imagery.
    """
    
    def __init__(self):
        super().__init__('sahi_object_detection_node')
        
        # Declare parameters
        self.declare_parameter('model_path', 'yolo11s.pt')
        self.declare_parameter('confidence_threshold', 0.15)
        self.declare_parameter('slice_height', 512)  # Reverted to original size for better accuracy
        self.declare_parameter('slice_width', 512)   # Reverted to original size for better accuracy
        self.declare_parameter('overlap_height_ratio', 0.3)  # Reverted to original overlap
        self.declare_parameter('overlap_width_ratio', 0.3)   # Reverted to original overlap
        self.declare_parameter('check_interval', 2.0)
        self.declare_parameter('device', 'auto')
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.slice_height = self.get_parameter('slice_height').value
        self.slice_width = self.get_parameter('slice_width').value
        self.overlap_height_ratio = self.get_parameter('overlap_height_ratio').value
        self.overlap_width_ratio = self.get_parameter('overlap_width_ratio').value
        self.check_interval = self.get_parameter('check_interval').value
        self.device = self.get_parameter('device').value
        
        # ROS2 setup
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/sahi_detection_results', 10)
        self.detection_publisher = self.create_publisher(String, '/sahi_detection_info', 10)
        
        # Get camera_feed directory path
        self.camera_feed_path = os.path.join(
            get_package_share_directory("video_cam"), 
            "camera_feed"
        )
        
        # Get detection_results directory path
        self.detection_results_path = os.path.join(
            get_package_share_directory("video_cam"), 
            "detection_results_sahi"
        )
        
        # Create detection_results directory if it doesn't exist
        os.makedirs(self.detection_results_path, exist_ok=True)
        
        self.get_logger().info(f"SAHI Object Detection Node - Monitoring: {self.camera_feed_path}")
        self.get_logger().info(f"Detection results will be saved to: {self.detection_results_path}")
        
        # Auto-detect device
        if self.device == 'auto':
            self.device = self._get_device()
        
        self.get_logger().info(f"Using device: {self.device}")
        
        # Initialize SAHI model
        self.detection_model = None
        self.initialize_sahi_model()
        
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
        self.get_logger().info("="*80)
    
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
            
            # Check if model file exists, if not, check in ros2_ws directory
            model_path = self.model_path
            if not os.path.exists(model_path):
                # Try ros2_ws directory
                alt_path = os.path.join('/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws', self.model_path)
                if os.path.exists(alt_path):
                    model_path = alt_path
                    self.get_logger().info(f"Using model from: {model_path}")
                else:
                    self.get_logger().warn(f"Model file not found at {self.model_path}, will download from Ultralytics")
            
            self.get_logger().info("Loading SAHI YOLOv11 model for small object detection...")
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
            self.detection_model = AutoDetectionModel.from_pretrained(
                model_type='yolov8',  # Use yolov8 as model type for YOLO v8+ models
                model_path=model_path,
                confidence_threshold=self.confidence_threshold,
                device=self.device,
            )
            
            self.get_logger().info("✓ SAHI YOLOv11 model loaded successfully!")
            
            # Verify model is on correct device
            if TORCH_AVAILABLE and hasattr(self.detection_model, 'model'):
                try:
                    model_device = next(self.detection_model.model.model.parameters()).device
                    self.get_logger().info(f"✓ Model confirmed on device: {model_device}")
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
    
    def check_for_new_images(self):
        """Check for new images in camera_feed folder"""
        try:
            if not os.path.exists(self.camera_feed_path):
                self.get_logger().warn(f"Camera feed path does not exist: {self.camera_feed_path}")
                return
            
            if self.detection_model is None:
                self.get_logger().warn("SAHI model not initialized, skipping detection")
                return
            
            # Get all image files
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
                self.get_logger().info(f"✓ Processed {new_images_processed} new images")
                self._log_statistics()
                    
        except Exception as e:
            self.get_logger().error(f"Error checking for new images: {e}")
    
    def process_image(self, image_path):
        """Process a single image using SAHI for small object detection"""
        try:
            start_time = time.time()
            
            # Load image
            frame = cv2.imread(image_path)
            if frame is None:
                self.get_logger().warn(f"Could not load image: {image_path}")
                return
            
            height, width = frame.shape[:2]
            self.get_logger().info(f"Processing image: {os.path.basename(image_path)} ({width}x{height})")
            
            # Run SAHI prediction
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
            # Convert BGR to RGB for SAHI
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
                postprocess_type="NMS",  # Non-Maximum Suppression
                postprocess_match_metric="IOS",  # Intersection Over Smaller area
                postprocess_match_threshold=0.5,  # Threshold for merging detections
                postprocess_class_agnostic=False,  # Class-aware NMS
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
            
        except Exception as e:
            self.get_logger().error(f"Error in SAHI detection: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        
        return detections
    
    def _categorize_detection(self, class_name, confidence, bbox, frame):
        """
        Categorize YOLO detections into our target classes (person, tent)
        
        Args:
            class_name: YOLO class name
            confidence: Detection confidence
            bbox: Bounding box [x1, y1, x2, y2]
            frame: Original image frame
            
        Returns:
            Detection dict or None if not a target class
        """
        x1, y1, x2, y2 = bbox
        
        # Person detection
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
        
        # Tent detection - map various YOLO classes that could be tents
        # In aerial/drone imagery, tents might be detected as various objects
        tent_like_classes = {
            'backpack': 0.20,      # Tents often detected as backpacks
            'suitcase': 0.20,      # Or suitcases
            'umbrella': 0.20,      # Tent canopies might look like umbrellas
            'handbag': 0.15,       # Small tents
            'car': 0.15,           # Large tents might look like cars from above
            'truck': 0.15,         # Very large tents
            'boat': 0.20,          # Boat-shaped tents
            'sports ball': 0.15,   # Small rounded tents
            'kite': 0.20,          # Tent fabric might look like kites
            'surfboard': 0.15,     # Elongated tents
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
            offency_label = f"OFFENCY: {yolo_class} ({confidence:.2f})"
            # TODO: Implement actual MobileNet validation
            # mobilenet_label = f"MobileNet: {yolo_class} ({confidence:.2f})"
            
            # Calculate text size for background
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            thickness = 1
            
            (w1, h1), _ = cv2.getTextSize(target_label, font, font_scale, thickness)
            (w2, h2), _ = cv2.getTextSize(offency_label, font, font_scale, thickness)
            # (w3, h3), _ = cv2.getTextSize(mobilenet_label, font, font_scale, thickness)
            
            # Calculate background rectangle size
            max_width = max(w1, w2) + 10  # Removed w3 for MobileNet
            total_height = h1 + h2 + 15  # Reduced height since no MobileNet label
            
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
            cv2.putText(annotated_frame, offency_label, (x1 + 5, text_y),
                       font, font_scale, COLOR_TEXT_BG, 2)
            
            # TODO: Add MobileNet validation label when implemented
            # text_y += h3 + 5
            # cv2.putText(annotated_frame, mobilenet_label, (x1 + 5, text_y),
            #            font, font_scale, COLOR_TEXT_BG, 2)
        
        # Add header with detection info
        header_text = f"SAHI Object Detection - {len(detections)} objects detected"
        cv2.putText(annotated_frame, header_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, COLOR_TEXT, 2)
        
        # Add processing time
        time_text = f"Processing Time: {processing_time:.2f}s"
        cv2.putText(annotated_frame, time_text, (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_TEXT, 2)
        
        # Add SAHI mode indicator at bottom
        sahi_text = f"SAHI Mode: {self.slice_height}x{self.slice_width} slices, {self.overlap_height_ratio:.0%} overlap"
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
            detection_info = {
                'image': os.path.basename(image_path),
                'timestamp': datetime.now().isoformat(),
                'detections': len(detections),
                'saved_to': output_path,
                'method': 'sahi+yolo11s',
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