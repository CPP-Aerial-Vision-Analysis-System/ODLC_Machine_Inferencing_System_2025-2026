#!/usr/bin/env python3

"""
Real Object Detection Node using YOLO and MobileNet
Detects objects in images from camera_feed folder and shows results with bounding boxes
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import logging
from datetime import datetime
import os
from ament_index_python.packages import get_package_share_directory
from PIL import Image

# Try to import YOLO and MobileNet dependencies
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("Warning: ultralytics not available, YOLO detection will be disabled")

try:
    import tensorflow as tf
    from tensorflow.keras.applications import MobileNetV3Large
    from tensorflow.keras.applications.mobilenet_v3 import preprocess_input, decode_predictions
    from tensorflow.keras.preprocessing import image
    MOBILENET_AVAILABLE = True
except ImportError:
    MOBILENET_AVAILABLE = False
    print("Warning: tensorflow not available, MobileNet detection will be disabled")

try:
    from sahi import AutoDetectionModel
    from sahi.predict import get_sliced_prediction
    SAHI_AVAILABLE = True
except ImportError:
    SAHI_AVAILABLE = False
    print("Warning: sahi not available, SAHI slicing will be disabled")

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # ROS2 setup
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/detection_results', 10)
        self.detection_publisher = self.create_publisher(String, '/detection_info', 10)
        
        # Get camera_feed directory path
        self.camera_feed_path = os.path.join(
            get_package_share_directory("video_cam"), 
            "camera_feed"
        )
        
        # Get detection_results directory path
        self.detection_results_path = os.path.join(
            get_package_share_directory("video_cam"), 
            "detection_results"
        )
        
        # Create detection_results directory if it doesn't exist
        os.makedirs(self.detection_results_path, exist_ok=True)
        
        self.get_logger().info(f"Object Detection Node - Monitoring: {self.camera_feed_path}")
        self.get_logger().info(f"Detection results will be saved to: {self.detection_results_path}")
        
        # Initialize models
        self.initialize_models()
        
        # Processing state
        self.processed_images = set()  # Track processed images
        
        # Timer to check for new images
        self.timer = self.create_timer(2.0, self.check_for_new_images)
        
        self.get_logger().info("Object Detection Node initialized - using YOLO and MobileNet for object detection")
    
    def initialize_models(self):
        """Initialize YOLO, MobileNet, and SAHI models"""
        try:
            # Initialize SAHI detection model
            if SAHI_AVAILABLE and YOLO_AVAILABLE:
                self.sahi_model = AutoDetectionModel.from_pretrained(
                    model_type='yolov8',
                    model_path='yolo11s.pt',
                    confidence_threshold=0.3,
                    device='cpu'  # Use CPU for compatibility
                )
                self.get_logger().info("SAHI YOLOv11s model loaded successfully")
            else:
                self.sahi_model = None
                self.get_logger().warn("SAHI not available, using direct YOLO")
            
            # Initialize YOLO model (fallback)
            if YOLO_AVAILABLE:
                self.yolo_model = YOLO('yolo11s.pt')  # Load YOLOv11 small model
                self.get_logger().info("YOLOv11s model loaded successfully")
            else:
                self.yolo_model = None
                self.get_logger().warn("YOLO not available, using OpenCV fallback")
            
            # Initialize MobileNet model
            if MOBILENET_AVAILABLE:
                self.mobilenet_model = MobileNetV3Large(weights='imagenet')
                self.get_logger().info("MobileNetV3Large model loaded successfully")
            else:
                self.mobilenet_model = None
                self.get_logger().warn("MobileNet not available, using OpenCV fallback")
            
            # Initialize OpenCV fallback detectors
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            
            self.get_logger().info("Models initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize models: {e}")
            # Fallback to OpenCV only
            self.sahi_model = None
            self.yolo_model = None
            self.mobilenet_model = None
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            self.get_logger().warn("Falling back to OpenCV-only detection")
    
    def check_for_new_images(self):
        """Check for new images in camera_feed folder"""
        try:
            if not os.path.exists(self.camera_feed_path):
                self.get_logger().warn(f"Camera feed path does not exist: {self.camera_feed_path}")
                return
            
            # Get all image files
            image_files = []
            for file in os.listdir(self.camera_feed_path):
                if file.lower().endswith(('.jpg', '.jpeg', '.png')):
                    image_files.append(file)
            
            self.get_logger().info(f"Found {len(image_files)} total images, {len(self.processed_images)} already processed")
            self.get_logger().info(f"Image files: {image_files}")
            self.get_logger().info(f"Processed files: {list(self.processed_images)}")
            
            # Process new images
            new_images_processed = 0
            for image_file in image_files:
                if image_file not in self.processed_images:
                    self.get_logger().info(f"Processing new image: {image_file}")
                    image_path = os.path.join(self.camera_feed_path, image_file)
                    self.process_image(image_path)
                    self.processed_images.add(image_file)
                    new_images_processed += 1
                else:
                    self.get_logger().debug(f"Skipping already processed image: {image_file}")
            
            if new_images_processed > 0:
                self.get_logger().info(f"Processed {new_images_processed} new images")
            else:
                self.get_logger().debug("No new images to process")
                    
        except Exception as e:
            self.get_logger().error(f"Error checking for new images: {e}")
    
    def process_image(self, image_path):
        """Process a single image for object detection"""
        try:
            # Load image
            frame = cv2.imread(image_path)
            if frame is None:
                self.get_logger().warn(f"Could not load image: {image_path}")
                return
            
            self.get_logger().info(f"Processing image: {os.path.basename(image_path)}")
            
            # Run real object detection
            detections = self.detect_objects(frame)
            
            # Create annotated frame
            annotated_frame = self.annotate_frame(frame, detections)
            
            # Publish results
            self.publish_results(annotated_frame, detections, image_path)
                
        except Exception as e:
            self.get_logger().error(f"Error processing image {image_path}: {e}")
    
    def detect_objects(self, frame):
        """Detect objects using SAHI + YOLO + MobileNet with OpenCV fallback"""
        detections = []
        
        try:
            # Try SAHI detection first (best for small objects)
            if self.sahi_model is not None:
                sahi_detections = self.detect_with_sahi(frame)
                detections.extend(sahi_detections)
                self.get_logger().info(f"SAHI detected {len(sahi_detections)} objects")
            
            # If no SAHI detections, try direct YOLO
            if len(detections) == 0 and self.yolo_model is not None:
                yolo_detections = self.detect_with_yolo(frame)
                detections.extend(yolo_detections)
                self.get_logger().info(f"YOLO detected {len(yolo_detections)} objects")
            
            # Try MobileNet detection for additional validation
            if self.mobilenet_model is not None and len(detections) > 0:
                detections = self.validate_with_mobilenet(frame, detections)
                self.get_logger().info(f"After MobileNet validation: {len(detections)} objects")
            
            # If no ML detections, fall back to OpenCV
            if len(detections) == 0:
                self.get_logger().info("No ML detections, using OpenCV fallback")
                people_detections = self.detect_people_opencv(frame)
                tent_detections = self.detect_tents_opencv(frame)
                detections.extend(people_detections)
                detections.extend(tent_detections)
                self.get_logger().info(f"OpenCV detected {len(detections)} objects")
            
            # Apply non-maximum suppression to remove overlapping detections
            detections = self._apply_nms(detections)
            
            self.get_logger().info(f"Found {len(detections)} objects: {len([d for d in detections if d['class'] == 'person'])} people, {len([d for d in detections if d['class'] == 'tent'])} tents")
            
        except Exception as e:
            self.get_logger().error(f"Error in object detection: {e}")
        
        return detections
    
    def detect_with_sahi(self, frame):
        """Detect objects using SAHI (Slicing Aided Hyper Inference)"""
        detections = []
        
        try:
            # Convert frame to PIL Image for SAHI
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(frame_rgb)
            
            # Run SAHI prediction with slicing
            result = get_sliced_prediction(
                pil_image,
                self.sahi_model,
                slice_height=640,
                slice_width=640,
                overlap_height_ratio=0.2,
                overlap_width_ratio=0.2,
                postprocess_type="NMS",
                postprocess_match_metric="IOS",
                postprocess_match_threshold=0.5,
                postprocess_class_agnostic=False,
                verbose=0
            )
            
            # Convert SAHI results to our format
            for object_prediction in result.object_prediction_list:
                # Get bounding box
                bbox = object_prediction.bbox
                x1, y1, x2, y2 = bbox.minx, bbox.miny, bbox.maxx, bbox.maxy
                
                # Get class and confidence
                class_name = object_prediction.category.name
                confidence = object_prediction.score.value
                
                # Filter for person and tent-like objects
                if class_name == 'person' and confidence > 0.4:
                    detections.append({
                        'class': 'person',
                        'confidence': float(confidence),
                        'bbox': [int(x1), int(y1), int(x2), int(y2)],
                        'description': 'person/mannequin',
                        'mobilenet_class': 'person',
                        'mobilenet_confidence': float(confidence),
                        'method': 'sahi'
                    })
                elif class_name in ['backpack', 'suitcase', 'sports ball', 'umbrella', 'handbag', 'tie'] and confidence > 0.3:
                    # These could be tent-like objects
                    detections.append({
                        'class': 'tent',
                        'confidence': float(confidence),
                        'bbox': [int(x1), int(y1), int(x2), int(y2)],
                        'description': 'tent-like object',
                        'mobilenet_class': 'tent',
                        'mobilenet_confidence': float(confidence),
                        'method': 'sahi'
                    })
            
        except Exception as e:
            self.get_logger().error(f"Error in SAHI detection: {e}")
        
        return detections
    
    def detect_with_yolo(self, frame):
        """Detect objects using YOLO"""
        detections = []
        
        try:
            # Run YOLO inference
            results = self.yolo_model(frame)
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Get bounding box coordinates
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        confidence = box.conf[0].cpu().numpy()
                        class_id = int(box.cls[0].cpu().numpy())
                        class_name = self.yolo_model.names[class_id]
                        
                        # Filter for person and tent-like objects
                        if class_name == 'person' and confidence > 0.5:
                            detections.append({
                                'class': 'person',
                                'confidence': float(confidence),
                                'bbox': [int(x1), int(y1), int(x2), int(y2)],
                                'description': 'person/mannequin',
                                'mobilenet_class': 'person',
                                'mobilenet_confidence': float(confidence),
                                'method': 'yolo11s'
                            })
                        elif class_name in ['backpack', 'suitcase', 'sports ball', 'umbrella', 'handbag', 'tie'] and confidence > 0.3:
                            # These could be tent-like objects
                            detections.append({
                                'class': 'tent',
                                'confidence': float(confidence),
                                'bbox': [int(x1), int(y1), int(x2), int(y2)],
                                'description': 'tent-like object',
                                'mobilenet_class': 'tent',
                                'mobilenet_confidence': float(confidence),
                                'method': 'yolo11s'
                            })
            
        except Exception as e:
            self.get_logger().error(f"Error in YOLO detection: {e}")
        
        return detections
    
    def validate_with_mobilenet(self, frame, detections):
        """Validate detections using MobileNet"""
        validated_detections = []
        
        try:
            for detection in detections:
                x1, y1, x2, y2 = detection['bbox']
                
                # Extract ROI
                roi = frame[y1:y2, x1:x2]
                if roi.size == 0:
                    continue
                
                # Resize for MobileNetV3 (224x224 input size)
                roi_resized = cv2.resize(roi, (224, 224))
                roi_rgb = cv2.cvtColor(roi_resized, cv2.COLOR_BGR2RGB)
                roi_array = np.expand_dims(roi_rgb, axis=0)
                roi_preprocessed = preprocess_input(roi_array)
                
                # Get MobileNet predictions
                predictions = self.mobilenet_model.predict(roi_preprocessed)
                decoded_predictions = decode_predictions(predictions, top=3)[0]
                
                # Check if MobileNet agrees with YOLO
                mobilenet_class = decoded_predictions[0][1]
                mobilenet_confidence = float(decoded_predictions[0][2])
                
                # Update detection with MobileNet info
                detection['mobilenet_class'] = mobilenet_class
                detection['mobilenet_confidence'] = mobilenet_confidence
                
                # Only keep if MobileNet confidence is reasonable
                if mobilenet_confidence > 0.1:
                    validated_detections.append(detection)
            
        except Exception as e:
            self.get_logger().error(f"Error in MobileNet validation: {e}")
            return detections  # Return original detections if validation fails
        
        return validated_detections
    
    def _apply_nms(self, detections, overlap_threshold=0.3):
        """Apply Non-Maximum Suppression to remove overlapping detections"""
        if len(detections) == 0:
            return detections
        
        # Sort detections by confidence (highest first)
        detections = sorted(detections, key=lambda x: x['confidence'], reverse=True)
        
        keep = []
        while detections:
            # Take the detection with highest confidence
            current = detections.pop(0)
            keep.append(current)
            
            # Remove detections that overlap significantly with current
            remaining = []
            for detection in detections:
                if self._calculate_iou(current['bbox'], detection['bbox']) < overlap_threshold:
                    remaining.append(detection)
            detections = remaining
        
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
    
    def detect_people_opencv(self, frame):
        """Detect people using HOG descriptor with improved filtering"""
        detections = []
        
        try:
            # Convert to grayscale for better HOG detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Apply Gaussian blur to reduce noise
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Resize frame for better detection
            height, width = blurred.shape[:2]
            if width > 800:
                scale = 800.0 / width
                new_width = int(width * scale)
                new_height = int(height * scale)
                resized_frame = cv2.resize(blurred, (new_width, new_height))
            else:
                resized_frame = blurred.copy()
                scale = 1.0
            
            # Detect people with stricter parameters
            (rects, weights) = self.hog.detectMultiScale(
                resized_frame,
                winStride=(8, 8),  # Larger stride to reduce false positives
                padding=(16, 16),  # More padding
                scale=1.1,  # Larger scale steps
                hitThreshold=0.0,
                finalThreshold=3  # Higher final threshold
            )
            
            # Convert back to original scale and apply balanced filtering
            for i, (x, y, w, h) in enumerate(rects):
                if weights[i] > 0.4:  # Balanced confidence threshold
                    # Scale back to original image size
                    x = int(x / scale)
                    y = int(y / scale)
                    w = int(w / scale)
                    h = int(h / scale)
                    
                    # Additional validation: check aspect ratio and size
                    aspect_ratio = w / h
                    area = w * h
                    
                    # Person should be roughly 1.5-5 times taller than wide
                    # and have reasonable size (not too small or too large)
                    if (0.2 < aspect_ratio < 0.8 and 
                        area > 2000 and area < 200000 and
                        h > 60 and w > 30):  # More lenient size requirements
                        
                        # Check if this region looks like a person (not text/drawing)
                        roi = frame[y:y+h, x:x+w]
                        if self._is_likely_person(roi):
                            detections.append({
                                'class': 'person',
                                'confidence': float(weights[i]),
                                'bbox': [x, y, x + w, y + h],
                                'description': 'person/mannequin',
                                'mobilenet_class': 'person',
                                'mobilenet_confidence': float(weights[i])
                            })
            
        except Exception as e:
            self.get_logger().error(f"Error detecting people: {e}")
        
        return detections
    
    def _is_likely_person(self, roi):
        """Check if ROI is likely a person (not text or drawing)"""
        try:
            if roi.size == 0:
                return False
            
            # Convert to grayscale
            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            
            # Check for text-like features (high contrast, sharp edges)
            edges = cv2.Canny(gray_roi, 50, 150)
            edge_density = np.sum(edges > 0) / edges.size
            
            # Check for uniform color (drawings often have uniform colors)
            std_dev = np.std(gray_roi)
            
            # More lenient criteria - person should have moderate edge density and color variation
            # Text has very high edge density, drawings have low variation
            return 0.02 < edge_density < 0.5 and std_dev > 10
            
        except Exception:
            return False
    
    def detect_tents_opencv(self, frame):
        """Detect tent-like objects using improved color and shape analysis"""
        detections = []
        
        try:
            # Convert to HSV for better color detection
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Define range for tent-like colors (orange, red, blue, green, gray/white for metal roofs)
            # Orange tent detection
            lower_orange = np.array([5, 50, 50])  # More lenient thresholds
            upper_orange = np.array([15, 255, 255])
            mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
            
            # Red tent detection
            lower_red1 = np.array([0, 50, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 50, 50])
            upper_red2 = np.array([180, 255, 255])
            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = mask_red1 + mask_red2
            
            # Blue tent detection
            lower_blue = np.array([100, 50, 50])
            upper_blue = np.array([130, 255, 255])
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
            
            # Green tent detection
            lower_green = np.array([40, 50, 50])
            upper_green = np.array([80, 255, 255])
            mask_green = cv2.inRange(hsv, lower_green, upper_green)
            
            # Gray/white metal roof detection (for corrugated metal)
            lower_gray = np.array([0, 0, 100])  # Low saturation, high value
            upper_gray = np.array([180, 30, 255])
            mask_gray = cv2.inRange(hsv, lower_gray, upper_gray)
            
            # Combine all tent color masks
            mask_tent = mask_orange + mask_red + mask_blue + mask_green + mask_gray
            
            # Apply morphological operations to clean up the mask
            kernel = np.ones((5,5), np.uint8)
            mask_tent = cv2.morphologyEx(mask_tent, cv2.MORPH_CLOSE, kernel)
            mask_tent = cv2.morphologyEx(mask_tent, cv2.MORPH_OPEN, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask_tent, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                if area > 5000:  # More lenient minimum area threshold
                    # Get bounding rectangle
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Calculate confidence based on area and aspect ratio
                    aspect_ratio = w / h
                    area_ratio = area / (w * h)  # How much of the bounding box is filled
                    
                    # Tent should have reasonable aspect ratio and fill most of its bounding box
                    if (0.2 < aspect_ratio < 4.0 and 
                        area_ratio > 0.2 and  # At least 20% of bounding box filled
                        w > 50 and h > 40):  # More lenient size requirements
                        
                        # Additional validation: check if it looks like a tent
                        roi = frame[y:y+h, x:x+w]
                        is_tent = self._is_likely_tent(roi, mask_tent[y:y+h, x:x+w])
                        
                        if is_tent:
                            confidence = min(0.9, area / 50000.0)  # Normalize confidence
                            
                            detections.append({
                                'class': 'tent',
                                'confidence': confidence,
                                'bbox': [x, y, x + w, y + h],
                                'description': 'tent',
                                'mobilenet_class': 'tent',
                                'mobilenet_confidence': confidence
                            })
            
        except Exception as e:
            self.get_logger().error(f"Error detecting tents: {e}")
        
        return detections
    
    def _is_likely_tent(self, roi, mask_roi):
        """Check if ROI is likely a tent (not text or drawing)"""
        try:
            if roi.size == 0 or mask_roi.size == 0:
                return False
            
            # Check color consistency in the masked region
            masked_pixels = roi[mask_roi > 0]
            if len(masked_pixels) == 0:
                return False
            
            # Calculate color variance - tents should have some color variation
            color_std = np.std(masked_pixels, axis=0)
            avg_color_std = np.mean(color_std)
            
            # Check if the shape is roughly triangular or rectangular (tent-like)
            contours, _ = cv2.findContours(mask_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                return False
            
            # Get the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Approximate the contour to see if it's roughly tent-shaped
            epsilon = 0.02 * cv2.arcLength(largest_contour, True)
            approx = cv2.approxPolyDP(largest_contour, epsilon, True)
            
            # More lenient criteria - tent should have 3-10 vertices (triangular to decagonal)
            # and reasonable color variation (not uniform like text)
            return (3 <= len(approx) <= 10 and 
                    avg_color_std > 8 and 
                    cv2.contourArea(largest_contour) > 2000)
            
        except Exception:
            return False
    
    def annotate_frame(self, frame, detections):
        """Annotate frame with detection results"""
        annotated_frame = frame.copy()
        
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            class_name = detection['class']
            confidence = detection['confidence']
            description = detection.get('description', class_name)
            mobilenet_confidence = detection.get('mobilenet_confidence', confidence)
            
            # Choose color based on class
            if class_name == 'person':
                box_color = (0, 255, 0)  # Green for person/mannequin
                text_color = (0, 0, 0)   # Black text
                target_label = "TARGET: PERSON"
            elif class_name == 'tent':
                box_color = (0, 255, 255)  # Yellow for tent
                text_color = (0, 0, 0)     # Black text
                target_label = "TARGET: TENT"
            else:
                box_color = (128, 128, 128)  # Grey for other objects
                text_color = (0, 0, 255)     # Red text
                target_label = f"NON-TARGET: {class_name.upper()}"
            
            # Draw bounding box
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), box_color, 3)
            
            # Draw labels with background
            method = detection.get('method', 'opencv')
            yolo_label = f"{method.upper()}: {class_name} ({confidence:.2f})"
            mobilenet_label = f"MobileNet: {description} ({mobilenet_confidence:.2f})"
            
            # Background for text
            text_bg_height = 80
            cv2.rectangle(annotated_frame, (x1, y1-text_bg_height), (x2, y1), box_color, -1)
            
            # Draw text with appropriate colors
            cv2.putText(annotated_frame, target_label, (x1+5, y1-65), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
            cv2.putText(annotated_frame, yolo_label, (x1+5, y1-40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)
            cv2.putText(annotated_frame, mobilenet_label, (x1+5, y1-20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)
        
        # Add title and legend
        title = f"Real Object Detection - {len(detections)} objects detected"
        cv2.putText(annotated_frame, title, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(annotated_frame, title, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 1)
        
        # Add legend
        legend = "GREEN: Target Objects (Person) | YELLOW: Target Objects (Tent) | GREY: Other Objects"
        cv2.putText(annotated_frame, legend, (10, frame.shape[0] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(annotated_frame, legend, (10, frame.shape[0] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
        
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
            
            # Save annotated image to detection_results folder
            original_filename = os.path.basename(image_path)
            name, ext = os.path.splitext(original_filename)
            output_filename = f"detected_{name}{ext}"
            output_path = os.path.join(self.detection_results_path, output_filename)
            cv2.imwrite(output_path, annotated_frame)
            
            # Create detection info message
            detection_info = {
                'image': os.path.basename(image_path),
                'timestamp': datetime.now().isoformat(),
                'detections': len(detections),
                'saved_to': output_path,
                'objects': [
                    {
                        'yolo_class': d['class'], 
                        'yolo_confidence': d['confidence'],
                        'mobilenet_class': d.get('mobilenet_class', d['class']),
                        'mobilenet_confidence': d.get('mobilenet_confidence', d['confidence']),
                        'description': d.get('description', d['class'])
                    } 
                    for d in detections
                ]
            }
            
            # Publish detection info
            info_msg = String()
            info_msg.data = str(detection_info)
            self.detection_publisher.publish(info_msg)
            
            self.get_logger().info(
                f"Published and saved results for {os.path.basename(image_path)}: "
                f"{len(detections)} objects detected -> {output_filename}"
            )
            
        except Exception as e:
            self.get_logger().error(f"Error publishing/saving results: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
