#!/usr/bin/env python3
"""SIYI A8 Mini ROS2 Node - Refactored"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, String
from mavros_msgs.msg import StatusText
from rclpy.qos import qos_profile_sensor_data

import os
import cv2
import time
import numpy as np
from threading import Lock, Event, Thread
from typing import Optional

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except Exception as e:
    print(f"Warning: cv_bridge import failed: {e}")
    CV_BRIDGE_AVAILABLE = False

from .config import (
    STREAM_LOOP_PERIOD,
    DEFAULT_USE_REAL_CAMERA,
    DEFAULT_MIN_ALTITUDE_AGL,
    DEFAULT_CAMERA_IP,
    DEFAULT_CTRL_PORT,
    DEFAULT_MEDIA_PORT,
    DEFAULT_RTSP_PORT,
    DEFAULT_HTTP_TIMEOUT,
    DEFAULT_CAPTURE_TIMEOUT,
    DEFAULT_MIN_FREE_SPACE_MB,
    TOPIC_IMAGE_RAW,
    TOPIC_MAVROS_STATUS,
    TOPIC_CAMERA_STATUS,
    TOPIC_DISK_STATUS,
    TOPIC_CAMERA_TRIGGER,
    TOPIC_SET_RESOLUTION,
    TOPIC_ALTITUDE,
    TOPIC_SIM_IMAGE,
    QUEUE_SIZE_DEFAULT,
    QUEUE_SIZE_IMAGE,
    MAVROS_SEVERITY_INFO,
    PHOTO_RESOLUTIONS,
)
from .camera_interface import CameraInterface
from .storage_manager import StorageManager
from .pipeline_orchestrator import PipelineOrchestrator, PipelineError


class SIYINode(Node):
    """ROS2 node wrapper for SIYI camera pipeline"""
    
    def __init__(self):
        super().__init__('siyi_unified_pipeline')
        
        # ROS Parameters
        self._declare_parameters()
        self._load_parameters()
        
        # ROS Publishers
        self.image_pub = self.create_publisher(
            Image, TOPIC_IMAGE_RAW, QUEUE_SIZE_DEFAULT)
        self.status_pub = self.create_publisher(
            StatusText, TOPIC_MAVROS_STATUS, QUEUE_SIZE_DEFAULT)
        self.camera_status_pub = self.create_publisher(
            String, TOPIC_CAMERA_STATUS, QUEUE_SIZE_DEFAULT)
        self.disk_status_pub = self.create_publisher(
            Float64, TOPIC_DISK_STATUS, QUEUE_SIZE_DEFAULT)
        
        # ROS Subscribers
        self.create_subscription(
            Bool, TOPIC_CAMERA_TRIGGER, self.camera_trigger_callback, QUEUE_SIZE_DEFAULT)
        self.create_subscription(
            String, TOPIC_SET_RESOLUTION, self.set_resolution_callback, QUEUE_SIZE_DEFAULT)
        self.create_subscription(
            Float64, TOPIC_ALTITUDE, self.altitude_callback, qos_profile_sensor_data)
        
        # Simulation mode subscriber
        if not self.use_real_camera:
            self.create_subscription(
                Image, TOPIC_SIM_IMAGE, self.sim_image_callback, QUEUE_SIZE_IMAGE)
        
        # State
        self.camera_enabled = True
        self.config_lock = Lock()
        self.capture_requested = Event()
        
        # Simulation mode
        self.latest_image_msg: Optional[Image] = None
        
        # CV Bridge
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warn("cv_bridge not available, using alternative conversion")
        
        # Initialize components
        self._initialize_components()
        
        # Start main loop
        self.pipeline_timer = self.create_timer(STREAM_LOOP_PERIOD, self._pipeline_loop)
        
        self._log_initialization_complete()
    
    def _declare_parameters(self):
        """Declare all ROS parameters"""
        self.declare_parameter('use_real_camera', DEFAULT_USE_REAL_CAMERA)
        self.declare_parameter('min_altitude_agl', DEFAULT_MIN_ALTITUDE_AGL)
        self.declare_parameter('camera_ip', DEFAULT_CAMERA_IP)
        self.declare_parameter('ctrl_port', DEFAULT_CTRL_PORT)
        self.declare_parameter('media_port', DEFAULT_MEDIA_PORT)
        self.declare_parameter('rtsp_port', DEFAULT_RTSP_PORT)
        self.declare_parameter('http_timeout_sec', DEFAULT_HTTP_TIMEOUT)
        self.declare_parameter('capture_timeout_sec', DEFAULT_CAPTURE_TIMEOUT)
        self.declare_parameter('min_free_space_mb', DEFAULT_MIN_FREE_SPACE_MB)
    
    def _load_parameters(self):
        """Load parameter values"""
        self.use_real_camera = self.get_parameter('use_real_camera').value
        self.altitude_threshold = self.get_parameter('min_altitude_agl').value
        self.camera_ip = self.get_parameter('camera_ip').value
        self.ctrl_port = self.get_parameter('ctrl_port').value
        self.media_port = self.get_parameter('media_port').value
        self.rtsp_port = self.get_parameter('rtsp_port').value
        self.http_timeout = self.get_parameter('http_timeout_sec').value
        self.capture_timeout = self.get_parameter('capture_timeout_sec').value
        self.min_free_space_mb = self.get_parameter('min_free_space_mb').value
    
    def _initialize_components(self):
        """Initialize camera, storage, and pipeline components"""
        # Find workspace root
        workspace_root = self._find_ros2_workspace()
        
        # Initialize storage manager
        self.storage = StorageManager(workspace_root, logger=self.get_logger())
        
        if self.use_real_camera:
            # Initialize camera interface
            self.camera = CameraInterface(
                camera_ip=self.camera_ip,
                ctrl_port=self.ctrl_port,
                media_port=self.media_port,
                rtsp_port=self.rtsp_port,
                http_timeout=self.http_timeout,
                logger=self.get_logger()
            )
            
            # Initialize pipeline orchestrator
            self.pipeline = PipelineOrchestrator(
                camera=self.camera,
                storage=self.storage,
                logger=self.get_logger()
            )
            
            # RTSP video stream disabled - not needed for capture/save/detect workflow
            # Only needed for live video preview during flight
            # if self.camera.connect_video_stream():
            #     self._send_status("Real camera initialized")
            # else:
            #     self._send_status("WARNING: Camera video stream unavailable")
            self._send_status("Real camera initialized (RTSP disabled)")
            
            # Initialize SD card
            self.pipeline.initialize_sd_card()
        else:
            # Simulation mode
            self.camera = None
            self.pipeline = None
            self.get_logger().info("Simulation mode: Waiting for images on /camera/image...")
            self._send_status("Simulation camera initialized")
    
    def _find_ros2_workspace(self) -> str:
        """Locate the ROS2 workspace root directory"""
        current_file = os.path.abspath(__file__)
        search_dir = os.path.dirname(current_file)
        
        # Search up to 10 levels
        for _ in range(10):
            parent = os.path.dirname(search_dir)
            if os.path.basename(search_dir) == 'ros2_ws':
                return search_dir
            if parent == search_dir:  # Reached root
                break
            search_dir = parent
        
        # Fallback to home directory
        return os.path.expanduser('~')
    
    def _log_initialization_complete(self):
        """Log initialization summary"""
        self.get_logger().info("=" * 70)
        self.get_logger().info(" SIYI UNIFIED PIPELINE INITIALIZED")
        self.get_logger().info("=" * 70)
        self.get_logger().info(
            f" Camera mode: {'SIMULATION' if not self.use_real_camera else 'REAL CAMERA'}")
        self.get_logger().info(" Pipeline ready. Waiting for triggers...")
        self.get_logger().info("=" * 70)
    
    def _pipeline_loop(self):
        """
        Main execution loop.
        
        Responsibilities:
        1. Execute capture pipeline when triggered (in separate thread)
        2. Publish disk status
        """
        # Check if camera is enabled (altitude check)
        with self.config_lock:
            camera_enabled = self.camera_enabled
        
        if not camera_enabled:
            return
        
        # Handle capture requests
        if self.capture_requested.is_set():
            self.capture_requested.clear()
            self._handle_capture_request()
        
        # RTSP video streaming disabled - not needed for capture workflow
        # Uncomment below if you need live video preview during flight
        # self._publish_video_stream()
        
        # Publish disk status
        self._publish_disk_status()
    
    def _handle_capture_request(self):
        """Handle capture request (executed in separate thread)"""
        if self.use_real_camera:
            # Check if previous pipeline is still running
            if self.pipeline.is_busy():
                self.get_logger().warn("Previous pipeline still running, skipping request")
                return
            
            # Execute pipeline in separate thread (non-blocking)
            capture_thread = Thread(target=self._execute_real_camera_capture, daemon=True)
            capture_thread.start()
        else:
            # Simulation mode: save current image
            self._execute_simulation_capture()
    
    def _execute_real_camera_capture(self):
        """Execute real camera capture pipeline"""
        try:
            success = self.pipeline.execute_pipeline()
            
            if success:
                stats = self.pipeline.get_stats()
                self._send_status(
                    f"SUCCESS: Captured {stats['resolution']} image #{stats['photo_count']}")
                self._publish_camera_status(
                    f"SUCCESS: {stats['resolution']} image captured")
                
                # Publish the captured image
                # Get the last saved image and publish it
                self._publish_captured_image()
            else:
                self._send_status("FAILED: Capture pipeline error")
                self._publish_camera_status("FAILURE: Capture pipeline error")
                
        except PipelineError as e:
            self.get_logger().error(f"Pipeline error: {e}")
            self._send_status(f"FAILED: {e}")
            self._publish_camera_status(f"FAILURE: {e}")
    
    def _execute_simulation_capture(self):
        """Execute simulation capture (save current image)"""
        if self.latest_image_msg is None:
            self.get_logger().warn("Trigger received but no simulation image available")
            return
        
        try:
            # Convert ROS Image to OpenCV format
            if self.bridge is not None:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_image_msg, 'bgr8')
            else:
                cv_image = self._imgmsg_to_cv2_manual(self.latest_image_msg, 'bgr8')
            
            # Save to mapping directory
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = f"mapping_photo_{timestamp}.jpg"
            mapping_dir = self.storage.get_mapping_dir()
            filepath = os.path.join(mapping_dir, filename)
            
            cv2.imwrite(filepath, cv_image)
            self.get_logger().info(f"Simulation photo saved: {filepath}")
            self._send_status(f"Simulation photo captured: {timestamp}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save simulation image: {e}")
    
    def _publish_captured_image(self):
        """Publish the most recently captured image to ROS"""
        try:
            mapping_dir = self.storage.get_mapping_dir()
            # Find the most recently modified image file
            image_files = [
                os.path.join(mapping_dir, f) for f in os.listdir(mapping_dir)
                if f.lower().endswith(('.jpg', '.jpeg', '.png'))
            ]
            if not image_files:
                self.get_logger().warn("No image files found to publish")
                return

            latest_file = max(image_files, key=os.path.getmtime)
            img = cv2.imread(latest_file)
            if img is None:
                self.get_logger().error(f"Failed to read image: {latest_file}")
                return

            if self.bridge is not None:
                msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            else:
                msg = self._cv2_to_imgmsg_manual(img, 'bgr8')

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.image_pub.publish(msg)
            self.get_logger().info(f"Published image to {TOPIC_IMAGE_RAW}: {os.path.basename(latest_file)}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish captured image: {e}")
    
    def _publish_video_stream(self):
        """Publish live video stream - DISABLED (not needed for capture workflow)"""
        # RTSP video streaming commented out - only needed for live preview
        # Uncomment this entire method if you need continuous video feed
        # Current workflow: trigger capture → save to SD → download → detect
        # This doesn't require continuous RTSP streaming
        pass
        # if self.use_real_camera:
        #     if self.camera is None:
        #         return
        #     
        #     frame = self.camera.read_video_frame()
        #     if frame is not None:
        #         # Convert to ROS message
        #         if self.bridge is not None:
        #             msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        #         else:
        #             msg = self._cv2_to_imgmsg_manual(frame, 'bgr8')
        #         
        #         msg.header.stamp = self.get_clock().now().to_msg()
        #         msg.header.frame_id = "camera_link"
        #         
        #         self.image_pub.publish(msg)
        # else:
        #     # Simulation mode: republish simulation image
        #     if self.latest_image_msg is not None:
        #         self.get_logger().info(
        #             "Publishing simulation image", throttle_duration_sec=10.0)
        #         self.image_pub.publish(self.latest_image_msg)
    
    def _publish_disk_status(self):
        """Publish disk space status"""
        free_mb = self.storage.get_free_space_mb()
        msg = Float64()
        msg.data = free_mb
        self.disk_status_pub.publish(msg)
    
    def camera_trigger_callback(self, msg: Bool):
        """Handle capture trigger requests"""
        if msg.data:
            self.get_logger().info("Capture trigger received!")
            self.capture_requested.set()
        else:
            self.get_logger().debug("Trigger received with data=False, ignoring")
    
    def set_resolution_callback(self, msg: String):
        """Handle resolution change requests"""
        resolution = msg.data.upper()
        
        if resolution in PHOTO_RESOLUTIONS:
            if self.pipeline is not None:
                self.pipeline.set_resolution(resolution)
            self._send_status(f"Resolution set to {resolution}")
        else:
            self.get_logger().warn(f"Invalid resolution: {resolution}")
    
    def altitude_callback(self, msg: Float64):
        """Handle altitude updates for camera enable/disable"""
        current_alt = msg.data
        
        with self.config_lock:
            was_enabled = self.camera_enabled
            
            if current_alt >= self.altitude_threshold:
                self.camera_enabled = True
                if not was_enabled:
                    self.get_logger().info(
                        f"Altitude {current_alt:.2f}m >= {self.altitude_threshold:.2f}m - "
                        "Camera ENABLED")
                    self._send_status("Altitude threshold reached - Camera enabled")
            else:
                self.camera_enabled = False
                if was_enabled:
                    self.get_logger().info(
                        f"Altitude {current_alt:.2f}m < {self.altitude_threshold:.2f}m - "
                        "Camera DISABLED")
                    self._send_status("Below altitude threshold - Camera disabled")
    
    def sim_image_callback(self, msg: Image):
        """Callback for simulation images"""
        self.latest_image_msg = msg
    
    def _send_status(self, text: str):
        """Send status message to MAVROS"""
        msg = StatusText()
        msg.severity = MAVROS_SEVERITY_INFO
        msg.text = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status: {text}")
    
    def _publish_camera_status(self, text: str):
        """Publish camera-specific status"""
        msg = String()
        msg.data = text
        self.camera_status_pub.publish(msg)
    
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
    
    def _imgmsg_to_cv2_manual(self, img_msg: Image, desired_encoding: str = 'bgr8') -> np.ndarray:
        """Convert ROS Image message to OpenCV image without cv_bridge"""
        if img_msg.encoding != desired_encoding:
            self.get_logger().warn(
                f'Image encoding mismatch: {img_msg.encoding} vs {desired_encoding}')
        
        dtype = np.uint8
        n_channels = 3 if desired_encoding == 'bgr8' else 1
        
        img_buf = np.asarray(img_msg.data, dtype=dtype)
        cv_image = img_buf.reshape(img_msg.height, img_msg.width, n_channels)
        
        return cv_image
    
    def shutdown(self):
        """Proper shutdown handler"""
        try:
            self.get_logger().info("Shutting down SIYI pipeline...")
        except Exception:
            pass
        
        # Close camera interface
        if self.camera is not None:
            try:
                self.camera.close()
            except Exception:
                pass
        
        try:
            self.get_logger().info("✓ Shutdown complete")
        except Exception:
            pass


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = SIYINode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Initiating shutdown...")
        node.shutdown()
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
