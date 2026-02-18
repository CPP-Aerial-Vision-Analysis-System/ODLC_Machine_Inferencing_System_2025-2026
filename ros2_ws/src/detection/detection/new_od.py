#!/usr/bin/env python3

# SAHI Object Detection ROS2 Lifecycle Node
# Watches camera-feed dir for new images, runs SAHI+YOLO inference,
# publishes results on /image_detection for main.py (MainController).

import os
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'
os.environ['YOLO_AUTOINSTALL'] = '0'
os.environ.setdefault('TRT_LOG_LEVEL', '2')
os.environ.setdefault('CUDA_MODULE_LOADING', 'LAZY')


import time
import threading
import queue
import json
from datetime import datetime
from typing import List, Dict, Tuple

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from interfaces.msg import ImageResult
from mavros_msgs.msg import WaypointReached
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

import cv2
import numpy as np

from detection.gpu_utils import (
    detect_device,
    optimize_gpu_memory,
    cleanup_gpu,
    TORCH_AVAILABLE,
)
from detection.model_manager import (
    get_ros2_ws_directory,
    resolve_model_path,
    load_sahi_model,
    warmup_model,
    MODEL_FORMAT_PYTORCH,
    MODEL_FORMAT_TENSORRT,
    MODEL_FORMAT_AUTO,
    SAHI_AVAILABLE,
    YOLO_AVAILABLE,
)
from detection.detection_processor import run_sahi_detection
from detection.annotation import annotate_frame, save_top_matches_crop

DEFAULT_CONFIDENCE = 0.25
DEFAULT_SLICE = 640
DEFAULT_OVERLAP = 0.25
DEFAULT_CHECK_INTERVAL = 2.0
CLASS_ID = {"person": "0", "tent": "1", "object": "2"}


class SAHIObjectDetectionNode(LifecycleNode):

    def __init__(self):
        super().__init__('new_od')

        # Lifecycle flags
        self.shutdown_requested = False
        self._active = False

        self.declare_parameter('model_path', 'yolo26m.pt')
        self.declare_parameter('model_format', MODEL_FORMAT_AUTO)
        self.declare_parameter('auto_convert_tensorrt', True)
        self.declare_parameter('tensorrt_workspace', 4)
        self.declare_parameter('confidence_threshold', DEFAULT_CONFIDENCE)
        self.declare_parameter('slice_height', DEFAULT_SLICE)
        self.declare_parameter('slice_width', DEFAULT_SLICE)
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

        # Instance state
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

        # Background model-loading thread
        self._model_ready = threading.Event()
        self._model_load_thread = None

        # Worker thread
        self.work_q: queue.Queue = queue.Queue(maxsize=50)
        self.worker_thread = None
        self.worker_stop = threading.Event()

        self._stats_lock = threading.Lock()
        self.processed_images: Dict[str, float] = {}

        self.stats = {
            'total_images_processed': 0,
            'total_detections': 0,
            'total_tents': 0,
            'total_people': 0,
            'avg_processing_time': 0.0,
            'last_processing_time': 0.0,
            'node_start_time': time.time(),
            'errors': 0,
        }

        self.health_status = {
            'is_healthy': True,
            'last_successful_detection': None,
            'consecutive_errors': 0,
        }

        self.waypoint_reached = 0

        # Set in on_configure
        self.camera_feed_path = None
        self.detection_results_path = None
        self.device = None
        self.model_format_detected = None
        self._check_count = 0
        self._model_wait_log_count = 0

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configuring node...")
        try:
            self._load_parameters()

            ros2_ws = get_ros2_ws_directory()

            cam = self.get_parameter('camera_feed_path').value
            out = self.get_parameter('detection_results_path').value
            self.camera_feed_path = cam or os.path.join(ros2_ws, "src", "video_cam", "mapping_photos")
            self.detection_results_path = out or os.path.join(ros2_ws, "src", "detection", "detection_results_sahi")

            os.makedirs(self.detection_results_path, exist_ok=True)
            if not os.path.exists(self.camera_feed_path):
                self.get_logger().warn(f"Camera feed dir missing, creating: {self.camera_feed_path}")
                os.makedirs(self.camera_feed_path, exist_ok=True)

            self.get_logger().info(f"Results dir: {self.detection_results_path}")

            # OpenCV hints for Jetson
            cv2.setNumThreads(0)
            cv2.ocl.setUseOpenCL(False)

            if self.device == 'auto':
                self.device = detect_device(self.get_logger())
            self.get_logger().info(f"Device: {self.device}")

            # GPU memory tuning (returns possibly-adjusted slice params)
            if self.device.startswith('cuda'):
                self.slice_height, self.slice_width, self.overlap_height_ratio, self.overlap_width_ratio = (
                    optimize_gpu_memory(
                        self.device,
                        self.slice_height, self.slice_width,
                        self.overlap_height_ratio, self.overlap_width_ratio,
                        self.get_logger(),
                    )
                )

            self.bridge = CvBridge()
            self.add_on_set_parameters_callback(self._parameter_callback)

            self.get_logger().info("Configuration complete")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activating node...")
        try:
            self._active = True

            # Background model load (may involve TRT conversion)
            self._model_ready.clear()
            self._model_load_thread = threading.Thread(
                target=self._background_model_load, daemon=True, name="model_loader",
            )
            self._model_load_thread.start()
            self.get_logger().info("Model loading in background...")

            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

            self.publisher = self.create_publisher(Image, '/sahi_detection_results', qos)
            self.detection_publisher = self.create_publisher(String, '/sahi_detection_info', qos)
            self.detection_pub = self.create_publisher(ImageResult, '/image_detection', qos)

            from std_srvs.srv import Trigger
            self.stats_service = self.create_service(Trigger, 'sahi/get_statistics', self._stats_srv_cb)
            self.health_service = self.create_service(Trigger, 'sahi/get_health', self._health_srv_cb)

            self.waypoint_subscription = self.create_subscription(
                WaypointReached, "/mavros/mission/reached", self._waypoint_cb, 10,
            )

            # Worker thread
            self.worker_stop.clear()
            self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
            self.worker_thread.start()

            self.timer = self.create_timer(self.check_interval, self.check_for_new_images)

            if self.enable_gpu_memory_cleanup and self.device.startswith('cuda'):
                self.gpu_cleanup_timer = self.create_timer(30.0, self._periodic_gpu_cleanup)

            self.stats['node_start_time'] = time.time()
            self.get_logger().info("Node activated and ready")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Activation failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Deactivating node...")
        self._active = False

        if self._model_load_thread and self._model_load_thread.is_alive():
            self._model_load_thread.join(timeout=10.0)

        self.worker_stop.set()
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=5.0)

        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        if self.gpu_cleanup_timer is not None:
            self.gpu_cleanup_timer.cancel()
            self.gpu_cleanup_timer = None

        self.get_logger().info("Node deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Cleaning up node...")

        if self._model_load_thread and self._model_load_thread.is_alive():
            self._model_load_thread.join(timeout=10.0)
        self.worker_stop.set()
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=5.0)

        for attr in ('timer', 'gpu_cleanup_timer'):
            obj = getattr(self, attr, None)
            if obj is not None:
                self.destroy_timer(obj)
                setattr(self, attr, None)

        for attr in ('publisher', 'detection_publisher', 'detection_pub'):
            obj = getattr(self, attr, None)
            if obj is not None:
                self.destroy_publisher(obj)
                setattr(self, attr, None)

        for attr in ('stats_service', 'health_service'):
            obj = getattr(self, attr, None)
            if obj is not None:
                self.destroy_service(obj)
                setattr(self, attr, None)

        if self.waypoint_subscription is not None:
            self.destroy_subscription(self.waypoint_subscription)
            self.waypoint_subscription = None

        if self.detection_model is not None:
            del self.detection_model
            self.detection_model = None

        cleanup_gpu()
        self.bridge = None
        self.get_logger().info("Cleanup complete")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Shutting down...")
        self.shutdown_requested = True

        try:
            # Cancel timers (safe even if already destroyed)
            for attr in ('timer', 'gpu_cleanup_timer'):
                obj = getattr(self, attr, None)
                if obj is not None:
                    obj.cancel()
                    setattr(self, attr, None)

            # Release model
            if self.detection_model is not None:
                del self.detection_model
                self.detection_model = None

            cleanup_gpu()
            if TORCH_AVAILABLE:
                try:
                    import torch
                    if torch.cuda.is_available():
                        torch.cuda.synchronize()
                except Exception:
                    pass

            # Final stats
            self.get_logger().info("=" * 60)
            self.get_logger().info("Final Statistics:")
            with self._stats_lock:
                self.get_logger().info(f"  Images Processed: {self.stats['total_images_processed']}")
                self.get_logger().info(f"  Detections: {self.stats['total_detections']}")
                self.get_logger().info(f"  People: {self.stats['total_people']}")
                self.get_logger().info(f"  Tents: {self.stats['total_tents']}")
                self.get_logger().info(f"  Avg Time: {self.stats['avg_processing_time']:.2f}s")
                self.get_logger().info(f"  Errors: {self.stats['errors']}")
            uptime = time.time() - self.stats['node_start_time']
            self.get_logger().info(f"  Uptime: {uptime:.1f}s")
            self.get_logger().info("=" * 60)
        except Exception as e:
            self.get_logger().error(f"Shutdown error: {e}")

        return TransitionCallbackReturn.SUCCESS

    # Timer & Worker

    def check_for_new_images(self) -> None:
        """Timer callback: scan camera_feed and enqueue new images."""
        if not self._active or self.shutdown_requested:
            return
        if not os.path.exists(self.camera_feed_path):
            return

        # Wait for model
        if not self._model_ready.is_set():
            self._model_wait_log_count += 1
            if self._model_wait_log_count % 5 == 1:
                self.get_logger().info("Waiting for model to finish loading...")
            return
        if self.detection_model is None:
            self.get_logger().warn("Model failed to load -- skipping")
            return

        try:
            self._cleanup_old_images()
            self._prune_processed()

            files: List[Tuple[str, float]] = []
            for fname in os.listdir(self.camera_feed_path):
                if fname.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')):
                    fpath = os.path.join(self.camera_feed_path, fname)
                    try:
                        files.append((fname, os.path.getmtime(fpath)))
                    except OSError:
                        continue
            files.sort(key=lambda x: x[1])

            self._check_count += 1
            if self._check_count % 10 == 0:
                self.get_logger().info(
                    f"Check #{self._check_count}: {len(files)} images, "
                    f"{len(self.processed_images)} processed, queue={self.work_q.qsize()}"
                )

            enqueued = 0
            for fname, _ in files:
                if enqueued >= self.max_images_per_cycle:
                    break
                if fname in self.processed_images:
                    continue
                fpath = os.path.join(self.camera_feed_path, fname)
                if not self._is_file_ready(fpath):
                    continue
                try:
                    self.work_q.put_nowait(fpath)
                    self.processed_images[fname] = time.time()
                    enqueued += 1
                    self.get_logger().info(f"Enqueued: {fname}")
                except queue.Full:
                    self.get_logger().warn("Work queue full")
                    break

        except Exception as e:
            self.get_logger().error(f"Error scanning images: {e}")
            with self._stats_lock:
                self.stats['errors'] += 1

    def _worker_loop(self):
        """Background worker: dequeue and process images one at a time."""
        while not self.worker_stop.is_set():
            try:
                path = self.work_q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._process_image(path)
                self.health_status['last_successful_detection'] = time.time()
                self.health_status['consecutive_errors'] = 0
                self.health_status['is_healthy'] = True
            except Exception as e:
                self.get_logger().error(f"Worker error ({os.path.basename(path)}): {e}")
                with self._stats_lock:
                    self.stats['errors'] += 1
                self.health_status['consecutive_errors'] += 1
            finally:
                self.work_q.task_done()

    # Image processing

    def _process_image(self, image_path: str) -> None:
        """Load, detect, annotate, publish for one image."""
        start = time.time()

        frame = cv2.imread(image_path)
        if frame is None:
            self.get_logger().warn(f"Could not load: {image_path}")
            return

        h, w = frame.shape[:2]
        self.get_logger().info(f"Processing: {os.path.basename(image_path)} ({w}x{h})")

        frame_orig = frame.copy()

        detections = run_sahi_detection(
            frame=frame,
            model=self.detection_model,
            slice_height=self.slice_height,
            slice_width=self.slice_width,
            overlap_h=self.overlap_height_ratio,
            overlap_w=self.overlap_width_ratio,
            confidence_threshold=self.confidence_threshold,
            min_area=self.min_detection_area,
            max_area=self.max_detection_area,
            min_aspect=self.min_aspect_ratio,
            max_aspect=self.max_aspect_ratio,
            model_format=self.model_format_detected or MODEL_FORMAT_PYTORCH,
            device=self.device,
            enable_gpu_cleanup=self.enable_gpu_memory_cleanup,
            images_processed=self.stats['total_images_processed'],
            logger=self.get_logger(),
        )

        elapsed = time.time() - start

        annotated = annotate_frame(
            frame, detections, elapsed,
            self.slice_height, self.slice_width,
            self.model_format_detected or MODEL_FORMAT_PYTORCH,
        )

        save_top_matches_crop(frame_orig, detections, image_path, self.detection_results_path)

        self._publish_results(annotated, detections, image_path)

        # Update stats (thread-safe)
        n_people = sum(1 for d in detections if d['class'] == 'person')
        n_tents = sum(1 for d in detections if d['class'] == 'tent')
        n_objects = sum(1 for d in detections if d['class'] == 'object')

        with self._stats_lock:
            self.stats['total_images_processed'] += 1
            self.stats['total_detections'] += len(detections)
            self.stats['total_people'] += n_people
            self.stats['total_tents'] += n_tents
            self.stats['last_processing_time'] = elapsed
            n = self.stats['total_images_processed']
            self.stats['avg_processing_time'] = (
                (self.stats['avg_processing_time'] * (n - 1) + elapsed) / n
            )

        self.get_logger().info(
            f"Found {len(detections)} objects in {elapsed:.2f}s: "
            f"{n_people} people, {n_tents} tents, {n_objects} other"
        )

    # Publishing (message format consumed by main.py)

    def _publish_results(self, annotated: np.ndarray, detections: List[Dict], image_path: str) -> None:
        """Publish annotated image, ImageResult, and detection-info JSON."""
        if self.publisher is None or self.detection_pub is None or self.bridge is None:
            return

        try:
            image_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera'
            self.publisher.publish(image_msg)

            original_filename = os.path.basename(image_path)
            name, ext = os.path.splitext(original_filename)
            output_path = os.path.join(self.detection_results_path, f"sahi_detected_{name}{ext}")
            cv2.imwrite(output_path, annotated)

            method = (
                'sahi+yolo+tensorrt' if self.model_format_detected == MODEL_FORMAT_TENSORRT
                else 'sahi+yolo'
            )

            ir = ImageResult()
            ir.header = image_msg.header
            ir.image_name = original_filename
            ir.timestamp = datetime.now().isoformat()
            ir.num_detections = len(detections)
            ir.saved_to = output_path
            ir.method = method
            ir.slice_size = f"{self.slice_height}x{self.slice_width}"
            ir.overlap = f"{self.overlap_height_ratio}x{self.overlap_width_ratio}"
            ir.waypoint_index = self.waypoint_reached

            det_array = Detection2DArray()
            det_array.header = image_msg.header

            classes, confidences, areas, descriptions, masks = [], [], [], [], []

            for det in detections:
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

            ir.detections = det_array
            ir.masks = masks
            ir.classes = classes
            ir.confidences = confidences
            ir.areas = areas
            ir.descriptions = descriptions

            self.detection_pub.publish(ir)

            n_people = sum(1 for d in detections if d['class'] == 'person')
            n_tents = sum(1 for d in detections if d['class'] == 'tent')
            n_objects = sum(1 for d in detections if d['class'] == 'object')

            info = {
                'image': original_filename,
                'timestamp': datetime.now().isoformat(),
                'num_people': n_people,
                'num_tents': n_tents,
                'num_objects': n_objects,
                'total_detections': len(detections),
                'method': method,
                'slice_size': f"{self.slice_height}x{self.slice_width}",
                'overlap': f"{self.overlap_height_ratio}x{self.overlap_width_ratio}",
            }
            info_msg = String()
            info_msg.data = json.dumps(info, separators=(",", ":"))
            self.detection_publisher.publish(info_msg)

        except Exception as e:
            self.get_logger().error(f"Publish error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            with self._stats_lock:
                self.stats['errors'] += 1

    # Background model loading

    def _background_model_load(self):
        """Load model in a daemon thread so on_activate returns immediately."""
        try:
            self.get_logger().info("Resolving model path...")
            resolved, fmt = resolve_model_path(
                self.model_path, self.model_format, self.auto_convert_tensorrt,
                self.slice_height, self.slice_width, self.tensorrt_workspace,
                self.device, self.get_logger(),
            )
            if resolved is None:
                self.get_logger().error("Model path resolution failed")
                self.health_status['is_healthy'] = False
                return

            self.model_format_detected = fmt

            model = load_sahi_model(
                resolved, fmt, self.confidence_threshold, self.device, self.get_logger(),
            )
            if model is None:
                self.health_status['is_healthy'] = False
                return

            self.detection_model = model
            warmup_model(
                model, self.slice_height, self.slice_width,
                self.overlap_height_ratio, self.overlap_width_ratio, self.get_logger(),
            )
            self.get_logger().info("Model ready -- processing can begin")
        except Exception as e:
            import traceback
            self.get_logger().error(f"Model load failed: {e}\n{traceback.format_exc()}")
            self.health_status['is_healthy'] = False
        finally:
            self._model_ready.set()

    # Utilities

    def _load_parameters(self) -> None:
        """Load and validate all declared parameters."""
        self.model_path = self.get_parameter('model_path').value
        self.model_format = self.get_parameter('model_format').value.lower()
        self.auto_convert_tensorrt = self.get_parameter('auto_convert_tensorrt').value
        self.tensorrt_workspace = self.get_parameter('tensorrt_workspace').value
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

        # Validate
        if self.model_format not in (MODEL_FORMAT_PYTORCH, MODEL_FORMAT_TENSORRT, MODEL_FORMAT_AUTO):
            self.get_logger().warn(f"Invalid model_format '{self.model_format}', using 'auto'")
            self.model_format = MODEL_FORMAT_AUTO
        if not 0 < self.confidence_threshold <= 1.0:
            self.get_logger().warn(f"Bad confidence {self.confidence_threshold}, using {DEFAULT_CONFIDENCE}")
            self.confidence_threshold = DEFAULT_CONFIDENCE
        if self.slice_height < 64 or self.slice_width < 64:
            self.slice_height = max(64, self.slice_height)
            self.slice_width = max(64, self.slice_width)
        if not 0 <= self.overlap_height_ratio < 1.0 or not 0 <= self.overlap_width_ratio < 1.0:
            self.overlap_height_ratio = DEFAULT_OVERLAP
            self.overlap_width_ratio = DEFAULT_OVERLAP
        if self.check_interval < 0.1:
            self.check_interval = DEFAULT_CHECK_INTERVAL
        if self.max_images_per_cycle < 1:
            self.max_images_per_cycle = 1

    @staticmethod
    def _is_file_ready(path: str, min_age: float = 0.2) -> bool:
        """Return True if a file is fully written and stable."""
        try:
            st = os.stat(path)
            if time.time() - st.st_mtime < min_age or st.st_size == 0:
                return False
            time.sleep(0.05)
            return st.st_size == os.stat(path).st_size
        except OSError:
            return False

    def _prune_processed(self, max_age: float = 3600.0, max_entries: int = 2000):
        """Evict old entries from processed_images to cap memory usage."""
        now = time.time()
        old = [k for k, v in self.processed_images.items() if now - v > max_age]
        for k in old:
            self.processed_images.pop(k, None)
        if len(self.processed_images) > max_entries:
            items = sorted(self.processed_images.items(), key=lambda kv: kv[1])
            for k, _ in items[: len(self.processed_images) - max_entries]:
                self.processed_images.pop(k, None)

    def _cleanup_old_images(self) -> None:
        """Delete oldest images if camera_feed exceeds the configured limit."""
        if self.max_camera_feed_images <= 0:
            return
        try:
            files = []
            for f in os.listdir(self.camera_feed_path):
                if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')):
                    fp = os.path.join(self.camera_feed_path, f)
                    try:
                        files.append((f, os.path.getmtime(fp)))
                    except OSError:
                        continue
            files.sort(key=lambda x: x[1])
            excess = len(files) - self.max_camera_feed_images
            if excess > 0:
                removed = 0
                for f, _ in files[:excess]:
                    try:
                        os.remove(os.path.join(self.camera_feed_path, f))
                        self.processed_images.pop(f, None)
                        removed += 1
                    except OSError:
                        pass
                if removed:
                    self.get_logger().info(f"Cleaned up {removed} old images")
        except OSError as e:
            self.get_logger().debug(f"Image cleanup error: {e}")

    def _periodic_gpu_cleanup(self) -> None:
        cleanup_gpu()

    def _waypoint_cb(self, msg: WaypointReached) -> None:
        self.waypoint_reached = msg.wp_seq

    def _parameter_callback(self, params: List[Parameter]):
        from rclpy.node import SetParametersResult
        for p in params:
            try:
                if p.name == 'confidence_threshold':
                    if 0 < p.value <= 1.0:
                        self.confidence_threshold = p.value
                    else:
                        return SetParametersResult(successful=False, reason="Must be 0..1")
                elif p.name == 'check_interval':
                    if p.value >= 0.1:
                        self.check_interval = p.value
                        if self.timer is not None:
                            self.timer.cancel()
                            self.destroy_timer(self.timer)
                            self.timer = self.create_timer(self.check_interval, self.check_for_new_images)
                    else:
                        return SetParametersResult(successful=False, reason="Must be >= 0.1")
                elif p.name == 'max_images_per_cycle':
                    if p.value >= 1:
                        self.max_images_per_cycle = p.value
                    else:
                        return SetParametersResult(successful=False, reason="Must be >= 1")
            except Exception as e:
                return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    def _stats_srv_cb(self, request, response):
        with self._stats_lock:
            s = self.stats
            txt = (
                f"Images: {s['total_images_processed']}, "
                f"Detections: {s['total_detections']}, "
                f"People: {s['total_people']}, Tents: {s['total_tents']}, "
                f"AvgTime: {s['avg_processing_time']:.2f}s, "
                f"Errors: {s['errors']}, "
                f"Uptime: {time.time() - s['node_start_time']:.0f}s"
            )
        response.success = True
        response.message = txt
        return response

    def _health_srv_cb(self, request, response):
        hs = self.health_status
        txt = (
            f"Healthy: {hs['is_healthy']}, "
            f"ConsecErrors: {hs['consecutive_errors']}, "
            f"ModelLoaded: {self.detection_model is not None}, "
            f"Format: {self.model_format_detected}"
        )
        response.success = hs['is_healthy']
        response.message = txt
        return response


def main(args=None):
    rclpy.init(args=args)

    if not SAHI_AVAILABLE or not YOLO_AVAILABLE:
        print("=" * 60)
        print("ERROR: Missing dependencies!")
        if not SAHI_AVAILABLE:
            print("  pip install sahi")
        if not YOLO_AVAILABLE:
            print("  pip install ultralytics")
        print("=" * 60)
        return

    node = SAHIObjectDetectionNode()

    try:
        if node.trigger_configure() != TransitionCallbackReturn.SUCCESS:
            node.get_logger().error("Configure failed")
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            return

        if node.trigger_activate() != TransitionCallbackReturn.SUCCESS:
            node.get_logger().error("Activate failed")
            node.trigger_cleanup()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            return

        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        node.get_logger().info(
            "Node active. 'ros2 lifecycle set /new_od deactivate' to pause."
        )
        executor.spin()

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt")
    finally:
        if not node.shutdown_requested:
            node.get_logger().info("Initiating lifecycle shutdown...")
            try:
                label = node.get_current_state().label
                if label == 'active':
                    node.trigger_deactivate()
                    node.trigger_cleanup()
                elif label == 'inactive':
                    node.trigger_cleanup()
            except Exception as e:
                node.get_logger().error(f"Shutdown error: {e}")

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
