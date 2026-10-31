"""Live ROS adapter around the repository's shared SAHI inference functions."""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from astra_live_detection.latest_worker import LatestWorker


class LiveDetection(Node):
    def __init__(self):
        super().__init__('astra_live_detection')
        def parameter(name, default):
            return self.declare_parameter(name, default).value
        model_path = Path(parameter('model_path', '')).expanduser()
        device = parameter('device', 'cpu')
        self.confidence = parameter('confidence_threshold', 0.3)
        self.slice_height = parameter('slice_height', 640)
        self.slice_width = parameter('slice_width', 640)
        self.overlap = parameter('overlap', 0.2)
        max_age = parameter('max_pending_age_sec', 2.0)
        enabled = parameter('enabled', True)
        if not model_path.is_file() or model_path.suffix not in ('.pt', '.engine'):
            raise ValueError('model_path must point to an existing .pt or .engine file')
        if not (math.isfinite(self.confidence) and 0 < self.confidence <= 1 and
                math.isfinite(self.overlap) and 0 <= self.overlap < 1 and
                32 <= self.slice_height <= 4096 and 32 <= self.slice_width <= 4096 and
                math.isfinite(max_age) and max_age > 0):
            raise ValueError('invalid confidence, slice size, overlap or frame age')

        # Lazy imports let interface tests inject a deterministic backend.
        from astra_live_detection.model_manager import load_sahi_model
        from astra_live_detection.detection_processor import run_sahi_detection
        from astra_live_detection.annotation import annotate_frame
        self.model_format = 'tensorrt' if model_path.suffix == '.engine' else 'pytorch'
        self.model = load_sahi_model(str(model_path.resolve()), self.model_format,
                                     self.confidence, device, self.get_logger())
        if self.model is None:
            raise RuntimeError('SAHI/YOLO model could not be loaded; detection is unavailable')
        self.detect = run_sahi_detection
        self.annotate = annotate_frame
        self.device = device
        self.bridge = CvBridge()
        self.inference_count = 0
        self.last_received = None
        self.last_published = None
        self.last_publish_error = ''
        self.logged_worker_error = ''
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.results_pub = self.create_publisher(Detection2DArray, 'detection/results', qos)
        self.image_pub = self.create_publisher(Image, 'detection/annotated', qos)
        self.info_pub = self.create_publisher(String, 'detection/info', qos)
        self.worker = LatestWorker(self.process, max_age)
        self.worker.set_enabled(enabled)
        self.subscription = self.create_subscription(Image, 'zr10/image_raw', self.receive,
                                                    qos_profile_sensor_data)
        self.enable_service = self.create_service(SetBool, 'detection/set_enabled', self.set_enabled)
        self.health_service = self.create_service(Trigger, 'detection/get_health', self.get_health)
        self.stats_service = self.create_service(Trigger, 'detection/get_statistics', self.get_statistics)
        self.timer = self.create_timer(0.05, self.publish_result)
        self.worker.start()

    def receive(self, message):
        self.last_received = time.monotonic()
        self.worker.submit(message)

    def process(self, message):
        started = time.monotonic()
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        self.inference_count += 1
        detections = self.detect(
            frame, self.model, self.slice_height, self.slice_width, self.overlap, self.overlap,
            self.confidence, 25, 1000000, 0.1, 10.0,
            self.model_format, self.device, False, self.inference_count, self.get_logger())
        duration = time.monotonic() - started
        annotated = self.annotate(frame, detections, duration, self.slice_height,
                                  self.slice_width, self.model_format)
        return message.header, detections, annotated, duration

    def publish_result(self):
        error = self.worker.status()['last_error']
        if error and error != self.logged_worker_error:
            self.get_logger().error(f'Inference failed: {error}')
        self.logged_worker_error = error
        result = self.worker.take_result()
        if result is None:
            return
        try:
            header, detections, annotated, duration = result
            output = Detection2DArray()
            output.header = header
            for item in detections:
                detection = Detection2D()
                detection.header = header
                x1, y1, x2, y2 = item['bbox']
                # vision_msgs 3+ uses center.position; older releases use Pose2D.
                center = detection.bbox.center
                position = getattr(center, 'position', center)
                position.x, position.y = float(x1 + x2) / 2, float(y1 + y2) / 2
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = item.get('yolo_class', item['class'])
                hypothesis.hypothesis.score = float(item['confidence'])
                detection.results.append(hypothesis)
                output.detections.append(detection)
            image = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            image.header = header
            info = String()
            info.data = json.dumps(dict(
                frame_id=header.frame_id,
                stamp=dict(sec=header.stamp.sec, nanosec=header.stamp.nanosec),
                processing_sec=duration, objects=detections))
            self.results_pub.publish(output)
            self.image_pub.publish(image)
            self.info_pub.publish(info)
            self.last_published = time.monotonic()
            self.last_publish_error = ''
        except Exception as exc:
            self.last_publish_error = str(exc)
            self.get_logger().error(f'Cannot publish detection result: {exc}')

    def set_enabled(self, request, response):
        self.worker.set_enabled(request.data)
        response.success = True
        response.message = 'enabled' if request.data else 'paused; in-flight result will be discarded'
        return response

    def get_statistics(self, request, response):
        response.success = True
        response.message = json.dumps(self.worker.status())
        return response

    def get_health(self, request, response):
        status = self.worker.status()
        status['input_age_sec'] = (None if self.last_received is None else
                                   time.monotonic() - self.last_received)
        status['result_age_sec'] = (None if self.last_published is None else
                                    time.monotonic() - self.last_published)
        status['publish_error'] = self.last_publish_error
        # Paused is an intentional state; input must still be arriving.
        response.success = bool(status['worker_alive'] and not status['last_error'] and
                                not self.last_publish_error and status['input_age_sec'] is not None and
                                status['input_age_sec'] < 5)
        response.message = json.dumps(status)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LiveDetection()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.worker.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
