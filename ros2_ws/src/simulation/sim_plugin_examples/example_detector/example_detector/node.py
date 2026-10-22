"""
example_detector/node.py -- PURPOSE: the ROS 2 node of the TEMPLATE detector. It does EXACTLY what the team's
detector (detection/new_od) does on the outside, so main_controller and the demo panel use it unchanged:

  1. watches the team photo folder (the one video_cam writes, new_od's "camera_feed_path":
     <ros2_ws>/src/video_cam/mapping_photos) for new photos "<lat> , <lon>.jpg";
  2. runs the detection logic (blob_logic.detect: colour blobs; replace it with your model);
  3. publishes ONE interfaces/ImageResult per photo on /image_detection (RELIABLE, depth 10), also when
     nothing was found, because main_controller waits until every photo name was reported;
  4. answers /sahi/get_health like new_od ("ModelLoaded: True"), so demo.sh can wait until it is ready.

Copy this package to make your own detector (README.md: copy -> rename -> change -> plug in -> check).
Parameters (all optional):
  camera_feed_path   ""  = the team photo folder found like new_od does; or any folder
  scan_period_s      1.0 how often the folder is looked at
  process_existing   true = photos already in the folder at start are processed too (like new_od)
  min_blob_area_px   150 smallest object, in full-photo pixels;  max_blob_area_px 200000 largest
  downscale          4   work on a 1/4 size copy (faster)
  person_hsv_low / person_hsv_high / tent_hsv_low / tent_hsv_high   colour ranges, OpenCV HSV [h, s, v]
  serve_health       true = serve /sahi/get_health (set false if new_od also runs)
  method_name        the text in ImageResult.method (shown by the panel; must not start with truth_detector)
"""

from __future__ import annotations

import os
import time
from datetime import datetime
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from . import blob_logic


def team_photo_dir() -> str:
    """The team photo folder, found the way video_cam / new_od find it: walk up from this file to the folder
    that has both install/ and src/ (the ros2_ws), then src/video_cam/mapping_photos."""
    here = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        if os.path.isdir(os.path.join(here, 'install')) and os.path.isdir(os.path.join(here, 'src')):
            return os.path.join(here, 'src', 'video_cam', 'mapping_photos')
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent
    return os.path.expanduser('~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/video_cam/mapping_photos')


class ExampleDetector(Node):
    """Folder in, /image_detection out (see the module docstring)."""

    def __init__(self):
        super().__init__('example_detector')
        p = self.declare_parameter
        p('camera_feed_path', '')
        p('scan_period_s', 1.0)
        p('process_existing', True)
        p('min_blob_area_px', 150)
        p('max_blob_area_px', 200000)
        p('downscale', 4)
        for cid, name in blob_logic.CLASS_NAMES.items():
            low, high = blob_logic.DEFAULT_RANGES[cid]
            p(f'{name}_hsv_low', list(low))
            p(f'{name}_hsv_high', list(high))
        p('serve_health', True)
        p('method_name', 'example_detector (colour blobs)')
        get = lambda n: self.get_parameter(n).value          # noqa: E731
        self.folder = get('camera_feed_path') or team_photo_dir()
        self.min_area = float(get('min_blob_area_px'))
        self.max_area = float(get('max_blob_area_px'))
        self.downscale = int(get('downscale'))
        self.method = str(get('method_name'))
        self.ranges = {cid: (list(get(f'{name}_hsv_low')), list(get(f'{name}_hsv_high')))
                       for cid, name in blob_logic.CLASS_NAMES.items()}
        os.makedirs(self.folder, exist_ok=True)

        from interfaces.msg import ImageResult
        self._ImageResult = ImageResult
        # RELIABLE, depth 10: exactly like new_od. main_controller subscribes RELIABLE; a BEST_EFFORT
        # publisher would never reach it.
        self.pub = self.create_publisher(ImageResult, '/image_detection',
                                         QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.waypoint_index = -1
        try:
            from mavros_msgs.msg import WaypointReached
            self.create_subscription(WaypointReached, '/mavros/mission/reached', self._on_reached, 10)
        except ImportError:                                  # mavros_msgs missing: waypoint_index stays -1
            pass
        self.errors = 0
        if get('serve_health'):
            from std_srvs.srv import Trigger
            self.create_service(Trigger, '/sahi/get_health', self._on_health)
        self.seen = set()
        self.sizes: Dict[str, int] = {}
        if not get('process_existing'):
            self.seen.update(n for n in os.listdir(self.folder) if blob_logic.is_photo(n))
        self.create_timer(float(get('scan_period_s')), self._scan)
        self.get_logger().info(f'watching {self.folder} for photos; publishing /image_detection '
                               f'(method "{self.method}")')

    # ------------------------------------------------------------------ callbacks
    def _on_reached(self, msg) -> None:
        self.waypoint_index = int(msg.wp_seq)

    def _on_health(self, _req, resp):
        """Same text format as new_od (stack_check --need-detector looks for "ModelLoaded: True")."""
        resp.success = True
        resp.message = f'Healthy: True, ConsecErrors: {self.errors}, ModelLoaded: True, Format: colour-blobs'
        return resp

    def _scan(self) -> None:
        """Find finished new photos (oldest first) and process each once."""
        try:
            names = [n for n in os.listdir(self.folder) if blob_logic.is_photo(n) and n not in self.seen]
        except OSError as exc:
            self.get_logger().warn(f'cannot list {self.folder}: {exc}')
            return
        paths = sorted((os.path.join(self.folder, n) for n in names), key=lambda f: os.path.getmtime(f))
        for path in paths:
            name = os.path.basename(path)
            size = os.path.getsize(path)
            # A photo counts as finished when its size did not change since the last scan (video_cam writes
            # atomically, so this is only a safety net for other writers).
            if self.sizes.get(name) != size or time.time() - os.path.getmtime(path) < 0.3:
                self.sizes[name] = size
                continue
            self.seen.add(name)
            self._process(path)

    # ------------------------------------------------------------------ the work
    def _process(self, path: str) -> None:
        import cv2
        name = os.path.basename(path)
        image = cv2.imread(path)
        if image is None:
            self.errors += 1
            self.get_logger().warn(f'cannot read {name}: publishing an empty result for it')
            detections: List[blob_logic.Detection] = []
        else:
            detections = blob_logic.detect(image, self.ranges, self.min_area, self.max_area, self.downscale)
        self.pub.publish(self._result_message(name, detections))
        found = ', '.join(f'{d.name} {d.score:.2f}' for d in detections) or 'nothing'
        self.get_logger().info(f'{name}: {found}')

    def _result_message(self, name: str, detections: List[blob_logic.Detection]):
        """One ImageResult, filled exactly like new_od fills it."""
        from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
        msg = self._ImageResult()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        arr = Detection2DArray()
        arr.header = msg.header
        for d in detections:
            det = Detection2D()
            det.header = msg.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.class_id          # "0" person, "1" tent
            hyp.hypothesis.score = float(d.score)         # 0..1
            det.results.append(hyp)
            det.bbox.center.position.x = float(d.cx)      # full-photo pixels
            det.bbox.center.position.y = float(d.cy)
            det.bbox.size_x = float(d.w)
            det.bbox.size_y = float(d.h)
            arr.detections.append(det)
        msg.detections = arr
        msg.image_name = name                             # the file name only, exactly as in the folder
        msg.timestamp = datetime.now().isoformat()
        msg.num_detections = len(detections)
        msg.saved_to = ''
        msg.method = self.method
        msg.slice_size = 'n/a'
        msg.overlap = 'n/a'
        msg.waypoint_index = self.waypoint_index
        lat, lon = blob_logic.latlon_from_name(name)
        msg.latitude, msg.longitude = float(lat), float(lon)
        msg.classes = [d.name for d in detections]
        msg.confidences = [float(d.score) for d in detections]
        msg.areas = [float(d.area) for d in detections]
        msg.descriptions = [f'{d.name} at ({d.cx:.0f}, {d.cy:.0f}) px' for d in detections]
        return msg


def main(args=None) -> None:
    """Entry point: ros2 run example_detector example_detector."""
    try:
        from rclpy.executors import ExternalShutdownException
    except ImportError:                      # older rclpy
        ExternalShutdownException = KeyboardInterrupt  # noqa: N806
    rclpy.init(args=args)
    node = ExampleDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C, or the launch system stopping us
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
