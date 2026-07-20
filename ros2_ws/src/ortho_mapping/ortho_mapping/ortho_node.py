#!/usr/bin/env python3
"""ROS2 wrapper for the batch orthomosaic builder.

This node is PASSIVE until told to map. It is meant to be started (or
already running) after the mapping flight, when no more photos will be
taken. Triggers, any of:

  - `MAP_START` on /mapping/command (std_msgs/String)
  - a GCS statustext containing `MAP_START` on /mavros/statustext/recv
  - launch with autostart:=true (maps immediately, then stays up)

The heavy lifting runs in a background thread so the executor stays
responsive; progress and the final result are published on
/mapping/status and echoed to the GCS via /mavros/statustext/send.
"""

import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from mavros_msgs.msg import StatusText

from .config import (
    DEFAULT_ALTITUDE_AGL_M,
    DEFAULT_GSD_M,
    DEFAULT_HFOV_DEG,
    DEFAULT_MAX_CANVAS_MP,
    DEFAULT_MAX_REFINE_SHIFT_M,
    MappingConfig,
)
from .mapper import build_map

# Progress is throttled to every N images so statustext (50-char budget,
# telemetry bandwidth) isn't spammed at one message per photo.
_PROGRESS_EVERY = 10


def find_default_images_dir() -> str:
    """Locate ros2_ws/video_cam/mapping_photos the same way video_cam does:
    walk up from this file until a directory holding install/ + src/."""
    search_dir = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        if (os.path.isdir(os.path.join(search_dir, 'install'))
                and os.path.isdir(os.path.join(search_dir, 'src'))):
            return os.path.join(search_dir, 'src', 'video_cam', 'mapping_photos')
        parent = os.path.dirname(search_dir)
        if parent == search_dir:
            break
        search_dir = parent
    return os.path.join(os.path.expanduser('~'), 'video_cam', 'mapping_photos')


class OrthoNode(Node):

    def __init__(self):
        super().__init__('ortho_mapping')

        default_dir = find_default_images_dir()
        self.declare_parameter('images_dir', default_dir)
        self.declare_parameter('output_dir', '')  # '' -> <images_dir>/../mapping_output
        self.declare_parameter('gsd_cm', DEFAULT_GSD_M * 100.0)
        self.declare_parameter('hfov_deg', DEFAULT_HFOV_DEG)
        self.declare_parameter('default_altitude_agl', DEFAULT_ALTITUDE_AGL_M)
        self.declare_parameter('heading_source', 'auto')  # auto|sidecar|track|fixed
        self.declare_parameter('fixed_heading_deg', 0.0)
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('use_gimbal_attitude', True)
        self.declare_parameter('refine', True)
        self.declare_parameter('max_refine_shift_m', DEFAULT_MAX_REFINE_SHIFT_M)
        self.declare_parameter('max_canvas_mp', DEFAULT_MAX_CANVAS_MP)
        self.declare_parameter('autostart', False)

        self.status_pub = self.create_publisher(String, '/mapping/status', 10)
        self.statustext_pub = self.create_publisher(
            StatusText, '/mavros/statustext/send', 10)
        self.create_subscription(
            String, '/mapping/command', self._command_cb, 10)
        self.create_subscription(
            StatusText, '/mavros/statustext/recv', self._statustext_cb,
            qos_profile_sensor_data)

        self._run_lock = threading.Lock()
        self._worker = None

        self.get_logger().info(
            f"ortho_mapping ready (images_dir={self.get_parameter('images_dir').value}). "
            "Send MAP_START on /mapping/command to build the map.")

        if self.get_parameter('autostart').value:
            self._start_mapping()

    # ------------------------------------------------------------------
    # Triggers
    # ------------------------------------------------------------------

    def _command_cb(self, msg: String):
        command = msg.data.strip().upper()
        if command in ('MAP_START', 'START', 'STITCH_START'):
            self._start_mapping()
        elif command:
            self.get_logger().warn(f"Unknown mapping command: {msg.data!r}")

    def _statustext_cb(self, msg: StatusText):
        if 'MAP_START' in msg.text.upper():
            self._start_mapping()

    def _start_mapping(self):
        if not self._run_lock.acquire(blocking=False):
            self.get_logger().warn("Mapping already running; trigger ignored")
            self._publish_status("BUSY: mapping already running")
            return
        self._worker = threading.Thread(target=self._run_mapping, daemon=True)
        self._worker.start()

    # ------------------------------------------------------------------
    # Worker
    # ------------------------------------------------------------------

    def _build_config(self) -> MappingConfig:
        images_dir = self.get_parameter('images_dir').value
        output_dir = self.get_parameter('output_dir').value
        if not output_dir:
            output_dir = os.path.join(os.path.dirname(images_dir.rstrip('/\\')),
                                      'mapping_output')
        return MappingConfig(
            images_dir=images_dir,
            output_dir=output_dir,
            gsd_m=float(self.get_parameter('gsd_cm').value) / 100.0,
            hfov_deg=float(self.get_parameter('hfov_deg').value),
            default_altitude_agl_m=float(
                self.get_parameter('default_altitude_agl').value),
            heading_source=str(self.get_parameter('heading_source').value),
            fixed_heading_deg=float(self.get_parameter('fixed_heading_deg').value),
            yaw_offset_deg=float(self.get_parameter('yaw_offset_deg').value),
            use_gimbal_attitude=bool(
                self.get_parameter('use_gimbal_attitude').value),
            refine=bool(self.get_parameter('refine').value),
            max_refine_shift_m=float(
                self.get_parameter('max_refine_shift_m').value),
            max_canvas_mp=float(self.get_parameter('max_canvas_mp').value),
        )

    def _run_mapping(self):
        try:
            cfg = self._build_config()
            self._publish_status(f"MAPPING: started on {cfg.images_dir}")

            def progress(done: int, of: int):
                if done % _PROGRESS_EVERY == 0 or done == of:
                    self._publish_status(f"MAPPING: {done}/{of} images placed")

            result = build_map(cfg, log=self.get_logger().info,
                               progress=progress)

            if result.ok:
                self._publish_status(
                    f"SUCCESS: {result.message} -> {result.image_path}")
            else:
                self._publish_status(f"FAILED: {result.message}")
        except Exception as exc:  # never let the worker kill the node
            self.get_logger().error(f"Mapping crashed: {exc}")
            self._publish_status(f"FAILED: {exc}")
        finally:
            self._run_lock.release()

    def _publish_status(self, text: str):
        self.get_logger().info(text)
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        st = StatusText()
        st.severity = 6
        st.text = text[:120]
        self.statustext_pub.publish(st)


def main(args=None):
    rclpy.init(args=args)
    node = OrthoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
