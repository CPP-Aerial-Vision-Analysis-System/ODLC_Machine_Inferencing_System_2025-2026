#!/usr/bin/env python3
"""SIYI Camera ROS2 Node."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, String
from mavros_msgs.msg import StatusText
from rclpy.qos import qos_profile_sensor_data

import json
import os
import cv2
import numpy as np
from threading import Lock, Event, Thread
from typing import Dict, Any, List

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except Exception as e:
    print(f"Warning: cv_bridge import failed: {e}")
    CV_BRIDGE_AVAILABLE = False

from .config import (
    NODE_LOOP_PERIOD,
    HEALTH_CHECK_PERIOD,
    DEFAULT_USE_REAL_CAMERA,
    DEFAULT_MIN_ALTITUDE_AGL,
    DEFAULT_CAMERA_IP,
    DEFAULT_CTRL_PORT,
    DEFAULT_MEDIA_PORT,
    DEFAULT_HTTP_TIMEOUT,
    DEFAULT_CAPTURE_TIMEOUT,
    DEFAULT_MIN_FREE_SPACE_MB,
    PHOTO_RESOLUTIONS,
)
from .camera_interface import CameraInterface, CameraConnectionError
from .storage_manager import StorageManager
from .pipeline_orchestrator import PipelineOrchestrator, PipelineError


class SIYINode(Node):
    """ROS2 node wrapper for SIYI camera pipeline"""

    def __init__(self):
        super().__init__('siyi_unified_pipeline')

        self._declare_parameters()
        self._load_parameters()

        # Publishers
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.status_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
        self.camera_status_pub = self.create_publisher(String, '/camera/status', 10)
        self.disk_status_pub = self.create_publisher(Float64, '/camera/disk_free_mb', 10)
        self.health_pub = self.create_publisher(String, '/camera/health', 10)

        # Subscribers
        self.create_subscription(Bool, '/camera/trigger', self.camera_trigger_callback, 10)
        self.create_subscription(String, '/camera/set_resolution', self.set_resolution_callback, 10)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.altitude_callback, qos_profile_sensor_data)
        self.create_subscription(String, '/camera/command', self.camera_command_callback, 10)

        # State
        self.camera_enabled = True
        self.camera_connected = self.use_real_camera
        self.config_lock = Lock()
        self.camera_control_lock = Lock()
        self.capture_lock = Lock()
        self.capture_requested = Event()

        # CV Bridge
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        else:
            self.bridge = None
            self.get_logger().warn("cv_bridge not available, using alternative conversion")

        self._initialize_components()

        # Capture polling timer (1 Hz)
        self.pipeline_timer = self.create_timer(NODE_LOOP_PERIOD, self._pipeline_loop)

        # Health + disk status on a slower cadence
        self.health_timer = self.create_timer(HEALTH_CHECK_PERIOD, self._health_check)

        self._log_initialization_complete()

    def _declare_parameters(self):
        self.declare_parameter('use_real_camera', DEFAULT_USE_REAL_CAMERA)
        self.declare_parameter('min_altitude_agl', DEFAULT_MIN_ALTITUDE_AGL)
        self.declare_parameter('camera_ip', DEFAULT_CAMERA_IP)
        self.declare_parameter('ctrl_port', DEFAULT_CTRL_PORT)
        self.declare_parameter('media_port', DEFAULT_MEDIA_PORT)
        self.declare_parameter('http_timeout_sec', DEFAULT_HTTP_TIMEOUT)
        self.declare_parameter('capture_timeout_sec', DEFAULT_CAPTURE_TIMEOUT)
        self.declare_parameter('min_free_space_mb', DEFAULT_MIN_FREE_SPACE_MB)
        self.declare_parameter('workspace_path', '')

    def _load_parameters(self):
        self.use_real_camera = self.get_parameter('use_real_camera').value
        self.altitude_threshold = self.get_parameter('min_altitude_agl').value
        self.camera_ip = self.get_parameter('camera_ip').value
        self.ctrl_port = self.get_parameter('ctrl_port').value
        self.media_port = self.get_parameter('media_port').value
        self.http_timeout = self.get_parameter('http_timeout_sec').value
        self.capture_timeout = self.get_parameter('capture_timeout_sec').value
        self.min_free_space_mb = self.get_parameter('min_free_space_mb').value
        self.workspace_path = self.get_parameter('workspace_path').value

    def _initialize_components(self):
        if self.workspace_path:
            workspace_root = self.workspace_path
        else:
            workspace_root = self._find_ros2_workspace()

        self.storage = StorageManager(workspace_root, logger=self.get_logger())

        if self.use_real_camera:
            self.camera = CameraInterface(
                camera_ip=self.camera_ip,
                ctrl_port=self.ctrl_port,
                media_port=self.media_port,
                http_timeout=self.http_timeout,
                logger=self.get_logger()
            )

            self.pipeline = PipelineOrchestrator(
                camera=self.camera,
                storage=self.storage,
                logger=self.get_logger()
            )

            self._send_status("Real camera initialized")
            self.pipeline.initialize_sd_card()
        else:
            self.camera = None
            self.pipeline = None
            self.get_logger().info("Simulation mode: No real camera configured")
            self._send_status("Simulation camera initialized")

    def _find_ros2_workspace(self) -> str:
        current_file = os.path.abspath(__file__)
        search_dir = os.path.dirname(current_file)

        for _ in range(10):
            if (os.path.exists(os.path.join(search_dir, "install")) and
                    os.path.exists(os.path.join(search_dir, "src"))):
                return os.path.join(search_dir, "src", "video_cam")
            parent = os.path.dirname(search_dir)
            if parent == search_dir:
                break
            search_dir = parent

        raise RuntimeError(
            "Could not auto-detect ROS2 workspace (no directory with both install/ and src/). "
            "Set the 'workspace_path' parameter."
        )

    def _log_initialization_complete(self):
        mode = 'SIMULATION' if not self.use_real_camera else 'REAL CAMERA'
        self.get_logger().info(f"SIYI pipeline initialized ({mode}). Waiting for triggers.")
        self.get_logger().info("Command service ready at /camera/command")

    def _pipeline_loop(self):
        with self.config_lock:
            camera_enabled = self.camera_enabled

        if not camera_enabled:
            return

        if self.capture_requested.is_set():
            self.capture_requested.clear()
            self._handle_capture_request()

    def _handle_capture_request(self):
        if not self.use_real_camera:
            return

        if not self.camera_connected:
            self.get_logger().warn("Camera unreachable, skipping capture request")
            self._publish_camera_status("UNREACHABLE: Capture request dropped")
            return

        if not self.capture_lock.acquire(blocking=False):
            self.get_logger().warn("Capture already in flight, skipping request")
            self._publish_camera_status("BUSY: Capture request dropped, capture in flight")
            return

        capture_thread = Thread(target=self._execute_real_camera_capture, daemon=True)
        capture_thread.start()

    def _execute_real_camera_capture(self):
        try:
            with self.camera_control_lock:
                filepath = self.pipeline.execute_pipeline()

            stats = self.pipeline.get_stats()
            self._send_status(
                f"SUCCESS: Captured {stats['resolution']} image #{stats['photo_count']}")
            self._publish_camera_status(
                f"SUCCESS: {stats['resolution']} image captured")

            self._publish_captured_image(filepath)

        except PipelineError as e:
            self.get_logger().error(f"Pipeline error: {e}")
            self._send_status(f"FAILED: {e}")
            self._publish_camera_status(f"FAILURE: {e}")
        finally:
            self.capture_lock.release()

    @staticmethod
    def _result_payload(ok: bool, **kwargs: Any) -> Dict[str, Any]:
        payload: Dict[str, Any] = {'ok': ok}
        payload.update(kwargs)
        return payload

    @staticmethod
    def _split_csv(parameter: str, expected_len: int) -> List[str]:
        parts = [item.strip() for item in parameter.split(',') if item.strip()]
        if len(parts) != expected_len:
            raise ValueError(
                f"Expected {expected_len} CSV values, got {len(parts)}"
            )
        return parts

    def _execute_camera_command(self, command: str, parameter: str) -> Dict[str, Any]:
        if not self.use_real_camera or self.camera is None:
            return self._result_payload(
                False,
                error='Camera command service requires use_real_camera=true',
            )

        cmd = command.strip().lower().replace('-', '_')
        param = parameter.strip()

        if cmd == 'capture':
            resolution = '4K'
            if param:
                resolution = param.upper()
                if resolution not in PHOTO_RESOLUTIONS:
                    raise ValueError(
                        f"Invalid resolution '{resolution}'. Valid: {sorted(PHOTO_RESOLUTIONS.keys())}"
                    )

            if self.capture_lock.locked():
                return self._result_payload(False, action='capture', error='Capture already in flight')

            if self.pipeline is not None:
                self.pipeline.set_resolution(resolution)

            self.capture_requested.set()
            return self._result_payload(True, action='capture', queued=True,
                                        resolution=resolution)

        if self.pipeline is not None and self.pipeline.is_busy():
            return self._result_payload(False, action=cmd, error='Pipeline is busy')

        if cmd == 'autofocus':
            x = 0
            y = 0
            if param:
                x_raw, y_raw = self._split_csv(param, 2)
                x = int(x_raw)
                y = int(y_raw)
            ok = self.camera.auto_focus(touch_x=x, touch_y=y)
            return self._result_payload(ok, action='autofocus', touch_x=x, touch_y=y)

        if cmd == 'zoom_manual':
            direction_map = {'in': 1, 'out': -1, 'stop': 0}
            direction = param.lower()
            if direction not in direction_map:
                raise ValueError("zoom_manual parameter must be one of: in, out, stop")
            value = self.camera.manual_zoom(direction_map[direction])
            return self._result_payload(
                value is not None,
                action='zoom_manual',
                direction=direction,
                zoom_x=value,
            )

        if cmd in {'zoom_absolute', 'zoom_auto'}:
            if not param:
                raise ValueError(f"{cmd} requires a zoom multiple, e.g. 4.5")
            multiple = float(param)
            ok = self.camera.absolute_zoom_autofocus(multiple)
            return self._result_payload(ok, action=cmd, target_zoom_x=multiple)

        if cmd == 'zoom_range':
            data = self.camera.get_supported_zoom_range()
            return self._result_payload(data is not None, action='zoom_range', data=data)

        if cmd == 'zoom_current':
            value = self.camera.get_current_zoom_magnification()
            return self._result_payload(value is not None, action='zoom_current', zoom_x=value)

        if cmd == 'focus_manual':
            direction_map = {'far': 1, 'near': -1, 'stop': 0}
            direction = param.lower()
            if direction not in direction_map:
                raise ValueError("focus_manual parameter must be one of: far, near, stop")
            ok = self.camera.manual_focus(direction_map[direction])
            return self._result_payload(ok, action='focus_manual', direction=direction)

        if cmd == 'gimbal_rotate':
            yaw_raw, pitch_raw = self._split_csv(param, 2)
            yaw = int(yaw_raw)
            pitch = int(pitch_raw)
            ok = self.camera.rotate_gimbal(yaw_speed=yaw, pitch_speed=pitch)
            return self._result_payload(ok, action='gimbal_rotate', yaw=yaw, pitch=pitch)

        if cmd == 'gimbal_stop':
            ok = self.camera.stop_gimbal_rotation()
            return self._result_payload(ok, action='gimbal_stop')

        if cmd == 'gimbal_center':
            ok = self.camera.center_gimbal()
            return self._result_payload(ok, action='gimbal_center')

        if cmd == 'gimbal_attitude':
            data = self.camera.request_gimbal_attitude()
            return self._result_payload(data is not None, action='gimbal_attitude', data=data)

        if cmd == 'gimbal_set_angles':
            yaw_raw, pitch_raw = self._split_csv(param, 2)
            yaw = float(yaw_raw)
            pitch = float(pitch_raw)
            data = self.camera.set_gimbal_angles(yaw_deg=yaw, pitch_deg=pitch)
            return self._result_payload(
                data is not None,
                action='gimbal_set_angles',
                target_yaw_deg=yaw,
                target_pitch_deg=pitch,
                data=data,
            )

        if cmd == 'gimbal_set_axis':
            axis_raw, angle_raw = self._split_csv(param, 2)
            axis = axis_raw.lower()
            angle = float(angle_raw)
            data = self.camera.set_single_axis_angle(axis=axis, angle_deg=angle)
            return self._result_payload(
                data is not None,
                action='gimbal_set_axis',
                axis=axis,
                target_angle_deg=angle,
                data=data,
            )

        if cmd == 'gimbal_mode_get':
            mode = self.camera.get_gimbal_mode()
            return self._result_payload(mode is not None, action='gimbal_mode_get', mode=mode)

        if cmd == 'gimbal_mode_set':
            if not param:
                raise ValueError("gimbal_mode_set requires one of: lock, follow, fpv")
            mode = param.lower()
            ok = self.camera.set_gimbal_motion_mode(mode)
            mode_after = self.camera.get_gimbal_mode()
            return self._result_payload(
                ok,
                action='gimbal_mode_set',
                mode_requested=mode,
                mode_after=mode_after,
            )

        if cmd == 'sd_format':
            if param.lower() != 'yes':
                raise ValueError("sd_format is destructive; pass parameter yes to continue")
            ok = self.camera.format_sd_card()
            return self._result_payload(ok, action='sd_format')

        raise ValueError(f"Unsupported command: {command}")

    def camera_command_callback(self, msg: String):
        try:
            try:
                data = json.loads(msg.data)
                command = data.get('command', '')
                parameter = data.get('parameter', '')
            except json.JSONDecodeError:
                parts = msg.data.split(' ', 1)
                command = parts[0]
                parameter = parts[1] if len(parts) > 1 else ''

            if not self.camera_control_lock.acquire(blocking=False):
                self._publish_camera_status(f"CMD BUSY: {command}")
                self.get_logger().warn(f"Camera busy, command dropped: {command}")
                return

            try:
                result = self._execute_camera_command(command, parameter)
            finally:
                self.camera_control_lock.release()

            success = bool(result.get('ok', False))

            if success:
                self._publish_camera_status(f"CMD SUCCESS: {command}")
                self.get_logger().info(f"Camera command success: {command}")
            else:
                self._publish_camera_status(f"CMD FAILED: {command} - {result.get('error', 'Unknown')}")
                self.get_logger().error(f"Camera command failed: {command} - {result.get('error', 'Unknown')}")

        except (ValueError, CameraConnectionError, OSError) as exc:
            self._publish_camera_status(f"CMD ERROR: {msg.data}")
            self.get_logger().error(f"Camera command error for {msg.data}: {exc}")
        except Exception as exc:
            self.get_logger().error(f"Unexpected camera command error for {msg.data}: {exc}")
            self._publish_camera_status(f"CMD ERROR: {msg.data}")

    def _publish_captured_image(self, filepath: str):
        """Publish a captured image to ROS by its filepath."""
        try:
            img = cv2.imread(filepath)
            if img is None:
                self.get_logger().error(f"Failed to read image: {filepath}")
                return

            if self.bridge is not None:
                msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            else:
                msg = self._cv2_to_imgmsg_manual(img, 'bgr8')

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.image_pub.publish(msg)
            self.get_logger().info(f"Published image to image_raw: {os.path.basename(filepath)}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish captured image: {e}")

    def _publish_disk_status(self):
        free_mb = self.storage.get_free_space_mb()
        msg = Float64()
        msg.data = free_mb
        self.disk_status_pub.publish(msg)

    def _health_check(self):
        """Periodic camera connectivity check + disk status (runs on slow timer)."""
        if not self.use_real_camera or self.camera is None:
            return

        was_connected = self.camera_connected
        self.camera_connected = self.camera.ping()

        health_msg = String()
        if self.camera_connected:
            health_msg.data = "connected"
            if not was_connected:
                self.get_logger().info("Camera connection restored")
                self._send_status("Camera connection restored")
        else:
            health_msg.data = "unreachable"
            if was_connected:
                self.get_logger().warn("Camera connection lost")
                self._send_status("WARNING: Camera unreachable")

        self.health_pub.publish(health_msg)
        self._publish_disk_status()

    def camera_trigger_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Capture trigger received!")
            self.capture_requested.set()
        else:
            self.get_logger().debug("Trigger received with data=False, ignoring")

    def set_resolution_callback(self, msg: String):
        resolution = msg.data.upper()

        if resolution in PHOTO_RESOLUTIONS:
            if self.pipeline is not None:
                self.pipeline.set_resolution(resolution)
            self._send_status(f"Resolution set to {resolution}")
        else:
            self.get_logger().warn(f"Invalid resolution: {resolution}")

    def altitude_callback(self, msg: Float64):
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

    def _send_status(self, text: str):
        """Send status message to MAVROS, mission planner, ardupilot messages tab"""
        msg = StatusText()
        msg.severity = 6
        msg.text = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status: {text}")

    def _publish_camera_status(self, text: str):
        msg = String()
        msg.data = text
        self.camera_status_pub.publish(msg)

    def _cv2_to_imgmsg_manual(self, cv_image: np.ndarray, encoding: str = 'bgr8') -> Image:
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

    def shutdown(self):
        try:
            self.get_logger().info("Shutting down SIYI pipeline...")
        except Exception:
            pass

        if hasattr(self, 'pipeline_timer'):
            self.pipeline_timer.cancel()
        if hasattr(self, 'health_timer'):
            self.health_timer.cancel()

        if self.camera is not None:
            try:
                self.camera.close()
            except Exception:
                pass

        try:
            self.get_logger().info("Shutdown complete")
        except Exception:
            pass


def main(args=None):
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
