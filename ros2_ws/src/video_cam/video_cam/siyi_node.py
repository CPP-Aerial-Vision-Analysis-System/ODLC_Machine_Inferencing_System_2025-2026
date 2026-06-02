#!/usr/bin/env python3

from video_cam.logging_utils import configure_console_format, install_wallclock_logging
configure_console_format()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Bool, Float64, String
from mavros_msgs.msg import StatusText
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge

import json
import os
import cv2
import time
import numpy as np
from threading import Lock, Event, Thread
from typing import Optional, Dict, Any, List

from video_cam.storage_manager import get_ros2_ws_directory

from .config import (
    NODE_LOOP_PERIOD,
    DEFAULT_USE_REAL_CAMERA,
    DEFAULT_MIN_ALTITUDE_AGL,
    DEFAULT_RESOLUTION,
    DEFAULT_ROTATE_180,
    CAMERA_IP,
    CONTROL_PORT,
    MEDIA_PORT,
    HTTP_TIMEOUT_SECONDS,
    CAPTURE_TIMEOUT_SECONDS,
    MIN_FREE_SPACE_MB,
    PHOTO_RESOLUTIONS,
)
from .camera_interface import CameraInterface, CameraConnectionError
from .storage_manager import StorageManager
from .pipeline_orchestrator import PipelineOrchestrator


class SIYINode(Node):
    """ROS2 node wrapper for SIYI camera pipeline."""
    
    def __init__(self):
        super().__init__('siyi')
        install_wallclock_logging(self)

        # Parameters
        self.declare_parameter('use_real_camera', DEFAULT_USE_REAL_CAMERA)
        self.declare_parameter('min_altitude_agl', DEFAULT_MIN_ALTITUDE_AGL)
        self.declare_parameter('camera_ip', CAMERA_IP)
        self.declare_parameter('ctrl_port', CONTROL_PORT)
        self.declare_parameter('media_port', MEDIA_PORT)
        self.declare_parameter('http_timeout_sec', HTTP_TIMEOUT_SECONDS)
        self.declare_parameter('capture_timeout_sec', CAPTURE_TIMEOUT_SECONDS)
        self.declare_parameter('min_free_space_mb', MIN_FREE_SPACE_MB)
        self.declare_parameter('resolution', DEFAULT_RESOLUTION)
        self.declare_parameter('rotate_180', DEFAULT_ROTATE_180)

        self.use_real_camera = self.get_parameter('use_real_camera').value
        self.altitude_threshold = self.get_parameter('min_altitude_agl').value
        self.camera_ip = self.get_parameter('camera_ip').value
        self.ctrl_port = self.get_parameter('ctrl_port').value
        self.media_port = self.get_parameter('media_port').value
        self.http_timeout = self.get_parameter('http_timeout_sec').value
        self.capture_timeout = self.get_parameter('capture_timeout_sec').value
        self.min_free_space_mb = self.get_parameter('min_free_space_mb').value
        self.resolution = self.get_parameter('resolution').value
        self.rotate_180 = self.get_parameter('rotate_180').value
        if self.resolution not in PHOTO_RESOLUTIONS:
            self.get_logger().warn(
                f"Invalid resolution '{self.resolution}', falling back to '{DEFAULT_RESOLUTION}'")
            self.resolution = DEFAULT_RESOLUTION
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.status_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
        self.camera_status_pub = self.create_publisher(String, '/camera/status', 10)
        self.disk_status_pub = self.create_publisher(Float64, '/camera/disk_free_mb', 10)
        
        # Subscribers
        self.create_subscription(Bool, '/camera/trigger', self.camera_trigger_callback, 10)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.altitude_callback, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/camera/command', self.camera_command_callback, 10)
        if not self.use_real_camera:
            self.create_subscription(Image, '/camera/image', self.sim_image_callback, 1)

        # State
        self.camera_enabled = True
        self.camera_control_lock = Lock()
        self.capture_requested = Event()
        self.latest_image_msg: Optional[Image] = None
        self.latest_gps = None
        self.bridge = CvBridge()

        # Components
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
                rotate_180=self.rotate_180,
                logger=self.get_logger()
            )
            self.pipeline.set_resolution(self.resolution)
            self.pipeline.initialize_sd_card()
        else:
            self.camera = None
            self.pipeline = None
            self.get_logger().info("Simulation mode: Waiting for images on /camera/image...")
            self._send_status("Simulation camera initialized")
        
        # Main loop timer
        self.pipeline_timer = self.create_timer(NODE_LOOP_PERIOD, self._pipeline_loop)

        mode = 'SIMULATION' if not self.use_real_camera else 'REAL CAMERA'
        self.get_logger().info(f"SIYI pipeline initialized ({mode})")
    
    def _find_ros2_workspace(self) -> str:
        ros2_ws_dir = get_ros2_ws_directory()
        video_cam_dir = os.path.join(ros2_ws_dir, "video_cam")
        os.makedirs(video_cam_dir, exist_ok=True)
        return video_cam_dir

    def _pipeline_loop(self):
        # Main execution loop. Execute capture pipeline when triggered (in separate thread)
        # Check if camera is enabled (altitude check)
        if not self.camera_enabled:
            return
        
        # Handle capture requests
        if self.capture_requested.is_set():
            self.capture_requested.clear()
            self._handle_capture_request()

        self._publish_disk_status()
    
    def _handle_capture_request(self):
        """Handle capture request (executed in separate thread)"""
        if self.use_real_camera:
            # is_busy() reflects ONLY phases 1+2 (the UDP shutter + SD
            # index). Phase 3 (HTTP download) is intentionally not counted
            # as busy, so a new trigger arriving while the previous
            # capture is still downloading will proceed and overlap its
            # shutter with that download — this is the whole point of the
            # pipelined-capture design.
            if self.pipeline.is_busy():
                self.get_logger().warn("Previous capture still in shutter/index phase, skipping request")
                return

            # Execute pipeline in separate thread (non-blocking)
            capture_thread = Thread(target=self._execute_real_camera_capture, daemon=True)
            capture_thread.start()
        else:
            # Simulation mode: save current image
            self._execute_simulation_capture()
    
    def _execute_real_camera_capture(self):
        """Execute real camera capture pipeline with pipelined phases.

        Phases 1+2 (UDP shutter + SD index) run under camera_control_lock so
        they are serialized with other camera commands. Phase 3 (HTTP download
        on a separate port) runs OUTSIDE the lock, so the NEXT capture's
        shutter and the previous capture's download can overlap — cutting
        back-to-back capture latency roughly in half.
        """
        try:
            # Snapshot a GPS-based filename BEFORE the pipeline runs so that
            # phase 3 saves the downloaded image directly under its final
            # "<lat> , <lon>.jpg" name. This avoids a race where new_od
            # would see the file under its SD card name and enqueue it
            # before a post-hoc rename could happen. It also pins the
            # lat/lon to ~shutter time (phase 1 fires immediately after
            # this call) rather than to download-completion time, which
            # can be 2-3s later -- ~40-60m of drift at airspeed.
            gps_filename = self._generate_gps_filename()
            if gps_filename is None:
                self.get_logger().warn( "No valid GPS info /mavros/global_position/global; using the SD name")
                self._send_status("WARN: No GPS fix - image will not have lat/lon name")

            # Phases 1+2: UDP shutter + SD card indexing. Hold the camera
            # control lock here so gimbal/zoom commands can't race with
            # the UDP capture command.
            with self.camera_control_lock:
                file_info = self.pipeline.capture_and_index()

            if file_info is None:
                self._send_status("FAILED: Capture shutter/index error")
                self._publish_camera_status("FAILURE: Capture shutter/index error")
                return

            # Phase 3: HTTP download on port 82. Intentionally NOT holding
            # camera_control_lock here so that (a) gimbal/zoom commands can
            # run concurrently and (b) the next capture's phases 1+2 can
            # overlap with this download.
            result = self.pipeline.download_and_save(
                file_info, filename_override=gps_filename
            )

            if result is None:
                self._send_status("FAILED: Capture download error")
                self._publish_camera_status("FAILURE: Capture download error")
                return

            saved_path, img = result
            stats = self.pipeline.get_stats()
            self._send_status(
                f"SUCCESS: Captured {stats['resolution']} image #{stats['photo_count']}")
            # self._publish_camera_status(
            #     f"SUCCESS: {stats['resolution']} image captured")

            # Publish the in-memory decoded ndarray directly — no disk
            # re-read, no directory scan, no rename.
            self._publish_captured_image(img, saved_path)

        except Exception as e:
            self.get_logger().error(f"Pipeline error: {e}")
            self._send_status(f"FAILED: {e}")
            self._publish_camera_status(f"FAILURE: {e}")

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

    @staticmethod
    def _parse_switch(value: str) -> bool:
        normalized = value.strip().lower()
        if normalized in {'on', '1', 'true', 'enable', 'enabled'}:
            return True
        if normalized in {'off', '0', 'false', 'disable', 'disabled'}:
            return False
        raise ValueError("Switch value must be on/off (or true/false, 1/0)")

    def gps_cb(self, msg):
        """Callback to store the latest GPS data."""
        self.latest_gps = msg

    def _execute_camera_command(self, command: str, parameter: str) -> Dict[str, Any]:
        if not self.use_real_camera or self.camera is None:
            return self._result_payload(
                False,
                error='Camera command service requires use_real_camera=true',
            )

        cmd = command.strip().lower().replace('-', '_')
        param = parameter.strip()

        # Capture command is queued through the existing pipeline trigger path.
        if cmd == 'capture':
            resolution = '4K'
            if param:
                resolution = param.upper()
                if resolution not in PHOTO_RESOLUTIONS:
                    raise ValueError(
                        f"Invalid resolution '{resolution}'. Valid: {sorted(PHOTO_RESOLUTIONS.keys())}"
                    )

            if self.pipeline is not None:
                if self.pipeline.is_busy():
                    return self._result_payload(False, action='capture', error='Pipeline is busy')
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
        """Handle camera control commands from topic."""
        try:
            # Try parsing as JSON first
            try:
                data = json.loads(msg.data)
                command = data.get('command', '')
                parameter = data.get('parameter', '')
            except json.JSONDecodeError:
                # Fallback to simple format block: "command parameter"
                parts = msg.data.split(' ', 1)
                command = parts[0]
                parameter = parts[1] if len(parts) > 1 else ''
            
            with self.camera_control_lock:
                result = self._execute_camera_command(command, parameter)

            success = bool(result.get('ok', False))
            
            if success:
                self._publish_camera_status(f"CMD SUCCESS: {command}")
                # self.get_logger().info(f"Camera command success: {command}")
            else:
                self._publish_camera_status(f"CMD FAILED: {command} - {result.get('error', 'Unknown')}")
                self.get_logger().error(f"Camera command failed: {command} - {result.get('error', 'Unknown')}")

        except (ValueError, CameraConnectionError, OSError) as exc:
            self._publish_camera_status(f"CMD ERROR: {msg.data}")
            self.get_logger().error(f"Camera command error for {msg.data}: {exc}")
        except Exception as exc:
            self.get_logger().error(f"Unexpected camera command error for {msg.data}: {exc}")
            self._publish_camera_status(f"CMD ERROR: {msg.data}")
    
    def _execute_simulation_capture(self):
        """Execute simulation capture (save current image)"""
        if self.latest_image_msg is None:
            self.get_logger().warn("Trigger received but no simulation image available")
            return
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image_msg, 'bgr8')

            # Rotate image to fix upside-down physical mounting
            if self.rotate_180:
                cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
            
            # Save to mapping directory
            mapping_dir = self.storage.get_mapping_dir()
            filename = self._generate_gps_filename()
            if filename is None:
                filename = f"sim_{time.strftime('%Y%m%d-%H%M%S')}.jpg"
            filepath = os.path.join(mapping_dir, filename)
            
            cv2.imwrite(filepath, cv_image)
            self.get_logger().info(f"Simulation photo saved: {filepath}")
            self._send_status(f"Simulation photo captured: {time.strftime('%Y%m%d-%H%M%S')}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save simulation image: {e}")

    def _generate_gps_filename(self) -> Optional[str]:
        """Build a '<lat> , <lon>.jpg' filename from the latest cached fix.

        IMPORTANT: the '" , "' delimiter (space-comma-space) is parsed by
        detection/new_od.py on the consuming side via `name.split(' , ')`,
        which then does `float()` on each half to populate
        ImageResult.latitude / ImageResult.longitude. Do NOT change the
        delimiter or append anything else to the stem without updating
        new_od's parser in lockstep, or main_controller will start placing
        detection waypoints at (0.0, 0.0).
        """
        if self.latest_gps is None:
            return None

        try:
            lat = float(self.latest_gps.latitude)
            lon = float(self.latest_gps.longitude)
            self.get_logger().info(f"{self.latest_gps.latitude}, {self.latest_gps.longitude}")
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(
                f"Cached NavSatFix has invalid coordinates: {exc}")
            return None

        # Null Island: the FCU publishes (0.0, 0.0) before GPS lock, so
        # treat it as "no fix" rather than baking it into a filename.
        if lat == 0.0 and lon == 0.0:
            self.get_logger().warn(
                "Cached GPS fix is (0.0, 0.0) - treating as no fix")
            return None

        return f"{lat:.6f} , {lon:.6f}.jpg"

    def _publish_captured_image(self, img: np.ndarray, saved_path: str):
        """Publish an already-decoded image to ROS /image_raw.

        The pipeline's download_and_save() hands back the decoded ndarray
        directly, so this method publishes it without a round-trip through
        cv2.imread(). Saves one full-resolution disk read per capture.

        Args:
            img: Decoded BGR image (as returned by download_and_save).
            saved_path: Absolute path the pipeline wrote — used only for
                the log line and the ROS header's frame_id context.
        """
        try:
            if img is None:
                self.get_logger().warn(
                    "Pipeline handed back a None image; "
                    "nothing to publish on /image_raw")
                return

            msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.image_pub.publish(msg)
            # self.get_logger().info(
            #     f"Published image to image_raw: {os.path.basename(saved_path)}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish captured image: {e}")
    
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
    
    def altitude_callback(self, msg: Float64):
        """Handle altitude updates for camera enable/disable"""
        current_alt = msg.data
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
        msg.severity = 6
        msg.text = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status: {text}")
    
    def _publish_camera_status(self, text: str):
        """Publish camera-specific status"""
        msg = String()
        msg.data = text
        self.camera_status_pub.publish(msg)
    
    def shutdown(self):
        """Proper shutdown handler"""
        # try:
        #     self.get_logger().info("Shutting down SIYI pipeline...")
        # except Exception:
        #     pass
        
        # Close camera interface
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