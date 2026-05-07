#!/usr/bin/env python3
"""SIYI Camera ROS2 Node."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Bool, Float64, String
from mavros_msgs.msg import StatusText
from rclpy.qos import qos_profile_sensor_data

import json
import os
import cv2
import time
import numpy as np
from threading import Lock, Event, Thread
from typing import Optional, Dict, Any, List

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except Exception as e:
    print(f"Warning: cv_bridge import failed: {e}")
    CV_BRIDGE_AVAILABLE = False

from .config import (
    NODE_LOOP_PERIOD,
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
    """ROS2 node wrapper for SIYI camera pipeline."""
    
    def __init__(self):
        super().__init__('siyi_unified_pipeline')
        
        # Parameters
        self.declare_parameter('use_real_camera', DEFAULT_USE_REAL_CAMERA)
        self.declare_parameter('min_altitude_agl', DEFAULT_MIN_ALTITUDE_AGL)
        self.declare_parameter('camera_ip', DEFAULT_CAMERA_IP)
        self.declare_parameter('ctrl_port', DEFAULT_CTRL_PORT)
        self.declare_parameter('media_port', DEFAULT_MEDIA_PORT)
        self.declare_parameter('http_timeout_sec', DEFAULT_HTTP_TIMEOUT)
        self.declare_parameter('capture_timeout_sec', DEFAULT_CAPTURE_TIMEOUT)
        self.declare_parameter('min_free_space_mb', DEFAULT_MIN_FREE_SPACE_MB)

        self.use_real_camera = self.get_parameter('use_real_camera').value
        self.altitude_threshold = self.get_parameter('min_altitude_agl').value
        self.camera_ip = self.get_parameter('camera_ip').value
        self.ctrl_port = self.get_parameter('ctrl_port').value
        self.media_port = self.get_parameter('media_port').value
        self.http_timeout = self.get_parameter('http_timeout_sec').value
        self.capture_timeout = self.get_parameter('capture_timeout_sec').value
        self.min_free_space_mb = self.get_parameter('min_free_space_mb').value
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.status_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
        self.camera_status_pub = self.create_publisher(String, '/camera/status', 10)
        self.disk_status_pub = self.create_publisher(Float64, '/camera/disk_free_mb', 10)
        
        # Subscribers
        self.create_subscription(Bool, '/camera/trigger', self.camera_trigger_callback, 10)
        self.create_subscription(String, '/camera/set_resolution', self.set_resolution_callback, 10)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.altitude_callback, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/camera/command', self.camera_command_callback, 10)
        if not self.use_real_camera:
            self.create_subscription(Image, '/camera/image', self.sim_image_callback, 1)
        
        # State
        self.camera_enabled = True
        self.config_lock = Lock()
        self.camera_control_lock = Lock()
        self.capture_requested = Event()
        self.latest_image_msg: Optional[Image] = None
        self.latest_gps = None
        self.bridge = CvBridge() if CV_BRIDGE_AVAILABLE else None
        if not CV_BRIDGE_AVAILABLE:
            self.get_logger().warn("cv_bridge not available, using alternative conversion")
        
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
                logger=self.get_logger()
            )
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
        current_file = os.path.abspath(__file__)
        current_dir = os.path.dirname(current_file)
        
        # Navigate up to find ros2_ws (look for install/ or src/ directories)
        search_dir = current_dir
        ros2_ws_dir = None
        
        for _ in range(10):  # Limit search depth
            if os.path.exists(os.path.join(search_dir, "install")) or os.path.exists(os.path.join(search_dir, "src")):
                if os.path.exists(os.path.join(search_dir, "install")) and os.path.exists(os.path.join(search_dir, "src")):
                    ros2_ws_dir = search_dir
                    break
                parent = os.path.dirname(search_dir)
                if os.path.exists(os.path.join(parent, "install")) and os.path.exists(os.path.join(parent, "src")):
                    ros2_ws_dir = parent
                    break
            search_dir = os.path.dirname(search_dir)
            if search_dir == "/":
                break

        
        if ros2_ws_dir and os.path.exists(os.path.join(ros2_ws_dir, "src")):
            ros2_ws_dir = os.path.join(ros2_ws_dir, "src")
            
        # Fallback: construct path directly
        if ros2_ws_dir is None:
            ros2_ws_dir = "/astra/ros2_ws/src"
        
        video_cam_dir = os.path.join(ros2_ws_dir, "video_cam")
        os.makedirs(video_cam_dir, exist_ok=True)
        
        return video_cam_dir

    def _pipeline_loop(self):
        # Main execution loop. Execute capture pipeline when triggered (in separate thread)
        # Check if camera is enabled (altitude check)
        with self.config_lock:
            camera_enabled = self.camera_enabled
        
        if not camera_enabled:
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
            # is_busy() reflects ONLY phases 1+2 (the UDP shutter + SD
            # index). Phase 3 (HTTP download) is intentionally not counted
            # as busy, so a new trigger arriving while the previous
            # capture is still downloading will proceed and overlap its
            # shutter with that download — this is the whole point of the
            # pipelined-capture design.
            if self.pipeline.is_busy():
                self.get_logger().warn(
                    "Previous capture still in shutter/index phase, skipping request")
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
            # self.get_logger().info(self._generate_gps_filename())
            if gps_filename is None:
                self.get_logger().warn(
                    "No valid GPS info "
                    "/mavros/global_position/global; using the SD name")
                self._send_status(
                    "WARN: No GPS fix - image will not have lat/lon name")

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

        except PipelineError as e:
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

        if cmd == 'laser_distance':
            distance_m = self.camera.request_laser_distance_measurement()
            return self._result_payload(
                distance_m is not None,
                action='laser_distance',
                distance_m=distance_m,
            )

        if cmd == 'laser_target':
            data = self.camera.request_laser_target_longitude_latitude()
            return self._result_payload(data is not None, action='laser_target', data=data)

        if cmd == 'laser_state_get':
            state = self.camera.get_laser_state()
            return self._result_payload(state is not None, action='laser_state_get', laser_on=state)

        if cmd == 'laser_state_set':
            if not param:
                raise ValueError("laser_state_set requires parameter on/off")
            enabled = self._parse_switch(param)
            ok = self.camera.set_laser_state(enabled)
            state = self.camera.get_laser_state()
            return self._result_payload(
                ok,
                action='laser_state_set',
                requested='on' if enabled else 'off',
                laser_on=state,
            )

        if cmd == 'laser_stream':
            if not param:
                enable = True
                frequency = 4
            else:
                parts = [item.strip() for item in param.split(',') if item.strip()]
                if len(parts) == 1:
                    if parts[0].isdigit():
                        enable = True
                        frequency = int(parts[0])
                    else:
                        enable = self._parse_switch(parts[0])
                        frequency = 4
                elif len(parts) == 2:
                    enable = self._parse_switch(parts[0])
                    frequency = int(parts[1])
                else:
                    raise ValueError("laser_stream format: enable,4 or disable")

            ok = self.camera.configure_laser_distance_stream(enable=enable, frequency=frequency)
            return self._result_payload(
                ok,
                action='laser_stream',
                enable=enable,
                frequency=frequency,
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
            if self.bridge is not None:
                cv_image = self.bridge.imgmsg_to_cv2(self.latest_image_msg, 'bgr8')
            else:
                cv_image = self._imgmsg_to_cv2_manual(self.latest_image_msg, 'bgr8')
            
            # Rotate image to fix upside-down physical mounting
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

            if self.bridge is not None:
                msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            else:
                msg = self._cv2_to_imgmsg_manual(img, 'bgr8')

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
        msg.severity = 6
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