"""
siyi_emulator/node.py -- PURPOSE: the ROS 2 entry point of the emulator (executable ``siyi_emulator``).
It declares one ROS parameter per EmulatorConfig setting (same names, see the README table), starts
the emulator core (emulator.py), forwards its log lines to the ROS logger, and publishes two
simulator-only topics that help the demo panel and debugging:

    /sim/camera/photo_info      std_msgs/String (JSON), one message per saved photo: SD name, URL, camera
                                pose, gimbal angles, zoom, field of view (reliable, transient local, depth 10)
    /sim/camera/emulator_state  std_msgs/String (JSON) at publish_state_hz: model, gimbal, zoom, Gazebo health

The emulator itself talks to video_cam over UDP/HTTP and to Gazebo over gz-transport, not over ROS.
Without ROS (rclpy missing, or with --standalone) the same executable runs the plain command line of
emulator.py, so unit tests and quick experiments need no ROS at all:

    ros2 run siyi_emulator siyi_emulator --ros-args -p camera_model:=a8mini -p strict:=true
    python3 -m siyi_emulator.node --standalone --bind-ip 127.0.0.3 --no-gazebo
"""

from __future__ import annotations

import json
import logging
import sys
from typing import List, Optional

from . import emulator as emu

# Short descriptions shown by `ros2 param describe /siyi_emulator <name>`.
PARAM_DOCS = {
    "camera_model": "Camera to emulate: zr10 (2560x1440 photos, DEFAULT, what last year's code targets) or a8mini (3840x2160).",
    "strict": "true = stay silent on 0x04/0x06/0x16/0x18/0x19 like a real A8 mini; false = answer everything.",
    "max_zoom": "> 0 overrides the model's maximum zoom (x). a8mini default 1.0 (no zoom in 4K), zr10 30.0.",
    "photo_width": "> 0 overrides the model's photo width in pixels.",
    "photo_height": "> 0 overrides the model's photo height in pixels.",
    "camera_ip": "Address clients use for the camera (video_cam default 192.168.144.25).",
    "bind_ip": "Address to listen on. Empty = camera_ip if it exists on this computer, else 127.0.0.1.",
    "ctrl_port": "UDP port of the SIYI SDK (37260).",
    "media_port": "TCP port of the media web server (82; needs net.ipv4.ip_unprivileged_port_start=80).",
    "media_url_host": "host[:port] written into photo URLs. Empty = the Host header of each request.",
    "sd_card_dir": "Folder of the emulated SD card (photos go to <sd_card_dir>/<photo_dir_name>/IMG_NNNN.jpg).",
    "photo_dir_name": "The one photo directory reported by getdirectories (real cameras: 101SIYI_IMG).",
    "clear_sd_on_start": "true = start every run with an empty card.",
    "restart_numbering_after_format": "true = IMG_0001 again after 0x48 format (can confuse video_cam, see README).",
    "sd_card_present": "false = simulate 'no TF card' (0x0A record_sta 2, photos answer 0x0B info_type 1).",
    "jpeg_quality": "JPEG quality of the photos (1-100).",
    "write_sidecar": "true = write IMG_NNNN.json (camera pose, zoom, FOV) next to each photo.",
    "photo_timeout_s": "Seconds to wait for the Gazebo photo frame after triggering.",
    "retrigger_after_s": "Seconds after which the photo trigger is re-sent (at most twice) if no frame came.",
    "feedback_deadline_s": "Push 0x0B 'photo OK' no later than this after 0x0C (video_cam waits 0.8 s).",
    "test_pattern_fallback": "true = save a labelled test pattern when Gazebo gives no frame (else the photo is lost).",
    "initial_mode": "Gimbal mode at start: lock, follow (default) or fpv.",
    "initial_yaw_deg": "Gimbal yaw at start, deg, + = left.",
    "initial_pitch_deg": "Gimbal pitch at start, deg, + = up, -90 = straight down (default: survey camera looks down).",
    "max_slew_dps": "Gimbal slew-rate limit, deg/s.",
    "max_rate_dps": "What a 0x07 speed of 100 means, deg/s.",
    "rate_yaw_sign": "Sign that turns a SIYI yaw speed (+ = right) into the joint frame (+ = left). -1.",
    "stabilize": "true = keep the camera level when the drone tilts (uses /sim/drone/odometry).",
    "control_rate_hz": "How often the gimbal joint targets are sent to Gazebo, Hz.",
    "zoom_speed_x_per_s": "Zoom travel speed, x per second (0x18 reports the zoom while it travels).",
    "use_gazebo": "false = no Gazebo connection (SDK + media server only, test-pattern photos).",
    "gz_partition": "GZ_PARTITION for the Gazebo connection. Empty = keep the environment variable.",
    "gz_yaw_cmd_topic": "gz topic, gz.msgs.Double, yaw joint target (rad).",
    "gz_roll_cmd_topic": "gz topic, gz.msgs.Double, roll joint target (rad).",
    "gz_pitch_cmd_topic": "gz topic, gz.msgs.Double, pitch joint target (rad).",
    "gz_joint_states_topic": "gz topic, gz.msgs.Model from JointStatePublisher (measured joints).",
    "gz_odometry_topic": "gz topic, gz.msgs.Odometry of the drone (attitude for stabilisation).",
    "gz_photo_image_topic": "gz topic, gz.msgs.Image of the triggered photo camera.",
    "gz_photo_camera_info_topic": "gz topic, gz.msgs.CameraInfo of the photo camera (metadata only).",
    "gz_photo_trigger_topic": "gz topic, gz.msgs.Boolean that makes the photo camera render one frame.",
    "yaw_joint": "Name of the gimbal yaw joint in the drone SDF.",
    "roll_joint": "Name of the gimbal roll joint in the drone SDF.",
    "pitch_joint": "Name of the gimbal pitch joint in the drone SDF.",
    "optical_zoom_topics": "Comma list of CameraZoomPlugin topics (gz.msgs.Double). Empty = digital zoom only.",
    "hfov_rad": "Horizontal FOV of the Gazebo photo camera at 1x (rad); 1.4137 = 81 deg.",
    "optical_zoom_ref_hfov_rad": "CameraZoomPlugin's built-in reference HFOV (2.0 rad); used to compute its command.",
    "log_commands": "true = log every SDK command and reply at INFO level.",
    "publish_photo_info": "true = publish /sim/camera/photo_info (JSON) for every saved photo.",
    "photo_info_topic": "ROS topic of the photo metadata.",
    "publish_state_hz": "Rate of /sim/camera/emulator_state (JSON); 0 disables it.",
    "state_topic": "ROS topic of the emulator state.",
}

EXTRA_PARAMS = {
    "publish_photo_info": True,
    "photo_info_topic": "/sim/camera/photo_info",
    "publish_state_hz": 2.0,
    "state_topic": "/sim/camera/emulator_state",
}


def _strip_flag(argv: List[str], *flags: str) -> List[str]:
    return [a for a in argv if a not in flags]


def main(args: Optional[List[str]] = None) -> int:
    """Entry point: ROS node when rclpy is available, plain command line otherwise (or --standalone)."""
    argv = list(sys.argv[1:] if args is None else args)
    if "--standalone" in argv or "--no-ros" in argv:
        return emu.main(_strip_flag(argv, "--standalone", "--no-ros"))
    try:
        import rclpy  # noqa: F401  (only checking that ROS is there)
    except ImportError:
        print("[siyi_emulator] rclpy not found: running without ROS (same as --standalone)", file=sys.stderr)
        return emu.main(argv)
    return _run_ros(argv)


def _run_ros(argv: List[str]) -> int:
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=["siyi_emulator"] + argv)
    node = None
    try:
        node = _make_node()
    except (emu.EmulatorStartError, ValueError) as exc:   # port problem, or e.g. unknown camera_model
        rclpy.logging.get_logger("siyi_emulator").fatal(f"cannot start the SIYI emulator: {exc}")
        if rclpy.ok():
            rclpy.shutdown()
        return 2
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


class _RosLogHandler(logging.Handler):
    """Forwards Python ``logging`` records of the emulator core to a ROS logger."""

    def __init__(self, ros_logger) -> None:
        super().__init__()
        self.ros_logger = ros_logger

    def emit(self, record: logging.LogRecord) -> None:
        try:
            text = record.getMessage()
            if record.name not in ("siyi_emulator",):
                text = f"[{record.name.split('.')[-1]}] {text}"
            if record.levelno >= logging.ERROR:
                self.ros_logger.error(text)
            elif record.levelno >= logging.WARNING:
                self.ros_logger.warning(text)
            elif record.levelno >= logging.INFO:
                self.ros_logger.info(text)
            else:
                self.ros_logger.debug(text)
        except Exception:  # pragma: no cover - logging must never raise
            pass


def _make_node():
    """Build the ROS node (separate function so the rclpy import stays optional)."""
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    class SiyiEmulatorNode(Node):
        """ROS 2 node 'siyi_emulator': parameters -> EmulatorConfig -> SiyiEmulator."""

        def __init__(self) -> None:
            super().__init__("siyi_emulator")
            defaults = emu.EmulatorConfig()
            values = {}
            # dynamic_typing: "-p initial_pitch_deg:=-90" arrives as an INTEGER; accept it and convert.
            for name in emu.EmulatorConfig.field_names() + list(EXTRA_PARAMS):
                default = EXTRA_PARAMS[name] if name in EXTRA_PARAMS else getattr(defaults, name)
                desc = ParameterDescriptor(description=PARAM_DOCS.get(name, ""), dynamic_typing=True)
                values[name] = self.declare_parameter(name, default, desc).value
            extras = {k: values.pop(k) for k in EXTRA_PARAMS}
            config, _ = defaults.updated(values)

            self.py_log = logging.getLogger("siyi_emulator")
            self.py_log.handlers = [_RosLogHandler(self.get_logger())]
            self.py_log.propagate = False
            debug = self.get_logger().get_effective_level() <= 10   # rclpy DEBUG = 10
            self.py_log.setLevel(logging.DEBUG if debug else logging.INFO)

            self.emulator = emu.SiyiEmulator(config, logger=self.py_log)
            self.photo_pub = None
            if emu._coerce(extras["publish_photo_info"], bool):
                qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.photo_pub = self.create_publisher(String, str(extras["photo_info_topic"]), qos)
                self.emulator.photo_listeners.append(self._publish_photo)
            self.state_pub = None
            rate = float(extras["publish_state_hz"])
            self.emulator.start()          # raises EmulatorStartError with a helpful message
            if rate > 0.0:
                self.state_pub = self.create_publisher(String, str(extras["state_topic"]), 10)
                self.create_timer(1.0 / rate, self._publish_state)

        def _publish_photo(self, event: emu.PhotoEvent) -> None:
            payload = dict(event.metadata)
            payload.update({
                "sd_name": event.photo.name, "sd_path": event.photo.path, "size_bytes": event.photo.size,
                "url": f"http://{self.emulator.cfg.camera_ip}:{self.emulator.media_port}/{event.photo.rel_path}",
            })
            msg = String()
            msg.data = json.dumps(payload, default=str)
            self.photo_pub.publish(msg)

        def _publish_state(self) -> None:
            msg = String()
            msg.data = json.dumps(self.emulator.status(), default=str)
            self.state_pub.publish(msg)

        def shutdown(self) -> None:
            self.emulator.stop()

    return SiyiEmulatorNode()


if __name__ == "__main__":
    raise SystemExit(main())
