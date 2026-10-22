"""
siyi_emulator/gz_link.py -- PURPOSE: the emulator's only connection to Gazebo, through the gz-transport
Python bindings (gz.transport13 + gz.msgs10, apt packages python3-gz-transport13 / python3-gz-msgs10).
No ROS bridge is needed. Topic names are parameters; the defaults are the simulation contract's:

    publishes   /sim/gimbal/yaw/cmd_pos, /sim/gimbal/roll/cmd_pos, /sim/gimbal/pitch/cmd_pos
                    gz.msgs.Double, joint targets in radians (JointPositionController)
                /sim/camera/photo/trigger      gz.msgs.Boolean, one message = one rendered photo frame
                <optical zoom topics>          gz.msgs.Double, optional, for ardupilot_gazebo's CameraZoomPlugin
    subscribes  /sim/gimbal/joint_states       gz.msgs.Model (JointStatePublisher): measured joint angles
                /sim/drone/odometry            gz.msgs.Odometry (OdometryPublisher): drone attitude, ENU
                /sim/camera/photo/image        gz.msgs.Image: the rendered photo
                /sim/camera/photo/camera_info  gz.msgs.CameraInfo: lens focal length (for the metadata)

Received data is handed to callbacks as small plain-Python records (OdomSample, JointSample,
RawImage, CameraInfoSample), so the rest of the emulator never touches protobuf.
:class:`NullGzLink` has the same interface and does nothing: used when Gazebo is disabled
(use_gazebo:=false), when the bindings are missing, and in unit tests.
"""

from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple


# ---------------------------------------------------------------------------------------------
# Plain records handed to the emulator
# ---------------------------------------------------------------------------------------------
@dataclass
class GzTopics:
    """Every gz topic the emulator uses (defaults = the simulation contract)."""

    yaw_cmd: str = "/sim/gimbal/yaw/cmd_pos"
    roll_cmd: str = "/sim/gimbal/roll/cmd_pos"
    pitch_cmd: str = "/sim/gimbal/pitch/cmd_pos"
    joint_states: str = "/sim/gimbal/joint_states"
    odometry: str = "/sim/drone/odometry"
    photo_image: str = "/sim/camera/photo/image"
    photo_camera_info: str = "/sim/camera/photo/camera_info"
    photo_trigger: str = "/sim/camera/photo/trigger"
    optical_zoom: Tuple[str, ...] = ()


@dataclass
class OdomSample:
    """Drone pose from /sim/drone/odometry. stamp = sim time in s (None if absent); ENU metres; quat x,y,z,w."""

    stamp: Optional[float]
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]
    received: float = field(default_factory=time.monotonic)


@dataclass
class JointSample:
    """Measured gimbal joints. positions/velocities keyed 'yaw', 'roll', 'pitch' (rad, rad/s)."""

    stamp: Optional[float]
    positions: Dict[str, float]
    velocities: Dict[str, float]
    received: float = field(default_factory=time.monotonic)


@dataclass
class RawImage:
    """A rendered photo frame, still as raw bytes (converted to OpenCV later, off the gz thread)."""

    stamp: Optional[float]
    width: int
    height: int
    step: int
    pixel_format: int
    data: bytes
    received: float = field(default_factory=time.monotonic)


@dataclass
class CameraInfoSample:
    """Lens data of the photo camera: image size and focal length fx (pixels)."""

    width: int
    height: int
    fx: float


def _stamp_of(msg) -> Optional[float]:
    """Sim time (s) of a gz message header, or None when it has no stamp."""
    try:
        if not msg.HasField("header") or not msg.header.HasField("stamp"):
            return None
        return msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    except Exception:
        return None


def match_joint_names(available: Sequence[str], wanted: Dict[str, str]) -> Dict[str, str]:
    """Map our roles ('yaw', 'roll', 'pitch') to joint names found in a JointStatePublisher message.

    A wanted name matches exactly, or as the last part of a scoped name ('iris::gimbal_yaw',
    'model/gimbal_yaw'), or (last resort) as a substring. Roles that do not match are left out.
    """
    found: Dict[str, str] = {}
    for role, name in wanted.items():
        if not name:
            continue
        exact = [a for a in available if a == name]
        scoped = [a for a in available if a.endswith("::" + name) or a.endswith("/" + name)]
        loose = [a for a in available if name in a]
        for candidates in (exact, scoped, loose):
            if candidates:
                found[role] = candidates[0]
                break
    return found


# ---------------------------------------------------------------------------------------------
# Null link (no Gazebo)
# ---------------------------------------------------------------------------------------------
class NullGzLink:
    """Same interface as :class:`GzLink`, does nothing. ``available`` is False."""

    available = False

    def __init__(self, reason: str = "Gazebo disabled") -> None:
        self.reason = reason
        self.on_odometry: Optional[Callable[[OdomSample], None]] = None
        self.on_joint_states: Optional[Callable[[JointSample], None]] = None
        self.on_photo: Optional[Callable[[RawImage], None]] = None
        self.on_camera_info: Optional[Callable[[CameraInfoSample], None]] = None
        self.triggers_sent = 0

    def start(self) -> bool:
        """Nothing to start; returns False (not connected)."""
        return False

    def stop(self) -> None:
        """Nothing to stop."""

    def publish_joint_targets(self, yaw: float, roll: float, pitch: float) -> None:
        """Ignored."""

    def trigger_photo(self) -> bool:
        """Ignored; returns False (no frame will come)."""
        self.triggers_sent += 1
        return False

    def publish_optical_zoom(self, value: float) -> None:
        """Ignored."""

    def trigger_has_subscribers(self) -> Optional[bool]:
        """Unknown without Gazebo."""
        return None


# ---------------------------------------------------------------------------------------------
# Real link
# ---------------------------------------------------------------------------------------------
class GzLink(NullGzLink):
    """gz-transport connection.

    Args:
        topics:       :class:`GzTopics`.
        joint_names:  {'yaw': name, 'roll': name, 'pitch': name} as written in the drone SDF.
        partition:    GZ_PARTITION to use ('' = keep the environment's).
        logger:       ``logging.Logger``.

    Call :meth:`start`; it returns False (and sets ``reason``) when the bindings are missing.
    Set the ``on_*`` callbacks before starting. Callbacks run on gz-transport threads: keep them short.
    """

    def __init__(self, topics: Optional[GzTopics] = None, joint_names: Optional[Dict[str, str]] = None,
                 partition: str = "", logger: Optional[logging.Logger] = None) -> None:
        super().__init__(reason="not started")
        self.topics = topics or GzTopics()
        self.joint_names = joint_names or {"yaw": "gimbal_yaw", "roll": "gimbal_roll", "pitch": "gimbal_pitch"}
        self.partition = (partition or "").strip()
        self.log = logger or logging.getLogger("siyi_emulator.gz")
        self.available = False
        self._node = None
        self._pub: Dict[str, object] = {}
        self._zoom_pubs: List[object] = []
        self._msgs: Dict[str, type] = {}
        self._joint_map: Optional[Dict[str, str]] = None
        self._joint_warned = False

    def start(self) -> bool:
        """Create the gz node, advertise and subscribe. Returns True when connected to gz-transport."""
        try:
            from gz.transport13 import Node
            from gz.msgs10.boolean_pb2 import Boolean
            from gz.msgs10.camera_info_pb2 import CameraInfo
            from gz.msgs10.double_pb2 import Double
            from gz.msgs10.image_pb2 import Image
            from gz.msgs10.model_pb2 import Model
            from gz.msgs10.odometry_pb2 import Odometry
        except ImportError as exc:
            self.reason = (f"Gazebo Python bindings not found ({exc}). Install python3-gz-transport13 and "
                           f"python3-gz-msgs10, or run with use_gazebo:=false")
            return False
        if self.partition:
            os.environ["GZ_PARTITION"] = self.partition
        self._msgs = {"Double": Double, "Boolean": Boolean}
        node = Node()
        t = self.topics
        self._pub = {
            "yaw": node.advertise(t.yaw_cmd, Double),
            "roll": node.advertise(t.roll_cmd, Double),
            "pitch": node.advertise(t.pitch_cmd, Double),
            "trigger": node.advertise(t.photo_trigger, Boolean),
        }
        self._zoom_pubs = [node.advertise(topic, Double) for topic in t.optical_zoom if topic]
        ok = True
        ok &= bool(node.subscribe(Odometry, t.odometry, self._on_odometry_msg))
        ok &= bool(node.subscribe(Model, t.joint_states, self._on_joint_msg))
        ok &= bool(node.subscribe(Image, t.photo_image, self._on_image_msg))
        if t.photo_camera_info:
            ok &= bool(node.subscribe(CameraInfo, t.photo_camera_info, self._on_camera_info_msg))
        self._node = node
        self.available = True
        self.reason = "connected" if ok else "connected (some subscriptions failed)"
        self.log.info("Gazebo link up (GZ_PARTITION=%s): gimbal -> %s, %s, %s; photo trigger -> %s; "
                      "listening to %s, %s, %s", os.environ.get("GZ_PARTITION", "<default>"),
                      t.yaw_cmd, t.roll_cmd, t.pitch_cmd, t.photo_trigger, t.odometry, t.joint_states, t.photo_image)
        return True

    def stop(self) -> None:
        """Drop the node (gz-transport cleans up its threads)."""
        self._node = None
        self._pub = {}
        self._zoom_pubs = []
        self.available = False

    # ------------------------------------------------------------------ publishing
    def publish_joint_targets(self, yaw: float, roll: float, pitch: float) -> None:
        """Send the three joint targets (radians, contract joint convention)."""
        if not self.available:
            return
        double = self._msgs["Double"]
        self._pub["yaw"].publish(double(data=float(yaw)))
        self._pub["roll"].publish(double(data=float(roll)))
        self._pub["pitch"].publish(double(data=float(pitch)))

    def trigger_photo(self) -> bool:
        """Ask the triggered photo camera for ONE frame. Returns True when the message was sent."""
        self.triggers_sent += 1
        if not self.available:
            return False
        return bool(self._pub["trigger"].publish(self._msgs["Boolean"](data=True)))

    def publish_optical_zoom(self, value: float) -> None:
        """Send a CameraZoomPlugin command (see photo.zoom_plugin_command) to every zoom topic."""
        if not self.available:
            return
        for pub in self._zoom_pubs:
            pub.publish(self._msgs["Double"](data=float(value)))

    def trigger_has_subscribers(self) -> Optional[bool]:
        """True/False when gz-transport can tell whether the photo camera listens to the trigger."""
        pub = self._pub.get("trigger")
        if pub is None:
            return None
        try:
            return bool(pub.has_connections())
        except Exception:
            return None

    # ------------------------------------------------------------------ callbacks (gz threads)
    def _on_odometry_msg(self, msg) -> None:
        if self.on_odometry is None:
            return
        p, q = msg.pose.position, msg.pose.orientation
        self.on_odometry(OdomSample(_stamp_of(msg), (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))

    def _on_joint_msg(self, msg) -> None:
        if self.on_joint_states is None:
            return
        names = [j.name for j in msg.joint]
        if self._joint_map is None or any(n not in names for n in self._joint_map.values()):
            self._joint_map = match_joint_names(names, self.joint_names)
            missing = [r for r in ("yaw", "roll", "pitch") if r not in self._joint_map]
            if missing and not self._joint_warned:
                self._joint_warned = True
                self.log.warning("joint_states has no joint for %s (wanted %s; message has %s). Set the "
                                 "yaw_joint/roll_joint/pitch_joint parameters. Using commanded angles instead.",
                                 missing, self.joint_names, names)
        positions, velocities = {}, {}
        by_name = {j.name: j for j in msg.joint}
        for role, name in self._joint_map.items():
            joint = by_name.get(name)
            if joint is not None and joint.HasField("axis1"):
                positions[role] = joint.axis1.position
                velocities[role] = joint.axis1.velocity
        if positions:
            self.on_joint_states(JointSample(_stamp_of(msg), positions, velocities))

    def _on_image_msg(self, msg) -> None:
        if self.on_photo is None:
            return
        self.on_photo(RawImage(_stamp_of(msg), int(msg.width), int(msg.height), int(msg.step),
                               int(msg.pixel_format_type), bytes(msg.data)))

    def _on_camera_info_msg(self, msg) -> None:
        if self.on_camera_info is None:
            return
        try:
            fx = float(msg.intrinsics.k[0]) if len(msg.intrinsics.k) >= 1 else 0.0
        except Exception:
            fx = 0.0
        self.on_camera_info(CameraInfoSample(int(msg.width), int(msg.height), fx))
