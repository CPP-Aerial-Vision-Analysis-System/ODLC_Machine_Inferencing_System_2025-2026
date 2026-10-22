"""
siyi_emulator/gimbal_control.py -- PURPOSE: the "brain" of the emulated 3-axis gimbal. It turns SDK
gimbal commands (0x0E angles, 0x41 single axis, 0x07 speed, 0x08 centre, lock/follow/FPV mode) into
the three joint angles Gazebo must reach, keeps the camera level when the drone tilts
(stabilisation, using the drone attitude from /sim/drone/odometry), limits the slew rate to about
90 deg/s, and computes the attitude the real camera would report in 0x0D.

Salvaged from the first twin prototype (ros2_ws/src/siyi_sim/siyi_sim/gimbal_model.py, whose unit
tests were ported to test/test_gimbal_control.py) and adapted to the SDK. numpy only, no ROS.

Frames and sign conventions (the simulation contract, same as the prototype's ARCHITECTURE.md s.4):
  * World frame: Gazebo ENU (x east, y north, z up). Drone body frame: FLU (x forward, y left, z up).
    ``R_world_body`` is the rotation matrix of the odometry quaternion.
  * Gimbal chain:  body --yaw joint (axis +Z)--> --roll joint (axis +X)--> --pitch joint (axis -Y)--> camera
    so ``R_body_cam = Rz(yaw_j) @ Rx(roll_j) @ Ry(-pitch_j)``; the camera looks along its +X.
  * Joint yaw + = camera turns LEFT (counter-clockwise seen from above); roll + = right side down;
    pitch + = tilt UP, -pi/2 = straight down.
  * SIYI ANGLES (0x0D/0x0E/0x41) use the same signs in degrees: yaw + = left, pitch + = up.
  * SIYI SPEEDS (0x07) use the opposite yaw sign: + = turn RIGHT ("swipe right" in the SDK text).
    ``rate_yaw_sign`` (default -1) converts a SIYI speed into the angle frame.
Joint tuples are always ordered (yaw, roll, pitch), the chain order.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple, Optional, Sequence, Tuple

import numpy as np

# Joint origins of the A8 mini chain (siyi_ros2 a8_mini.urdf.xacro), metres, each in its parent.
# They are only used to place the camera (a few cm) in the photo metadata.
YAW_JOINT_XYZ = (0.0, 0.0, -0.011)
ROLL_JOINT_XYZ = (-0.018, 0.0, -0.027)
PITCH_JOINT_XYZ = (0.027, -0.025, 0.001)
CAMERA_LINK_XYZ = (0.013, 0.025, 0.001)
# camera_link (x forward) -> camera_optical_link (z forward, x right, y down)
OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)
# Where the gimbal base sits in the drone body frame (under the nose). Metadata only.
DEFAULT_MOUNT_XYZ = (0.07, 0.0, -0.06)

MODE_LOCK = 0
MODE_FOLLOW = 1
MODE_FPV = 2
MODE_NAMES = {MODE_LOCK: "lock", MODE_FOLLOW: "follow", MODE_FPV: "fpv"}

RATE_CMD_MAX = 100   # 0x07 speeds are -100..100


# ---------------------------------------------------------------------------------------------
# Rotation helpers (3x3 numpy matrices)
# ---------------------------------------------------------------------------------------------
def rot_x(a: float) -> np.ndarray:
    """Rotation matrix about X by ``a`` radians."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    """Rotation matrix about Y by ``a`` radians."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    """Rotation matrix about Z by ``a`` radians."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """SDF/URDF convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll) (radians)."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def rot_from_quat(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Rotation matrix from a (x, y, z, w) quaternion. A zero quaternion gives the identity."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_from_rot(R: np.ndarray) -> Tuple[float, float, float, float]:
    """(x, y, z, w) unit quaternion from a rotation matrix (Shepperd's method)."""
    m = np.asarray(R, dtype=float)
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


def euler_zyx_from_rot(R: np.ndarray) -> Tuple[float, float, float]:
    """(roll, pitch, yaw) radians such that R = Rz(yaw) @ Ry(pitch) @ Rx(roll).

    Careful: for an x-forward frame a POSITIVE pitch here means nose DOWN. Use
    :func:`camera_heading_pitch_roll` for the SIYI style "pitch + = up".
    """
    m = np.asarray(R, dtype=float)
    sp_ = max(-1.0, min(1.0, -m[2, 0]))
    pitch = math.asin(sp_)
    if abs(sp_) < 0.999999:
        roll = math.atan2(m[2, 1], m[2, 2])
        yaw = math.atan2(m[1, 0], m[0, 0])
    else:  # looking straight up/down: put everything into yaw
        roll = 0.0
        yaw = math.atan2(-m[0, 1], m[1, 1])
    return roll, pitch, yaw


def wrap180(deg: float) -> float:
    """Wrap an angle in degrees into [-180, 180)."""
    return ((deg + 180.0) % 360.0) - 180.0


def enu_yaw_to_compass_deg(yaw_rad: float) -> float:
    """ENU yaw (radians, counter-clockwise from east) -> compass bearing (deg clockwise from north)."""
    return (90.0 - math.degrees(yaw_rad)) % 360.0


# ---------------------------------------------------------------------------------------------
# Kinematics of the yaw -> roll -> pitch chain
# ---------------------------------------------------------------------------------------------
class Joints(NamedTuple):
    """Gimbal joint angles in radians, chain order (yaw, roll, pitch)."""

    yaw: float
    roll: float
    pitch: float


def body_to_camera(joints: Sequence[float]) -> np.ndarray:
    """Forward kinematics: R_body_cam = Rz(yaw) @ Rx(roll) @ Ry(-pitch) for joints (yaw, roll, pitch)."""
    yaw, roll, pitch = joints
    return rot_z(yaw) @ rot_x(roll) @ rot_y(-pitch)


def solve_joints(R_body_cam: np.ndarray) -> Joints:
    """Inverse kinematics of :func:`body_to_camera`: split R = Rz(a) @ Rx(b) @ Ry(c).

    Written out, R = [[ca*cc - sa*sb*sc, -sa*cb, ca*sc + sa*sb*cc],
                      [sa*cc + ca*sb*sc,  ca*cb, sa*sc - ca*sb*cc],
                      [-cb*sc,            sb,    cb*cc          ]]
    so b = asin(R[2,1]), a = atan2(-R[0,1], R[1,1]), c = atan2(-R[2,0], R[2,2]); pitch joint = -c.
    The singular case (roll = +-90 deg) is far outside the +-30 deg roll range.
    """
    m = np.asarray(R_body_cam, dtype=float)
    sb = max(-1.0, min(1.0, m[2, 1]))
    b = math.asin(sb)
    if abs(sb) < 0.999999:
        a = math.atan2(-m[0, 1], m[1, 1])
        c = math.atan2(-m[2, 0], m[2, 2])
    else:
        a = math.atan2(m[1, 0], m[0, 0])
        c = 0.0
    return Joints(a, b, -c)


def solve_joints_fpv(R_world_body: np.ndarray, heading: float, pitch: float) -> Joints:
    """FPV mode: point the camera at (world heading, elevation) with the roll joint at zero.

    Only the look direction is enforced, so the horizon rolls with the aircraft (what FPV does).
    """
    f_world = np.array([math.cos(heading) * math.cos(pitch), math.sin(heading) * math.cos(pitch), math.sin(pitch)])
    f_body = np.asarray(R_world_body, dtype=float).T @ f_world
    return Joints(math.atan2(f_body[1], f_body[0]), 0.0, math.atan2(f_body[2], math.hypot(f_body[0], f_body[1])))


def camera_heading_pitch_roll(R_world_cam: np.ndarray) -> Tuple[float, float, float]:
    """(heading, pitch, roll) radians of an x-forward camera frame in the world.

    heading = ENU yaw of the look direction; pitch = elevation, + up (SIYI style);
    roll = rotation about the look direction, + = right side down.
    """
    roll, pitch_zyx, yaw = euler_zyx_from_rot(R_world_cam)
    return yaw, -pitch_zyx, roll


def camera_position_body(joints: Sequence[float], mount_xyz: Sequence[float] = DEFAULT_MOUNT_XYZ) -> np.ndarray:
    """Position of the camera in the drone body frame (metres) for the given joints."""
    yaw, roll, pitch = joints
    p = np.array(PITCH_JOINT_XYZ) + rot_y(-pitch) @ np.array(CAMERA_LINK_XYZ)
    p = np.array(ROLL_JOINT_XYZ) + rot_x(roll) @ p
    p = np.array(YAW_JOINT_XYZ) + rot_z(yaw) @ p
    return np.asarray(mount_xyz, dtype=float) + p


@dataclass
class CameraWorldPose:
    """Where the camera is and where it looks, in the world (ENU)."""

    position: np.ndarray          # metres
    R_camera_link: np.ndarray     # x-forward camera frame
    R_optical: np.ndarray         # z-forward / x-right / y-down optical frame
    heading_rad: float            # ENU yaw of the look direction
    pitch_rad: float              # elevation, + up
    roll_rad: float

    @property
    def look_direction(self) -> np.ndarray:
        """Unit vector of the optical axis in the world (ENU)."""
        return self.R_camera_link[:, 0]

    @property
    def quat_optical_xyzw(self) -> Tuple[float, float, float, float]:
        """Orientation of the optical frame as an (x, y, z, w) quaternion."""
        return quat_from_rot(self.R_optical)

    @property
    def compass_heading_deg(self) -> float:
        """Compass bearing of the look direction (deg clockwise from north)."""
        return enu_yaw_to_compass_deg(self.heading_rad)


def camera_world_pose(drone_position: Sequence[float], R_world_body: np.ndarray, joints: Sequence[float],
                      mount_xyz: Sequence[float] = DEFAULT_MOUNT_XYZ) -> CameraWorldPose:
    """Forward kinematics from the drone pose (ENU position, R_world_body) and the joint angles."""
    R_wb = np.asarray(R_world_body, dtype=float)
    pos = np.asarray(drone_position, dtype=float) + R_wb @ camera_position_body(joints, mount_xyz)
    R_cam = R_wb @ body_to_camera(joints)
    R_opt = R_cam @ rot_from_rpy(*OPTICAL_RPY)
    heading, pitch, roll = camera_heading_pitch_roll(R_cam)
    return CameraWorldPose(pos, R_cam, R_opt, heading, pitch, roll)


# ---------------------------------------------------------------------------------------------
# Limits
# ---------------------------------------------------------------------------------------------
@dataclass
class GimbalLimits:
    """Commandable SIYI limits and mechanical joint limits, all in degrees.

    Commandable = what 0x0E / 0x41 / 0x07 may ask for (clamped like the hardware does).
    Mechanical  = what the joints may physically reach while stabilising (contract: yaw +-160,
    roll +-30, pitch -135..+45), wider so stabilisation still works near the commandable edge.
    """

    yaw_min_deg: float = -135.0
    yaw_max_deg: float = 135.0
    pitch_min_deg: float = -90.0
    pitch_max_deg: float = 25.0
    joint_yaw_max_deg: float = 160.0
    joint_roll_max_deg: float = 30.0
    joint_pitch_min_deg: float = -135.0
    joint_pitch_max_deg: float = 45.0

    def clamp_yaw(self, yaw_deg: float) -> float:
        """Clamp a commanded yaw (deg) to the commandable range."""
        return max(self.yaw_min_deg, min(self.yaw_max_deg, float(yaw_deg)))

    def clamp_pitch(self, pitch_deg: float) -> float:
        """Clamp a commanded pitch (deg) to the commandable range."""
        return max(self.pitch_min_deg, min(self.pitch_max_deg, float(pitch_deg)))

    def clamp_joints(self, joints: Sequence[float]) -> Joints:
        """Clamp joint angles (rad) to the mechanical limits."""
        yaw, roll, pitch = joints
        jy = math.radians(self.joint_yaw_max_deg)
        jr = math.radians(self.joint_roll_max_deg)
        return Joints(
            max(-jy, min(jy, yaw)),
            max(-jr, min(jr, roll)),
            max(math.radians(self.joint_pitch_min_deg), min(math.radians(self.joint_pitch_max_deg), pitch)),
        )


# ---------------------------------------------------------------------------------------------
# The gimbal: setpoints, modes, slew, stabilisation, inverse kinematics
# ---------------------------------------------------------------------------------------------
class GimbalModel:
    """State machine of the emulated gimbal (not thread-safe: the emulator holds a lock).

    How the yaw setpoint is stored depends on the mode:
      * follow / fpv: ``yaw_tgt`` / ``yaw_cmd`` are relative to the airframe (SIYI deg, + = left),
        so the camera heading = drone heading + yaw.
      * lock: they are world headings (ENU deg); the camera keeps that heading when the drone turns.
        Switching modes converts the stored values so the camera never jumps.
    ``*_tgt`` is where the user wants to go; ``*_cmd`` follows it at no more than ``max_slew_dps``.
    A speed command (0x07) moves the target away from the command every step.

    Args:
        limits:         :class:`GimbalLimits` (defaults = A8 mini / ZR10).
        max_slew_dps:   slew-rate limit (deg/s), about 90 for SIYI gimbals.
        rate_yaw_sign:  converts a SIYI speed (+ = right) into the angle frame (+ = left); -1 by default.
        max_rate_dps:   what a 0x07 speed of 100 means in deg/s.
        initial_mode:   0 lock, 1 follow (default), 2 fpv.
        stabilize:      True = cancel the drone's tilt (pitch/roll level to the horizon).
        initial_yaw_deg / initial_pitch_deg: where the camera points at start.
        mount_xyz:      gimbal base position in the body frame (metadata only).
    """

    def __init__(self, limits: Optional[GimbalLimits] = None, max_slew_dps: float = 90.0,
                 rate_yaw_sign: float = -1.0, max_rate_dps: float = 90.0, initial_mode: int = MODE_FOLLOW,
                 stabilize: bool = True, initial_yaw_deg: float = 0.0, initial_pitch_deg: float = 0.0,
                 mount_xyz: Sequence[float] = DEFAULT_MOUNT_XYZ) -> None:
        self.limits = limits or GimbalLimits()
        self.max_slew_dps = float(max_slew_dps)
        self.rate_yaw_sign = 1.0 if float(rate_yaw_sign) >= 0.0 else -1.0
        self.max_rate_dps = float(max_rate_dps)
        self.stabilize = bool(stabilize)
        self.mount_xyz = tuple(float(v) for v in mount_xyz)
        self.mode = int(initial_mode) if int(initial_mode) in MODE_NAMES else MODE_FOLLOW

        self._R_world_body = np.eye(3)
        self._drone_yaw_deg = 0.0

        yaw0 = self.limits.clamp_yaw(initial_yaw_deg)
        pitch0 = self.limits.clamp_pitch(initial_pitch_deg)
        self.yaw_tgt = self.yaw_cmd = self._from_relative(yaw0)
        self.pitch_tgt = self.pitch_cmd = pitch0
        self.rate_yaw_dps = 0.0     # active speeds in the ANGLE frame (yaw + = left)
        self.rate_pitch_dps = 0.0
        self.joint_cmd = self.solve()

    # ------------------------------------------------------------------ inputs
    def set_drone_attitude(self, qx: float, qy: float, qz: float, qw: float) -> None:
        """Feed the latest drone orientation (quaternion x, y, z, w; world ENU -> body FLU)."""
        self.set_drone_rotation(rot_from_quat(qx, qy, qz, qw))

    def set_drone_rotation(self, R_world_body: np.ndarray) -> None:
        """Feed the latest drone orientation as a rotation matrix."""
        self._R_world_body = np.asarray(R_world_body, dtype=float)
        self._drone_yaw_deg = math.degrees(math.atan2(self._R_world_body[1, 0], self._R_world_body[0, 0]))

    @property
    def R_world_body(self) -> np.ndarray:
        """Latest drone orientation (identity until odometry arrives)."""
        return self._R_world_body

    @property
    def drone_yaw_deg(self) -> float:
        """Drone heading, ENU degrees (counter-clockwise from east)."""
        return self._drone_yaw_deg

    # ------------------------------------------------------------------ frame conversion
    def _to_relative(self, stored_yaw_deg: float) -> float:
        if self.mode == MODE_LOCK:
            return wrap180(stored_yaw_deg - self._drone_yaw_deg)
        return stored_yaw_deg

    def _from_relative(self, rel_yaw_deg: float) -> float:
        if self.mode == MODE_LOCK:
            return wrap180(self._drone_yaw_deg + rel_yaw_deg)
        return rel_yaw_deg

    @property
    def yaw_cmd_relative_deg(self) -> float:
        """Commanded yaw relative to the airframe right now (SIYI degrees, + = left)."""
        return self._to_relative(self.yaw_cmd)

    @property
    def yaw_tgt_relative_deg(self) -> float:
        """Target yaw relative to the airframe (SIYI degrees)."""
        return self._to_relative(self.yaw_tgt)

    # ------------------------------------------------------------------ commands
    def set_mode(self, mode: int) -> None:
        """0 = lock, 1 = follow, 2 = fpv. The stored yaw is converted so nothing jumps."""
        mode = int(mode)
        if mode not in MODE_NAMES:
            raise ValueError(f"Unknown mode {mode}. Expected 0=lock, 1=follow, 2=fpv.")
        if mode == self.mode:
            return
        rel_tgt = self.limits.clamp_yaw(self._to_relative(self.yaw_tgt))
        rel_cmd = self.limits.clamp_yaw(self._to_relative(self.yaw_cmd))
        self.mode = mode
        self.yaw_tgt = self._from_relative(rel_tgt)
        self.yaw_cmd = self._from_relative(rel_cmd)

    def set_attitude(self, yaw_deg: float, pitch_deg: float) -> Tuple[float, float]:
        """0x0E: absolute SIYI angles (deg; yaw + = left, pitch + = up). Clamped like the hardware.

        Returns the clamped (yaw, pitch) actually used. Any running speed command stops.
        """
        self.rate_yaw_dps = 0.0
        self.rate_pitch_dps = 0.0
        yaw = self.limits.clamp_yaw(float(yaw_deg))
        pitch = self.limits.clamp_pitch(float(pitch_deg))
        self.yaw_tgt = self._from_relative(yaw)
        self.pitch_tgt = pitch
        return yaw, pitch

    def set_yaw(self, yaw_deg: float) -> float:
        """0x41 with axis 0: change only the yaw target. Returns the clamped value."""
        self.rate_yaw_dps = 0.0
        yaw = self.limits.clamp_yaw(float(yaw_deg))
        self.yaw_tgt = self._from_relative(yaw)
        return yaw

    def set_pitch(self, pitch_deg: float) -> float:
        """0x41 with axis 1: change only the pitch target. Returns the clamped value."""
        self.rate_pitch_dps = 0.0
        self.pitch_tgt = self.limits.clamp_pitch(float(pitch_deg))
        return self.pitch_tgt

    def center(self) -> None:
        """0x08 one-key centre: yaw 0, pitch 0."""
        self.set_attitude(0.0, 0.0)

    def set_rate_angle_frame(self, yaw_dps: float, pitch_dps: float) -> None:
        """Speeds already in the angle frame (yaw + = left, pitch + = up), deg/s. 0, 0 stops."""
        self.rate_yaw_dps = float(yaw_dps)
        self.rate_pitch_dps = float(pitch_dps)

    def set_rate_command(self, turn_yaw: int, turn_pitch: int) -> Tuple[float, float]:
        """0x07: SIYI speeds -100..100 (yaw + = RIGHT, pitch + = up). Returns the angle-frame deg/s."""
        turn_yaw = max(-RATE_CMD_MAX, min(RATE_CMD_MAX, int(turn_yaw)))
        turn_pitch = max(-RATE_CMD_MAX, min(RATE_CMD_MAX, int(turn_pitch)))
        yaw_dps = self.rate_yaw_sign * turn_yaw / RATE_CMD_MAX * self.max_rate_dps
        pitch_dps = turn_pitch / RATE_CMD_MAX * self.max_rate_dps
        self.set_rate_angle_frame(yaw_dps, pitch_dps)
        return yaw_dps, pitch_dps

    @property
    def rate_active(self) -> bool:
        """True while a 0x07 speed command is moving the gimbal."""
        return self.rate_yaw_dps != 0.0 or self.rate_pitch_dps != 0.0

    # ------------------------------------------------------------------ integration
    def step(self, dt: float) -> Joints:
        """Advance the setpoints by ``dt`` seconds and return the joint targets (rad) for Gazebo."""
        dt = max(0.0, min(float(dt), 0.5))
        if dt > 0.0:
            if self.rate_active:
                rel = self.yaw_cmd_relative_deg + self.rate_yaw_dps * dt
                self.yaw_tgt = self._from_relative(self.limits.clamp_yaw(rel))
                self.pitch_tgt = self.limits.clamp_pitch(self.pitch_cmd + self.rate_pitch_dps * dt)
            max_step = self.max_slew_dps * dt
            d_yaw = wrap180(self.yaw_tgt - self.yaw_cmd) if self.mode == MODE_LOCK else self.yaw_tgt - self.yaw_cmd
            self.yaw_cmd += max(-max_step, min(max_step, d_yaw))
            if self.mode == MODE_LOCK:
                self.yaw_cmd = wrap180(self.yaw_cmd)
            d_pitch = self.pitch_tgt - self.pitch_cmd
            self.pitch_cmd += max(-max_step, min(max_step, d_pitch))
        self.joint_cmd = self.solve()
        return self.joint_cmd

    def solve(self) -> Joints:
        """Inverse kinematics for the current command angles and drone attitude."""
        rel_yaw_deg = self.yaw_cmd_relative_deg
        pitch_rad = math.radians(self.pitch_cmd)
        if not self.stabilize:
            joints = Joints(math.radians(rel_yaw_deg), 0.0, pitch_rad)
        else:
            heading = math.radians(self._drone_yaw_deg + rel_yaw_deg)
            if self.mode == MODE_FPV:
                joints = solve_joints_fpv(self._R_world_body, heading, pitch_rad)
            else:
                R_des = rot_z(heading) @ rot_y(-pitch_rad)      # level horizon, zero roll
                joints = solve_joints(self._R_world_body.T @ R_des)
        return self.limits.clamp_joints(joints)

    # ------------------------------------------------------------------ reporting
    def reported_attitude(self, joints_meas: Sequence[float], earth_frame: bool = True) -> Tuple[float, float, float]:
        """What the real camera reports in 0x0D / 0x0E: (yaw_deg, pitch_deg, roll_deg).

        yaw = the measured yaw joint, relative to the airframe (the A8 uses a magnetic encoder);
        pitch / roll = relative to the horizon (camera IMU) when stabilising, else the raw joints.
        """
        yaw_j, roll_j, pitch_j = joints_meas
        yaw_deg = math.degrees(yaw_j)
        if earth_frame and self.stabilize:
            R_cam = self._R_world_body @ body_to_camera(joints_meas)
            _, pitch, roll = camera_heading_pitch_roll(R_cam)
            return yaw_deg, math.degrees(pitch), math.degrees(roll)
        return yaw_deg, math.degrees(pitch_j), math.degrees(roll_j)

    def camera_pose(self, drone_position: Sequence[float], joints: Sequence[float]) -> CameraWorldPose:
        """Camera pose in the world for a drone position and joint angles (uses the latest attitude)."""
        return camera_world_pose(drone_position, self._R_world_body, joints, self.mount_xyz)
