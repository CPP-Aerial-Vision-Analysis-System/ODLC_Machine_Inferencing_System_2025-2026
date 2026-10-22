"""
sim_models/test/acceptance/camera_geometry.py -- PURPOSE: answer "where SHOULD this world point appear
in the gimbal camera image?" so the acceptance test can check the camera really looks where the
gimbal joint angles say.

  GimbalKinematics  camera pose in the world from the drone pose + the three joint angles. The link
                    offsets and joint axes are READ FROM THE GENERATED MODEL SDF, so the check
                    follows the model (it is not a second copy of the numbers).
  PinholeCamera     Gazebo's ideal pinhole camera: looks along +X of its frame, image right = -Y,
                    image down = -Z, fx = fy = (width / 2) / tan(hfov / 2), centre = (w/2, h/2).
  find_blob         centre of all pixels of one bright colour (red, magenta, blue, cyan) in an RGB image.

Only numpy is needed. Angles in radians, lengths in metres, poses as 4x4 matrices.
"""

import math
import xml.etree.ElementTree as ET

import numpy as np


# ------------------------------------------------------------------ small 3D helpers
def quat_to_matrix(qx, qy, qz, qw):
    """3x3 rotation matrix of a unit quaternion (x, y, z, w)."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rpy_to_matrix(roll, pitch, yaw):
    """3x3 rotation of SDF roll/pitch/yaw (fixed axes X, then Y, then Z)."""
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def axis_angle_matrix(axis, angle):
    """3x3 rotation by `angle` about the unit vector `axis` (Rodrigues)."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def make_transform(rot=None, trans=(0, 0, 0)):
    """4x4 homogeneous transform."""
    t = np.eye(4)
    if rot is not None:
        t[:3, :3] = rot
    t[:3, 3] = trans
    return t


def pose_text_to_transform(text):
    """'x y z roll pitch yaw' (SDF <pose>) -> 4x4 transform."""
    v = [float(s) for s in (text or "0 0 0 0 0 0").split()]
    v += [0.0] * (6 - len(v))
    return make_transform(rpy_to_matrix(v[3], v[4], v[5]), v[:3])


# ------------------------------------------------------------------ gimbal kinematics
class GimbalKinematics:
    """Camera pose from drone pose + joint angles, using the offsets/axes in the model SDF.

    Chain: base_link -> gimbal_base -> (gimbal_yaw) gimbal_yaw_link -> (gimbal_roll) gimbal_roll_link
           -> (gimbal_pitch) gimbal_pitch_link -> frame a8mini_camera_link.
    In SDF each joint sits at its child link's origin with its axis in the child frame, so
    parent_T_child(q) = parent_T_child(0) * Rot(axis, q).
    """

    def __init__(self, sdf_text):
        """Args: sdf_text: the generated iris_a8mini SDF (string)."""
        model = ET.fromstring(sdf_text).find("model")
        links = {lk.get("name"): lk for lk in model.findall("link")}
        joints = {j.get("name"): j for j in model.findall("joint")}
        frames = {f.get("name"): f for f in model.findall("frame")}

        def pose_of(el):
            p = el.find("pose")
            return pose_text_to_transform(p.text if p is not None else None)

        self.base_T_gimbal = pose_of(links["gimbal_base"])  # gimbal_base pose is in the model frame
        self.offsets = {
            "yaw": pose_of(links["gimbal_yaw_link"]),  # relative_to gimbal_base
            "roll": pose_of(links["gimbal_roll_link"]),  # relative_to gimbal_yaw_link
            "pitch": pose_of(links["gimbal_pitch_link"]),  # relative_to gimbal_roll_link
        }
        self.axes = {
            k: [float(s) for s in joints[f"gimbal_{k}"].find("axis/xyz").text.split()]
            for k in ("yaw", "roll", "pitch")
        }
        self.pitch_T_camera = pose_of(frames["a8mini_camera_link"])

    def camera_pose(self, drone_pos, drone_quat, yaw, roll, pitch):
        """World pose (4x4) of a8mini_camera_link (x = along the lens).

        Args:
            drone_pos: (x, y, z) of base_link in the world [m].
            drone_quat: (qx, qy, qz, qw) of base_link in the world.
            yaw, roll, pitch: gimbal joint angles [rad].
        """
        t = make_transform(quat_to_matrix(*drone_quat), drone_pos) @ self.base_T_gimbal
        for name, angle in (("yaw", yaw), ("roll", roll), ("pitch", pitch)):
            t = t @ self.offsets[name] @ make_transform(axis_angle_matrix(self.axes[name], angle))
        return t @ self.pitch_T_camera


# ------------------------------------------------------------------ pinhole camera
class PinholeCamera:
    """Ideal Gazebo camera: +X forward, image right = -Y, image down = -Z."""

    def __init__(self, width, height, hfov):
        self.width, self.height = int(width), int(height)
        self.fx = (width / 2.0) / math.tan(hfov / 2.0)
        self.fy = self.fx
        self.cx, self.cy = width / 2.0, height / 2.0

    def project(self, world_T_camera, point):
        """Pixel (u, v) of a world point, or None if it is behind the camera."""
        p = np.linalg.inv(world_T_camera) @ np.array([point[0], point[1], point[2], 1.0])
        x, y, z = p[:3]
        if x <= 1e-6:
            return None
        return self.cx - self.fx * y / x, self.cy - self.fy * z / x

    def optical_axis(self, world_T_camera):
        """Unit vector (world) along which the camera looks."""
        return world_T_camera[:3, 0] / np.linalg.norm(world_T_camera[:3, 0])


# ------------------------------------------------------------------ colour blobs
def color_mask(rgb, color):
    """Boolean mask of strongly coloured pixels. rgb: HxWx3 uint8."""
    r, g, b = (rgb[:, :, i].astype(np.int16) for i in range(3))
    if color == "red":
        return (r > 150) & (g < 90) & (b < 90)
    if color == "magenta":
        return (r > 150) & (b > 150) & (g < 100)
    if color == "blue":
        return (b > 150) & (r < 90) & (g < 90)
    if color == "cyan":
        return (g > 150) & (b > 150) & (r < 100)
    raise ValueError(f"unknown colour {color}")


def find_blob(rgb, color, min_pixels=40):
    """(u, v, pixel_count) = centre of all `color` pixels, or None if fewer than min_pixels."""
    mask = color_mask(rgb, color)
    count = int(mask.sum())
    if count < min_pixels:
        return None
    vs, us = np.nonzero(mask)
    return float(us.mean()), float(vs.mean()), count
