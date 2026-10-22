"""
test/test_camera_geometry.py -- PURPOSE: unit tests of test/acceptance/camera_geometry.py, the gimbal
forward kinematics + pinhole projection the acceptance test uses. They double as executable
documentation of the contract's sign conventions, checked on the REAL generated model:
  * pitch -pi/2 looks straight down, and the top of the picture points to the drone's nose
  * yaw + turns the camera LEFT (counter-clockwise seen from above)
  * roll + puts the right side down
  * a point straight ahead projects to the image centre; Gazebo image right = camera -Y

Run:  python3 -m pytest -q test/test_camera_geometry.py   (needs xacro: ROS 2 sourced)
"""

import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "acceptance"))
sys.path.insert(0, os.path.dirname(HERE))

from camera_geometry import (
    GimbalKinematics,
    PinholeCamera,
    find_blob,  # noqa: E402
    quat_to_matrix,
)

pytest.importorskip("xacro", reason="xacro is needed to generate the model")
from sim_models.model_generator import generate_model_sdf  # noqa: E402

XACRO = os.path.join(os.path.dirname(HERE), "models", "iris_a8mini", "model.sdf.xacro")
LEVEL = (0.0, 0.0, 0.0, 1.0)  # drone level, nose along world +X
NORTH = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))  # yaw +90 deg: nose north


@pytest.fixture(scope="module")
def kin():
    return GimbalKinematics(generate_model_sdf(XACRO))


def look(kin, quat, yaw, roll, pitch):
    pose = kin.camera_pose((0.0, 0.0, 10.0), quat, yaw, roll, pitch)
    return pose, pose[:3, 0], pose[:3, 2]  # pose, lens direction (+X), image "up" (+Z)


def test_zero_angles_look_out_of_the_nose(kin):
    _, fwd, up = look(kin, LEVEL, 0, 0, 0)
    assert np.allclose(fwd, [1, 0, 0], atol=1e-9)
    assert np.allclose(up, [0, 0, 1], atol=1e-9)


def test_pitch_minus_90_looks_straight_down_top_of_picture_is_the_nose(kin):
    _, fwd, up = look(kin, LEVEL, 0, 0, -math.pi / 2)
    assert np.allclose(fwd, [0, 0, -1], atol=1e-9)
    assert np.allclose(up, [1, 0, 0], atol=1e-9)


def test_pitch_positive_tilts_up(kin):
    _, fwd, _ = look(kin, LEVEL, 0, 0, 0.3)
    assert fwd[2] > 0.29


def test_yaw_positive_turns_left(kin):
    _, fwd, _ = look(kin, LEVEL, math.radians(30), 0, 0)
    assert fwd[1] > 0.49  # +Y of the drone = its left side
    assert math.degrees(math.atan2(fwd[1], fwd[0])) == pytest.approx(30.0, abs=1e-6)


def test_roll_positive_puts_right_side_down(kin):
    pose, _, _ = look(kin, LEVEL, 0, math.radians(15), 0)
    right = -pose[:3, 1]  # camera -Y = image right
    assert right[2] < -0.25


def test_body_attitude_is_applied(kin):
    _, fwd, _ = look(kin, NORTH, 0, 0, 0)
    assert np.allclose(fwd, [0, 1, 0], atol=1e-9)  # nose north -> camera looks north


def test_camera_hangs_under_the_nose(kin):
    pose, _, _ = look(kin, LEVEL, 0, 0, 0)
    offset = pose[:3, 3] - np.array([0, 0, 10.0])
    assert 0.05 < offset[0] < 0.12 and abs(offset[1]) < 0.005 and -0.13 < offset[2] < -0.07


def test_projection_centre_and_axes():
    cam = PinholeCamera(1280, 720, 1.4137)
    assert cam.fx == pytest.approx(640 / math.tan(1.4137 / 2))
    pose = np.eye(4)
    assert cam.project(pose, (10, 0, 0)) == pytest.approx((640, 360))
    u, v = cam.project(pose, (10, -1, 0))  # to the camera's right
    assert u > 640 and v == pytest.approx(360)
    u, v = cam.project(pose, (10, 0, 1))  # above
    assert v < 360 and u == pytest.approx(640)
    assert cam.project(pose, (-1, 0, 0)) is None  # behind


def test_ground_point_below_is_image_centre_when_looking_down(kin):
    cam = PinholeCamera(1280, 720, 1.4137)
    pose, _, _ = look(kin, NORTH, 0, 0, -math.pi / 2)
    below = pose[:3, 3] - np.array([0, 0, pose[2, 3]])
    u, v = cam.project(pose, below)
    assert (u, v) == pytest.approx((640, 360), abs=1e-6)
    u, v = cam.project(pose, below + np.array([0, 5.0, 0]))  # 5 m north = ahead
    assert v < 360 and u == pytest.approx(640, abs=1e-6)  # appears above the centre


def test_quaternion_matrix():
    assert np.allclose(quat_to_matrix(*NORTH) @ [1, 0, 0], [0, 1, 0])


def test_find_blob():
    img = np.zeros((100, 200, 3), dtype=np.uint8)
    img[40:50, 150:160] = (255, 0, 0)
    u, v, n = find_blob(img, "red")
    assert (u, v, n) == pytest.approx((154.5, 44.5, 100))
    assert find_blob(img, "blue") is None
