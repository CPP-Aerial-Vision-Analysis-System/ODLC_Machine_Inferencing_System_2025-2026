"""
test/test_gimbal_control.py -- PURPOSE: unit tests of siyi_emulator.gimbal_control: rotation helpers,
the sign conventions (+yaw = camera turns LEFT, -90 pitch = straight down, SIYI speed + = RIGHT),
limits and slew rate, inverse-kinematics round trips, level-camera stabilisation while the drone
tilts, lock/follow/FPV modes and what 0x0D reports. Most tests are ported from the first twin
prototype (siyi_sim/test/test_gimbal_model.py); the 0x07 / 0x41 / initial-pose tests are new.
Run: python3 -m pytest test/   (no ROS needed)
"""

import math
import random

import numpy as np
import pytest

from siyi_emulator import gimbal_control as gm


def settle(model, seconds=4.0, dt=0.01):
    """Run the model until the slew-limited command reaches the target."""
    joints = model.joint_cmd
    for _ in range(int(seconds / dt)):
        joints = model.step(dt)
    return joints


def drone(model, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    R = gm.rot_from_rpy(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
    model.set_drone_attitude(*gm.quat_from_rot(R))


def camera_hpr_deg(model, joints):
    R_cam = model.R_world_body @ gm.body_to_camera(joints)
    h, p, r = gm.camera_heading_pitch_roll(R_cam)
    return math.degrees(h), math.degrees(p), math.degrees(r)


# ------------------------------------------------------------------------- rotations
def test_rotation_helpers_round_trip():
    for _ in range(50):
        r, p, y = (random.uniform(-1.4, 1.4) for _ in range(3))
        R = gm.rot_from_rpy(r, p, y)
        assert np.allclose(gm.euler_zyx_from_rot(R), (r, p, y), atol=1e-9)
        assert np.allclose(gm.rot_from_quat(*gm.quat_from_rot(R)), R, atol=1e-9)


def test_zero_quaternion_is_identity():
    assert np.allclose(gm.rot_from_quat(0, 0, 0, 0), np.eye(3))


def test_compass_from_enu_yaw():
    assert gm.enu_yaw_to_compass_deg(0.0) == pytest.approx(90.0)          # +x = east
    assert gm.enu_yaw_to_compass_deg(math.pi / 2) == pytest.approx(0.0)   # +y = north
    assert gm.enu_yaw_to_compass_deg(-math.pi / 2) == pytest.approx(180.0)


# ------------------------------------------------------------------ sign conventions
def test_yaw_positive_is_left():
    model = gm.GimbalModel()
    model.set_attitude(30.0, 0.0)
    j = settle(model)
    assert j.yaw == pytest.approx(math.radians(30.0), abs=1e-6)
    assert (j.roll, j.pitch) == pytest.approx((0.0, 0.0), abs=1e-9)
    h, _, _ = camera_hpr_deg(model, j)
    assert h == pytest.approx(30.0, abs=1e-6)          # heading turned counter-clockwise = LEFT


def test_pitch_minus_90_is_straight_down():
    model = gm.GimbalModel()
    model.set_attitude(0.0, -90.0)
    j = settle(model)
    assert j.pitch == pytest.approx(-math.pi / 2, abs=1e-6)
    f = (model.R_world_body @ gm.body_to_camera(j))[:, 0]     # look direction
    assert np.allclose(f, [0.0, 0.0, -1.0], atol=1e-9)


def test_initial_pose_parameters():
    model = gm.GimbalModel(initial_yaw_deg=10.0, initial_pitch_deg=-90.0)
    assert model.yaw_cmd == 10.0 and model.pitch_cmd == -90.0
    assert model.joint_cmd.pitch == pytest.approx(-math.pi / 2)
    assert gm.GimbalModel(initial_pitch_deg=-200.0).pitch_cmd == -90.0     # clamped


def test_commands_clamped_to_limits():
    model = gm.GimbalModel()
    assert model.set_attitude(200.0, 50.0) == (135.0, 25.0)
    assert model.set_attitude(-200.0, -100.0) == (-135.0, -90.0)
    assert model.set_yaw(170.0) == 135.0
    assert model.set_pitch(-120.0) == -90.0


def test_custom_limits_per_model():
    limits = gm.GimbalLimits(yaw_min_deg=-160.0, yaw_max_deg=160.0)
    model = gm.GimbalModel(limits)
    assert model.set_attitude(150.0, 0.0) == (150.0, 0.0)


def test_slew_rate_limit_90_dps():
    model = gm.GimbalModel(max_slew_dps=90.0)
    model.set_attitude(90.0, 0.0)
    for _ in range(50):
        model.step(0.01)
    assert model.yaw_cmd == pytest.approx(45.0, abs=1.0)
    for _ in range(60):
        model.step(0.01)
    assert model.yaw_cmd == pytest.approx(90.0, abs=1e-6)


def test_single_axis_changes_only_one_axis():
    model = gm.GimbalModel()
    model.set_attitude(20.0, -30.0)
    settle(model)
    model.set_yaw(-40.0)
    settle(model)
    assert (model.yaw_cmd, model.pitch_cmd) == pytest.approx((-40.0, -30.0))
    model.set_pitch(10.0)
    settle(model)
    assert (model.yaw_cmd, model.pitch_cmd) == pytest.approx((-40.0, 10.0))


# ------------------------------------------------------------------ 0x07 speed commands
def test_positive_siyi_speed_turns_right():
    model = gm.GimbalModel(rate_yaw_sign=-1.0, max_rate_dps=90.0)
    yaw_dps, pitch_dps = model.set_rate_command(50, 0)        # +50 = right at 45 deg/s
    assert (yaw_dps, pitch_dps) == pytest.approx((-45.0, 0.0))
    for _ in range(100):
        model.step(0.01)
    assert model.yaw_cmd == pytest.approx(-45.0, abs=0.5)     # right = negative angle
    h, _, _ = camera_hpr_deg(model, model.joint_cmd)
    assert h == pytest.approx(-45.0, abs=0.5)


def test_speed_command_pitch_and_limits_and_stop():
    model = gm.GimbalModel()
    model.set_rate_command(100, -50)
    assert model.rate_yaw_dps == pytest.approx(-90.0) and model.rate_pitch_dps == pytest.approx(-45.0)
    settle(model, seconds=10.0)                    # runs into the limits and stays there
    assert model.yaw_cmd == pytest.approx(-135.0) and model.pitch_cmd == pytest.approx(-90.0)
    model.set_rate_command(0, 0)
    assert not model.rate_active


def test_speed_values_are_clamped():
    model = gm.GimbalModel(max_rate_dps=90.0)
    yaw_dps, _ = model.set_rate_command(127, 0)    # int8 maximum, protocol says -100..100
    assert yaw_dps == pytest.approx(-90.0)


def test_rate_yaw_sign_parameter_flips_direction():
    model = gm.GimbalModel(rate_yaw_sign=+1.0)
    model.set_rate_command(50, 0)
    for _ in range(100):
        model.step(0.01)
    assert model.yaw_cmd == pytest.approx(45.0, abs=0.5)


def test_set_attitude_stops_speed():
    model = gm.GimbalModel()
    model.set_rate_command(10, 10)
    model.set_attitude(10.0, 10.0)
    assert not model.rate_active


# ------------------------------------------------------------------ inverse kinematics
def test_ik_round_trip_random_joints():
    rng = random.Random(1)
    for _ in range(500):
        j = gm.Joints(math.radians(rng.uniform(-150, 150)), math.radians(rng.uniform(-29, 29)),
                      math.radians(rng.uniform(-130, 40)))
        assert np.allclose(gm.solve_joints(gm.body_to_camera(j)), j, atol=1e-9)


def test_level_drone_is_identity_mapping():
    model = gm.GimbalModel()
    model.set_attitude(30.0, -20.0)
    j = settle(model)
    assert j == pytest.approx((math.radians(30.0), 0.0, math.radians(-20.0)), abs=1e-6)


def test_tilted_drone_keeps_camera_level_and_on_target():
    model = gm.GimbalModel()
    drone(model, roll_deg=20.0, pitch_deg=10.0, yaw_deg=45.0)
    model.set_attitude(30.0, -20.0)
    j = settle(model)
    h, p, r = camera_hpr_deg(model, j)
    assert h == pytest.approx(75.0, abs=1e-6)      # drone heading + 30 (follow mode)
    assert p == pytest.approx(-20.0, abs=1e-6)     # stabilised to the horizon
    assert r == pytest.approx(0.0, abs=1e-6)       # level horizon
    assert abs(j.roll) > 1e-3                      # the roll joint did real work


def test_camera_stays_level_while_drone_wobbles():
    model = gm.GimbalModel()
    model.set_attitude(0.0, -45.0)
    settle(model)
    for i in range(200):
        drone(model, roll_deg=25.0 * math.sin(i / 10.0), pitch_deg=12.0 * math.cos(i / 7.0))
        h, p, r = camera_hpr_deg(model, model.step(0.01))
        assert (h, p, r) == pytest.approx((0.0, -45.0, 0.0), abs=1e-6)


def test_nadir_camera_stays_nadir_while_drone_tilts():
    # The survey case: camera straight down while the drone pitches/rolls in the wind.
    model = gm.GimbalModel(initial_pitch_deg=-90.0)
    for roll, pitch in ((10, 0), (0, -12), (-15, 8), (20, 20)):
        drone(model, roll_deg=roll, pitch_deg=pitch, yaw_deg=33.0)
        j = model.step(0.01)
        f = (model.R_world_body @ gm.body_to_camera(j))[:, 0]
        assert np.allclose(f, [0.0, 0.0, -1.0], atol=1e-6), (roll, pitch, f)


def test_lock_mode_holds_world_heading():
    model = gm.GimbalModel(initial_mode=gm.MODE_LOCK)
    model.set_attitude(30.0, 0.0)
    settle(model)
    drone(model, yaw_deg=50.0)
    j = settle(model)
    assert camera_hpr_deg(model, j)[0] == pytest.approx(30.0, abs=1e-6)
    assert j.yaw == pytest.approx(math.radians(-20.0), abs=1e-6)


def test_follow_mode_turns_with_drone():
    model = gm.GimbalModel(initial_mode=gm.MODE_FOLLOW)
    model.set_attitude(30.0, 0.0)
    settle(model)
    drone(model, yaw_deg=50.0)
    j = settle(model)
    assert camera_hpr_deg(model, j)[0] == pytest.approx(80.0, abs=1e-6)


def test_fpv_roll_follows_drone_but_look_direction_is_stabilised():
    model = gm.GimbalModel(initial_mode=gm.MODE_FPV)
    drone(model, roll_deg=15.0, pitch_deg=10.0)
    model.set_attitude(0.0, 0.0)
    j = settle(model)
    assert j.roll == pytest.approx(0.0, abs=1e-9)
    h, p, r = camera_hpr_deg(model, j)
    assert (h, p) == pytest.approx((0.0, 0.0), abs=1e-6)
    assert abs(r) > 10.0


def test_mode_switch_does_not_jump():
    model = gm.GimbalModel(initial_mode=gm.MODE_FOLLOW)
    model.set_attitude(30.0, -10.0)
    settle(model)
    drone(model, yaw_deg=20.0)
    before = settle(model)
    model.set_mode(gm.MODE_LOCK)
    assert np.allclose(before, model.step(0.01), atol=1e-9)
    drone(model, yaw_deg=40.0)
    assert camera_hpr_deg(model, settle(model))[0] == pytest.approx(50.0, abs=1e-6)
    model.set_mode(gm.MODE_FOLLOW)
    assert model.yaw_cmd_relative_deg == pytest.approx(10.0, abs=1e-6)


def test_set_mode_rejects_bad_values():
    with pytest.raises(ValueError, match="Unknown mode 7"):
        gm.GimbalModel().set_mode(7)


def test_lock_mode_joint_clamped_at_mechanical_limit():
    model = gm.GimbalModel(initial_mode=gm.MODE_LOCK)
    model.set_attitude(0.0, 0.0)
    settle(model)
    drone(model, yaw_deg=170.0)
    assert settle(model).yaw == pytest.approx(-math.radians(160.0), abs=1e-6)


def test_stabilisation_can_be_disabled():
    model = gm.GimbalModel(stabilize=False)
    drone(model, roll_deg=20.0, pitch_deg=10.0, yaw_deg=45.0)
    model.set_attitude(30.0, -20.0)
    assert np.allclose(settle(model), (math.radians(30.0), 0.0, math.radians(-20.0)), atol=1e-6)


# ------------------------------------------------------------------ reporting (0x0D)
def test_reported_attitude_earth_frame_vs_joint():
    model = gm.GimbalModel()
    drone(model, pitch_deg=10.0)              # nose DOWN 10 deg (ZYX pitch +)
    model.set_attitude(0.0, -20.0)
    j = settle(model)
    # nose already 10 deg down, so the joint only adds 10 deg more to look 20 deg below the horizon
    assert j.pitch == pytest.approx(math.radians(-10.0), abs=1e-6)
    yaw, pitch, roll = model.reported_attitude(j, earth_frame=True)
    assert (yaw, pitch, roll) == pytest.approx((0.0, -20.0, 0.0), abs=1e-6)
    assert model.reported_attitude(j, earth_frame=False)[1] != pytest.approx(-20.0, abs=1e-3)


def test_camera_world_pose_level_drone():
    model = gm.GimbalModel()
    pose = model.camera_pose([10.0, 20.0, 30.0], gm.Joints(0.0, 0.0, 0.0))
    assert np.allclose(pose.position, [10.092, 20.0, 29.904], atol=1e-9)
    assert pose.compass_heading_deg == pytest.approx(90.0)                 # looking east
    assert np.allclose(pose.R_optical[:, 2], [1.0, 0.0, 0.0], atol=1e-9)   # optical z = look direction
    assert np.allclose(pose.R_optical[:, 0], [0.0, -1.0, 0.0], atol=1e-9)  # optical x = right
    assert np.allclose(pose.look_direction, [1.0, 0.0, 0.0], atol=1e-9)
