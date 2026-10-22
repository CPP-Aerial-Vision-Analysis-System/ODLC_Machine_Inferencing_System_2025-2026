"""
test/test_node.py -- PURPOSE: tests of the ROS 2 wrapper (siyi_emulator.node): every EmulatorConfig field
is a ROS parameter, "-p initial_pitch_deg:=-90" (an INTEGER on the command line) is accepted for a
float setting, and one photo produces one JSON message on /sim/camera/photo_info and state messages on
/sim/camera/emulator_state. Runs the node in-process on 127.0.0.1 with free ports and no Gazebo.
Skipped when rclpy is not available.
"""

import json
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from siyi_emulator import node as node_module  # noqa: E402
from siyi_emulator.emulator import EmulatorConfig  # noqa: E402
from siyi_emulator.sdk_client import SiyiClient  # noqa: E402


@pytest.fixture()
def ros_node(tmp_path):
    rclpy.init(args=["--ros-args", "-p", "bind_ip:=127.0.0.1", "-p", "ctrl_port:=0", "-p", "media_port:=0",
                     "-p", "use_gazebo:=false", "-p", f"sd_card_dir:={tmp_path}", "-p", "initial_pitch_deg:=-90",
                     "-p", "camera_model:=a8mini", "-p", "publish_state_hz:=5.0"])
    node = node_module._make_node()
    yield node
    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()


def test_every_setting_is_a_parameter(ros_node):
    for name in EmulatorConfig.field_names():
        assert ros_node.has_parameter(name), name
    assert ros_node.get_parameter("camera_model").value == "a8mini"
    assert ros_node.emulator.cfg.initial_pitch_deg == -90.0            # INTEGER -90 accepted for a float
    assert isinstance(ros_node.emulator.cfg.initial_pitch_deg, float)
    assert ros_node.emulator.profile.display_name == "A8MINI"


def test_photo_info_and_state_topics(ros_node):
    from std_msgs.msg import String
    got_photo, got_state = [], []
    listener = rclpy.create_node("siyi_emulator_test_listener")
    listener.create_subscription(String, "/sim/camera/photo_info", lambda m: got_photo.append(m.data), 10)
    listener.create_subscription(String, "/sim/camera/emulator_state", lambda m: got_state.append(m.data), 10)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(ros_node)
    executor.add_node(listener)
    try:
        emu = ros_node.emulator
        with SiyiClient("127.0.0.1", emu.udp_port, emu.media_port, timeout=1.0) as client:
            assert client.hardware_id().startswith("73")
            assert client.take_photo() == 0
            end = time.monotonic() + 8.0
            while time.monotonic() < end and not (got_photo and got_state):
                executor.spin_once(timeout_sec=0.1)
        assert got_photo, "no /sim/camera/photo_info message"
        info = json.loads(got_photo[0])
        assert info["sd_name"] == "IMG_0001.jpg" and info["camera_model"] == "A8MINI"
        assert info["image"]["source"] == "test_pattern"                 # no Gazebo in this test
        assert info["url"].endswith("/101SIYI_IMG/IMG_0001.jpg")
        assert "camera_world" in info and "zoom" in info
        state = json.loads(got_state[-1])
        assert state["camera_model"] == "A8MINI" and state["gimbal"]["pitch_deg"] == pytest.approx(-90.0, abs=0.5)
    finally:
        executor.remove_node(listener)
        executor.remove_node(ros_node)
        listener.destroy_node()
