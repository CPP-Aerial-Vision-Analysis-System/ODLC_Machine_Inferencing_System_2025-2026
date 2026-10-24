"""Purpose: verify controller numeric overrides with real ROS parameter type handling, without FCU I/O."""

import os
import sys
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('interfaces.msg')
from rclpy.node import Node  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from main_controller import MainController  # noqa: E402


@pytest.mark.parametrize('name,default,value,expected', [
    ('default_camera_pitch_deg', -90.0, '-90', -90.0),
    ('default_camera_hfov_deg', 81.0, '70', 70.0),
    ('target_ground_height_m', 0.0, '3', 3.0),
    ('target_ground_height_m', 0.0, '3.5', 3.5),
    ('max_aim_offset_m', 150.0, '120', 120.0),
    ('target_ground_height_m', 0.0, 'true', 0.0),
    ('target_ground_height_m', 0.0, 'typo', 0.0),
])
def test_numeric_override_uses_ros_type_and_falls_back(name, default, value, expected):
    """Exercise the actual controller declaration on a node isolated from any flight graph."""
    context = rclpy.context.Context()
    rclpy.init(context=context, domain_id=110 + os.getpid() % 20,
               args=['--ros-args', '-p', f'{name}:={value}'])
    node = Node('controller_parameter_test', context=context)
    try:
        result = MainController._number_parameter(node, name, default)
        assert result == expected
        assert isinstance(result, float)
    finally:
        node.destroy_node()
        context.shutdown()
