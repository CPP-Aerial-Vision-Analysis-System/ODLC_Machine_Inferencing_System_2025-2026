"""
siyi_emulator/launch/siyi_emulator.launch.py -- PURPOSE: start the SIYI camera emulator node with its
default parameter file (config/siyi_emulator.yaml) plus the most common overrides as launch arguments,
so a bringup launch file (or a person) can start it in one line:

    ros2 launch siyi_emulator siyi_emulator.launch.py
    ros2 launch siyi_emulator siyi_emulator.launch.py camera_model:=a8mini strict:=true
    ros2 launch siyi_emulator siyi_emulator.launch.py bind_ip:=127.0.0.3 use_gazebo:=false

Every other parameter can be changed in a copy of the YAML file passed as params_file:=<path>.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# (name, default, description, type) of the launch arguments that become parameters
ARGUMENTS = [
    ("camera_model", "zr10", "zr10 (2560x1440 photos) or a8mini (3840x2160)", str),
    ("strict", "false", "true = silent on 0x04/0x06/0x16/0x18/0x19 like a real A8 mini", bool),
    ("camera_ip", "192.168.144.25", "address clients expect the camera at", str),
    ("bind_ip", "", "listen address ('' = camera_ip if it exists here, else 127.0.0.1)", str),
    ("ctrl_port", "37260", "UDP SDK port", int),
    ("media_port", "82", "TCP media port", int),
    ("use_gazebo", "true", "false = no Gazebo link (test-pattern photos)", bool),
    ("initial_pitch_deg", "-90.0", "gimbal pitch at start, deg (-90 = straight down)", float),
    ("sd_card_dir", "~/.ros/siyi_emulator/sd_card", "folder of the emulated SD card", str),
    ("log_commands", "false", "log every SDK command at INFO", bool),
]


def generate_launch_description() -> LaunchDescription:
    """Declare the arguments and the siyi_emulator node."""
    default_params = os.path.join(get_package_share_directory("siyi_emulator"), "config", "siyi_emulator.yaml")
    actions = [DeclareLaunchArgument("params_file", default_value=default_params,
                                     description="YAML parameter file (all parameters, see README)")]
    overrides = {}
    for name, default, description, kind in ARGUMENTS:
        actions.append(DeclareLaunchArgument(name, default_value=default, description=description))
        overrides[name] = ParameterValue(LaunchConfiguration(name), value_type=kind)
    actions.append(Node(
        package="siyi_emulator",
        executable="siyi_emulator",
        name="siyi_emulator",
        output="screen",
        parameters=[LaunchConfiguration("params_file"), overrides],
    ))
    return LaunchDescription(actions)
