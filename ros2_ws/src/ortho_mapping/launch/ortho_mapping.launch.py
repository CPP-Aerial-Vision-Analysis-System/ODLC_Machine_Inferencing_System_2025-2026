#!/usr/bin/env python3
"""Launch the batch ortho-mapping node (passive until MAP_START).

    ros2 launch ortho_mapping ortho_mapping.launch.py
    ros2 launch ortho_mapping ortho_mapping.launch.py autostart:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='false',
                              description='Build the map immediately on launch'),
        DeclareLaunchArgument('gsd_cm', default_value='3.0',
                              description='Output resolution in cm/px'),
        DeclareLaunchArgument('default_altitude_agl', default_value='30.0',
                              description='Assumed AGL (m) for images with no sidecar'),
        DeclareLaunchArgument('refine', default_value='true',
                              description='ECC seam refinement on/off'),
        Node(
            package='ortho_mapping',
            executable='ortho_node',
            name='ortho_mapping',
            output='screen',
            parameters=[{
                'autostart': LaunchConfiguration('autostart'),
                'gsd_cm': LaunchConfiguration('gsd_cm'),
                'default_altitude_agl': LaunchConfiguration('default_altitude_agl'),
                'refine': LaunchConfiguration('refine'),
            }],
        ),
    ])
