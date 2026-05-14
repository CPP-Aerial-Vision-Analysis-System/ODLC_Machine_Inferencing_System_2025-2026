#!/usr/bin/env python3

"""
Launch file for SIYI A8 Mini Unified Pipeline

This launches the single-node architecture that handles:
- Camera capture control
- SD card indexing
- Incremental download
- ROS image publication

Usage:
    ros2 launch video_cam siyi_unified_pipeline.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    camera_ip = DeclareLaunchArgument(
        'camera_ip',
        default_value='192.168.144.25',
        description='IP address of SIYI camera'
    )
    
    resolution = DeclareLaunchArgument(
        'resolution',
        default_value='4K',
        description='Photo resolution (4K, 2.7K, or 1080P)'
    )
    
    altitude_threshold = DeclareLaunchArgument(
        'altitude_threshold',
        default_value='-13.716',
        description='Altitude threshold for enabling camera'
    )
    
    # Unified pipeline node
    pipeline_node = Node(
        package='video_cam',
        executable='siyi',
        name='siyi_unified_pipeline',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_real_camera': True,
            'camera_ip': LaunchConfiguration('camera_ip'),
            'resolution': LaunchConfiguration('resolution'),
            'altitude_threshold': LaunchConfiguration('altitude_threshold'),
        }],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
        ]
    )
    
    return LaunchDescription([
        camera_ip,
        resolution,
        altitude_threshold,
        LogInfo(msg='Starting SIYI Unified Pipeline...'),
        pipeline_node,
        LogInfo(msg='SIYI Unified Pipeline launched successfully'),
    ])
