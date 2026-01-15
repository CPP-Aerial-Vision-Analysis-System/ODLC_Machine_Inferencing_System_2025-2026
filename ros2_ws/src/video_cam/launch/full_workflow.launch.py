#!/usr/bin/env python3

"""
Launch file for Full Workflow SIYI Camera Node

This launch file starts the complete workflow node that:
1. Streams live video from SIYI A8 camera
2. Captures 4K photos to SD card on trigger
3. Downloads ALL images from SD card to Jetson

Usage:
    ros2 launch video_cam full_workflow.launch.py

To trigger photo capture:
    ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='video_cam',
            executable='full_workflow',
            name='full_workflow_siyi_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': False,
            }],
            remappings=[
                # Remap topics if needed
            ]
        )
    ])
