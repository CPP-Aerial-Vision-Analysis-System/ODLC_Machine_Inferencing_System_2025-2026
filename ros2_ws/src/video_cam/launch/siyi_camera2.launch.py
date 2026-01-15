#!/usr/bin/env python3

"""
Launch file for SIYI A8 Combined Camera System
Starts the image_pub_siyi2 node with proper configuration
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for SIYI camera system"""
    
    # Declare launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation mode (no real camera)'
    )
    
    camera_ip_arg = DeclareLaunchArgument(
        'camera_ip',
        default_value='192.168.144.25',
        description='IP address of SIYI camera'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='4K',
        description='Photo resolution (4K, 2.7K, or 1080P)'
    )
    
    # Create the node
    siyi_camera_node = Node(
        package='video_cam',
        executable='image_pub_siyi2',
        name='siyi_a8_camera',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim')
        }],
        emulate_tty=True,
        respawn=False
    )
    
    # Log startup info
    startup_info = LogInfo(
        msg=[
            '\n',
            '=' * 80, '\n',
            'SIYI A8 COMBINED CAMERA SYSTEM STARTING\n',
            '=' * 80, '\n',
            'Camera IP: ', LaunchConfiguration('camera_ip'), '\n',
            'Resolution: ', LaunchConfiguration('resolution'), '\n',
            'Simulation: ', LaunchConfiguration('use_sim'), '\n',
            '=' * 80, '\n',
            '\nTo capture a 4K photo:\n',
            '  ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"\n',
            '\nTo change resolution:\n',
            '  ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: \'4K\'"\n',
            '\n', '=' * 80, '\n'
        ]
    )
    
    return LaunchDescription([
        use_sim_arg,
        camera_ip_arg,
        resolution_arg,
        startup_info,
        siyi_camera_node
    ])
