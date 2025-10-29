from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='temperature_pkg',     # your package name
            executable='talker', # entry point name in setup.py
            name='talker'
        ),
        Node(
            package='temperature_pkg',
            executable='listener',
            name='listener'
        )
    ])
