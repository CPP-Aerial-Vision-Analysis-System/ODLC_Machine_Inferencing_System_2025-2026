from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {'rtsp_url': ('rtsp://192.168.144.25:8554/main.264', str),
                'width': ('1280', int), 'height': ('720', int), 'fps': ('10.0', float),
                'max_frame_age_sec': ('2.0', float),
                'frame_id': ('zr10_camera', str),
                'snapshot_directory': ('stream_snapshots', str),
                'image_topic': ('zr10/image_raw', str)}
    return LaunchDescription([
        DeclareLaunchArgument(key, default_value=value[0]) for key, value in defaults.items()
    ] + [Node(package='zr10_stream', executable='stream_node.py', output='screen',
              parameters=[{key: ParameterValue(LaunchConfiguration(key), value_type=value[1])
                           for key, value in defaults.items() if key != 'image_topic'}],
              remappings=[('zr10/image_raw', LaunchConfiguration('image_topic'))])])
