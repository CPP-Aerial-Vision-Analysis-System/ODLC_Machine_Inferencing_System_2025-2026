"""
example_detector/launch/example_detector.launch.py -- PURPOSE: start the TEMPLATE detector with its settings as
launch arguments, so a plug-in profile can set them (detector.custom.args in simulation/profiles/*.yaml):

    ros2 launch example_detector example_detector.launch.py [min_blob_area_px:=150] [camera_feed_path:=DIR]
                                                            [scan_period_s:=1.0] [serve_health:=true]
The node is named "example_detector": that is the node_name the profile gives, so demo.sh's stack_check waits
for it and check_slot finds it.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Entry point used by `ros2 launch` (and by team_stack.launch.py through the profile)."""
    args = [
        DeclareLaunchArgument('min_blob_area_px', default_value='150',
                              description='smallest object in full-photo pixels'),
        DeclareLaunchArgument('camera_feed_path', default_value='',
                              description='photo folder ("" = the team folder, found like new_od does)'),
        DeclareLaunchArgument('scan_period_s', default_value='1.0', description='seconds between folder scans'),
        DeclareLaunchArgument('serve_health', default_value='true',
                              description='serve /sahi/get_health like new_od (false if new_od also runs)'),
    ]
    # Launch arguments are text: ParameterValue(..., value_type=...) gives the node real numbers / booleans.
    node = Node(
        package='example_detector', executable='example_detector', name='example_detector', output='screen',
        parameters=[{
            'min_blob_area_px': ParameterValue(LaunchConfiguration('min_blob_area_px'), value_type=int),
            'camera_feed_path': ParameterValue(LaunchConfiguration('camera_feed_path'), value_type=str),
            'scan_period_s': ParameterValue(LaunchConfiguration('scan_period_s'), value_type=float),
            'serve_health': ParameterValue(LaunchConfiguration('serve_health'), value_type=bool),
        }])
    return LaunchDescription(args + [node])
