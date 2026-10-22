"""
sim_models/launch/payload_release.launch.py -- PURPOSE: start the payload_release node (the simulated
drone's drop mechanism) with its most useful parameters as launch arguments.

    ros2 launch sim_models payload_release.launch.py
    ros2 launch sim_models payload_release.launch.py world_origin_lat:=36.2124 world_origin_lon:=-96.0040

The node needs a running Gazebo with the iris_a8mini model (same GZ_PARTITION / GZ_IP environment as
the simulator). It listens to /sim/payload/servo11 and /sim/payload/servo13 and detaches the
person / tent payload when ArduPilot opens that servo (>= open_threshold_us). Events (release,
detached, landed) go to the ROS topic /sim/payload/events as JSON strings.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Declare the launch arguments and the payload_release node."""
    args = [
        DeclareLaunchArgument(
            "open_threshold_us",
            default_value="1400.0",
            description="servo pulse width [us] that means OPEN (flight code: 1400)",
        ),
        DeclareLaunchArgument(
            "min_open_time_s",
            default_value="0.0",
            description="servo must stay open this long [s] before the drop",
        ),
        DeclareLaunchArgument(
            "world_origin_lat",
            default_value="nan",
            description="world <spherical_coordinates> latitude [deg] (nan = unknown)",
        ),
        DeclareLaunchArgument(
            "world_origin_lon",
            default_value="nan",
            description="world <spherical_coordinates> longitude [deg]",
        ),
        DeclareLaunchArgument(
            "world_origin_alt",
            default_value="nan",
            description="world <spherical_coordinates> elevation [m]",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="use /clock (only if the bringup bridges it)",
        ),
    ]

    def num(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    node = Node(
        package="sim_models",
        executable="payload_release",
        name="payload_release",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "open_threshold_us": num("open_threshold_us"),
                "min_open_time_s": num("min_open_time_s"),
                "world_origin_lat": num("world_origin_lat"),
                "world_origin_lon": num("world_origin_lon"),
                "world_origin_alt": num("world_origin_alt"),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
            }
        ],
    )
    return LaunchDescription(args + [node])
