"""
lattice_bridge/launch/lattice_reporter.launch.py -- PURPOSE: the opt-in way to start the Jetson-side target
reporter (node `lattice_reporter`) together with its settings file. NOTHING starts this file for you:
launcher.sh and main/launch/tracker.launch.xml do not mention it, so the drone flies exactly as before unless
somebody runs, in a second terminal after ./launcher.sh:

    ros2 launch lattice_bridge lattice_reporter.launch.py                 # the defaults
    ros2 launch lattice_bridge lattice_reporter.launch.py dry_run:=true   # decide and log, send nothing
    ros2 launch lattice_bridge lattice_reporter.launch.py params_file:=/path/to/my_copy.yaml

Launch arguments:
  params_file  the settings file (default: the installed config/lattice_reporter.yaml, every option explained)
  dry_run      "true" or "false"; empty (the default) = use the value in the settings file
  log_level    debug | info | warn | error (default info)
A launch argument that you do not give never overrides the settings file.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _reporter_node(context, *args, **kwargs):
    """Build the node once the launch arguments are known (so empty ones can be left out)."""
    parameters = [LaunchConfiguration('params_file').perform(context)]
    dry_run = LaunchConfiguration('dry_run').perform(context).strip().lower()
    if dry_run in ('true', 'false'):
        parameters.append({'dry_run': dry_run == 'true'})
    elif dry_run:
        raise RuntimeError(f'dry_run must be true or false (or empty), got {dry_run!r}')
    log_level = LaunchConfiguration('log_level').perform(context).strip() or 'info'
    return [Node(package='lattice_bridge', executable='lattice_reporter', name='lattice_reporter',
                 output='screen', parameters=parameters, arguments=['--ros-args', '--log-level', log_level])]


def generate_launch_description():
    """Entry point used by `ros2 launch`."""
    default_params = os.path.join(get_package_share_directory('lattice_bridge'), 'config', 'lattice_reporter.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='settings file (every option is explained inside it)'),
        DeclareLaunchArgument('dry_run', default_value='',
                              description='true = decide and log but publish nothing; empty = settings file'),
        DeclareLaunchArgument('log_level', default_value='info', description='debug | info | warn | error'),
        OpaqueFunction(function=_reporter_node),
    ])
