"""
sim_bringup -- PURPOSE: Python side of the ASTRA simulator's bring-up package.

Modules (each one usable alone; see README.md for the full interface table):
  sim_paths       finds files/ports/profiles for the launch file and scripts (no ROS needed)
  wait_then_exec  waits for a port or file, then starts a program (start order of Gazebo -> SITL -> MAVProxy)
  gz_follow       makes the Gazebo 3D view follow the drone (/gui/follow)
  odom_tf         publishes the drone pose as a TF frame for RViz
  mission_tool    MAVLink ground-station tool (upload mission, wait until ready, arm, start, status)
  phase_monitor   prints the mission phases in the terminal and writes a phase log
  stack_check     waits until the team's flight stack is ready to fly
  photo_trigger   takes one photo (team topic /camera/trigger, or straight to the camera emulator)
"""

__version__ = '0.1.0'
