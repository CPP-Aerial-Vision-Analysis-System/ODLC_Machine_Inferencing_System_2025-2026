"""
sim_bringup/setup.py -- PURPOSE: build/installation recipe of the sim_bringup ROS 2 package.

It installs the Python helper modules in sim_bringup/, the launch files (sim.launch.py starts the whole
simulator, team_stack.launch.py wraps the team's own tracker.launch.xml), the RViz layout, the Gazebo
window layout, the README, and creates the commands below (run them with `ros2 run sim_bringup <name>`):

    wait_then_exec  wait for a UDP/TCP port (or a file), then start a program  (used by sim.launch.py)
    gz_follow       make the Gazebo 3D view follow the drone                    (used by sim.launch.py)
    odom_tf         turn the drone odometry into a TF frame for RViz            (used by sim.launch.py)
    mission_tool    MAVLink ground-station tool: upload mission, wait, arm, start, status
    phase_monitor   prints the mission phases (takeoff, search, photos, detections, drops, RTL, landed)
    stack_check     waits until the team's flight stack (./launcher.sh sim) is ready to fly
    photo_trigger   "press the shutter": /camera/trigger, or straight to the camera emulator
"""
from glob import glob

from setuptools import find_packages, setup

package_name = 'sim_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ASTRA simulation team',
    maintainer_email='astra-sim@example.com',
    description='One-command bring-up of the ASTRA Gazebo simulator and its helper tools',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wait_then_exec = sim_bringup.wait_then_exec:main',
            'gz_follow = sim_bringup.gz_follow:main',
            'odom_tf = sim_bringup.odom_tf:main',
            'mission_tool = sim_bringup.mission_tool:main',
            'phase_monitor = sim_bringup.phase_monitor:main',
            'stack_check = sim_bringup.stack_check:main',
            'photo_trigger = sim_bringup.photo_trigger:main',
        ],
    },
)
