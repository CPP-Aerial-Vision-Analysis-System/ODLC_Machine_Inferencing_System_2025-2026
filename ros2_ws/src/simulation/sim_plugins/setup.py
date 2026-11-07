"""
sim_plugins/setup.py -- PURPOSE: build/installation recipe of the sim_plugins ROS 2 package.

It installs the Python modules in sim_plugins/ and the README, and creates the commands below
(run them with `ros2 run sim_plugins <name>`, or `python3 -m sim_plugins.<module>` without ROS):

    profile_tool   read, check and explain a plug-in profile (simulation/profiles/*.yaml):
                   validate, plan, resolve, list, contract (used by demo.sh and start_sim.sh)
    check_slot     look at the LIVE ROS graph and say whether the component in a slot (detector,
                   mission_brain, team_stack, camera, autopilot, world) keeps that slot's contract; exit 0/1
    model_check    check a detector weights file before a flight (exists, format, loads, classes); exit 0/1
    run_meta       write <run>/run_meta/{autopilot,detector,team_stack}.json: which pieces really ran
"""
from setuptools import find_packages, setup

package_name = 'sim_plugins'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ASTRA simulation team',
    maintainer_email='astra-sim@example.com',
    description='Plug-and-play slots, profiles and the slot conformance checker of the ASTRA simulator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'profile_tool = sim_plugins.profile_tool:main',
            'check_slot = sim_plugins.check_slot:main',
            'model_check = sim_plugins.model_check:main',
            'run_meta = sim_plugins.run_meta:main',
        ],
    },
)
