"""
lattice_bridge/setup.py -- PURPOSE: build/installation recipe of the lattice_bridge ROS 2 package.

Phase 1 installs the Python modules in lattice_bridge/ (including lattice_bridge.sinks) and the README: the
library is imported by the programs (`from lattice_bridge import build_uav_asset, ...`).
Phase 2 adds, one by one, the programs (console scripts, all OFF unless somebody starts them) and the files they
need (launch/, config/):
  lattice_reporter   the Jetson-side target reporter (reporter_node.py), started by launch/lattice_reporter.launch.py
  bridge             the ground-laptop bridge program (bridge_node.py): source -> core -> publisher -> sink
  fake_lattice       a pretend Lattice with a live map page (fake_lattice.py), for trying everything with no account
"""
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'lattice_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ASTRA simulation team',
    maintainer_email='astra-sim@example.com',
    description='ASTRA to Anduril Lattice bridge: entity builders, clustering, radio codec, publisher, sources, sinks, programs',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lattice_reporter = lattice_bridge.reporter_node:main',
            'bridge = lattice_bridge.bridge_node:main',
            'fake_lattice = lattice_bridge.fake_lattice:main',
        ],
    },
)
