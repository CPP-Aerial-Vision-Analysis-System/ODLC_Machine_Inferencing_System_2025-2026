"""
siyi_emulator/setup.py -- PURPOSE: build/install recipe of the ament_python package. It installs the
Python modules, the launch file, the default parameter file, the helper scripts, the Gazebo test
fixture and the README, and creates two executables:
    siyi_emulator  -> siyi_emulator.node:main        (the emulator; ROS node or stand-alone)
    siyi_client    -> siyi_emulator.sdk_client:main  (tiny SIYI SDK client for poking a camera)
"""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'siyi_emulator'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh') + glob('scripts/*.py')),
        (os.path.join('share', package_name, 'test'), glob('test/*.py')),
        (os.path.join('share', package_name, 'test', 'fixtures'), glob('test/fixtures/*')),
        (os.path.join('share', package_name, 'test', 'e2e'), glob('test/e2e/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ASTRA simulation',
    maintainer_email='astra-sim@example.com',
    description='SIYI ZR10 / A8 mini camera emulator (SDK UDP 37260 + media HTTP 82) for the Gazebo simulator.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'siyi_emulator = siyi_emulator.node:main',
            'siyi_client = siyi_emulator.sdk_client:main',
        ],
    },
)
