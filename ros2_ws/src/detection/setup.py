from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    # watchdog gives event-driven (inotify) image pickup in new_od; the node
    # falls back to polling if it is missing, so it is a soft dependency.
    install_requires=['setuptools', 'watchdog'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='student.joshuaestrada@gmail.com',
    description='Detection package with SAHI+YOLO object detection and image publishing',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'new_od = detection.new_od:main',
        ],
    },
)
