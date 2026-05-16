from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'video_cam'

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
    install_requires=[
        'setuptools',
        'requests>=2.28.0',
        'opencv-python>=4.8.0',
        'numpy>=1.24.0',
    ],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='student.joshuaestrada@gmail.com',
    description='SIYI camera ROS2 driver: capture, download, and publish images.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'siyi = video_cam.siyi_node:main',
        ],
    },
)
