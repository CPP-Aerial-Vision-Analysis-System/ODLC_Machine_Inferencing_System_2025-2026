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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='student.joshuaestrada@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_pub = video_cam.image_pub_siyi:main',
            'object_detection = video_cam.object_detection:main',
            'object_detection_sahi = video_cam.object_detection_sahi:main',
            'object_detection_sahi_mobilenet = video_cam.object_detection_sahi_mobilenet:main',
        ],
    },
)
