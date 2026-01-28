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
    install_requires=['setuptools'],
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
            'image_pub = detection.image_pub_siyi:main',
            'object_detection = detection.object_detection:main',
            'object_detection_sahi = detection.object_detection_sahi:main',
            'new_od = detection.new_od:main',
            'new_od_aro = detection.new_od_aro:main',
            'steroids_od = detection.new_od_on_steroids:main',
        ],
    },
)
