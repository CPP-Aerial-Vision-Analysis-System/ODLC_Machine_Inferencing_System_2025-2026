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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'working_pub = video_cam.working_pub:main',
            'image_pub = video_cam.image_pub_siyi:main',
            'image_pub_backup = video_cam.image_pub_siyi_backup:main',
            'image_pub_siyi2 = video_cam.image_pub_siyi2:main',
            'siyi2 = video_cam.siyi2:main',
            'siyi_unified_pipeline = video_cam.siyi_unified_pipeline:main',
            'camera_control = video_cam.camera_control_client:main',
            'object_detection = video_cam.object_detection:main',
            'object_detection_sahi = video_cam.object_detection_sahi:main',
            'object_detection_sahi_mobilenet = video_cam.object_detection_sahi_mobilenet:main',
            'full_workflow = video_cam.full_workflow:main',
        ],
    },
)
