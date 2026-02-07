from setuptools import setup
import os
from glob import glob

package_name = 'astra'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ODLC Team',
    maintainer_email='dev@example.com',
    description='Combined camera capture and object detection node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'capture_detect = astra.capture_detect:main',
            'main_controller_aro = astra.main_controller_aro:main',
        ],
    },
)
