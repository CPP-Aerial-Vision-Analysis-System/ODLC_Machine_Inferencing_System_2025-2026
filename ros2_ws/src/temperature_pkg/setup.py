from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'temperature_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='student.joshuaestrada@gmail.com',
    description='Publisher/Subscriber Task: SUAS 2025-2026',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'talker = temperature_pkg.publisher_member_function:main',
            'listener = temperature_pkg.subscriber_member_function:main',
            'convert_client = temperature_pkg.convert_temp_client:main',
            'convert_server = temperature_pkg.convert_temp_server:main',
        ],
    },
)
