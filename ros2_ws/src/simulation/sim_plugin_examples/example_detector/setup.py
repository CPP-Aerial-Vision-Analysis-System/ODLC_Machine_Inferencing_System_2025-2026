"""
example_detector/setup.py -- PURPOSE: build/installation recipe of the TEMPLATE detector package.

Installs the Python module, the launch file (so a profile can say `launch: example_detector.launch.py`) and the
README, and creates the command  `ros2 run example_detector example_detector`.
When you copy this package, rename package_name below, the folder example_detector/, resource/example_detector
and the entry point (README.md, "copy -> rename").
"""
from glob import glob

from setuptools import find_packages, setup

package_name = 'example_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ASTRA simulation team',
    maintainer_email='astra-sim@example.com',
    description='Template detector for the ASTRA simulator plug-and-play detector slot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_detector = example_detector.node:main',
        ],
    },
)
