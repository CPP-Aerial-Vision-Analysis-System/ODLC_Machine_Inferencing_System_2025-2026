"""
example_listener/setup.py -- PURPOSE: build/installation recipe of the TEMPLATE listener ("extra") package.

Installs the Python module and the README, and creates the command  `ros2 run example_listener example_listener`.
When you copy this package, rename package_name below, the folder example_listener/, resource/example_listener
and the entry point (README.md, "copy -> rename").
"""
from setuptools import find_packages, setup

package_name = 'example_listener'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ASTRA simulation team',
    maintainer_email='astra-sim@example.com',
    description='Template extra (telemetry / mission / detection listener) for the ASTRA simulator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_listener = example_listener.node:main',
        ],
    },
)
