"""
sim_demo/setup.py -- PURPOSE: build/installation recipe of the sim_demo ROS 2 package.

It installs the Python modules in sim_demo/, the launch file, the default parameter file,
the README, and creates the four commands teammates run with `ros2 run sim_demo <name>`:
    demo_panel      the demo window (live camera POV, newest photo + boxes, detections, phase)
    truth_detector  optional stand-in detector that publishes the same /image_detection
    demo_markers    RViz markers (search area, targets, detections, footprints, track, drops)
    score_run       records a run and writes a Markdown + JSON score report
"""
from glob import glob

from setuptools import find_packages, setup

package_name = 'sim_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/docs', glob('docs/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ASTRA simulation team',
    maintainer_email='astra-sim@example.com',
    description='ASTRA simulation demo: panel, truth detector stand-in, RViz markers, scorer',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo_panel = sim_demo.demo_panel:main',
            'truth_detector = sim_demo.truth_detector:main',
            'demo_markers = sim_demo.demo_markers:main',
            'score_run = sim_demo.score_run:main',
        ],
    },
)
