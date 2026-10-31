from setuptools import find_packages, setup
from glob import glob

setup(
    name='astra_live_detection', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/astra_live_detection']),
                ('share/astra_live_detection', ['package.xml', 'UPSTREAM.json']),
                ('share/astra_live_detection/launch', glob('launch/*.launch.py'))],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='ASTRA', maintainer_email='maintainer@example.com',
    description='Topic-based camera input for the ASTRA SAHI/YOLO detector.',
    license='Proprietary',
    entry_points={'console_scripts': ['live_detection = astra_live_detection.live_node:main']},
)
