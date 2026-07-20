from setuptools import find_packages, setup

package_name = 'ortho_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ortho_mapping.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aro',
    maintainer_email='aregar20012001@gmail.com',
    description='Batch GPS-georeferenced orthomosaic builder (post-flight).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ortho_node = ortho_mapping.ortho_node:main',
            'offline_map = ortho_mapping.offline_map:main',
            'gps_mosaic = ortho_mapping.gps_mosaic:main',
        ],
    },
)
