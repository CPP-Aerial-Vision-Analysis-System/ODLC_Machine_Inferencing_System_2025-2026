from setuptools import find_packages, setup

package_name = 'ros2_image_stitching_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'incremental_stitcher = ros2_image_stitching_pkg.incremental_stitcher:main',
            'mock_camera_node = ros2_image_stitching_pkg.mock_camera_node:main',
        ],
    },
)
