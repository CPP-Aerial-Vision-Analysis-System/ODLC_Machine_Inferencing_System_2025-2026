from setuptools import find_packages, setup

package_name = 'package1'

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
    maintainer_email='lensouthbay@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = package1.temp_publisher:main',
            'listener = package1.temp_subscriber:main',
            'temp_service = package1.temp_service:main',
            'convert_server = package1.convert_server:main',
            'convert_client = package1.convert_client:main'
        ],
    },
)
