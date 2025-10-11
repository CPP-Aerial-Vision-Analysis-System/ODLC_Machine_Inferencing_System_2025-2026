from setuptools import find_packages
from setuptools import setup

setup(
    name='gps_mavros',
    version='0.0.0',
    packages=find_packages(
        include=('gps_mavros', 'gps_mavros.*')),
)
