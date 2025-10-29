from setuptools import find_packages
from setuptools import setup

setup(
    name='temperature_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('temperature_interfaces', 'temperature_interfaces.*')),
)
