from setuptools import find_packages
from setuptools import setup

setup(
    name='carla_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('carla_msgs', 'carla_msgs.*')),
)
