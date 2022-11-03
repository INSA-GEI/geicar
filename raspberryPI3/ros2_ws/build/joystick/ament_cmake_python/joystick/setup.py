from setuptools import find_packages
from setuptools import setup

setup(
    name='joystick',
    version='0.0.0',
    packages=find_packages(
        include=('joystick', 'joystick.*')),
)
