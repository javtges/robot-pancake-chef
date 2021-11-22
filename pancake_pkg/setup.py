from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['pancake_pkg'],
    package_dir={'': 'src'}
)

setup(**d)


## Source: https://roboticsbackend.com/ros-import-python-module-from-another-package/
