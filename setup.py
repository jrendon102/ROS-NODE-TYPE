#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Generate setup information using catkin_pkg
d = generate_distutils_setup(
    packages=["ros_utils"],
    package_dir={"": ""},
    version="1.0.0",
    author="Julian A Rendon",
    author_email="julianrendon514@gmail.com",
    maintainer="Julian A Rendon",
    maintainer_email="julianrendon514@gmail.com",
    license="MIT",
    description="The ros_utils package is a versatile utility package for ROS development.",
    url="https://github.com/jrendon102/ros_utils.git",
)
setup(**d)
