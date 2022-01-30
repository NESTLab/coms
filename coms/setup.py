#!/usr/bin/python3.8

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['coms', 'msg'],
    package_dir={'': 'src'}
)

setup(**d)
