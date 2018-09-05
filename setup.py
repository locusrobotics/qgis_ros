#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=[
        'qgis_ros',
        'qgis_ros.core',
        'qgis_ros.core.translators',
        'qgis_ros.ui'
    ],
    package_dir={'': 'src'})

setup(**setup_args)
