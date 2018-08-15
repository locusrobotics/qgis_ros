# ROS QGIS Plugin

A plugin for interacting with geospatial data from ROS topics in QGIS.

Note: this project is a work in progress. Better install and usage instructions are coming, along with example videos and example data.

## Dependencies (TODO cleanup, talk about how one might resolve dependencies not available through rosdep)
- python-geojson
- rospy
- geometry_msgs
- nav_msgs
- json_transport
- gnupg (rosbag in Python3)

## Installation (TODO cleanup)

1. Install QGIS 3.1 or newer: https://qgis.org/en/site/forusers/download.html
2. Clone this repo
3. Launch QGIS using the "Use" instructions below.


## Use
Because the plugin is not officially registered in the QGIS Plugin Registry, we can announce its presence to QGIS with the `QGIS_PLUGINPATH` envvar:

`QGIS_PLUGINPATH=~/repos/qgis-ros qgis`

Plugin is activated from within the "Plugins" menu. Once activated, it will show up as a new toolbar.

## Adding a custom set of ROS message translators
Additional translators can be registered in order to enable support for custom message types. A valid Translator reimplements the base `qgisros.core.Translator` class. Author an additional set of translators as a Python library with an iterable of Translators available for import. In the below example, `locus_qgis.translators` is a tuple of Translator subclasses. Comma separate additional translator module paths.

`QGIS_PLUGINPATH=~/repos/qgis-ros ROS_QGIS_EXTRA_TRANSLATORS=custom_qgis.translators,foo.translators qgis`

## Usage
- TODO examples of loading vector and raster data, subscribing to vector layers, and building a pipeline for various analytical functions.

## A note on code style
Developers will notice that `camelCase` is being used in Python. This may seem unusual, but PyQT and QGIS both use `camelCase`.So we follow that standard in accordance with PEP8.
