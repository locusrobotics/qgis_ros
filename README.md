# QGIS-ROS

A QGIS plugin for interacting with data from ROS topics and bags.

## Requirements

- Ubuntu >= 16.04
- Python 3.5
- QGIS >= 3.1
- ROS 1

QGIS can be installed from: https://qgis.org/en/site/forusers/download.html You need to add a PPA if using Ubuntu 16.04.

## Installation

```bash
cd ~/my_ros_ws/src/
git clone git@github.com:locusrobotics/qgis_ros.git
catkin build
```

## Use

Simple:
```bash
rosrun qgis_ros start_qgis_ros
```

With custom ROS Message Translators:
```
QGIS_ROS_EXTRA_TRANSLATORS='custompythonmodule.translators'
rosrun qgis_ros start_qgis_ros
```

QGIS ROS is a fully qualified QGIS plugin but is not yet uploaded to the Plugin Registry. Therefore, activating the plugin is done by setting `QGIS_PLUGINPATH` to where this plugin is found, and then runing `qgis`.  Optionally, you can launch it in a terminal where the `ROS_MASTER_URI` is set to your desired ROS Master. Otherwise only bagfile access is available.

The plugin *should* be available and loaded by default, but you may have to open the Plugin Manager from within QGIS and enable it once.

## ROS Message Translators
QGIS is able to read and write data from hundreds of formats and sources by translating these formats into a common in-memory format. In order to represent ROS messages in this format, we target common interchange formats (GeoJSON and GeoTIFF) to make it easy to extend. If you want to make use of a custom ROS message type, all you have to do is:

1. subclass `qgis_ros.core.translators.Translator`
2. implement a `translate` function that accepts a ROS message and returns a GeoJSON-like dict or a path to a GeoTIFF on disk
3. Add the `VectorTranslatorMixin` or `RasterTranslatorMixin`
4. Register before launching by setting the envvar `QGIS_ROS_EXTRA_TRANSLATORS` to a python module path to an iterable of translators.

Check out the source code for more details.

# TODO: add an example to the ROSCon presentation repository and link here.

## Contributions

Contributions are appreciated. Please open a pull request.

Developers will notice that `camelCase` is being used in Python. This may seem unusual, but PyQT and QGIS both use `camelCase`.So we follow that standard in accordance with PEP8.

## Contact

Queries that don't fit well as GitHub issues can be directed to Andrew Blakey at ablakey@locusrobotics.com

Being a robogeographer is a lonely life. If there are any others out there, don't hesitate to say hi!
