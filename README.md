# QGIS-ROS

**Warning: This package is not currently maintained and may not work without some additional technical care. It looks like a newer version of QGIS has changed a bunch of the Python interfaces. Try using QGIS 3.10 or perhaps a bit older. PRs are welcome!**

A QGIS plugin for interacting with data from ROS topics and bags.

The ROSCon 2018 Presentation on QGIS-ROS can be found here: https://vimeo.com/293539252. The presentation PDF can be found here: https://roscon.ros.org/2018/presentations/ROSCon2018_UnleashingGISToolbox.pdf

## Requirements

- Ubuntu >= 16.04
- Python 3.5
- QGIS >= 3.1
- ROS 1

QGIS can be installed from [the QGIS download page.][1] You need to add a PPA if using Ubuntu 16.04.

## Installation (source)

```bash
cd ~/my_ros_ws/src/
git clone git@github.com:locusrobotics/qgis_ros.git
git clone git@github.com:clearpathrobotics/wireless.git
git clone git@github.com:locusrobotics/json_transport.git
catkin build

cd ~/my_ros_ws/src/qgis_ros
pip3 install -r requirements.txt
```

## Usage (source)

```bash
rosrun qgis_ros start_qgis_ros
```

Once QGIS is loaded, navigate to Plugins -> Installed -> and enable QGIS_ROS with the checkbox. You only need to do this once.

With custom ROS Message Translators:

```bash
export QGIS_ROS_EXTRA_TRANSLATORS='custompythonmodule.translators'
rosrun qgis_ros start_qgis_ros
```

## Usage (docker)

It can often be very tricky to comfortably resolve all dependencies in your workspace because of the combination of ROS1, Python3, GDAL 2, and QGIS.

It uses a lazy/reckless way to expose X server to the container. For alternatives, check out the [ROS Docker GUI Tutorial][2].

```
cd ~/my_catkin_ws/src/qgis_ros/docker
docker-compose build
xhost +local:root
docker-compose up
xhost -local:root # Remember to remove X server permissions after!
```

To use extra translators, you'll need to mount a volume with them and/or extend the image.

## ROS Message Translators

QGIS is able to read and write data from hundreds of formats and sources by translating these formats into a common in-memory format. In order to represent ROS messages in this format, we target common interchange formats (GeoJSON and GeoTIFF) to make it easy to extend. If you want to make use of a custom ROS message type, all you have to do is:

1. subclass `qgis_ros.core.translators.Translator`
2. implement a `translate` function that accepts a ROS message and returns a GeoJSON-like dict or a path to a GeoTIFF on disk
3. Add the `VectorTranslatorMixin` or `RasterTranslatorMixin`
4. Register before launching by setting the envvar `QGIS_ROS_EXTRA_TRANSLATORS` to a python module path to an iterable of translators.

Check out the source code for more details.

## Troubleshooting

### My topic does not appear in the list of topics

Only topics that have ROS Message Translators will appear. Not every message has been implemented. If your message is custom, look above for how to create a custom translator. If it is standard to ROS, create an issue or raise a Pull Request with a new one.TODO: add an example to the ROSCon presentation repository and link here.
51

## Contributions

Contributions are appreciated. Please open a pull request.

Developers will notice that `camelCase` is being used in Python. This may seem unusual, but PyQT and QGIS both use `camelCase`.So we follow that standard in accordance with PEP8.

## Contact

Queries that don't fit well as GitHub issues can be directed to Andrew Blakey at ablakey@locusrobotics.com

Being a robogeographer is a lonely life. If there are any others out there, don't hesitate to say hi!

[1]: https://qgis.org/en/site/forusers/download.html
[2]: http://wiki.ros.org/docker/Tutorials/GUI
