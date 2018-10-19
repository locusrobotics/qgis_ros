FROM osrf/ros:lunar-desktop-full-xenial

### Installing system packages (your responsibility)
RUN apt update && \
    apt install -y wget software-properties-common python3-pip python-catkin-tools

### Installing QGIS (QGIS docs responsibility: https://qgis.org/en/site/forusers/alldownloads.html#debian-ubuntu)
RUN sh -c 'echo "deb http://qgis.org/ubuntugis xenial main" >> /etc/apt/sources.list'
RUN wget -O - http://qgis.org/downloads/qgis-2017.gpg.key | gpg --import; exit 0
RUN gpg --export --armor CAEB3DC3BDF7FB45 | apt-key add -
RUN add-apt-repository -y ppa:ubuntugis/ubuntugis-unstable && \
    apt update && \
    apt install -y gdal-bin python-gdal python3-gdal qgis

RUN pip3 install rospkg

### Installing QGIS_ROS (QGIS_ROS docs responsibility: https://github.com/locusrobotics/qgis_ros)
RUN mkdir -p ~/catkin_ws/src
RUN /bin/bash -c " \
    source /opt/ros/lunar/setup.bash && \
    cd ~/catkin_ws/ && \
    catkin init && \
    git clone http://github.com/locusrobotics/qgis_ros.git src/qgis_ros && \
    git clone http://github.com/locusrobotics/json_transport.git src/json_transport && \
    git clone http://github.com/clearpathrobotics/wireless.git src/wireless && \
    catkin build"
