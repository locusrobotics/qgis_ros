cmake_minimum_required(VERSION 2.8.3)
project(qgis_ros)

find_package(catkin REQUIRED COMPONENTS
  catkin_virtualenv
)

catkin_package()

catkin_generate_virtualenv(
    PYTHON_VERSION_MAJOR 3
    USE_SYSTEM_PACKAGES FALSE
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  PROGRAMS
    scripts/start_qgis_ros
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    requirements.txt
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
