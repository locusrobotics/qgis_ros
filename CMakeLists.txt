cmake_minimum_required(VERSION 2.8.3)
project(qgis_ros)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package()

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(
  PROGRAMS
    scripts/start_qgis_ros
    scripts/navsatfix
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES
    requirements.txt
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
