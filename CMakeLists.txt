cmake_minimum_required(VERSION 2.8.3)
project(stats_error_detection)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED rosgraph_msgs)
catkin_package()

install(PROGRAMS scripts/training.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
