cmake_minimum_required(VERSION 2.8.3)
project(stats_error_detection)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED rosgraph_msgs dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/Talker.cfg
)

# make sure configure headers are built before any node using them
#add_dependencies(talker ${PROJECT_NAME}_gencfg)

catkin_package()

install(PROGRAMS scripts/training.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
