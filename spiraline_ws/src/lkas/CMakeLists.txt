cmake_minimum_required(VERSION 2.8.3)
project(lkas)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  jsk_recognition_msgs
  autoware_msgs
  geometry_msgs
  nmea_msgs
  std_msgs
  tf
  gnss
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lkas
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)

add_executable(lkas_combiner
  src/lkas_combiner_node.cpp
  src/lkas_combiner.cpp
)
add_dependencies(lkas_combiner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lkas_combiner
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS src/lane_keeping_module.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)