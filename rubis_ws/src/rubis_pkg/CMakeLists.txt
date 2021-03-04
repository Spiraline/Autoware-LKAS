cmake_minimum_required(VERSION 3.0.2)
project(rubis_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  pcl_ros
  roscpp
  std_msgs
  nmea_msgs
  tf
  gnss
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rubis_pkg
#  CATKIN_DEPENDS autoware_msgs geometry_msgs pcl_ros roscpp std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include ${catkin_INCLUDE_DIRS}
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcd_clipper
  src/pcd_clipper_node.cpp
  src/pcd_clipper.cpp
)

add_dependencies(pcd_clipper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pcd_clipper
  ${catkin_LIBRARIES}
)

add_executable(gnss_localizer
  src/gnss_localizer_node.cpp
  src/gnss_localizer.cpp
)
add_dependencies(gnss_localizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gnss_localizer
  ${catkin_LIBRARIES}
)