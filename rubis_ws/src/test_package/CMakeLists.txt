cmake_minimum_required(VERSION 2.8.3)
project(test_package)

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
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_package
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

add_executable(fake_object_generator
  src/fake_object_generator.cpp
)
add_dependencies(fake_object_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fake_object_generator
  ${catkin_LIBRARIES}
)

add_executable(fake_traffic_signal_generator
  src/fake_traffic_signal_generator.cpp
)
add_dependencies(fake_traffic_signal_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fake_traffic_signal_generator
  ${catkin_LIBRARIES}
)

add_executable(lidar_republisher
  src/lidar_republisher.cpp
)
add_dependencies(lidar_republisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_republisher
  ${catkin_LIBRARIES}
)

add_executable(camera_republisher
  src/camera_republisher.cpp
)
add_dependencies(camera_republisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_republisher
  ${catkin_LIBRARIES}
)

add_executable(TF_test
  src/TF_test.cpp
)
add_dependencies(TF_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(TF_test
  ${catkin_LIBRARIES}
)

add_executable(gnss_republisher
  src/gnss_republisher.cpp
)
add_dependencies(gnss_republisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gnss_republisher
  ${catkin_LIBRARIES}
)

add_executable(twist_cmd_publisher
  src/twist_cmd_publisher.cpp
)
add_dependencies(twist_cmd_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(twist_cmd_publisher
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS src/lgsvl_traffic_signal_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
