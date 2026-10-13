cmake_minimum_required(VERSION 3.16)
project(control_utils)

add_library(${PROJECT_NAME}_lib SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/pid.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/interpolation.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/geometry.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/trajectory.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
  autoware_planning_msgs
  autoware_control_msgs
  pacmod_msgs
  rosbag2_cpp
)

install(TARGETS ${PROJECT_NAME}_lib
  DESTINATION lib
)
