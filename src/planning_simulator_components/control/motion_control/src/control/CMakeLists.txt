cmake_minimum_required(VERSION 3.16)
project(control)

add_library(${PROJECT_NAME}_lib SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/motion_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/trajectory_following.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/precise_positioning.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/trajectory_tracker.cpp
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
  reverse_vehicle_msg
  std_msgs
  visualization_msgs
)

add_executable(${PROJECT_NAME}_exe
  ${CMAKE_CURRENT_SOURCE_DIR}/motion_controller_main.cpp
)

target_link_libraries(${PROJECT_NAME}_exe
  ${PROJECT_NAME}_lib
  control_utils_lib
)

ament_target_dependencies(${PROJECT_NAME}_exe
  rclcpp
  autoware_planning_msgs
  autoware_control_msgs
  reverse_vehicle_msg
)

# 安装动态库
install(TARGETS ${PROJECT_NAME}_lib
  DESTINATION lib
)

# Install executable
install(TARGETS ${PROJECT_NAME}_exe
  DESTINATION lib/${PACKAGE_NAME}
)