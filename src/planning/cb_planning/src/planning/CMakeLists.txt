cmake_minimum_required(VERSION 3.16)
project(planning)

add_library(${PROJECT_NAME}_lib SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/cb_path_planning.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cb_velocity_planning.cpp
  # ${CMAKE_CURRENT_SOURCE_DIR}/path_planner/optimization_path_planning.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/cb_planning_node.cpp
)

add_executable(${PROJECT_NAME}_exe
  ${CMAKE_CURRENT_SOURCE_DIR}/cb_planning_node_main.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  autoware_planning_msgs
  tf2
  tf2_ros
  pacmod_msgs
)

target_link_libraries(${PROJECT_NAME}_exe
  ${PROJECT_NAME}_lib
  config_reader
)

# 安装动态库
install(TARGETS ${PROJECT_NAME}_lib
  DESTINATION lib
)

# Install executable
install(TARGETS ${PROJECT_NAME}_exe
  DESTINATION lib/${PACKAGE_NAME}
)