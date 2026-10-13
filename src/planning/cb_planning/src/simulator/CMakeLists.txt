cmake_minimum_required(VERSION 3.16)
project(vehicle_sim)

# simple trajectory follower
add_executable(${PROJECT_NAME}_exe
  ${CMAKE_CURRENT_SOURCE_DIR}/vehicle_sim_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_exe
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

# Install executable
install(TARGETS ${PROJECT_NAME}_exe
  DESTINATION lib/${PACKAGE_NAME}
)