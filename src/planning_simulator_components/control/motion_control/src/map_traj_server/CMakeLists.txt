cmake_minimum_required(VERSION 3.16)
project(map_traj_server)

# simple trajectory follower
add_executable(${PROJECT_NAME}_exe
  ${CMAKE_CURRENT_SOURCE_DIR}/map_traj_server.cpp
)

target_link_libraries(${PROJECT_NAME}_exe
  control_utils_lib
)

ament_target_dependencies(${PROJECT_NAME}_exe
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  autoware_planning_msgs
  autoware_control_msgs
  # 20250912,zhangyue
  std_msgs
)

# Install executable
install(TARGETS ${PROJECT_NAME}_exe
  DESTINATION lib/${PACKAGE_NAME}
)