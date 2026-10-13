cmake_minimum_required(VERSION 3.14)
project(planning_ego_acceleration_estimator)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp_components REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ego_acceleration_estimator_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "planning_ego_acceleration_estimator::EgoAccelerationEstimator"
  EXECUTABLE ego_acceleration_estimator_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
