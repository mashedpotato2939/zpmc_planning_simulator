cmake_minimum_required(VERSION 3.14)
project(ad_api_adaptors)

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
  src/initial_pose_adaptor.cpp
  src/routing_adaptor.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ad_api_adaptors::InitialPoseAdaptor"
  EXECUTABLE initial_pose_adaptor_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ad_api_adaptors::RoutingAdaptor"
  EXECUTABLE routing_adaptor_node
)

ament_auto_package(INSTALL_TO_SHARE config launch)
