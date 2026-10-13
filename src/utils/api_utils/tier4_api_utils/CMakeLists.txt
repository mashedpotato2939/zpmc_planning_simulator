cmake_minimum_required(VERSION 3.14)
project(tier4_api_utils)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  include_directories(include)
  ament_add_ros_isolated_gtest(${PROJECT_NAME}_test test/test.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp tier4_external_api_msgs)
endif()

ament_auto_package()
