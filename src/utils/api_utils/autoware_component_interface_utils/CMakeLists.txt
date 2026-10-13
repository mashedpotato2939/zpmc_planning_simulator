cmake_minimum_required(VERSION 3.14)
project(autoware_component_interface_utils)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  SYSTEM
  ${rclcpp_INCLUDE_DIRS}
  ${rosidl_runtime_cpp_INCLUDE_DIRS}
  ${rcl_INCLUDE_DIRS}
  ${autoware_adapi_v1_msgs_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  ament_add_ros_isolated_gtest(test_component_interface_utils
    test/test_component_interface_utils.cpp
  )

  target_include_directories(test_component_interface_utils
    PRIVATE include
  )
endif()

ament_auto_package()
