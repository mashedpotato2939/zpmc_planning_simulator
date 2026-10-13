cmake_minimum_required(VERSION 3.14)
project(autoware_path_distance_calculator)

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
  src/path_distance_calculator.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::path_distance_calculator::PathDistanceCalculator"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE launch)
