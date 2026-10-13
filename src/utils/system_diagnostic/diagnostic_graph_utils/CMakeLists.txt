cmake_minimum_required(VERSION 3.14)
project(diagnostic_graph_utils)

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
  src/lib/graph.cpp
  src/lib/subscription.cpp
)

ament_auto_add_library(${PROJECT_NAME}_tools SHARED
  src/node/dump.cpp
  src/node/converter.cpp
  src/node/logging.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_tools
  PLUGIN "diagnostic_graph_utils::DumpNode"
  EXECUTABLE dump_node
)

rclcpp_components_register_node(${PROJECT_NAME}_tools
  PLUGIN "diagnostic_graph_utils::ConverterNode"
  EXECUTABLE converter_node
)

rclcpp_components_register_node(${PROJECT_NAME}_tools
  PLUGIN "diagnostic_graph_utils::LoggingNode"
  EXECUTABLE logging_node
)

ament_auto_package(INSTALL_TO_SHARE launch)
