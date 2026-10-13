cmake_minimum_required(VERSION 3.14)
project(diagnostic_graph_aggregator)

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
  src/common/graph/error.cpp
  src/common/graph/data.cpp
  src/common/graph/config.cpp
  src/common/graph/loader.cpp
  src/common/graph/graph.cpp
  src/common/graph/units.cpp
)

ament_auto_add_executable(tree
  src/tool/tree.cpp
)
target_include_directories(tree PRIVATE src/common)

ament_auto_add_executable(plantuml
  src/tool/plantuml.cpp
)
target_include_directories(plantuml PRIVATE src/common)

ament_auto_add_library(aggregator SHARED
  src/node/aggregator.cpp
  src/node/availability.cpp
)
target_include_directories(aggregator PRIVATE src/common)
ament_target_dependencies(aggregator diagnostic_msgs rclcpp rclcpp_components tier4_system_msgs)

rclcpp_components_register_node(aggregator
  PLUGIN "diagnostic_graph_aggregator::AggregatorNode"
  EXECUTABLE aggregator_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  get_filename_component(RESOURCE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/test/files ABSOLUTE)
  ament_add_gtest(gtest_${PROJECT_NAME}
    test/src/test1.cpp
    test/src/test2.cpp
    test/src/utils.cpp
  )
  target_compile_definitions(gtest_${PROJECT_NAME} PRIVATE TEST_RESOURCE_PATH="${RESOURCE_PATH}")
  target_include_directories(gtest_${PROJECT_NAME} PRIVATE src/common)
  target_link_libraries(gtest_${PROJECT_NAME} ${PROJECT_NAME})
  ament_target_dependencies(gtest_${PROJECT_NAME} diagnostic_msgs rclcpp rclcpp_components tier4_system_msgs)
endif()

ament_auto_package(INSTALL_TO_SHARE config example launch)
