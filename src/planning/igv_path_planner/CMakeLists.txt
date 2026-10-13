cmake_minimum_required(VERSION 3.14)
project(igv_path_planner)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Find dependencies
find_package(nlohmann_json REQUIRED)

# Path Generator Library (for lane change, turn, curve generation)
# Each maneuver-specific generator lives in src/generators/ so new types
# can be added by dropping in a new .cpp without touching path_generator.cpp.
ament_auto_add_library(path_generator SHARED
  src/generators/path_generator.cpp
  src/generators/oblique_generator.cpp
  src/generators/qturn_generator.cpp
)

# Map Path Generator Library (snaps FMS waypoints to HD Map lane centerlines)
ament_auto_add_library(map_path_generator SHARED
  src/generators/map_path_generator.cpp
)
target_link_libraries(map_path_generator path_generator)

# Crab-walking avoidance and reference-line return in a lane-aligned frame.
ament_auto_add_library(crab_avoidance_planner SHARED
  src/avoidance_planner/crab_avoidance_planner.cpp
)

# FMS Protocol Library (pluggable FMS protocol support)
ament_auto_add_library(fms_protocol SHARED
  src/protocols/fms_protocol.cpp
  src/protocols/default_protocol.cpp
  src/protocols/legacy_protocol.cpp
)
target_link_libraries(fms_protocol
  nlohmann_json::nlohmann_json
)

# FMS Path Converter Node
ament_auto_add_library(fms_path_converter_node SHARED
  src/fms_path_planning_node.cpp
  src/utils/path_planning_utils.cpp
)
target_link_libraries(fms_path_converter_node
  nlohmann_json::nlohmann_json
  path_generator
  map_path_generator
  crab_avoidance_planner
  fms_protocol
)
rclcpp_components_register_node(fms_path_converter_node
  PLUGIN "igv_path_planner::FMSPathConverterNode"
  EXECUTABLE igv_path_converter
)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(route_segment_selector_test test/route_segment_selector_test.cpp)
  target_compile_features(route_segment_selector_test PRIVATE cxx_std_17)
  target_include_directories(route_segment_selector_test PRIVATE include)
  ament_target_dependencies(route_segment_selector_test igv_planning_msgs geometry_msgs)
  ament_add_gtest(qturn_compare_test test/qturn_compare_test.cpp)
  target_include_directories(qturn_compare_test PRIVATE test)
  target_link_libraries(qturn_compare_test path_generator)
  ament_target_dependencies(qturn_compare_test
    autoware_planning_msgs geometry_msgs)

  ament_add_gtest(qturn_auto_sizing_test test/qturn_auto_sizing_test.cpp)
  target_link_libraries(qturn_auto_sizing_test path_generator)
  ament_target_dependencies(qturn_auto_sizing_test
    autoware_planning_msgs geometry_msgs)

  ament_add_gtest(path_generator_speed_limit_test
    test/path_generator_speed_limit_test.cpp)
  target_link_libraries(path_generator_speed_limit_test path_generator)
  ament_target_dependencies(path_generator_speed_limit_test
    autoware_planning_msgs geometry_msgs)

  ament_add_gtest(map_path_generator_reverse_test
    test/map_path_generator_reverse_test.cpp)
  target_link_libraries(map_path_generator_reverse_test map_path_generator)
  ament_target_dependencies(map_path_generator_reverse_test
    autoware_planning_msgs geometry_msgs lanelet2_core)

  ament_add_gtest(path_truncation_test test/path_truncation_test.cpp)
  target_link_libraries(path_truncation_test fms_path_converter_node)
  ament_target_dependencies(path_truncation_test
    rclcpp autoware_planning_msgs geometry_msgs)

  ament_add_gtest(crab_avoidance_planner_test
    test/crab_avoidance_planner_test.cpp)
  target_link_libraries(crab_avoidance_planner_test crab_avoidance_planner)
  ament_target_dependencies(crab_avoidance_planner_test
    autoware_planning_msgs autoware_perception_msgs geometry_msgs nav_msgs tf2)
endif()

# Install
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
