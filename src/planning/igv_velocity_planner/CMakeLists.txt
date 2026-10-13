cmake_minimum_required(VERSION 3.14)
project(igv_velocity_planner)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Detected Objects -> OccupancyGrid rasterizer — OFF-BY-DEFAULT fallback. Feeds
# the velocity shaper's occupancy pathway from close-range DetectedObjects
# (footprint inflated by the ego half-width) when the probabilistic perception
# OGM under-marks a tracked object on the path. Selected via enable_detection_grid
# in the FMS launch; the shaper consumes /perception/occupancy_grid_map/map by default.
ament_auto_add_library(detected_objects_grid_lib SHARED
  src/detected_objects_grid.cpp
)
ament_auto_add_library(detected_objects_grid_node SHARED
  src/detected_objects_grid_node.cpp
)
target_link_libraries(detected_objects_grid_node detected_objects_grid_lib)
rclcpp_components_register_node(detected_objects_grid_node
  PLUGIN "igv_velocity_planner::DetectedObjectsToGridNode"
  EXECUTABLE detected_objects_to_grid
)

# Velocity-shaper pathway library — one file per concern (mirrors the
# kinematic_smoother folder layout). The node target below links against it.
ament_auto_add_library(velocity_shaper_lib SHARED
  src/velocity_shaper/geometry_utils.cpp
  src/velocity_shaper/diagnostics.cpp
  src/velocity_shaper/static_obstacle_stop.cpp
  src/velocity_shaper/dynamic_obstacle_stop.cpp
  src/velocity_shaper/adaptive_cruise.cpp
  src/velocity_shaper/slow_down.cpp
  src/velocity_shaper/constraint_merger.cpp
  src/velocity_shaper/distance_map.cpp
  src/velocity_shaper/occupancy_debug.cpp
  src/velocity_shaper/occupancy_stop.cpp
  src/velocity_shaper/generators.cpp
)
ament_target_dependencies(velocity_shaper_lib
  igv_motion_utils
)

# FMS Velocity Shaper Node — single owner of obstacle-driven v(s). Produces
# Trajectory from Path + perception inputs. The pathway algorithms live in
# velocity_shaper_lib; this target is just the node shell.
ament_auto_add_library(fms_velocity_shaper_node SHARED
  src/fms_velocity_shaper_node.cpp
)
target_link_libraries(fms_velocity_shaper_node velocity_shaper_lib)
rclcpp_components_register_node(fms_velocity_shaper_node
  PLUGIN "igv_velocity_planner::FmsVelocityShaperNode"
  EXECUTABLE igv_velocity_shaper
)

# Kinematic-feasibility library (ported from autoware_velocity_smoother;
# Analytical algorithm only — no OSQP / qp_interface dependency).
ament_auto_add_library(kinematic_smoother_lib SHARED
  src/kinematic_smoother/smoother_base.cpp
  src/kinematic_smoother/analytical_smoother.cpp
  src/kinematic_smoother/velocity_planning_utils.cpp
  src/kinematic_smoother/trajectory_utils.cpp
  src/kinematic_smoother/resample.cpp
)
ament_target_dependencies(kinematic_smoother_lib
  igv_motion_utils
)

# FMS Kinematic Smoother Node — slim FMS-mode replacement for
# autoware_velocity_smoother (Analytical algorithm; engage/stop/lat-acc
# filters and OBLIQUE/Q_TURN orientation cache preserved).
ament_auto_add_library(fms_kinematic_smoother_node SHARED
  src/fms_kinematic_smoother_node.cpp
)
target_link_libraries(fms_kinematic_smoother_node
  kinematic_smoother_lib
)
ament_target_dependencies(fms_kinematic_smoother_node
  igv_motion_utils
)
rclcpp_components_register_node(fms_kinematic_smoother_node
  PLUGIN "igv_velocity_planner::FmsKinematicSmootherNode"
  EXECUTABLE igv_kinematic_smoother
)

# Simple Object Predictor Node (velocity-based, no map dependency)
ament_auto_add_library(simple_object_predictor_node SHARED
  src/simple_object_predictor_node.cpp
)
rclcpp_components_register_node(simple_object_predictor_node
  PLUGIN "igv_velocity_planner::SimpleObjectPredictorNode"
  EXECUTABLE simple_object_predictor
)

# Trajectory Switch Node
ament_auto_add_library(trajectory_switch_node SHARED
  src/trajectory_switch_node.cpp
)
rclcpp_components_register_node(trajectory_switch_node
  PLUGIN "igv_velocity_planner::TrajectorySwitchNode"
  EXECUTABLE trajectory_switch
)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(fms_velocity_shaper_test
    test/fms_velocity_shaper_test.cpp)
  target_link_libraries(fms_velocity_shaper_test velocity_shaper_lib)
  ament_target_dependencies(fms_velocity_shaper_test
    autoware_planning_msgs autoware_perception_msgs geometry_msgs nav_msgs std_msgs
    visualization_msgs)

  ament_add_gtest(fms_kinematic_smoother_test
    test/fms_kinematic_smoother_test.cpp)
  target_link_libraries(fms_kinematic_smoother_test kinematic_smoother_lib)
  ament_target_dependencies(fms_kinematic_smoother_test
    autoware_planning_msgs geometry_msgs)

  # Constraint merger: priority/min resolution + back-to-front stop insertion.
  ament_add_gtest(constraint_merger_test
    test/constraint_merger_test.cpp)
  target_link_libraries(constraint_merger_test velocity_shaper_lib)
  ament_target_dependencies(constraint_merger_test
    autoware_planning_msgs geometry_msgs)

  # Distance map (EDT) + occupancy corridor generator.
  ament_add_gtest(occupancy_distance_map_test
    test/occupancy_distance_map_test.cpp)
  target_link_libraries(occupancy_distance_map_test velocity_shaper_lib)
  ament_target_dependencies(occupancy_distance_map_test
    autoware_planning_msgs geometry_msgs nav_msgs visualization_msgs)

  # Generator wrappers: emitted constraints match the legacy pathway behavior.
  ament_add_gtest(generators_test
    test/generators_test.cpp)
  target_link_libraries(generators_test velocity_shaper_lib)
  ament_target_dependencies(generators_test
    autoware_planning_msgs autoware_perception_msgs geometry_msgs nav_msgs)

  # Detection-grid rasterizer + producer->consumer integration with the new
  # OccupancyStopGenerator (the off-by-default fallback path).
  ament_add_gtest(detected_objects_grid_test
    test/detected_objects_grid_test.cpp)
  target_link_libraries(detected_objects_grid_test
    detected_objects_grid_lib velocity_shaper_lib)
  ament_target_dependencies(detected_objects_grid_test
    autoware_planning_msgs autoware_perception_msgs geometry_msgs nav_msgs)
endif()

# Install
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
