// Copyright 2026 ZPMC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FMS_TRAJECTORY_BRIDGE__FMS_VELOCITY_SHAPER_NODE_HPP_
#define FMS_TRAJECTORY_BRIDGE__FMS_VELOCITY_SHAPER_NODE_HPP_

#include "igv_velocity_planner/velocity_shaper/constraint_merger.hpp"
#include "igv_velocity_planner/velocity_shaper/generators.hpp"
#include "igv_velocity_planner/velocity_shaper/occupancy_stop.hpp"
#include "igv_velocity_planner/velocity_shaper/params.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/velocity_factor_array.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tier4_planning_msgs/msg/stop_reason_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <vector>

namespace igv_velocity_planner
{

// FMS-mode velocity shaper. Subscribes to the FMS-shaped Path + perception
// inputs, runs a set of constraint generators (one per pathway) against a
// single tick snapshot, merges the resulting VelocityConstraints into a raw
// velocity profile, and publishes the shaped Trajectory plus diagnostics
// (VelocityFactor / StopReason / markers). Generators and the merger live under
// `include/igv_velocity_planner/velocity_shaper/`; this node is the orchestrator.
class FmsVelocityShaperNode : public rclcpp::Node
{
public:
  explicit FmsVelocityShaperNode(const rclcpp::NodeOptions & options);

private:
  using Path = autoware_planning_msgs::msg::Path;
  using Trajectory = autoware_planning_msgs::msg::Trajectory;
  using TrajectoryPoint = autoware_planning_msgs::msg::TrajectoryPoint;
  using PredictedObjects = autoware_perception_msgs::msg::PredictedObjects;
  using Odometry = nav_msgs::msg::Odometry;
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
  using VelocityFactorArray = autoware_adapi_v1_msgs::msg::VelocityFactorArray;
  using StopReasonArray = tier4_planning_msgs::msg::StopReasonArray;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  void onPath(const Path::ConstSharedPtr msg);
  void onObjects(const PredictedObjects::ConstSharedPtr msg);
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg);

  // Runtime-live toggles: lets each perception pathway be enabled/disabled via
  // `ros2 param set` while the node runs (debug: isolate perception impact).
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  Trajectory shapeTrajectory(const Path & path);

  // Transform `points` into the occupancy grid's frame (identity copy when the
  // frames already match). Returns false on TF failure — the caller then leaves
  // the occupancy generator unfed, so it no-ops this cycle.
  bool transformPointsIntoGridFrame(
    const std::vector<TrajectoryPoint> & points, const std_msgs::msg::Header & path_header,
    std::vector<TrajectoryPoint> & out);

  // Emit VelocityFactor / StopReason / Marker diagnostics from the merge result.
  void emitDiagnostics(
    const std_msgs::msg::Header & header, const std::vector<TrajectoryPoint> & points,
    const velocity_shaper::MergeResult & mr);
  void emitOccupancyDebug(const OccupancyGrid * grid);

  // ROS interfaces
  rclcpp::Subscription<Path>::SharedPtr path_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr objects_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr occupancy_grid_sub_;

  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<VelocityFactorArray>::SharedPtr velocity_factors_pub_;
  rclcpp::Publisher<StopReasonArray>::SharedPtr stop_reasons_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr stop_markers_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr occupancy_corridor_debug_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_distance_map_debug_pub_;

  // TF — transforms the occupancy_grid (when its frame differs from the
  // trajectory frame) into the trajectory's frame for the corridor query.
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Cached inputs
  PredictedObjects::ConstSharedPtr objects_;
  Odometry::ConstSharedPtr odometry_;
  OccupancyGrid::ConstSharedPtr occupancy_grid_;

  velocity_shaper::Params params_;

  // base_link offsets and half the vehicle width, fed into the generators so
  // collision/slow-down bands sit beyond the ego footprint.
  double ego_front_clearance_{0.0};
  double ego_min_longitudinal_offset_{0.0};
  double ego_half_width_{0.0};

  // Constraint generators (own per-UUID hysteresis state). Declared after
  // params_/ego_* so their reference members bind to live values.
  velocity_shaper::StaticObstacleStopGenerator static_stop_gen_;
  velocity_shaper::DynamicStopGenerator dynamic_gen_;
  velocity_shaper::OccupancyStopGenerator occupancy_gen_;
  velocity_shaper::SlowDownGenerator slowdown_gen_;
  velocity_shaper::AdaptiveCruiseGenerator acc_gen_;
  std::vector<velocity_shaper::IConstraintGenerator *> generators_;

  // Scratch buffer for the grid-frame trajectory points, refreshed per cycle
  // and handed to the occupancy generator.
  std::vector<TrajectoryPoint> grid_frame_points_;

  // Handle for the on-set-parameters callback that makes the enable flags live.
  OnSetParametersCallbackHandle::SharedPtr set_param_handle_;
};

}  // namespace igv_velocity_planner

#endif  // FMS_TRAJECTORY_BRIDGE__FMS_VELOCITY_SHAPER_NODE_HPP_
