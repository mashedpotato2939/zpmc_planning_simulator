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

#include "igv_velocity_planner/fms_velocity_shaper_node.hpp"

#include "igv_velocity_planner/velocity_shaper/constraint_generator.hpp"
#include "igv_velocity_planner/velocity_shaper/diagnostics.hpp"
#include "igv_velocity_planner/velocity_shaper/geometry_utils.hpp"
#include "igv_velocity_planner/velocity_shaper/occupancy_debug.hpp"

#include "autoware/universe_utils/ros/update_param.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <autoware_adapi_v1_msgs/msg/velocity_factor.hpp>
#include <tier4_planning_msgs/msg/stop_factor.hpp>
#include <tier4_planning_msgs/msg/stop_reason.hpp>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace igv_velocity_planner
{

namespace vs = velocity_shaper;

FmsVelocityShaperNode::FmsVelocityShaperNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("fms_velocity_shaper", options),
  static_stop_gen_(params_.static_obstacle_stop, ego_front_clearance_, ego_half_width_),
  dynamic_gen_(params_.dynamic_obstacle_stop, ego_front_clearance_, ego_half_width_),
  occupancy_gen_(params_.occupancy_grid_check, params_.perception_inputs.use_occupancy_grid),
  slowdown_gen_(params_.slow_down, ego_half_width_),
  acc_gen_(params_.adaptive_cruise)
{
  // ---- Vehicle info ----------------------------------------------------
  // base_link → front-bumper distance feeds dynamic_obstacle_stop so the stop
  // point keeps the front bumper behind the obstacle's rear face. Half of the
  // vehicle width is added to every lateral threshold so the collision/slow-down
  // bands sit beyond the ego footprint, not at the path centerline.
  const auto vehicle_info =
    autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  ego_front_clearance_ = vehicle_info.max_longitudinal_offset_m;
  ego_min_longitudinal_offset_ = vehicle_info.min_longitudinal_offset_m;
  ego_half_width_ = 0.5 * vehicle_info.vehicle_width_m;

  // ---- Parameters ------------------------------------------------------
  auto & dos = params_.dynamic_obstacle_stop;
  dos.enable = declare_parameter<bool>("dynamic_obstacle_stop.enable", dos.enable);
  dos.minimum_object_velocity =
    declare_parameter<double>("dynamic_obstacle_stop.minimum_object_velocity", dos.minimum_object_velocity);
  dos.stop_distance_buffer =
    declare_parameter<double>("dynamic_obstacle_stop.stop_distance_buffer", dos.stop_distance_buffer);
  dos.time_horizon = declare_parameter<double>("dynamic_obstacle_stop.time_horizon", dos.time_horizon);
  dos.hysteresis = declare_parameter<double>("dynamic_obstacle_stop.hysteresis", dos.hysteresis);
  dos.extra_object_width =
    declare_parameter<double>("dynamic_obstacle_stop.extra_object_width", dos.extra_object_width);
  dos.add_stop_duration_buffer = declare_parameter<double>(
    "dynamic_obstacle_stop.add_stop_duration_buffer", dos.add_stop_duration_buffer);
  dos.remove_stop_duration_buffer = declare_parameter<double>(
    "dynamic_obstacle_stop.remove_stop_duration_buffer", dos.remove_stop_duration_buffer);
  dos.minimum_object_distance_from_ego_trajectory = declare_parameter<double>(
    "dynamic_obstacle_stop.minimum_object_distance_from_ego_trajectory",
    dos.minimum_object_distance_from_ego_trajectory);

  auto & sos = params_.static_obstacle_stop;
  sos.enable = declare_parameter<bool>("static_obstacle_stop.enable", sos.enable);
  sos.maximum_object_velocity = declare_parameter<double>(
    "static_obstacle_stop.maximum_object_velocity", sos.maximum_object_velocity);
  sos.lateral_clearance =
    declare_parameter<double>("static_obstacle_stop.lateral_clearance", sos.lateral_clearance);
  sos.stop_distance_buffer = declare_parameter<double>(
    "static_obstacle_stop.stop_distance_buffer", sos.stop_distance_buffer);
  sos.max_distance =
    declare_parameter<double>("static_obstacle_stop.max_distance", sos.max_distance);
  sos.add_stop_duration_buffer = declare_parameter<double>(
    "static_obstacle_stop.add_stop_duration_buffer", sos.add_stop_duration_buffer);
  sos.remove_stop_duration_buffer = declare_parameter<double>(
    "static_obstacle_stop.remove_stop_duration_buffer", sos.remove_stop_duration_buffer);

  auto & acc = params_.adaptive_cruise;
  acc.enable = declare_parameter<bool>("adaptive_cruise.enable", acc.enable);
  acc.follow_distance_min =
    declare_parameter<double>("adaptive_cruise.follow_distance_min", acc.follow_distance_min);
  acc.follow_time_gap = declare_parameter<double>("adaptive_cruise.follow_time_gap", acc.follow_time_gap);
  acc.velocity_margin = declare_parameter<double>("adaptive_cruise.velocity_margin", acc.velocity_margin);
  acc.max_deceleration = declare_parameter<double>("adaptive_cruise.max_deceleration", acc.max_deceleration);
  acc.leader_lateral_margin =
    declare_parameter<double>("adaptive_cruise.leader_lateral_margin", acc.leader_lateral_margin);
  acc.leader_heading_max_diff =
    declare_parameter<double>("adaptive_cruise.leader_heading_max_diff", acc.leader_heading_max_diff);
  acc.leader_search_distance =
    declare_parameter<double>("adaptive_cruise.leader_search_distance", acc.leader_search_distance);

  auto & tpp = params_.trajectory_preprocessing;
  tpp.resample_interval = declare_parameter<double>(
    "trajectory_preprocessing.resample_interval", tpp.resample_interval);

  auto & pi = params_.perception_inputs;
  pi.use_occupancy_grid =
    declare_parameter<bool>("perception_inputs.use_occupancy_grid", pi.use_occupancy_grid);

  auto & ogc = params_.occupancy_grid_check;
  ogc.occupancy_threshold = static_cast<int8_t>(
    declare_parameter<int>("occupancy_grid_check.occupancy_threshold", ogc.occupancy_threshold));
  ogc.required_clearance =
    declare_parameter<double>("occupancy_grid_check.required_clearance", ogc.required_clearance);
  ogc.enable_slow_down =
    declare_parameter<bool>("occupancy_grid_check.enable_slow_down", ogc.enable_slow_down);
  ogc.slow_band = declare_parameter<double>("occupancy_grid_check.slow_band", ogc.slow_band);
  ogc.slow_min_target_velocity = declare_parameter<double>(
    "occupancy_grid_check.slow_min_target_velocity", ogc.slow_min_target_velocity);
  ogc.slow_max_target_velocity = declare_parameter<double>(
    "occupancy_grid_check.slow_max_target_velocity", ogc.slow_max_target_velocity);
  ogc.stop_distance_buffer =
    declare_parameter<double>("occupancy_grid_check.stop_distance_buffer", ogc.stop_distance_buffer);
  ogc.max_distance = declare_parameter<double>("occupancy_grid_check.max_distance", ogc.max_distance);
  ogc.transform_timeout =
    declare_parameter<double>("occupancy_grid_check.transform_timeout", ogc.transform_timeout);

  auto & sd = params_.slow_down;
  sd.enable = declare_parameter<bool>("slow_down.enable", sd.enable);
  sd.inner_lateral_margin =
    declare_parameter<double>("slow_down.inner_lateral_margin", sd.inner_lateral_margin);
  sd.outer_lateral_margin =
    declare_parameter<double>("slow_down.outer_lateral_margin", sd.outer_lateral_margin);
  sd.min_target_velocity =
    declare_parameter<double>("slow_down.min_target_velocity", sd.min_target_velocity);
  sd.max_target_velocity =
    declare_parameter<double>("slow_down.max_target_velocity", sd.max_target_velocity);
  sd.forward_margin = declare_parameter<double>("slow_down.forward_margin", sd.forward_margin);
  sd.backward_margin = declare_parameter<double>("slow_down.backward_margin", sd.backward_margin);
  sd.max_distance = declare_parameter<double>("slow_down.max_distance", sd.max_distance);
  sd.add_duration_buffer =
    declare_parameter<double>("slow_down.add_duration_buffer", sd.add_duration_buffer);
  sd.remove_duration_buffer =
    declare_parameter<double>("slow_down.remove_duration_buffer", sd.remove_duration_buffer);

  // ---- Generator pipeline ----------------------------------------------
  // Order is irrelevant to the merged result (the merger resolves by priority),
  // but kept perception-source-grouped for readability.
  generators_ = {&static_stop_gen_, &dynamic_gen_, &occupancy_gen_, &slowdown_gen_, &acc_gen_};

  // ---- TF --------------------------------------------------------------
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ---- ROS interfaces --------------------------------------------------
  rclcpp::QoS qos(1);
  qos.reliable();

  path_sub_ = create_subscription<Path>(
    "~/input/path", qos, std::bind(&FmsVelocityShaperNode::onPath, this, std::placeholders::_1));
  objects_sub_ = create_subscription<PredictedObjects>(
    "~/input/objects", qos,
    std::bind(&FmsVelocityShaperNode::onObjects, this, std::placeholders::_1));
  odometry_sub_ = create_subscription<Odometry>(
    "~/input/odometry", qos,
    std::bind(&FmsVelocityShaperNode::onOdometry, this, std::placeholders::_1));
  // Free-space occupancy from perception. Subscribed from day one; the
  // occupancy generator is a benign no-op until the topic is published.
  occupancy_grid_sub_ = create_subscription<OccupancyGrid>(
    "~/input/occupancy_grid", qos,
    std::bind(&FmsVelocityShaperNode::onOccupancyGrid, this, std::placeholders::_1));

  trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", qos);
  velocity_factors_pub_ = create_publisher<VelocityFactorArray>("~/output/velocity_factors", qos);
  stop_reasons_pub_ = create_publisher<StopReasonArray>("~/output/stop_reasons", qos);
  stop_markers_pub_ = create_publisher<MarkerArray>("~/output/stop_markers", qos);

  rclcpp::QoS debug_qos(1);
  debug_qos.reliable();
  debug_qos.transient_local();
  occupancy_corridor_debug_pub_ =
    create_publisher<MarkerArray>("~/debug/occupancy_corridor", debug_qos);
  occupancy_distance_map_debug_pub_ =
    create_publisher<OccupancyGrid>("~/debug/occupancy_distance_map", debug_qos);

  set_param_handle_ = add_on_set_parameters_callback(
    std::bind(&FmsVelocityShaperNode::onSetParameters, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "FmsVelocityShaperNode initialized (constraint pipeline)");
}

void FmsVelocityShaperNode::onObjects(const PredictedObjects::ConstSharedPtr msg) { objects_ = msg; }
void FmsVelocityShaperNode::onOdometry(const Odometry::ConstSharedPtr msg) { odometry_ = msg; }
void FmsVelocityShaperNode::onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg)
{
  occupancy_grid_ = msg;
}

rcl_interfaces::msg::SetParametersResult FmsVelocityShaperNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware::universe_utils::updateParam;

  for (const auto & parameter : parameters) {
    if (parameter.get_name() != "trajectory_preprocessing.resample_interval") continue;

    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = "trajectory_preprocessing.resample_interval must be a double";
      return result;
    }

    if (parameter.as_double() <= 0.0) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = "trajectory_preprocessing.resample_interval must be positive";
      return result;
    }
  }

  updateParam<double>(
    parameters, "trajectory_preprocessing.resample_interval",
    params_.trajectory_preprocessing.resample_interval);
  updateParam<bool>(parameters, "dynamic_obstacle_stop.enable", params_.dynamic_obstacle_stop.enable);
  updateParam<bool>(parameters, "static_obstacle_stop.enable", params_.static_obstacle_stop.enable);
  updateParam<bool>(parameters, "adaptive_cruise.enable", params_.adaptive_cruise.enable);
  updateParam<bool>(parameters, "slow_down.enable", params_.slow_down.enable);
  updateParam<bool>(
    parameters, "perception_inputs.use_occupancy_grid",
    params_.perception_inputs.use_occupancy_grid);
  updateParam<bool>(
    parameters, "occupancy_grid_check.enable_slow_down",
    params_.occupancy_grid_check.enable_slow_down);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void FmsVelocityShaperNode::onPath(const Path::ConstSharedPtr msg)
{
  if (!msg) return;
  auto trajectory = shapeTrajectory(*msg);
  trajectory_pub_->publish(trajectory);
}

bool FmsVelocityShaperNode::transformPointsIntoGridFrame(
  const std::vector<TrajectoryPoint> & points, const std_msgs::msg::Header & path_header,
  std::vector<TrajectoryPoint> & out)
{
  if (!occupancy_grid_) return false;
  if (occupancy_grid_->header.frame_id == path_header.frame_id) {
    out = points;  // already in the grid frame
    return true;
  }
  try {
    const auto tf_stamped = tf_buffer_->lookupTransform(
      occupancy_grid_->header.frame_id, path_header.frame_id, path_header.stamp,
      rclcpp::Duration::from_seconds(params_.occupancy_grid_check.transform_timeout));
    tf2::Transform xform;
    tf2::fromMsg(tf_stamped.transform, xform);
    out.clear();
    out.reserve(points.size());
    for (const auto & p : points) {
      TrajectoryPoint tp = p;
      tf2::Vector3 v(p.pose.position.x, p.pose.position.y, p.pose.position.z);
      v = xform * v;
      tp.pose.position.x = v.x();
      tp.pose.position.y = v.y();
      tp.pose.position.z = v.z();
      out.push_back(tp);
    }
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "fms_velocity_shaper: occupancy_grid transform unavailable (%s); skipping occupancy", ex.what());
    return false;
  }
}

autoware_planning_msgs::msg::Trajectory FmsVelocityShaperNode::shapeTrajectory(const Path & path)
{
  Trajectory out;
  out.header = path.header;

  auto points = vs::convertPathToTrajectoryPoints(path);
  if (points.empty()) return out;
  points = vs::resampleTrajectoryPoints(
    points, params_.trajectory_preprocessing.resample_interval);

  const double ego_s = odometry_ != nullptr ?
    vs::projectOntoPath(points, odometry_->pose.pose.position).s :
    0.0;

  // --- 1) Snapshot the tick -------------------------------------------------
  vs::TickInput tick{points,
                     objects_.get(),
                     occupancy_grid_.get(),
                     odometry_.get(),
                     ego_front_clearance_,
                     ego_half_width_,
                     this->now(),
                     ego_s,
                     ego_min_longitudinal_offset_};

  // Feed the occupancy generator the grid-frame trajectory points (TF is a node
  // concern). On failure the generator gets nothing → no-op for this cycle.
  if (params_.perception_inputs.use_occupancy_grid &&
      transformPointsIntoGridFrame(points, path.header, grid_frame_points_)) {
    occupancy_gen_.setGridFramePoints(&grid_frame_points_);
  } else {
    grid_frame_points_.clear();
    occupancy_gen_.setGridFramePoints(nullptr);
  }

  // --- 2) Generate constraints ---------------------------------------------
  std::vector<vs::VelocityConstraint> constraints;
  for (auto * g : generators_) {
    if (g->enabled()) g->generate(tick, constraints);
  }
  emitOccupancyDebug(tick.grid);

  // --- 3) Merge into the raw velocity profile (mutates points) -------------
  const auto mr = vs::mergeConstraints(points, constraints);

  // --- 4) Emit ------------------------------------------------------------
  out.points.assign(points.begin(), points.end());
  emitDiagnostics(path.header, points, mr);
  return out;
}

void FmsVelocityShaperNode::emitOccupancyDebug(const OccupancyGrid * grid)
{
  if (!params_.perception_inputs.use_occupancy_grid || grid == nullptr) return;
  if (!occupancy_gen_.ensureDistanceMap(*grid)) return;

  const auto * distance_map = occupancy_gen_.distanceMap();
  if (distance_map == nullptr) return;

  const double slow_band =
    params_.occupancy_grid_check.enable_slow_down &&
        params_.occupancy_grid_check.slow_band > 0.0 ?
      params_.occupancy_grid_check.slow_band :
      0.0;
  const double display_distance_cap =
    ego_half_width_ + params_.occupancy_grid_check.required_clearance + slow_band;

  if (grid_frame_points_.size() >= 2) {
    auto markers = vs::makeOccupancyCorridorMarkers(
      grid->header, *grid, *distance_map, grid_frame_points_, ego_half_width_,
      params_.occupancy_grid_check.max_distance, display_distance_cap);
    if (!markers.markers.empty()) occupancy_corridor_debug_pub_->publish(markers);
  }

  auto distance_grid = vs::makeOccupancyDistanceMapGrid(*grid, *distance_map, display_distance_cap);
  if (distance_grid) occupancy_distance_map_debug_pub_->publish(*distance_grid);
}

void FmsVelocityShaperNode::emitDiagnostics(
  const std_msgs::msg::Header & header, const std::vector<TrajectoryPoint> & points,
  const vs::MergeResult & mr)
{
  VelocityFactorArray factors;
  factors.header = header;
  StopReasonArray reasons;
  reasons.header = header;
  MarkerArray markers;
  {
    // Lead with DELETEALL so stale stops from the previous cycle don't linger.
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
  }

  // First occurrence per distinct source (preserving order) — one diagnostic per
  // obstacle/pathway, matching the legacy per-pathway emission cardinality.
  struct Region
  {
    size_t idx;
    vs::ConstraintType type;
    std::optional<geometry_msgs::msg::Point> anchor;
  };
  std::vector<std::pair<std::string, Region>> regions;
  std::unordered_map<std::string, bool> seen;
  for (size_t i = 0; i < points.size() && i < mr.winner.size(); ++i) {
    if (mr.winner[i] == vs::ConstraintType::kFmsBase) continue;
    const auto & src = mr.source[i];
    if (seen.find(src) != seen.end()) continue;
    seen[src] = true;
    regions.push_back({src, Region{i, mr.winner[i], mr.anchor[i]}});
  }

  int hard_stop_marker_id = 0;
  for (const auto & [src, r] : regions) {
    const auto & stop_pose = points[r.idx].pose;
    const double s = vs::arcLengthTo(points, r.idx);

    switch (r.type) {
      case vs::ConstraintType::kAdaptiveCruise: {
        autoware_adapi_v1_msgs::msg::VelocityFactor vf;
        vf.pose = points.front().pose;
        vf.distance = 0.0f;
        vf.status = autoware_adapi_v1_msgs::msg::VelocityFactor::APPROACHING;
        vf.behavior = "follow";
        vf.sequence = "fms_velocity_shaper.adaptive_cruise";
        vf.detail = "following leader";
        factors.factors.push_back(vf);
        break;
      }
      case vs::ConstraintType::kSlowDown: {
        autoware_adapi_v1_msgs::msg::VelocityFactor vf;
        vf.pose = stop_pose;
        vf.distance = static_cast<float>(s);
        vf.status = autoware_adapi_v1_msgs::msg::VelocityFactor::APPROACHING;
        vf.behavior = "slow_down";
        if (src == "occupancy_slow_down") {
          vf.sequence = "fms_velocity_shaper.occupancy_slow_down";
          vf.detail = "slowing for occupied cells near path";
        } else {
          vf.sequence = "fms_velocity_shaper.slow_down";
          vf.detail = "slowing for obstacle near path";
        }
        factors.factors.push_back(vf);
        break;
      }
      case vs::ConstraintType::kOccupancyStop:
      case vs::ConstraintType::kStaticObstacleStop:
      case vs::ConstraintType::kDynamicStop: {
        tier4_planning_msgs::msg::StopReason reason;
        reason.reason = tier4_planning_msgs::msg::StopReason::OBSTACLE_STOP;
        tier4_planning_msgs::msg::StopFactor factor;
        factor.stop_pose = stop_pose;
        factor.dist_to_stop_pose = s;
        if (r.anchor) factor.stop_factor_points.push_back(*r.anchor);
        reason.stop_factors.push_back(factor);
        reasons.stop_reasons.push_back(reason);

        autoware_adapi_v1_msgs::msg::VelocityFactor vf;
        vf.pose = stop_pose;
        vf.distance = static_cast<float>(s);
        vf.status = autoware_adapi_v1_msgs::msg::VelocityFactor::APPROACHING;
        vf.behavior = "stop";
        vf.sequence = vs::hardStopVelocityFactorSequence(r.type);
        vf.detail = vs::hardStopReasonText(r.type);
        factors.factors.push_back(vf);

        vs::HardStopMarkerData marker_data;
        marker_data.type = r.type;
        marker_data.stop_pose = stop_pose;
        marker_data.stop_distance = s;
        marker_data.anchor = r.anchor;
        vs::appendHardStopMarkers(markers, header, marker_data, hard_stop_marker_id++);
        break;
      }
      default:
        break;  // kFmsBase emit no diagnostics (as before)
    }
  }

  velocity_factors_pub_->publish(factors);
  stop_reasons_pub_->publish(reasons);
  stop_markers_pub_->publish(markers);
}

}  // namespace igv_velocity_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(igv_velocity_planner::FmsVelocityShaperNode)
