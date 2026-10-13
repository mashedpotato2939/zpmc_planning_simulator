// Copyright 2024 ZPMC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "igv_path_planner/fms_path_converter_node.hpp"
#include "igv_path_planner/utils/path_planning_utils.hpp"

#include "igv_path_planner/protocols/default_protocol.hpp"
#include "igv_path_planner/protocols/legacy_protocol.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace igv_path_planner
{
namespace
{

double normalizeYaw(double yaw)
{
  while (yaw > M_PI) yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;
  return yaw;
}

size_t appendPathPoints(
  std::vector<autoware_planning_msgs::msg::PathPoint> & dst,
  std::vector<bool> & avoidance_allowed,
  const std::vector<autoware_planning_msgs::msg::PathPoint> & src,
  const bool allow_avoidance)
{
  size_t first_index = dst.size();
  bool contributed = false;
  for (const auto & point : src) {
    if (!dst.empty() &&
      utils::distance2D(dst.back().pose.position, point.pose.position) < 0.05)
    {
      if (!contributed) first_index = dst.size() - 1;
      avoidance_allowed.back() = avoidance_allowed.back() && allow_avoidance;
      contributed = true;
      continue;
    }
    dst.push_back(point);
    avoidance_allowed.push_back(allow_avoidance);
    if (!contributed) first_index = dst.size() - 1;
    contributed = true;
  }
  return contributed ? first_index : dst.size();
}

void cropPathForwardLength(autoware_planning_msgs::msg::Path & path, const double max_length)
{
  if (path.points.size() < 2 || !(max_length > 0.0)) {
    return;
  }

  const bool have_bounds =
    path.left_bound.size() == path.points.size() && path.right_bound.size() == path.points.size();
  double length = 0.0;
  for (size_t i = 1; i < path.points.size(); ++i) {
    const double segment_length =
      utils::distance2D(path.points[i - 1].pose.position, path.points[i].pose.position);
    if (length + segment_length <= max_length) {
      length += segment_length;
      continue;
    }

    const double ratio = segment_length > 1e-6 ? (max_length - length) / segment_length : 0.0;
    auto end_point = utils::interpolatePathPoint(path.points[i - 1], path.points[i], ratio);
    end_point.is_final = false;
    path.points.erase(path.points.begin() + i, path.points.end());
    if (utils::distance2D(path.points.back().pose.position, end_point.pose.position) > 0.05) {
      path.points.push_back(end_point);
    } else {
      path.points.back() = end_point;
    }

    if (have_bounds) {
      const auto left =
        utils::interpolatePoint(path.left_bound[i - 1], path.left_bound[i], ratio);
      const auto right =
        utils::interpolatePoint(path.right_bound[i - 1], path.right_bound[i], ratio);
      path.left_bound.erase(path.left_bound.begin() + i, path.left_bound.end());
      path.right_bound.erase(path.right_bound.begin() + i, path.right_bound.end());
      if (path.left_bound.size() < path.points.size()) {
        path.left_bound.push_back(left);
        path.right_bound.push_back(right);
      } else {
        path.left_bound.back() = left;
        path.right_bound.back() = right;
      }
    }
    return;
  }
}

void applyTerminalStopProfile(
  autoware_planning_msgs::msg::Path & path, const double deceleration)
{
  double remaining = 0.0;
  for (size_t i = path.points.size(); i-- > 0;) {
    if (i + 1 < path.points.size()) {
      remaining +=
        utils::distance2D(path.points[i].pose.position, path.points[i + 1].pose.position);
    }
    auto & point = path.points[i];
    const double limit = std::sqrt(2.0 * deceleration * remaining);
    const double old = std::abs(point.longitudinal_velocity_mps);
    const double ratio = old > 1e-6 ? std::min(1.0, limit / old) : 0.0;
    point.longitudinal_velocity_mps *= ratio;
    point.lateral_velocity_mps *= ratio;
  }
}

}  // namespace

FMSPathConverterNode::FMSPathConverterNode(const rclcpp::NodeOptions & options)
: Node("fms_path_converter", options)
{
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  default_velocity_ = declare_parameter<double>("default_velocity", default_velocity_);
  min_velocity_ = declare_parameter<double>("min_velocity", min_velocity_);
  max_velocity_ = declare_parameter<double>("max_velocity", max_velocity_);
  path_interval_ = declare_parameter<double>("path_interval", path_interval_);
  forward_path_length_ = declare_parameter<double>("forward_path_length", forward_path_length_);
  backward_path_length_ = declare_parameter<double>("backward_path_length", backward_path_length_);
  stale_threshold_ = declare_parameter<double>("stale_threshold", stale_threshold_);
  ego_nearest_dist_threshold_ =
    declare_parameter<double>("ego_nearest_dist_threshold", ego_nearest_dist_threshold_);
  ego_nearest_yaw_threshold_ =
    declare_parameter<double>("ego_nearest_yaw_threshold", ego_nearest_yaw_threshold_);
  replan_lateral_deviation_threshold_ = declare_parameter<double>(
    "replan_lateral_deviation_threshold", replan_lateral_deviation_threshold_);
  publish_rate_ = declare_parameter<double>("publish_rate", publish_rate_);

  auto & tracking = route_segment_selector_.parameters;
  tracking.max_distance = declare_parameter<double>("segment_tracking.max_distance", 10.0);
  tracking.max_yaw_error =
    declare_parameter<double>("segment_tracking.max_yaw_error", ego_nearest_yaw_threshold_);
  tracking.connector_max_distance =
    declare_parameter<double>("segment_tracking.connector_max_distance", 50.0);
  tracking.connector_max_yaw_error =
    declare_parameter<double>("segment_tracking.connector_max_yaw_error", M_PI / 4.0);
  tracking.endpoint_distance = declare_parameter<double>("segment_tracking.endpoint_distance", 0.5);
  tracking.stopped_speed = declare_parameter<double>("segment_tracking.stopped_speed", 0.1);
  tracking.stopped_duration = declare_parameter<double>("segment_tracking.stopped_duration", 1.0);
  tracking.ambiguity_margin = declare_parameter<double>("segment_tracking.ambiguity_margin", 0.25);
  tracking.backtrack_tolerance = declare_parameter<double>("segment_tracking.backtrack_tolerance", 1.0);
  tracking.progress_slack = declare_parameter<double>("segment_tracking.progress_slack", 2.0);

  if (!(tracking.endpoint_distance > 0.0) || !(tracking.stopped_speed >= 0.0) ||
    !(tracking.stopped_duration > 0.0) || !(tracking.ambiguity_margin >= 0.0) ||
    !(tracking.backtrack_tolerance >= 0.0) || !(tracking.progress_slack >= 0.0) ||
    !(tracking.max_distance > 0.0) || !(tracking.max_yaw_error > 0.0) ||
    !(tracking.connector_max_distance > 0.0) || !(tracking.connector_max_yaw_error > 0.0)) {
    throw std::invalid_argument("Invalid segment tracking thresholds");
  }

  PathGenerator::Parameters gen_params;
  gen_params.default_velocity = default_velocity_;
  gen_params.path_interval = path_interval_;
  gen_params.lane_change_lateral_threshold =
    declare_parameter<double>("lane_change_lateral_threshold", gen_params.lane_change_lateral_threshold);
  gen_params.turn_yaw_threshold =
    declare_parameter<double>("turn_yaw_threshold", gen_params.turn_yaw_threshold);
  gen_params.min_lane_change_length =
    declare_parameter<double>("min_lane_change_length", gen_params.min_lane_change_length);
  gen_params.lane_change_velocity_ratio =
    declare_parameter<double>("lane_change_velocity_ratio", gen_params.lane_change_velocity_ratio);
  gen_params.turn_speed = declare_parameter<double>("turn_speed", gen_params.turn_speed);
  gen_params.qturn_speed_limit =
    declare_parameter<double>("qturn_speed_limit", gen_params.qturn_speed_limit);
  gen_params.oblique_speed_limit =
    declare_parameter<double>("oblique_speed_limit", gen_params.oblique_speed_limit);
  gen_params.qturn_min_radius =
    declare_parameter<double>("qturn_min_radius", gen_params.qturn_min_radius);
  gen_params.qturn_yaw_min_radius =
    declare_parameter<double>("qturn_yaw_min_radius", gen_params.qturn_yaw_min_radius);
  gen_params.enable_4ws = declare_parameter<bool>("enable_4ws", gen_params.enable_4ws);
  gen_params.vehicle_wheel_base =
    declare_parameter<double>("vehicle_wheel_base", gen_params.vehicle_wheel_base);
  gen_params.vehicle_length =
    declare_parameter<double>("vehicle_length", gen_params.vehicle_length);
  const double vehicle_width = declare_parameter<double>("vehicle_width", 3.0);
  gen_params.vehicle_front_overhang =
    declare_parameter<double>("vehicle_front_overhang", gen_params.vehicle_front_overhang);
  gen_params.max_lateral_velocity =
    declare_parameter<double>("max_lateral_velocity", gen_params.max_lateral_velocity);
  gen_params.curvature_4ws_threshold =
    declare_parameter<double>("curvature_4ws_threshold", gen_params.curvature_4ws_threshold);
  gen_params.lateral_velocity_gain =
    declare_parameter<double>("lateral_velocity_gain", gen_params.lateral_velocity_gain);
  gen_params.oblique_max_lateral_distance = declare_parameter<double>(
    "oblique_max_lateral_distance", gen_params.oblique_max_lateral_distance);
  gen_params.crab_dymax = declare_parameter<double>("crab_dymax", gen_params.crab_dymax);
  gen_params.crab_ddymax = declare_parameter<double>("crab_ddymax", gen_params.crab_ddymax);
  gen_params.crab_ddymin = declare_parameter<double>("crab_ddymin", gen_params.crab_ddymin);
  gen_params.rear_steer_ratio =
    declare_parameter<double>("rear_steer_ratio", gen_params.rear_steer_ratio);
  gen_params.debug = declare_parameter<bool>("path_generator_debug", false);
  path_generator_ = std::make_unique<PathGenerator>(gen_params);

  const double lane_width = declare_parameter<double>("lane_width", 3.5);
  crab_avoidance_enabled_ =
    declare_parameter<bool>("crab_avoidance.enable", false);
  crab_planning_params_.enable_dynamic_obstacle_avoidance = declare_parameter<bool>(
    "crab_avoidance.dynamic_obstacles.enable", false);
  crab_planning_params_.ego_length = gen_params.vehicle_length;
  crab_planning_params_.ego_width = vehicle_width;
  crab_planning_params_.obstacle_clearance = declare_parameter<double>(
    "crab_avoidance.clearance", 0.5);
  const double max_lateral_offset = declare_parameter<double>(
    "crab_avoidance.max_lateral_offset", 3.5);
  crab_planning_params_.lane_sampling_begin_y = -max_lateral_offset;
  crab_planning_params_.lane_sampling_end_y = max_lateral_offset;
  crab_planning_params_.lane_y_sampling_interval = declare_parameter<double>(
    "crab_avoidance.lateral_sampling_interval", 0.25);
  crab_planning_params_.lane_x_sampling_interval = declare_parameter<double>(
    "crab_avoidance.longitudinal_sampling_interval", 1.0);
  crab_planning_params_.lane_width = lane_width;
  crab_planning_params_.lane_num = 3;
  crab_planning_params_.lane_begin_y = -1.5 * lane_width;
  crab_planning_params_.Min_Collision_free_distance = declare_parameter<double>(
    "crab_avoidance.min_collision_free_distance", 20.0);
  crab_planning_params_.local_map_forward_x = declare_parameter<double>(
    "crab_avoidance.local_map_forward_x", 50.0);
  crab_planning_params_.local_map_backward_x = backward_path_length_;
  crab_planning_params_.start_point_mindis_threshold = declare_parameter<double>(
    "crab_avoidance.start_point_mindis_threshold", 1.0);
  crab_planning_params_.steer_angle_max = declare_parameter<double>(
    "crab_avoidance.steer_angle_max_deg", 30.0);
  crab_planning_params_.steer_angular_vel_max = declare_parameter<double>(
    "crab_avoidance.steer_angular_vel_max_deg_s", 30.0);
  crab_planning_params_.return_length_lateral_ratio = declare_parameter<double>(
    "crab_avoidance.return_length_lateral_ratio", 5.0);
  crab_planning_params_.reference_merge_slope_tolerance = declare_parameter<double>(
    "crab_avoidance.reference_merge_slope_tolerance", 0.02);
  crab_max_heading_curvature_ = declare_parameter<double>(
    "crab_avoidance.max_heading_curvature", crab_max_heading_curvature_);
  crab_max_heading_deviation_ = declare_parameter<double>(
    "crab_avoidance.max_heading_deviation", crab_max_heading_deviation_);
  crab_avoidance_object_timeout_ = declare_parameter<double>(
    "crab_avoidance.object_timeout", crab_avoidance_object_timeout_);
  if (!(crab_planning_params_.ego_length > 0.0) ||
    !(crab_planning_params_.ego_width > 0.0) ||
    !(lane_width > 0.0) ||
    !(crab_planning_params_.obstacle_clearance >= 0.0) ||
    !(max_lateral_offset > 0.0) ||
    !(crab_planning_params_.lane_y_sampling_interval > 0.0) ||
    !(crab_planning_params_.lane_x_sampling_interval > 0.0) ||
    !(crab_planning_params_.Min_Collision_free_distance > 0.0) ||
    !(crab_planning_params_.local_map_forward_x >=
      crab_planning_params_.Min_Collision_free_distance) ||
    !(crab_planning_params_.steer_angle_max > 0.0) ||
    !(crab_planning_params_.steer_angular_vel_max > 0.0) ||
    !(crab_planning_params_.return_length_lateral_ratio > 0.0) ||
    !(crab_planning_params_.reference_merge_slope_tolerance >= 0.0) ||
    !(crab_max_heading_curvature_ > 0.0) ||
    !(crab_max_heading_deviation_ > 0.0) ||
    !(crab_avoidance_object_timeout_ > 0.0))
  {
    throw std::invalid_argument("Invalid crab avoidance parameters");
  }
  crab_avoidance_planner_ = std::make_unique<cb_planning::CB_Path_Planning>();

  generate_drivable_area_ =
    declare_parameter<bool>("generate_drivable_area", generate_drivable_area_);
  left_bound_offset_ = declare_parameter<double>("left_bound_offset", lane_width / 2.0);
  right_bound_offset_ = declare_parameter<double>("right_bound_offset", lane_width / 2.0);

  map_snap_distance_threshold_ =
    declare_parameter<double>("map_snap_distance_threshold", map_snap_distance_threshold_);
  const bool use_lane_speed_limits = declare_parameter<bool>("use_lane_speed_limits", false);
  MapPathGenerator::Parameters map_params;
  map_params.snap_distance_threshold = map_snap_distance_threshold_;
  map_params.path_interval = path_interval_;
  map_params.default_velocity = default_velocity_;
  map_params.use_lane_speed_limits = use_lane_speed_limits;
  map_path_generator_ = std::make_unique<MapPathGenerator>(map_params);
  map_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS(1).reliable().transient_local(),
    [this](const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg) {
      auto map = std::make_shared<lanelet::LaneletMap>();
      lanelet::utils::conversion::fromBinMsg(*msg, map);
      std::lock_guard<std::mutex> lock(mutex_);
      map_path_generator_->setMap(map);
      RCLCPP_INFO(get_logger(), "Lanelet2 map loaded for path snapping");
    });

  vehicle_id_ = declare_parameter<std::string>("vehicle_id", "v001");

  ProtocolCallbacks callbacks;
  callbacks.on_route_window = [this](const igv_planning_msgs::msg::FMSRouteWindow::SharedPtr msg) {
    onRouteWindow(msg);
  };
  callbacks.on_stop_point = [this](const igv_planning_msgs::msg::FMSStopPoint::SharedPtr msg) {
    onStopPoint(msg);
  };

  const auto protocol_type = declare_parameter<std::string>("fms_protocol.type", "default");
  protocol_ = FMSProtocolFactory::instance().create(protocol_type);
  if (!protocol_) {
    throw std::runtime_error("Unknown FMS protocol type: " + protocol_type);
  }
  protocol_->initialize(*this, callbacks);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", rclcpp::SensorDataQoS(),
    std::bind(&FMSPathConverterNode::odometryCallback, this, std::placeholders::_1));
  if (crab_avoidance_enabled_) {
    objects_sub_ = create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
      "~/input/objects", rclcpp::QoS(1).reliable(),
      std::bind(&FMSPathConverterNode::onObjects, this, std::placeholders::_1));
  }

  path_pub_ = create_publisher<autoware_planning_msgs::msg::Path>("~/output/path", 10);
  status_pub_ = create_publisher<igv_planning_msgs::msg::FMSStatus>("~/output/status", 10);
  reverse_state_pub_ = create_publisher<reverse_vehicle_msg::msg::VehicleReversed>(
    "/planning/reverse_state", rclcpp::QoS(10).transient_local());

  rclcpp::QoS route_qos(1);
  route_qos.reliable();
  route_qos.transient_local();
  route_pub_ = create_publisher<autoware_planning_msgs::msg::LaneletRoute>(
    "~/output/route", route_qos);

  reverse_state_service_ = create_service<reverse_vehicle_msg::srv::SetVehicleReverseState>(
    "/set_reverse_vehicle_state_service",
    std::bind(
      &FMSPathConverterNode::handleReverseStateService, this,
      std::placeholders::_1, std::placeholders::_2));
  publishReverseState();

  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_)),
    std::bind(&FMSPathConverterNode::timerCallback, this));

  last_fms_message_time_ = now();
  RCLCPP_INFO(get_logger(), "FMS path planning node initialized, protocol=%s", protocol_->name().c_str());
}

void FMSPathConverterNode::onRouteWindow(const igv_planning_msgs::msg::FMSRouteWindow::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto normalized = std::make_shared<igv_planning_msgs::msg::FMSRouteWindow>(*msg);
  const auto inserted = planning::prependPreviousSegmentEndpoints(
    *normalized, route_segment_selector_.parameters.endpoint_distance);
  if (inserted > 0) {
    RCLCPP_WARN(
      get_logger(),
      "Inserted %zu missing segment boundary point(s), route_id=%s",
      inserted, normalized->route_id.c_str());
  }

  last_window_ = normalized;
  last_fms_message_time_ = now();
  fms_connected_ = true;
  if (!normalized->vehicle_id.empty()) {
    vehicle_id_ = normalized->vehicle_id;
  }

  updateSegmentPath();
}

void FMSPathConverterNode::holdSegmentPath(const std::string & reason)
{
  last_error_ = reason;
  // Replace the cache, so stop-point callbacks and the publish timer cannot
  // accidentally republish an old moving path while matching is unresolved.
  auto stop = last_output_path_ ? *last_output_path_ : autoware_planning_msgs::msg::Path{};
  stop.header.frame_id = frame_id_;
  if (stop.points.empty() && current_odom_) {
    autoware_planning_msgs::msg::PathPoint point;
    point.pose = current_odom_->pose.pose;
    stop.points.push_back(point);
    stop.points.push_back(point);
  }
  for (auto & point : stop.points) {
    point.longitudinal_velocity_mps = 0.0f;
    point.lateral_velocity_mps = 0.0f;
    point.heading_rate_rps = 0.0f;
  }
  last_reference_path_ = std::make_shared<autoware_planning_msgs::msg::Path>(stop);
  last_published_route_id_.clear();
  last_published_base_route_id_.clear();
  publishPathFromReference();
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", reason.c_str());
}

void FMSPathConverterNode::updateSegmentPath()
{
  if (!last_window_) return;
  const auto & window = *last_window_;
  if (window.status != igv_planning_msgs::msg::FMSRouteWindow::STATUS_ACTIVE) {
    if (window.status != igv_planning_msgs::msg::FMSRouteWindow::STATUS_PAUSED) {
      route_segment_selector_.reset();
    }
    holdSegmentPath("Route is not active");
    return;
  }
  if (!current_odom_ || !last_odometry_time_ ||
    (now() - *last_odometry_time_).seconds() > stale_threshold_) {
    route_segment_selector_.reset();
    holdSegmentPath("Waiting for fresh localization to select route segment");
    return;
  }
  RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 2000, "Selecting segment from window with "
    << window.segments.size() << " segments, route_id=" << window.route_id);
  const auto & velocity = current_odom_->twist.twist.linear;
  const auto base_route_id = utils::baseRouteId(window.route_id);
  const auto match = route_segment_selector_.select(
    window, base_route_id, current_odom_->pose.pose,
    std::hypot(velocity.x, velocity.y), last_odometry_time_->seconds());
  
  if (!match.valid) {
    holdSegmentPath(match.reason);
    return;
  }
  const bool fresh_objects =
    crab_avoidance_enabled_ && objects_ && last_objects_time_ &&
    (now() - *last_objects_time_).seconds() <= crab_avoidance_object_timeout_;
  const bool obstacle_state_changed = crab_avoidance_enabled_ &&
    (objects_revision_ != last_planned_objects_revision_ ||
    fresh_objects != last_plan_used_fresh_objects_);
  if (!match.changed && !obstacle_state_changed && last_error_.empty() &&
    base_route_id == last_published_base_route_id_ &&
    last_reference_path_ && computeLateralDeviationFromCachedPath() < replan_lateral_deviation_threshold_)
  {
    publishPathFromReference();
    return;
  }
  try {
    auto path = convertRouteWindowToAutowarePath(window);
    if (path.points.size() < 2) {
      holdSegmentPath("Selected segment produced fewer than two path points");
      return;
    }
    connectEgoToPath(path);
    // A destination and a forward/reverse transition are real stop
    // boundaries. A same-direction segment boundary or the end of a sliding
    // route window is only a planning-horizon edge and must remain moving.
    if (planning::requiresTerminalStop(window, match.index)) {
      applyTerminalStopProfile(path, 0.8);
    }
    last_generated_path_ = std::make_shared<autoware_planning_msgs::msg::Path>(path);
    last_reference_path_ = std::make_shared<autoware_planning_msgs::msg::Path>(path);
    last_published_route_id_ = window.route_id;
    last_published_base_route_id_ = base_route_id;
    last_planned_objects_revision_ = objects_revision_;
    last_plan_used_fresh_objects_ = fresh_objects;
    last_error_.clear();
    updateReverseState(window.segments[match.index].orientation ==
      igv_planning_msgs::msg::FMSPathSegment::ORIENT_REVERSE);
    publishPathFromReference();
  } catch (const std::exception & error) {
    holdSegmentPath(std::string("Segment path generation failed: ") + error.what());
  }
}

void FMSPathConverterNode::onStopPoint(const igv_planning_msgs::msg::FMSStopPoint::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_stop_ = msg->clear ? std::nullopt : std::optional<igv_planning_msgs::msg::FMSStopPoint>(*msg);
  publishPathFromReference();
}

void FMSPathConverterNode::onObjects(
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  objects_ = msg;
  last_objects_time_ = now();
  ++objects_revision_;
  if (fms_connected_ && last_window_) {
    updateSegmentPath();
  }
}

autoware_planning_msgs::msg::Path FMSPathConverterNode::convertRouteWindowToAutowarePath(
  const igv_planning_msgs::msg::FMSRouteWindow & window)
{
  autoware_planning_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = frame_id_;

  const size_t selected_index = route_segment_selector_.current(window);
  const size_t motion_end_index =
    planning::continuousMotionEndIndex(window, selected_index);
  const bool is_reverse = window.segments.at(selected_index).orientation ==
                          igv_planning_msgs::msg::FMSPathSegment::ORIENT_REVERSE;
  path_generator_->setReverseState(is_reverse);

  has_original_goal_ = false;
  std::vector<bool> avoidance_allowed;
  std::vector<std::pair<size_t, size_t>> protected_ranges;

  // Build one continuous path across every visible same-direction successor.
  // publishPathFromReference() later crops this combined path to the configured
  // forward horizon.
  for (size_t segment_index = selected_index; segment_index <= motion_end_index;
    ++segment_index)
  {
    const auto & segment = window.segments.at(segment_index);
    std::vector<autoware_planning_msgs::msg::PathPoint> sparse_points;
    sparse_points.reserve(segment.points.size());
    for (const auto & waypoint : segment.points) {
      sparse_points.push_back(makePathPoint(waypoint, is_reverse));
    }

    for (const auto & subsegment : planning::splitSubsegments(segment)) {
      const auto first = sparse_points.begin() + subsegment.first;
      const auto last = sparse_points.begin() + subsegment.last + 1;
      const std::vector<autoware_planning_msgs::msg::PathPoint> points(first, last);
      const bool allow_avoidance =
        subsegment.type == planning::SubsegmentType::Normal ||
        subsegment.type == planning::SubsegmentType::LaneChange;
      const auto generated = generateSubsegmentPath(points, subsegment.type, is_reverse);
      const size_t first_output_index = appendPathPoints(
        path.points, avoidance_allowed, generated, allow_avoidance);
      if ((subsegment.type == planning::SubsegmentType::LocalPlan ||
        subsegment.type == planning::SubsegmentType::PreviousFrame) &&
        first_output_index < path.points.size())
      {
        protected_ranges.emplace_back(first_output_index, path.points.size() - 1);
      }
    }
  }

  applyCrabAvoidance(path, avoidance_allowed, protected_ranges, is_reverse);

  if (!path.points.empty()) {
    path.points.back().is_final = planning::requiresTerminalStop(window, selected_index);
  }
  return path;
}

void FMSPathConverterNode::applyCrabAvoidance(
  autoware_planning_msgs::msg::Path & path,
  const std::vector<bool> & avoidance_allowed,
  const std::vector<std::pair<size_t, size_t>> & protected_ranges,
  const bool is_reverse)
{
  if (!crab_avoidance_enabled_ || !crab_avoidance_planner_ || !current_odom_ ||
    !objects_ || objects_->objects.empty() || !last_objects_time_ || is_reverse ||
    path.points.size() < 2 || avoidance_allowed.size() != path.points.size() ||
    (now() - *last_objects_time_).seconds() > crab_avoidance_object_timeout_)
  {
    return;
  }
  if (!objects_->header.frame_id.empty() && objects_->header.frame_id != path.header.frame_id) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignore crab avoidance objects in frame '%s'; expected '%s'",
      objects_->header.frame_id.c_str(), path.header.frame_id.c_str());
    return;
  }

  // 只取同向、航向近似不变的 NORMAL + LC 连续段；较缓的 NORMAL 曲线按曲率阈值放行。
  std::vector<std::pair<size_t, size_t>> runs;
  for (size_t begin = 0; begin < path.points.size();) {
    if (!avoidance_allowed[begin]) {
      ++begin;
      continue;
    }
    const double anchor_yaw = utils::getYaw(path.points[begin].pose.orientation);
    const double c = std::cos(anchor_yaw);
    const double s = std::sin(anchor_yaw);
    size_t end = begin + 1;
    for (; end < path.points.size() && avoidance_allowed[end]; ++end) {
      const auto & prev = path.points[end - 1];
      const auto & curr = path.points[end];
      const double dx = curr.pose.position.x - prev.pose.position.x;
      const double dy = curr.pose.position.y - prev.pose.position.y;
      const double distance = std::hypot(dx, dy);
      if (distance < 1e-4 || dx * c + dy * s <= 1e-4) break;
      const double yaw_prev = utils::getYaw(prev.pose.orientation);
      const double yaw_curr = utils::getYaw(curr.pose.orientation);
      const double curvature = std::abs(normalizeYaw(yaw_curr - yaw_prev)) / distance;
      if (curvature > crab_max_heading_curvature_ ||
        std::abs(normalizeYaw(yaw_curr - anchor_yaw)) > crab_max_heading_deviation_)
      {
        break;
      }
    }
    if (end - begin >= 2) runs.emplace_back(begin, end - 1);
    begin = end;
  }
  if (runs.empty()) return;

  const size_t ego_index = utils::findNearestIndex(path, current_odom_->pose.pose.position);
  std::optional<std::pair<size_t, size_t>> selected;
  bool use_segment_start_as_ego = false;
  for (const auto & run : runs) {
    if (run.first <= ego_index && ego_index <= run.second) {
      selected = run;
      break;
    }
  }
  if (!selected) {
    for (const auto & qturn : protected_ranges) {
      if (qturn.first <= ego_index && ego_index <= qturn.second) {
        const auto next = std::find_if(
          runs.begin(), runs.end(),
          [&qturn](const auto & run) { return run.first == qturn.second + 1; });
        if (next != runs.end()) {
          selected = *next;
          use_segment_start_as_ego = true;
        }
        break;
      }
    }
  }
  if (!selected) return;

  auto reference = std::make_shared<autoware_planning_msgs::msg::Path>();
  reference->header = path.header;
  reference->points.assign(
    path.points.begin() + selected->first, path.points.begin() + selected->second + 1);
  const auto & first = reference->points.front();
  const auto & last = reference->points.back();
  auto params = std::make_shared<cb_planning::Planning_Para>(crab_planning_params_);
  params->lane_x_bias = first.pose.position.x;
  params->lane_y_bias = first.pose.position.y;
  params->lane_yaw_bias = utils::getYaw(first.pose.orientation);
  const double c = std::cos(params->lane_yaw_bias);
  const double s = std::sin(params->lane_yaw_bias);
  params->lane_end_x =
    (last.pose.position.x - params->lane_x_bias) * c +
    (last.pose.position.y - params->lane_y_bias) * s;
  if (params->lane_end_x < params->Min_Collision_free_distance + 1.0) return;

  // 不在本段前方/侧向搜索范围内的物体，不触发一次无意义的斜行重规划。
  const bool relevant_object = std::any_of(
    objects_->objects.begin(), objects_->objects.end(), [&](const auto & object) {
      const auto & position = object.kinematics.initial_pose_with_covariance.pose.position;
      const double dx = position.x - params->lane_x_bias;
      const double dy = position.y - params->lane_y_bias;
      const double longitudinal = dx * c + dy * s;
      const double lateral = dy * c - dx * s;
      const double possible_motion = params->enable_dynamic_obstacle_avoidance ?
        6.0 * std::hypot(
          object.kinematics.initial_twist_with_covariance.twist.linear.x,
          object.kinematics.initial_twist_with_covariance.twist.linear.y) : 0.0;
      return longitudinal >= -params->ego_length - possible_motion &&
        longitudinal <= params->lane_end_x + params->ego_length + possible_motion &&
        std::abs(lateral) <=
        std::max(std::abs(params->lane_sampling_begin_y),
          std::abs(params->lane_sampling_end_y)) +
        params->ego_width + object.shape.dimensions.y +
        params->obstacle_clearance + possible_motion;
    });
  if (!relevant_object) return;

  auto planning_odom = std::make_shared<nav_msgs::msg::Odometry>(*current_odom_);
  if (use_segment_start_as_ego) {
    planning_odom->pose.pose = first.pose;
    planning_odom->twist.twist.linear.x =
      std::max(0.1, static_cast<double>(std::abs(first.longitudinal_velocity_mps)));
    planning_odom->twist.twist.linear.y = 0.0;
  }
  crab_avoidance_planner_->setParameter(params);
  crab_avoidance_planner_->setReferencePath(reference);
  crab_avoidance_planner_->setOdometry(planning_odom);
  crab_avoidance_planner_->setObjectsInfo(*objects_);
  if (!crab_avoidance_planner_->run()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Crab avoidance could not generate a collision-free returnable path; keeping reference");
    return;
  }
  const auto & output = crab_avoidance_planner_->outputPath();
  if (output.points.size() != reference->points.size() ||
    std::any_of(output.points.begin(), output.points.end(), [](const auto & point) {
      return !std::isfinite(point.pose.position.x) ||
             !std::isfinite(point.pose.position.y);
    }))
  {
    RCLCPP_ERROR(get_logger(), "Crab avoidance output is not aligned with its reference path");
    return;
  }
  std::copy(output.points.begin(), output.points.end(), path.points.begin() + selected->first);
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "Crab avoidance replaced %zu points in segment [%zu, %zu]",
    output.points.size(), selected->first, selected->second);
}

autoware_planning_msgs::msg::PathPoint FMSPathConverterNode::makePathPoint(
  const igv_planning_msgs::msg::SegmentWaypoint & waypoint, const bool is_reverse)
{
  autoware_planning_msgs::msg::PathPoint point;
  point.pose.position.x = waypoint.x;
  point.pose.position.y = waypoint.y;
  point.pose.position.z = waypoint.z;
  point.pose.orientation = utils::createQuaternionFromYaw(
    normalizeYaw(waypoint.heading + (is_reverse ? M_PI : 0.0)));

  const double speed = std::clamp(
    waypoint.speed_limit > 0.1 ? waypoint.speed_limit : default_velocity_,
    min_velocity_, max_velocity_);
  point.longitudinal_velocity_mps = static_cast<float>(is_reverse ? -speed : speed);
  point.lateral_velocity_mps = 0.0f;
  point.heading_rate_rps = 0.0f;
  point.is_final = false;

  if (waypoint.point_type == igv_planning_msgs::msg::SegmentWaypoint::POINT_DESTINATION) {
    point.longitudinal_velocity_mps = 0.0f;
    original_goal_position_ = point.pose.position;
    has_original_goal_ = true;
  }
  return point;
}

std::vector<autoware_planning_msgs::msg::PathPoint> FMSPathConverterNode::generateSubsegmentPath(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const planning::SubsegmentType type, const bool is_reverse)
{
  if (type == planning::SubsegmentType::Pending) {
    return {};
  }

  if (type == planning::SubsegmentType::PreviousFrame) {
    return extractPreviousFrameSubpath(points);
  }

  if (points.size() < 2) {
    return points;
  }

  if (type == planning::SubsegmentType::Normal &&
    map_path_generator_ && map_path_generator_->hasMap())
  {
    std::vector<uint8_t> lane_change_types(points.size(), 0);
    auto snapped = map_path_generator_->generateMapSnappedPath(
      points, lane_change_types, is_reverse, nullptr);
    if (!snapped.points.empty()) {
      return snapped.points;
    }
  }

  if (type == planning::SubsegmentType::LaneChange) {
    if (map_path_generator_ && map_path_generator_->hasMap()) {
      std::vector<uint8_t> lane_change_types(points.size(), 1);
      auto snapped = map_path_generator_->generateMapSnappedPath(
        points, lane_change_types, is_reverse, path_generator_.get());
      if (!snapped.points.empty()) {
        return snapped.points;
      }
    }
    auto lane_change = path_generator_->generateObliqueLaneChangeSegment(points.front(), points.back());
    if (!lane_change.empty()) {
      return lane_change;
    }
  }

  if (type == planning::SubsegmentType::LocalPlan) {
    auto qturn = path_generator_->generateQTurnSegment(points.front(), points.back());
    if (qturn.empty() || path_generator_->lastQTurnFellBack()) {
      throw std::runtime_error("LOCAL_PLAN subsegment Q-turn generation failed");
    }
    return qturn;
  }

  autoware_planning_msgs::msg::Path fallback;
  fallback.points = points;
  interpolatePath(fallback);
  return fallback.points;
}

std::vector<autoware_planning_msgs::msg::PathPoint>
FMSPathConverterNode::extractPreviousFrameSubpath(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & visible_points) const
{
  const auto source_path = last_generated_path_ ? last_generated_path_ : last_reference_path_;
  if (!source_path || source_path->points.empty()) {
    throw std::runtime_error("Cropped special subsegment requires previous generated path");
  }
  if (visible_points.empty()) {
    return {};
  }

  const auto & previous = source_path->points;
  const auto & start_anchor = current_odom_
    ? current_odom_->pose.pose.position
    : visible_points.front().pose.position;
  const size_t start_idx = utils::findNearestIndex(*source_path, start_anchor);
  const size_t end_idx =
    utils::findNearestIndex(*source_path, visible_points.back().pose.position);
  if (end_idx < start_idx) {
    return {};
  }

  const double max_match_distance = route_segment_selector_.parameters.max_distance;
  const double end_error = utils::distance2D(
    previous[end_idx].pose.position, visible_points.back().pose.position);
  if (end_error > max_match_distance) {
    throw std::runtime_error("Previous generated path is too far from cropped special subsegment");
  }

  return {previous.begin() + start_idx, previous.begin() + end_idx + 1};
}

bool FMSPathConverterNode::publishPathFromReference()
{
  if (!last_reference_path_ || last_reference_path_->points.empty()) {
    return false;
  }

  auto path = *last_reference_path_;
  path.header.stamp = now();

  if (current_odom_) {
    truncatePathFromEgo(path);
    // Direction is owned by the confirmed segment, including while stopped.
  }
  applyStopPoint(path);
  generateDrivableBounds(path);

  last_output_path_ = std::make_shared<autoware_planning_msgs::msg::Path>(path);
  path_pub_->publish(path);
  return true;
}

void FMSPathConverterNode::connectEgoToPath(autoware_planning_msgs::msg::Path & path)
{
  if (!current_odom_ || path.points.empty()) {
    return;
  }

  const auto & ego = current_odom_->pose.pose.position;
  const auto & first = path.points.front().pose.position;
  if (utils::distance2D(ego, first) <= 2.0) {
    return;
  }

  if (path.points.size() >= 2) {
    const auto & second = path.points[1].pose.position;
    const double connector_yaw = std::atan2(first.y - ego.y, first.x - ego.x);
    const double path_yaw = std::atan2(second.y - first.y, second.x - first.x);
    const double yaw_error = std::abs(normalizeYaw(connector_yaw - path_yaw));
    constexpr double max_connector_yaw_error = M_PI / 4.0;
    if (yaw_error > max_connector_yaw_error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Skip ego connector: connector yaw differs from path tangent by %.1f deg",
        yaw_error * 180.0 / M_PI);
      return;
    }
  }

  auto ego_point = path.points.front();
  ego_point.pose.position = ego;
  double yaw = std::atan2(first.y - ego.y, first.x - ego.x);
  if (path.points.front().longitudinal_velocity_mps < 0.0f) {
    yaw = normalizeYaw(yaw + M_PI);
  }
  ego_point.pose.orientation = utils::createQuaternionFromYaw(yaw);
  if (std::abs(ego_point.longitudinal_velocity_mps) < 0.1f) {
    ego_point.longitudinal_velocity_mps = static_cast<float>(default_velocity_);
  }
  path.points.insert(path.points.begin(), ego_point);
}

void FMSPathConverterNode::truncatePathFromEgo(autoware_planning_msgs::msg::Path & path)
{
  utils::truncatePathFromEgoPose(
    path, current_odom_->pose.pose, ego_nearest_dist_threshold_, ego_nearest_yaw_threshold_,
    backward_path_length_);
  cropPathForwardLength(path, forward_path_length_);
}

void FMSPathConverterNode::interpolatePath(autoware_planning_msgs::msg::Path & path)
{
  if (path.points.size() < 2) {
    return;
  }

  std::vector<autoware_planning_msgs::msg::PathPoint> interpolated;
  for (size_t i = 0; i + 1 < path.points.size(); ++i) {
    const auto & start = path.points[i];
    const auto & end = path.points[i + 1];
    interpolated.push_back(start);

    const double distance = utils::distance2D(start.pose.position, end.pose.position);
    const int count = static_cast<int>(distance / path_interval_);
    for (int j = 1; j < count; ++j) {
      interpolated.push_back(
        utils::interpolatePathPoint(start, end, static_cast<double>(j) / count));
    }
  }
  interpolated.push_back(path.points.back());
  path.points = std::move(interpolated);
}

void FMSPathConverterNode::applyStopPoint(autoware_planning_msgs::msg::Path & path)
{
  if (!active_stop_ || path.points.empty()) {
    return;
  }

  geometry_msgs::msg::Point stop;
  stop.x = active_stop_->x;
  stop.y = active_stop_->y;
  const size_t stop_idx = utils::findNearestIndex(path, stop);

  for (size_t i = stop_idx; i < path.points.size(); ++i) {
    path.points[i].longitudinal_velocity_mps = 0.0f;
    path.points[i].lateral_velocity_mps = 0.0f;
  }
}

void FMSPathConverterNode::generateDrivableBounds(autoware_planning_msgs::msg::Path & path)
{
  if (!generate_drivable_area_ || path.points.empty()) {
    return;
  }
  if (path.left_bound.size() == path.points.size() &&
    path.right_bound.size() == path.points.size())
  {
    return;
  }

  path.left_bound.clear();
  path.right_bound.clear();
  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto & point = path.points[i];
    double yaw = utils::getYaw(point.pose.orientation);
    if (i + 1 < path.points.size()) {
      const auto & next = path.points[i + 1].pose.position;
      yaw = std::atan2(next.y - point.pose.position.y, next.x - point.pose.position.x);
    } else if (i > 0) {
      const auto & prev = path.points[i - 1].pose.position;
      yaw = std::atan2(point.pose.position.y - prev.y, point.pose.position.x - prev.x);
    }

    geometry_msgs::msg::Point left = point.pose.position;
    geometry_msgs::msg::Point right = point.pose.position;
    left.x += left_bound_offset_ * std::cos(yaw + M_PI_2);
    left.y += left_bound_offset_ * std::sin(yaw + M_PI_2);
    right.x += right_bound_offset_ * std::cos(yaw - M_PI_2);
    right.y += right_bound_offset_ * std::sin(yaw - M_PI_2);
    path.left_bound.push_back(left);
    path.right_bound.push_back(right);
  }
}

void FMSPathConverterNode::publishStatus()
{
  igv_planning_msgs::msg::FMSStatus status;
  status.header.stamp = now();
  status.header.frame_id = frame_id_;
  status.fms_connected = fms_connected_;
  status.last_message_time = (now() - last_fms_message_time_).seconds();
  status.current_mode = igv_planning_msgs::msg::FMSStatus::MODE_FMS_WITH_SAFETY;
  status.trajectory_valid = last_output_path_ && !last_output_path_->points.empty();
  status.trajectory_stale = !fms_connected_;
  status.last_error = last_error_;

  if (last_window_) {
    status.current_trajectory_id = last_window_->route_id;
    status.current_task_id = last_window_->job_task_id;
    for (const auto & segment : last_window_->segments) {
      status.waypoints_remaining += segment.points.size();
    }
  }
  if (last_output_path_ && !last_output_path_->points.empty() && current_odom_) {
    status.distance_to_goal = utils::distance2D(
      current_odom_->pose.pose.position, last_output_path_->points.back().pose.position);
  }
  status_pub_->publish(status);
}

void FMSPathConverterNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_odom_ = msg;
  last_odometry_time_ = now();
}

void FMSPathConverterNode::timerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if ((now() - last_fms_message_time_).seconds() > stale_threshold_) {
    fms_connected_ = false;
  }
  if (fms_connected_) {
    updateSegmentPath();
  } else if (last_reference_path_) {
    route_segment_selector_.reset();
    holdSegmentPath("Route window timed out");
  }
  publishStatus();
}

double FMSPathConverterNode::computeLateralDeviationFromCachedPath() const
{
  if (!last_reference_path_ || last_reference_path_->points.size() < 2 || !current_odom_) {
    return std::numeric_limits<double>::infinity();
  }
  return std::abs(utils::calcLateralOffset(
    last_reference_path_->points, current_odom_->pose.pose.position));
}

unique_identifier_msgs::msg::UUID FMSPathConverterNode::generateUUID()
{
  unique_identifier_msgs::msg::UUID uuid;
  const uint64_t stamp = static_cast<uint64_t>(now().nanoseconds());
  std::memcpy(uuid.uuid.data(), &stamp, sizeof(stamp));
  for (size_t i = sizeof(stamp); i < uuid.uuid.size(); ++i) {
    uuid.uuid[i] = static_cast<uint8_t>(i + stamp % 256);
  }
  return uuid;
}

void FMSPathConverterNode::updateReverseStateFromPath(
  const autoware_planning_msgs::msg::Path & path)
{
  if (!current_odom_ || path.points.empty()) {
    return;
  }

  const auto nearest_idx = utils::findNearestIndex(path, current_odom_->pose.pose.position);
  for (size_t i = nearest_idx; i < path.points.size(); ++i) {
    const float velocity = path.points[i].longitudinal_velocity_mps;
    if (velocity < -0.01f) {
      updateReverseState(true);
      return;
    }
    if (velocity > 0.01f) {
      updateReverseState(false);
      return;
    }
  }
}

void FMSPathConverterNode::updateReverseState(const bool is_reverse, const bool force_publish)
{
  current_reverse_state_ = is_reverse;
  if (path_generator_) {
    path_generator_->setReverseState(is_reverse);
  }
  if (force_publish || current_reverse_state_ != last_reverse_state_) {
    publishReverseState();
    last_reverse_state_ = current_reverse_state_;
  }
}

void FMSPathConverterNode::publishReverseState()
{
  reverse_vehicle_msg::msg::VehicleReversed msg;
  msg.is_reversed = current_reverse_state_;
  reverse_state_pub_->publish(msg);
}

void FMSPathConverterNode::handleReverseStateService(
  const std::shared_ptr<reverse_vehicle_msg::srv::SetVehicleReverseState::Request> request,
  std::shared_ptr<reverse_vehicle_msg::srv::SetVehicleReverseState::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  updateReverseState(request->is_reversed, true);
  response->success = true;
}

}  // namespace igv_path_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(igv_path_planner::FMSPathConverterNode)
