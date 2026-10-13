// Copyright 2021 Tier IV, Inc.
// Copyright 2026 ZPMC (slim port for igv_velocity_planner — Analytical only,
//                      operation-mode coupling + debug publishers dropped)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "igv_velocity_planner/fms_kinematic_smoother_node.hpp"

#include "autoware/motion_utils/marker/marker_helper.hpp"
#include "igv_motion_utils/trajectory_resample.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace igv_velocity_planner
{

namespace ks = kinematic_smoother;

FmsKinematicSmootherNode::FmsKinematicSmootherNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("fms_kinematic_smoother", node_options)
{
  using std::placeholders::_1;

  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  wheelbase_ = vehicle_info.wheel_base_m;
  base_link2front_ = vehicle_info.max_longitudinal_offset_m;
  initCommonParam();

  // Construct the time-keeper *before* the smoother so the algorithm libs
  // can take a non-null shared_ptr. The detail-publisher is intentionally
  // a no-op publisher here (we keep the API but don't expose the topic).
  time_keeper_ = std::make_shared<autoware::universe_utils::TimeKeeper>();

  // Single algorithm: Analytical (no QP/OSQP dependency).
  smoother_ = std::make_shared<ks::AnalyticalJerkConstrainedSmoother>(*this, time_keeper_);
  smoother_->setWheelBase(wheelbase_);

  // I/O
  pub_trajectory_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_processing_time_ = create_publisher<Float64Stamped>("~/debug/processing_time_ms", 1);
  sub_current_trajectory_ = create_subscription<Trajectory>(
    "~/input/trajectory", 1, std::bind(&FmsKinematicSmootherNode::onCurrentTrajectory, this, _1));

  // FMS-specific: maneuver_type subscription. Clearing the path cache on
  // every maneuver-type change guarantees we never re-inject orientation
  // from a stale-maneuver Path.
  sub_maneuver_type_ = create_subscription<std_msgs::msg::UInt8>(
    "~/input/maneuver_type", rclcpp::QoS(10).transient_local(),
    [this](const std_msgs::msg::UInt8::SharedPtr msg) {
      if (current_maneuver_type_ != msg->data) {
        behavior_path_msg_.reset();
      }
      current_maneuver_type_ = msg->data;
    });

  // FMS-specific: behavior_path subscription for OBLIQUE/Q_TURN body-yaw
  // re-injection after our internal resample overwrites pose.orientation.
  // VOLATILE QoS to match the publisher's wire QoS (see comment in the
  // upstream velocity_smoother — TRANSIENT_LOCAL subscriber would be
  // rejected by DDS QoS negotiation).
  sub_behavior_path_ = create_subscription<autoware_planning_msgs::msg::Path>(
    "~/input/behavior_path", rclcpp::QoS(1),
    [this](const autoware_planning_msgs::msg::Path::ConstSharedPtr msg) {
      behavior_path_msg_ = msg;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "[FmsKinSmoother] behavior_path cached: N=%zu",
        msg->points.size());
    });

  external_velocity_limit_.velocity = node_param_.max_velocity;
  max_velocity_with_deceleration_ = node_param_.max_velocity;
  clock_ = get_clock();
}

void FmsKinematicSmootherNode::initCommonParam()
{
  auto & p = node_param_;
  p.enable_lateral_acc_limit = declare_parameter<bool>("enable_lateral_acc_limit");
  p.enable_steering_rate_limit = declare_parameter<bool>("enable_steering_rate_limit");

  p.max_velocity = declare_parameter<double>("max_vel");
  p.margin_to_insert_external_velocity_limit =
    declare_parameter<double>("margin_to_insert_external_velocity_limit");
  p.replan_vel_deviation = declare_parameter<double>("replan_vel_deviation");
  p.engage_velocity = declare_parameter<double>("engage_velocity");
  p.engage_acceleration = declare_parameter<double>("engage_acceleration");
  p.engage_exit_ratio = declare_parameter<double>("engage_exit_ratio");
  p.engage_exit_ratio = std::min(std::max(p.engage_exit_ratio, 0.0), 1.0);
  p.stopping_velocity = declare_parameter<double>("stopping_velocity");
  p.stopping_distance = declare_parameter<double>("stopping_distance");
  p.extract_ahead_dist = declare_parameter<double>("extract_ahead_dist");
  p.extract_behind_dist = declare_parameter<double>("extract_behind_dist");
  p.stop_dist_to_prohibit_engage = declare_parameter<double>("stop_dist_to_prohibit_engage");
  p.ego_nearest_dist_threshold = declare_parameter<double>("ego_nearest_dist_threshold");
  p.ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");
  p.post_resample_param.max_trajectory_length =
    declare_parameter<double>("post_max_trajectory_length");
  p.post_resample_param.min_trajectory_length =
    declare_parameter<double>("post_min_trajectory_length");
  p.post_resample_param.resample_time = declare_parameter<double>("post_resample_time");
  p.post_resample_param.dense_resample_dt = declare_parameter<double>("post_dense_resample_dt");
  p.post_resample_param.dense_min_interval_distance =
    declare_parameter<double>("post_dense_min_interval_distance");
  p.post_resample_param.sparse_resample_dt = declare_parameter<double>("post_sparse_resample_dt");
  p.post_resample_param.sparse_min_interval_distance =
    declare_parameter<double>("post_sparse_min_interval_distance");
}

bool FmsKinematicSmootherNode::checkData() const
{
  if (!current_odometry_ptr_ || !base_traj_raw_ptr_ || !current_acceleration_ptr_) {
    RCLCPP_DEBUG(
      get_logger(), "wait topics : current_vel = %d, base_traj = %d, acceleration = %d",
      (bool)current_odometry_ptr_, (bool)base_traj_raw_ptr_, (bool)current_acceleration_ptr_);
    return false;
  }
  if (base_traj_raw_ptr_->points.size() < 2) {
    RCLCPP_ERROR(get_logger(), "input trajectory size must > 1. Skip computation.");
    return false;
  }
  return true;
}

void FmsKinematicSmootherNode::onCurrentTrajectory(const Trajectory::ConstSharedPtr msg)
{
  stop_watch_.tic();
  base_traj_raw_ptr_ = msg;

  current_odometry_ptr_ = sub_current_odometry_.takeData();
  current_acceleration_ptr_ = sub_current_acceleration_.takeData();
  external_velocity_limit_ptr_ = sub_external_velocity_limit_.takeData();

  if (!checkData()) {
    return;
  }

  auto input_points = autoware::motion_utils::convertToTrajectoryPointArray(*base_traj_raw_ptr_);
  input_points = autoware::motion_utils::removeOverlapPoints(input_points);
  if (input_points.size() < 2) {
    RCLCPP_ERROR(get_logger(), "No enough points in trajectory after overlap points removal");
    return;
  }
  // Preserve the endpoint velocity decided by the upstream velocity shaper.
  // A zero endpoint is a real stop boundary; a non-zero endpoint is only the
  // edge of the rolling planning horizon and must not become a synthetic stop.

  if (!prev_output_.empty()) {
    current_closest_point_from_prev_output_ = calcProjectedTrajectoryPointFromEgo(prev_output_);
  }

  calcExternalVelocityLimit();
  updateDataForExternalVelocityLimit();

  is_reverse_ = isReverse(input_points);
  if (is_reverse_) {
    flipVelocity(input_points);
  }

  const auto output = calcTrajectoryVelocity(input_points);
  if (output.empty()) {
    RCLCPP_WARN(get_logger(), "Output Point is empty");
    return;
  }

  auto output_resampled = ks::resampling::resampleTrajectory(
    output, current_odometry_ptr_->twist.twist.linear.x, current_odometry_ptr_->pose.pose,
    node_param_.ego_nearest_dist_threshold, node_param_.ego_nearest_yaw_threshold,
    node_param_.post_resample_param, false);

  updatePrevValues(output);

  if (is_reverse_) {
    flipVelocity(output_resampled);
  }

  // OBLIQUE / Q_TURN orientation re-injection from cached behavior_path.
  // See upstream velocity_smoother node.cpp ~536-630 for the rationale.
  constexpr uint8_t MANEUVER_OBLIQUE = 2;
  constexpr uint8_t MANEUVER_Q_TURN = 6;
  if (
    current_maneuver_type_ == MANEUVER_OBLIQUE || current_maneuver_type_ == MANEUVER_Q_TURN) {
    for (auto & pt : output_resampled) {
      pt.lateral_velocity_mps = 0.0f;
    }
    if (behavior_path_msg_ && behavior_path_msg_->points.size() >= 2) {
      igv_motion_utils::overwriteOrientationFromPathProjection(
        output_resampled, *behavior_path_msg_);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[FmsKinSmoother] OBLIQUE/Q_TURN: no behavior_path cached; orientation will follow "
        "path tangent until next behavior_path arrives.");
    }
  }

  publishTrajectory(output_resampled);

  Float64Stamped pt{};
  pt.stamp = this->now();
  pt.data = stop_watch_.toc();
  pub_processing_time_->publish(pt);
}

void FmsKinematicSmootherNode::publishTrajectory(const TrajectoryPoints & trajectory) const
{
  Trajectory publishing_trajectory = autoware::motion_utils::convertToTrajectory(trajectory);
  publishing_trajectory.header = base_traj_raw_ptr_->header;
  pub_trajectory_->publish(publishing_trajectory);
}

void FmsKinematicSmootherNode::calcExternalVelocityLimit()
{
  if (!external_velocity_limit_ptr_) {
    return;
  }

  external_velocity_limit_.sender = external_velocity_limit_ptr_->sender;

  if (prev_output_.empty() || !current_closest_point_from_prev_output_) {
    external_velocity_limit_.velocity = external_velocity_limit_ptr_->max_velocity;
    return;
  }

  constexpr double eps = 1.0E-04;
  const double margin = node_param_.margin_to_insert_external_velocity_limit;

  if (
    std::fabs(current_odometry_ptr_->twist.twist.linear.x) < eps &&
    external_velocity_limit_.velocity < eps) {
    external_velocity_limit_.dist = 0.0;
  }

  if (
    std::fabs((external_velocity_limit_.velocity - external_velocity_limit_ptr_->max_velocity)) >
    eps) {
    const double v0 = current_closest_point_from_prev_output_->longitudinal_velocity_mps;
    const double a0 = current_closest_point_from_prev_output_->acceleration_mps2;

    if (isEngageStatus(v0)) {
      max_velocity_with_deceleration_ = external_velocity_limit_ptr_->max_velocity;
      external_velocity_limit_.dist = 0.0;
    } else {
      const auto & cstr = external_velocity_limit_ptr_->constraints;
      const auto a_min = external_velocity_limit_ptr_->use_constraints ? cstr.min_acceleration
                                                                       : smoother_->getMinDecel();
      const auto j_max =
        external_velocity_limit_ptr_->use_constraints ? cstr.max_jerk : smoother_->getMaxJerk();
      const auto j_min =
        external_velocity_limit_ptr_->use_constraints ? cstr.min_jerk : smoother_->getMinJerk();

      if (a0 > 0) {
        max_velocity_with_deceleration_ = v0 - 0.5 * a0 * a0 / j_min;
      } else {
        max_velocity_with_deceleration_ = v0;
      }

      if (external_velocity_limit_ptr_->max_velocity < max_velocity_with_deceleration_) {
        if (v0 < external_velocity_limit_ptr_->max_velocity) {
          RCLCPP_WARN(
            get_logger(),
            "Stronger jerk than expected may be applied to external velocity limit.");
        }
        double stop_dist = 0.0;
        std::map<double, double> jerk_profile;
        if (!ks::trajectory_utils::calcStopDistWithJerkConstraints(
              v0, a0, j_max, j_min, a_min, external_velocity_limit_ptr_->max_velocity, jerk_profile,
              stop_dist)) {
          RCLCPP_WARN(get_logger(), "Stop distance calculation failed!");
        }
        external_velocity_limit_.dist = stop_dist + margin;
      } else {
        max_velocity_with_deceleration_ = external_velocity_limit_ptr_->max_velocity;
        external_velocity_limit_.dist = 0.0;
      }
    }
  }

  external_velocity_limit_.velocity = external_velocity_limit_ptr_->max_velocity;
}

void FmsKinematicSmootherNode::updateDataForExternalVelocityLimit()
{
  if (prev_output_.empty()) {
    return;
  }
  const double travel_dist = calcTravelDistance();
  external_velocity_limit_.dist = std::max(external_velocity_limit_.dist - travel_dist, 0.0);
}

TrajectoryPoints FmsKinematicSmootherNode::calcTrajectoryVelocity(
  const TrajectoryPoints & traj_input) const
{
  const size_t input_closest = findNearestIndexFromEgo(traj_input);

  auto traj_extracted = ks::trajectory_utils::extractPathAroundIndex(
    traj_input, input_closest, node_param_.extract_ahead_dist, node_param_.extract_behind_dist);
  if (traj_extracted.empty()) {
    RCLCPP_WARN(get_logger(), "Fail to extract the path from the input trajectory");
    return prev_output_;
  }

  applyExternalVelocityLimit(traj_extracted);

  const size_t traj_extracted_closest = findNearestIndexFromEgo(traj_extracted);

  applyStopApproachingVelocity(traj_extracted);

  TrajectoryPoints output{};
  if (!smoothVelocity(traj_extracted, traj_extracted_closest, output)) {
    return prev_output_;
  }

  return output;
}

bool FmsKinematicSmootherNode::smoothVelocity(
  const TrajectoryPoints & input, const size_t input_closest,
  TrajectoryPoints & traj_smoothed) const
{
  if (input.empty()) {
    return false;
  }

  const auto [initial_motion, type] = calcInitialMotion(input, input_closest);

  constexpr bool enable_smooth_limit = true;
  constexpr bool use_resampling = true;
  const auto traj_lateral_acc_filtered =
    node_param_.enable_lateral_acc_limit
      ? smoother_->applyLateralAccelerationFilter(
          input, initial_motion.vel, initial_motion.acc, enable_smooth_limit, use_resampling)
      : input;

  const auto traj_steering_rate_limited =
    node_param_.enable_steering_rate_limit
      ? smoother_->applySteeringRateLimit(traj_lateral_acc_filtered, false)
      : traj_lateral_acc_filtered;

  auto traj_resampled = smoother_->resampleTrajectory(
    traj_steering_rate_limited, current_odometry_ptr_->twist.twist.linear.x,
    current_odometry_ptr_->pose.pose, node_param_.ego_nearest_dist_threshold,
    node_param_.ego_nearest_yaw_threshold);

  const size_t traj_resampled_closest = findNearestIndexFromEgo(traj_resampled);

  TrajectoryPoints clipped;
  clipped.insert(
    clipped.end(), traj_resampled.begin() + traj_resampled_closest, traj_resampled.end());

  std::vector<TrajectoryPoints> debug_trajectories;
  if (!smoother_->apply(
        initial_motion.vel, initial_motion.acc, clipped, traj_smoothed, debug_trajectories,
        false /* publish_debug_trajs */)) {
    RCLCPP_WARN(get_logger(), "Fail to solve optimization.");
  }

  overwriteStopPoint(clipped, traj_smoothed);

  traj_smoothed.insert(
    traj_smoothed.begin(), traj_resampled.begin(), traj_resampled.begin() + traj_resampled_closest);

  ks::trajectory_utils::applyMaximumVelocityLimit(
    traj_resampled_closest, traj_smoothed.size(), node_param_.max_velocity, traj_smoothed);

  insertBehindVelocity(traj_resampled_closest, type, traj_smoothed);
  return true;
}

void FmsKinematicSmootherNode::insertBehindVelocity(
  const size_t output_closest, const InitializeType type, TrajectoryPoints & output) const
{
  const bool keep_closest_vel_for_behind =
    (type == InitializeType::EGO_VELOCITY || type == InitializeType::LARGE_DEVIATION_REPLAN ||
     type == InitializeType::ENGAGING);

  for (size_t i = output_closest - 1; i < output.size(); --i) {
    if (keep_closest_vel_for_behind) {
      output.at(i).longitudinal_velocity_mps = output.at(output_closest).longitudinal_velocity_mps;
      output.at(i).acceleration_mps2 = output.at(output_closest).acceleration_mps2;
    } else {
      const size_t seg_idx = [&]() {
        const auto opt_nearest_seg_idx = autoware::motion_utils::findNearestSegmentIndex(
          prev_output_, output.at(i).pose, node_param_.ego_nearest_dist_threshold,
          node_param_.ego_nearest_yaw_threshold);
        if (opt_nearest_seg_idx) return opt_nearest_seg_idx.value();
        const auto opt_second_nearest_seg_idx = autoware::motion_utils::findNearestSegmentIndex(
          prev_output_, output.at(i).pose, node_param_.ego_nearest_dist_threshold);
        if (opt_second_nearest_seg_idx) return opt_second_nearest_seg_idx.value();
        return autoware::motion_utils::findNearestSegmentIndex(
          prev_output_, output.at(i).pose.position);
      }();
      const auto prev_output_point = ks::trajectory_utils::calcInterpolatedTrajectoryPoint(
        prev_output_, output.at(i).pose, seg_idx);
      output.at(i).longitudinal_velocity_mps = std::abs(prev_output_point.longitudinal_velocity_mps);
      output.at(i).acceleration_mps2 = prev_output_point.acceleration_mps2;
    }
  }
}

std::pair<Motion, FmsKinematicSmootherNode::InitializeType>
FmsKinematicSmootherNode::calcInitialMotion(
  const TrajectoryPoints & input_traj, const size_t input_closest) const
{
  const double vehicle_speed = std::fabs(current_odometry_ptr_->twist.twist.linear.x);
  const double target_vel = std::fabs(input_traj.at(input_closest).longitudinal_velocity_mps);

  if (!current_closest_point_from_prev_output_) {
    return {Motion{vehicle_speed, 0.0}, InitializeType::EGO_VELOCITY};
  }

  const double desired_vel = current_closest_point_from_prev_output_->longitudinal_velocity_mps;
  const double desired_acc = current_closest_point_from_prev_output_->acceleration_mps2;
  const double vel_error = vehicle_speed - std::fabs(desired_vel);

  if (std::fabs(vel_error) > node_param_.replan_vel_deviation) {
    RCLCPP_DEBUG(
      get_logger(), "Large deviation. desired_vel=%f vehicle_speed=%f vel_error=%f", desired_vel,
      vehicle_speed, vel_error);
    return {Motion{vehicle_speed, desired_acc}, InitializeType::LARGE_DEVIATION_REPLAN};
  }

  const double engage_vel_thr = node_param_.engage_velocity * node_param_.engage_exit_ratio;
  if (vehicle_speed < engage_vel_thr) {
    if (target_vel >= node_param_.engage_velocity) {
      const double stop_dist = ks::trajectory_utils::calcStopDistance(input_traj, input_closest);
      if (stop_dist > node_param_.stop_dist_to_prohibit_engage) {
        return {
          Motion{node_param_.engage_velocity, node_param_.engage_acceleration},
          InitializeType::ENGAGING};
      }
    } else if (target_vel > 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *clock_, 3000,
        "target velocity (%.3f) is lower than engage velocity (%.3f).", target_vel,
        node_param_.engage_velocity);
    }
  }

  return {Motion{desired_vel, desired_acc}, InitializeType::NORMAL};
}

void FmsKinematicSmootherNode::overwriteStopPoint(
  const TrajectoryPoints & input, TrajectoryPoints & output) const
{
  const auto stop_idx = autoware::motion_utils::searchZeroVelocityIndex(input);
  if (!stop_idx) {
    return;
  }
  const auto nearest_output_point_idx = autoware::motion_utils::findNearestIndex(
    output, input.at(*stop_idx).pose, node_param_.ego_nearest_dist_threshold,
    node_param_.ego_nearest_yaw_threshold);
  if (nearest_output_point_idx) {
    ks::trajectory_utils::applyMaximumVelocityLimit(
      *nearest_output_point_idx, output.size(), 0.0, output);
  }
}

void FmsKinematicSmootherNode::applyExternalVelocityLimit(TrajectoryPoints & traj) const
{
  if (traj.size() < 1) {
    return;
  }
  ks::trajectory_utils::applyMaximumVelocityLimit(
    0, traj.size(), max_velocity_with_deceleration_, traj);

  const auto & current_pose = current_odometry_ptr_->pose.pose;
  const size_t closest_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      traj, current_pose, node_param_.ego_nearest_dist_threshold,
      node_param_.ego_nearest_yaw_threshold);
  const auto inserted_index =
    autoware::motion_utils::insertTargetPoint(closest_seg_idx, external_velocity_limit_.dist, traj);
  if (!inserted_index) {
    traj.back().longitudinal_velocity_mps = std::min(
      traj.back().longitudinal_velocity_mps,
      static_cast<float>(external_velocity_limit_.velocity));
    return;
  }
  ks::trajectory_utils::applyMaximumVelocityLimit(
    *inserted_index, traj.size(), external_velocity_limit_.velocity, traj);
}

void FmsKinematicSmootherNode::applyStopApproachingVelocity(TrajectoryPoints & traj) const
{
  const auto stop_idx = autoware::motion_utils::searchZeroVelocityIndex(traj);
  if (!stop_idx) {
    return;
  }
  double distance_sum = 0.0;
  for (size_t i = *stop_idx - 1; i < traj.size(); --i) {
    distance_sum += autoware::universe_utils::calcDistance2d(traj.at(i), traj.at(i + 1));
    if (distance_sum > node_param_.stopping_distance) {
      break;
    }
    if (traj.at(i).longitudinal_velocity_mps > node_param_.stopping_velocity) {
      traj.at(i).longitudinal_velocity_mps = node_param_.stopping_velocity;
    }
  }
}

void FmsKinematicSmootherNode::updatePrevValues(const TrajectoryPoints & final_result)
{
  prev_output_ = final_result;
  prev_closest_point_ = calcProjectedTrajectoryPointFromEgo(final_result);
}

double FmsKinematicSmootherNode::calcTravelDistance() const
{
  const auto closest_point = calcProjectedTrajectoryPointFromEgo(prev_output_);
  if (prev_closest_point_) {
    return autoware::universe_utils::calcDistance2d(*prev_closest_point_, closest_point);
  }
  return 0.0;
}

bool FmsKinematicSmootherNode::isEngageStatus(const double target_vel) const
{
  const double vehicle_speed = std::fabs(current_odometry_ptr_->twist.twist.linear.x);
  const double engage_vel_thr = node_param_.engage_velocity * node_param_.engage_exit_ratio;
  return vehicle_speed < engage_vel_thr && target_vel >= node_param_.engage_velocity;
}

Trajectory FmsKinematicSmootherNode::toTrajectoryMsg(const TrajectoryPoints & points) const
{
  auto trajectory = autoware::motion_utils::convertToTrajectory(points);
  trajectory.header = base_traj_raw_ptr_->header;
  return trajectory;
}

size_t FmsKinematicSmootherNode::findNearestIndexFromEgo(const TrajectoryPoints & points) const
{
  return autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    points, current_odometry_ptr_->pose.pose, node_param_.ego_nearest_dist_threshold,
    node_param_.ego_nearest_yaw_threshold);
}

bool FmsKinematicSmootherNode::isReverse(const TrajectoryPoints & points) const
{
  if (points.empty()) return true;
  return std::any_of(
    points.begin(), points.end(), [](const auto & pt) { return pt.longitudinal_velocity_mps < 0; });
}

void FmsKinematicSmootherNode::flipVelocity(TrajectoryPoints & points) const
{
  for (auto & pt : points) {
    pt.longitudinal_velocity_mps *= -1.0;
  }
}

TrajectoryPoint FmsKinematicSmootherNode::calcProjectedTrajectoryPoint(
  const TrajectoryPoints & trajectory, const Pose & pose) const
{
  const size_t current_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      trajectory, pose, node_param_.ego_nearest_dist_threshold,
      node_param_.ego_nearest_yaw_threshold);
  return ks::trajectory_utils::calcInterpolatedTrajectoryPoint(trajectory, pose, current_seg_idx);
}

TrajectoryPoint FmsKinematicSmootherNode::calcProjectedTrajectoryPointFromEgo(
  const TrajectoryPoints & trajectory) const
{
  return calcProjectedTrajectoryPoint(trajectory, current_odometry_ptr_->pose.pose);
}

}  // namespace igv_velocity_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(igv_velocity_planner::FmsKinematicSmootherNode)
