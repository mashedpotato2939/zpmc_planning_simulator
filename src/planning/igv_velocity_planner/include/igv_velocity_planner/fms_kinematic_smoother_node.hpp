// Copyright 2021 Tier IV, Inc.
// Copyright 2026 ZPMC (slim port for igv_velocity_planner — Analytical only,
//                      operation-mode coupling + debug publishers dropped)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef FMS_TRAJECTORY_BRIDGE__FMS_KINEMATIC_SMOOTHER_NODE_HPP_
#define FMS_TRAJECTORY_BRIDGE__FMS_KINEMATIC_SMOOTHER_NODE_HPP_

#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/math/unit_conversion.hpp"
#include "autoware/universe_utils/ros/polling_subscriber.hpp"
#include "autoware/universe_utils/ros/self_pose_listener.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"
#include "autoware/universe_utils/system/time_keeper.hpp"
#include "igv_velocity_planner/kinematic_smoother/analytical_smoother.hpp"
#include "igv_velocity_planner/kinematic_smoother/resample.hpp"
#include "igv_velocity_planner/kinematic_smoother/smoother_base.hpp"
#include "igv_velocity_planner/kinematic_smoother/trajectory_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_planning_msgs/msg/path.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tier4_debug_msgs/msg/float64_stamped.hpp"
#include "tier4_planning_msgs/msg/velocity_limit.hpp"

#include <boost/optional.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace igv_velocity_planner
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;
using tier4_debug_msgs::msg::Float64Stamped;
using tier4_planning_msgs::msg::VelocityLimit;

struct Motion
{
  double vel{0.0};
  double acc{0.0};
  Motion() = default;
  Motion(const double v, const double a) : vel(v), acc(a) {}
};

class FmsKinematicSmootherNode : public rclcpp::Node
{
public:
  explicit FmsKinematicSmootherNode(const rclcpp::NodeOptions & node_options);

private:
  using AnalyticalJerkConstrainedSmoother =
    igv_velocity_planner::kinematic_smoother::AnalyticalJerkConstrainedSmoother;
  using SmootherBase = igv_velocity_planner::kinematic_smoother::SmootherBase;

  enum class InitializeType {
    EGO_VELOCITY = 0,
    LARGE_DEVIATION_REPLAN = 1,
    ENGAGING = 2,
    NORMAL = 3,
  };

  struct Param
  {
    bool enable_lateral_acc_limit;
    bool enable_steering_rate_limit;

    double max_velocity;                              // [m/s]
    double margin_to_insert_external_velocity_limit;  // [m]
    double replan_vel_deviation;                      // [m/s]
    double engage_velocity;                           // [m/s]
    double engage_acceleration;                       // [m/ss]
    double engage_exit_ratio;
    double stopping_velocity;             // [m/s]
    double stopping_distance;             // [m]
    double extract_ahead_dist;            // [m]
    double extract_behind_dist;           // [m]
    double stop_dist_to_prohibit_engage;  // [m]
    double ego_nearest_dist_threshold;
    double ego_nearest_yaw_threshold;

    kinematic_smoother::resampling::ResampleParam post_resample_param;
  } node_param_{};

  struct ExternalVelocityLimit
  {
    double velocity{0.0};
    double dist{0.0};
    std::string sender;
  } external_velocity_limit_;

  // ---- ROS interfaces -------------------------------------------------
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr pub_processing_time_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_current_trajectory_;
  autoware::universe_utils::InterProcessPollingSubscriber<Odometry> sub_current_odometry_{
    this, "/localization/kinematic_state"};
  autoware::universe_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped>
    sub_current_acceleration_{this, "~/input/acceleration"};
  autoware::universe_utils::InterProcessPollingSubscriber<
    VelocityLimit, autoware::universe_utils::polling_policy::Newest>
    sub_external_velocity_limit_{this, "~/input/external_velocity_limit_mps"};

  // Maneuver type + behavior_path cache (FMS-specific OBLIQUE/Q_TURN orientation re-injection).
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_maneuver_type_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Path>::SharedPtr sub_behavior_path_;
  uint8_t current_maneuver_type_{0};
  autoware_planning_msgs::msg::Path::ConstSharedPtr behavior_path_msg_{nullptr};

  // ---- Cached inputs --------------------------------------------------
  Odometry::ConstSharedPtr current_odometry_ptr_;
  AccelWithCovarianceStamped::ConstSharedPtr current_acceleration_ptr_;
  VelocityLimit::ConstSharedPtr external_velocity_limit_ptr_{nullptr};
  Trajectory::ConstSharedPtr base_traj_raw_ptr_;

  double max_velocity_with_deceleration_{0.0};
  double wheelbase_{0.0};
  double base_link2front_{0.0};

  TrajectoryPoints prev_output_;
  boost::optional<TrajectoryPoint> prev_closest_point_{};
  boost::optional<TrajectoryPoint> current_closest_point_from_prev_output_{};

  bool is_reverse_{false};

  std::shared_ptr<SmootherBase> smoother_;

  // ---- Callbacks ------------------------------------------------------
  void onCurrentTrajectory(const Trajectory::ConstSharedPtr msg);

  // ---- Core pipeline --------------------------------------------------
  bool checkData() const;
  void calcExternalVelocityLimit();
  void updateDataForExternalVelocityLimit();
  TrajectoryPoints calcTrajectoryVelocity(const TrajectoryPoints & traj_input) const;
  bool smoothVelocity(
    const TrajectoryPoints & input, const size_t input_closest,
    TrajectoryPoints & traj_smoothed) const;
  std::pair<Motion, InitializeType> calcInitialMotion(
    const TrajectoryPoints & input_traj, const size_t input_closest) const;
  void applyExternalVelocityLimit(TrajectoryPoints & traj) const;
  void applyStopApproachingVelocity(TrajectoryPoints & traj) const;
  void overwriteStopPoint(const TrajectoryPoints & input, TrajectoryPoints & output) const;
  void insertBehindVelocity(
    const size_t output_closest, const InitializeType type, TrajectoryPoints & output) const;
  void updatePrevValues(const TrajectoryPoints & final_result);
  double calcTravelDistance() const;
  bool isEngageStatus(const double target_vel) const;

  // ---- Helpers --------------------------------------------------------
  void initCommonParam();
  Trajectory toTrajectoryMsg(const TrajectoryPoints & points) const;
  size_t findNearestIndexFromEgo(const TrajectoryPoints & points) const;
  bool isReverse(const TrajectoryPoints & points) const;
  void flipVelocity(TrajectoryPoints & points) const;
  TrajectoryPoint calcProjectedTrajectoryPoint(
    const TrajectoryPoints & trajectory, const Pose & pose) const;
  TrajectoryPoint calcProjectedTrajectoryPointFromEgo(const TrajectoryPoints & trajectory) const;

  void publishTrajectory(const TrajectoryPoints & traj) const;

  autoware::universe_utils::StopWatch<std::chrono::milliseconds> stop_watch_;
  mutable std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_{nullptr};
  mutable rclcpp::Clock::SharedPtr clock_;
};

}  // namespace igv_velocity_planner

#endif  // FMS_TRAJECTORY_BRIDGE__FMS_KINEMATIC_SMOOTHER_NODE_HPP_
