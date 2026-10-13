// Copyright 2024 ZPMC
// Licensed under the Apache License, Version 2.0

#ifndef FMS_TRAJECTORY_BRIDGE__SIMPLE_OBJECT_PREDICTOR_NODE_HPP_
#define FMS_TRAJECTORY_BRIDGE__SIMPLE_OBJECT_PREDICTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <memory>
#include <mutex>
#include <vector>

namespace igv_velocity_planner
{

/// Simple velocity-based object predictor. No map dependency.
/// Predicts object trajectories using constant-velocity model for a short horizon (2-3s).
/// Replaces map_based_prediction in FMS mode where lane-based prediction is not needed.
class SimpleObjectPredictorNode : public rclcpp::Node
{
public:
  explicit SimpleObjectPredictorNode(const rclcpp::NodeOptions & options);

private:
  void objectsCallback(
    const autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr msg);

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::Subscription<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr sub_objects_;
  rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr pub_objects_;
  OnSetParametersCallbackHandle::SharedPtr set_param_handle_;

  std::mutex param_mutex_;
  double prediction_time_horizon_;   // seconds (default 3.0)
  double prediction_sampling_dt_;    // seconds (default 0.5)
};

}  // namespace igv_velocity_planner

#endif  // FMS_TRAJECTORY_BRIDGE__SIMPLE_OBJECT_PREDICTOR_NODE_HPP_
