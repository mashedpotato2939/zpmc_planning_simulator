// Copyright 2024 ZPMC
// Licensed under the Apache License, Version 2.0

#include "igv_velocity_planner/simple_object_predictor_node.hpp"

#include <tf2/utils.h>

#include <cmath>

namespace igv_velocity_planner
{

SimpleObjectPredictorNode::SimpleObjectPredictorNode(const rclcpp::NodeOptions & options)
: Node("simple_object_predictor", options)
{
  prediction_time_horizon_ = declare_parameter<double>("prediction_time_horizon", 3.0);
  prediction_sampling_dt_ = declare_parameter<double>("prediction_sampling_dt", 0.5);

  set_param_handle_ = add_on_set_parameters_callback(
    std::bind(&SimpleObjectPredictorNode::onSetParameters, this, std::placeholders::_1));

  sub_objects_ = create_subscription<autoware_perception_msgs::msg::TrackedObjects>(
    "~/input/objects", rclcpp::QoS(10),
    std::bind(&SimpleObjectPredictorNode::objectsCallback, this, std::placeholders::_1));

  pub_objects_ = create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
    "~/output/objects", rclcpp::QoS(10));

  RCLCPP_INFO(get_logger(), "Simple Object Predictor initialized (no map dependency)");
  RCLCPP_INFO(get_logger(), "  prediction_time_horizon: %.1f s", prediction_time_horizon_);
  RCLCPP_INFO(get_logger(), "  prediction_sampling_dt: %.2f s", prediction_sampling_dt_);
}

rcl_interfaces::msg::SetParametersResult SimpleObjectPredictorNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name != "prediction_time_horizon" && name != "prediction_sampling_dt") {
      continue;
    }

    rcl_interfaces::msg::SetParametersResult result;
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      result.successful = false;
      result.reason = name + " must be a double";
      return result;
    }

    if (parameter.as_double() <= 0.0) {
      result.successful = false;
      result.reason = name + " must be positive";
      return result;
    }
  }

  double prediction_time_horizon = 0.0;
  double prediction_sampling_dt = 0.0;
  {
    std::lock_guard<std::mutex> lock(param_mutex_);
    for (const auto & parameter : parameters) {
      const auto & name = parameter.get_name();
      if (name == "prediction_time_horizon") {
        prediction_time_horizon_ = parameter.as_double();
      } else if (name == "prediction_sampling_dt") {
        prediction_sampling_dt_ = parameter.as_double();
      }
    }
    prediction_time_horizon = prediction_time_horizon_;
    prediction_sampling_dt = prediction_sampling_dt_;
  }

  RCLCPP_INFO(
    get_logger(), "Updated prediction parameters: prediction_time_horizon=%.2f s, "
    "prediction_sampling_dt=%.2f s",
    prediction_time_horizon, prediction_sampling_dt);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void SimpleObjectPredictorNode::objectsCallback(
  const autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr msg)
{
  double prediction_time_horizon;
  double prediction_sampling_dt;
  {
    std::lock_guard<std::mutex> lock(param_mutex_);
    prediction_time_horizon = prediction_time_horizon_;
    prediction_sampling_dt = prediction_sampling_dt_;
  }

  autoware_perception_msgs::msg::PredictedObjects output;
  output.header = msg->header;

  for (const auto & tracked_obj : msg->objects) {
    autoware_perception_msgs::msg::PredictedObject pred_obj;
    pred_obj.object_id = tracked_obj.object_id;
    pred_obj.existence_probability = tracked_obj.existence_probability;
    pred_obj.classification = tracked_obj.classification;
    pred_obj.shape = tracked_obj.shape;

    // Copy kinematics
    pred_obj.kinematics.initial_pose_with_covariance =
      tracked_obj.kinematics.pose_with_covariance;
    pred_obj.kinematics.initial_twist_with_covariance =
      tracked_obj.kinematics.twist_with_covariance;
    pred_obj.kinematics.initial_acceleration_with_covariance =
      tracked_obj.kinematics.acceleration_with_covariance;

    // Generate constant-velocity predicted path
    autoware_perception_msgs::msg::PredictedPath predicted_path;
    predicted_path.confidence = 1.0;
    predicted_path.time_step = rclcpp::Duration::from_seconds(prediction_sampling_dt);

    const auto & pose = tracked_obj.kinematics.pose_with_covariance.pose;
    const auto & twist = tracked_obj.kinematics.twist_with_covariance.twist;

    const double vx = twist.linear.x;
    const double vy = twist.linear.y;
    const double yaw = tf2::getYaw(pose.orientation);
    const double omega = twist.angular.z;

    // World-frame velocities
    const double vx_world = vx * std::cos(yaw) - vy * std::sin(yaw);
    const double vy_world = vx * std::sin(yaw) + vy * std::cos(yaw);

    const int num_steps =
      static_cast<int>(prediction_time_horizon / prediction_sampling_dt) + 1;

    for (int i = 0; i < num_steps; ++i) {
      const double dt = i * prediction_sampling_dt;

      geometry_msgs::msg::Pose predicted_pose;
      predicted_pose.position.x = pose.position.x + vx_world * dt;
      predicted_pose.position.y = pose.position.y + vy_world * dt;
      predicted_pose.position.z = pose.position.z;

      // Update orientation with angular velocity
      const double predicted_yaw = yaw + omega * dt;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, predicted_yaw);
      predicted_pose.orientation.x = q.x();
      predicted_pose.orientation.y = q.y();
      predicted_pose.orientation.z = q.z();
      predicted_pose.orientation.w = q.w();

      predicted_path.path.push_back(predicted_pose);
    }

    pred_obj.kinematics.predicted_paths.push_back(predicted_path);
    output.objects.push_back(pred_obj);
  }

  pub_objects_->publish(output);
}

}  // namespace igv_velocity_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(igv_velocity_planner::SimpleObjectPredictorNode)
