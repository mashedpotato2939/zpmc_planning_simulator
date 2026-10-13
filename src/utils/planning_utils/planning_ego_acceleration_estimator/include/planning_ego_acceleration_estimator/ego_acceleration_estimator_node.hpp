// Copyright 2026 ZPMC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef PLANNING_EGO_ACCELERATION_ESTIMATOR__EGO_ACCELERATION_ESTIMATOR_NODE_HPP_
#define PLANNING_EGO_ACCELERATION_ESTIMATOR__EGO_ACCELERATION_ESTIMATOR_NODE_HPP_

#include <autoware/signal_processing/lowpass_filter_1d.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <memory>
#include <string>

namespace planning_ego_acceleration_estimator
{

class EgoAccelerationEstimator : public rclcpp::Node
{
public:
  explicit EgoAccelerationEstimator(const rclcpp::NodeOptions & options);

private:
  void on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // Returns first derivative of v(t) at the most-recent sample, fit by
  // weighted least squares to a polynomial of degree polyorder_ over the
  // contents of buffer_. Returns false if the fit is ill-conditioned.
  bool savitzky_golay_derivative(double & a_out) const;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr pub_accel_;

  // parameters
  double window_seconds_;
  int min_window_samples_;
  int polyorder_;
  double min_dt_;
  double max_dt_;
  double alpha_;
  double linear_variance_;
  double angular_variance_;

  // state
  struct Sample
  {
    double t;  // seconds (relative to first sample to keep magnitudes small)
    double v;
  };
  std::deque<Sample> buffer_;
  bool have_t0_{false};
  double t0_{0.0};
  double prev_t_abs_{0.0};
  std::shared_ptr<autoware::signal_processing::LowpassFilter1d> lpf_;
};

}  // namespace planning_ego_acceleration_estimator

#endif  // PLANNING_EGO_ACCELERATION_ESTIMATOR__EGO_ACCELERATION_ESTIMATOR_NODE_HPP_
