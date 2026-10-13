// Copyright 2026 ZPMC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "planning_ego_acceleration_estimator/ego_acceleration_estimator_node.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace planning_ego_acceleration_estimator
{

EgoAccelerationEstimator::EgoAccelerationEstimator(const rclcpp::NodeOptions & options)
: rclcpp::Node("ego_acceleration_estimator", options)
{
  window_seconds_ = declare_parameter<double>("window_seconds", 0.4);
  min_window_samples_ = declare_parameter<int>("min_window_samples", 5);
  polyorder_ = declare_parameter<int>("polyorder", 3);
  min_dt_ = declare_parameter<double>("min_dt", 0.005);
  max_dt_ = declare_parameter<double>("max_dt", 0.5);
  alpha_ = declare_parameter<double>("alpha", 1.0);
  linear_variance_ = declare_parameter<double>("linear_variance", 1.0);
  angular_variance_ = declare_parameter<double>("angular_variance", 0.05);
  const auto input_topic = declare_parameter<std::string>(
    "input_topic", "/localization/kinematic_state");
  const auto output_topic = declare_parameter<std::string>(
    "output_topic", "/planning/acceleration");

  if (polyorder_ < 1) {
    RCLCPP_WARN(get_logger(), "polyorder=%d is invalid; clamping to 1", polyorder_);
    polyorder_ = 1;
  }
  if (min_window_samples_ < polyorder_ + 1) {
    min_window_samples_ = polyorder_ + 1;
  }

  // alpha=1.0 means LPF disabled (output passes through unmodified). The
  // LowpassFilter1d uses gain in the form y = gain*y_prev + (1-gain)*u, so the
  // coefficient passed to it is (1 - alpha).
  lpf_ = std::make_shared<autoware::signal_processing::LowpassFilter1d>(1.0 - alpha_);

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    input_topic, rclcpp::QoS(1),
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { this->on_odometry(msg); });
  pub_accel_ = create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
    output_topic, rclcpp::QoS(1));

  RCLCPP_INFO(
    get_logger(),
    "ego_acceleration_estimator: input=%s output=%s window=%.3fs polyorder=%d alpha=%.3f",
    input_topic.c_str(), output_topic.c_str(), window_seconds_, polyorder_, alpha_);
}

void EgoAccelerationEstimator::on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const double t_abs = rclcpp::Time(msg->header.stamp).seconds();

  if (!have_t0_) {
    t0_ = t_abs;
    have_t0_ = true;
    prev_t_abs_ = t_abs;
    buffer_.push_back({0.0, msg->twist.twist.linear.x});
    return;
  }

  const double dt = t_abs - prev_t_abs_;
  if (dt < min_dt_) {
    // duplicate / out-of-order sample; ignore
    return;
  }
  if (dt > max_dt_) {
    // gap; reset
    buffer_.clear();
    lpf_->reset();
    t0_ = t_abs;
    prev_t_abs_ = t_abs;
    buffer_.push_back({0.0, msg->twist.twist.linear.x});
    return;
  }

  prev_t_abs_ = t_abs;
  buffer_.push_back({t_abs - t0_, msg->twist.twist.linear.x});

  // trim oldest samples outside the rolling window
  const double t_rel_now = t_abs - t0_;
  const double t_cutoff = t_rel_now - window_seconds_;
  while (!buffer_.empty() && buffer_.front().t < t_cutoff) {
    buffer_.pop_front();
  }

  if (static_cast<int>(buffer_.size()) < min_window_samples_) {
    return;  // not enough samples yet
  }

  double a_raw = 0.0;
  if (!savitzky_golay_derivative(a_raw)) {
    return;  // ill-conditioned; skip this sample
  }
  const double a_filt = (alpha_ >= 1.0) ? a_raw : lpf_->filter(a_raw);

  geometry_msgs::msg::AccelWithCovarianceStamped accel_msg;
  accel_msg.header.stamp = msg->header.stamp;
  accel_msg.header.frame_id =
    msg->child_frame_id.empty() ? std::string("base_link") : msg->child_frame_id;
  accel_msg.accel.accel.linear.x = a_filt;
  accel_msg.accel.accel.linear.y = 0.0;
  accel_msg.accel.accel.linear.z = 0.0;
  accel_msg.accel.accel.angular.x = 0.0;
  accel_msg.accel.accel.angular.y = 0.0;
  accel_msg.accel.accel.angular.z = 0.0;
  accel_msg.accel.covariance[0 * 6 + 0] = linear_variance_;
  accel_msg.accel.covariance[1 * 6 + 1] = linear_variance_;
  accel_msg.accel.covariance[2 * 6 + 2] = linear_variance_;
  accel_msg.accel.covariance[3 * 6 + 3] = angular_variance_;
  accel_msg.accel.covariance[4 * 6 + 4] = angular_variance_;
  accel_msg.accel.covariance[5 * 6 + 5] = angular_variance_;

  pub_accel_->publish(accel_msg);
}

bool EgoAccelerationEstimator::savitzky_golay_derivative(double & a_out) const
{
  // Fit v(τ) ≈ Σ_{k=0..K} c_k * τ^k by ordinary least squares, where τ = t - t_now,
  // so the most-recent sample sits at τ = 0 and v'(0) = c_1 is what we want.
  //
  // Solve normal equations (A^T A) c = A^T b with hand-rolled Gauss elimination.
  // K = polyorder_ ≤ 3 in practice, so the matrix is at most 4x4.
  const int K = polyorder_;
  const int N = static_cast<int>(buffer_.size());
  const int dim = K + 1;
  const double t_now = buffer_.back().t;

  // Build Gram matrix M = A^T A and rhs r = A^T b.
  // M[i][j] = Σ τ^(i+j) ;  r[i] = Σ τ^i * v
  std::vector<std::vector<double>> M(dim, std::vector<double>(dim, 0.0));
  std::vector<double> r(dim, 0.0);
  for (int n = 0; n < N; ++n) {
    const double tau = buffer_[n].t - t_now;
    const double v = buffer_[n].v;
    double tau_pow_i = 1.0;
    std::vector<double> powers(2 * dim - 1, 0.0);
    powers[0] = 1.0;
    for (int p = 1; p < 2 * dim - 1; ++p) {
      powers[p] = powers[p - 1] * tau;
    }
    for (int i = 0; i < dim; ++i) {
      r[i] += powers[i] * v;
      for (int j = 0; j < dim; ++j) {
        M[i][j] += powers[i + j];
      }
    }
    (void)tau_pow_i;
  }

  // Gauss elimination with partial pivoting on [M | r].
  std::vector<std::vector<double>> A(dim, std::vector<double>(dim + 1, 0.0));
  for (int i = 0; i < dim; ++i) {
    for (int j = 0; j < dim; ++j) {
      A[i][j] = M[i][j];
    }
    A[i][dim] = r[i];
  }
  for (int col = 0; col < dim; ++col) {
    // pivot
    int pivot = col;
    double pivot_abs = std::abs(A[col][col]);
    for (int row = col + 1; row < dim; ++row) {
      if (std::abs(A[row][col]) > pivot_abs) {
        pivot = row;
        pivot_abs = std::abs(A[row][col]);
      }
    }
    if (pivot_abs < 1e-12) {
      return false;  // singular
    }
    if (pivot != col) {
      std::swap(A[col], A[pivot]);
    }
    // eliminate
    for (int row = 0; row < dim; ++row) {
      if (row == col) continue;
      const double factor = A[row][col] / A[col][col];
      for (int k = col; k <= dim; ++k) {
        A[row][k] -= factor * A[col][k];
      }
    }
  }

  // c_1 (the slope at τ = 0) is the second unknown.
  if (dim < 2) {
    return false;
  }
  a_out = A[1][dim] / A[1][1];
  return std::isfinite(a_out);
}

}  // namespace planning_ego_acceleration_estimator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  planning_ego_acceleration_estimator::EgoAccelerationEstimator)
