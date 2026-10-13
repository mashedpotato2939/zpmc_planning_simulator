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

#include "vehicle_agent/ros_bridge/ros_subscriber.hpp"

#include <chrono>
#include <cmath>
#include <utility>

namespace vehicle_agent
{

namespace
{
uint64_t now_ms()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

double yaw_from_quat(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
}  // namespace

RosSubscriber::RosSubscriber(rclcpp::Node & node, Config cfg, Sinks sinks)
: logger_(node.get_logger()),
  cfg_(std::move(cfg)),
  sinks_(std::move(sinks))
{
  const auto qos = rclcpp::QoS(10).best_effort();
  odom_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
    cfg_.odometry_topic, qos,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { onOdom(std::move(m)); });
  RCLCPP_INFO(logger_, "ros_subscriber: odom='%s' rate<=%.1fHz vehicle_id='%s'",
              cfg_.odometry_topic.c_str(), cfg_.max_rate_hz, cfg_.vehicle_id.c_str());
}

void RosSubscriber::updateMissionContext(const std::string & mission_id,
                                         uint8_t mission_state,
                                         bool e_stop_active)
{
  std::lock_guard<std::mutex> lock(ctx_mtx_);
  mission_id_ = mission_id;
  mission_state_ = mission_state;
  e_stop_ = e_stop_active;
}

void RosSubscriber::onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const uint64_t now = now_ms();
  const uint64_t min_gap = (cfg_.max_rate_hz > 0.0)
    ? static_cast<uint64_t>(1000.0 / cfg_.max_rate_hz) : 0u;

  ir::Telemetry t;
  {
    std::lock_guard<std::mutex> lock(ctx_mtx_);
    if (min_gap > 0 && last_emit_ms_ != 0 && (now - last_emit_ms_) < min_gap) {
      return;
    }
    last_emit_ms_ = now;
    t.vehicle_id = cfg_.vehicle_id;
    t.current_mission_id = mission_id_;
    t.mission_state = mission_state_;
    t.e_stop_active = e_stop_;
  }

  t.stamp_ms = now;
  t.pose.x = msg->pose.pose.position.x;
  t.pose.y = msg->pose.pose.position.y;
  t.pose.yaw = yaw_from_quat(
    msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  t.speed_mps = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  t.steering_rad = 0.0;   // populated in Sprint 3 via SteeringReport

  if (sinks_.on_telemetry) sinks_.on_telemetry(t);
}

}  // namespace vehicle_agent
