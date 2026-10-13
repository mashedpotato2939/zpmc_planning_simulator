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

#ifndef VEHICLE_AGENT__ROS_BRIDGE__ROS_SUBSCRIBER_HPP_
#define VEHICLE_AGENT__ROS_BRIDGE__ROS_SUBSCRIBER_HPP_

#include "vehicle_agent/ir/telemetry.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <mutex>
#include <string>

namespace vehicle_agent
{

// Subscribes to vehicle telemetry ROS topics and produces ir::Telemetry
// samples. Sprint 2 minimum: /localization/kinematic_state (Odometry) gives
// us pose + speed. Sprint 3+ will add steering, control_mode, etc.
class RosSubscriber
{
public:
  struct Config
  {
    std::string vehicle_id;
    std::string odometry_topic{"/localization/kinematic_state"};
    double max_rate_hz{10.0};
  };

  struct Sinks
  {
    std::function<void(const ir::Telemetry &)> on_telemetry;
  };

  RosSubscriber(rclcpp::Node & node, Config cfg, Sinks sinks);

  // Mission-state enrichment. Called by the FSM status sink so emitted
  // telemetry carries the latest mission context.
  void updateMissionContext(const std::string & mission_id,
                            uint8_t mission_state,
                            bool e_stop_active);

private:
  void onOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  rclcpp::Logger logger_;
  Config cfg_;
  Sinks sinks_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  mutable std::mutex ctx_mtx_;
  std::string mission_id_;
  uint8_t mission_state_{0};
  bool e_stop_{false};
  uint64_t last_emit_ms_{0};
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__ROS_BRIDGE__ROS_SUBSCRIBER_HPP_
