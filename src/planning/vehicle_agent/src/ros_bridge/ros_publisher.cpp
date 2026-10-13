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

#include "vehicle_agent/ros_bridge/ros_publisher.hpp"

namespace vehicle_agent
{

RosPublisher::RosPublisher(rclcpp::Node & node, const std::string & ns_prefix)
: logger_(node.get_logger())
{
  const auto event_qos = rclcpp::QoS(10);
  const auto state_qos = rclcpp::QoS(10).transient_local();
  mission_pub_ = node.create_publisher<igv_planning_msgs::msg::IrMission>(
    ns_prefix + "/mission", event_qos);
  command_pub_ = node.create_publisher<igv_planning_msgs::msg::IrCommand>(
    ns_prefix + "/command", event_qos);
  status_pub_ = node.create_publisher<igv_planning_msgs::msg::IrMissionStatus>(
    ns_prefix + "/mission_status", state_qos);
}

void RosPublisher::publishMission(const ir::Mission & m)
{
  igv_planning_msgs::msg::IrMission msg;
  msg.id = m.id;
  msg.kind = static_cast<uint8_t>(m.kind);
  msg.seq = m.seq;
  msg.issued_at_ms = m.issued_at_ms;
  msg.deadline_ms = m.deadline_ms.value_or(0);
  msg.source_adapter = m.source_adapter;
  msg.waypoints.reserve(m.waypoints.size());
  for (const auto & w : m.waypoints) {
    igv_planning_msgs::msg::IrWaypoint wp;
    wp.x = w.x;
    wp.y = w.y;
    wp.z = w.z;
    wp.yaw = w.yaw;
    wp.target_speed = w.target_speed;
    wp.segment_id = w.segment_id;
    wp.point_type = w.point_type;
    msg.waypoints.push_back(wp);
  }
  mission_pub_->publish(msg);
}

void RosPublisher::publishCommand(const ir::Command & c)
{
  igv_planning_msgs::msg::IrCommand msg;
  msg.mission_id = c.mission_id;
  msg.kind = static_cast<uint8_t>(c.kind);
  msg.issued_at_ms = c.issued_at_ms;
  msg.source_adapter = c.source_adapter;
  command_pub_->publish(msg);
}

void RosPublisher::publishMissionStatus(const ir::MissionStatus & s)
{
  igv_planning_msgs::msg::IrMissionStatus msg;
  msg.mission_id = s.mission_id;
  msg.state = static_cast<uint8_t>(s.state);
  msg.stamp_ms = s.stamp_ms;
  msg.progress = s.progress;
  msg.detail = s.detail;
  status_pub_->publish(msg);
}

}  // namespace vehicle_agent
