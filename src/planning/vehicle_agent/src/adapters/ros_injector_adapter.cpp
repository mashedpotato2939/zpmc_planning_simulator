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

#include "vehicle_agent/adapters/ros_injector_adapter.hpp"

#include <utility>

namespace vehicle_agent
{

RosInjectorAdapter::RosInjectorAdapter(rclcpp::Node * node, Config cfg)
: node_(node), cfg_(std::move(cfg))
{
}

AdapterCapabilities RosInjectorAdapter::capabilities() const
{
  AdapterCapabilities c;
  c.supports_mission = true;
  c.supports_command = true;
  c.supports_telemetry_uplink = false;
  c.supports_ack = false;
  c.supports_estop = true;          // ESTOP commands can be injected too
  c.priority = 50;                  // operator override sits below cloud adapters
  return c;
}

void RosInjectorAdapter::start(AdapterCallbacks callbacks)
{
  if (started_.exchange(true)) return;
  callbacks_ = std::move(callbacks);

  sub_mission_ = node_->create_subscription<igv_planning_msgs::msg::IrMission>(
    cfg_.mission_topic, rclcpp::QoS(10),
    [this](igv_planning_msgs::msg::IrMission::ConstSharedPtr msg) {
      onMission(msg);
    });
  sub_command_ = node_->create_subscription<igv_planning_msgs::msg::IrCommand>(
    cfg_.command_topic, rclcpp::QoS(10),
    [this](igv_planning_msgs::msg::IrCommand::ConstSharedPtr msg) {
      onCommand(msg);
    });

  if (callbacks_.on_connection_state_changed) {
    callbacks_.on_connection_state_changed(name(), true);
  }
}

void RosInjectorAdapter::stop()
{
  if (!started_.exchange(false)) return;
  // Drop subscriptions first so no further callbacks land after we report
  // disconnected — avoids a use-after-stop on the connection-state sink.
  sub_mission_.reset();
  sub_command_.reset();
  if (callbacks_.on_connection_state_changed) {
    callbacks_.on_connection_state_changed(name(), false);
  }
}

void RosInjectorAdapter::onMission(
  igv_planning_msgs::msg::IrMission::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lk(counters_mtx_);
    ++counters_.missions_in;
  }
  ir::Mission m;
  m.id = msg->id;
  m.kind = static_cast<ir::MissionKind>(msg->kind);
  m.seq = msg->seq;
  m.issued_at_ms = msg->issued_at_ms;
  if (msg->deadline_ms != 0) m.deadline_ms = msg->deadline_ms;
  m.waypoints.reserve(msg->waypoints.size());
  for (const auto & w : msg->waypoints) {
    ir::Waypoint wp;
    wp.x = w.x;
    wp.y = w.y;
    wp.z = w.z;
    wp.yaw = w.yaw;
    wp.target_speed = w.target_speed;
    wp.segment_id = w.segment_id;
    wp.point_type = w.point_type;
    m.waypoints.push_back(wp);
  }
  // Stamp the source unconditionally so the FSM and downstream consumers
  // can attribute the inbound regardless of what the panel set (the panel
  // is "honest by toggle position", not by string contents).
  m.source_adapter = name();
  if (callbacks_.on_mission) callbacks_.on_mission(std::move(m));
}

void RosInjectorAdapter::onCommand(
  igv_planning_msgs::msg::IrCommand::ConstSharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lk(counters_mtx_);
    ++counters_.commands_in;
  }
  ir::Command c;
  c.mission_id = msg->mission_id;
  c.kind = static_cast<ir::CommandKind>(msg->kind);
  c.issued_at_ms = msg->issued_at_ms;
  c.source_adapter = name();
  if (callbacks_.on_command) callbacks_.on_command(std::move(c));
}

RosInjectorAdapter::Counters RosInjectorAdapter::counters() const
{
  std::lock_guard<std::mutex> lk(counters_mtx_);
  return counters_;
}

}  // namespace vehicle_agent
