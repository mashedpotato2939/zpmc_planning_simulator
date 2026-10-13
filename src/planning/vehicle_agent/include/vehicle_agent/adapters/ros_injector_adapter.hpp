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

#ifndef VEHICLE_AGENT__ADAPTERS__ROS_INJECTOR_ADAPTER_HPP_
#define VEHICLE_AGENT__ADAPTERS__ROS_INJECTOR_ADAPTER_HPP_

#include "vehicle_agent/adapters/i_cloud_protocol_adapter.hpp"

#include <igv_planning_msgs/msg/ir_command.hpp>
#include <igv_planning_msgs/msg/ir_mission.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

namespace vehicle_agent
{

// Adapter that ingests IR-aligned ROS messages from operator-side producers
// (RViz panels, debug scripts, replay tools) and feeds them into the FSM as
// if they came from a cloud broker. Lets the existing FmsBridgePanel exercise
// the full VA path (FSM, queue, RosPublisher, route_bridge) end-to-end.
//
// Why an adapter instead of a new RosSubscriber:
//   - downstream code already routes adapter callbacks through the offline
//     queue and the FSM. Reusing the contract means no special-case wiring.
//   - the multiplexer can run ros_injector alongside ahtfms_ns so an operator
//     can inject test missions while a real cloud link is also live.
class RosInjectorAdapter : public ICloudProtocolAdapter
{
public:
  struct Config
  {
    std::string mission_topic{"/vehicle_agent/inject_mission"};
    std::string command_topic{"/vehicle_agent/inject_command"};
  };

  struct Counters
  {
    uint64_t missions_in{0};
    uint64_t commands_in{0};
  };

  // Production constructor: binds to the live VehicleAgentNode for its
  // subscriptions. Keeping a raw rclcpp::Node * is safe because the node
  // owns the adapter and the adapter is destroyed before the node.
  RosInjectorAdapter(rclcpp::Node * node, Config cfg);

  std::string name() const override { return "ros_injector"; }
  AdapterCapabilities capabilities() const override;

  void start(AdapterCallbacks callbacks) override;
  void stop() override;
  bool connected() const override { return started_.load(); }

  // Uplink no-ops: the operator side has nowhere meaningful to send these.
  void publishTelemetry(const ir::Telemetry &) override {}
  void publishMissionStatus(const ir::MissionStatus &) override {}
  void publishAck(const ir::Ack &) override {}

  Counters counters() const;

private:
  void onMission(igv_planning_msgs::msg::IrMission::ConstSharedPtr msg);
  void onCommand(igv_planning_msgs::msg::IrCommand::ConstSharedPtr msg);

  rclcpp::Node * node_;
  Config cfg_;
  AdapterCallbacks callbacks_;
  std::atomic<bool> started_{false};

  rclcpp::Subscription<igv_planning_msgs::msg::IrMission>::SharedPtr sub_mission_;
  rclcpp::Subscription<igv_planning_msgs::msg::IrCommand>::SharedPtr sub_command_;

  mutable std::mutex counters_mtx_;
  Counters counters_{};
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__ADAPTERS__ROS_INJECTOR_ADAPTER_HPP_
