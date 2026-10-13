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

#ifndef VEHICLE_AGENT__ROS_BRIDGE__ROS_PUBLISHER_HPP_
#define VEHICLE_AGENT__ROS_BRIDGE__ROS_PUBLISHER_HPP_

#include "vehicle_agent/ir/command.hpp"
#include "vehicle_agent/ir/mission.hpp"
#include "vehicle_agent/ir/mission_status.hpp"

#include <igv_planning_msgs/msg/ir_command.hpp>
#include <igv_planning_msgs/msg/ir_mission.hpp>
#include <igv_planning_msgs/msg/ir_mission_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace vehicle_agent
{

class RosPublisher
{
public:
  RosPublisher(rclcpp::Node & node, const std::string & ns_prefix);

  void publishMission(const ir::Mission & m);
  void publishCommand(const ir::Command & c);
  void publishMissionStatus(const ir::MissionStatus & s);

private:
  rclcpp::Logger logger_;
  rclcpp::Publisher<igv_planning_msgs::msg::IrMission>::SharedPtr mission_pub_;
  rclcpp::Publisher<igv_planning_msgs::msg::IrCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<igv_planning_msgs::msg::IrMissionStatus>::SharedPtr status_pub_;
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__ROS_BRIDGE__ROS_PUBLISHER_HPP_
