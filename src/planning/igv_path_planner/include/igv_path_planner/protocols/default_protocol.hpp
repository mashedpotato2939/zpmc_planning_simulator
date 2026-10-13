// Copyright 2024 ZPMC
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

#ifndef FMS_TRAJECTORY_BRIDGE__PROTOCOLS__DEFAULT_PROTOCOL_HPP_
#define FMS_TRAJECTORY_BRIDGE__PROTOCOLS__DEFAULT_PROTOCOL_HPP_

#include "igv_path_planner/protocols/fms_protocol.hpp"

namespace igv_path_planner
{

/// ICD-aligned protocol — both downlink (§2) and uplink (§1).
/// Downlink: subscribes to FMSRouteWindow/FMSStopPoint (ICD §2.2/§2.3)
/// Uplink: publishes VehicleStatus, TrajectoryReport, JobEvent, DetourRequest
class DefaultProtocol : public FMSProtocol
{
public:
  void initialize(rclcpp::Node & node, const ProtocolCallbacks & cb) override;
  std::string name() const override { return "default"; }

  void reportVehicleStatus(const igv_planning_msgs::msg::VehicleStatus & status) override;
  void reportTrajectory(const igv_planning_msgs::msg::TrajectoryReport & report) override;
  void reportJobEvent(const igv_planning_msgs::msg::JobEvent & event) override;
  void requestDetour(const igv_planning_msgs::msg::DetourRequest & request) override;

private:
  // Downlink subscriptions
  rclcpp::Subscription<igv_planning_msgs::msg::FMSRouteWindow>::SharedPtr route_sub_;
  rclcpp::Subscription<igv_planning_msgs::msg::FMSStopPoint>::SharedPtr stop_sub_;
  // Uplink publishers
  rclcpp::Publisher<igv_planning_msgs::msg::VehicleStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<igv_planning_msgs::msg::TrajectoryReport>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<igv_planning_msgs::msg::JobEvent>::SharedPtr job_event_pub_;
  rclcpp::Publisher<igv_planning_msgs::msg::DetourRequest>::SharedPtr detour_pub_;
};

}  // namespace igv_path_planner
#endif  // FMS_TRAJECTORY_BRIDGE__PROTOCOLS__DEFAULT_PROTOCOL_HPP_
