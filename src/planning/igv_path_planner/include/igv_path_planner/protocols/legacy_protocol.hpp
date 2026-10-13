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

#ifndef FMS_TRAJECTORY_BRIDGE__PROTOCOLS__LEGACY_PROTOCOL_HPP_
#define FMS_TRAJECTORY_BRIDGE__PROTOCOLS__LEGACY_PROTOCOL_HPP_

#include "igv_path_planner/protocols/fms_protocol.hpp"
#include "igv_planning_msgs/msg/fms_path.hpp"
#include <std_msgs/msg/string.hpp>

namespace igv_path_planner
{

/// Legacy protocol — backward-compatible with existing FMSPath input.
/// Subscribes to FMSPath on ~/input/fms_path and optionally JSON on ~/input/fms_json.
/// Converts FMSPath → FMSRouteWindow internally (single segment, maps waypoint types).
/// Uplink methods are no-ops (legacy mode has no FMS uplink reporting).
class LegacyProtocol : public FMSProtocol
{
public:
  void initialize(rclcpp::Node & node, const ProtocolCallbacks & cb) override;
  std::string name() const override { return "legacy"; }

  // Uplink: no-ops for legacy mode
  void reportVehicleStatus(const igv_planning_msgs::msg::VehicleStatus &) override {}
  void reportTrajectory(const igv_planning_msgs::msg::TrajectoryReport &) override {}
  void reportJobEvent(const igv_planning_msgs::msg::JobEvent &) override {}
  void requestDetour(const igv_planning_msgs::msg::DetourRequest &) override {}

private:
  /// Convert an FMSPath message to FMSRouteWindow for the unified pipeline
  igv_planning_msgs::msg::FMSRouteWindow convertFMSPathToRouteWindow(
    const igv_planning_msgs::msg::FMSPath & fms_path, rclcpp::Node & node) const;

  // Subscriptions
  rclcpp::Subscription<igv_planning_msgs::msg::FMSPath>::SharedPtr fms_path_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fms_json_sub_;

  // Node pointer for JSON parsing (timestamp, logger)
  rclcpp::Node * node_{nullptr};
  ProtocolCallbacks callbacks_;
};

}  // namespace igv_path_planner
#endif  // FMS_TRAJECTORY_BRIDGE__PROTOCOLS__LEGACY_PROTOCOL_HPP_
