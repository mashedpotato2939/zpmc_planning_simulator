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

#ifndef FMS_TRAJECTORY_BRIDGE__FMS_PROTOCOL_HPP_
#define FMS_TRAJECTORY_BRIDGE__FMS_PROTOCOL_HPP_

#include <rclcpp/rclcpp.hpp>
#include "igv_planning_msgs/msg/fms_route_window.hpp"
#include "igv_planning_msgs/msg/fms_stop_point.hpp"
#include "igv_planning_msgs/msg/vehicle_status.hpp"
#include "igv_planning_msgs/msg/trajectory_report.hpp"
#include "igv_planning_msgs/msg/job_event.hpp"
#include "igv_planning_msgs/msg/detour_request.hpp"
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace igv_path_planner
{

/// Callbacks the protocol plugin uses to deliver parsed DOWNLINK data to the core node
struct ProtocolCallbacks
{
  std::function<void(const igv_planning_msgs::msg::FMSRouteWindow::SharedPtr)> on_route_window;
  std::function<void(const igv_planning_msgs::msg::FMSStopPoint::SharedPtr)> on_stop_point;
};

/// Base class for FMS protocol plugins (handles both downlink and uplink)
class FMSProtocol
{
public:
  virtual ~FMSProtocol() = default;

  /// Initialize subscriptions (downlink) and publishers (uplink)
  virtual void initialize(
    rclcpp::Node & node,
    const ProtocolCallbacks & callbacks) = 0;

  /// Human-readable protocol name for logging
  virtual std::string name() const = 0;

  // --- Uplink methods (core node calls these to report back to FMS) ---

  /// Report vehicle status (ICD §1.1, called at 1Hz by timer)
  virtual void reportVehicleStatus(const igv_planning_msgs::msg::VehicleStatus & status) = 0;

  /// Report planned trajectory (ICD §1.2, called at 1Hz by timer)
  virtual void reportTrajectory(const igv_planning_msgs::msg::TrajectoryReport & report) = 0;

  /// Report job event (ICD §1.5, called on arrival at destination)
  virtual void reportJobEvent(const igv_planning_msgs::msg::JobEvent & event) = 0;

  /// Request detour (ICD §1.6, called when path is blocked)
  virtual void requestDetour(const igv_planning_msgs::msg::DetourRequest & request) = 0;
};

/// Factory + registry for FMS protocol plugins
class FMSProtocolFactory
{
public:
  using Creator = std::function<std::unique_ptr<FMSProtocol>()>;

  static FMSProtocolFactory & instance();
  void registerProtocol(const std::string & name, Creator creator);
  std::unique_ptr<FMSProtocol> create(const std::string & name) const;
  std::vector<std::string> availableProtocols() const;

private:
  std::unordered_map<std::string, Creator> creators_;
};

#define REGISTER_FMS_PROTOCOL(ClassName, ProtocolName) \
  namespace { \
  static bool _reg_##ClassName = []() { \
    FMSProtocolFactory::instance().registerProtocol( \
      ProtocolName, []() { return std::make_unique<ClassName>(); }); \
    return true; \
  }(); \
  }

}  // namespace igv_path_planner
#endif  // FMS_TRAJECTORY_BRIDGE__FMS_PROTOCOL_HPP_
