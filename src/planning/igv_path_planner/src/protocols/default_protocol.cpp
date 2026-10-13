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

#include "igv_path_planner/protocols/default_protocol.hpp"

namespace igv_path_planner
{

void DefaultProtocol::initialize(rclcpp::Node & node, const ProtocolCallbacks & cb)
{
  // Read topic names from parameters with defaults
  auto route_topic = node.declare_parameter<std::string>(
    "fms_protocol.default.route_topic", "~/input/route_window");
  auto stop_topic = node.declare_parameter<std::string>(
    "fms_protocol.default.stop_topic", "~/input/stop_point");

  // Downlink subscriptions
  route_sub_ = node.create_subscription<igv_planning_msgs::msg::FMSRouteWindow>(
    route_topic, rclcpp::QoS(1),
    [cb](const igv_planning_msgs::msg::FMSRouteWindow::SharedPtr msg) {
      if (cb.on_route_window) cb.on_route_window(msg);
    });

  stop_sub_ = node.create_subscription<igv_planning_msgs::msg::FMSStopPoint>(
    stop_topic, rclcpp::QoS(1),
    [cb](const igv_planning_msgs::msg::FMSStopPoint::SharedPtr msg) {
      if (cb.on_stop_point) cb.on_stop_point(msg);
    });

  // Uplink publishers
  status_pub_ = node.create_publisher<igv_planning_msgs::msg::VehicleStatus>(
    "~/output/vehicle_status", rclcpp::QoS(1));
  trajectory_pub_ = node.create_publisher<igv_planning_msgs::msg::TrajectoryReport>(
    "~/output/trajectory_report", rclcpp::QoS(1));
  job_event_pub_ = node.create_publisher<igv_planning_msgs::msg::JobEvent>(
    "~/output/job_event", rclcpp::QoS(1));
  detour_pub_ = node.create_publisher<igv_planning_msgs::msg::DetourRequest>(
    "~/output/detour_request", rclcpp::QoS(1));

  RCLCPP_INFO(node.get_logger(), "Default protocol initialized (route: %s, stop: %s)",
    route_topic.c_str(), stop_topic.c_str());
}

void DefaultProtocol::reportVehicleStatus(const igv_planning_msgs::msg::VehicleStatus & status)
{
  status_pub_->publish(status);
}

void DefaultProtocol::reportTrajectory(const igv_planning_msgs::msg::TrajectoryReport & report)
{
  trajectory_pub_->publish(report);
}

void DefaultProtocol::reportJobEvent(const igv_planning_msgs::msg::JobEvent & event)
{
  job_event_pub_->publish(event);
}

void DefaultProtocol::requestDetour(const igv_planning_msgs::msg::DetourRequest & request)
{
  detour_pub_->publish(request);
}

REGISTER_FMS_PROTOCOL(DefaultProtocol, "default");

}  // namespace igv_path_planner
