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

#include "igv_path_planner/protocols/legacy_protocol.hpp"
#include <nlohmann/json.hpp>

namespace igv_path_planner
{

void LegacyProtocol::initialize(rclcpp::Node & node, const ProtocolCallbacks & cb)
{
  node_ = &node;
  callbacks_ = cb;

  // Read topic names from parameters with defaults
  auto fms_path_topic = node.declare_parameter<std::string>(
    "fms_protocol.legacy.fms_path_topic", "~/input/fms_path");
  auto use_json = node.declare_parameter<bool>(
    "fms_protocol.legacy.use_json_input", false);
  auto fms_json_topic = node.declare_parameter<std::string>(
    "fms_protocol.legacy.fms_json_topic", "~/input/fms_json");

  // Subscribe to FMSPath messages
  fms_path_sub_ = node.create_subscription<igv_planning_msgs::msg::FMSPath>(
    fms_path_topic, rclcpp::QoS(10),
    [this, &node](const igv_planning_msgs::msg::FMSPath::SharedPtr fms_msg) {
      auto window = std::make_shared<igv_planning_msgs::msg::FMSRouteWindow>(
        convertFMSPathToRouteWindow(*fms_msg, node));
      if (callbacks_.on_route_window) {
        callbacks_.on_route_window(window);
      }
    });

  // Optionally subscribe to JSON input
  if (use_json) {
    fms_json_sub_ = node.create_subscription<std_msgs::msg::String>(
      fms_json_topic, rclcpp::QoS(10),
      [this, &node](const std_msgs::msg::String::SharedPtr json_msg) {
        try {
          auto json = nlohmann::json::parse(json_msg->data);

          igv_planning_msgs::msg::FMSPath fms_path;
          fms_path.header.stamp = node.now();
          fms_path.header.frame_id = "map";

          if (json.contains("path_id")) {
            fms_path.path_id = json["path_id"].get<std::string>();
          }
          if (json.contains("task_id")) {
            fms_path.task_id = json["task_id"].get<std::string>();
          }

          if (json.contains("waypoints") && json["waypoints"].is_array()) {
            for (const auto & wp : json["waypoints"]) {
              igv_planning_msgs::msg::FMSWaypoint waypoint;
              waypoint.x = wp.value("x", 0.0);
              waypoint.y = wp.value("y", 0.0);
              waypoint.z = wp.value("z", 0.0);
              waypoint.yaw = wp.value("yaw", 0.0);
              waypoint.velocity = wp.value("velocity", -1.0);
              waypoint.acceleration = wp.value("acceleration", 0.0);
              waypoint.curvature = wp.value("curvature", 0.0);
              waypoint.time_from_start = wp.value("time_from_start", 0.0);
              waypoint.waypoint_type = wp.value("type", 0);
              waypoint.lane_id = wp.value("lane_id", 0);
              fms_path.waypoints.push_back(waypoint);
            }
          }

          if (json.contains("left_bound") && json["left_bound"].is_array()) {
            for (const auto & pt : json["left_bound"]) {
              geometry_msgs::msg::Point point;
              point.x = pt.value("x", 0.0);
              point.y = pt.value("y", 0.0);
              point.z = pt.value("z", 0.0);
              fms_path.left_bound.push_back(point);
            }
          }
          if (json.contains("right_bound") && json["right_bound"].is_array()) {
            for (const auto & pt : json["right_bound"]) {
              geometry_msgs::msg::Point point;
              point.x = pt.value("x", 0.0);
              point.y = pt.value("y", 0.0);
              point.z = pt.value("z", 0.0);
              fms_path.right_bound.push_back(point);
            }
          }

          auto window = std::make_shared<igv_planning_msgs::msg::FMSRouteWindow>(
            convertFMSPathToRouteWindow(fms_path, node));
          if (callbacks_.on_route_window) {
            callbacks_.on_route_window(window);
          }

        } catch (const nlohmann::json::exception & e) {
          RCLCPP_ERROR(node.get_logger(), "Legacy protocol: JSON parse error: %s", e.what());
        }
      });
  }

  RCLCPP_INFO(node.get_logger(), "Legacy protocol initialized (fms_path: %s, json: %s)",
    fms_path_topic.c_str(), use_json ? fms_json_topic.c_str() : "disabled");
}

igv_planning_msgs::msg::FMSRouteWindow LegacyProtocol::convertFMSPathToRouteWindow(
  const igv_planning_msgs::msg::FMSPath & fms_path, rclcpp::Node & node) const
{
  igv_planning_msgs::msg::FMSRouteWindow window;
  window.header = fms_path.header;
  window.route_id = fms_path.path_id;
  window.job_task_id = fms_path.task_id;
  window.status = igv_planning_msgs::msg::FMSRouteWindow::STATUS_ACTIVE;

  if (fms_path.waypoints.empty()) {
    return window;
  }

  // Detect overall reverse direction from velocity signs or is_reverse flags
  bool is_reverse = false;
  int reverse_count = 0;
  int negative_vel_count = 0;
  for (const auto & wp : fms_path.waypoints) {
    if (wp.is_reverse) reverse_count++;
    if (wp.velocity < -0.1) negative_vel_count++;
  }
  if (reverse_count > static_cast<int>(fms_path.waypoints.size() / 2) ||
      negative_vel_count > static_cast<int>(fms_path.waypoints.size() / 2)) {
    is_reverse = true;
  }

  // Create a single segment containing all waypoints
  igv_planning_msgs::msg::FMSPathSegment segment;
  segment.orientation = is_reverse
    ? igv_planning_msgs::msg::FMSPathSegment::ORIENT_REVERSE
    : igv_planning_msgs::msg::FMSPathSegment::ORIENT_FORWARD;

  for (const auto & wp : fms_path.waypoints) {
    igv_planning_msgs::msg::SegmentWaypoint swp;
    swp.x = wp.x;
    swp.y = wp.y;
    swp.z = wp.z;
    swp.heading = wp.yaw;
    swp.speed_limit = std::abs(wp.velocity);  // SegmentWaypoint uses unsigned speed_limit

    // Map FMSWaypoint.waypoint_type → SegmentWaypoint.point_type
    switch (wp.waypoint_type) {
      case igv_planning_msgs::msg::FMSWaypoint::TYPE_STOP:
      case igv_planning_msgs::msg::FMSWaypoint::TYPE_GOAL:
        swp.point_type = igv_planning_msgs::msg::SegmentWaypoint::POINT_DESTINATION;
        break;
      default:
        // Map lane_change_type markers
        if (wp.lane_change_type == 1 || wp.lane_change_type == 2) {
          swp.point_type = wp.is_lane_change_start
            ? igv_planning_msgs::msg::SegmentWaypoint::POINT_LC_START
            : (wp.is_lane_change_end
              ? igv_planning_msgs::msg::SegmentWaypoint::POINT_LC_END
              : igv_planning_msgs::msg::SegmentWaypoint::POINT_NORMAL);
        } else {
          swp.point_type = igv_planning_msgs::msg::SegmentWaypoint::POINT_NORMAL;
        }
        break;
    }

    segment.points.push_back(swp);
  }

  window.segments.push_back(segment);

  RCLCPP_DEBUG(node.get_logger(),
    "Legacy: converted FMSPath (%zu waypoints) → FMSRouteWindow (1 segment, %s)",
    fms_path.waypoints.size(), is_reverse ? "reverse" : "forward");

  return window;
}

REGISTER_FMS_PROTOCOL(LegacyProtocol, "legacy");

}  // namespace igv_path_planner
