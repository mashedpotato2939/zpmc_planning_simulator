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

#ifndef VEHICLE_AGENT__ROS_BRIDGE__ROUTE_BRIDGE_HPP_
#define VEHICLE_AGENT__ROS_BRIDGE__ROUTE_BRIDGE_HPP_

#include <igv_planning_msgs/msg/fms_route_window.hpp>
#include <igv_planning_msgs/msg/fms_stop_point.hpp>
#include <igv_planning_msgs/msg/ir_command.hpp>
#include <igv_planning_msgs/msg/ir_mission.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <optional>
#include <string>

namespace vehicle_agent
{

// Translates vehicle_agent's IR-aligned ROS topics into the legacy FMS topic
// shapes that fms_path_converter consumes. Latches the last IrMission and
// republishes an ego-centric windowed FMSRouteWindow at a fixed cadence so
// downstream liveness is decoupled from upstream FMS push cadence — a single
// FMS push gets a steady stream of sliding windows for free, and same-id
// re-pushes (ICD §8.63 RouteNotify) seamlessly update the latched geometry.
//
// Subscriptions:
//   /vehicle_agent/mission           IrMission         (latched)
//   /vehicle_agent/command           IrCommand         → /fms/stop_point  (PAUSE/RESUME)
//   /localization/kinematic_state    Odometry          (latched for PAUSE x/y and windowing)
//
// Publications:
//   /fms/route_window                FMSRouteWindow    ego-centric window @ window_rate_hz
//   /fms/stop_point                  FMSStopPoint      on command
//
// Mapping tables live in doc/plan-rviz-va-integration.md §2.2-§2.3.
class RouteBridgeNode : public rclcpp::Node
{
public:
  explicit RouteBridgeNode(const rclcpp::NodeOptions & options);

  // Translator entry points exposed for unit testing — pure functions modulo
  // the latched ego pose passed in.
  static igv_planning_msgs::msg::FMSRouteWindow translateMission(
    const igv_planning_msgs::msg::IrMission & m,
    const std::string & vehicle_id,
    const rclcpp::Time & stamp);

  // Translate an IrCommand to an FMSStopPoint. Returns nullopt for ESTOP and
  // CLEAR_FAULT (which the converter has no concept for). PAUSE uses the
  // supplied ego pose; RESUME emits clear=true with zeroed x/y; ABORT is a
  // best-effort stop with clear=false (the converter cannot distinguish it
  // from PAUSE, so we log a WARN at the live call site).
  static std::optional<igv_planning_msgs::msg::FMSStopPoint> translateCommand(
    const igv_planning_msgs::msg::IrCommand & c,
    const std::string & vehicle_id,
    double ego_x,
    double ego_y,
    bool ego_received,
    const rclcpp::Time & stamp);

  // Legacy helper: crop a full FMSRouteWindow to an ego-centric slice: ~behind_m before the
  // nearest waypoint, ~ahead_m past it, with LC pair integrity preserved and a
  // ≥1-preceding-waypoint guard so the downstream path generator never sees a
  // degenerate single-point window. Segment boundaries (orientation flips,
  // segment_index breaks) are preserved by re-emitting FMSPathSegment splits at
  // the same transitions as the input. When ego_received is false, returns
  // `full` unchanged — safe pre-localization fallback so the first publish
  // carries the entire polyline.
  static igv_planning_msgs::msg::FMSRouteWindow sliceAroundEgo(
    const igv_planning_msgs::msg::FMSRouteWindow & full,
    double ego_x,
    double ego_y,
    bool ego_received,
    double behind_m,
    double ahead_m);

private:
  void onMission(igv_planning_msgs::msg::IrMission::ConstSharedPtr msg);
  void onCommand(igv_planning_msgs::msg::IrCommand::ConstSharedPtr msg);
  void onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onWindowTimer();

  std::string vehicle_id_;
  // Deprecated launch-compatible knobs. The bridge now publishes the full route;
  // igv_path_converter owns segment selection and local path truncation.
  double window_ahead_m_{100.0};
  double window_lookback_m_{10.0};
  double window_rate_hz_{1.0};
  double repartition_step_m_{50.0};

  rclcpp::Subscription<igv_planning_msgs::msg::IrMission>::SharedPtr sub_mission_;
  rclcpp::Subscription<igv_planning_msgs::msg::IrCommand>::SharedPtr sub_command_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  rclcpp::Publisher<igv_planning_msgs::msg::FMSRouteWindow>::SharedPtr pub_route_;
  rclcpp::Publisher<igv_planning_msgs::msg::FMSStopPoint>::SharedPtr pub_stop_;

  rclcpp::TimerBase::SharedPtr window_timer_;

  mutable std::mutex ego_mtx_;
  double ego_x_{0.0};
  double ego_y_{0.0};
  bool ego_received_{false};
  // Deprecated repartition state kept only to avoid churn in tests/docs that
  // still exercise the legacy slice helper.
  double partition_anchor_x_{0.0};
  double partition_anchor_y_{0.0};
  bool partition_anchor_set_{false};
  uint64_t partition_index_{0};

  mutable std::mutex route_mtx_;
  igv_planning_msgs::msg::IrMission::ConstSharedPtr latched_;
  uint64_t latched_seq_{0};
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__ROS_BRIDGE__ROUTE_BRIDGE_HPP_
