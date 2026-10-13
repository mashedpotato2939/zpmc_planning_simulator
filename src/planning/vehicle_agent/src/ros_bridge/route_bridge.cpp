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

#include "vehicle_agent/ros_bridge/route_bridge.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

namespace vehicle_agent
{

namespace
{
using IrMission = igv_planning_msgs::msg::IrMission;
using IrCommand = igv_planning_msgs::msg::IrCommand;
using IrWaypoint = igv_planning_msgs::msg::IrWaypoint;
using FMSRouteWindow = igv_planning_msgs::msg::FMSRouteWindow;
using FMSPathSegment = igv_planning_msgs::msg::FMSPathSegment;
using SegmentWaypoint = igv_planning_msgs::msg::SegmentWaypoint;
using FMSStopPoint = igv_planning_msgs::msg::FMSStopPoint;

// Parse the codec's segment_id convention into a sort key + orientation. The
// codec emits "segN:fwd" or "segN:rev"; anything else (legacy producers,
// hand-rolled scenarios) collapses to a single forward segment.
struct SegKey
{
  int index{0};        // sortable; -1 if the prefix wasn't "segN"
  uint8_t orientation{FMSPathSegment::ORIENT_FORWARD};
  bool well_formed{false};
};

SegKey parseSegmentId(const std::string & s)
{
  SegKey k;
  // Expect "seg<digits>(:fwd|:rev)?"
  if (s.size() < 4 || s.compare(0, 3, "seg") != 0) return k;
  std::size_t colon = s.find(':', 3);
  const std::string digits = (colon == std::string::npos) ? s.substr(3)
                                                          : s.substr(3, colon - 3);
  if (digits.empty() || !std::all_of(digits.begin(), digits.end(),
                                     [](char c) { return c >= '0' && c <= '9'; })) {
    return k;
  }
  k.index = std::stoi(digits);
  if (colon != std::string::npos) {
    const auto suffix = s.substr(colon + 1);
    if (suffix == "rev") k.orientation = FMSPathSegment::ORIENT_REVERSE;
    else if (suffix == "fwd") k.orientation = FMSPathSegment::ORIENT_FORWARD;
  }
  k.well_formed = true;
  return k;
}
}  // namespace

FMSRouteWindow RouteBridgeNode::translateMission(
  const IrMission & m, const std::string & vehicle_id, const rclcpp::Time & stamp)
{
  FMSRouteWindow out;
  out.header.stamp = stamp;
  out.header.frame_id = "map";
  out.vehicle_id = vehicle_id;
  out.route_id = m.id;
  out.job_id = "";
  out.job_task_id = "";
  out.request_id = "";

  switch (m.kind) {
    case IrMission::KIND_ABORT:
    case IrMission::KIND_RESUME:
      out.status = FMSRouteWindow::STATUS_CANCELLED;
      return out;
    case IrMission::KIND_ROUTE:
    case IrMission::KIND_STOP_AT:
    case IrMission::KIND_PARK:
    default:
      out.status = FMSRouteWindow::STATUS_ACTIVE;
      break;
  }

  // Walk waypoints in order, grouping consecutive runs with the same
  // (segment_index, orientation). The codec emits points already grouped by
  // segment, so a single linear pass is correct; we only rely on segment_id
  // as the boundary marker, not for sort order.
  FMSPathSegment * current_seg = nullptr;
  int current_index = -2;            // sentinel: not yet started
  uint8_t current_orient = FMSPathSegment::ORIENT_FORWARD;
  for (const auto & w : m.waypoints) {
    const auto key = parseSegmentId(w.segment_id);
    const int idx = key.well_formed ? key.index : 0;
    const uint8_t orient = key.well_formed ? key.orientation
                                           : FMSPathSegment::ORIENT_FORWARD;

    if (!current_seg || idx != current_index || orient != current_orient) {
      out.segments.emplace_back();
      current_seg = &out.segments.back();
      current_seg->orientation = orient;
      current_index = idx;
      current_orient = orient;
    }

    SegmentWaypoint sw;
    sw.x = w.x;
    sw.y = w.y;
    sw.z = w.z;
    sw.heading = w.yaw;
    sw.speed_limit = w.target_speed;
    sw.point_type = w.point_type;
    current_seg->points.push_back(sw);
  }

  // Safety net for producers that don't mark a destination: only auto-tag the
  // last point if nothing in the route already carries POINT_DESTINATION. The
  // explicit marker wins so LOCAL_PLAN_* zones and operator-set destinations
  // survive untouched.
  bool has_destination = false;
  for (const auto & seg : out.segments) {
    for (const auto & pt : seg.points) {
      if (pt.point_type == SegmentWaypoint::POINT_DESTINATION) {
        has_destination = true;
        break;
      }
    }
    if (has_destination) break;
  }
  if (!has_destination && !out.segments.empty() && !out.segments.back().points.empty()) {
    out.segments.back().points.back().point_type = SegmentWaypoint::POINT_DESTINATION;
  }
  return out;
}

std::optional<FMSStopPoint> RouteBridgeNode::translateCommand(
  const IrCommand & c, const std::string & vehicle_id,
  double ego_x, double ego_y, bool ego_received, const rclcpp::Time & stamp)
{
  FMSStopPoint sp;
  sp.header.stamp = stamp;
  sp.header.frame_id = "map";
  sp.vehicle_id = vehicle_id;
  switch (c.kind) {
    case IrCommand::KIND_PAUSE:
    case IrCommand::KIND_ABORT:
      sp.clear = false;
      sp.x = ego_received ? ego_x : 0.0;
      sp.y = ego_received ? ego_y : 0.0;
      return sp;
    case IrCommand::KIND_RESUME:
      sp.clear = true;
      sp.x = 0.0;
      sp.y = 0.0;
      return sp;
    case IrCommand::KIND_ESTOP:
    case IrCommand::KIND_CLEAR_FAULT:
    default:
      return std::nullopt;
  }
}

RouteBridgeNode::RouteBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("va_route_bridge", options)
{
  vehicle_id_ = declare_parameter<std::string>("vehicle_id", "IGV2001");
  const auto mission_topic = declare_parameter<std::string>(
    "mission_topic", "/vehicle_agent/mission");
  const auto command_topic = declare_parameter<std::string>(
    "command_topic", "/vehicle_agent/command");
  const auto odom_topic = declare_parameter<std::string>(
    "odometry_topic", "/localization/kinematic_state");
  const auto route_topic = declare_parameter<std::string>(
    "route_window_topic", "/fms/route_window");
  const auto stop_topic = declare_parameter<std::string>(
    "stop_point_topic", "/fms/stop_point");
  window_ahead_m_ = declare_parameter<double>("window_ahead_m", 100.0);
  window_lookback_m_ = declare_parameter<double>("window_lookback_m", 10.0);
  window_rate_hz_ = declare_parameter<double>("window_rate_hz", 1.0);
  repartition_step_m_ = declare_parameter<double>("repartition_step_m", 50.0);

  pub_route_ = create_publisher<FMSRouteWindow>(route_topic, rclcpp::QoS(10));
  pub_stop_ = create_publisher<FMSStopPoint>(stop_topic, rclcpp::QoS(10));

  sub_mission_ = create_subscription<IrMission>(
    mission_topic, rclcpp::QoS(10),
    std::bind(&RouteBridgeNode::onMission, this, std::placeholders::_1));
  sub_command_ = create_subscription<IrCommand>(
    command_topic, rclcpp::QoS(10),
    std::bind(&RouteBridgeNode::onCommand, this, std::placeholders::_1));
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS(),
    std::bind(&RouteBridgeNode::onOdometry, this, std::placeholders::_1));

  const double rate = (window_rate_hz_ > 0.0) ? window_rate_hz_ : 1.0;
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / rate));
  window_timer_ = create_wall_timer(
    period, std::bind(&RouteBridgeNode::onWindowTimer, this));

  RCLCPP_INFO(get_logger(),
    "va_route_bridge up: %s,%s → %s,%s (vehicle=%s, full-route publish @ %.1fHz)",
    mission_topic.c_str(), command_topic.c_str(),
    route_topic.c_str(), stop_topic.c_str(), vehicle_id_.c_str(), rate);
}

void RouteBridgeNode::onMission(IrMission::ConstSharedPtr msg)
{
  // KIND_ABORT short-circuits the latch and emits a single STATUS_CANCELLED
  // so the converter releases its cached path immediately, without waiting
  // for the next window tick.
  if (msg->kind == IrMission::KIND_ABORT) {
    {
      std::lock_guard<std::mutex> lk(route_mtx_);
      latched_.reset();
      latched_seq_ = 0;
    }
    pub_route_->publish(translateMission(*msg, vehicle_id_, now()));
    return;
  }

  bool is_new_base = false;
  {
    std::lock_guard<std::mutex> lk(route_mtx_);
    // Out-of-order guard: only meaningful when the producer sets a non-zero
    // seq (ros_injector and scenario_loopback do; ahtfms_ns decoder leaves
    // seq=0 today, which simply disables this check).
    if (msg->seq != 0 && latched_ && msg->seq < latched_seq_) {
      return;
    }
    // A different base route_id (or first mission) restarts partition numbering;
    // a same-id refresh keeps the rolling partition index intact.
    is_new_base = (!latched_ || latched_->id != msg->id);
    latched_ = msg;
    latched_seq_ = msg->seq;
  }

  // Restart short-route partitioning at the next window for a genuinely new route.
  if (is_new_base) {
    std::lock_guard<std::mutex> lk(ego_mtx_);
    partition_anchor_set_ = false;
    partition_index_ = 0;
  }

  // Fire one window immediately so a fresh mission becomes visible downstream
  // without waiting up to one timer period.
  onWindowTimer();
}

void RouteBridgeNode::onWindowTimer()
{
  IrMission::ConstSharedPtr m;
  {
    std::lock_guard<std::mutex> lk(route_mtx_);
    m = latched_;
  }
  if (!m) return;

  auto full = translateMission(*m, vehicle_id_, now());
  pub_route_->publish(full);
}

void RouteBridgeNode::onCommand(IrCommand::ConstSharedPtr msg)
{
  double x = 0.0, y = 0.0;
  bool have_ego = false;
  {
    std::lock_guard<std::mutex> lk(ego_mtx_);
    x = ego_x_;
    y = ego_y_;
    have_ego = ego_received_;
  }
  auto out = translateCommand(*msg, vehicle_id_, x, y, have_ego, now());
  if (!out) {
    // ESTOP and CLEAR_FAULT never go through the converter; the FSM and
    // EstopChannel own those flows. Logging at INFO so it's auditable but
    // not noisy on every estop.
    RCLCPP_INFO(get_logger(),
      "command kind=%u for mission=%s not routed to /fms/stop_point",
      msg->kind, msg->mission_id.c_str());
    return;
  }
  if (msg->kind == IrCommand::KIND_ABORT) {
    RCLCPP_WARN(get_logger(),
      "ABORT for mission=%s emitted as best-effort stop_point — converter "
      "cannot distinguish from PAUSE", msg->mission_id.c_str());
  }
  if ((msg->kind == IrCommand::KIND_PAUSE || msg->kind == IrCommand::KIND_ABORT)
      && !have_ego) {
    RCLCPP_WARN(get_logger(),
      "no ego pose latched yet — stop_point published with (0,0)");
  }
  pub_stop_->publish(*out);
}

void RouteBridgeNode::onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lk(ego_mtx_);
  ego_x_ = msg->pose.pose.position.x;
  ego_y_ = msg->pose.pose.position.y;
  ego_received_ = true;
}

FMSRouteWindow RouteBridgeNode::sliceAroundEgo(
  const FMSRouteWindow & full, double ego_x, double ego_y,
  bool ego_received, double behind_m, double ahead_m)
{
  // Pre-localization: emit the full polyline so the converter sees the route
  // at least once even before /localization/kinematic_state lands.
  if (!ego_received) return full;

  // Flatten segments into a parallel index so the windowing math is a simple
  // 1D walk. We remember which (segment, orientation) each waypoint came from
  // so we can re-split into FMSPathSegments at the end without losing
  // boundaries.
  struct Flat
  {
    SegmentWaypoint pt;
    std::size_t seg_index;
    uint8_t orientation;
  };
  std::vector<Flat> flat;
  std::size_t total = 0;
  for (const auto & seg : full.segments) total += seg.points.size();
  flat.reserve(total);
  for (std::size_t s = 0; s < full.segments.size(); ++s) {
    const auto & seg = full.segments[s];
    for (const auto & pt : seg.points) {
      flat.push_back({pt, s, seg.orientation});
    }
  }
  if (flat.empty()) return full;

  // Nearest-waypoint search (Euclidean, squared — sqrt unnecessary).
  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < flat.size(); ++i) {
    const double dx = flat[i].pt.x - ego_x;
    const double dy = flat[i].pt.y - ego_y;
    const double d = dx * dx + dy * dy;
    if (d < best) { best = d; nearest = i; }
  }

  // Backward arc-length walk.
  std::size_t win_start = nearest;
  {
    double acc = 0.0;
    for (std::size_t i = nearest; i > 0; --i) {
      const double dx = flat[i].pt.x - flat[i - 1].pt.x;
      const double dy = flat[i].pt.y - flat[i - 1].pt.y;
      acc += std::sqrt(dx * dx + dy * dy);
      if (acc > behind_m) break;
      win_start = i - 1;
    }
  }

  // Forward arc-length walk.
  std::size_t win_end = nearest;
  {
    double acc = 0.0;
    for (std::size_t i = nearest; i + 1 < flat.size(); ++i) {
      const double dx = flat[i + 1].pt.x - flat[i].pt.x;
      const double dy = flat[i + 1].pt.y - flat[i].pt.y;
      acc += std::sqrt(dx * dx + dy * dy);
      win_end = i + 1;
      if (acc > ahead_m) break;
    }
  }

  // LC pair integrity — don't ship a window that opens or closes mid-zone.
  // If win_start is inside an LC zone (a prior LC_START with no matching
  // LC_END before it), extend backward to that LC_START.
  bool inside_lc = false;
  for (std::size_t i = 0; i < win_start; ++i) {
    if (flat[i].pt.point_type == SegmentWaypoint::POINT_LC_START) inside_lc = true;
    else if (flat[i].pt.point_type == SegmentWaypoint::POINT_LC_END) inside_lc = false;
  }
  if (inside_lc) {
    for (std::size_t i = win_start; i > 0; --i) {
      if (flat[i - 1].pt.point_type == SegmentWaypoint::POINT_LC_START) {
        win_start = i - 1;
        break;
      }
    }
  }
  // If the window contains an unmatched LC_START, extend forward to its LC_END.
  inside_lc = false;
  for (std::size_t i = win_start; i <= win_end; ++i) {
    if (flat[i].pt.point_type == SegmentWaypoint::POINT_LC_START) inside_lc = true;
    else if (flat[i].pt.point_type == SegmentWaypoint::POINT_LC_END) inside_lc = false;
  }
  if (inside_lc) {
    for (std::size_t i = win_end + 1; i < flat.size(); ++i) {
      win_end = i;
      if (flat[i].pt.point_type == SegmentWaypoint::POINT_LC_END) break;
    }
  }

  // ≥1 preceding waypoint guard: a single-point window collapses the path
  // generator to a stop. Always include one extra point behind if available.
  if (win_start == win_end && win_start > 0) win_start = win_start - 1;

  // Re-emit FMSPathSegments, opening a new one whenever (seg_index, orientation)
  // changes — same grouping rule translateMission() applies on the input side.
  // The boundary waypoint is pushed to both segments (matches the rviz panel's
  // redundant-boundary convention) so the downstream path generator can build
  // a continuous yaw across orientation flips.
  FMSRouteWindow out;
  out.header = full.header;
  out.vehicle_id = full.vehicle_id;
  out.route_id = full.route_id;
  out.job_id = full.job_id;
  out.job_task_id = full.job_task_id;
  out.request_id = full.request_id;
  out.status = full.status;

  FMSPathSegment cur;
  cur.orientation = flat[win_start].orientation;
  std::size_t cur_seg_idx = flat[win_start].seg_index;
  for (std::size_t i = win_start; i <= win_end; ++i) {
    const bool boundary = (flat[i].seg_index != cur_seg_idx) ||
                          (flat[i].orientation != cur.orientation);
    if (boundary) {
      cur.points.push_back(flat[i].pt);    // boundary point also closes old seg
      out.segments.push_back(std::move(cur));
      cur = FMSPathSegment{};
      cur.orientation = flat[i].orientation;
      cur_seg_idx = flat[i].seg_index;
    }
    cur.points.push_back(flat[i].pt);
  }
  if (!cur.points.empty()) out.segments.push_back(std::move(cur));
  return out;
}

}  // namespace vehicle_agent

RCLCPP_COMPONENTS_REGISTER_NODE(vehicle_agent::RouteBridgeNode)
