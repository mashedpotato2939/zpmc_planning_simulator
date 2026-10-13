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

#include "igv_velocity_planner/detected_objects_grid_node.hpp"

#include "igv_velocity_planner/detected_objects_grid.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cstdint>

namespace igv_velocity_planner
{

namespace dog = detected_objects_grid;

DetectedObjectsToGridNode::DetectedObjectsToGridNode(const rclcpp::NodeOptions & options)
: Node("detected_objects_to_grid", options)
{
  enable_ = declare_parameter<bool>("enable", enable_);
  target_frame_ = declare_parameter<std::string>("target_frame", target_frame_);
  resolution_ = declare_parameter<double>("resolution", resolution_);
  grid_length_m_ = declare_parameter<double>("grid_length_m", grid_length_m_);
  hold_time_s_ = declare_parameter<double>("hold_time_s", hold_time_s_);
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", publish_rate_hz_);
  occupancy_value_ = declare_parameter<int>("occupancy_value", occupancy_value_);
  lateral_margin_ = declare_parameter<double>("lateral_margin", lateral_margin_);
  min_existence_probability_ =
    declare_parameter<double>("min_existence_probability", min_existence_probability_);

  // Inflate every footprint by the ego half-width (+ margin): the consumer
  // (fms_velocity_shaper.occupancy_grid_check) samples only the path centerline
  // cell, so without inflation a laterally-offset obstacle would be missed.
  const auto vehicle_info =
    autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  const double ego_half_width = 0.5 * vehicle_info.vehicle_width_m;
  inflation_ = ego_half_width + lateral_margin_;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  sub_detected_objects_ =
    create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
      "~/input/detected_objects", rclcpp::QoS(10),
      std::bind(&DetectedObjectsToGridNode::onDetectedObjects, this, std::placeholders::_1));
  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", rclcpp::QoS(10),
    std::bind(&DetectedObjectsToGridNode::onOdometry, this, std::placeholders::_1));

  pub_grid_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "~/output/occupancy_grid", rclcpp::QoS(1).reliable());

  const double rate = (publish_rate_hz_ > 0.0) ? publish_rate_hz_ : 10.0;
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate),
    std::bind(&DetectedObjectsToGridNode::onTimer, this));

  RCLCPP_INFO(get_logger(), "Detected Objects -> Grid initialized");
  RCLCPP_INFO(get_logger(), "  enable: %s", enable_ ? "true" : "false");
  RCLCPP_INFO(
    get_logger(), "  resolution: %.2f  grid_length_m: %.1f", resolution_, grid_length_m_);
  RCLCPP_INFO(
    get_logger(), "  hold_time_s: %.2f  publish_rate_hz: %.1f", hold_time_s_, publish_rate_hz_);
  RCLCPP_INFO(
    get_logger(), "  inflation: %.2f (ego_half_width %.2f + margin %.2f)", inflation_,
    ego_half_width, lateral_margin_);
  RCLCPP_INFO(get_logger(), "  target_frame: %s", target_frame_.c_str());
}

void DetectedObjectsToGridNode::onDetectedObjects(
  autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg)
{
  if (!enable_) return;
  recent_.emplace_back(this->now(), msg);
}

void DetectedObjectsToGridNode::onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  ego_x_ = msg->pose.pose.position.x;
  ego_y_ = msg->pose.pose.position.y;
  have_ego_ = true;
}

void DetectedObjectsToGridNode::onTimer()
{
  if (!enable_) return;

  // Drop detections older than the hold window. recent_ is in ascending arrival
  // order, so pop from the front (mirrors detected_objects_grid::firstRetainedIndex).
  const rclcpp::Time now = this->now();
  while (!recent_.empty() && (now - recent_.front().first).seconds() > hold_time_s_) {
    recent_.pop_front();
  }

  // Without an ego pose we cannot place the grid window — skip until odometry
  // arrives (the shaper simply finds no grid and leaves the path untouched).
  if (!have_ego_) return;

  auto grid = dog::makeEgoCenteredGrid(
    ego_x_, ego_y_, target_frame_, resolution_, grid_length_m_);
  grid.header.stamp = now;

  const auto value = static_cast<int8_t>(std::clamp(occupancy_value_, 0, 100));

  for (const auto & entry : recent_) {
    const auto & msg = entry.second;
    if (!msg || msg->objects.empty()) continue;

    // Resolve the detection frame -> grid frame transform once per message.
    const std::string & src_frame = msg->header.frame_id;
    const bool needs_tf = !src_frame.empty() && src_frame != target_frame_;
    geometry_msgs::msg::TransformStamped tf_to_target;
    if (needs_tf) {
      try {
        tf_to_target = tf_buffer_->lookupTransform(target_frame_, src_frame, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "detection frame '%s' -> '%s' TF lookup failed: %s — skipping these detections.",
          src_frame.c_str(), target_frame_.c_str(), ex.what());
        continue;
      }
    }

    for (const auto & det : msg->objects) {
      if (det.existence_probability < min_existence_probability_) continue;
      geometry_msgs::msg::Pose pose_in_grid = det.kinematics.pose_with_covariance.pose;
      if (needs_tf) {
        tf2::doTransform(pose_in_grid, pose_in_grid, tf_to_target);
      }
      dog::markObjectFootprint(grid, det, pose_in_grid, inflation_, value);
    }
  }

  pub_grid_->publish(grid);
}

}  // namespace igv_velocity_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(igv_velocity_planner::DetectedObjectsToGridNode)
