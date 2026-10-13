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

#ifndef FMS_TRAJECTORY_BRIDGE__DETECTED_OBJECTS_GRID_NODE_HPP_
#define FMS_TRAJECTORY_BRIDGE__DETECTED_OBJECTS_GRID_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <memory>
#include <string>
#include <utility>

namespace igv_velocity_planner
{

// Rasterizes close-range DetectedObjects into a nav_msgs/OccupancyGrid that
// fms_velocity_shaper consumes via its (identity-free) occupancy-grid stop
// check. An OccupancyGrid carries no object identity, so the shaper's per-UUID
// stop hysteresis can no longer chatter when the detector re-IDs or drops a
// close obstacle frame-to-frame — the failure mode of feeding such detections
// in as synthesized PredictedObjects.
//
// A short recent-detections ring buffer provides a binary temporal hold: each
// publish cycle the union of detections seen within `hold_time_s` is rasterized,
// so a one- or two-frame dropout does not free the cells (the identity-free
// equivalent of the stop hysteresis).
class DetectedObjectsToGridNode : public rclcpp::Node
{
public:
  explicit DetectedObjectsToGridNode(const rclcpp::NodeOptions & options);

private:
  void onDetectedObjects(autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg);
  void onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onTimer();

  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr
    sub_detected_objects_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Recent detections kept for the hold window (ascending arrival time).
  std::deque<
    std::pair<rclcpp::Time, autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr>>
    recent_;

  // Latest ego position in the grid/target frame (from odometry).
  bool have_ego_{false};
  double ego_x_{0.0};
  double ego_y_{0.0};

  // Parameters
  bool enable_{true};
  std::string target_frame_{"map"};
  double resolution_{0.2};
  double grid_length_m_{80.0};
  double hold_time_s_{0.5};
  double publish_rate_hz_{10.0};
  int occupancy_value_{100};
  double lateral_margin_{0.1};
  double min_existence_probability_{0.0};

  // ego_half_width (from vehicle_info) + lateral_margin. Footprints are inflated
  // by this so the consumer's centerline-only sampling still catches laterally
  // offset obstacles.
  double inflation_{0.6};
};

}  // namespace igv_velocity_planner

#endif  // FMS_TRAJECTORY_BRIDGE__DETECTED_OBJECTS_GRID_NODE_HPP_
