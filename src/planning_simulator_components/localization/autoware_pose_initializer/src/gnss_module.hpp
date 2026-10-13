// Copyright 2022 The Autoware Contributors
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

#ifndef GNSS_MODULE_HPP_
#define GNSS_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <optional>

namespace autoware::pose_initializer
{

// Simple stub implementation for MapHeightFitter since the package is missing
class MapHeightFitter
{
public:
  explicit MapHeightFitter(rclcpp::Node* node) : node_(node) {}
  
  std::optional<geometry_msgs::msg::Point> fit(
    const geometry_msgs::msg::Point& position, 
    const std::string& frame_id)
  {
    // For now, just return the original position without height fitting
    // This can be enhanced later when the proper package is available
    (void)frame_id;  // Suppress unused parameter warning
    return position;
  }

private:
  rclcpp::Node* node_;
};

class GnssModule
{
private:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

public:
  explicit GnssModule(rclcpp::Node * node);
  PoseWithCovarianceStamped get_pose();

private:
  void on_pose(PoseWithCovarianceStamped::ConstSharedPtr msg);

  MapHeightFitter fitter_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_gnss_pose_;
  PoseWithCovarianceStamped::ConstSharedPtr pose_;
  double timeout_;
};
}  // namespace autoware::pose_initializer

#endif  // GNSS_MODULE_HPP_
