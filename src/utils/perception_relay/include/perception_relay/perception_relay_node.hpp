// Copyright 2024 ZPMC Autonomous
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

#ifndef PERCEPTION_RELAY__PERCEPTION_RELAY_NODE_HPP_
#define PERCEPTION_RELAY__PERCEPTION_RELAY_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

// Predicted Objects (Galactic -> Humble)
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"

// Detected Objects (Galactic -> Humble)
#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"
#include "autoware_perception_msgs/msg/detected_objects.hpp"

// Tracked Objects (Galactic -> Humble)
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"
#include "autoware_perception_msgs/msg/tracked_objects.hpp"

namespace perception_relay
{

class PerceptionRelayNode : public rclcpp::Node
{
public:
  explicit PerceptionRelayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Predicted Objects
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr
    predicted_objects_sub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr
    predicted_objects_pub_;

  // Detected Objects
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr
    detected_objects_sub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr
    detected_objects_pub_;

  // Tracked Objects
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr
    tracked_objects_sub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr
    tracked_objects_pub_;

  // Callback functions
  void onPredictedObjects(
    const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg);

  void onDetectedObjects(
    const autoware_auto_perception_msgs::msg::DetectedObjects::SharedPtr msg);

  void onTrackedObjects(
    const autoware_auto_perception_msgs::msg::TrackedObjects::SharedPtr msg);
};

}  // namespace perception_relay

#endif  // PERCEPTION_RELAY__PERCEPTION_RELAY_NODE_HPP_
