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

#ifndef PERCEPTION_RELAY__GALACTIC_PERCEPTION_TEST_PUBLISHER_HPP_
#define PERCEPTION_RELAY__GALACTIC_PERCEPTION_TEST_PUBLISHER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

// Humble (input) - from dummy perception ground truth
#include "autoware_perception_msgs/msg/tracked_objects.hpp"

// Galactic (output) - to perception_relay
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"

namespace perception_relay
{

/**
 * @brief Galactic Perception Test Publisher
 *
 * This node is used for testing the perception_relay module. It:
 * 1. Subscribes to dummy perception ground truth (Humble format)
 * 2. Converts to Galactic format (autoware_auto_perception_msgs)
 * 3. Publishes to topics that perception_relay expects
 *
 * This enables end-to-end testing of the perception relay pipeline:
 *   Dummy Perception (Humble) -> This Node -> Galactic msgs -> perception_relay -> Humble msgs
 */
class GalacticPerceptionTestPublisher : public rclcpp::Node
{
public:
  explicit GalacticPerceptionTestPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Subscription for ground truth objects from dummy perception (Humble format)
  rclcpp::Subscription<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr
    ground_truth_sub_;

  // Publishers for Galactic format messages
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr
    predicted_objects_pub_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr
    detected_objects_pub_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr
    tracked_objects_pub_;

  // Callback function
  void onGroundTruthObjects(
    const autoware_perception_msgs::msg::TrackedObjects::SharedPtr msg);
};

}  // namespace perception_relay

#endif  // PERCEPTION_RELAY__GALACTIC_PERCEPTION_TEST_PUBLISHER_HPP_
