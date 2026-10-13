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

#include "perception_relay/perception_relay_node.hpp"

// Additional message headers for field access

// Predicted Objects (Galactic -> Humble)
#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"
#include "autoware_auto_perception_msgs/msg/object_classification.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_object_kinematics.hpp"
#include "autoware_auto_perception_msgs/msg/shape.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_path.hpp"

#include "autoware_perception_msgs/msg/predicted_object.hpp"
#include "autoware_perception_msgs/msg/object_classification.hpp"
#include "autoware_perception_msgs/msg/predicted_object_kinematics.hpp"
#include "autoware_perception_msgs/msg/shape.hpp"
#include "autoware_perception_msgs/msg/predicted_path.hpp"

// Detected Objects (Galactic -> Humble)
#include "autoware_auto_perception_msgs/msg/detected_object.hpp"
#include "autoware_auto_perception_msgs/msg/detected_object_kinematics.hpp"

#include "autoware_perception_msgs/msg/detected_object.hpp"
#include "autoware_perception_msgs/msg/detected_object_kinematics.hpp"

// Tracked Objects (Galactic -> Humble)
#include "autoware_auto_perception_msgs/msg/tracked_object.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_object_kinematics.hpp"

#include "autoware_perception_msgs/msg/tracked_object.hpp"
#include "autoware_perception_msgs/msg/tracked_object_kinematics.hpp"

namespace perception_relay
{

PerceptionRelayNode::PerceptionRelayNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("perception_relay_node", options)
{
  // PredictedObjects subscription and publisher
  predicted_objects_sub_ =
    this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
      "~/input/predicted_objects",
      rclcpp::QoS(1),
      std::bind(&PerceptionRelayNode::onPredictedObjects, this, std::placeholders::_1));

  predicted_objects_pub_ =
    this->create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
      "~/output/predicted_objects",
      rclcpp::QoS(1));

  // DetectedObjects subscription and publisher
  detected_objects_sub_ =
    this->create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
      "~/input/detected_objects",
      rclcpp::QoS(1),
      std::bind(&PerceptionRelayNode::onDetectedObjects, this, std::placeholders::_1));

  detected_objects_pub_ =
    this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
      "~/output/detected_objects",
      rclcpp::QoS(1));

  // TrackedObjects subscription and publisher
  tracked_objects_sub_ =
    this->create_subscription<autoware_auto_perception_msgs::msg::TrackedObjects>(
      "~/input/tracked_objects",
      rclcpp::QoS(1),
      std::bind(&PerceptionRelayNode::onTrackedObjects, this, std::placeholders::_1));

  tracked_objects_pub_ =
    this->create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
      "~/output/tracked_objects",
      rclcpp::QoS(1));

  RCLCPP_INFO(this->get_logger(), "Perception relay node initialized");
}

void PerceptionRelayNode::onPredictedObjects(
  const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg)
{
  autoware_perception_msgs::msg::PredictedObjects output_msg;
  output_msg.header = msg->header;

  for (const auto & input_object : msg->objects) {
    autoware_perception_msgs::msg::PredictedObject output_object;

    output_object.object_id = input_object.object_id;
    output_object.existence_probability = input_object.existence_probability;

    // Classification
    for (const auto & input_classification : input_object.classification) {
      autoware_perception_msgs::msg::ObjectClassification output_classification;
      output_classification.label = input_classification.label;
      output_classification.probability = input_classification.probability;
      output_object.classification.push_back(output_classification);
    }

    // Kinematics
    output_object.kinematics.initial_pose_with_covariance =
      input_object.kinematics.initial_pose_with_covariance;
    output_object.kinematics.initial_acceleration_with_covariance =
      input_object.kinematics.initial_acceleration_with_covariance;
    output_object.kinematics.initial_twist_with_covariance =
      input_object.kinematics.initial_twist_with_covariance;

    // Predicted paths
    for (const auto & input_path : input_object.kinematics.predicted_paths) {
      autoware_perception_msgs::msg::PredictedPath output_path;
      output_path.confidence = input_path.confidence;
      output_path.time_step = input_path.time_step;

      for (const auto & pose : input_path.path) {
        output_path.path.push_back(pose);
      }

      output_object.kinematics.predicted_paths.push_back(output_path);
    }

    // Shape
    output_object.shape.dimensions = input_object.shape.dimensions;
    output_object.shape.footprint = input_object.shape.footprint;
    output_object.shape.type = input_object.shape.type;

    output_msg.objects.push_back(output_object);
  }

  predicted_objects_pub_->publish(output_msg);
}

void PerceptionRelayNode::onDetectedObjects(
  const autoware_auto_perception_msgs::msg::DetectedObjects::SharedPtr msg)
{
  autoware_perception_msgs::msg::DetectedObjects output_msg;
  output_msg.header = msg->header;

  for (const auto & input_object : msg->objects) {
    autoware_perception_msgs::msg::DetectedObject output_object;

    // Existence probability
    output_object.existence_probability = input_object.existence_probability;

    // Classification
    for (const auto & input_classification : input_object.classification) {
      autoware_perception_msgs::msg::ObjectClassification output_classification;
      output_classification.label = input_classification.label;
      output_classification.probability = input_classification.probability;
      output_object.classification.push_back(output_classification);
    }

    // Kinematics
    output_object.kinematics.pose_with_covariance =
      input_object.kinematics.pose_with_covariance;
    output_object.kinematics.has_position_covariance =
      input_object.kinematics.has_position_covariance;
    output_object.kinematics.orientation_availability =
      input_object.kinematics.orientation_availability;

    output_object.kinematics.twist_with_covariance =
      input_object.kinematics.twist_with_covariance;
    output_object.kinematics.has_twist =
      input_object.kinematics.has_twist;
    output_object.kinematics.has_twist_covariance =
      input_object.kinematics.has_twist_covariance;

    // Shape
    output_object.shape.type = input_object.shape.type;
    output_object.shape.footprint = input_object.shape.footprint;
    output_object.shape.dimensions = input_object.shape.dimensions;

    output_msg.objects.push_back(output_object);
  }

  detected_objects_pub_->publish(output_msg);
}

void PerceptionRelayNode::onTrackedObjects(
  const autoware_auto_perception_msgs::msg::TrackedObjects::SharedPtr msg)
{
  autoware_perception_msgs::msg::TrackedObjects output_msg;
  output_msg.header = msg->header;

  for (const auto & input_object : msg->objects) {
    autoware_perception_msgs::msg::TrackedObject output_object;

    // ID and existence
    output_object.object_id = input_object.object_id;
    output_object.existence_probability = input_object.existence_probability;

    // Classification
    for (const auto & input_classification : input_object.classification) {
      autoware_perception_msgs::msg::ObjectClassification output_classification;
      output_classification.label = input_classification.label;
      output_classification.probability = input_classification.probability;
      output_object.classification.push_back(output_classification);
    }

    // Kinematics
    output_object.kinematics.pose_with_covariance =
      input_object.kinematics.pose_with_covariance;
    output_object.kinematics.twist_with_covariance =
      input_object.kinematics.twist_with_covariance;
    output_object.kinematics.acceleration_with_covariance =
      input_object.kinematics.acceleration_with_covariance;
    output_object.kinematics.orientation_availability =
      input_object.kinematics.orientation_availability;
    output_object.kinematics.is_stationary =
      input_object.kinematics.is_stationary;

    // Shape
    output_object.shape.type = input_object.shape.type;
    output_object.shape.footprint = input_object.shape.footprint;
    output_object.shape.dimensions = input_object.shape.dimensions;

    output_msg.objects.push_back(output_object);
  }

  tracked_objects_pub_->publish(output_msg);
}

}  // namespace perception_relay
