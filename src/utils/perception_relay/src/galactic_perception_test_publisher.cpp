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

#include "perception_relay/galactic_perception_test_publisher.hpp"

// Additional message headers for field access

// Humble (input)
#include "autoware_perception_msgs/msg/tracked_object.hpp"
#include "autoware_perception_msgs/msg/tracked_object_kinematics.hpp"
#include "autoware_perception_msgs/msg/object_classification.hpp"
#include "autoware_perception_msgs/msg/shape.hpp"

// Galactic (output)
#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_object_kinematics.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_path.hpp"
#include "autoware_auto_perception_msgs/msg/detected_object.hpp"
#include "autoware_auto_perception_msgs/msg/detected_object_kinematics.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_object.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_object_kinematics.hpp"
#include "autoware_auto_perception_msgs/msg/object_classification.hpp"
#include "autoware_auto_perception_msgs/msg/shape.hpp"

namespace perception_relay
{

GalacticPerceptionTestPublisher::GalacticPerceptionTestPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("galactic_perception_test_publisher", options)
{
  // Subscribe to ground truth objects from dummy perception (Humble format)
  ground_truth_sub_ =
    this->create_subscription<autoware_perception_msgs::msg::TrackedObjects>(
      "~/input/ground_truth_objects",
      rclcpp::QoS(1),
      std::bind(&GalacticPerceptionTestPublisher::onGroundTruthObjects, this, std::placeholders::_1));

  // Create publishers for Galactic format messages
  predicted_objects_pub_ =
    this->create_publisher<autoware_auto_perception_msgs::msg::PredictedObjects>(
      "~/output/predicted_objects",
      rclcpp::QoS(1));

  detected_objects_pub_ =
    this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
      "~/output/detected_objects",
      rclcpp::QoS(1));

  tracked_objects_pub_ =
    this->create_publisher<autoware_auto_perception_msgs::msg::TrackedObjects>(
      "~/output/tracked_objects",
      rclcpp::QoS(1));

  RCLCPP_INFO(this->get_logger(), "Galactic perception test publisher initialized");
  RCLCPP_INFO(this->get_logger(), "  Subscribing to ground truth (Humble) and publishing Galactic format");
  RCLCPP_INFO(this->get_logger(), "  This enables end-to-end testing of the perception_relay module");
}

void GalacticPerceptionTestPublisher::onGroundTruthObjects(
  const autoware_perception_msgs::msg::TrackedObjects::SharedPtr msg)
{
  // Convert Humble TrackedObjects to Galactic TrackedObjects
  autoware_auto_perception_msgs::msg::TrackedObjects tracked_output;
  tracked_output.header = msg->header;

  // Also create Predicted and Detected versions
  autoware_auto_perception_msgs::msg::PredictedObjects predicted_output;
  predicted_output.header = msg->header;

  autoware_auto_perception_msgs::msg::DetectedObjects detected_output;
  detected_output.header = msg->header;

  for (const auto & input_object : msg->objects) {
    // --- TrackedObject (Humble -> Galactic) ---
    autoware_auto_perception_msgs::msg::TrackedObject tracked_obj;

    tracked_obj.object_id = input_object.object_id;
    tracked_obj.existence_probability = input_object.existence_probability;

    // Classification
    for (const auto & input_class : input_object.classification) {
      autoware_auto_perception_msgs::msg::ObjectClassification galactic_class;
      galactic_class.label = input_class.label;
      galactic_class.probability = input_class.probability;
      tracked_obj.classification.push_back(galactic_class);
    }

    // Kinematics
    tracked_obj.kinematics.pose_with_covariance =
      input_object.kinematics.pose_with_covariance;
    tracked_obj.kinematics.twist_with_covariance =
      input_object.kinematics.twist_with_covariance;
    tracked_obj.kinematics.acceleration_with_covariance =
      input_object.kinematics.acceleration_with_covariance;
    tracked_obj.kinematics.orientation_availability =
      input_object.kinematics.orientation_availability;
    tracked_obj.kinematics.is_stationary =
      input_object.kinematics.is_stationary;

    // Shape
    tracked_obj.shape.type = input_object.shape.type;
    tracked_obj.shape.footprint = input_object.shape.footprint;
    tracked_obj.shape.dimensions = input_object.shape.dimensions;

    tracked_output.objects.push_back(tracked_obj);

    // --- PredictedObject (derived from tracked) ---
    autoware_auto_perception_msgs::msg::PredictedObject predicted_obj;

    predicted_obj.object_id = input_object.object_id;
    predicted_obj.existence_probability = input_object.existence_probability;

    // Classification (same as tracked)
    for (const auto & input_class : input_object.classification) {
      autoware_auto_perception_msgs::msg::ObjectClassification galactic_class;
      galactic_class.label = input_class.label;
      galactic_class.probability = input_class.probability;
      predicted_obj.classification.push_back(galactic_class);
    }

    // Kinematics for predicted object
    predicted_obj.kinematics.initial_pose_with_covariance =
      input_object.kinematics.pose_with_covariance;
    predicted_obj.kinematics.initial_twist_with_covariance =
      input_object.kinematics.twist_with_covariance;
    predicted_obj.kinematics.initial_acceleration_with_covariance =
      input_object.kinematics.acceleration_with_covariance;

    // Create a simple predicted path (stationary or linear motion)
    autoware_auto_perception_msgs::msg::PredictedPath predicted_path;
    predicted_path.confidence = 1.0;
    predicted_path.time_step.sec = 0;
    predicted_path.time_step.nanosec = 500000000;  // 0.5 sec

    // Add current pose as the predicted path (simple stationary prediction)
    predicted_path.path.push_back(input_object.kinematics.pose_with_covariance.pose);
    predicted_obj.kinematics.predicted_paths.push_back(predicted_path);

    // Shape (same as tracked)
    predicted_obj.shape.type = input_object.shape.type;
    predicted_obj.shape.footprint = input_object.shape.footprint;
    predicted_obj.shape.dimensions = input_object.shape.dimensions;

    predicted_output.objects.push_back(predicted_obj);

    // --- DetectedObject (derived from tracked) ---
    autoware_auto_perception_msgs::msg::DetectedObject detected_obj;

    detected_obj.existence_probability = input_object.existence_probability;

    // Classification (same as tracked)
    for (const auto & input_class : input_object.classification) {
      autoware_auto_perception_msgs::msg::ObjectClassification galactic_class;
      galactic_class.label = input_class.label;
      galactic_class.probability = input_class.probability;
      detected_obj.classification.push_back(galactic_class);
    }

    // Kinematics for detected object
    detected_obj.kinematics.pose_with_covariance =
      input_object.kinematics.pose_with_covariance;
    detected_obj.kinematics.has_position_covariance = true;
    detected_obj.kinematics.orientation_availability =
      input_object.kinematics.orientation_availability;
    detected_obj.kinematics.twist_with_covariance =
      input_object.kinematics.twist_with_covariance;
    detected_obj.kinematics.has_twist = true;
    detected_obj.kinematics.has_twist_covariance = true;

    // Shape (same as tracked)
    detected_obj.shape.type = input_object.shape.type;
    detected_obj.shape.footprint = input_object.shape.footprint;
    detected_obj.shape.dimensions = input_object.shape.dimensions;

    detected_output.objects.push_back(detected_obj);
  }

  // Publish all three message types
  tracked_objects_pub_->publish(tracked_output);
  predicted_objects_pub_->publish(predicted_output);
  detected_objects_pub_->publish(detected_output);

  RCLCPP_DEBUG(
    this->get_logger(),
    "Published %zu objects as Galactic format (Tracked, Predicted, Detected)",
    msg->objects.size());
}

}  // namespace perception_relay
