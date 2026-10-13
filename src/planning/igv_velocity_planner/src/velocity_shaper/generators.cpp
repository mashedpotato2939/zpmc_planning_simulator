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

#include "igv_velocity_planner/velocity_shaper/generators.hpp"

#include "igv_velocity_planner/velocity_shaper/geometry_utils.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace igv_velocity_planner::velocity_shaper
{

namespace
{

bool isObjectFootprintFullyBehindEgo(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const geometry_msgs::msg::Pose & ego_pose,
  const double ego_min_longitudinal_offset)
{
  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  const double object_yaw = yawFromQuat(object_pose.orientation);
  const double object_cos = std::cos(object_yaw);
  const double object_sin = std::sin(object_yaw);
  const double ego_yaw = yawFromQuat(ego_pose.orientation);
  const double ego_cos = std::cos(ego_yaw);
  const double ego_sin = std::sin(ego_yaw);
  const double object_half_length =
    0.5 * std::max(0.0, static_cast<double>(object.shape.dimensions.x));
  const double object_half_width =
    0.5 * std::max(0.0, static_cast<double>(object.shape.dimensions.y));

  const double offsets_x[2] = {-object_half_length, object_half_length};
  const double offsets_y[2] = {-object_half_width, object_half_width};
  for (const double offset_x : offsets_x) {
    for (const double offset_y : offsets_y) {
      const double world_x =
        object_pose.position.x + offset_x * object_cos - offset_y * object_sin;
      const double world_y =
        object_pose.position.y + offset_x * object_sin + offset_y * object_cos;
      const double dx = world_x - ego_pose.position.x;
      const double dy = world_y - ego_pose.position.y;
      const double ego_frame_x = ego_cos * dx + ego_sin * dy;
      if (ego_frame_x >= ego_min_longitudinal_offset) {
        return false;
      }
    }
  }

  return true;
}

}  // namespace

void AdaptiveCruiseGenerator::generate(const TickInput & in, std::vector<VelocityConstraint> & out)
{
  if (in.objects == nullptr || in.odometry == nullptr) return;
  const double ego_v = in.odometry->twist.twist.linear.x;
  const auto cap = computeAdaptiveCruiseVelocityCap(in.points, in.objects->objects, ego_v, params_);
  if (!cap.has_value()) return;

  VelocityConstraint c;
  c.start_s = 0.0;
  c.end_s = std::numeric_limits<double>::infinity();
  c.target_velocity = *cap;
  c.type = ConstraintType::kAdaptiveCruise;
  c.priority = priorityOf(ConstraintType::kAdaptiveCruise);
  c.source = "adaptive_cruise";
  if (!in.points.empty()) c.anchor = in.points.front().pose.position;
  out.push_back(std::move(c));
}

void DynamicStopGenerator::generate(const TickInput & in, std::vector<VelocityConstraint> & out)
{
  if (in.objects == nullptr) return;
  const rclcpp::Time now = in.stamp;

  std::unordered_map<std::string, bool> seen;
  seen.reserve(in.objects->objects.size());

  for (const auto & obj : in.objects->objects) {
    size_t stop_idx = 0;
    double stop_dist = 0.0;
    const bool behind_ego =
      in.odometry != nullptr && in.ego_min_longitudinal_offset.has_value() &&
      isObjectFootprintFullyBehindEgo(
        obj, in.odometry->pose.pose, *in.ego_min_longitudinal_offset);
    const bool has_collision =
      !behind_ego &&
      computeDynamicObstacleStop(
        in.points, obj, params_, ego_front_clearance_, ego_half_width_, in.ego_s, stop_idx,
        stop_dist);

    const std::string key = uuidToString(obj.object_id);
    seen[key] = true;
    auto & state = decisions_[key];

    if (has_collision) {
      if (state.first_collision_time.nanoseconds() == 0) state.first_collision_time = now;
      state.last_collision_time = now;
      state.last_stop_distance = stop_dist;
      const double sustained = (now - state.first_collision_time).seconds();
      if (!state.committed && sustained >= params_.add_stop_duration_buffer) {
        state.committed = true;
      }
    } else if (state.committed) {
      const double clear_duration = (now - state.last_collision_time).seconds();
      if (clear_duration > params_.remove_stop_duration_buffer) {
        state.committed = false;
        state.first_collision_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
    }

    if (state.committed) {
      VelocityConstraint c;
      c.start_s = state.last_stop_distance;
      c.end_s = std::numeric_limits<double>::infinity();
      c.target_velocity = 0.0;
      c.type = ConstraintType::kDynamicStop;
      c.priority = priorityOf(ConstraintType::kDynamicStop);
      c.source = "dynamic:" + key;
      c.anchor = obj.kinematics.initial_pose_with_covariance.pose.position;
      out.push_back(std::move(c));
    }
  }

  for (auto it = decisions_.begin(); it != decisions_.end();) {
    if (seen.find(it->first) == seen.end()) {
      it = decisions_.erase(it);
    } else {
      ++it;
    }
  }
}

void StaticObstacleStopGenerator::generate(
  const TickInput & in, std::vector<VelocityConstraint> & out)
{
  if (in.objects == nullptr) return;
  const rclcpp::Time now = in.stamp;

  std::unordered_map<std::string, bool> seen;
  seen.reserve(in.objects->objects.size());

  for (const auto & obj : in.objects->objects) {
    double s_near = 0.0;
    const bool intrudes =
      computeStaticObstacleStop(in.points, obj, params_, ego_half_width_, in.ego_s, s_near);

    const std::string key = uuidToString(obj.object_id);
    seen[key] = true;
    auto & state = decisions_[key];

    if (intrudes) {
      if (state.first_intrusion_time.nanoseconds() == 0) state.first_intrusion_time = now;
      state.last_intrusion_time = now;
      // Back the stop off so the ego front face keeps a gap before the near face.
      state.last_stop_distance =
        std::max(in.ego_s, s_near - ego_front_clearance_ - params_.stop_distance_buffer);
      const double sustained = (now - state.first_intrusion_time).seconds();
      if (!state.committed && sustained >= params_.add_stop_duration_buffer) {
        state.committed = true;
      }
    } else if (state.committed) {
      const double clear = (now - state.last_intrusion_time).seconds();
      if (clear > params_.remove_stop_duration_buffer) {
        state.committed = false;
        state.first_intrusion_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
    }

    if (state.committed) {
      VelocityConstraint c;
      c.start_s = state.last_stop_distance;
      c.end_s = std::numeric_limits<double>::infinity();
      c.target_velocity = 0.0;
      c.type = ConstraintType::kStaticObstacleStop;
      c.priority = priorityOf(ConstraintType::kStaticObstacleStop);
      c.source = "static_stop:" + key;
      c.anchor = obj.kinematics.initial_pose_with_covariance.pose.position;
      out.push_back(std::move(c));
    }
  }

  for (auto it = decisions_.begin(); it != decisions_.end();) {
    if (seen.find(it->first) == seen.end()) {
      it = decisions_.erase(it);
    } else {
      ++it;
    }
  }
}

void SlowDownGenerator::generate(const TickInput & in, std::vector<VelocityConstraint> & out)
{
  if (in.objects == nullptr) return;
  const rclcpp::Time now = in.stamp;

  std::unordered_map<std::string, bool> seen;
  seen.reserve(in.objects->objects.size());

  for (const auto & obj : in.objects->objects) {
    const auto window = computeSlowDownWindow(in.points, obj, params_, ego_half_width_);
    const std::string key = uuidToString(obj.object_id);
    seen[key] = true;
    auto & state = decisions_[key];

    if (window) {
      if (state.first_detection_time.nanoseconds() == 0) state.first_detection_time = now;
      state.last_detection_time = now;
      const double sustained = (now - state.first_detection_time).seconds();
      if (!state.committed && sustained >= params_.add_duration_buffer) state.committed = true;
    } else if (state.committed) {
      const double clear = (now - state.last_detection_time).seconds();
      if (clear > params_.remove_duration_buffer) {
        state.committed = false;
        state.first_detection_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
    }

    if (state.committed && window) {
      VelocityConstraint c;
      c.start_s = window->s_start;
      // Legacy capVelocityRange caps the inclusive index idx_end; widen end_s
      // just past that point's arc-length so the half-open [start_s,end_s) test
      // covers the same samples.
      const double s_end_inclusive = arcLengthTo(in.points, window->idx_end);
      c.end_s = std::max(window->s_end, s_end_inclusive) + 1e-6;
      c.target_velocity = window->target_velocity;
      c.type = ConstraintType::kSlowDown;
      c.priority = priorityOf(ConstraintType::kSlowDown);
      c.source = "slow_down:" + key;
      c.anchor = window->obj_point;
      out.push_back(std::move(c));
    }
  }

  for (auto it = decisions_.begin(); it != decisions_.end();) {
    if (seen.find(it->first) == seen.end()) {
      it = decisions_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace igv_velocity_planner::velocity_shaper
