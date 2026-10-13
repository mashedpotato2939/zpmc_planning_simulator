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

#include "igv_velocity_planner/velocity_shaper/adaptive_cruise.hpp"

#include "igv_velocity_planner/velocity_shaper/geometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace igv_velocity_planner::velocity_shaper
{

std::optional<double> computeAdaptiveCruiseVelocityCap(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects,
  double ego_velocity,
  const Params::AdaptiveCruise & params)
{
  if (!params.enable || points.size() < 2 || objects.empty()) {
    return std::nullopt;
  }

  // Find the closest "leader": object inside the lateral band, ahead of ego,
  // heading aligned with our trajectory, and within search_distance.
  double best_s = std::numeric_limits<double>::infinity();
  const autoware_perception_msgs::msg::PredictedObject * leader = nullptr;

  for (const auto & obj : objects) {
    if (objectSpeed(obj) < 0.5) continue;  // static obstacles handled elsewhere.
    const auto & pose = obj.kinematics.initial_pose_with_covariance.pose;
    const auto proj = projectOntoPath(points, pose.position);

    const double lat = std::abs(proj.lat);
    if (lat > params.leader_lateral_margin + objectHalfWidth(obj)) continue;

    // Heading-alignment uses the nearest vertex's orientation as an approximation
    // of the local path heading at the projection. For evenly-sampled paths this
    // is fine; for sparse paths the segment direction would be more accurate,
    // but the leader_heading_max_diff threshold (~40°) is loose enough that the
    // small vertex-vs-segment heading delta doesn't change classification.
    const size_t idx = nearestIndex(points, pose.position);
    const double traj_yaw = yawFromQuat(points[idx].pose.orientation);
    const double obj_yaw = yawFromQuat(pose.orientation);
    if (std::abs(normalizeAngle(obj_yaw - traj_yaw)) > params.leader_heading_max_diff) {
      continue;
    }

    const double s_obj = proj.s;
    if (s_obj > params.leader_search_distance) continue;
    if (s_obj < best_s) {
      best_s = s_obj;
      leader = &obj;
    }
  }

  if (leader == nullptr) return std::nullopt;

  const double leader_v = objectSpeed(*leader);
  const double desired_gap = params.follow_distance_min + params.follow_time_gap * ego_velocity;
  const double gap_error = best_s - desired_gap;  // positive = too far, negative = too close

  // Velocity cap from leader speed minus margin.
  double v_cap_leader = std::max(0.0, leader_v - params.velocity_margin);

  // Velocity cap from gap closure with the kinematic limit.
  // If we're too close, reduce velocity further so we can decelerate within
  // the available gap.  v = sqrt(2 * a_max * extra_gap_to_close)
  double v_cap_gap = std::numeric_limits<double>::infinity();
  if (gap_error < 0.0) {
    const double extra_distance_to_close =
      std::max(0.0, best_s - params.follow_distance_min);
    const double v_brake =
      std::sqrt(2.0 * params.max_deceleration * extra_distance_to_close);
    v_cap_gap = std::min(v_cap_leader, v_brake);
  }

  return std::min(v_cap_leader, v_cap_gap);
}

}  // namespace igv_velocity_planner::velocity_shaper
