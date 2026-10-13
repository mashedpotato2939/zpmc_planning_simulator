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

#include "igv_velocity_planner/velocity_shaper/dynamic_obstacle_stop.hpp"

#include "igv_velocity_planner/velocity_shaper/geometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace igv_velocity_planner::velocity_shaper
{

bool computeDynamicObstacleStop(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const autoware_perception_msgs::msg::PredictedObject & object,
  const Params::DynamicObstacleStop & params,
  double ego_front_clearance,
  double ego_half_width,
  double ego_s,
  size_t & stop_index,
  double & stop_distance)
{
  if (points.size() < 2) return false;
  const double v = objectSpeed(object);
  if (v < params.minimum_object_velocity) return false;

  const double effective_half_width =
    objectHalfWidth(object) + 0.5 * params.extra_object_width + ego_half_width;

  // Walk the highest-confidence predicted path of the object and check whether
  // it intersects the trajectory band within time_horizon.
  if (object.kinematics.predicted_paths.empty()) return false;
  const auto * best_path = &object.kinematics.predicted_paths.front();
  for (const auto & pp : object.kinematics.predicted_paths) {
    if (pp.confidence > best_path->confidence) best_path = &pp;
  }
  const double time_step =
    best_path->time_step.sec + best_path->time_step.nanosec * 1e-9;

  double first_collision_s = 0.0;
  bool collision_found = false;
  const double min_collision_s = ego_s + params.minimum_object_distance_from_ego_trajectory;
  for (size_t i = 0; i < best_path->path.size(); ++i) {
    const double t = static_cast<double>(i) * time_step;
    if (t > params.time_horizon) break;
    const auto & obj_pose = best_path->path[i];
    const auto proj = projectOntoPath(points, obj_pose.position);
    if (std::abs(proj.lat) <= effective_half_width) {
      if (proj.s < min_collision_s) continue;
      first_collision_s = proj.s;
      collision_found = true;
      break;
    }
  }
  if (!collision_found) return false;

  // Pull the stop back so that ego's *front bumper* lands behind the object's
  // *rear face*, not so that base_link lands at the object's centroid. The
  // object's shape.dimensions.x is its length along its own heading; treating
  // it as longitudinal extent along the trajectory is conservative whenever
  // the object is roughly path-aligned (and over-conservative — never unsafe —
  // for cross-traffic).
  const double obj_half_length =
    0.5 * std::max(0.0, static_cast<double>(object.shape.dimensions.x));
  const double s_collision = first_collision_s;
  const double raw_s_stop =
    s_collision - obj_half_length - ego_front_clearance - params.stop_distance_buffer;
  const double s_stop = std::max(ego_s, raw_s_stop);

  stop_index = indexBeforeArclength(points, s_stop);
  stop_distance = s_stop;
  return true;
}

}  // namespace igv_velocity_planner::velocity_shaper
