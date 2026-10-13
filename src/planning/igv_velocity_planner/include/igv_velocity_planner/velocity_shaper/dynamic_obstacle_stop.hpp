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

#ifndef FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__DYNAMIC_OBSTACLE_STOP_HPP_
#define FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__DYNAMIC_OBSTACLE_STOP_HPP_

#include "igv_velocity_planner/velocity_shaper/params.hpp"

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <vector>

namespace igv_velocity_planner::velocity_shaper
{

// Decide whether an object's highest-confidence predicted path crosses the
// trajectory inside the collision band within `time_horizon`. On success,
// writes the stop index (offset by `stop_distance_buffer`, the object's
// longitudinal half-extent, and `ego_front_clearance` so the ego's front
// bumper lands behind the obstacle's rear face) and the arc-length to that
// stop. Pure / stateless — the caller owns hysteresis.
//
// `ego_front_clearance` is the distance from base_link to the front bumper
// (i.e. `VehicleInfo::max_longitudinal_offset_m`).
// `ego_half_width` is half of `VehicleInfo::vehicle_width_m`; it is added to
// the lateral collision band so the predicted path is checked against the
// ego footprint plus `0.5 * extra_object_width` of safety buffer, not just
// the path centerline plus the object's own half-width.
// `ego_s` is ego's current arc length on `points`; predicted collisions behind
// it are ignored, and stop distances are never placed behind it.
bool computeDynamicObstacleStop(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const autoware_perception_msgs::msg::PredictedObject & object,
  const Params::DynamicObstacleStop & params,
  double ego_front_clearance,
  double ego_half_width,
  double ego_s,
  size_t & stop_index,
  double & stop_distance);

}  // namespace igv_velocity_planner::velocity_shaper

#endif  // FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__DYNAMIC_OBSTACLE_STOP_HPP_
