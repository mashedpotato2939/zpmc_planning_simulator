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

#ifndef FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__STATIC_OBSTACLE_STOP_HPP_
#define FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__STATIC_OBSTACLE_STOP_HPP_

#include "igv_velocity_planner/velocity_shaper/params.hpp"

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <vector>

namespace igv_velocity_planner::velocity_shaper
{

// Footprint-aware static-obstacle stop test. For a near-static object (speed
// below `maximum_object_velocity`), samples the object's oriented bounding-box
// perimeter (corners + edge midpoints + center), projects each sample onto the
// path, and reports whether any sample intrudes the ego corridor
// (|lateral| <= ego_half_width + lateral_clearance) within `max_distance`
// forward of `ego_s`.
// On success, `out_s_near` is the smallest arc-length among the intruding
// samples. Pure/stateless; the caller owns hysteresis and turns this into a
// v=0 constraint backed off by ego_front_clearance + stop_distance_buffer.
// Intrusions whose full footprint interval is behind `ego_s` are ignored.
bool computeStaticObstacleStop(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const autoware_perception_msgs::msg::PredictedObject & object,
  const Params::StaticObstacleStop & params,
  double ego_half_width,
  double ego_s,
  double & out_s_near);

}  // namespace igv_velocity_planner::velocity_shaper

#endif  // FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__STATIC_OBSTACLE_STOP_HPP_
