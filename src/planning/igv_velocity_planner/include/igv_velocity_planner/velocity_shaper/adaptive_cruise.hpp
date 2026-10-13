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

#ifndef FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__ADAPTIVE_CRUISE_HPP_
#define FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__ADAPTIVE_CRUISE_HPP_

#include "igv_velocity_planner/velocity_shaper/params.hpp"

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <optional>
#include <vector>

namespace igv_velocity_planner::velocity_shaper
{

// Single-leader ACC cap. Finds the closest in-lane leader (inside the lateral
// band, heading-aligned, within search distance) and returns the velocity cap
// to apply from index 0. Returns nullopt when no valid leader is found.
std::optional<double> computeAdaptiveCruiseVelocityCap(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects,
  double ego_velocity,
  const Params::AdaptiveCruise & params);

}  // namespace igv_velocity_planner::velocity_shaper

#endif  // FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__ADAPTIVE_CRUISE_HPP_
