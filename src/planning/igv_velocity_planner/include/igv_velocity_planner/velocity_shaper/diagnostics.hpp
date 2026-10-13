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

#ifndef FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__DIAGNOSTICS_HPP_
#define FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__DIAGNOSTICS_HPP_

#include "igv_velocity_planner/velocity_shaper/constraint.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstdint>
#include <optional>

namespace igv_velocity_planner::velocity_shaper
{

struct HardStopMarkerData
{
  ConstraintType type{ConstraintType::kFmsBase};
  geometry_msgs::msg::Pose stop_pose;
  double stop_distance{0.0};
  std::optional<geometry_msgs::msg::Point> anchor;
};

bool isHardStopType(ConstraintType type);
const char * hardStopReasonText(ConstraintType type);
const char * hardStopVelocityFactorSequence(ConstraintType type);

void appendHardStopMarkers(
  visualization_msgs::msg::MarkerArray & markers, const std_msgs::msg::Header & header,
  const HardStopMarkerData & data, int32_t marker_group_id);

}  // namespace igv_velocity_planner::velocity_shaper

#endif  // FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__DIAGNOSTICS_HPP_
