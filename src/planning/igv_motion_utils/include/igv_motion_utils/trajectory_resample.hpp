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

#ifndef IGV_MOTION_UTILS__TRAJECTORY_RESAMPLE_HPP_
#define IGV_MOTION_UTILS__TRAJECTORY_RESAMPLE_HPP_

#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <vector>

namespace igv_motion_utils
{

using TrajectoryPoint = autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

TrajectoryPoints resampleTrajectoryPreserveOrientation(
  const TrajectoryPoints & points, double interval, bool use_zero_order_hold_for_twist = true,
  bool resample_input_trajectory_stop_point = true);

TrajectoryPoints resampleTrajectoryPreserveOrientation(
  const TrajectoryPoints & points, const std::vector<double> & resampled_arclength,
  bool use_zero_order_hold_for_twist = true);

void overwriteOrientationFromPathProjection(
  TrajectoryPoints & points, const autoware_planning_msgs::msg::Path & behavior_path);

}  // namespace igv_motion_utils

#endif  // IGV_MOTION_UTILS__TRAJECTORY_RESAMPLE_HPP_
