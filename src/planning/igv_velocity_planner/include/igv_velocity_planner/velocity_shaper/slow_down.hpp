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

#ifndef FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__SLOW_DOWN_HPP_
#define FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__SLOW_DOWN_HPP_

#include "igv_velocity_planner/velocity_shaper/params.hpp"

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <optional>
#include <vector>

namespace igv_velocity_planner::velocity_shaper
{

// Localized velocity dip around a specific outer-band object. The node body
// applies the window via capVelocityRange once hysteresis commits.
struct SlowDownWindow
{
  size_t idx_start{0};
  size_t idx_end{0};
  double target_velocity{0.0};            // [m/s] inside the window
  double s_start{0.0};                    // [m] arc-length to window start
  double s_end{0.0};                      // [m] arc-length to window end
  double s_obj{0.0};                      // [m] arc-length to the obstacle (diagnostic)
  geometry_msgs::msg::Point obj_point;    // world-frame obstacle position (diagnostic)
};

// Decide whether `object` qualifies for outer-band slow-down. Pure / stateless;
// the caller owns hysteresis. `ego_half_width` is half of
// `VehicleInfo::vehicle_width_m` — both inner and outer lateral margins are
// expanded by it so the band sits beyond the ego edge, not at the path
// centerline.
std::optional<SlowDownWindow> computeSlowDownWindow(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const autoware_perception_msgs::msg::PredictedObject & object,
  const Params::SlowDown & params,
  double ego_half_width = 0.0);

}  // namespace igv_velocity_planner::velocity_shaper

#endif  // FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__SLOW_DOWN_HPP_
