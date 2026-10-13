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

#include "igv_velocity_planner/velocity_shaper/slow_down.hpp"

#include "igv_velocity_planner/velocity_shaper/geometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace igv_velocity_planner::velocity_shaper
{

std::optional<SlowDownWindow> computeSlowDownWindow(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const autoware_perception_msgs::msg::PredictedObject & object,
  const Params::SlowDown & params,
  double ego_half_width)
{
  if (!params.enable || points.size() < 2) return std::nullopt;
  if (params.outer_lateral_margin <= params.inner_lateral_margin) return std::nullopt;

  const auto & obj_pos = object.kinematics.initial_pose_with_covariance.pose.position;
  const auto proj = projectOntoPath(points, obj_pos);

  const double lat = std::abs(proj.lat);
  // Shift both band edges out by the ego half-width so the band lives beyond
  // the ego footprint, not at the path centerline. Without this shift the
  // inner edge sat at the literal sideswipe boundary on `scale_igv` and inside
  // the ego on the full IGV.
  const double inner = params.inner_lateral_margin + ego_half_width;
  const double outer = params.outer_lateral_margin + ego_half_width;
  // Outer band only: strictly outside the inner band (which static stop /
  // occupancy owns) and inside the outer band.
  if (lat <= inner) return std::nullopt;
  if (lat >  outer) return std::nullopt;

  const double s_obj = proj.s;
  if (s_obj <= 0.0 || s_obj > params.max_distance) return std::nullopt;

  // Linear target ramp by lateral distance — closer to the path → slower.
  const double span = outer - inner;
  const double ratio = std::clamp((lat - inner) / span, 0.0, 1.0);
  const double target_v =
    params.min_target_velocity + ratio * (params.max_target_velocity - params.min_target_velocity);

  const double s_start = std::max(0.0, s_obj - params.forward_margin);
  const double s_end   = s_obj + params.backward_margin;

  SlowDownWindow w;
  w.idx_start = indexBeforeArclength(points, s_start);
  w.idx_end   = indexBeforeArclength(points, s_end);
  w.target_velocity = target_v;
  w.s_start = s_start;
  w.s_end   = s_end;
  w.s_obj   = s_obj;
  w.obj_point = obj_pos;
  return w;
}

}  // namespace igv_velocity_planner::velocity_shaper
