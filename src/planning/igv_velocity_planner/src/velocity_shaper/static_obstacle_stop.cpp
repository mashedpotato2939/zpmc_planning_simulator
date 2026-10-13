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

#include "igv_velocity_planner/velocity_shaper/static_obstacle_stop.hpp"

#include "igv_velocity_planner/velocity_shaper/geometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace igv_velocity_planner::velocity_shaper
{

bool computeStaticObstacleStop(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const autoware_perception_msgs::msg::PredictedObject & object,
  const Params::StaticObstacleStop & params,
  double ego_half_width,
  double ego_s,
  double & out_s_near)
{
  if (points.size() < 2) return false;
  // Moving objects are the dynamic_obstacle_stop pathway's job.
  if (objectSpeed(object) >= params.maximum_object_velocity) return false;

  const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
  const double cx = pose.position.x;
  const double cy = pose.position.y;
  const double yaw = yawFromQuat(pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double hl = 0.5 * std::max(0.0, static_cast<double>(object.shape.dimensions.x));
  const double hw = 0.5 * std::max(0.0, static_cast<double>(object.shape.dimensions.y));
  const double thresh = ego_half_width + params.lateral_clearance;
  const double upper_s = ego_s + params.max_distance;

  // Sample the oriented footprint perimeter: corners, edge midpoints, center.
  const double offs_x[3] = {-hl, 0.0, hl};
  const double offs_y[3] = {-hw, 0.0, hw};

  double best_s = std::numeric_limits<double>::infinity();
  double farthest_s = -std::numeric_limits<double>::infinity();
  bool found = false;
  for (const double sx : offs_x) {
    for (const double sy : offs_y) {
      geometry_msgs::msg::Point p;
      p.x = cx + sx * c - sy * s;
      p.y = cy + sx * s + sy * c;
      const auto proj = projectOntoPath(points, p);
      if (proj.s > upper_s) continue;
      if (std::abs(proj.lat) <= thresh) {
        found = true;
        best_s = std::min(best_s, proj.s);
        farthest_s = std::max(farthest_s, proj.s);
      }
    }
  }
  if (!found) return false;
  if (farthest_s < ego_s) return false;
  out_s_near = best_s;
  return true;
}

}  // namespace igv_velocity_planner::velocity_shaper
