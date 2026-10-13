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

#ifndef IGV_VELOCITY_PLANNER__TRAJECTORY_ORIENTATION_HPP_
#define IGV_VELOCITY_PLANNER__TRAJECTORY_ORIENTATION_HPP_

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace igv_velocity_planner::trajectory_orientation
{

using TrajectoryPoint = autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

inline geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

inline double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double distance2d(
  const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

inline std::vector<double> cumulativeArclength(const TrajectoryPoints & points)
{
  std::vector<double> arclength;
  if (points.empty()) {
    return arclength;
  }

  arclength.reserve(points.size());
  arclength.push_back(0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    arclength.push_back(
      arclength.back() + distance2d(points[i - 1].pose.position, points[i].pose.position));
  }
  return arclength;
}

inline std::vector<double> unwrappedYaw(const TrajectoryPoints & points)
{
  std::vector<double> yaw;
  yaw.reserve(points.size());
  for (const auto & point : points) {
    yaw.push_back(yawFromQuaternion(point.pose.orientation));
  }
  for (size_t i = 1; i < yaw.size(); ++i) {
    yaw[i] = yaw[i - 1] + normalizeAngle(yaw[i] - yaw[i - 1]);
  }
  return yaw;
}

inline double interpolateByArclength(
  const std::vector<double> & arclength, const std::vector<double> & values, double target_s)
{
  if (arclength.empty() || values.empty()) {
    return 0.0;
  }
  if (arclength.size() == 1 || values.size() == 1) {
    return values.front();
  }

  target_s = std::clamp(target_s, arclength.front(), arclength.back());
  const auto upper = std::upper_bound(arclength.begin(), arclength.end(), target_s);
  if (upper == arclength.begin()) {
    return values.front();
  }
  if (upper == arclength.end()) {
    return values.back();
  }

  const size_t i1 = static_cast<size_t>(std::distance(arclength.begin(), upper));
  const size_t i0 = i1 - 1;
  const double s0 = arclength[i0];
  const double s1 = arclength[i1];
  const double ratio = (s1 > s0) ? (target_s - s0) / (s1 - s0) : 0.0;
  return values[i0] + ratio * (values[i1] - values[i0]);
}

inline double projectArclength(
  const TrajectoryPoints & reference, const std::vector<double> & reference_s,
  const geometry_msgs::msg::Point & query)
{
  if (reference.size() < 2 || reference_s.size() != reference.size()) {
    return 0.0;
  }

  double best_s = reference_s.front();
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < reference.size(); ++i) {
    const auto & p0 = reference[i].pose.position;
    const auto & p1 = reference[i + 1].pose.position;
    const double vx = p1.x - p0.x;
    const double vy = p1.y - p0.y;
    const double wx = query.x - p0.x;
    const double wy = query.y - p0.y;
    const double len2 = vx * vx + vy * vy;
    const double ratio =
      len2 > 1.0e-12 ? std::clamp((wx * vx + wy * vy) / len2, 0.0, 1.0) : 0.0;
    const double proj_x = p0.x + ratio * vx;
    const double proj_y = p0.y + ratio * vy;
    const double dx = query.x - proj_x;
    const double dy = query.y - proj_y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_s = reference_s[i] + ratio * (reference_s[i + 1] - reference_s[i]);
    }
  }
  return best_s;
}

inline void overwriteOrientationFromProjection(
  const TrajectoryPoints & reference, TrajectoryPoints & target)
{
  if (reference.empty() || target.empty()) {
    return;
  }
  if (reference.size() == 1) {
    for (auto & point : target) {
      point.pose.orientation = reference.front().pose.orientation;
    }
    return;
  }

  const auto reference_s = cumulativeArclength(reference);
  const auto reference_yaw = unwrappedYaw(reference);
  for (auto & point : target) {
    const double s = projectArclength(reference, reference_s, point.pose.position);
    point.pose.orientation =
      createQuaternionFromYaw(interpolateByArclength(reference_s, reference_yaw, s));
  }
}

}  // namespace igv_velocity_planner::trajectory_orientation

#endif  // IGV_VELOCITY_PLANNER__TRAJECTORY_ORIENTATION_HPP_
