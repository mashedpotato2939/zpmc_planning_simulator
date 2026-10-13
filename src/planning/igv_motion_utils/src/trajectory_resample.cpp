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

#include "igv_motion_utils/trajectory_resample.hpp"

#include "autoware/motion_utils/constants.hpp"
#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace igv_motion_utils
{
namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void unwrapYaw(std::vector<double> & yaw)
{
  for (size_t i = 1; i < yaw.size(); ++i) {
    yaw[i] = yaw[i - 1] + normalizeAngle(yaw[i] - yaw[i - 1]);
  }
}

std::vector<double> cumulativeArclength(const TrajectoryPoints & points)
{
  std::vector<double> arclength;
  if (points.empty()) {
    return arclength;
  }

  arclength.reserve(points.size());
  arclength.push_back(0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    const auto & prev = points[i - 1].pose.position;
    const auto & curr = points[i].pose.position;
    arclength.push_back(
      arclength.back() + autoware::universe_utils::calcDistance2d(prev, curr));
  }
  return arclength;
}

std::vector<double> cumulativeArclength(const autoware_planning_msgs::msg::Path & path)
{
  std::vector<double> arclength;
  if (path.points.empty()) {
    return arclength;
  }

  arclength.reserve(path.points.size());
  arclength.push_back(0.0);
  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto & prev = path.points[i - 1].pose.position;
    const auto & curr = path.points[i].pose.position;
    arclength.push_back(
      arclength.back() + autoware::universe_utils::calcDistance2d(prev, curr));
  }
  return arclength;
}

std::vector<double> unwrappedYaw(const TrajectoryPoints & points)
{
  std::vector<double> yaw;
  yaw.reserve(points.size());
  for (const auto & point : points) {
    yaw.push_back(yawFromQuaternion(point.pose.orientation));
  }
  unwrapYaw(yaw);
  return yaw;
}

std::vector<double> unwrappedYaw(const autoware_planning_msgs::msg::Path & path)
{
  std::vector<double> yaw;
  yaw.reserve(path.points.size());
  for (const auto & point : path.points) {
    yaw.push_back(yawFromQuaternion(point.pose.orientation));
  }
  unwrapYaw(yaw);
  return yaw;
}

double interpolateByArclength(
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

void restoreOrientationByArclength(
  const TrajectoryPoints & input, const std::vector<double> & target_arclength,
  TrajectoryPoints & output)
{
  if (input.empty() || output.empty() || target_arclength.size() != output.size()) {
    return;
  }

  const auto input_arclength = cumulativeArclength(input);
  const auto input_yaw = unwrappedYaw(input);
  for (size_t i = 0; i < output.size(); ++i) {
    output[i].pose.orientation = createQuaternionFromYaw(
      interpolateByArclength(input_arclength, input_yaw, target_arclength[i]));
  }
}

std::vector<double> makeResamplingArclength(
  const TrajectoryPoints & points, const double interval,
  const bool resample_input_trajectory_stop_point)
{
  std::vector<double> arclength;
  if (points.size() < 2 || interval <= 0.0) {
    return arclength;
  }

  const double input_length = autoware::motion_utils::calcArcLength(points);
  for (double s = 0.0; s < input_length; s += interval) {
    arclength.push_back(s);
  }
  if (arclength.empty()) {
    return arclength;
  }

  if (input_length - arclength.back() < autoware::motion_utils::overlap_threshold) {
    arclength.back() = input_length;
  } else {
    arclength.push_back(input_length);
  }

  if (resample_input_trajectory_stop_point) {
    const auto stop_s = autoware::motion_utils::calcDistanceToForwardStopPoint(points, 0);
    if (stop_s) {
      for (size_t i = 1; i < arclength.size(); ++i) {
        if (arclength[i - 1] <= *stop_s && *stop_s < arclength[i]) {
          const double dist_to_prev = std::fabs(*stop_s - arclength[i - 1]);
          const double dist_to_next = std::fabs(arclength[i] - *stop_s);
          if (dist_to_prev < autoware::motion_utils::overlap_threshold) {
            arclength[i - 1] = *stop_s;
          } else if (dist_to_next < autoware::motion_utils::overlap_threshold) {
            arclength[i] = *stop_s;
          } else {
            arclength.insert(arclength.begin() + static_cast<std::ptrdiff_t>(i), *stop_s);
          }
          break;
        }
      }
    }
  }

  return arclength;
}

}  // namespace

TrajectoryPoints resampleTrajectoryPreserveOrientation(
  const TrajectoryPoints & points, const double interval,
  const bool use_zero_order_hold_for_twist, const bool resample_input_trajectory_stop_point)
{
  const auto arclength =
    makeResamplingArclength(points, interval, resample_input_trajectory_stop_point);
  if (arclength.empty()) {
    return points;
  }
  return resampleTrajectoryPreserveOrientation(points, arclength, use_zero_order_hold_for_twist);
}

TrajectoryPoints resampleTrajectoryPreserveOrientation(
  const TrajectoryPoints & points, const std::vector<double> & resampled_arclength,
  const bool use_zero_order_hold_for_twist)
{
  if (points.size() < 2 || resampled_arclength.size() < 2) {
    return points;
  }

  constexpr bool use_akima_spline_for_xy = false;
  constexpr bool use_lerp_for_z = true;
  const auto resampled = autoware::motion_utils::resampleTrajectory(
    autoware::motion_utils::convertToTrajectory(points), resampled_arclength,
    use_akima_spline_for_xy, use_lerp_for_z, use_zero_order_hold_for_twist);
  auto output = autoware::motion_utils::convertToTrajectoryPointArray(resampled);
  restoreOrientationByArclength(points, resampled_arclength, output);
  return output;
}

void overwriteOrientationFromPathProjection(
  TrajectoryPoints & points, const autoware_planning_msgs::msg::Path & behavior_path)
{
  if (points.empty() || behavior_path.points.empty()) {
    return;
  }
  if (behavior_path.points.size() == 1) {
    for (auto & point : points) {
      point.pose.orientation = behavior_path.points.front().pose.orientation;
    }
    return;
  }

  const auto path_arclength = cumulativeArclength(behavior_path);
  const auto path_yaw = unwrappedYaw(behavior_path);

  for (auto & point : points) {
    double best_d2 = std::numeric_limits<double>::infinity();
    double best_s = path_arclength.front();

    for (size_t i = 0; i + 1 < behavior_path.points.size(); ++i) {
      const auto & p0 = behavior_path.points[i].pose.position;
      const auto & p1 = behavior_path.points[i + 1].pose.position;
      const double vx = p1.x - p0.x;
      const double vy = p1.y - p0.y;
      const double wx = point.pose.position.x - p0.x;
      const double wy = point.pose.position.y - p0.y;
      const double len2 = vx * vx + vy * vy;
      const double ratio =
        len2 > 1.0e-12 ? std::clamp((wx * vx + wy * vy) / len2, 0.0, 1.0) : 0.0;
      const double px = p0.x + ratio * vx;
      const double py = p0.y + ratio * vy;
      const double dx = point.pose.position.x - px;
      const double dy = point.pose.position.y - py;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best_s = path_arclength[i] + ratio * (path_arclength[i + 1] - path_arclength[i]);
      }
    }

    point.pose.orientation =
      createQuaternionFromYaw(interpolateByArclength(path_arclength, path_yaw, best_s));
  }
}

}  // namespace igv_motion_utils
