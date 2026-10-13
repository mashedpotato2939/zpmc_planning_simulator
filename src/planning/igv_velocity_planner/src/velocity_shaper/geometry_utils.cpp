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

#include "igv_velocity_planner/velocity_shaper/geometry_utils.hpp"

#include "igv_motion_utils/trajectory_resample.hpp"
#include "igv_velocity_planner/trajectory_orientation.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace igv_velocity_planner::velocity_shaper
{

double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  // ZYX Euler yaw from quaternion (sufficient for vehicle yaw).
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double normalizeAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double distance2D(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

double objectSpeed(const autoware_perception_msgs::msg::PredictedObject & obj)
{
  const auto & v = obj.kinematics.initial_twist_with_covariance.twist.linear;
  return std::hypot(v.x, v.y);
}

double objectHalfWidth(const autoware_perception_msgs::msg::PredictedObject & obj)
{
  return 0.5 * std::max(0.1, static_cast<double>(obj.shape.dimensions.y));
}

size_t nearestIndex(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & query)
{
  if (points.empty()) return std::numeric_limits<size_t>::max();
  size_t best_i = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < points.size(); ++i) {
    const double dx = points[i].pose.position.x - query.x;
    const double dy = points[i].pose.position.y - query.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_i = i;
    }
  }
  return best_i;
}

double signedLateral(
  const autoware_planning_msgs::msg::TrajectoryPoint & traj_point,
  const geometry_msgs::msg::Point & point)
{
  const double yaw = yawFromQuat(traj_point.pose.orientation);
  const double dx = point.x - traj_point.pose.position.x;
  const double dy = point.y - traj_point.pose.position.y;
  return -std::sin(yaw) * dx + std::cos(yaw) * dy;
}

double arcLengthTo(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, size_t idx)
{
  if (idx == 0 || points.size() < 2) return 0.0;
  double s = 0.0;
  const size_t k = std::min(idx, points.size() - 1);
  for (size_t i = 1; i <= k; ++i) {
    s += distance2D(points[i].pose.position, points[i - 1].pose.position);
  }
  return s;
}

double arcLengthToPoint(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & query)
{
  return projectOntoPath(points, query).s;
}

PathProjection projectOntoPath(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & query)
{
  // Walk every segment, project the query onto each, and keep the segment
  // with the smallest perpendicular distance. Snapping to the nearest path
  // *vertex* (the old approach) breaks on sparsely sampled paths — when the
  // query falls in the middle of a long segment, vertex snapping returns the
  // wrong segment's tangent for the lat and the cumulative arc length up to
  // the far vertex for the s, so a stop point ends up *past* the obstacle.
  PathProjection result;
  if (points.size() < 2) return result;
  double accum_s = 0.0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 1; i < points.size(); ++i) {
    const auto & a = points[i - 1].pose.position;
    const auto & b = points[i].pose.position;
    const double seg_dx = b.x - a.x;
    const double seg_dy = b.y - a.y;
    const double seg_len2 = seg_dx * seg_dx + seg_dy * seg_dy;
    if (seg_len2 < 1e-18) {
      continue;  // degenerate segment — skip without advancing accum_s
    }
    const double seg_len = std::sqrt(seg_len2);
    const double vx = query.x - a.x;
    const double vy = query.y - a.y;
    const double t = std::clamp((vx * seg_dx + vy * seg_dy) / seg_len2, 0.0, 1.0);
    const double proj_x = a.x + t * seg_dx;
    const double proj_y = a.y + t * seg_dy;
    const double dx = proj_x - query.x;
    const double dy = proj_y - query.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      result.s = accum_s + t * seg_len;
      // Signed lateral offset of the query from the *clamped* projection point
      // (proj_x, proj_y), with the sign giving the side: positive when the
      // query is to the left of the segment's forward direction.
      //
      // For an interior projection (t in (0,1)) the clamped point is the foot
      // of the perpendicular, so this equals (seg_dx*vy - seg_dy*vx)/seg_len —
      // matching signedLateral's convention. But when t clamps to a segment
      // endpoint (the query lies beyond the segment, e.g. an object behind the
      // path start) the perpendicular-to-the-infinite-line formula badly
      // understates the offset: a point 30 m behind the start that happens to
      // sit near the backward extension of the first segment would report a
      // sub-metre lat and masquerade as on-path. Using the true distance to the
      // clamped point keeps such far-off objects out of the collision band.
      result.lat = std::copysign(std::sqrt(d2), seg_dx * vy - seg_dy * vx);
    }
    accum_s += seg_len;
  }
  return result;
}

size_t indexBeforeArclength(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, double target_s)
{
  if (points.empty() || target_s <= 0.0) return 0;
  double s = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    const double seg = distance2D(points[i].pose.position, points[i - 1].pose.position);
    if (s + seg >= target_s) return i - 1;
    s += seg;
  }
  return points.size() - 1;
}

void capVelocityFrom(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, size_t idx, double cap_vel)
{
  for (size_t i = idx; i < points.size(); ++i) {
    if (points[i].longitudinal_velocity_mps > cap_vel) {
      points[i].longitudinal_velocity_mps = static_cast<float>(cap_vel);
    }
  }
}

void capVelocityRange(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  size_t i_start, size_t i_end, double cap_vel)
{
  if (points.empty()) return;
  const size_t i1 = std::min(i_end, points.size() - 1);
  for (size_t i = i_start; i <= i1; ++i) {
    if (points[i].longitudinal_velocity_mps > cap_vel) {
      points[i].longitudinal_velocity_mps = static_cast<float>(cap_vel);
    }
  }
}

size_t insertStopAtArcLength(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, double s_stop)
{
  if (points.empty()) return 0;
  if (s_stop <= 0.0) {
    points.front().longitudinal_velocity_mps = 0.0f;
    return 0;
  }
  double s = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    const double seg = distance2D(points[i].pose.position, points[i - 1].pose.position);
    if (s + seg >= s_stop) {
      // Stop lands inside segment (i-1 → i): linearly interpolate position and
      // copy the upstream point's orientation (segment-aligned is fine; v=0).
      const double t = (seg > 1e-9) ? (s_stop - s) / seg : 0.0;
      autoware_planning_msgs::msg::TrajectoryPoint p = points[i - 1];
      p.pose.position.x =
        points[i - 1].pose.position.x + t * (points[i].pose.position.x - points[i - 1].pose.position.x);
      p.pose.position.y =
        points[i - 1].pose.position.y + t * (points[i].pose.position.y - points[i - 1].pose.position.y);
      p.pose.position.z =
        points[i - 1].pose.position.z + t * (points[i].pose.position.z - points[i - 1].pose.position.z);
      p.longitudinal_velocity_mps = 0.0f;
      points.insert(points.begin() + i, p);
      return i;
    }
    s += seg;
  }
  // s_stop is past the trajectory end — fall back to capping the last point.
  points.back().longitudinal_velocity_mps = 0.0f;
  return points.size() - 1;
}

std::string uuidToString(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::ostringstream os;
  os << std::hex << std::setfill('0');
  for (size_t i = 0; i < uuid.uuid.size(); ++i) {
    os << std::setw(2) << static_cast<int>(uuid.uuid[i]);
  }
  return os.str();
}

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> convertPathToTrajectoryPoints(
  const autoware_planning_msgs::msg::Path & path)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> out;
  out.reserve(path.points.size());
  for (const auto & p : path.points) {
    autoware_planning_msgs::msg::TrajectoryPoint tp;
    tp.pose = p.pose;
    tp.longitudinal_velocity_mps = p.longitudinal_velocity_mps;
    tp.lateral_velocity_mps = p.lateral_velocity_mps;
    tp.heading_rate_rps = p.heading_rate_rps;
    tp.acceleration_mps2 = 0.0f;
    out.push_back(tp);
  }
  return out;
}

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> resampleTrajectoryPoints(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, double interval)
{
  if (points.size() < 2 || interval <= 0.0) return points;

  constexpr bool use_zero_order_hold_for_twist = true;
  constexpr bool resample_input_trajectory_stop_point = true;
  auto output = igv_motion_utils::resampleTrajectoryPreserveOrientation(
    points, interval, use_zero_order_hold_for_twist, resample_input_trajectory_stop_point);
  trajectory_orientation::overwriteOrientationFromProjection(points, output);
  return output;
}

}  // namespace igv_velocity_planner::velocity_shaper
