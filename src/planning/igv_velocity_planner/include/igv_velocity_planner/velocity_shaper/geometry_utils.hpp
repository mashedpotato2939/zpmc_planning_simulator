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

#ifndef FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__GEOMETRY_UTILS_HPP_
#define FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__GEOMETRY_UTILS_HPP_

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <string>
#include <vector>

// Shared geometric helpers used by every pathway in the velocity shaper.
// Kept in one header so each pathway cpp pulls only this for its math, and
// the node body shares the same uuid/conversion helpers.
namespace igv_velocity_planner::velocity_shaper
{

double yawFromQuat(const geometry_msgs::msg::Quaternion & q);
double normalizeAngle(double a);

double distance2D(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b);

double objectSpeed(const autoware_perception_msgs::msg::PredictedObject & obj);
double objectHalfWidth(const autoware_perception_msgs::msg::PredictedObject & obj);

// Find the nearest trajectory index to a query 2D point.
// Returns size_t SIZE_MAX if `points` is empty.
size_t nearestIndex(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & query);

// Signed lateral distance of `point` from the trajectory tangent at `traj_point`
// (positive left).
double signedLateral(
  const autoware_planning_msgs::msg::TrajectoryPoint & traj_point,
  const geometry_msgs::msg::Point & point);

// Arc length from index 0 to `idx` along the trajectory.
double arcLengthTo(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, size_t idx);

// Arc length from points[0] to a query 2D point (snapped to nearest index).
double arcLengthToPoint(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & query);

// Projection of a 2D query onto the path. Computes the closest point ON the
// path (not the closest path vertex) by iterating segments, then returns:
//   s   — arc length to that projection
//   lat — signed perpendicular distance to the path (positive = left of
//         forward), matching `signedLateral`'s convention.
// On a straight, evenly-sampled path this matches `nearestIndex` +
// `signedLateral` exactly. On a sparse path (long segments / bends) it
// avoids the vertex-snapping error that throws off both `arcLengthToPoint`
// and `signedLateral` when the query falls in the middle of a segment.
struct PathProjection
{
  double s{0.0};
  double lat{0.0};
};
PathProjection projectOntoPath(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & query);

// Index whose arc length from start is just under `target_s`.
size_t indexBeforeArclength(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, double target_s);

// Cap velocity at `idx` and every point after to at most `cap_vel`.
void capVelocityFrom(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  size_t idx, double cap_vel);

// Insert a new TrajectoryPoint at exactly `s_stop` arc-length, interpolating
// pose linearly between the two surrounding points (and inheriting their
// orientation). Returns the index of the inserted point with velocity = 0.
// Necessary when the upstream path is coarsely sampled — without it, a stop
// target that lands inside a long segment would otherwise snap back to the
// segment's start (via `indexBeforeArclength`), zeroing the whole trajectory.
size_t insertStopAtArcLength(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, double s_stop);

// Cap velocity over the inclusive range [i_start, i_end]. Sibling of
// capVelocityFrom — used by the slow-down pathways to depress velocity in a
// localized window without affecting downstream points.
void capVelocityRange(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  size_t i_start, size_t i_end, double cap_vel);

// Stringify a UUID for use as an unordered_map key (per-UUID hysteresis state).
std::string uuidToString(const unique_identifier_msgs::msg::UUID & uuid);

// Convert a Path 1:1 into TrajectoryPoints, preserving per-point velocity.
std::vector<autoware_planning_msgs::msg::TrajectoryPoint> convertPathToTrajectoryPoints(
  const autoware_planning_msgs::msg::Path & path);

// Densify trajectory points by arc length before velocity-shaper constraints run.
std::vector<autoware_planning_msgs::msg::TrajectoryPoint> resampleTrajectoryPoints(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, double interval);

}  // namespace igv_velocity_planner::velocity_shaper

#endif  // FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__GEOMETRY_UTILS_HPP_
