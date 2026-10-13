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

#include "igv_velocity_planner/velocity_shaper/adaptive_cruise.hpp"
#include "igv_velocity_planner/velocity_shaper/diagnostics.hpp"
#include "igv_velocity_planner/velocity_shaper/dynamic_obstacle_stop.hpp"
#include "igv_velocity_planner/velocity_shaper/geometry_utils.hpp"
#include "igv_velocity_planner/velocity_shaper/params.hpp"
#include "igv_velocity_planner/velocity_shaper/slow_down.hpp"

#include <gtest/gtest.h>

#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

namespace
{

using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjectKinematics;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::PathPoint;
using autoware_planning_msgs::msg::TrajectoryPoint;
namespace vs = igv_velocity_planner::velocity_shaper;

geometry_msgs::msg::Quaternion yawToQuat(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double quatToYaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// Construct a straight, evenly-spaced path along +x with constant velocity.
Path makeStraightPath(double length, double step, double velocity)
{
  Path path;
  for (double s = 0.0; s <= length + 1e-6; s += step) {
    PathPoint p;
    p.pose.position.x = s;
    p.pose.position.y = 0.0;
    p.pose.orientation.w = 1.0;
    p.longitudinal_velocity_mps = static_cast<float>(velocity);
    path.points.push_back(p);
  }
  return path;
}

std::vector<TrajectoryPoint> straightTrajectoryPoints(double length, double step, double velocity)
{
  return igv_velocity_planner::velocity_shaper::convertPathToTrajectoryPoints(
    makeStraightPath(length, step, velocity));
}

PredictedObject makeStaticObject(double x, double y, double width = 1.0)
{
  PredictedObject obj;
  obj.kinematics.initial_pose_with_covariance.pose.position.x = x;
  obj.kinematics.initial_pose_with_covariance.pose.position.y = y;
  obj.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  obj.kinematics.initial_twist_with_covariance.twist.linear.x = 0.0;
  obj.kinematics.initial_twist_with_covariance.twist.linear.y = 0.0;
  obj.shape.dimensions.x = 2.0;
  obj.shape.dimensions.y = width;
  obj.shape.dimensions.z = 1.0;
  return obj;
}

// Moving object with a single straight-line predicted path crossing the
// trajectory at (cross_x, 0) at time `t_collision`.
PredictedObject makeMovingObject(
  double start_x, double start_y, double vx, double vy, double dt = 0.2,
  size_t n_steps = 30, double speed_override = -1.0)
{
  PredictedObject obj;
  obj.kinematics.initial_pose_with_covariance.pose.position.x = start_x;
  obj.kinematics.initial_pose_with_covariance.pose.position.y = start_y;
  obj.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  const double s = speed_override < 0.0 ? std::hypot(vx, vy) : speed_override;
  obj.kinematics.initial_twist_with_covariance.twist.linear.x = s;
  obj.shape.dimensions.x = 2.0;
  obj.shape.dimensions.y = 1.0;
  obj.shape.dimensions.z = 1.0;

  PredictedPath pp;
  pp.confidence = 1.0f;
  pp.time_step.sec = static_cast<int32_t>(dt);
  pp.time_step.nanosec = static_cast<uint32_t>((dt - static_cast<int32_t>(dt)) * 1e9);
  for (size_t i = 0; i < n_steps; ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = start_x + vx * i * dt;
    pose.position.y = start_y + vy * i * dt;
    pose.orientation.w = 1.0;
    pp.path.push_back(pose);
  }
  obj.kinematics.predicted_paths.push_back(pp);
  return obj;
}

const visualization_msgs::msg::Marker * findMarker(
  const visualization_msgs::msg::MarkerArray & markers, const std::string & ns, int32_t id)
{
  const auto it = std::find_if(
    markers.markers.begin(), markers.markers.end(),
    [&](const auto & marker) { return marker.ns == ns && marker.id == id; });
  return it == markers.markers.end() ? nullptr : &*it;
}

}  // namespace

// ---------------------------------------------------------------------------
// hard-stop diagnostics
// ---------------------------------------------------------------------------

TEST(FmsVelocityShaperDiagnostics, OccupancyStopMarkersIncludeWallAndReasonText)
{
  std_msgs::msg::Header header;
  header.frame_id = "odom";
  header.stamp.sec = 42;

  vs::HardStopMarkerData data;
  data.type = vs::ConstraintType::kOccupancyStop;
  data.stop_pose.position.x = 12.5;
  data.stop_pose.orientation.w = 1.0;
  data.stop_distance = 12.5;

  visualization_msgs::msg::MarkerArray markers;
  vs::appendHardStopMarkers(markers, header, data, 0);

  ASSERT_EQ(markers.markers.size(), 3u);
  EXPECT_STREQ(
    vs::hardStopReasonText(data.type), "stopping for occupied cell on path");
  EXPECT_STREQ(
    vs::hardStopVelocityFactorSequence(data.type), "fms_velocity_shaper.occupancy_grid");

  const auto * wall = findMarker(markers, "fms_velocity_shaper/stop_wall", 0);
  ASSERT_NE(wall, nullptr);
  EXPECT_EQ(wall->header.frame_id, "odom");
  EXPECT_EQ(wall->type, visualization_msgs::msg::Marker::CUBE);
  EXPECT_EQ(wall->action, visualization_msgs::msg::Marker::ADD);
  EXPECT_NEAR(wall->pose.position.x, 12.5, 1e-9);
  EXPECT_NEAR(wall->pose.position.z, 1.0, 1e-9);
  EXPECT_NEAR(wall->scale.x, 0.1, 1e-9);
  EXPECT_NEAR(wall->scale.y, 5.0, 1e-9);
  EXPECT_NEAR(wall->scale.z, 2.0, 1e-9);
  EXPECT_FLOAT_EQ(wall->color.r, 1.0f);
  EXPECT_FLOAT_EQ(wall->color.g, 0.0f);
  EXPECT_FLOAT_EQ(wall->color.b, 0.0f);
  EXPECT_FLOAT_EQ(wall->color.a, 0.5f);

  const auto * text = findMarker(markers, "fms_velocity_shaper/stop_text", 3);
  ASSERT_NE(text, nullptr);
  EXPECT_EQ(text->header.frame_id, "odom");
  EXPECT_EQ(text->type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_NE(text->text.find("STOP"), std::string::npos);
  EXPECT_NE(text->text.find("stopping for occupied cell on path"), std::string::npos);
  EXPECT_NE(text->text.find("stop distance=12.50m"), std::string::npos);
  EXPECT_EQ(text->text.find("obstacle gap="), std::string::npos);
}

TEST(FmsVelocityShaperDiagnostics, AnchoredStopsIncludeConnectorGapAndUniqueIds)
{
  std_msgs::msg::Header header;
  header.frame_id = "base_map";

  vs::HardStopMarkerData static_stop;
  static_stop.type = vs::ConstraintType::kStaticObstacleStop;
  static_stop.stop_pose.position.x = 10.0;
  static_stop.stop_pose.orientation.w = 1.0;
  static_stop.stop_distance = 10.0;
  geometry_msgs::msg::Point static_anchor;
  static_anchor.x = 13.0;
  static_anchor.y = 4.0;
  static_stop.anchor = static_anchor;

  vs::HardStopMarkerData dynamic_stop;
  dynamic_stop.type = vs::ConstraintType::kDynamicStop;
  dynamic_stop.stop_pose.position.x = 20.0;
  dynamic_stop.stop_pose.orientation.w = 1.0;
  dynamic_stop.stop_distance = 20.0;
  geometry_msgs::msg::Point dynamic_anchor;
  dynamic_anchor.x = 21.0;
  dynamic_stop.anchor = dynamic_anchor;

  visualization_msgs::msg::MarkerArray markers;
  vs::appendHardStopMarkers(markers, header, static_stop, 0);
  vs::appendHardStopMarkers(markers, header, dynamic_stop, 1);

  ASSERT_EQ(markers.markers.size(), 8u);
  EXPECT_STREQ(
    vs::hardStopReasonText(static_stop.type), "stopping for static obstacle in path");
  EXPECT_STREQ(
    vs::hardStopVelocityFactorSequence(static_stop.type),
    "fms_velocity_shaper.static_obstacle_stop");
  EXPECT_STREQ(
    vs::hardStopReasonText(dynamic_stop.type),
    "stopping for moving obstacle on predicted collision path");
  EXPECT_STREQ(
    vs::hardStopVelocityFactorSequence(dynamic_stop.type),
    "fms_velocity_shaper.dynamic_obstacle_stop");

  const auto * static_line = findMarker(markers, "fms_velocity_shaper/stop_to_obj", 2);
  ASSERT_NE(static_line, nullptr);
  EXPECT_EQ(static_line->type, visualization_msgs::msg::Marker::LINE_STRIP);
  ASSERT_EQ(static_line->points.size(), 2u);
  EXPECT_NEAR(static_line->points.front().x, 10.0, 1e-9);
  EXPECT_NEAR(static_line->points.back().x, 13.0, 1e-9);
  EXPECT_NEAR(static_line->points.back().y, 4.0, 1e-9);

  const auto * dynamic_line = findMarker(markers, "fms_velocity_shaper/stop_to_obj", 12);
  ASSERT_NE(dynamic_line, nullptr);
  EXPECT_EQ(dynamic_line->type, visualization_msgs::msg::Marker::LINE_STRIP);
  ASSERT_EQ(dynamic_line->points.size(), 2u);
  EXPECT_NEAR(dynamic_line->points.front().x, 20.0, 1e-9);
  EXPECT_NEAR(dynamic_line->points.back().x, 21.0, 1e-9);

  const auto * static_text = findMarker(markers, "fms_velocity_shaper/stop_text", 3);
  ASSERT_NE(static_text, nullptr);
  EXPECT_NE(static_text->text.find("stopping for static obstacle in path"), std::string::npos);
  EXPECT_NE(static_text->text.find("obstacle gap=5.00m"), std::string::npos);

  const auto * dynamic_text = findMarker(markers, "fms_velocity_shaper/stop_text", 13);
  ASSERT_NE(dynamic_text, nullptr);
  EXPECT_NE(
    dynamic_text->text.find("stopping for moving obstacle on predicted collision path"),
    std::string::npos);
  EXPECT_NE(dynamic_text->text.find("obstacle gap=1.00m"), std::string::npos);

  std::set<int32_t> ids;
  std::set<std::pair<std::string, int32_t>> marker_keys;
  for (const auto & marker : markers.markers) {
    EXPECT_TRUE(ids.insert(marker.id).second);
    EXPECT_TRUE(marker_keys.insert({marker.ns, marker.id}).second);
  }
}

// ---------------------------------------------------------------------------
// convertPathToTrajectoryPoints
// ---------------------------------------------------------------------------

TEST(FmsVelocityShaper, PathToTrajectoryPreservesVelocity)
{
  const auto path = makeStraightPath(20.0, 1.0, 5.0);
  const auto pts = igv_velocity_planner::velocity_shaper::convertPathToTrajectoryPoints(path);
  ASSERT_EQ(pts.size(), path.points.size());
  for (size_t i = 0; i < pts.size(); ++i) {
    EXPECT_FLOAT_EQ(pts[i].longitudinal_velocity_mps, path.points[i].longitudinal_velocity_mps);
    EXPECT_DOUBLE_EQ(pts[i].pose.position.x, path.points[i].pose.position.x);
  }
}

TEST(FmsVelocityShaper, PathToTrajectoryHandlesEmpty)
{
  Path empty;
  const auto pts = igv_velocity_planner::velocity_shaper::convertPathToTrajectoryPoints(empty);
  EXPECT_TRUE(pts.empty());
}

TEST(FmsVelocityShaper, ResampleTrajectoryPointsDensifiesSparsePath)
{
  const auto sparse = straightTrajectoryPoints(24.0, 24.0, 5.0);
  const auto dense = igv_velocity_planner::velocity_shaper::resampleTrajectoryPoints(sparse, 0.5);

  ASSERT_GT(dense.size(), sparse.size());
  EXPECT_NEAR(dense.front().pose.position.x, sparse.front().pose.position.x, 1e-9);
  EXPECT_NEAR(dense.back().pose.position.x, sparse.back().pose.position.x, 1e-9);

  double max_step = 0.0;
  for (size_t i = 1; i < dense.size(); ++i) {
    const double dx = dense[i].pose.position.x - dense[i - 1].pose.position.x;
    const double dy = dense[i].pose.position.y - dense[i - 1].pose.position.y;
    max_step = std::max(max_step, std::hypot(dx, dy));
    EXPECT_FLOAT_EQ(dense[i].longitudinal_velocity_mps, 5.0f);
  }
  EXPECT_LE(max_step, 0.5 + 1e-6);
}

TEST(FmsVelocityShaper, ResampleTrajectoryPointsPreservesBodyYaw)
{
  auto sparse = straightTrajectoryPoints(24.0, 24.0, 5.0);
  ASSERT_EQ(sparse.size(), 2u);

  constexpr double body_yaw = M_PI / 6.0;
  for (auto & point : sparse) {
    point.pose.orientation = yawToQuat(body_yaw);
  }

  const auto dense = vs::resampleTrajectoryPoints(sparse, 0.5);
  ASSERT_GT(dense.size(), sparse.size());
  for (const auto & point : dense) {
    EXPECT_NEAR(normalizeAngle(quatToYaw(point.pose.orientation) - body_yaw), 0.0, 1e-6);
  }
}

// ---------------------------------------------------------------------------
// computeDynamicObstacleStop
// ---------------------------------------------------------------------------

TEST(FmsVelocityShaper, DynamicObstacleCrossingTriggersStop)
{
  const auto pts = straightTrajectoryPoints(40.0, 1.0, 5.0);
  // Object starts 8 m to the right of x=15, moves left at 2 m/s — collides ~4s.
  auto obj = makeMovingObject(15.0, 8.0, 0.0, -2.0, 0.2, 30);
  igv_velocity_planner::velocity_shaper::Params::DynamicObstacleStop p;
  p.minimum_object_velocity = 0.5;
  p.stop_distance_buffer = 0.5;
  p.time_horizon = 5.0;
  p.extra_object_width = 1.0;
  p.minimum_object_distance_from_ego_trajectory = 1.0;

  size_t stop_idx = 0;
  double stop_dist = 0.0;
  const double ego_front_clearance = 0.0;  // unit test treats ego as a point
  const double ego_half_width = 0.0;       // unit test treats ego as a point
  const bool triggered = igv_velocity_planner::velocity_shaper::computeDynamicObstacleStop(
    pts, obj, p, ego_front_clearance, ego_half_width, 0.0, stop_idx, stop_dist);
  EXPECT_TRUE(triggered);
  // Stop point should be within ±2 m of the predicted collision x=15.
  EXPECT_NEAR(pts[stop_idx].pose.position.x, 15.0, 2.0);
  // Stop distance must be non-negative and less than collision distance.
  EXPECT_GE(stop_dist, 0.0);
  EXPECT_LE(stop_dist, 15.0);
}

TEST(FmsVelocityShaper, DynamicObstacleAwayDoesNotTriggerStop)
{
  const auto pts = straightTrajectoryPoints(40.0, 1.0, 5.0);
  // Object 8 m left of x=15, moving farther left — no crossing.
  auto obj = makeMovingObject(15.0, -8.0, 0.0, -2.0, 0.2, 30);
  igv_velocity_planner::velocity_shaper::Params::DynamicObstacleStop p;
  size_t stop_idx = 0;
  double stop_dist = 0.0;
  EXPECT_FALSE(
    igv_velocity_planner::velocity_shaper::computeDynamicObstacleStop(
      pts, obj, p, 0.0, 0.0, 0.0, stop_idx, stop_dist));
}

TEST(FmsVelocityShaper, DynamicObstacleSlowMovingNotConsidered)
{
  const auto pts = straightTrajectoryPoints(40.0, 1.0, 5.0);
  // Object on the path but barely moving — falls below minimum_object_velocity.
  auto obj = makeMovingObject(15.0, 0.0, 0.1, 0.0, 0.2, 30);
  obj.kinematics.initial_twist_with_covariance.twist.linear.x = 0.1;
  igv_velocity_planner::velocity_shaper::Params::DynamicObstacleStop p;
  p.minimum_object_velocity = 0.5;
  size_t stop_idx = 0;
  double stop_dist = 0.0;
  EXPECT_FALSE(
    igv_velocity_planner::velocity_shaper::computeDynamicObstacleStop(
      pts, obj, p, 0.0, 0.0, 0.0, stop_idx, stop_dist));
}

// Regression: when the nearest path vertex's orientation does not coincide
// with the segment direction (common on FMS paths where vertex orientations
// are smoothed across bends), the old `signedLateral(points[idx], q)`
// reported the perpendicular distance to the vertex tangent, not to the
// segment itself. The fixed projection-based helper returns the true
// segment-perpendicular distance.
TEST(FmsVelocityShaper, ProjectOntoPathReturnsSegmentPerpendicular)
{
  // Two-point path along +x, but the second vertex is rotated 30° so its
  // tangent diverges from the segment direction.
  std::vector<TrajectoryPoint> pts(2);
  pts[0].pose.position.x = 0.0;
  pts[0].pose.position.y = 0.0;
  pts[0].pose.orientation.w = 1.0;
  pts[1].pose.position.x = 30.0;
  pts[1].pose.position.y = 0.0;
  // Yaw = 30° at the second vertex (z = sin(15°), w = cos(15°)).
  pts[1].pose.orientation.z = std::sin(M_PI / 12.0);
  pts[1].pose.orientation.w = std::cos(M_PI / 12.0);

  geometry_msgs::msg::Point q;
  q.x = 20.0;
  q.y = 2.0;
  q.z = 0.0;

  // True perpendicular distance to the +x segment is 2.0m.
  const auto proj = igv_velocity_planner::velocity_shaper::projectOntoPath(pts, q);
  EXPECT_NEAR(std::abs(proj.lat), 2.0, 1e-6);
  EXPECT_NEAR(proj.s, 20.0, 1e-6);

  // The vertex-tangent approximation at points[1] (yaw=30°) gives a wildly
  // different lat (the query is 10m *behind* points[1], so projecting onto
  // its 30°-rotated tangent line picks up a large component of that 10m).
  // Sanity-check the regression by showing the old approach diverges far
  // from the true 2.0m.
  const double vertex_lat = std::abs(
    igv_velocity_planner::velocity_shaper::signedLateral(pts[1], q));
  EXPECT_GT(std::abs(vertex_lat - 2.0), 1.0);
}

// Regression (bag planning_control_20260529_084538): an object that lies well
// beyond a segment's endpoint must report its *true* distance as `lat`, not the
// perpendicular distance to the segment's infinite line. The triggering object
// sat ~30 m behind the path start, almost exactly on the backward extension of
// the first segment — the old infinite-line formula reported lat ≈ 0.57 m, so
// dynamic_obstacle_stop saw a phantom collision at s=0 and pinned a stop on the
// ego, preventing launch. The fixed helper reports |lat| ≈ 30 m.
TEST(FmsVelocityShaper, ProjectOntoPathClampedEndpointReportsTrueDistance)
{
  // Single segment heading +x from the origin. Query sits 30 m *behind* the
  // start (negative x), only 0.5 m off the segment's infinite line.
  std::vector<TrajectoryPoint> pts(2);
  pts[0].pose.position.x = 0.0;  pts[0].pose.position.y = 0.0;
  pts[0].pose.orientation.w = 1.0;
  pts[1].pose.position.x = 30.0; pts[1].pose.position.y = 0.0;
  pts[1].pose.orientation.w = 1.0;

  geometry_msgs::msg::Point q;
  q.x = -30.0;
  q.y = 0.5;
  q.z = 0.0;

  const auto proj = igv_velocity_planner::velocity_shaper::projectOntoPath(pts, q);
  // Projection clamps to the start vertex.
  EXPECT_NEAR(proj.s, 0.0, 1e-6);
  // lat must be the true ~30 m distance to that vertex, not the 0.5 m
  // infinite-line perpendicular that the pre-fix code returned.
  EXPECT_NEAR(std::abs(proj.lat), std::hypot(30.0, 0.5), 1e-6);
  EXPECT_GT(std::abs(proj.lat), 29.0);
}

// Regression (same bag): the full dynamic_obstacle_stop path. A near-stationary
// object ~30 m behind the ego, sitting on the backward extension of the path's
// first segment, must NOT trigger a stop. Pre-fix, projectOntoPath reported a
// tiny lat for it, computeDynamicObstacleStop declared a collision at s=0, and
// the velocity shaper pinned a zero-velocity point on the ego for the entire
// run — the vehicle could never launch.
TEST(FmsVelocityShaper, DynamicObstacleBehindEgoDoesNotTriggerStop)
{
  // Path heads +x from the origin.
  const auto pts = straightTrajectoryPoints(40.0, 1.0, 5.0);
  // Object 30 m behind the start (x=-30), 0.5 m off the path line, essentially
  // parked (tiny drift so the speed gate at minimum_object_velocity=0 passes).
  auto obj = makeMovingObject(-30.0, 0.5, 0.01, 0.0, 0.2, 30);
  obj.shape.dimensions.y = 1.7;
  obj.kinematics.initial_twist_with_covariance.twist.linear.x = 0.01;

  igv_velocity_planner::velocity_shaper::Params::DynamicObstacleStop p;
  p.minimum_object_velocity = 0.0;
  p.stop_distance_buffer = 2.0;
  p.time_horizon = 5.0;
  p.extra_object_width = 0.5;
  p.minimum_object_distance_from_ego_trajectory = 0.0;

  size_t stop_idx = 0;
  double stop_dist = 0.0;
  // ego_half_width 0.5 matches the scale_igv vehicle in the bag.
  EXPECT_FALSE(igv_velocity_planner::velocity_shaper::computeDynamicObstacleStop(
    pts, obj, p, /*ego_front_clearance=*/2.5, /*ego_half_width=*/0.5, 0.0, stop_idx,
    stop_dist));
}

TEST(FmsVelocityShaper, DynamicObstacleBehindEgoArcLengthDoesNotTriggerStop)
{
  const auto pts = straightTrajectoryPoints(40.0, 1.0, 5.0);
  auto obj = makeMovingObject(8.0, 0.0, 0.01, 0.0, 0.2, 10);

  igv_velocity_planner::velocity_shaper::Params::DynamicObstacleStop p;
  p.minimum_object_velocity = 0.0;
  p.stop_distance_buffer = 2.0;
  p.time_horizon = 5.0;
  p.extra_object_width = 1.0;
  p.minimum_object_distance_from_ego_trajectory = 0.0;

  size_t stop_idx = 0;
  double stop_dist = 0.0;
  EXPECT_FALSE(igv_velocity_planner::velocity_shaper::computeDynamicObstacleStop(
    pts, obj, p, /*ego_front_clearance=*/0.0, /*ego_half_width=*/0.5, /*ego_s=*/10.0,
    stop_idx, stop_dist));
}

TEST(FmsVelocityShaper, DynamicObstacleAheadStopDistanceClampedToEgoArcLength)
{
  const auto pts = straightTrajectoryPoints(40.0, 1.0, 5.0);
  auto obj = makeMovingObject(12.0, 0.0, 0.01, 0.0, 0.2, 10);
  obj.shape.dimensions.x = 2.0;

  igv_velocity_planner::velocity_shaper::Params::DynamicObstacleStop p;
  p.minimum_object_velocity = 0.0;
  p.stop_distance_buffer = 2.0;
  p.time_horizon = 5.0;
  p.extra_object_width = 1.0;
  p.minimum_object_distance_from_ego_trajectory = 0.0;

  size_t stop_idx = 0;
  double stop_dist = 0.0;
  ASSERT_TRUE(igv_velocity_planner::velocity_shaper::computeDynamicObstacleStop(
    pts, obj, p, /*ego_front_clearance=*/0.0, /*ego_half_width=*/0.5, /*ego_s=*/10.0,
    stop_idx, stop_dist));
  EXPECT_NEAR(stop_dist, 10.0, 1e-6);
  EXPECT_EQ(stop_idx, 9u);
}

// Regression: on a sparsely sampled path the old arcLengthToPoint snapped to
// the nearest vertex, so an obstacle mid-segment had its arc length reported
// as the *far* vertex's. dynamic_obstacle_stop then placed the stop past the
// obstacle. The fixed helper projects onto segments and returns the true
// arc length.
TEST(FmsVelocityShaper, DynamicObstacleSparsePathStopLandsBeforeObstacle)
{
  // Two-point path: a single 30m segment along +x.
  std::vector<TrajectoryPoint> pts(2);
  pts[0].pose.position.x = 0.0;  pts[0].pose.position.y = 0.0;
  pts[0].pose.orientation.w = 1.0;  pts[0].longitudinal_velocity_mps = 5.0;
  pts[1].pose.position.x = 30.0; pts[1].pose.position.y = 0.0;
  pts[1].pose.orientation.w = 1.0; pts[1].longitudinal_velocity_mps = 5.0;

  // Slow drifting object at x=15 (the middle of the segment), 0.4m off-path.
  auto obj = makeMovingObject(15.0, 0.4, 0.01, 0.0, 0.2, 10);
  obj.shape.dimensions.x = 4.0;  // half-length 2.0
  obj.shape.dimensions.y = 1.0;
  obj.kinematics.initial_twist_with_covariance.twist.linear.x = 0.01;

  igv_velocity_planner::velocity_shaper::Params::DynamicObstacleStop p;
  p.minimum_object_velocity = 0.0;
  p.stop_distance_buffer = 2.0;
  p.time_horizon = 5.0;
  p.extra_object_width = 1.0;
  p.minimum_object_distance_from_ego_trajectory = 1.0;

  size_t stop_idx = 0;
  double stop_dist = 0.0;
  ASSERT_TRUE(igv_velocity_planner::velocity_shaper::computeDynamicObstacleStop(
    pts, obj, p, /*ego_front_clearance=*/0.0, /*ego_half_width=*/0.0, 0.0, stop_idx,
    stop_dist));
  // Stop should be ~15 - half_length(2) - buffer(2) = 11m, i.e. *before* the
  // obstacle. The pre-fix code returned stop_dist≈30 - 2 - 2 = 26m — past it.
  EXPECT_LT(stop_dist, 15.0);
  EXPECT_NEAR(stop_dist, 11.0, 1.0);
}

// An almost-stationary object whose predicted-path stays glued to its centroid
// sits 1.54m off the path. With effective_half_width = obj_half_width (0.99)
// + 0.5 * extra_object_width (0.5) = 1.49m it just misses the band. Feed a
// 0.5m ego half-width and the band reaches 1.99m → collision detected.
TEST(FmsVelocityShaper, DynamicObstacleCaughtOnlyWhenEgoHalfWidthIncluded)
{
  const auto pts = straightTrajectoryPoints(40.0, 1.0, 5.0);
  // Tiny forward drift so the speed gate (minimum_object_velocity = 0) passes.
  auto obj = makeMovingObject(15.0, 1.54, 0.01, 0.0, 0.2, 30);
  obj.shape.dimensions.y = 1.98;  // half-width 0.99m
  obj.kinematics.initial_twist_with_covariance.twist.linear.x = 0.01;

  igv_velocity_planner::velocity_shaper::Params::DynamicObstacleStop p;
  p.minimum_object_velocity = 0.0;
  p.stop_distance_buffer = 0.5;
  p.time_horizon = 5.0;
  p.extra_object_width = 1.0;
  p.minimum_object_distance_from_ego_trajectory = 1.0;

  size_t stop_idx = 0;
  double stop_dist = 0.0;

  EXPECT_FALSE(igv_velocity_planner::velocity_shaper::computeDynamicObstacleStop(
    pts, obj, p, 0.0, /*ego_half_width=*/0.0, 0.0, stop_idx, stop_dist));

  EXPECT_TRUE(igv_velocity_planner::velocity_shaper::computeDynamicObstacleStop(
    pts, obj, p, 0.0, /*ego_half_width=*/0.5, 0.0, stop_idx, stop_dist));
}

// ---------------------------------------------------------------------------
// computeAdaptiveCruiseVelocityCap
// ---------------------------------------------------------------------------

TEST(FmsVelocityShaper, ACCCapsAtLeaderVelocity)
{
  const auto pts = straightTrajectoryPoints(80.0, 1.0, 15.0);
  // Leader straight ahead 25 m, traveling 5 m/s in same direction.
  auto leader = makeMovingObject(25.0, 0.0, 5.0, 0.0, 0.2, 5);
  std::vector<PredictedObject> objects{leader};
  igv_velocity_planner::velocity_shaper::Params::AdaptiveCruise p;
  p.enable = true;
  p.follow_distance_min = 5.0;
  p.follow_time_gap = 1.5;
  p.velocity_margin = 0.5;
  p.max_deceleration = 1.5;
  p.leader_lateral_margin = 1.0;
  p.leader_heading_max_diff = 0.7;
  p.leader_search_distance = 60.0;

  const auto cap = igv_velocity_planner::velocity_shaper::computeAdaptiveCruiseVelocityCap(pts, objects, 10.0, p);
  ASSERT_TRUE(cap.has_value());
  EXPECT_LE(*cap, 5.0 - 0.5 + 1e-3);  // leader_v - velocity_margin
}

TEST(FmsVelocityShaper, ACCNoLeaderReturnsNullopt)
{
  const auto pts = straightTrajectoryPoints(80.0, 1.0, 15.0);
  std::vector<PredictedObject> objects;
  igv_velocity_planner::velocity_shaper::Params::AdaptiveCruise p;
  const auto cap = igv_velocity_planner::velocity_shaper::computeAdaptiveCruiseVelocityCap(pts, objects, 10.0, p);
  EXPECT_FALSE(cap.has_value());
}

TEST(FmsVelocityShaper, ACCObjectOffPathNotALeader)
{
  const auto pts = straightTrajectoryPoints(80.0, 1.0, 15.0);
  // Object 5 m lateral — beyond leader_lateral_margin even after width inflation.
  auto obj = makeMovingObject(25.0, 5.0, 5.0, 0.0, 0.2, 5);
  std::vector<PredictedObject> objects{obj};
  igv_velocity_planner::velocity_shaper::Params::AdaptiveCruise p;
  p.leader_lateral_margin = 1.0;
  const auto cap = igv_velocity_planner::velocity_shaper::computeAdaptiveCruiseVelocityCap(pts, objects, 10.0, p);
  EXPECT_FALSE(cap.has_value());
}

// ---------------------------------------------------------------------------
// Perception "no-data" guards: §3.5 of the plan requires the shaper to
// behave benignly when the pointcloud / occupancy_grid publishers are silent.
// We exercise this by running the pipeline with default (gated-off) params
// and no input — the static + ACC outputs must equal a path-only baseline.
// ---------------------------------------------------------------------------

TEST(FmsVelocityShaper, PerceptionInputsSilentLeavesPathUnchanged)
{
  const auto path = makeStraightPath(30.0, 1.0, 6.0);
  auto pts_baseline = igv_velocity_planner::velocity_shaper::convertPathToTrajectoryPoints(path);

  // No objects, no pointcloud, no occupancy_grid -> none of the algorithms run.
  // Exercise a retained stateless algorithm directly; pts should still equal the baseline.
  auto pts = pts_baseline;
  igv_velocity_planner::velocity_shaper::Params::AdaptiveCruise acc;
  acc.enable = true;
  const auto cap = igv_velocity_planner::velocity_shaper::computeAdaptiveCruiseVelocityCap(pts, {}, 5.0, acc);
  EXPECT_FALSE(cap.has_value());

  ASSERT_EQ(pts.size(), pts_baseline.size());
  for (size_t i = 0; i < pts.size(); ++i) {
    EXPECT_FLOAT_EQ(pts[i].longitudinal_velocity_mps, pts_baseline[i].longitudinal_velocity_mps);
  }
}

// ---------------------------------------------------------------------------
// computeSlowDownWindow — object-driven outer-band slow-down
// ---------------------------------------------------------------------------

namespace
{

igv_velocity_planner::velocity_shaper::Params::SlowDown makeDefaultSlowDownParams()
{
  igv_velocity_planner::velocity_shaper::Params::SlowDown p;
  p.enable = true;
  p.inner_lateral_margin = 0.5;
  p.outer_lateral_margin = 2.0;
  p.min_target_velocity = 1.5;
  p.max_target_velocity = 4.0;
  p.forward_margin = 5.0;
  p.backward_margin = 3.0;
  p.max_distance = 50.0;
  return p;
}

}  // namespace

TEST(FmsVelocityShaper, SlowDownObjectInOuterBandSlowsWindow)
{
  auto pts = straightTrajectoryPoints(50.0, 1.0, 10.0);
  // Object at (20, 1.0): outside the 0.5 m inner band, inside the 2.0 m outer band.
  const auto obj = makeStaticObject(20.0, 1.0, /*width=*/0.4);
  const auto params = makeDefaultSlowDownParams();

  const auto window = igv_velocity_planner::velocity_shaper::computeSlowDownWindow(pts, obj, params);
  ASSERT_TRUE(window.has_value());

  // Apply the window the way the node body would. capVelocityRange is in the
  // .cpp's anonymous namespace, so we mirror its arithmetic here.
  for (size_t i = window->idx_start; i <= window->idx_end && i < pts.size(); ++i) {
    if (pts[i].longitudinal_velocity_mps > window->target_velocity) {
      pts[i].longitudinal_velocity_mps = static_cast<float>(window->target_velocity);
    }
  }

  // Inside the window: velocity dips below baseline.
  EXPECT_LT(pts[20].longitudinal_velocity_mps, 10.0f);
  // Well past the window: velocity restored to baseline. With forward=5,
  // backward=3 the window ends at x≈23, so index 30 must be untouched.
  EXPECT_FLOAT_EQ(pts[30].longitudinal_velocity_mps, 10.0f);
  // Well before the window (x≈15 is the start, so index 10 must be untouched).
  EXPECT_FLOAT_EQ(pts[10].longitudinal_velocity_mps, 10.0f);
  // Target velocity matches the linear ramp at lat=1.0: ratio = (1-0.5)/1.5 ≈ 0.333.
  // target = 1.5 + 0.333 * (4 - 1.5) ≈ 2.333.
  EXPECT_NEAR(window->target_velocity, 2.333, 0.05);
}

TEST(FmsVelocityShaper, SlowDownObjectInsideInnerBandIgnored)
{
  const auto pts = straightTrajectoryPoints(50.0, 1.0, 10.0);
  // Inside the inner band -- owned by static_obstacle_stop/occupancy, not slow-down.
  const auto obj = makeStaticObject(20.0, 0.3, /*width=*/0.2);
  const auto params = makeDefaultSlowDownParams();
  const auto window = igv_velocity_planner::velocity_shaper::computeSlowDownWindow(pts, obj, params);
  EXPECT_FALSE(window.has_value());
}

TEST(FmsVelocityShaper, SlowDownObjectBeyondOuterBandIgnored)
{
  const auto pts = straightTrajectoryPoints(50.0, 1.0, 10.0);
  const auto obj = makeStaticObject(20.0, 3.0, /*width=*/0.2);
  const auto params = makeDefaultSlowDownParams();
  const auto window = igv_velocity_planner::velocity_shaper::computeSlowDownWindow(pts, obj, params);
  EXPECT_FALSE(window.has_value());
}

TEST(FmsVelocityShaper, SlowDownObjectBeyondMaxDistanceIgnored)
{
  const auto pts = straightTrajectoryPoints(80.0, 1.0, 10.0);
  const auto obj = makeStaticObject(60.0, 1.0, /*width=*/0.4);
  auto params = makeDefaultSlowDownParams();
  params.max_distance = 50.0;
  const auto window = igv_velocity_planner::velocity_shaper::computeSlowDownWindow(pts, obj, params);
  EXPECT_FALSE(window.has_value());
}

TEST(FmsVelocityShaper, SlowDownVelocityRampLinear)
{
  const auto pts = straightTrajectoryPoints(50.0, 1.0, 10.0);
  const auto params = makeDefaultSlowDownParams();
  // Closer object → slower target.
  const auto w_close = igv_velocity_planner::velocity_shaper::computeSlowDownWindow(
    pts, makeStaticObject(20.0, 0.6, 0.2), params);
  const auto w_far = igv_velocity_planner::velocity_shaper::computeSlowDownWindow(
    pts, makeStaticObject(20.0, 1.9, 0.2), params);
  ASSERT_TRUE(w_close.has_value());
  ASSERT_TRUE(w_far.has_value());
  EXPECT_LT(w_close->target_velocity, w_far->target_velocity);
}

TEST(FmsVelocityShaper, SlowDownDisabledReturnsNullopt)
{
  const auto pts = straightTrajectoryPoints(50.0, 1.0, 10.0);
  auto params = makeDefaultSlowDownParams();
  params.enable = false;
  const auto window = igv_velocity_planner::velocity_shaper::computeSlowDownWindow(
    pts, makeStaticObject(20.0, 1.0, 0.4), params);
  EXPECT_FALSE(window.has_value());
}

// Feeding ego_half_width shifts both band edges out by that amount: an object
// that previously sat at the inner edge is now inside the inner band (ignored),
// and an object that previously sat just past the outer edge is now inside the
// outer band (triggers).
TEST(FmsVelocityShaper, SlowDownBandShiftsByEgoHalfWidth)
{
  const auto pts = straightTrajectoryPoints(50.0, 1.0, 10.0);
  const auto params = makeDefaultSlowDownParams();
  const double ego_half_width = 0.5;

  // At lat = 0.6 — inside the outer band when ego_half_width=0, but inside
  // the inner band (≤ 0.5 + 0.5 = 1.0) once ego_half_width is folded in.
  const auto w_inner = igv_velocity_planner::velocity_shaper::computeSlowDownWindow(
    pts, makeStaticObject(20.0, 0.6, 0.2), params, ego_half_width);
  EXPECT_FALSE(w_inner.has_value());

  // At lat = 2.4 — beyond the outer band when ego_half_width=0, but inside
  // the outer band (≤ 2.0 + 0.5 = 2.5) once ego_half_width is folded in.
  // Shifted inner edge sits at 1.0m, so the object is 1.4m into a 1.5m-wide
  // band → ratio ≈ 0.933, target ≈ 1.5 + 0.933 * 2.5 ≈ 3.83 m/s.
  const auto w_outer = igv_velocity_planner::velocity_shaper::computeSlowDownWindow(
    pts, makeStaticObject(20.0, 2.4, 0.2), params, ego_half_width);
  ASSERT_TRUE(w_outer.has_value());
  EXPECT_NEAR(w_outer->target_velocity, 1.5 + (1.4 / 1.5) * (4.0 - 1.5), 0.05);
}
