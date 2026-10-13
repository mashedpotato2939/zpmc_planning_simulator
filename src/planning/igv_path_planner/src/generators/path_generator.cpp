// Copyright 2024 ZPMC
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

#include "igv_path_planner/path_generator.hpp"

#include <tf2/utils.h>
#include <cmath>
#include <limits>
#include <iostream>
#include <string>

namespace igv_path_planner
{

// ============================================================================
// QuinticPolynomial Implementation
// ============================================================================

QuinticPolynomial::QuinticPolynomial(
  double x0, double v0, double a0,
  double x1, double v1, double a1,
  double T)
: T_(T)
{
  // Solve for coefficients using boundary conditions
  // x(0) = x0, x'(0) = v0, x''(0) = a0
  // x(T) = x1, x'(T) = v1, x''(T) = a1

  a0_ = x0;
  a1_ = v0;
  a2_ = a0 / 2.0;

  const double T2 = T * T;
  const double T3 = T2 * T;
  const double T4 = T3 * T;
  const double T5 = T4 * T;

  // Solve the system of equations for a3, a4, a5
  const double dx = x1 - x0 - v0 * T - 0.5 * a0 * T2;
  const double dv = v1 - v0 - a0 * T;
  const double da = a1 - a0;

  // Matrix solution (simplified)
  a3_ = (20.0 * dx - (8.0 * dv + da * T) * T) / (2.0 * T3);
  a4_ = (-30.0 * dx + (14.0 * dv + 2.0 * da * T) * T) / (2.0 * T4);
  a5_ = (12.0 * dx - (6.0 * dv + da * T) * T) / (2.0 * T5);
}

double QuinticPolynomial::position(double t) const
{
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;
  return a0_ + a1_ * t + a2_ * t2 + a3_ * t3 + a4_ * t4 + a5_ * t5;
}

double QuinticPolynomial::velocity(double t) const
{
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  return a1_ + 2.0 * a2_ * t + 3.0 * a3_ * t2 + 4.0 * a4_ * t3 + 5.0 * a5_ * t4;
}

double QuinticPolynomial::acceleration(double t) const
{
  const double t2 = t * t;
  const double t3 = t2 * t;
  return 2.0 * a2_ + 6.0 * a3_ * t + 12.0 * a4_ * t2 + 20.0 * a5_ * t3;
}

// ============================================================================
// BezierCurve Implementation
// ============================================================================

geometry_msgs::msg::Point BezierCurve::cubicBezier(
  const geometry_msgs::msg::Point & p0,
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3,
  double t)
{
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double mt = 1.0 - t;
  const double mt2 = mt * mt;
  const double mt3 = mt2 * mt;

  geometry_msgs::msg::Point result;
  result.x = mt3 * p0.x + 3.0 * mt2 * t * p1.x + 3.0 * mt * t2 * p2.x + t3 * p3.x;
  result.y = mt3 * p0.y + 3.0 * mt2 * t * p1.y + 3.0 * mt * t2 * p2.y + t3 * p3.y;
  result.z = mt3 * p0.z + 3.0 * mt2 * t * p1.z + 3.0 * mt * t2 * p2.z + t3 * p3.z;
  return result;
}

std::vector<geometry_msgs::msg::Point> BezierCurve::generateLaneChangeCurve(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & end,
  double start_yaw,
  double end_yaw,
  int num_points)
{
  // Calculate control points for smooth S-curve
  const double dist = std::hypot(end.x - start.x, end.y - start.y);
  const double control_dist = dist / 3.0;

  // Control point 1: extend from start in start direction
  geometry_msgs::msg::Point p1;
  p1.x = start.x + control_dist * std::cos(start_yaw);
  p1.y = start.y + control_dist * std::sin(start_yaw);
  p1.z = start.z;

  // Control point 2: extend from end in opposite end direction
  geometry_msgs::msg::Point p2;
  p2.x = end.x - control_dist * std::cos(end_yaw);
  p2.y = end.y - control_dist * std::sin(end_yaw);
  p2.z = end.z;

  // Generate points along Bezier curve
  std::vector<geometry_msgs::msg::Point> points;
  for (int i = 0; i <= num_points; ++i) {
    const double t = static_cast<double>(i) / num_points;
    points.push_back(cubicBezier(start, p1, p2, end, t));
  }

  return points;
}

std::vector<geometry_msgs::msg::Point> BezierCurve::generateTurnCurve(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & end,
  double start_yaw,
  double end_yaw,
  int num_points)
{
  // For turns, use longer control distances to create smoother arc
  const double dist = std::hypot(end.x - start.x, end.y - start.y);

  // Control distance based on turn angle
  double yaw_diff = end_yaw - start_yaw;
  while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
  while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;

  // Larger control distance for sharper turns
  const double control_factor = 0.5 + 0.3 * std::abs(yaw_diff) / M_PI;
  const double control_dist = dist * control_factor;

  geometry_msgs::msg::Point p1;
  p1.x = start.x + control_dist * std::cos(start_yaw);
  p1.y = start.y + control_dist * std::sin(start_yaw);
  p1.z = start.z;

  geometry_msgs::msg::Point p2;
  p2.x = end.x - control_dist * std::cos(end_yaw);
  p2.y = end.y - control_dist * std::sin(end_yaw);
  p2.z = end.z;

  std::vector<geometry_msgs::msg::Point> points;
  for (int i = 0; i <= num_points; ++i) {
    const double t = static_cast<double>(i) / num_points;
    points.push_back(cubicBezier(start, p1, p2, end, t));
  }

  return points;
}

// ============================================================================
// PathGenerator Implementation
// ============================================================================

PathGenerator::PathGenerator(const Parameters & params)
: params_(params)
{
}

PathGenerator::PathGenerator()
: params_()
{
}

double PathGenerator::getYaw(const geometry_msgs::msg::Quaternion & q)
{
  return tf2::getYaw(q);
}

geometry_msgs::msg::Quaternion PathGenerator::createQuaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw / 2.0);
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  return q;
}

double PathGenerator::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double PathGenerator::calcDistance2D(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

double PathGenerator::calcLateralOffset(
  const autoware_planning_msgs::msg::PathPoint & base,
  const autoware_planning_msgs::msg::PathPoint & target)
{
  // Calculate lateral offset of target from base's heading direction
  const double base_yaw = getYaw(base.pose.orientation);
  const double dx = target.pose.position.x - base.pose.position.x;
  const double dy = target.pose.position.y - base.pose.position.y;

  // Lateral offset is perpendicular distance
  return -dx * std::sin(base_yaw) + dy * std::cos(base_yaw);
}

double PathGenerator::calculateLateralVelocity(double curvature, double longitudinal_velocity) const
{
  if (!params_.enable_4ws || std::abs(curvature) < params_.curvature_4ws_threshold ||
      std::abs(longitudinal_velocity) < 0.1)
  {
    return 0.0;
  }

  // Physics-based lateral velocity for 4WS counter-steer mode
  // Vy = v × (L/2) × ((1-k)/(1+k)) × κ
  //
  // For L=6m, k=0.5, κ=0.125 (R=8m), v=2.0 m/s:
  //   Vy = 2.0 × 3.0 × (0.5/1.5) × 0.125 = 0.25 m/s
  //
  // Note: Controller uses pure pursuit for Q-turn steering directly,
  //       but this value is set on trajectory points for logging/consumers.

  const double L = params_.vehicle_wheel_base;
  const double k = params_.rear_steer_ratio;
  const double physics_gain = (L / 2.0) * ((1.0 - k) / (1.0 + k));

  double lat_vel = curvature * longitudinal_velocity * physics_gain;
  lat_vel = std::clamp(lat_vel, -params_.max_lateral_velocity, params_.max_lateral_velocity);

  return lat_vel;
}

double PathGenerator::calculateCurvatureAtIndex(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  size_t index) const
{
  // Use Menger curvature formula: k = 4 * Area(triangle) / (|AB| * |BC| * |CA|)
  if (points.size() < 3) {
    return 0.0;
  }

  size_t i0, i1, i2;
  if (index == 0) {
    i0 = 0; i1 = 1; i2 = 2;
  } else if (index >= points.size() - 1) {
    i0 = points.size() - 3;
    i1 = points.size() - 2;
    i2 = points.size() - 1;
  } else {
    i0 = index - 1;
    i1 = index;
    i2 = index + 1;
  }

  const auto & p0 = points[i0].pose.position;
  const auto & p1 = points[i1].pose.position;
  const auto & p2 = points[i2].pose.position;

  // Calculate side lengths
  const double a = std::hypot(p1.x - p0.x, p1.y - p0.y);  // |AB|
  const double b = std::hypot(p2.x - p1.x, p2.y - p1.y);  // |BC|
  const double c = std::hypot(p2.x - p0.x, p2.y - p0.y);  // |CA|

  // Avoid division by zero for collinear or coincident points
  if (a < 1e-6 || b < 1e-6 || c < 1e-6) {
    return 0.0;
  }

  // Calculate area using cross product: Area = 0.5 * |AB x AC|
  const double cross = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
  const double area = 0.5 * std::abs(cross);

  // Menger curvature: k = 4 * Area / (a * b * c)
  const double curvature = 4.0 * area / (a * b * c);

  // Determine sign based on cross product (positive = left turn, negative = right turn)
  return (cross >= 0) ? curvature : -curvature;
}

void PathGenerator::apply4WSLateralVelocity(
  std::vector<autoware_planning_msgs::msg::PathPoint> & points) const
{
  if (!params_.enable_4ws || points.size() < 3) {
    return;
  }

  for (size_t i = 0; i < points.size(); ++i) {
    // Don't override lateral_velocity if already set (e.g., by oblique lane change)
    if (std::abs(points[i].lateral_velocity_mps) > 0.001) {
      continue;
    }
    const double curvature = calculateCurvatureAtIndex(points, i);
    const double v_x = static_cast<double>(points[i].longitudinal_velocity_mps);
    points[i].lateral_velocity_mps = static_cast<float>(calculateLateralVelocity(curvature, v_x));
  }

  if (params_.debug) {
    // Log 4WS statistics only in debug mode
    double max_lat_vel = 0.0;
    double max_curvature = 0.0;
    int nonzero_lat_vel_count = 0;
    for (size_t i = 0; i < points.size(); ++i) {
      const double lat_vel = std::abs(points[i].lateral_velocity_mps);
      max_lat_vel = std::max(max_lat_vel, lat_vel);
      max_curvature = std::max(max_curvature, std::abs(calculateCurvatureAtIndex(points, i)));
      if (lat_vel > 0.001) nonzero_lat_vel_count++;
    }
    std::cerr << "[PathGen] 4WS applied: max_curvature=" << max_curvature
              << " (threshold=" << params_.curvature_4ws_threshold << ")"
              << ", max_lat_vel=" << max_lat_vel
              << ", nonzero_count=" << nonzero_lat_vel_count << "/" << points.size()
              << ", rear_factor=" << ((params_.vehicle_length - params_.vehicle_front_overhang - params_.vehicle_wheel_base) / params_.vehicle_wheel_base)
              << std::endl;
  }
}

ManeuverType PathGenerator::detectManeuver(
  const autoware_planning_msgs::msg::PathPoint & p1,
  const autoware_planning_msgs::msg::PathPoint & p2,
  uint8_t lane_change_type)
{
  const double yaw1 = getYaw(p1.pose.orientation);
  const double yaw2 = getYaw(p2.pose.orientation);
  const double yaw_diff = std::abs(normalizeAngle(yaw2 - yaw1));

  const double lateral_offset = std::abs(calcLateralOffset(p1, p2));
  const double dist = calcDistance2D(p1.pose.position, p2.pose.position);

  if (params_.debug) {
    std::cerr << "[PathGen] detectManeuver: p1=(" << p1.pose.position.x << "," << p1.pose.position.y
              << ") p2=(" << p2.pose.position.x << "," << p2.pose.position.y << ")"
              << " dist=" << dist << " lateral=" << lateral_offset
              << " yaw1=" << yaw1 << " yaw2=" << yaw2 << " yaw_diff=" << yaw_diff
              << " lane_change_type=" << static_cast<int>(lane_change_type)
              << " (threshold=" << params_.lane_change_lateral_threshold << ")"
              << std::endl;
  }

  // Oblique lane change detection - explicit type from FMS takes priority
  // Trust the lane_change_type flag without requiring lateral offset threshold,
  // because FMS may send dense waypoints where lateral offset between consecutive
  // points is small even though the total lane change is significant.
  constexpr uint8_t LANE_CHANGE_OBLIQUE = 1;
  if (lane_change_type == LANE_CHANGE_OBLIQUE) {
    // Only enforce max lateral distance limit (for safety)
    if (lateral_offset > params_.oblique_max_lateral_distance) {
      if (params_.debug) {
        std::cerr << "[PathGen]   -> OBLIQUE exceeds max distance (" << lateral_offset
                  << " > " << params_.oblique_max_lateral_distance << "), using LANE_CHANGE" << std::endl;
      }
      return ManeuverType::LANE_CHANGE;
    }
    if (params_.debug) std::cerr << "[PathGen]   -> OBLIQUE_LANE_CHANGE (explicit)" << std::endl;
    return ManeuverType::OBLIQUE_LANE_CHANGE;
  }

  // U-turn detection
  if (yaw_diff > params_.u_turn_yaw_threshold) {
    if (params_.debug) std::cerr << "[PathGen]   -> U_TURN" << std::endl;
    return ManeuverType::U_TURN;
  }

  // Q-turn detection — strict 90° elbow window only.
  // Fresnel-clothoid algorithm is only geometrically valid for ~90° turns.
  if (yaw_diff > params_.qturn_yaw_min_threshold &&
      yaw_diff < params_.qturn_yaw_max_threshold) {
    if (params_.debug) std::cerr << "[PathGen]   -> Q_TURN (yaw_diff=" << yaw_diff << " rad)" << std::endl;
    return ManeuverType::Q_TURN;
  }

  // Non-90° significant yaw changes → Bezier TURN.
  if (yaw_diff > params_.turn_yaw_threshold) {
    if (params_.debug) std::cerr << "[PathGen]   -> TURN (yaw_diff=" << yaw_diff << " rad)" << std::endl;
    return ManeuverType::TURN;
  }

  // Lane change detection
  // Large lateral offset with small yaw change at endpoints
  if (lateral_offset > params_.lane_change_lateral_threshold &&
      yaw_diff < params_.turn_yaw_threshold) {
    if (params_.debug) std::cerr << "[PathGen]   -> LANE_CHANGE" << std::endl;
    return ManeuverType::LANE_CHANGE;
  }

  // Curve detection
  if (yaw_diff > params_.curve_yaw_threshold) {
    if (params_.debug) std::cerr << "[PathGen]   -> CURVE" << std::endl;
    return ManeuverType::CURVE;
  }

  if (params_.debug) std::cerr << "[PathGen]   -> STRAIGHT" << std::endl;
  return ManeuverType::STRAIGHT;
}

std::vector<PathSegment> PathGenerator::segmentPath(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
  const std::vector<uint8_t> & lane_change_types)
{
  std::vector<PathSegment> segments;

  if (points.size() < 2) {
    return segments;
  }

  // First pass: create raw segments
  // Then merge consecutive segments with the same non-zero lane_change_type
  // This is important because FMS may send dense waypoints where consecutive
  // points have small lateral offset, but they represent a single lane change maneuver.

  size_t i = 0;
  while (i + 1 < points.size()) {
    uint8_t lc_type = (!lane_change_types.empty() && i < lane_change_types.size())
      ? lane_change_types[i] : 0;

    // Find the end index of this segment
    // For lane change segments, merge consecutive waypoints with the same type
    size_t end_idx = i + 1;  // Default: just next waypoint
    if (lc_type != 0) {
      // Look for consecutive waypoints with the same lane_change_type
      // Continue while the NEXT waypoint has the same type (so end_idx stays valid)
      while (end_idx < points.size() - 1 &&
             !lane_change_types.empty() &&
             end_idx < lane_change_types.size() &&
             lane_change_types[end_idx] == lc_type) {
        end_idx++;
      }
      // end_idx is now the last waypoint in the lane change sequence
    }

    PathSegment seg;
    seg.start_idx = i;
    seg.end_idx = end_idx;  // Inclusive - points to actual end waypoint
    seg.lane_change_type = lc_type;

    // Detect maneuver using the full segment (start to end)
    seg.type = detectManeuver(points[i], points[end_idx], lc_type);

    // Calculate properties for the full segment
    seg.lateral_offset = calcLateralOffset(points[i], points[end_idx]);
    seg.yaw_change = normalizeAngle(
      getYaw(points[end_idx].pose.orientation) - getYaw(points[i].pose.orientation));
    seg.arc_length = calcDistance2D(points[i].pose.position, points[end_idx].pose.position);

    if (params_.debug && lc_type != 0) {
      std::cerr << "[PathGen] segmentPath: merged lane change segment [" << i << "->" << end_idx << "]"
                << " lc_type=" << static_cast<int>(lc_type)
                << " lateral=" << seg.lateral_offset
                << " arc_len=" << seg.arc_length << std::endl;
    }

    segments.push_back(seg);

    // Move to the end of this segment for the next iteration
    i = end_idx;
  }

  return segments;
}

std::vector<autoware_planning_msgs::msg::PathPoint> PathGenerator::generateStraightSegment(
  const autoware_planning_msgs::msg::PathPoint & start,
  const autoware_planning_msgs::msg::PathPoint & end)
{
  std::vector<autoware_planning_msgs::msg::PathPoint> points;

  const double dist = calcDistance2D(start.pose.position, end.pose.position);
  const int num_points = std::max(2, static_cast<int>(dist / params_.path_interval));

  // Check if end point is a goal (velocity near zero, accounting for reverse driving)
  const bool end_is_goal = (std::abs(end.longitudinal_velocity_mps) < 0.1);

  for (int i = 0; i <= num_points; ++i) {
    const double ratio = static_cast<double>(i) / num_points;

    autoware_planning_msgs::msg::PathPoint pt;

    // Linear interpolation
    pt.pose.position.x = start.pose.position.x + ratio * (end.pose.position.x - start.pose.position.x);
    pt.pose.position.y = start.pose.position.y + ratio * (end.pose.position.y - start.pose.position.y);
    pt.pose.position.z = start.pose.position.z + ratio * (end.pose.position.z - start.pose.position.z);

    // Interpolate yaw
    const double start_yaw = getYaw(start.pose.orientation);
    const double end_yaw = getYaw(end.pose.orientation);
    double yaw_diff = normalizeAngle(end_yaw - start_yaw);
    const double interp_yaw = start_yaw + ratio * yaw_diff;
    pt.pose.orientation = createQuaternionFromYaw(interp_yaw);

    // Velocity: maintain start velocity, only set zero at very last point if end is goal
    if (i == num_points && end_is_goal) {
      pt.longitudinal_velocity_mps = 0.0;
    } else {
      pt.longitudinal_velocity_mps = start.longitudinal_velocity_mps;
    }
    pt.lateral_velocity_mps = 0.0;
    pt.heading_rate_rps = 0.0;
    pt.is_final = (i == num_points);

    points.push_back(pt);
  }

  return points;
}

std::vector<autoware_planning_msgs::msg::PathPoint> PathGenerator::generateLaneChangeSegment(
  const autoware_planning_msgs::msg::PathPoint & start,
  const autoware_planning_msgs::msg::PathPoint & end)
{
  std::vector<autoware_planning_msgs::msg::PathPoint> points;

  const double start_yaw = getYaw(start.pose.orientation);
  const double end_yaw = getYaw(end.pose.orientation);
  const double dist = calcDistance2D(start.pose.position, end.pose.position);

  // Ensure minimum lane change length
  const double lane_change_length = std::max(dist, params_.min_lane_change_length);
  const int num_points = std::max(10, static_cast<int>(lane_change_length / params_.path_interval));

  // Use Bezier curve for smooth S-curve
  auto bezier_points = BezierCurve::generateLaneChangeCurve(
    start.pose.position, end.pose.position,
    start_yaw, end_yaw, num_points);

  // Also generate lateral profile using quintic polynomial for velocity
  const double lateral_offset = calcLateralOffset(start, end);
  QuinticPolynomial lat_poly(0.0, 0.0, 0.0, lateral_offset, 0.0, 0.0, 1.0);

  // Use start velocity for lane change - don't interpolate toward zero
  // Only set velocity=0 at the very last point if end is a goal
  const bool end_is_goal = (std::abs(end.longitudinal_velocity_mps) < 0.1);
  const double lane_change_vel = start.longitudinal_velocity_mps * params_.lane_change_velocity_ratio;

  for (size_t i = 0; i < bezier_points.size(); ++i) {
    autoware_planning_msgs::msg::PathPoint pt;
    pt.pose.position = bezier_points[i];

    // Calculate yaw from direction to next point
    if (i + 1 < bezier_points.size()) {
      const double dx = bezier_points[i + 1].x - bezier_points[i].x;
      const double dy = bezier_points[i + 1].y - bezier_points[i].y;
      pt.pose.orientation = createQuaternionFromYaw(std::atan2(dy, dx));
    } else {
      pt.pose.orientation = end.pose.orientation;
    }

    // Velocity: maintain lane change velocity, don't ramp to zero
    // Only the very last point gets zero velocity if end is goal
    if (i == bezier_points.size() - 1 && end_is_goal) {
      pt.longitudinal_velocity_mps = 0.0;
    } else {
      pt.longitudinal_velocity_mps = lane_change_vel;
    }

    pt.lateral_velocity_mps = 0.0;
    pt.heading_rate_rps = 0.0;
    pt.is_final = (i == bezier_points.size() - 1);

    points.push_back(pt);
  }

  return points;
}

// generateObliqueLaneChangeSegment is implemented in
// src/generators/oblique_generator.cpp (along with its piecewise-parabolic
// lateral-profile solver).

std::vector<autoware_planning_msgs::msg::PathPoint> PathGenerator::generateTurnSegment(
  const autoware_planning_msgs::msg::PathPoint & start,
  const autoware_planning_msgs::msg::PathPoint & end)
{
  std::vector<autoware_planning_msgs::msg::PathPoint> points;

  const double start_yaw = getYaw(start.pose.orientation);
  const double end_yaw = getYaw(end.pose.orientation);
  const double dist = calcDistance2D(start.pose.position, end.pose.position);

  const int num_points = std::max(10, static_cast<int>(dist / params_.path_interval));

  // Use Bezier curve for smooth turn
  auto bezier_points = BezierCurve::generateTurnCurve(
    start.pose.position, end.pose.position,
    start_yaw, end_yaw, num_points);

  // Check if end point is a goal (velocity near zero, accounting for reverse driving)
  const bool end_is_goal = (std::abs(end.longitudinal_velocity_mps) < 0.1);
  // Limit turn speed while preserving direction sign
  const double start_vel = start.longitudinal_velocity_mps;
  const double turn_vel = (start_vel >= 0)
    ? std::min(static_cast<double>(start_vel), params_.turn_speed)
    : std::max(static_cast<double>(start_vel), -params_.turn_speed);

  for (size_t i = 0; i < bezier_points.size(); ++i) {
    autoware_planning_msgs::msg::PathPoint pt;
    pt.pose.position = bezier_points[i];

    // Calculate yaw from direction to next point
    if (i + 1 < bezier_points.size()) {
      const double dx = bezier_points[i + 1].x - bezier_points[i].x;
      const double dy = bezier_points[i + 1].y - bezier_points[i].y;
      pt.pose.orientation = createQuaternionFromYaw(std::atan2(dy, dx));
    } else {
      pt.pose.orientation = end.pose.orientation;
    }

    // Velocity: maintain turn speed, only set zero at very last point if end is goal
    if (i == bezier_points.size() - 1 && end_is_goal) {
      pt.longitudinal_velocity_mps = 0.0;
    } else {
      pt.longitudinal_velocity_mps = turn_vel;
    }

    pt.lateral_velocity_mps = 0.0;
    pt.heading_rate_rps = 0.0;
    pt.is_final = (i == bezier_points.size() - 1);

    points.push_back(pt);
  }

  return points;
}

std::vector<autoware_planning_msgs::msg::PathPoint> PathGenerator::generateCurveSegment(
  const autoware_planning_msgs::msg::PathPoint & start,
  const autoware_planning_msgs::msg::PathPoint & end)
{
  // For curves, use similar approach to turns but with less curvature
  return generateTurnSegment(start, end);
}

// generateQTurnSegment is implemented in src/generators/qturn_generator.cpp
// (along with its Fresnel/clothoid numerics).

std::vector<autoware_planning_msgs::msg::PathPoint> PathGenerator::generatePath(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & sparse_points)
{
  // Call the overload with empty lane change types
  return generatePath(sparse_points, {});
}

std::vector<autoware_planning_msgs::msg::PathPoint> PathGenerator::generatePath(
  const std::vector<autoware_planning_msgs::msg::PathPoint> & sparse_points,
  const std::vector<uint8_t> & lane_change_types)
{
  if (params_.debug) {
    std::cerr << "[PathGen] generatePath called with " << sparse_points.size() << " sparse points" << std::endl;
  }

  if (sparse_points.empty()) {
    return {};
  }

  if (sparse_points.size() == 1) {
    return sparse_points;
  }

  // Segment the path by maneuver type
  auto segments = segmentPath(sparse_points, lane_change_types);

  // Store the primary maneuver type (first segment) for external queries
  if (!segments.empty()) {
    last_primary_maneuver_ = segments[0].type;
  } else {
    last_primary_maneuver_ = ManeuverType::STRAIGHT;
  }

  if (params_.debug) {
    std::cerr << "[PathGen] Detected " << segments.size() << " segments:" << std::endl;
    for (size_t i = 0; i < segments.size(); ++i) {
      std::string type_str;
      switch (segments[i].type) {
        case ManeuverType::STRAIGHT: type_str = "STRAIGHT"; break;
        case ManeuverType::LANE_CHANGE: type_str = "LANE_CHANGE"; break;
        case ManeuverType::OBLIQUE_LANE_CHANGE: type_str = "OBLIQUE_LANE_CHANGE"; break;
        case ManeuverType::TURN: type_str = "TURN"; break;
        case ManeuverType::CURVE: type_str = "CURVE"; break;
        case ManeuverType::U_TURN: type_str = "U_TURN"; break;
        case ManeuverType::Q_TURN: type_str = "Q_TURN"; break;
      }
      std::cerr << "  Segment " << i << ": " << type_str
                << " [" << segments[i].start_idx << "->" << segments[i].end_idx << "]"
                << " lateral=" << segments[i].lateral_offset
                << " lc_type=" << static_cast<int>(segments[i].lane_change_type) << std::endl;
    }
  }

  // Generate dense path for each segment
  std::vector<autoware_planning_msgs::msg::PathPoint> dense_path;

  for (const auto & seg : segments) {
    const auto & start = sparse_points[seg.start_idx];
    const auto & end = sparse_points[seg.end_idx];

    std::vector<autoware_planning_msgs::msg::PathPoint> segment_points;

    switch (seg.type) {
      case ManeuverType::STRAIGHT:
        segment_points = generateStraightSegment(start, end);
        break;

      case ManeuverType::LANE_CHANGE:
        segment_points = generateLaneChangeSegment(start, end);
        break;

      case ManeuverType::OBLIQUE_LANE_CHANGE:
        segment_points = generateObliqueLaneChangeSegment(start, end);
        break;

      case ManeuverType::TURN:
      case ManeuverType::U_TURN:
        segment_points = generateTurnSegment(start, end);
        break;

      case ManeuverType::Q_TURN:
        segment_points = generateQTurnSegment(start, end);
        break;

      case ManeuverType::CURVE:
        segment_points = generateCurveSegment(start, end);
        break;
    }

    // Append to dense path (skip first point if not first segment to avoid duplicates)
    for (size_t i = (dense_path.empty() ? 0 : 1); i < segment_points.size(); ++i) {
      dense_path.push_back(segment_points[i]);
    }
  }

  // Mark last point as final
  if (!dense_path.empty()) {
    dense_path.back().is_final = true;
  }

  // Apply 4WS lateral velocity based on path curvature
  // Skip for oblique lane change - lateral_velocity is already properly set by the generator
  // (including intentional zeros for straight sections)
  if (last_primary_maneuver_ != ManeuverType::OBLIQUE_LANE_CHANGE) {
    apply4WSLateralVelocity(dense_path);
  }

  return dense_path;
}

}  // namespace igv_path_planner
