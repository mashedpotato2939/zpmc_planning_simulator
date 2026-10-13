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

#ifndef FMS_TRAJECTORY_BRIDGE__PATH_GENERATOR_HPP_
#define FMS_TRAJECTORY_BRIDGE__PATH_GENERATOR_HPP_

#include <vector>
#include <cmath>
#include <algorithm>

#include <geometry_msgs/msg/pose.hpp>
#include <autoware_planning_msgs/msg/path_point.hpp>

namespace igv_path_planner
{

/**
 * @brief Maneuver type detected between two FMS waypoints
 */
enum class ManeuverType
{
  STRAIGHT,      // No significant lateral or yaw change
  LANE_CHANGE,   // Significant lateral offset, small yaw change at endpoints
  OBLIQUE_LANE_CHANGE,  // Crab-like lane change with constant heading using 4WS
  TURN,          // Significant yaw change (intersection turn)
  CURVE,         // Gradual curve (road curvature)
  U_TURN,        // Very large yaw change (>150 degrees)
  Q_TURN         // 90-degree clothoid turn (variable curvature)
};

/**
 * @brief Segment information for path generation
 */
struct PathSegment
{
  size_t start_idx;
  size_t end_idx;
  ManeuverType type;
  double lateral_offset;  // For lane change
  double yaw_change;      // For turns
  double arc_length;      // Estimated arc length
  uint8_t lane_change_type;  // 0=none, 1=oblique, 2=s-curve
};

/**
 * @brief Path generator for creating dense paths from sparse FMS waypoints
 */
class PathGenerator
{
public:
  struct Parameters
  {
    // Maneuver detection thresholds
    double lane_change_lateral_threshold = 1.5;   // meters - lateral offset to detect lane change
    double turn_yaw_threshold = 0.3;              // radians (~17 deg) - yaw change to detect turn
    double curve_yaw_threshold = 0.1;             // radians (~6 deg) - yaw change to detect curve
    double u_turn_yaw_threshold = 2.6;            // radians (~150 deg) - yaw change for U-turn

    // Path generation parameters
    double path_interval = 0.5;                   // meters - output path point interval
    double min_lane_change_length = 10.0;         // meters - minimum lane change distance
    double lane_change_time = 4.0;                // seconds - typical lane change duration

    // Turn parameters
    double min_turn_radius = 5.0;                 // meters - minimum turning radius
    double turn_speed = 3.0;                      // m/s - speed during turns

    // Q-Turn parameters (90-degree Fresnel-clothoid turns)
    double qturn_yaw_min_threshold = 1.3;         // radians (~75 deg) - min yaw for Q_TURN detection
    double qturn_yaw_max_threshold = 1.9;         // radians (~109 deg) - max yaw for Q_TURN detection
    double qturn_min_radius = 15.5;               // meters - apex radius (R) for position clothoid (matches map_traj_server)
    double qturn_yaw_min_radius = 8.1;            // meters - secondary radius for yaw/heading clothoid (4WS sideslip)
    double qturn_speed_limit = 3.0;               // m/s - max speed during Q-turns

    // Velocity parameters
    double default_velocity = 5.0;                // m/s
    double lane_change_velocity_ratio = 0.8;      // Reduce speed during lane change

    // 4WS parameters - calculate lateral_velocity from curvature
    bool enable_4ws = true;                       // Enable 4WS lateral velocity calculation
    double vehicle_wheel_base = 6.0;              // IGV wheelbase (meters)
    double vehicle_length = 15.0;                 // IGV total length (meters)
    double vehicle_front_overhang = 4.5;          // Front overhang (meters)
    double max_lateral_velocity = 2.0;            // Max lateral velocity (m/s) - allows ~34° sideslip at 3m/s
    double curvature_4ws_threshold = 0.01;        // Min curvature for 4WS (1/m)
    double lateral_velocity_gain = 3.5;           // Gain factor for 4WS lateral velocity calculation
    double rear_steer_ratio = 0.5;                // k: rear wheel counter-steer ratio for 4WS physics

    // Oblique (crab) lane change parameters
    double oblique_speed_limit = 3.0;             // m/s - max speed during oblique lane changes
    double oblique_max_lateral_distance = 8.0;    // meters - max lateral displacement (maneuver gating)

    // Crab-walking piecewise-parabolic lateral profile (ported from map_traj_server)
    double crab_dymax = 30.0 * M_PI / 180.0;      // max |dy/dx| (radians, ~crab angle limit)
    double crab_ddymax = 30.0 * M_PI / 180.0;     // max |d^2y/dx^2| (rad/m)
    double crab_ddymin = 0.0;                     // feasibility threshold on |d^2y/dx^2|

    // Debug
    bool debug = false;                           // Enable debug output to stderr
  };

  explicit PathGenerator(const Parameters & params);
  PathGenerator();  // Uses default Parameters

  /**
   * @brief Generate dense path from sparse FMS waypoints
   */
  std::vector<autoware_planning_msgs::msg::PathPoint> generatePath(
    const std::vector<autoware_planning_msgs::msg::PathPoint> & sparse_points);

  /**
   * @brief Generate dense path from sparse FMS waypoints with lane change types
   * @param sparse_points Sparse waypoints
   * @param lane_change_types Vector of lane change types (0=none, 1=oblique, 2=s-curve)
   */
  std::vector<autoware_planning_msgs::msg::PathPoint> generatePath(
    const std::vector<autoware_planning_msgs::msg::PathPoint> & sparse_points,
    const std::vector<uint8_t> & lane_change_types);

  /**
   * @brief Get the primary maneuver type detected in the last generatePath call
   * @return The maneuver type of the first (primary) segment
   */
  ManeuverType getLastPrimaryManeuver() const { return last_primary_maneuver_; }

  /**
   * @brief Whether the most recent generateQTurnSegment() hit its internal
   *        fallback (yaw outside window or Fresnel solver failure). Callers
   *        needing a true clothoid should check this after each call.
   */
  bool lastQTurnFellBack() const { return last_qturn_fell_back_; }

  /**
   * @brief Access parameters (thresholds, radii) so callers can gate on
   *        Q_TURN detection windows without re-declaring the constants.
   */
  const Parameters & getParameters() const { return params_; }

  /**
   * @brief Set whether the vehicle is currently reverse-driving. Mirrors the
   *        latest value published on /planning/reverse_state. Callers should
   *        sync this whenever their tracked reverse state changes; the
   *        generator caches it and consults it inside maneuver generators
   *        that need to distinguish motion direction from vehicle facing
   *        (currently: oblique crab walking).
   */
  void setReverseState(bool is_reverse) { is_reverse_ = is_reverse; }
  bool getReverseState() const { return is_reverse_; }

  /**
   * @brief Apply 4WS lateral velocity to path points based on curvature
   * @param points Path points to update
   *
   * This public method allows external callers (like fms_path_converter_node)
   * to apply 4WS lateral_velocity after their own interpolation, when
   * intelligent path generation is disabled.
   */
  void apply4WSLateralVelocity(
    std::vector<autoware_planning_msgs::msg::PathPoint> & points) const;

  /**
   * @brief Generate points for oblique (crab-like) lane change
   *
   * Creates a three-phase path: entry arc, diagonal middle, exit arc.
   * Vehicle heading stays constant while movement is diagonal.
   * Uses lateral_velocity for 4WS crab steering.
   *
   * @param start Start waypoint
   * @param end End waypoint
   * @return Dense path points with lateral_velocity set for crab steering
   *
   * Reverse driving is handled via setReverseState(): when reverse is set,
   * the local frame is built around the motion direction (start_yaw + pi)
   * so the crab profile is solved in the same orientation as forward;
   * output point orientations stay at start_yaw (constant vehicle facing).
   */
  std::vector<autoware_planning_msgs::msg::PathPoint> generateObliqueLaneChangeSegment(
    const autoware_planning_msgs::msg::PathPoint & start,
    const autoware_planning_msgs::msg::PathPoint & end);

  /**
   * @brief Generate points for a Q-turn (90-degree Fresnel clothoid turn).
   *        Public so callers (e.g. LOCAL_PLAN zone resolution) can invoke it
   *        directly and then inspect lastQTurnFellBack().
   */
  std::vector<autoware_planning_msgs::msg::PathPoint> generateQTurnSegment(
    const autoware_planning_msgs::msg::PathPoint & start,
    const autoware_planning_msgs::msg::PathPoint & end);

private:
  Parameters params_;
  mutable ManeuverType last_primary_maneuver_{ManeuverType::STRAIGHT};  // Store the primary maneuver from last path generation
  mutable bool last_qturn_fell_back_{false};
  bool is_reverse_{false};  // Mirrors /planning/reverse_state via setReverseState()

  /**
   * @brief Detect maneuver type between two waypoints
   * @param lane_change_type 0=none, 1=oblique, 2=s-curve (hint from FMS waypoint)
   */
  ManeuverType detectManeuver(
    const autoware_planning_msgs::msg::PathPoint & p1,
    const autoware_planning_msgs::msg::PathPoint & p2,
    uint8_t lane_change_type = 0);

  /**
   * @brief Segment the sparse path into maneuver segments
   * @param points Sparse waypoints
   * @param lane_change_types Lane change type for each waypoint (0=none, 1=oblique, 2=s-curve)
   */
  std::vector<PathSegment> segmentPath(
    const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
    const std::vector<uint8_t> & lane_change_types = {});

  /**
   * @brief Generate points for a straight segment
   */
  std::vector<autoware_planning_msgs::msg::PathPoint> generateStraightSegment(
    const autoware_planning_msgs::msg::PathPoint & start,
    const autoware_planning_msgs::msg::PathPoint & end);

  /**
   * @brief Generate points for a lane change using quintic polynomial
   */
  std::vector<autoware_planning_msgs::msg::PathPoint> generateLaneChangeSegment(
    const autoware_planning_msgs::msg::PathPoint & start,
    const autoware_planning_msgs::msg::PathPoint & end);

  /**
   * @brief Generate points for a turn using arc interpolation
   */
  std::vector<autoware_planning_msgs::msg::PathPoint> generateTurnSegment(
    const autoware_planning_msgs::msg::PathPoint & start,
    const autoware_planning_msgs::msg::PathPoint & end);

  /**
   * @brief Generate points for a curve segment
   */
  std::vector<autoware_planning_msgs::msg::PathPoint> generateCurveSegment(
    const autoware_planning_msgs::msg::PathPoint & start,
    const autoware_planning_msgs::msg::PathPoint & end);

  // Crab-walking lateral-profile solver and Q-turn Fresnel/clothoid numerics
  // are TU-private to their generator files. See:
  //   src/generators/oblique_generator.cpp  (generateObliqueLaneChangeSegment)
  //   src/generators/qturn_generator.cpp    (generateQTurnSegment)
  // To add a new generator type, add a new .cpp under src/generators/ that
  // defines a new PathGenerator member function and declare it here.

  // Utility functions
  double getYaw(const geometry_msgs::msg::Quaternion & q);
  geometry_msgs::msg::Quaternion createQuaternionFromYaw(double yaw);
  double normalizeAngle(double angle);
  double calcDistance2D(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);
  double calcLateralOffset(
    const autoware_planning_msgs::msg::PathPoint & base,
    const autoware_planning_msgs::msg::PathPoint & target);

  /**
   * @brief Calculate lateral velocity for 4WS from curvature
   * @param curvature Path curvature (1/m)
   * @param longitudinal_velocity Longitudinal velocity (m/s)
   * @return Lateral velocity (m/s) clamped to max_lateral_velocity
   */
  double calculateLateralVelocity(double curvature, double longitudinal_velocity) const;

  /**
   * @brief Calculate curvature at a point using Menger formula (three consecutive points)
   * @param points Vector of path points
   * @param index Index of the point to calculate curvature for
   * @return Curvature (1/m)
   */
  double calculateCurvatureAtIndex(
    const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
    size_t index) const;
};

/**
 * @brief Quintic polynomial for smooth lane change trajectory
 *
 * Generates S-curve with zero velocity and acceleration at endpoints
 * x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
 */
class QuinticPolynomial
{
public:
  QuinticPolynomial(
    double x0, double v0, double a0,  // Initial state
    double x1, double v1, double a1,  // Final state
    double T);                         // Duration

  double position(double t) const;
  double velocity(double t) const;
  double acceleration(double t) const;

private:
  double a0_, a1_, a2_, a3_, a4_, a5_;
  double T_;
};

/**
 * @brief Bezier curve generator for smooth path transitions
 */
class BezierCurve
{
public:
  /**
   * @brief Create cubic Bezier curve for lane change
   * @param start Start point
   * @param end End point
   * @param lateral_offset Lateral offset for control points
   */
  static std::vector<geometry_msgs::msg::Point> generateLaneChangeCurve(
    const geometry_msgs::msg::Point & start,
    const geometry_msgs::msg::Point & end,
    double start_yaw,
    double end_yaw,
    int num_points);

  /**
   * @brief Create cubic Bezier curve for turn
   */
  static std::vector<geometry_msgs::msg::Point> generateTurnCurve(
    const geometry_msgs::msg::Point & start,
    const geometry_msgs::msg::Point & end,
    double start_yaw,
    double end_yaw,
    int num_points);

private:
  static geometry_msgs::msg::Point cubicBezier(
    const geometry_msgs::msg::Point & p0,
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2,
    const geometry_msgs::msg::Point & p3,
    double t);
};

}  // namespace igv_path_planner

#endif  // FMS_TRAJECTORY_BRIDGE__PATH_GENERATOR_HPP_
