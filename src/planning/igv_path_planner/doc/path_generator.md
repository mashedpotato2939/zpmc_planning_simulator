# PathGenerator Library

Library for expanding sparse FMS waypoints into dense Autoware `PathPoint` sequences with maneuver-aware geometry and velocities.

## Overview

The `PathGenerator` class takes sparse waypoints (e.g., start and goal) and produces a dense, smooth path suitable for trajectory planning. It automatically detects the type of maneuver between consecutive waypoints and applies appropriate interpolation algorithms.

## Architecture

```
Sparse Waypoints (2-N points)
         │
         ▼
  ┌──────────────────┐
  │  segmentPath()   │  Classify each pair of waypoints
  └────────┬─────────┘
           │
           ▼
  ┌───────────────────────────────────────────────────────────────────────┐
  │                           detectManeuver()                             │
  │                                                                        │
  │  ┌─────────┐  ┌─────────────┐  ┌──────┐  ┌────────┐  ┌───────┐       │
  │  │STRAIGHT │  │LANE_CHANGE  │  │ TURN │  │ Q_TURN │  │ CURVE │  ...  │
  │  └────┬────┘  └──────┬──────┘  └──┬───┘  └───┬────┘  └───┬───┘       │
  └───────┼──────────────┼────────────┼──────────┼───────────┼───────────┘
          │              │            │          │           │
          ▼              ▼            ▼          ▼           ▼
  ┌────────────┐  ┌────────────┐  ┌────────┐  ┌──────────┐  ┌────────┐
  │  Linear    │  │  Bezier +  │  │ Bezier │  │ Fresnel  │  │ Bezier │
  │  Interp    │  │  Quintic   │  │ Curve  │  │ Clothoid │  │ Curve  │
  └─────┬──────┘  └─────┬──────┘  └───┬────┘  └────┬─────┘  └───┬────┘
        │               │             │            │             │
        └───────────────┴─────────────┴────────────┴─────────────┘
                        │
                        ▼
                Dense PathPoints
```

## Maneuver Types

| Type | Detection Criteria | Geometry |
|------|-------------------|----------|
| `STRAIGHT` | yaw_diff < 0.1 rad (~6°), lateral < 1.5m | Linear interpolation |
| `LANE_CHANGE` | lateral > 1.5m, yaw_diff < 0.3 rad (~17°) | Cubic Bezier S-curve |
| `OBLIQUE_LANE_CHANGE` | explicit FMS hint | Piecewise-parabolic crab profile |
| `Q_TURN` | 1.3 rad < yaw_diff < 1.9 rad (~75°-109°) | Fresnel clothoid-arc-clothoid |
| `TURN` | yaw_diff > 0.3 rad outside the Q_TURN window | Cubic Bezier arc |
| `CURVE` | 0.1 rad < yaw_diff < 0.3 rad | Cubic Bezier (same as turn) |
| `U_TURN` | yaw_diff > 2.6 rad (~150°) | Cubic Bezier (same as turn) |

### Q_TURN Detection

The Q_TURN type is specifically designed for 90° elbows common in industrial
vehicle operations. The Fresnel clothoid-arc-clothoid geometry is only valid
for turns close to 90°, so the gate is strict: only yaw changes in
`[qturn_yaw_min_threshold, qturn_yaw_max_threshold]` (default 75°–109°) select
`Q_TURN`. Non-90° turns fall through to `TURN` (Bezier).

## Parameters

```cpp
struct Parameters {
  // Maneuver detection thresholds
  double lane_change_lateral_threshold = 1.5;   // meters
  double turn_yaw_threshold = 0.3;              // radians (~17°)
  double curve_yaw_threshold = 0.1;             // radians (~6°)
  double u_turn_yaw_threshold = 2.6;            // radians (~150°)

  // Path generation
  double path_interval = 0.5;                   // meters between output points
  double min_lane_change_length = 10.0;         // meters
  double lane_change_time = 4.0;                // seconds (unused)

  // Turn parameters
  double min_turn_radius = 5.0;                 // meters
  double turn_speed = 3.0;                      // m/s

  // Q-Turn parameters (90-degree Fresnel-clothoid turns)
  double qturn_yaw_min_threshold = 1.3;         // radians (~75°)  detection window
  double qturn_yaw_max_threshold = 1.9;         // radians (~109°) detection window
  double qturn_min_radius = 8.5;                // meters — apex radius R (1/km_xy)
                                                //          for the position clothoid
  double qturn_yaw_min_radius = 8.1;            // meters — secondary radius (1/km_yaw)
                                                //          for the heading clothoid
                                                //          (produces 4WS sideslip β)
  double qturn_speed_limit = 3.0;               // m/s

  // Oblique (crab) lane change parameters
  double oblique_speed_limit = 3.0;             // m/s

  // Velocity
  double default_velocity = 5.0;                // m/s
  double lane_change_velocity_ratio = 0.8;      // speed reduction during lane change
};
```

## Geometry Algorithms

### Cubic Bezier Curve

Used for lane changes, turns, and curves. The cubic Bezier formula:

```
B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃
```

Where:
- `P₀` = start point
- `P₁` = control point 1 (along start direction)
- `P₂` = control point 2 (along end direction, reversed)
- `P₃` = end point
- `t` = parameter [0, 1]

#### Lane Change Control Points

```cpp
control_dist = distance / 3.0;
P1 = start + control_dist * (cos(start_yaw), sin(start_yaw))
P2 = end - control_dist * (cos(end_yaw), sin(end_yaw))
```

#### Turn Control Points

Control distance scales with turn sharpness:

```cpp
control_factor = 0.5 + 0.3 * |yaw_diff| / π
control_dist = distance * control_factor
```

### Quintic Polynomial

Used for lateral offset profiling during lane changes. Ensures smooth acceleration at boundaries:

```
x(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
```

Boundary conditions:
- `x(0) = x₀`, `x'(0) = v₀`, `x''(0) = a₀` (initial state)
- `x(T) = x₁`, `x'(T) = v₁`, `x''(T) = a₁` (final state)

### Fresnel Clothoid-Arc-Clothoid (Q_TURN)

Q_TURN uses a true Euler spiral built from Fresnel integrals — ported from
`motion_control/map_traj_server::traj_creator_turn_optimize`. The turn consists
of three phases:

```
  entry spiral (L1)    apex arc (L2)    exit spiral (L1)
  κ: 0 → km            κ: km            κ: km → 0
```

where `km = 1 / qturn_min_radius`. `L1` and `L2` are solved by bisection so the
endpoint lands exactly on the target `(dx_local, dy_local)` while producing
a 90° heading change. Fresnel integrals are evaluated via adaptive Simpson
(`eps = 1e-9`, `max_depth = 20`):

```
C(u) = ∫₀ᵘ cos(π/2 · t²) dt
S(u) = ∫₀ᵘ sin(π/2 · t²) dt
```

#### Dual-curvature design (4WS sideslip)

Two clothoids are sampled in parallel:

- A **position clothoid** with `km = 1/qturn_min_radius` drives `(x, y)`
  sampling and produces `yaw_path(s)` — the tangent of the spatial curve.
- A **heading clothoid** with `km_yaw = 1/qturn_yaw_min_radius` on arc length
  `ds_total = 2·L1 + L2` produces `yaw_drive(s)` — the vehicle body heading.

The vehicle heading follows `yaw_drive`, while the body travels the spatial
curve tangent to `yaw_path`. The difference is the 4WS sideslip angle:

```
β(s) = yaw_path(s) - yaw_drive(s)
lateral_velocity_mps = β · vx
heading_rate_rps = vx · κ_drive(s)
```

If the secondary radius is incompatible with the solved `ds_total`
(`L1_yaw ≤ 0` or `L2_yaw ≤ 0`), the generator degrades gracefully to `β = 0`
(heading follows the position clothoid — no sideslip, but geometrically valid).

#### Left/right turns

The source algorithm is hardcoded for left turns. The generator mirrors the
local-frame `y` coordinate on the sign of `yaw_diff_signed`, then un-mirrors
the output heading, `lateral_velocity_mps`, and `heading_rate_rps`, so both
left and right 90° elbows share the same solver.

## Velocity Handling

### Forward Driving

- `STRAIGHT`: Maintains start velocity
- `LANE_CHANGE`: Reduces to `velocity * lane_change_velocity_ratio` (80%)
- `TURN/CURVE`: Caps at `turn_speed` (3.0 m/s)
- `Q_TURN`: Caps at `qturn_speed_limit` (3.0 m/s), constant throughout turn
- `OBLIQUE_LANE_CHANGE`: Caps at `oblique_speed_limit` (3.0 m/s), preserving crab angle
- Goal point: Set to 0.0 m/s

### Reverse Driving

The library preserves velocity sign for reverse driving:

```cpp
// Turn velocity preserves direction
turn_vel = (start_vel >= 0)
  ? min(start_vel, turn_speed)
  : max(start_vel, -turn_speed);
```

- Negative velocities are passed through unchanged
- Goal detection uses `abs(velocity) < 0.1`

## API

### PathGenerator Class

```cpp
class PathGenerator {
public:
  explicit PathGenerator(const Parameters & params);
  PathGenerator();  // Uses default Parameters

  /**
   * @brief Generate dense path from sparse waypoints
   * @param sparse_points Input waypoints (minimum 2)
   * @return Dense path with points at path_interval spacing
   */
  std::vector<autoware_planning_msgs::msg::PathPoint> generatePath(
    const std::vector<autoware_planning_msgs::msg::PathPoint> & sparse_points);
};
```

### BezierCurve Class

Static utility for generating Bezier curves:

```cpp
class BezierCurve {
public:
  static std::vector<geometry_msgs::msg::Point> generateLaneChangeCurve(
    const geometry_msgs::msg::Point & start,
    const geometry_msgs::msg::Point & end,
    double start_yaw, double end_yaw,
    int num_points);

  static std::vector<geometry_msgs::msg::Point> generateTurnCurve(
    const geometry_msgs::msg::Point & start,
    const geometry_msgs::msg::Point & end,
    double start_yaw, double end_yaw,
    int num_points);
};
```

### QuinticPolynomial Class

```cpp
class QuinticPolynomial {
public:
  QuinticPolynomial(
    double x0, double v0, double a0,  // Initial position, velocity, acceleration
    double x1, double v1, double a1,  // Final position, velocity, acceleration
    double T);                         // Duration

  double position(double t) const;
  double velocity(double t) const;
  double acceleration(double t) const;
};
```

### Fresnel-Clothoid Helpers

Q_TURN geometry is implemented as private static methods on `PathGenerator`:

```cpp
// Adaptive-Simpson Fresnel integrals
static double fresnelC(double x);
static double fresnelS(double x);

// Endpoint of clothoid-arc-clothoid given km, L1 (L2 = π/(2·km) - L1)
static double computeEndpointX(double km, double L1);
static double computeEndpointY(double km, double L1);

// Bisection solver: find L1, L2 for a 90° clothoid-arc-clothoid
// matching target (dx, dy) in local frame. Returns false if infeasible.
static bool solveClothoidArc(double km, double dx, double dy,
                             double & L1, double & L2);

// Sample x/y and yaw/κ along the three phases.
static void samplingPointsXY(int pts_num, double ds_1, double ds_2,
                             double dk_ds, double kappa_max,
                             std::vector<double> & x_pts,
                             std::vector<double> & y_pts);
static void samplingPointsYaw(int pts_num, double ds_1, double ds_2,
                              double dk_ds, double kappa_max,
                              std::vector<double> & theta_pts,
                              std::vector<double> & kappa_pts);
```

## Usage Example

```cpp
#include "igv_path_planner/path_generator.hpp"

// Configure parameters
PathGenerator::Parameters params;
params.path_interval = 0.5;
params.default_velocity = 5.0;
params.min_turn_radius = 5.0;

PathGenerator generator(params);

// Create sparse waypoints
std::vector<autoware_planning_msgs::msg::PathPoint> sparse;

autoware_planning_msgs::msg::PathPoint start;
start.pose.position.x = 0.0;
start.pose.position.y = 0.0;
start.pose.orientation = createQuaternionFromYaw(0.0);
start.longitudinal_velocity_mps = 5.0;
sparse.push_back(start);

autoware_planning_msgs::msg::PathPoint goal;
goal.pose.position.x = 50.0;
goal.pose.position.y = 3.0;  // Lateral offset indicates lane change
goal.pose.orientation = createQuaternionFromYaw(0.0);
goal.longitudinal_velocity_mps = 0.0;
sparse.push_back(goal);

// Generate dense path
auto dense_path = generator.generatePath(sparse);
// Result: ~100 points with smooth S-curve lane change
```

## Debugging

Enable debug output by setting `params.debug = true`. This logs maneuver
detection decisions and segment information (including Q_TURN solve results:
`L1`, `L2`, `ds_total`, yaw-clothoid feasibility, point count) to stderr.

## Output Fields

Each generated `PathPoint` contains:

| Field | Description |
|-------|-------------|
| `pose.position` | x, y, z coordinates |
| `pose.orientation` | Quaternion (vehicle body heading — may differ from path tangent during Q_TURN/OBLIQUE) |
| `longitudinal_velocity_mps` | Target velocity (negative for reverse) |
| `lateral_velocity_mps` | Non-zero during Q_TURN (β·vx) and OBLIQUE_LANE_CHANGE (crab) |
| `heading_rate_rps` | Non-zero during Q_TURN (vx·κ_drive) |
| `is_final` | True for last point in path |

## Integration

Used by:
- **fms_goal_path_generator_node**: Generates geometric paths to goal poses
- **fms_path_converter_node**: May use for path densification
