# fms_kinematic_smoother_node

Owns kinematic feasibility in FMS mode. Reads the obstacle-shaped trajectory
from `fms_velocity_shaper`, enforces jerk / acceleration / lateral-accel /
steering-rate envelopes, handles engage-from-rest and tracking-deviation
replan, preserves stop points, and re-injects the FMS-encoded body yaw for
OBLIQUE / Q-TURN crab maneuvers — then hands the result to
`planning_validator`.

This node is a deliberately slim port of upstream
`autoware_velocity_smoother`: it keeps the operator-tuned kinematic math
verbatim but drops three QP-based algorithms, the multi-algorithm dispatch
plumbing, the operation-mode coupling, and the debug trajectory publishers
that aren't load-bearing in FMS mode. The result is ~3.1 K LOC of net new
code in `igv_velocity_planner`, **no `autoware_osqp_interface` /
`autoware_qp_interface` dependency**, and a single algorithm
(`AnalyticalJerkConstrainedSmoother`).

See [`docs/vehicle_agent/velocity_planner_simplification_plan.md`](../../../../docs/vehicle_agent/velocity_planner_simplification_plan.md)
for the full design rationale, including why the upstream
`autoware_velocity_smoother` package is left in place rather than slimmed
in-place.

## Overview

In FMS mode the planning chain ends in this sequence:

```
fms_path_converter
  └─► fms_velocity_shaper          # obstacle-driven v(s)
        └─► fms_kinematic_smoother  # this node — jerk / accel / curve feasibility
              └─► planning_validator
                    └─► /planning/scenario_planning/trajectory
```

Single-owner table for the four concerns:

| Concern | Owner |
|---|---|
| Reference `v(s)` profile | `fms_path_converter` (FMS-chosen) |
| Obstacle-driven `v(s)` overrides | `fms_velocity_shaper` |
| **Kinematic feasibility (jerk, accel, lateral-accel, steering-rate)** | **`fms_kinematic_smoother`** |
| Sanity gate (NaN, latacc, deviation) | `planning_validator` |

The contract on the wire is identical to upstream `autoware_velocity_smoother`:
the node subscribes to `~/input/trajectory` and publishes
`~/output/trajectory` on the same default topic names. Downstream
`planning_validator` and the external `motion_control` see no change.

## Architecture

```
                  ┌───────────────────────────────────────────────────────────┐
                  │                fms_kinematic_smoother                     │
                  │                                                           │
~/input/trajectory─┼──► onCurrentTrajectory(msg)                              │
                  │      │                                                    │
~/input/odometry ─┼──► (polling, used inside the cycle)                       │
~/input/acceleration ──► (polling)                                            │
~/input/external_velocity_limit_mps ─► (polling — RViz / API speed limits)    │
~/input/maneuver_type ──► current_maneuver_type_                              │
~/input/behavior_path ─► behavior_path_msg_  (OBLIQUE/Q_TURN body-yaw cache)  │
                  │      ▼                                                    │
                  │   ┌─────────────────────────────────────────────────┐     │
                  │   │ 1. checkData()  (odom + accel + traj ≥ 2)       │     │
                  │   │ 2. removeOverlapPoints, append v=0 terminal     │     │
                  │   │ 3. calcExternalVelocityLimit (jerk-constrained) │     │
                  │   │ 4. flipVelocity if reverse                      │     │
                  │   │ 5. calcTrajectoryVelocity:                      │     │
                  │   │     extractPathAroundIndex (±extract_dist)      │     │
                  │   │     applyExternalVelocityLimit                  │     │
                  │   │     applyStopApproachingVelocity (ramp to       │     │
                  │   │       stopping_velocity in stopping_distance)   │     │
                  │   │     smoothVelocity:                             │     │
                  │   │       calcInitialMotion  (EGO_VELOCITY /        │     │
                  │   │         LARGE_DEVIATION_REPLAN / ENGAGING /     │     │
                  │   │         NORMAL)                                 │     │
                  │   │       applyLateralAccelerationFilter            │     │
                  │   │       applySteeringRateLimit                    │     │
                  │   │       smoother_->resampleTrajectory             │     │
                  │   │       smoother_->apply  (Analytical jerk-       │     │
                  │   │         constrained, forward + backward pass)   │     │
                  │   │       overwriteStopPoint                        │     │
                  │   │       applyMaximumVelocityLimit (max_velocity)  │     │
                  │   │       insertBehindVelocity                      │     │
                  │   │ 6. resampling::resampleTrajectory (post-resample)│    │
                  │   │ 7. flipVelocity (back, if reverse)              │     │
                  │   │ 8. OBLIQUE/Q_TURN orientation re-inject from    │     │
                  │   │    behavior_path_msg_                           │     │
                  │   │ 9. publishTrajectory                            │     │
                  │   └─────────────────────────────────────────────────┘     │
                  │                                                           │
                  └─────────────────────┬─────────────────────────────────────┘
                                        │
                                        ├──► ~/output/trajectory
                                        └──► ~/debug/processing_time_ms
```

The cycle is **input-driven** — each `Trajectory` message from
`fms_velocity_shaper` triggers exactly one pass and one publish. Cadence is
inherited from the shaper (≈20 Hz, set by `fms_path_converter`'s publish
rate).

## Per-cycle pipeline

Each `onCurrentTrajectory` invocation walks the following 9 stages. The
ordering matters — see the per-stage notes.

### 1. `checkData()` gate

Bail out without publishing if any of `current_odometry_ptr_`,
`current_acceleration_ptr_`, or `base_traj_raw_ptr_` is null, or the input
has fewer than 2 points. This guard is what keeps the first cycle after
startup from emitting a junk trajectory that `planning_validator` would
reject (its `trajectory_shift` check looks at *consecutive* outputs, so the
first publish needs to be a valid one).

### 2. Input normalization

- `removeOverlapPoints` on the input — fixes the rare case where two
  consecutive points coincide due to the shaper's reverse-direction
  flip or a downstream resample upstream.
- Append `v=0` at the terminal so the smoother's stop-point math has a
  well-defined target even when the FMS path has a non-zero final velocity.

### 3. External velocity limit (`calcExternalVelocityLimit`)

If a `VelocityLimit` was received on `~/input/external_velocity_limit_mps`,
this stage computes the **distance** at which we need to start braking so
that we hit the limit smoothly under the current jerk/accel envelope.
Implementation: `calcStopDistWithJerkConstraints` with `(v0, a0)` from the
*previous output* (not from raw ego) — this avoids re-planning the same
brake every cycle.

When the engage gate is open (`isEngageStatus`), we apply the limit
immediately at zero distance instead. Otherwise we cap `max_velocity_with_deceleration_`
to the (continuous) intersection of "what we can reach by braking from the
previous v0/a0" and the published limit.

### 4. `updateDataForExternalVelocityLimit`

Subtract the distance traveled since the last cycle from the
`external_velocity_limit_.dist` so the insertion point doesn't slide
backward as the vehicle moves forward.

### 5. Reverse handling

`isReverse(input)` returns true if any input point has negative
`longitudinal_velocity_mps`. When true, `flipVelocity(input)` multiplies
all velocities by `-1` so the algorithm sees a forward-driving trajectory.
A matching `flipVelocity(output)` runs at stage 7 to restore sign.

### 6. Core shaping (`calcTrajectoryVelocity` → `smoothVelocity`)

The trajectory is extracted around the nearest-to-ego index (with
`extract_ahead_dist` ahead and `extract_behind_dist` behind), the external
velocity limit is overlaid, and the stop-approaching ramp is applied. Then
`smoothVelocity` runs the algorithm with the initial-motion state machine —
see [Initial-motion state machine](#initial-motion-state-machine).

### 7. Post-resample + reverse restore

The output is resampled by `kinematic_smoother::resampling::resampleTrajectory`
using `post_resample_param` (dense+sparse, `false` for ZOH velocity). If
the trajectory was reversed in stage 5, `flipVelocity` runs again.

### 8. OBLIQUE / Q_TURN body-yaw re-injection

When `current_maneuver_type_` is `2` (OBLIQUE) or `6` (Q_TURN), the
internal resample has overwritten each `pose.orientation` with the path
tangent — but the FMS-encoded `Path` is the only place the *intended
vehicle-facing body yaw* still exists. For each output point, find the
nearest XY neighbor in the cached `behavior_path_msg_` and copy its
orientation. `lateral_velocity_mps` is zeroed (the 4WS controller derives
the crab angle geometrically from `pose.orientation` vs. path tangent).

The match uses a moving cursor with symmetric backward step — not an
arc-length normalization — because the cached behavior_path and the
resampled output trace the same geometry but may span different arcs.
Scaling by total length maps points onto the wrong physical positions
whenever spans differ.

### 9. Publish

`publishTrajectory` converts to `autoware_planning_msgs::msg::Trajectory`,
preserves `base_traj_raw_ptr_->header`, and publishes on `~/output/trajectory`
(default: `/planning/scenario_planning/velocity_smoother/trajectory`).
`~/debug/processing_time_ms` carries the wall-clock duration of the cycle.

## Initial-motion state machine

`calcInitialMotion` picks the `(v0, a0)` pair the smoother starts from. It
returns one of four `InitializeType` states; each one biases the smoother
and the `insertBehindVelocity` post-pass differently.

| State | Trigger | `v0` | `a0` | Behind-velocity behavior |
|-------|---------|------|------|--------------------------|
| `EGO_VELOCITY` | No `prev_output_` yet (first cycle after startup) | `|ego.vx|` | `0.0` | Output copies closest point's v/a backward |
| `LARGE_DEVIATION_REPLAN` | \|ego.vx − prev_output_v\| > `replan_vel_deviation` (5.53 m/s) | `|ego.vx|` | `prev a` | Output copies closest point's v/a backward |
| `ENGAGING` | `|ego.vx| < engage_velocity·engage_exit_ratio` AND target_vel ≥ engage_velocity AND stop_dist > stop_dist_to_prohibit_engage | `engage_velocity` (0.25) | `engage_acceleration` (0.1) | Output copies closest point's v/a backward |
| `NORMAL` | Otherwise | prev_output v | prev_output a | Output interpolates from previous trajectory (preserves continuity) |

**Engage suppression near a stop.** When the engage gate fires but
`stop_dist ≤ stop_dist_to_prohibit_engage` (0.5 m), the engage step is
skipped and the vehicle stays at v=0. This is what stops the vehicle from
creeping into a stop line if it briefly idles at speed 0 right next to one.

**Manual-mode handling.** The upstream node also had a fifth pathway —
when `OperationModeState.mode != AUTONOMOUS`, the smoother would plan from
ego velocity for smooth re-engagement. FMS vehicles are always autonomous,
so this branch (and the `OperationModeState` subscription) was dropped.

## Parameters

All parameters live in
[`config/fms_kinematic_smoother.param.yaml`](../config/fms_kinematic_smoother.param.yaml).
Values are byte-for-byte copies of the operator-tuned upstream defaults.
**Do not "clean up" defaults without operator sign-off** — engage params
in particular have been tuned against the IGV hardware.

### Kinematic envelope (jerk / accel / decel)

| Parameter | Type | Default | Units | Description |
|-----------|------|---------|-------|-------------|
| `max_vel` | double | `11.1` | m/s | Hard cap on the input velocity (40 km/h) |
| `normal.max_acc` | double | `1.0` | m/s² | Max forward acceleration in planning |
| `normal.min_acc` | double | `-0.5` | m/s² | Max forward deceleration (mild brake) in planning |
| `normal.max_jerk` | double | `1.0` | m/s³ | Max forward jerk in planning |
| `normal.min_jerk` | double | `-0.5` | m/s³ | Max forward decel-jerk in planning |
| `limit.min_acc` | double | `-2.5` | m/s² | Hard limit used by the external velocity limit pathway |
| `limit.max_acc` | double | `1.0` | m/s² | Hard accel limit |
| `limit.min_jerk` | double | `-1.5` | m/s³ | Hard jerk limit |
| `limit.max_jerk` | double | `1.5` | m/s³ | Hard jerk limit |

### Engage / replan / stop

| Parameter | Type | Default | Units | Description |
|-----------|------|---------|-------|-------------|
| `max_velocity` | double | `20.0` | m/s | Final-cap floor applied to every output point |
| `stop_decel` | double | `0.0` | m/s² | Deceleration at a stop point (passed through to base param) |
| `replan_vel_deviation` | double | `5.53` | m/s | Tracking-error threshold to re-seed `LARGE_DEVIATION_REPLAN` |
| `engage_velocity` | double | `0.25` | m/s | Velocity injected when engaging from rest |
| `engage_acceleration` | double | `0.1` | m/s² | Acceleration injected when engaging from rest |
| `engage_exit_ratio` | double | `0.5` | — | Exit engage when `ego_v ≥ engage_velocity · engage_exit_ratio` |
| `stop_dist_to_prohibit_engage` | double | `0.5` | m | Engage suppressed if a stop is closer than this |
| `stopping_velocity` | double | `2.778` | m/s | Pre-stop velocity ramp target |
| `stopping_distance` | double | `0.0` | m | Distance over which the pre-stop ramp applies (0 disables) |
| `margin_to_insert_external_velocity_limit` | double | `0.3` | m | Inserted before the brake point for the API velocity limit |

### Trajectory extraction and resampling

| Parameter | Type | Default | Units | Description |
|-----------|------|---------|-------|-------------|
| `extract_ahead_dist` | double | `200.0` | m | Forward window around ego |
| `extract_behind_dist` | double | `5.0` | m | Backward window around ego |
| `delta_yaw_threshold` | double | `1.0472` | rad (~60°) | Pose-yaw delta tolerance |
| `ego_nearest_dist_threshold` | double | `3.0` | m | XY tolerance for nearest-index search |
| `ego_nearest_yaw_threshold` | double | `1.046` | rad | Yaw tolerance for nearest-index search |
| `max_trajectory_length` | double | `200.0` | m | Resample max length (used by Analytical) |
| `min_trajectory_length` | double | `150.0` | m | Resample min length |
| `resample_time` | double | `2.0` | s | Dense-sample horizon |
| `dense_resample_dt` | double | `0.2` | s | Dense-sample interval |
| `dense_min_interval_distance` | double | `0.1` | m | Dense-sample minimum spacing |
| `sparse_resample_dt` | double | `0.5` | s | Sparse-sample interval |
| `sparse_min_interval_distance` | double | `4.0` | m | Sparse-sample minimum spacing |
| `post_max_trajectory_length` | double | `300.0` | m | Post-resample (output) max length |
| `post_min_trajectory_length` | double | `30.0` | m | Post-resample (output) min length |
| `post_resample_time` | double | `10.0` | s | Post-resample horizon |
| `post_dense_resample_dt` | double | `0.1` | s | Post dense interval |
| `post_dense_min_interval_distance` | double | `0.1` | m | Post dense min spacing |
| `post_sparse_resample_dt` | double | `0.1` | s | Post sparse interval |
| `post_sparse_min_interval_distance` | double | `1.0` | m | Post sparse min spacing |

### Lateral-acceleration + steering-rate filter

| Parameter | Type | Default | Units | Description |
|-----------|------|---------|-------|-------------|
| `enable_lateral_acc_limit` | bool | `true` | — | Master toggle for the lat-accel filter |
| `max_lateral_accel` | double | `0.8` | m/s² | Cap on lateral acceleration through curves |
| `min_curve_velocity` | double | `2.74` | m/s | Floor on velocity through tight curves |
| `decel_distance_before_curve` | double | `3.5` | m | Pre-curve decel ramp length |
| `decel_distance_after_curve` | double | `2.0` | m | Post-curve recovery length |
| `min_decel_for_lateral_acc_lim_filter` | double | `-2.5` | m/s² | Decel limit when the lat-accel filter ramps down |
| `enable_steering_rate_limit` | bool | `true` | — | Master toggle for the steering-rate filter |
| `max_steering_angle_rate` | double | `40.0` | deg/s | Max steering angle rate |
| `resample_ds` | double | `0.1` | m | Trajectory point spacing for the filter |
| `curvature_threshold` | double | `0.02` | 1/m | Curvature above which the steering-rate filter fires |
| `curvature_calculation_distance` | double | `5.0` | m | Three-point curvature lookup distance |

### Analytical algorithm parameters

The only algorithm built into this node. Forward pass shapes acceleration
toward target velocity; backward pass enforces decel-to-stop with a jerk
sweep from `start_jerk` down to `min_jerk`.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `resample.ds_resample` | double | `0.1` | Analytical resample spacing |
| `resample.num_resample` | int | `1` | Resample multiplier per segment |
| `resample.delta_yaw_threshold` | double | `0.785` | Pose-yaw tolerance |
| `latacc.enable_constant_velocity_while_turning` | bool | `false` | Hold filtered velocity through a constant-yaw segment |
| `latacc.constant_velocity_dist_threshold` | double | `2.0` | Grouping distance for the hold above |
| `forward.max_acc` | double | `1.0` | Forward-pass max acceleration |
| `forward.min_acc` | double | `-1.0` | Forward-pass max deceleration |
| `forward.max_jerk` | double | `0.3` | Forward-pass max jerk |
| `forward.min_jerk` | double | `-0.3` | Forward-pass min jerk |
| `forward.kp` | double | `0.3` | Forward-pass feedback gain for velocity error |
| `backward.start_jerk` | double | `-0.1` | Initial jerk to try in the backward pass |
| `backward.min_jerk_mild_stop` | double | `-0.3` | Jerk threshold below which `min_acc_mild_stop` is used |
| `backward.min_jerk` | double | `-1.5` | Backward-pass minimum jerk |
| `backward.min_acc_mild_stop` | double | `-1.0` | Decel for mild stops |
| `backward.min_acc` | double | `-2.5` | Decel for hard stops |
| `backward.span_jerk` | double | `-0.01` | Sweep step from `start_jerk` toward `min_jerk` |

## Topics

### Subscriptions

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `~/input/trajectory` | `autoware_planning_msgs/Trajectory` | Reliable, depth 1 | The obstacle-shaped trajectory from `fms_velocity_shaper`; triggers each cycle |
| `/localization/kinematic_state` | `nav_msgs/Odometry` | InterProcessPollingSubscriber | Ego pose + velocity (used to find nearest index and compute initial motion) |
| `~/input/acceleration` | `geometry_msgs/AccelWithCovarianceStamped` | InterProcessPollingSubscriber | Filtered ego acceleration (from `planning_ego_acceleration_estimator`) |
| `~/input/external_velocity_limit_mps` | `tier4_planning_msgs/VelocityLimit` | InterProcessPollingSubscriber (Newest) | API speed limit from RViz panel / awapi adapter |
| `~/input/maneuver_type` | `std_msgs/UInt8` | Reliable, depth 10, TransientLocal | FMS maneuver type (0=STRAIGHT, 2=OBLIQUE, 6=Q_TURN) |
| `~/input/behavior_path` | `autoware_planning_msgs/Path` | Reliable, depth 1, **Volatile** | FMS-encoded path used for OBLIQUE/Q_TURN body-yaw re-injection |

> **Important QoS gotcha.** The behavior_path subscription must declare
> `RELIABLE` + `VOLATILE`. Declaring `TRANSIENT_LOCAL` on the subscriber
> would be DDS-incompatible with the publisher's wire QoS and silently
> reject all messages.

### Publications

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `~/output/trajectory` | `autoware_planning_msgs/Trajectory` | Reliable, depth 1 | Smoothed trajectory consumed by `planning_validator` |
| `~/debug/processing_time_ms` | `tier4_debug_msgs/Float64Stamped` | Reliable, depth 1 | Per-cycle wall-clock duration |

### Default launch remappings (`tier4_planning_component_fms.launch.xml`)

| Internal | External |
|----------|----------|
| `~/input/trajectory` | `/planning/scenario_planning/scenario_selector/trajectory` |
| `~/input/acceleration` | `/planning/acceleration` |
| `~/input/external_velocity_limit_mps` | `/planning/scenario_planning/max_velocity` |
| `~/input/maneuver_type` | `/fms/maneuver_type` |
| `~/input/behavior_path` | `/planning/scenario_planning/lane_driving/behavior_planning/path` |
| `~/output/trajectory` | `/planning/scenario_planning/velocity_smoother/trajectory` |

The output topic name is **identical** to upstream `autoware_velocity_smoother`,
so `planning_validator` sees no change.

## Algorithm details

### Static / dynamic velocity envelope (lateral-accel filter)

For each input point, sweep ±(`decel_distance_before_curve` /
`decel_distance_after_curve`) and find the maximum curvature in that
window. Cap velocity at `√(max_lateral_accel / κ)` (with the
`min_curve_velocity` floor). When `enable_smooth_limit` is true the
filter also enforces a velocity profile with constant jerk
(`min_jerk`) bounded by `min_decel_for_lateral_acc_lim_filter` to avoid
the controller seeing a step.

### Steering-rate filter

Compute steering angle from `atan(wheel_base · curvature)` for every
consecutive pair of points, then the steering rate from finite
differences (with a 3-tap mean filter to suppress noise). Where
`fabs(curvature) ≥ curvature_threshold` AND the computed steering rate
exceeds `max_steering_angle_rate`, scale velocity by
`max_steering_angle_rate / steering_rate` (clamped against
`min_curve_velocity`).

### Analytical jerk-constrained smoother

Single deterministic pass per cycle (no QP). Two parts:

1. **Forward pass** (`applyForwardJerkFilter`). Walk the trajectory from
   the start index; for each point compute the velocity error against
   the *reference* trajectory's longitudinal velocity, feed it through a
   P-controller with gain `forward.kp`, clamp the resulting acceleration
   to `[forward.min_acc, forward.max_acc]`, then clamp jerk to
   `[forward.min_jerk, forward.max_jerk]`. Integrate forward by `dt = ds/v`.

2. **Backward pass** (`applyBackwardDecelFilter`). For each deceleration
   target index (found by `searchDecelTargetIndices` — local minima of
   the velocity profile), try planning jerks from `backward.start_jerk`
   stepping by `backward.span_jerk` down to `backward.min_jerk`. The
   first jerk that produces a brake distance ≤ the available distance
   wins. Calls into `analytical_velocity_planning_utils::calcStopDistWithJerkAndAccConstraints`,
   which solves one of three closed-form cases (TRAPEZOID / TRIANGLE /
   LINEAR) based on whether the target velocity requires sustaining
   `min_acc` or just a triangular jerk pulse.

3. **Mild-stop branch**. When the planning jerk is above
   `backward.min_jerk_mild_stop`, the algorithm uses `backward.min_acc_mild_stop`
   instead of `backward.min_acc` — gentler decel for small velocity
   drops.

### External velocity limit insertion

When the API publishes a velocity limit, this node:

1. Applies the cap to every point in the extracted trajectory
   (`max_velocity_with_deceleration_` floor) so the controller sees a
   bounded profile even before the brake point.
2. Inserts a new trajectory point at `external_velocity_limit_.dist`
   meters ahead of the closest-to-ego segment, computed from
   `calcStopDistWithJerkConstraints(v0, a0, …, max_velocity)` + the
   `margin_to_insert_external_velocity_limit`. This is what produces
   the smooth slow-down ramp visible in RViz when the slider drops.

### Stop-point preservation

`overwriteStopPoint` searches the (pre-smoothing) input for the first
zero-velocity index, then finds the nearest output point to it (using
both XY and yaw thresholds), and forces all subsequent output points to
zero. This is the safety guarantee that the shaper's dynamic-obstacle
stop never gets "smoothed away" by the kinematic optimization.

### `insertBehindVelocity` (output continuity)

For each output point *behind* the closest-to-ego index, choose between:

- **Hold-closest** (used by `EGO_VELOCITY`, `LARGE_DEVIATION_REPLAN`,
  `ENGAGING`): copy the closest-point's v / a backward. This is the
  right behavior when we just re-seeded the trajectory.
- **Interpolate-from-prev** (used by `NORMAL`): for each behind-point,
  find the nearest segment in `prev_output_` and interpolate its v / a.
  Keeps the controller's nearest-lookup from seeing a velocity
  discontinuity at the segment boundary.

### OBLIQUE / Q_TURN body-yaw cache

Lives entirely in the input section (subscription + maneuver-type
clearing) plus the orientation-injection block at stage 8. The cache:

- Clears on every `current_maneuver_type_` change (so a stale-maneuver
  Path can never bleed into the next maneuver's output).
- Is consulted only when `current_maneuver_type_ ∈ {OBLIQUE, Q_TURN}`.
- Uses **XY nearest neighbor with a moving cursor**, not arc-length
  normalization. The cached path may extend past the resample window,
  and scaling by total length would map output points onto the wrong
  physical positions. The moving cursor (forward + symmetric backward
  step) finds each neighbor in amortized O(1).
- Zeros `lateral_velocity_mps` so the 4WS controller derives the crab
  angle geometrically from `pose.orientation` vs. path tangent rather
  than from a separate `vy` signal.

## Diagnostics

| Topic | When | Purpose |
|---|---|---|
| `~/debug/processing_time_ms` | Every cycle | Wall-clock duration of the full pipeline (for runtime budgeting) |

Notably **NOT** published by this node (intentionally dropped during the
slim port):

- `~/debug/trajectory_raw`, `~/debug/trajectory_external_velocity_limited`,
  `~/debug/trajectory_lateral_acc_filtered`,
  `~/debug/trajectory_steering_rate_limited`,
  `~/debug/trajectory_time_resampled` — stage-by-stage trajectory snapshots
  available in upstream `autoware_velocity_smoother`. If you need these for
  debugging, run the upstream node in parallel during the diagnosis window
  (instructions in the [Out-of-scope behaviors](#out-of-scope-behaviors)
  section below).
- `~/closest_velocity`, `~/closest_acceleration`, `~/closest_jerk`,
  `~/closest_max_velocity` — point-at-ego scalar telemetry.
- `~/distance_to_stopline` — stop-line distance.
- `~/virtual_wall` — RViz marker for the API-limit insertion point.
- `~/output/current_velocity_limit_mps` — echo of the active limit.
- `~/stop_speed_exceeded` — boolean warning when the optimization
  produces non-zero velocity at the stop index.

## Testing

The node ships with the [`test/fms_kinematic_smoother_test.cpp`](../test/fms_kinematic_smoother_test.cpp)
gtest suite. Tests live at the trajectory_utils level — the higher-level
smoothing pipeline (`calcInitialMotion`, `smoothVelocity`, …) is private
to the node and needs full ROS state (odometry, acceleration,
`prev_output_`) to exercise meaningfully. End-to-end behavior is
covered by the bag-replay regression plan in
`docs/vehicle_agent/velocity_planner_simplification_plan.md`.

| Test | What it asserts |
|------|-----------------|
| `ArclengthArrayMonotonic` | `calcArclengthArray` produces a non-decreasing sequence with the right total length |
| `ArclengthEmptyInput` | Empty trajectory → empty arclength array |
| `ExtractAroundIndexRespectsBounds` | `extractPathAroundIndex` overshoots by at most one segment in each direction (matches upstream loop semantics) |
| `ExtractInvalidIndexReturnsEmpty` | Out-of-range index returns empty |
| `MaxVelocityLimitClipsRangeOnly` | `applyMaximumVelocityLimit(from, to)` only touches `[from, to)` |
| `MaxVelocityLimitNeverIncreases` | Cap above current → no-op (the function only lowers velocity, never raises) |
| `StopDistanceFindsZeroVelocityIndex` | `calcStopDistance` finds the first zero-velocity index and returns its 2D distance |
| `StopDistanceNoStopReturnsLarge` | No zero-velocity index → returns `numeric_limits<double>::max()` |
| `CalcStopDistWithJerkConstraintsBrakeToZero` | The jerk-constrained brake-from-5m/s predicate succeeds and returns a positive distance + non-empty jerk profile |
| `CalcStopDistSmallVelocityIsSmall` | Braking from 0.1 m/s yields stop distance < 1 m (avoids the v0=target_vel singularity at the LINEAR branch) |
| `ConstantJerkProfileMonotonicWhenJerkPositive` | `calcVelocityProfileWithConstantJerkAndAccelerationLimit` produces a monotonically non-decreasing profile under positive jerk |

Run the suite:

```bash
colcon build --symlink-install --packages-select igv_velocity_planner
./build/igv_velocity_planner/fms_kinematic_smoother_test
```

Or via colcon:

```bash
colcon test --packages-select igv_velocity_planner \
  --ctest-args -R fms_kinematic_smoother_test
colcon test-result --test-result-base build/igv_velocity_planner --verbose
```

### What the bag-replay regression covers

End-to-end behavior (full pipeline, OBLIQUE/Q_TURN cache, engage from
rest) is validated by replaying recorded FMS rosbags through this node
alongside the upstream `autoware_velocity_smoother`, then diffing the two
output trajectories point-by-point. See plan §5.2 for the protocol.
Target tolerances:

- Per-cycle longitudinal-velocity Δ: < 0.05 m/s sustained, < 0.2 m/s
  transient at maneuver transitions.
- Per-cycle acceleration Δ: < 0.1 m/s².
- Stop-point arc-length Δ: 0 (identical input → identical stop).
- OBLIQUE/Q_TURN per-point `pose.orientation` Δ: < 1e-3 rad.

## Usage examples

### Default — runs automatically in FMS mode

When `fms_enabled=true` (default in `complete_autoware_system_fms.launch.xml`),
`tier4_planning_component_fms.launch.xml` automatically launches this node
alongside `fms_velocity_shaper` and suppresses upstream
`autoware_velocity_smoother` via `launch_velocity_smoother=false`. No
extra action needed:

```bash
# Terminal 1
./launch_fms_test_components.sh

# Terminal 2
./launch_with_fms_real_vehicle.sh
```

### Standalone — for testing or bag replay

```bash
ros2 run igv_velocity_planner igv_kinematic_smoother --ros-args \
  --params-file install/igv_velocity_planner/share/igv_velocity_planner/config/igv_kinematic_smoother.param.yaml \
  --params-file install/scale_igv_vehicle_description/share/scale_igv_vehicle_description/config/vehicle_info.param.yaml \
  -r ~/input/trajectory:=/planning/scenario_planning/scenario_selector/trajectory \
  -r ~/input/acceleration:=/planning/acceleration \
  -r ~/input/external_velocity_limit_mps:=/planning/scenario_planning/max_velocity \
  -r ~/input/maneuver_type:=/fms/maneuver_type \
  -r ~/input/behavior_path:=/planning/scenario_planning/lane_driving/behavior_planning/path \
  -r ~/output/trajectory:=/test/velocity_smoother/trajectory
```

For bag-replay diff, point `~/output/trajectory` to a separate test
topic and compare against the recorded
`/planning/scenario_planning/velocity_smoother/trajectory`.

### Inspect live output

```bash
# Smoothed trajectory
ros2 topic echo /planning/scenario_planning/velocity_smoother/trajectory \
  --qos-reliability reliable

# Per-cycle runtime
ros2 topic echo /fms_kinematic_smoother/debug/processing_time_ms
```

## Out-of-scope behaviors

Dropped during the slim port. None of these are reachable via parameter —
they were physically removed.

| Feature | Source | Why dropped |
|---|---|---|
| JerkFiltered algorithm | `autoware_velocity_smoother::JerkFilteredSmoother` | OSQP-based; Analytical is what FMS launches pin |
| L2 algorithm | `autoware_velocity_smoother::L2PseudoJerkSmoother` | OSQP-based; never selected in FMS mode |
| Linf algorithm | `autoware_velocity_smoother::LinfPseudoJerkSmoother` | OSQP-based; never selected in FMS mode |
| `setupSmoother()` dispatch | `node.cpp` ctor | Single algorithm now — direct construction |
| `onParameter` dynamic reconfigure | `node.cpp` | Restart-to-retune is fine for the first cut; can be added back if operators ask |
| `OperationModeState` subscription | `node.cpp` | FMS vehicles are always autonomous; no manual fallback |
| `plan_from_ego_speed_on_manual_mode` branch | `calcInitialMotion` | Same reason |
| Debug trajectory publishers (5 stages) | `node.cpp` | Adds bandwidth; per-stage snapshots available via upstream side-by-side run |
| JerkFiltered-only debug publishers (3) | `node.cpp` | Algorithm not built |
| `~/output/current_velocity_limit_mps` echo | `node.cpp` | RViz speed-limit slider already shows the value it publishes |
| `~/virtual_wall` marker | `node.cpp` | Per-cycle marker overhead; not load-bearing |
| `~/stop_speed_exceeded` warning | `node.cpp` | Diagnostic only; the validator catches the real condition |
| `LoggerLevelConfigure` | `node.cpp` | Standard `rclcpp_components` logging is enough |
| `PublishedTimePublisher` | `node.cpp` | Convenience plumbing, not behavior |

### Running upstream alongside this node (for diagnosis)

For stage-by-stage trajectory inspection during a bug hunt, you can
re-enable upstream `autoware_velocity_smoother` alongside this node by
passing `launch_velocity_smoother:=true` to
`tier4_planning_component_fms.launch.xml`. They'll both run, both publish
to the same topic (which is bad in production but useful for diagnosis if
you remap one of them). Don't ship with both running.

## Limitations and known gaps

- **No parameter hot-reload.** Tuning requires a node restart. The
  upstream `onParameter` callback isn't ported. Add back if operators
  ask.
- **No multi-trajectory diagnostics.** The upstream node publishes the
  trajectory at each stage of the pipeline (raw, vel-limited, lat-acc
  filtered, steering-rate limited, resampled). This node drops all of
  them to keep the bandwidth low and the API surface small. Use
  side-by-side replay against the upstream package for debugging.
- **Single algorithm.** JerkFiltered / L2 / Linf can't be enabled at
  runtime — the algorithm constructor is hardcoded to
  `AnalyticalJerkConstrainedSmoother`. The upstream package still ships
  the alternatives if you need them in a different launch context.
- **Manual-mode handling absent.** If FMS ever needs to drive a vehicle
  that briefly drops out of AUTONOMOUS (e.g. operator override during
  testing), this node will plan from the previous output's v / a rather
  than from ego — which can produce a deceleration spike at the next
  re-engage. Acceptable for the current FMS deployment; not for a
  mixed-mode vehicle.
- **`calcStopDistWithJerkConstraints` has a v0 == target_vel == 0
  singularity.** The LINEAR branch divides by `(target_vel - v0)`. Not
  reachable in normal operation because the external velocity limit
  pathway compares with `eps = 1e-4` before calling, but worth knowing
  when writing new tests.
- **OBLIQUE/Q_TURN cache discards stale Paths on maneuver change.** If
  the maneuver type flips while a long Path is mid-transit, the next
  output cycle reverts to the path-tangent orientation until a fresh
  Path arrives. Operators using fast maneuver transitions should set
  `fms_path_converter`'s `publish_rate` accordingly.

## Related nodes

- **[`fms_velocity_shaper_node`](fms_velocity_shaper_node.md)** —
  publishes this node's input. Owns the obstacle-driven `v(s)` overrides
  (static slowdown, dynamic stop with hysteresis, ACC).
- **`fms_path_converter_node`** — produces the Path that becomes the
  shaper's input and (separately) the cached behavior_path for the
  OBLIQUE/Q_TURN orientation re-injection here.
- **`planning_ego_acceleration_estimator`** — differentiates
  `/localization/kinematic_state.twist.linear.x` to produce the
  `/planning/acceleration` topic this node reads. Replaces the noisy
  `/localization/acceleration` everywhere in the FMS planning chain.
- **`autoware_planning_validator`** — consumes this node's output and
  enforces the last-line sanity checks (NaN, jerk, lateral acceleration,
  trajectory shift). Sees no contract change relative to upstream
  `autoware_velocity_smoother`.
- **`autoware_velocity_smoother`** — the upstream package this node is
  ported from. Still in the workspace because `motion_velocity_planner`
  and the non-FMS `tier4_planning_component_v1` launch use it.

## References

- Design plan: [`docs/vehicle_agent/velocity_planner_simplification_plan.md`](../../../../docs/vehicle_agent/velocity_planner_simplification_plan.md)
- Node source: [`src/fms_kinematic_smoother_node.cpp`](../src/fms_kinematic_smoother_node.cpp)
- Node header: [`include/igv_velocity_planner/fms_kinematic_smoother_node.hpp`](../include/igv_velocity_planner/fms_kinematic_smoother_node.hpp)
- Algorithm libraries: [`include/igv_velocity_planner/kinematic_smoother/`](../include/igv_velocity_planner/kinematic_smoother/) +
  [`src/kinematic_smoother/`](../src/kinematic_smoother/)
- Config: [`config/fms_kinematic_smoother.param.yaml`](../config/fms_kinematic_smoother.param.yaml)
- Tests: [`test/fms_kinematic_smoother_test.cpp`](../test/fms_kinematic_smoother_test.cpp)
- Launch wiring: [`src/launcher/autoware_launch/autoware_launch/launch/components/tier4_planning_component_fms.launch.xml`](../../../../src/launcher/autoware_launch/autoware_launch/launch/components/tier4_planning_component_fms.launch.xml)
- Upstream this is ported from: [`src/universe/autoware.universe/planning/autoware_velocity_smoother/`](../../../../src/universe/autoware.universe/planning/autoware_velocity_smoother/)
