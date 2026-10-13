# planning_ego_acceleration_estimator

Republishes a planning-only acceleration topic computed from the velocity
(`twist.twist.linear.x`) of `/localization/kinematic_state` via a sliding
Savitzky–Golay derivative.

## Why

`/localization/acceleration` exhibits a stationary bias (~0.2 m/s² on the
real scale-IGV), low-frequency drift, and high-frequency vibration that
make it unsuitable as input to the planning velocity smoother and obstacle
planners. This node sits in front of the planning chain, fits a low-order
polynomial to the most recent velocity samples each tick, and emits the
analytical first derivative as `geometry_msgs/AccelWithCovarianceStamped`
on `/planning/acceleration`. Control, perception, and evaluator subscribers
continue to read the original `/localization/acceleration` unchanged — the
override is scoped to planning only.

## Algorithm

For each incoming `Odometry` sample at time `t`:

1. If `Δt < min_dt` → drop (duplicate / out-of-order).
2. If `Δt > max_dt` → clear the buffer and the LPF (gap reset).
3. Append `(t, v)` to a ring buffer; trim samples older than
   `window_seconds`.
4. If the buffer holds at least `min_window_samples` samples, fit
   `v(τ) ≈ Σ_{k=0..K} c_k · τ^k` (with `τ = t' − t`, so the most recent
   sample sits at `τ = 0`) by ordinary least squares. Read `a = c_1` from
   the fit — that is `dv/dt` evaluated at the current sample.
5. Optionally pass `a` through a first-order LPF (disabled by default; set
   `alpha < 1.0` to enable).
6. Publish a `geometry_msgs/AccelWithCovarianceStamped` with `linear.x = a`,
   other components zero, and a diagonal covariance with parameterized
   variances. `header.stamp` is taken from the input message and
   `header.frame_id` from `child_frame_id` (typically `base_link`).

The polynomial fit uses the samples' actual timestamps so jitter, dropped
duplicates, and irregular sampling are handled naturally — there is no
assumption of uniform sampling.

## Topics

| Direction  | Topic (default)                  | Type                                          |
|------------|----------------------------------|-----------------------------------------------|
| Subscribe  | `/localization/kinematic_state`  | `nav_msgs/msg/Odometry`                       |
| Publish    | `/planning/acceleration`         | `geometry_msgs/msg/AccelWithCovarianceStamped`|

Both are configurable via the `input_topic` / `output_topic` parameters.

## Parameters

Defaults in `config/ego_acceleration_estimator.param.yaml`:

| Name                | Default                          | Description                                                  |
|---------------------|----------------------------------|--------------------------------------------------------------|
| `window_seconds`    | `0.4`                            | Length of the rolling SG fit window (seconds).               |
| `min_window_samples`| `5`                              | Minimum samples required before publishing.                  |
| `polyorder`         | `3`                              | Polynomial degree of the local fit (2 or 3 recommended).     |
| `min_dt`            | `0.005` s                        | Drop samples below this Δt.                                  |
| `max_dt`            | `0.5` s                          | Reset filter on Δt above this.                               |
| `alpha`             | `1.0`                            | Optional LPF gain on the SG output. `1.0` disables the LPF.  |
| `linear_variance`   | `1.0`                            | Diagonal covariance for linear axes in the output message.   |
| `angular_variance`  | `0.05`                           | Diagonal covariance for angular axes.                        |
| `input_topic`       | `/localization/kinematic_state`  | Input odometry topic.                                        |
| `output_topic`      | `/planning/acceleration`         | Output acceleration topic.                                   |

## Build

```bash
colcon build --symlink-install --packages-select planning_ego_acceleration_estimator
source install/setup.bash
```

## Run standalone

```bash
ros2 launch planning_ego_acceleration_estimator ego_acceleration_estimator.launch.xml
```

Override defaults from the launch CLI:

```bash
ros2 launch planning_ego_acceleration_estimator ego_acceleration_estimator.launch.xml \
  input_topic:=/localization/kinematic_state \
  output_topic:=/planning/acceleration
```

## Integration with the FMS planning chain

In FMS launches the relay is wired automatically by
`tier4_planning_component_fms.launch.xml`. The planning chain reads its
acceleration input from a single launch arg `planning_acceleration_topic`
that flows through `tier4_planning_launch` (`planning.launch.xml` →
`scenario_planning.launch.xml` → `lane_driving.launch.xml` →
`motion_planning.launch.xml` / `behavior_planning.launch.xml`).

- In FMS mode the arg defaults to `/planning/acceleration` (the relay's output).
- Outside FMS mode the arg defaults to `/localization/acceleration` (legacy).

To bypass the relay and consume the raw localization acceleration directly:

```bash
ros2 launch autoware_launch complete_autoware_system_fms.launch.xml \
  planning_acceleration_topic:=/localization/acceleration ...
```

## Verification

```bash
# Relay is up
ros2 node info /ego_acceleration_estimator
ros2 topic hz /planning/acceleration             # matches input rate after warmup

# Planning subscribers point at the relay
ros2 topic info /planning/acceleration --verbose
# Expected subscribers: velocity_smoother, obstacle_cruise_planner, obstacle_stop_planner,
# behavior_path_planner, behavior_velocity_planner, motion_velocity_planner.

# Localization-side consumers (control, evaluators, ...) are unaffected
ros2 topic info /localization/acceleration --verbose
```

Sanity checks while driving / replaying a bag:

- Steady velocity → `linear.x` settles near 0.
- Brake / accelerate event → `linear.x` follows `dv/dt` of the velocity.
- Reverse / oblique / crab → still uses only `twist.linear.x` (longitudinal
  body-frame), so the sign of `a` matches the sign convention of the input
  velocity.
- Brief input freeze (Δt > `max_dt`) → buffer is cleared, output resumes
  after `min_window_samples` new samples instead of spiking on the post-gap
  delta.

## Out of scope

- No replacement of `/localization/acceleration` itself — non-planning
  consumers are unaffected.
- No lateral or angular acceleration estimation — only `linear.x` is
  written, because that is what the planning chain consumes.
- No yaw-rate / angular acceleration differentiation; `linear.y/z` and
  `angular.*` are zeroed.
