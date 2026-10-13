# motion_control

ROS 2 node that turns a planned trajectory into 4-wheel-steer commands for the
ZPMC IGV. It sits between the planning stack and the vehicle bus, consuming
`/planning/scenario_planning/trajectory` and ego odometry, and publishing
high-level control commands (`autoware_control_msgs/Control` for
`vehicle_cmd_gate`) and low-level pacmod commands (`/pacmod/motor_vel_cmd`,
`/pacmod/steering_cmd`, `/pacmod/shift_cmd`).

The single executable is `control_exe` (entry point `motion_controller_main.cpp`).
A second executable, `map_traj_server_exe`, is an offline trajectory generator
used for bench testing and is gated behind `use_traj_server`.

## Why a custom controller

The IGV is a four-wheel-steer (4WS) platform — front and rear axles steer
independently. The Autoware Universe stack does not ship a 4WS-aware
trajectory follower, so this package replaces the upstream
`trajectory_follower_node`. The controller is purpose-built for the FMS
real-vehicle pipeline (`fms_enabled=true` in `complete_autoware_system_fms.launch.xml`):
it handles **reverse driving** and **constant-heading crab maneuvers** (OBLIQUE
lane change, Q_TURN) explicitly, recovering crab angle from the gap between
per-point body yaw and the geometric path tangent.

## Documents in this directory

- [architecture.md](architecture.md) — data flow, components, where this node
  fits in the planning/control stack, what every input topic is used for.
- [controller.md](controller.md) — the control math: trajectory preprocessing,
  error calculation, the steering law, the 4WS axis mapping, the reverse-mode
  sign flips, and the OBLIQUE/Q_TURN crab feedforward.
- [interface.md](interface.md) — ROS topic/service interface, full parameter
  table with units and tuning notes, and the layout of the `/debug/data`
  topic the node publishes every cycle.
- [troubleshooting.md](troubleshooting.md) — common failure modes (e.g. "cannot
  find closest waypoint", lateral drift, wrong-sign steer), the diagnostic
  recipes that isolate each one, and the parameter / launch knobs you'd
  reach for.

## Quick start

Build from the workspace root:

```bash
colcon build --symlink-install --packages-select motion_control
source install/setup.bash
```

Run standalone (rare — normally the FMS launch already starts it):

```bash
ros2 launch motion_control motion_control.launch.xml
```

The launch file expects upstream nodes to publish:

- `/localization/kinematic_state` (`nav_msgs/Odometry`) — ego pose + twist
- `/planning/scenario_planning/trajectory` (`autoware_planning_msgs/Trajectory`) — the planned trajectory
- `/planning/reverse_state` (`reverse_vehicle_msg/VehicleReversed`, latched) — direction-of-travel signal published by `igv_path_planner`

Outputs (see [interface.md](interface.md) for full QoS / message details):

- `/control/command/control_cmd` — high-level Control for `vehicle_cmd_gate`
- `/pacmod/{steering_cmd, motor_vel_cmd, shift_cmd, brake_cmd}` — direct
  pacmod commands (publishing of `motor_vel_cmd` / `steering_cmd` is gated on
  the `upper_test.enable` service flag; gear/brake init publish unconditionally)
- `/debug/data` — six floats of internal controller state every 20 ms

## Status / known limitations

- The controller's longitudinal command is **velocity-only** — it tracks the
  trajectory's longitudinal velocity at preview-index + 10 (~200 ms ahead at
  cruise) and lets pacmod's motor loop close acceleration. There is no
  explicit acceleration or jerk feedback.
- A separate `TrajectoryFollowing` class (pure-pursuit) and `PrecisePositioning`
  class exist in the source tree but are **not on the live signal path** —
  only `TrajectoryTracker` is invoked from `MotionController::onTimer()`.
- `findNearestIndex` uses a 2 m matching radius and aborts the cycle (emitting
  zero command) when it can't find a trajectory point within that radius —
  see [troubleshooting.md](troubleshooting.md) for the failure signature.
