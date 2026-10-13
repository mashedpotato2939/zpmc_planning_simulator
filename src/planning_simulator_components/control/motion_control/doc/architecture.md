# Architecture

This document describes where `motion_control` sits in the stack, the data
flow through one 20 ms control cycle, and the role of each source-tree
component. For the control math itself, see [controller.md](controller.md).
For the ROS interface and parameters, see [interface.md](interface.md).

## Place in the stack

```
        ┌───────────────────────────────────────────────────────────────┐
        │                       planning stack                          │
        │  (FMS chain or full Autoware)                                 │
        └─────────────┬──────────────────────────────────────┬──────────┘
                      │                                      │
                      │  /planning/scenario_planning/        │  /planning/reverse_state
                      │      trajectory                      │  (latched, transient_local)
                      │  autoware_planning_msgs/Trajectory   │  reverse_vehicle_msg/VehicleReversed
                      ▼                                      ▼
        ┌───────────────────────────────────────────────────────────────┐
        │                       motion_control                          │
        │  ┌────────────────┐    ┌──────────────────┐                   │
        │  │ MotionController│──▶│ TrajectoryTracker│                   │
        │  │  (timer 20 ms)  │   │  (run() →        │                   │
        │  └────────────────┘    │   TrackerCmd)    │                   │
        │           ▲            └──────────────────┘                   │
        │           │  /localization/kinematic_state                    │
        └───────────┴───────────────────────────────────────────────────┘
                      │                                      │
                      │ /control/command/control_cmd         │ /pacmod/{steering,motor_vel,shift}_cmd
                      │ autoware_control_msgs/Control        │
                      ▼                                      ▼
        ┌──────────────────────────┐         ┌──────────────────────────┐
        │   vehicle_cmd_gate       │         │      pacmod bus          │
        │   (AEB / OMT / shift)    │         │   (vehicle interface)    │
        └──────────────────────────┘         └──────────────────────────┘
```

The node publishes **both** a high-level `autoware_control_msgs/Control`
(consumed by `vehicle_cmd_gate` for safety filtering / mode arbitration)
and **direct pacmod commands** (when `upper_test.enable` has been set true
via the `/srv_upper_test` service — typically by an operator app). When
`upper_test.enable == false`, the pacmod commands publish zeros each
cycle so a stuck-on motor command can't carry across an operator pause.
The high-level `Control` is published every cycle regardless, because
`vehicle_cmd_gate` uses its arrival as a liveness heartbeat.

## Single-cycle data flow

`MotionController::onTimer()` fires every 20 ms (`t_interval`). One cycle:

1. **Data gate** (`isDataReady`). Bail if no trajectory or no fresh odometry.
2. **Gear command**. Publish pacmod `shift_cmd` reflecting `is_forward`
   (1 = reverse, 3 = drive). This is published every cycle, not just on
   state change, because the pacmod loop expects a continuous command.
3. **Update tracker inputs**.
   - If the trajectory changed: `tracker_->setTrajectory(...)`.
   - Always: `tracker_->setOdometry(...)` and `tracker_->setReverseCmd(is_forward)`.
4. **Run the tracker** (`tracker_->run()`). This is the entire control law
   — see [controller.md](controller.md) for the math. Returns `true` on
   success with `tracker_->cmd` populated, or `false` to indicate "no
   valid command this cycle" (e.g. no trajectory point within the search
   radius).
5. **Publish `/debug/data`** — six floats with the current ey, delta,
   delta_l, eyaw, d_delta, d_delta_l. This is always published, even on
   failure, so the diagnostic record stays continuous. See
   [interface.md](interface.md) for field semantics.
6. **Emit commands**.
   - On success **and** `upper_test.enable == true`: publish pacmod
     `motor_vel_cmd` and `steering_cmd` with the tracker's values.
   - On success but `upper_test.enable == false`: publish zeros to pacmod
     (the high-level Control still carries the real values).
   - On failure: publish zeros to pacmod and zeros in the high-level
     Control's lateral/longitudinal fields, and emit
     `RCLCPP_ERROR_THROTTLE "trajectory_tracker run failed"`.
7. **Publish high-level `Control`** to `output/control_cmd` (remapped in
   the launch to `/control/command/control_cmd`). The longitudinal
   velocity sign is restored from `is_forward` — the tracker keeps
   `velocity_cmd >= 0` internally; the sign lives in the gear and is
   pasted back here for `vehicle_cmd_gate`, which flips steering for
   reverse but **not** velocity.

## Source tree

```
motion_control/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── motion_control.param.yaml      # default parameters (see interface.md)
├── launch/
│   ├── motion_control.launch.xml      # main launch
│   └── interface_test.launch.xml      # bench-test launch
├── rviz/
│   └── motion_control.rviz
├── include/
│   ├── control/
│   │   ├── motion_controller.hpp      # ROS node
│   │   ├── trajectory_tracker.hpp     # the 4WS preview controller (LIVE)
│   │   ├── trajectory_following.hpp   # pure-pursuit follower (not on signal path)
│   │   └── precise_positioning.hpp    # stub (not on signal path)
│   ├── control_utils/
│   │   ├── geometry.hpp               # angle, distance, projection helpers
│   │   ├── interpolation.hpp          # spline / lerp utilities
│   │   ├── trajectory.hpp             # TrajectoryPoint helpers
│   │   └── pid.hpp                    # generic PID (used by interface_test)
│   ├── interface_test/
│   │   └── interface_test.hpp
│   └── map_traj_server/
│       └── map_traj_server.hpp
└── src/
    ├── control/
    │   ├── motion_controller_main.cpp   # control_exe entry
    │   ├── motion_controller.cpp        # ROS plumbing
    │   ├── trajectory_tracker.cpp       # the LIVE controller
    │   ├── trajectory_following.cpp     # pure-pursuit (not on signal path)
    │   └── precise_positioning.cpp      # stub
    ├── control_utils/
    │   └── … (matches headers)
    ├── interface_test/
    │   ├── interface_test_main.cpp
    │   └── interface_test.cpp           # bench-test driver for pacmod
    └── map_traj_server/
        ├── map_traj_server.cpp          # offline clothoid / crab generator (map_traj_server_exe)
        ├── readme.md                    # 直角弯 + 斜行 generator notes
        ├── load.txt                     # sample boundary input
        └── vel_change.txt               # sample velocity profile
```

### What's live, what isn't

| Component | On signal path | Purpose |
|---|---|---|
| `MotionController` | yes | ROS node, 20 ms timer, command publication |
| `TrajectoryTracker` | yes | The actual 4WS preview controller |
| `TrajectoryFollowing` | **no** | Pure-pursuit follower; constructed by `MotionController` but never invoked from `onTimer()` |
| `PrecisePositioning` | **no** | Empty stub kept as a placeholder |
| `map_traj_server` | only under `use_maptraj_server:=true` | Standalone exe that emits clothoid-arc-clothoid corners and crab-walk segments offline; primarily for bench validation, not normal operation |
| `interface_test` | only as a separate exe | Pacmod bench harness |

If you're investigating the runtime behavior, only `MotionController` and
`TrajectoryTracker` matter.

## Reverse-state plumbing

`/planning/reverse_state` is published by `igv_path_planner::FMSPathConverterNode`
with **`reliable + transient_local`** QoS. The motion_control subscription
also uses `reliable + transient_local`, so a late-joining node receives
the current state immediately. The callback updates a single bool
(`tracker_param_->is_forward = !msg->is_reversed`), which `MotionController::onTimer()`
pushes into the tracker on every cycle via `setReverseCmd(...)`.

`is_forward` propagates to:

- The gear command (1 vs 3) at the top of `onTimer`.
- `traj_preprocess`, which π-flips the per-point trajectory orientation
  and ego orientation in reverse so all math runs in a consistent
  motion-direction frame.
- `calcSteerCmd`, which selects the `*_reverse` gain set and applies the
  per-axis sign flip needed for body-frame steering when the vehicle is
  reversing.
- The high-level `Control` longitudinal sign restoration after the tracker.

See [controller.md](controller.md) for the full reverse-mode rationale.
