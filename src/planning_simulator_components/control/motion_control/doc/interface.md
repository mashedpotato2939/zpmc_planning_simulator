# ROS interface and parameters

This document lists every topic, service, and parameter `motion_control`
touches, with QoS settings and tuning notes. For the math that produces
these I/Os, see [controller.md](controller.md).

## Node

- **Name**: `motion_control`
- **Executable**: `control_exe`
- **Timer**: 20 ms (`t_interval`), single-threaded executor
- **Namespace**: root

## Subscriptions

| Topic | Type | QoS | Purpose |
|---|---|---|---|
| `input/kinematics` → `/localization/kinematic_state` | `nav_msgs/Odometry` | default (KeepLast 1, Reliable) | Ego pose + twist. Filtered through a third-order tracking differentiator before use. |
| `input/trajectory` → `/planning/scenario_planning/trajectory` | `autoware_planning_msgs/Trajectory` | default (KeepLast 1, Reliable) | Planned trajectory in `map` frame. Cached pointer; reprocessed each cycle. |
| `/planning/reverse_state` | `reverse_vehicle_msg/VehicleReversed` | **KeepLast 1, Reliable, TransientLocal** | Direction-of-travel signal. Published latched by `igv_path_planner`. Updates `is_forward` for the next cycle. |

`input/kinematics` and `input/trajectory` are remapped in the launch
file to the topics shown after the arrow.

## Publications

| Topic | Type | QoS | Purpose |
|---|---|---|---|
| `output/control_cmd` → `/control/command/control_cmd` | `autoware_control_msgs/Control` | KeepLast 1, Reliable | High-level control command for `vehicle_cmd_gate`. Carries `steering_tire_angle` (axis2 — front), `rear_steering_tire_angle` (axis3 — rear), `is_defined_rear_steering=true`, and `longitudinal.velocity` with the correct sign (negative when reversing). |
| `/pacmod/motor_vel_cmd` | `pacmod_msgs/SystemCmdFloat` | KeepLast 1, Reliable | Direct longitudinal command to the pacmod motor loop. **Publishes zeros** when `upper_test.enable == false`. |
| `/pacmod/steering_cmd` | `pacmod_msgs/SteeringCmd` | KeepLast 1, Reliable | Direct per-axis steering command (axis1..4). Same gating as `motor_vel_cmd`. |
| `/pacmod/shift_cmd` | `pacmod_msgs/SystemCmdInt` | KeepLast 1, Reliable | Gear: 1 = reverse, 3 = drive. Published every cycle. |
| `/pacmod/brake_cmd` | `pacmod_msgs/SystemCmdFloat` | KeepLast 1, Reliable | Brake command. Only published during `Vehicle_Init()` (once at startup, to release brake). |
| `/debug/data` | `std_msgs/Float32MultiArray` | KeepLast 10, Reliable | Six controller-internal floats — see "Debug topic layout" below. Published every cycle, even on failure. |

`output/control_cmd` is remapped in the launch.

## Services

| Service | Type | Direction | Purpose |
|---|---|---|---|
| `/srv_upper_test` | `pacmod_msgs/srv/UpperTest` | server | Operator-side enable/test interface. The `enable` flag in the request gates whether pacmod commands carry real values or zeros — used to safely arm the vehicle. |

The package code also wires up clients for `autoware_control_msgs/srv/MapCreatorService`
and `TrajCreatorService`, but they're only used when the legacy
`use_traj_server` flow is enabled (off by default — see
[architecture.md](architecture.md)).

## Parameters

All parameters are declared on node construction with the defaults
shown in the table. The configured values in
`config/motion_control.param.yaml` typically override these. Vehicle
geometry comes from the selected model's `vehicle_info.param.yaml` when
the launch chain passes a `vehicle_param_file`; its `wheel_base` and
`max_steer_angle` values override the legacy motion-control names
`wheelbase` and `max_steering_angle`. The standalone
`launch/motion_control.launch.xml` also loads `vehicle_param_file` and
then `config/motion_control.param.yaml`.

### General

| Parameter | Default (code) | YAML | Units | Description |
|---|---:|---:|---|---|
| `trk_omega` | 10.0 | 5.0 | rad/s | Tracking-differentiator bandwidth for the pose filter. Higher → faster response to odom but noisier `vx`/`vy` estimates. The `ey` differentiator is **hard-coded** to ω = 5 inside the tracker constructor. |
| `t_interval` | 0.02 | 0.02 | s | Controller timestep. Must match the timer period in `MotionController` — currently 20 ms is hard-coded in `onTimer`. |
| `wheel_base` / `wheelbase` | 2.3 | vehicle-specific / 6.0 fallback | m | Total wheelbase (axis1 to axis4). `wheel_base` from `vehicle_info.param.yaml` wins; `wheelbase` is the legacy fallback. |
| `wheelbase_ratio` | 2.3 | 0.431 | — | (axis2↔axis3 wheelbase) / (axis1↔axis4 wheelbase). Used to scale the differential steer between front-pair and rear-pair axles. For the IGV: 50/116 ≈ 0.431. |
| `max_steer_angle` / `max_steering_angle` | 0.55 | vehicle-specific / 0.78 fallback | rad | Per-axis absolute clamp on steer angle. `max_steer_angle` from `vehicle_info.param.yaml` wins; `max_steering_angle` is the legacy fallback. |
| `delta_limit` | `max_steering_angle` | 0.5236 | rad | Soft clamp on the **base** (crab) steer term before axis mixing. ~30° at 0.5236. |
| `d_delta_limit` | `max_steering_angle` | 0.1 | rad | Soft clamp on the **differential** steer term. ~5.7° at 0.1 keeps yaw-rate response calm relative to crab response. |
| `predicted_time` | 0.1 | 0.3 | s | Preview horizon for `calc_preview_point`. 0 = feedback only; ~0.3 s is typical. |
| `trk_vxmin` | 0.2 | 0.1 | m/s | Lower clamp on `|vx|` used inside the proportional gain denominator. Stops the gain from blowing up at standstill. |
| `is_forward` | true | true | — | Initial driving direction. Overwritten by `/planning/reverse_state` callbacks at runtime. |
| `use_traj_server` | false | false | — | If true, run the legacy `map_traj_server` flow at startup. Off in production. |

### Forward gains

| Parameter | Default (code) | YAML | Description |
|---|---:|---:|---|
| `trk_ky` | -1.0 | 0.3 | Lateral-error proportional gain in forward mode. |
| `trk_kyaw` | -1.0 | 0.8 | Yaw-error proportional gain in forward mode. |
| `trk_kvy` | 0.0 | 0.0 | Lateral-velocity (ėy) damping gain. Off by default. |
| `trk_kcom` | 0.0 | 0.0 | Centripetal-compensation feedforward (`vx · ωz`). Off by default. |

### Reverse gains

| Parameter | Default (code) | YAML | Description |
|---|---:|---:|---|
| `trk_ky_reverse` | -1.0 | 0.3 | Lateral-error gain in reverse mode. **Sign-sensitive** — see the comment block in `motion_control.param.yaml` for the polarity derivation. |
| `trk_kyaw_reverse` | -0.5 | 0.8 | Yaw-error gain in reverse mode. Same polarity story as `trk_ky_reverse`. |

`kvy` and `k_com` are shared between forward and reverse — they're
multiplied by velocity terms that already carry direction information.

Note that the code defaults and the YAML defaults differ. The YAML is
the **source of truth at runtime** when running under any launch that
loads `config/motion_control.param.yaml`; the code-side defaults are
fallback values that only apply if the YAML is bypassed (e.g. unit
tests or running `control_exe` standalone without a parameter file).

The `launch/motion_control.launch.xml` further overrides
`trk_ky_reverse: -0.3` and `trk_kyaw_reverse: -0.3` — see the launch
file for the rationale.

## Debug topic layout (`/debug/data`)

`std_msgs/Float32MultiArray` published every 20 ms. Six entries:

| index | name | meaning | typical range |
|---:|---|---|---|
| 0 | `ey` | lateral error in metres. Sign: + when path target is to the **left** of preview pose (motion-direction frame, after π-flip in reverse). | ±0.5 in nominal tracking; >2.0 indicates a tracker abort is imminent |
| 1 | `delta` | pre-clamp base (crab) steer command in rad. | ±0.5 |
| 2 | `delta_l` | post-clamp base steer in rad — what actually goes into the axis mixing. | ±0.5236 (= `delta_limit` from yaml) |
| 3 | `eyaw` | yaw error in rad, motion-direction frame. | ±0.05 in nominal tracking |
| 4 | `d_delta` | pre-clamp differential (yaw-rate) steer in rad. | ±0.1 |
| 5 | `d_delta_l` | post-clamp differential steer in rad. | ±0.1 (= `d_delta_limit` from yaml) |

Useful echo recipes:

```bash
# Raw values
ros2 topic echo /debug/data --field data

# Plot in PlotJuggler — open /debug/data and select data[0], data[1], data[3]
plotjuggler

# Time-aligned with the high-level command
ros2 topic echo /control/command/control_cmd \
  --field lateral.steering_tire_angle,lateral.rear_steering_tire_angle,longitudinal.velocity
```

When the tracker fails (`run()` returns false), the **previous**
debug values stay frozen because `MotionController` reads them from
the tracker's member fields — which the failure path doesn't update.
Watching for a frozen `/debug/data` is a reliable indicator that the
tracker is in the abort branch.

## QoS gotchas

- `/planning/reverse_state` is **transient_local**. If you subscribe to
  it elsewhere for debugging, use the matching QoS or `ros2 topic echo`
  with `--qos-durability transient_local --qos-reliability reliable`.
- The pacmod commands publish **every cycle** but the gating on
  `upper_test.enable` flips them to zero when disabled. A naïve
  `ros2 topic hz /pacmod/steering_cmd` reports a healthy 50 Hz in
  either state — look at the actual values, not the rate.
