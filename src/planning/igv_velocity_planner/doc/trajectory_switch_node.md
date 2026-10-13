# trajectory_switch_node

Runtime switch that selects between FMS and Autoware trajectories, with staleness checks, automatic fallback, and external control via service and parameter.

## Overview

This node acts as a multiplexer between two trajectory sources: FMS trajectories and native Autoware trajectories. It allows runtime switching between sources and provides automatic fallback if the selected source becomes unavailable.

## Architecture

```
                        ┌─────────────────────────────────────┐
                        │       trajectory_switch_node        │
                        │                                     │
~/input/fms_trajectory ─┼──► fmsCallback()                    │
                        │        │                            │
                        │        ▼                            │
                        │   [fms_traj_ + fms_last_time_]      │
                        │                                     │
~/input/autoware_traj ──┼──► autowareCallback()               │
                        │        │                            │
                        │        ▼                            │
                        │   [autoware_traj_ + time]           │
                        │                                     │
                        │   timerCallback() @ publish_rate    │
                        │        │                            │
                        │        ▼                            │
                        │   ┌─────────────────┐               │
                        │   │ use_fms_ = true │               │
                        │   └────────┬────────┘               │
                        │            │                        │
                        │     FMS available?                  │
                        │      /          \                   │
                        │    Yes           No                 │
                        │     │             │                 │
                        │   [FMS]     [Fallback to           │
                        │             Autoware]               │
                        │                                     │
                        │   ┌──────────────────┐              │
                        │   │ use_fms_ = false │              │
                        │   └────────┬─────────┘              │
                        │            │                        │
                        │     [Autoware traj]                 │──► ~/output/trajectory
                        │                                     │
                        └─────────────────────────────────────┘
                                     ▲
                                     │
               ~/switch_to_fms ──────┘ (SetBool service)
               use_fms_trajectory param ─┘ (dynamic reconfigure)
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_fms_trajectory` | bool | `false` | Initial mode: true=FMS, false=Autoware |
| `publish_rate` | double | `20.0` | Output publishing rate (Hz) |
| `timeout` | double | `1.0` | Staleness threshold for trajectories (s) |

## Topics

### Subscriptions

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `~/input/fms_trajectory` | `autoware_planning_msgs/Trajectory` | Default (10) | FMS trajectory input |
| `~/input/autoware_trajectory` | `autoware_planning_msgs/Trajectory` | Default (10) | Autoware trajectory input |

### Publications

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `~/output/trajectory` | `autoware_planning_msgs/Trajectory` | Default (10) | Selected trajectory output |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `~/switch_to_fms` | `std_srvs/SetBool` | Switch trajectory source: `true`=FMS, `false`=Autoware |

## Switching Logic

### When `use_fms_ = true`:

```
if (fms_traj exists AND age < timeout):
    output = fms_traj
else:
    WARN "FMS not available, falling back"
    if (autoware_traj exists AND age < timeout):
        output = autoware_traj
    else:
        WARN "No valid trajectory"
```

### When `use_fms_ = false`:

```
if (autoware_traj exists AND age < timeout):
    output = autoware_traj
else:
    WARN "No valid trajectory"
```

### Staleness Check

Each trajectory is considered stale if:
```cpp
(now() - last_receive_time).seconds() > timeout
```

## Dynamic Reconfiguration

The `use_fms_trajectory` parameter can be changed at runtime:

```bash
# Via parameter
ros2 param set /trajectory_switch use_fms_trajectory true

# Via service
ros2 service call /trajectory_switch/switch_to_fms std_srvs/srv/SetBool "{data: true}"
```

Both methods trigger the same mode change and log message.

## Usage Examples

### Launch with FMS Mode

```bash
ros2 run igv_velocity_planner trajectory_switch --ros-args \
  -p use_fms_trajectory:=true
```

### Switch to Autoware at Runtime

```bash
ros2 service call /trajectory_switch/switch_to_fms std_srvs/srv/SetBool "{data: false}"
```

### With Custom Timeout

```bash
ros2 run igv_velocity_planner trajectory_switch --ros-args \
  -p timeout:=2.0 \
  -p publish_rate:=30.0
```

## Output Behavior

- Output trajectory has timestamp updated to current time
- Published at constant rate regardless of input rate
- No modifications to trajectory content (points, velocities, etc.)
- If no valid trajectory available, nothing is published (with throttled warning)

## Typical Integration

```
[FMS Path Converter] ──► ~/input/fms_trajectory
                                               ┐
                                               ├──► [Trajectory Switch] ──► [Controller]
[Autoware Planning] ──► ~/input/autoware_traj ┘
```

## Warning Messages

| Message | Condition |
|---------|-----------|
| "FMS trajectory not available, falling back to Autoware" | FMS mode but FMS stale, Autoware available |
| "No valid trajectory available" | Neither source has fresh trajectory |

Both warnings are throttled to every 5 seconds.

## Related Nodes

- **fms_path_converter_node**: Produces FMS trajectories
- **fms_velocity_shaper_node**: Single owner of obstacle-driven v(s) in FMS mode (replaces trajectory_relay + obstacle_cruise_planner)
