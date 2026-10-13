# fms_path_converter_node

Converts incoming FMS paths (binary or JSON) into Autoware `Path` + synthetic `LaneletRoute`, with geometry smoothing, maneuver-aware interpolation, reverse driving support, and curvature-based drivable area expansion for large vehicles.

## Overview

This node serves as the bridge between FMS path messages and Autoware's planning stack. It transforms sparse FMS waypoints into dense, smooth paths suitable for Autoware's local planners, while also generating the required route and drivable bounds.

## Architecture

```
                        ┌───────────────────────────────────────────────────────┐
                        │             fms_path_converter_node                   │
                        │                                                       │
~/input/route_window ───┼──► DefaultProtocol                                    │
~/input/stop_point ─────┼──► DefaultProtocol                                    │
legacy FMSPath/JSON ────┼──► LegacyProtocol → FMSRouteWindow                    │
                        │        │                                              │
                        │        ▼                                              │
                        │   convertRouteWindowToAutowarePath()                  │
                        │        │                                              │
                        │        ├──► PathGenerator (intelligent mode)          │
                        │        │    or interpolatePath() (simple mode)        │
                        │        │                                              │
                        │        ▼                                              │
                        │   cropPathFromEgo()                                   │
                        │        │                                              │
                        │        ▼                                              │
                        │   addVelocityProfileForWindow()                       │
                        │        │                                              │
                        │        ▼                                              │
                        │   generateDrivableBounds()                            │──► ~/output/path
                        │        │                                              │
                        │        ▼                                              │
                        │   publishRouteFromWindow()                            │──► ~/output/route
                        │                                                       │
                        └───────────────────────────────────────────────────────┘
```

## Parameters

### Core Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `frame_id` | string | `"map"` | Reference frame for output messages |
| `default_velocity` | double | `5.0` | Default velocity when not specified (m/s) |
| `min_velocity` | double | `0.5` | Minimum velocity clamp (m/s) |
| `max_velocity` | double | `15.0` | Maximum velocity clamp (m/s) |
| `path_interval` | double | `1.0` | Spacing between interpolated points (m) |
| `forward_path_length` | double | `100.0` | Path length to keep ahead of ego (m) |
| `backward_path_length` | double | `5.0` | Path length to keep behind ego (m) |
| `stale_threshold` | double | `5.0` | Time before marking FMS as disconnected (s) |
| `publish_rate` | double | `10.0` | Output publishing rate (Hz) |

### Path Processing Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ego_nearest_dist_threshold` | double | `3.0` | Distance threshold for ego segment search (m) |
| `ego_nearest_yaw_threshold` | double | `1.57` | Yaw threshold for ego segment search (rad) |
| `enable_velocity_profile` | bool | `true` | Enable deceleration profile near goal |
| `fms_protocol.legacy.use_json_input` | bool | `false` | Enable legacy JSON input subscription |

### Intelligent Path Generation Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_intelligent_path_generation` | bool | `true` | Use PathGenerator for lane changes/turns |
| `lane_change_lateral_threshold` | double | `1.5` | Lateral offset to detect lane change (m) |
| `turn_yaw_threshold` | double | `0.3` | Yaw change to detect turn (rad ~17°) |
| `min_lane_change_length` | double | `10.0` | Minimum lane change distance (m) |
| `lane_change_velocity_ratio` | double | `0.8` | Speed reduction during lane change |
| `turn_speed` | double | `3.0` | Maximum speed during turns (m/s) |
| `qturn_speed_limit` | double | `3.0` | Maximum speed during Q-turns (m/s) |
| `oblique_speed_limit` | double | `3.0` | Maximum speed during oblique lane changes (m/s) |

### Drivable Area Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `generate_drivable_area` | bool | `true` | Generate synthetic left/right bounds |
| `lane_width` | double | `3.5` | Default lane width (m) |
| `left_bound_offset` | double | `-1.0` | Left bound distance (-1 = lane_width/2) |
| `right_bound_offset` | double | `-1.0` | Right bound distance (-1 = lane_width/2) |

### Curvature Expansion Parameters (for large vehicles)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_curvature_expansion` | bool | `true` | Expand bounds in curves |
| `vehicle_length` | double | `15.0` | Vehicle total length (m) |
| `vehicle_width` | double | `3.0` | Vehicle width (m) |
| `vehicle_front_overhang` | double | `4.5` | Front overhang (m) |
| `vehicle_wheel_base` | double | `6.0` | Wheelbase (m) |
| `min_bound_margin` | double | `0.5` | Minimum safety margin (m) |

## Topics

### Subscriptions

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `~/input/fms_path` | `igv_planning_msgs/FMSPath` | Default (10) | FMS path input (binary) |
| `~/input/fms_json` | `std_msgs/String` | Default (10) | FMS path input (JSON, if enabled) |
| `~/input/odometry` | `nav_msgs/Odometry` | SensorDataQoS | Vehicle odometry for cropping |

### Publications

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `~/output/path` | `autoware_planning_msgs/Path` | Default (10) | Converted Autoware path |
| `~/output/route` | `autoware_planning_msgs/LaneletRoute` | Reliable + TransientLocal (1) | Synthetic route for scenario selector |
| `~/output/status` | `igv_planning_msgs/FMSStatus` | Default (10) | Conversion status |
| `/planning/reverse_state` | `reverse_vehicle_msg/VehicleReversed` | TransientLocal (10) | Reverse driving state |

### Service Clients

| Service | Type | Description |
|---------|------|-------------|
| `set_reverse_vehicle_state_service` | `reverse_vehicle_msg/SetVehicleReverseState` | Update reverse planner state |

## Key Features

### Intelligent Path Generation

When `enable_intelligent_path_generation` is true, the node uses `PathGenerator` to create smooth paths:

- **Lane Changes**: Detected by lateral offset > 1.5m with small yaw change
- **Turns**: Detected by yaw change > 17°
- **Curves**: Detected by yaw change > 6°

Each maneuver type uses appropriate interpolation (Bezier curves for turns/lane changes, linear for straight segments).

### Reverse Driving Detection

The node auto-detects reverse driving from FMS paths using:

1. **is_reverse flag**: Majority of waypoints have `is_reverse = true`
2. **Negative velocity**: Majority of waypoints have `velocity < 0`

When detected, the node:
- Publishes reverse state to `/planning/reverse_state`
- Calls the reverse planner service
- Preserves negative velocities in output path

### Curvature-Based Drivable Area Expansion

For large vehicles (like IGV), the node expands drivable bounds in curves using the Ackermann steering model:

```
R_front_outer = sqrt((R + w/2)² + (L + a)²)
R_rear_outer = sqrt((R + w/2)² + b²)
min_width = 2 * (max(R_front, R_rear) - R) + w + 2*margin
```

Where:
- `R` = turn radius (1/curvature)
- `w` = vehicle width
- `L` = wheelbase
- `a` = front overhang
- `b` = rear overhang

This ensures the vehicle's swept path fits within the drivable bounds during turns.

### Velocity Profile

When enabled, applies deceleration profile near the goal:
- Starts 10m before goal
- Linear ramp from default velocity to 1.0 m/s
- Zero velocity at goal point

### Path Cropping

Uses Autoware's standard cropping functions:
- `findFirstNearestSegmentIndexWithSoftConstraints` for ego position
- `cropPoints` to extract forward/backward segments

## JSON Input Format

When `fms_protocol.legacy.use_json_input` is enabled, `LegacyProtocol` accepts JSON on `~/input/fms_json`:

```json
{
  "path_id": "path_001",
  "task_id": "task_001",
  "waypoints": [
    {
      "x": 100.0,
      "y": 50.0,
      "z": 0.0,
      "yaw": 1.57,
      "velocity": 5.0,
      "acceleration": 0.0,
      "curvature": 0.0,
      "time_from_start": 0.0,
      "type": 0,
      "lane_id": 1
    }
  ],
  "left_bound": [{"x": 99.0, "y": 50.0, "z": 0.0}],
  "right_bound": [{"x": 101.0, "y": 50.0, "z": 0.0}]
}
```

## Usage Examples

### Launch with Default Parameters

```bash
ros2 run igv_path_planner igv_path_converter
```

### Launch for Large Vehicle

```bash
ros2 run igv_path_planner igv_path_converter --ros-args \
  -p enable_curvature_expansion:=true \
  -p vehicle_length:=15.0 \
  -p vehicle_width:=3.0 \
  -p vehicle_wheel_base:=6.0
```

### Monitor Status

```bash
ros2 topic echo ~/output/status
```

## FMSStatus Fields

| Field | Type | Description |
|-------|------|-------------|
| `fms_connected` | bool | FMS communication status |
| `last_message_time` | float64 | Seconds since last FMS message |
| `current_trajectory_id` | string | Current path ID |
| `current_task_id` | string | Current task ID |
| `waypoints_remaining` | uint32 | Remaining waypoints in path |
| `trajectory_valid` | bool | Output path is valid |
| `trajectory_stale` | bool | FMS data is stale |
| `distance_to_goal` | float64 | Distance to goal (m) |
| `last_error` | string | Last conversion error |

## Related Nodes

- **fms_goal_path_generator_node**: Generates FMS paths from goal poses
- **path_generator** (library): Maneuver-aware path interpolation
- **trajectory_switch_node**: Switches between FMS and Autoware trajectories
