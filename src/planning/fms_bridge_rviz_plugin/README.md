# fms_bridge_rviz_plugin

## Purpose

An RViz panel plugin for testing and interacting with the FMS Trajectory Bridge. It provides a GUI to manually construct and send FMS route windows, issue stop/resume commands, and monitor the bridge's live status — without needing a real Fleet Management System.

The panel publishes the same `FMSRouteWindow` and `FMSStopPoint` messages that a real FMS protocol plugin would deliver, making it suitable for end-to-end testing of the path generation pipeline.

## Panel Layout

The panel has four sections:

### Route Section

- **Route metadata**: Vehicle ID, Route ID, Job ID, Task ID text inputs.
- **Orientation**: Forward / Reverse selector (maps to `FMSPathSegment.orientation`).
- **Waypoint table**: Editable table with columns for X, Y, Z, Heading, Type, Speed.
- **Point input controls**: Spin boxes for coordinates and heading, point type selector (ICD-aligned values), and speed limit (m/s).
- **Point types**: `NORMAL` (0), `DESTINATION` (1), `LC_START` (2), `LC_END` (3), `POI` (4), `LOCAL_PLAN_START` (5), `LOCAL_PLAN_END` (6), `DRAWING` (7).
- **Buttons**: Add Point, Remove Last, Clear All, Save Points, Load Points, Send Route.

### Sliding Window Section

Controls how waypoints are published to the bridge.

- **Enable sliding window** checkbox: When unchecked (default), "Send Route" publishes all waypoints in a single message. When checked, it stores the full route and publishes a sliding window around the ego position at 1 Hz.
- **Ahead**: Window size ahead of ego in meters (default 100m).
- **Behind**: Window size behind ego in meters (default 10m).
- **Stop Window** button: Stops the sliding window timer and publishes `STATUS_CANCELLED`.
- **Status label**: Shows the current window range, nearest waypoint index, and ego position.

The sliding window preserves LC pair integrity — if the window boundary falls between `LC_START` and `LC_END`, it extends to include the full lane change zone.

### Stop / Resume Section

- **Stop position**: X, Y coordinate inputs (UTM).
- **Send Stop**: Publishes `FMSStopPoint` with `clear=false` at the specified coordinates.
- **Send Resume**: Publishes `FMSStopPoint` with `clear=true` to clear the active stop.

### Status Section

- Live display updated from the `/fms_bridge/adapter_status` topic showing:
  - State (IDLE / ACTIVE / SUSPENDED / ERROR)
  - Current route and job IDs
  - Stop active flag
  - Route progress (0.000 - 1.000)
  - Last error message (if any)

## Interfaces

### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/fms/route_window` | `igv_planning_msgs/FMSRouteWindow` | Sliding window route with waypoints |
| `/fms/stop_point` | `igv_planning_msgs/FMSStopPoint` | Stop command (`clear=false`) or resume (`clear=true`) |

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/fms_bridge/adapter_status` | `igv_planning_msgs/AdapterStatus` | Live bridge state and diagnostics |
| `/localization/kinematic_state` | `nav_msgs/Odometry` | Ego vehicle pose (for sliding window) |

## Save / Load Format

Waypoints can be saved to and loaded from JSON files. The format:

```json
{
  "vehicle_id": "v001",
  "route_id": "test_001",
  "job_id": "job_01",
  "task_id": "task_01",
  "orientation": 0,
  "points": [
    { "x": 50.0, "y": 5.0, "z": 0.0, "heading": 0.0, "type_text": "LC_START", "type_enum": 2, "speed": 5.0 },
    { "x": 100.0, "y": 9.0, "z": 0.0, "heading": 0.0, "type_text": "LC_END", "type_enum": 3, "speed": 3.0 },
    { "x": 150.0, "y": 9.0, "z": 0.0, "heading": 0.0, "type_text": "DESTINATION", "type_enum": 1, "speed": 0.0 }
  ]
}
```

## Usage

### Build

```bash
colcon build --packages-select fms_bridge_rviz_plugin --symlink-install
source install/setup.bash
```

### Load in RViz

1. Launch RViz (standalone or via an Autoware launch file with `launch_rviz:=true`).
2. Go to **Panels** > **Add New Panel**.
3. Select **FmsBridgePanel** from the list.

### Workflow (one-shot mode)

1. Fill in route metadata (Vehicle ID, Route ID, Job ID, Task ID).
2. Select orientation (Forward / Reverse).
3. Add waypoints using the input controls and **Add Point** button.
   - Use `LC_START` and `LC_END` to define lane change zones.
   - Use `DESTINATION` for the final waypoint.
   - Set `speed_limit` to 0 on destination to trigger deceleration profile.
4. Leave the "Enable sliding window" checkbox unchecked.
5. Click **Send Route** to publish the full `FMSRouteWindow` to the bridge.
6. Monitor the **Status** section for bridge state changes.
7. Use **Send Stop** / **Send Resume** to test mid-route stop behavior.

### Workflow (sliding window mode)

1. Add waypoints as above, or **Load Points** from a JSON file.
2. Check **Enable sliding window**.
3. Set **Ahead** (default 100m) and **Behind** (default 10m) distances.
4. Click **Send Route** — the plugin stores the full route and starts publishing a windowed subset at 1 Hz.
5. As the vehicle moves, the window slides: passed waypoints are trimmed and new ones appear.
6. The window status label shows the current point range and ego position.
7. Click **Stop Window** to halt the timer and send `STATUS_CANCELLED`.

## Dependencies

- `rclcpp`
- `rviz_common`
- `pluginlib`
- `igv_planning_msgs` (message definitions: `FMSRouteWindow`, `FMSStopPoint`, `SegmentWaypoint`, `AdapterStatus`)
- `nav_msgs`
- `std_msgs`
- Qt5 (`libqt5-core`, `libqt5-widgets`, `qtbase5-dev`)
