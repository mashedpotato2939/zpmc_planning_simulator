# igv_path_planner

FMS **path** stage of the IGV planning chain: turns FMS route windows / paths
into an Autoware `Path`. Split out of the former `fms_trajectory_bridge` package
(velocity concerns now live in `igv_velocity_planner`; messages in
`igv_planning_msgs`).

## Nodes

| Executable | Class | Role |
|---|---|---|
| `igv_path_converter` | `FMSPathConverterNode` | FMS route window / path → Autoware `Path`. Maneuver detection, intelligent path generation (lane change, Q-turn, oblique), Lanelet2 centerline snapping, reverse-driving handling, and FMS uplink (vehicle status / trajectory report / job events). |
| `igv_trajectory_converter` | `FMSTrajectoryConverterNode` | **Legacy/deprecated** direct FMS-trajectory → Autoware-trajectory bridge. Superseded by `igv_path_converter` + `igv_velocity_planner`. |

## Libraries

- `path_generator` — Bezier / quintic / clothoid maneuver generators
  (`generators/oblique_generator`, `generators/qturn_generator`).
- `map_path_generator` — snaps FMS waypoints to Lanelet2 lane centerlines.
- `fms_protocol` — pluggable FMS protocol support (`default` ICD-aligned,
  `legacy` backward-compatible).

## Key topics (production wiring)

| Direction | Topic |
|---|---|
| in (FMS) | `/fms/route_window`, `/fms/stop_point`, `/fms/reference_path` |
| in | `/localization/kinematic_state` |
| **out (to velocity planner)** | `/planning/igv_path_planner/path` |
| out | `/planning/mission_planning/route`, `/fms/maneuver_type`, `/planning/reverse_state`, `/fms/vehicle_status`, `/fms/trajectory_report`, `/fms/job_event` |

`/fms/maneuver_type` and `/planning/reverse_state` are cross-cutting signals
consumed by the velocity smoother / controllers — they keep stable names.

## How it's launched (production)

In the real-vehicle / FMS deployment the node is **not** started from this
package's `launch/` directory. The chain is:

```
launch_with_fms_real_vehicle.sh           # repo root; planning + vehicle_agent terminal
  └─► igv_planning.launch.xml             # autoware_launch
        └─► components/tier4_planning_component_fms.launch.xml   # actual igv_path_converter wiring
```

The component file launches `igv_path_converter` directly with the production
remaps and feeds its `Path` into `igv_velocity_planner`. With `fms_enabled=true`
the whole stock `motion_planning` group is bypassed
(`launch_motion_planning=false`), so behavior planning, scenario selector,
path optimizer, etc. are not in the loop. The component file is the source of
truth for production wiring.

For ad-hoc / manual runs the package also ships a standalone launcher,
`launch/igv_path_converter.launch.xml`, which brings up just `igv_path_converter`
with the same remaps/params as the production chain (keep it in sync with the
node block in the component file):

```bash
ros2 launch igv_path_planner igv_path_converter.launch.xml
```

See `doc/` for per-node detail.

## Build & test

```bash
colcon build --symlink-install --packages-select igv_path_planner
colcon test --packages-select igv_path_planner
```
