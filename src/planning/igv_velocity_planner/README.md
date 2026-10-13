# igv_velocity_planner

FMS **velocity** stage of the IGV planning chain: shapes an Autoware `Path` into
an obstacle-aware, kinematically-feasible `Trajectory`. Split out of the former
`fms_trajectory_bridge` package (path concerns now live in `igv_path_planner`).
This package has **no custom-message dependency** — it speaks standard Autoware
planning/perception types.

## Nodes

| Executable | Class | Role |
|---|---|---|
| `igv_velocity_shaper` | `FmsVelocityShaperNode` | Single owner of obstacle-driven v(s): `Path` + perception → `Trajectory`. Runs constraint generators (static-obstacle slowdown + footprint-aware static stop, dynamic stop, free-space occupancy, slow-down, adaptive cruise), merges their `VelocityConstraint`s by priority/min, and writes the raw velocity profile. |
| `igv_kinematic_smoother` | `FmsKinematicSmootherNode` | Slim port of `autoware_velocity_smoother` (Analytical algorithm, no OSQP). Jerk/accel/lateral-accel/steering-rate feasibility + OBLIQUE/Q_TURN body-yaw cache. |
| `detected_objects_to_grid` | `DetectedObjectsToGridNode` | **Off-by-default fallback** (`enable_detection_grid`): rasterizes close-range `DetectedObjects` into an `OccupancyGrid` that feeds the shaper's occupancy pathway in place of `/perception/occupancy_grid_map/map`. |
| `simple_object_predictor` | `SimpleObjectPredictorNode` | Constant-velocity `TrackedObjects` → `PredictedObjects` (no map dependency). |
| `trajectory_switch` | `TrajectorySwitchNode` | Runtime switch between FMS and Autoware trajectories with timeout fallback. |

## Libraries

- `velocity_shaper_lib` — constraint generators + merger + distance map under `src/velocity_shaper/`.
- `kinematic_smoother_lib` — Analytical smoother under `src/kinematic_smoother/`.
- `detected_objects_grid_lib` — DetectedObjects→OccupancyGrid rasterizer (fallback occupancy source).

## Key topics (production wiring)

| Direction | Topic |
|---|---|
| **in (from path planner)** | `/planning/igv_path_planner/path` |
| in | `/perception/object_recognition/objects`, `/perception/occupancy_grid_map/map`, `/localization/kinematic_state` |
| **out (shaper → smoother)** | `/planning/igv_velocity_planner/trajectory` |
| out (smoother → stock velocity_smoother) | `/planning/scenario_planning/velocity_smoother/trajectory` |
| out (diag/viz) | `/planning/velocity_factors/igv_velocity_planner`, `/planning/igv_velocity_planner/stop_markers`, `/planning/scenario_planning/status/stop_reasons` |

`/planning/igv_velocity_planner/stop_markers` publishes the shaper's hard-stop
RViz diagnostics: red stop walls, stop-point spheres, optional obstacle
connectors, and reason text with stop distance/gap.

## How it's launched (production)

In the real-vehicle / FMS deployment the nodes are **not** started from this
package's `launch/` directory. The chain is:

```
launch_with_fms_real_vehicle.sh           # repo root; planning + vehicle_agent terminal
  └─► igv_planning.launch.xml             # autoware_launch
        └─► components/tier4_planning_component_fms.launch.xml   # actual node wiring
```

The component file launches `igv_velocity_shaper` and `igv_kinematic_smoother`
directly with the production remaps, consuming `/planning/igv_path_planner/path`
from `igv_path_planner` and emitting into the stock
`/planning/scenario_planning/velocity_smoother/trajectory`. The shaper's
free-space occupancy pathway consumes `/perception/occupancy_grid_map/map`
directly (gated by `use_occupancy_grid`, default true); it is a benign no-op
until that topic is published. Setting `enable_detection_grid:=true` instead
feeds the occupancy pathway from the local `detected_objects_to_grid` rasterizer
(useful when the probabilistic OGM under-marks tracked obstacles on the path).

The package-local `launch/trajectory_switch.launch.xml` is a **standalone
launcher** for ad-hoc testing only; `simple_object_predictor` and
`trajectory_switch` are optional utilities not used by the default FMS chain.
The component file above is the source of truth for production wiring.

See `doc/` for per-node detail.

## Build & test

```bash
colcon build --symlink-install --packages-select igv_velocity_planner
colcon test --packages-select igv_velocity_planner
```
