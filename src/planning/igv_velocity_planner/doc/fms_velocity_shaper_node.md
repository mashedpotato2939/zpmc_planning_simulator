# fms_velocity_shaper_node

Single owner of obstacle-driven velocity in FMS mode. Converts the FMS-shaped
`Path` into a `Trajectory` using a **constraint-based pipeline**, then hands the
result to `igv_kinematic_smoother` / `velocity_smoother` for kinematic
feasibility shaping.

It is the next step beyond the Phase-3 end state of
[`velocity_planner_simplification_plan.md`](../../../../docs/vehicle_agent/velocity_planner_simplification_plan.md):
the former `if + min` velocity clipper has been replaced by the architecture in
[`velocity_planner_refactor_proposal.md`](velocity_planner_refactor_proposal.md).

## Architecture

Each pathway is a pure **constraint generator** that, given one immutable tick
snapshot (`Path` + objects + occupancy grid + odometry), emits zero or more
segment `VelocityConstraint`s — it never mutates the trajectory. A central
`ConstraintMerger` resolves all constraints onto the arc-length axis and writes
the raw velocity profile.

```
   snapshot (path + objects + occupancy_grid + odometry)
        │
   ┌────┴───────────────── generators ──────────────────┐
   │ StaticObstacleStop → {s_stop, ∞, 0} (footprint-aware)│
   │ DynamicStop      → {s_stop, ∞, 0}  (per-UUID hyst.) │
   │ OccupancyStop    → hard stop + optional slow-down   │
   │ SlowDown         → {window, ramp}  (per-UUID hyst.) │
   │ AdaptiveCruise   → {0, ∞, leader cap}               │
   └────┬───────────────────────────────────────────────┘
        │  vector<VelocityConstraint>
        ▼
   ConstraintMerger        seed v_limit[i] = FMS path velocity;
        │                  resolve by priority (higher records provenance,
        │                  but only ever LOWERS velocity — monotonic guard,
        │                  so the result equals a pure min());
        │                  insert exact stop points (back-to-front).
        ▼
   /planning/igv_velocity_planner/trajectory  →  igv_kinematic_smoother
```

### Merge semantics
Priorities (`Priority` enum): `SlowDown 20 < AdaptiveCruise 50 <
StaticObstacleStop 70 < DynamicStop 80 < OccupancyStop 90 < Aeb 100 (reserved)`.
The merger records
the highest-priority *binding* constraint as the per-point `winner`/`source` (so
diagnostics can answer "who stopped the vehicle"), while the numeric profile is
a pure `min` over all constraints — identical to the legacy clipper for the
limit-only generators, but with traceable provenance and a single resolution
site. Stop points (`v=0`, `end_s=∞`) are inserted once at exact arc-length,
back-to-front, so insertions never invalidate not-yet-processed indices.

## Code layout

`namespace igv_velocity_planner::velocity_shaper`, one concern per file under
[`include/.../velocity_shaper/`](../include/igv_velocity_planner/velocity_shaper/)
and [`src/velocity_shaper/`](../src/velocity_shaper/):

| File | Concern |
|---|---|
| `constraint.hpp` | `VelocityConstraint`, `ConstraintType`, `Priority`, `priorityOf` |
| `constraint_generator.hpp` | `IConstraintGenerator`, `TickInput` |
| `constraint_merger.{hpp,cpp}` | `mergeConstraints` + `MergeResult` (priority/min resolution, central stop insertion) |
| `generators.{hpp,cpp}` | `StaticObstacleStopGenerator` (footprint-aware hard stop), `DynamicStopGenerator`, `SlowDownGenerator`, `AdaptiveCruiseGenerator` (stateful ones own per-UUID hysteresis) |
| `distance_map.{hpp,cpp}` | ROI Euclidean distance transform (Felzenszwalb two-pass) over the occupancy grid |
| `occupancy_stop.{hpp,cpp}` | `OccupancyStopGenerator` — distance-map + corridor hard stop / optional graded slow-down |
| `static_obstacle_stop.{hpp,cpp}` | `computeStaticObstacleStop` (footprint-aware static hard stop) |
| `dynamic_obstacle_stop.{hpp,cpp}` | `computeDynamicObstacleStop` (predicted-collision stop) |
| `adaptive_cruise.{hpp,cpp}` | `computeAdaptiveCruiseVelocityCap` (single-leader ACC) |
| `slow_down.{hpp,cpp}` | `computeSlowDownWindow` (object outer-band slow-down) |
| `geometry_utils.{hpp,cpp}` | Shared math: `projectOntoPath`, `arcLengthTo`, `indexBeforeArclength`, `insertStopAtArcLength`, `uuidToString`, `convertPathToTrajectoryPoints` |
| `params.hpp` | `Params` — aggregate parameter bundle (one nested struct per pathway) |

`velocity_shaper_lib` (SHARED) builds these; the node target and the gtest
targets link against it.

## Free-space occupancy (`/perception/occupancy_grid_map/map`)

The occupancy pathway consumes the perception occupancy grid **directly**
(`nav_msgs/OccupancyGrid`, cells `{-1 unknown, 0 free, 100 occupied}`; unknown is
treated as free). The node TF-transforms the trajectory points into the grid
frame and hands them to `OccupancyStopGenerator`, which:

1. builds a Euclidean distance map over the grid (cached per grid stamp);
2. samples a corridor (centerline ± ego half-width) against it;
3. emits a hard `OCCUPANCY_STOP` (`v=0`) at the first sample whose
   distance-to-nearest-obstacle drops below `ego_half_width + required_clearance`,
   backed off by `stop_distance_buffer`; and, when
   `occupancy_grid_check.enable_slow_down=true`, emits a graded `SlowDown` over
   the `slow_band` of distances just above that threshold.

If `use_occupancy_grid=false`, or no grid has been received, or the TF lookup
fails, the pathway is a benign no-op (the real vehicle may not publish the topic
yet). As an **off-by-default fallback**, `enable_detection_grid:=true` routes
`~/input/occupancy_grid` to the local `detected_objects_to_grid` rasterizer
instead of `/perception/occupancy_grid_map/map` — useful when the probabilistic
OGM under-marks tracked obstacles on the path. All point-cloud pathways have been
removed.

## Parameters

Tuning lives in
[`config/igv_velocity_shaper.param.yaml`](../config/igv_velocity_shaper.param.yaml).
Each generator reads its own nested `Params` struct; the `enable` flags and
`perception_inputs.use_occupancy_grid` are runtime-settable via `ros2 param set`
(they take effect on the next `onPath` cycle). Key groups:
`dynamic_obstacle_stop`, `static_obstacle_stop`, `adaptive_cruise`, `slow_down`,
`trajectory_preprocessing`, `occupancy_grid_check`.

`trajectory_preprocessing.resample_interval` densifies the incoming path into
trajectory points before any constraint generator samples it.

## Topics

| Direction | Topic (`~/...` remapped in the FMS component launch) | Type |
|---|---|---|
| in | `~/input/path` ← `/planning/igv_path_planner/path` | `autoware_planning_msgs/Path` |
| in | `~/input/objects` ← `/perception/object_recognition/objects` | `autoware_perception_msgs/PredictedObjects` |
| in | `~/input/odometry` ← `/localization/kinematic_state` | `nav_msgs/Odometry` |
| in | `~/input/occupancy_grid` ← `/perception/occupancy_grid_map/map` | `nav_msgs/OccupancyGrid` |
| out | `~/output/trajectory` → `/planning/igv_velocity_planner/trajectory` | `autoware_planning_msgs/Trajectory` |
| out (diag) | `~/output/velocity_factors`, `~/output/stop_reasons`, `~/output/stop_markers` | factor / stop-reason / marker arrays |
| out (debug) | `~/debug/occupancy_corridor` → `/planning/igv_velocity_planner/debug/occupancy_corridor` | `visualization_msgs/MarkerArray` |
| out (debug) | `~/debug/occupancy_distance_map` → `/planning/igv_velocity_planner/debug/occupancy_distance_map` | `nav_msgs/OccupancyGrid` |

## Diagnostics

Emitted once per cycle from the `MergeResult` provenance (one entry per distinct
winning `source`, matching the legacy per-pathway cardinality):

| `VelocityFactor.sequence` | behavior | when |
|---|---|---|
| `fms_velocity_shaper.adaptive_cruise` | follow | a leader cap is the binding limit |
| `fms_velocity_shaper.slow_down` | slow_down | an object outer-band window is binding |
| `fms_velocity_shaper.occupancy_slow_down` | slow_down | the enabled graded occupancy band is binding |
| `fms_velocity_shaper.occupancy_grid` | stop | a free-space corridor intrusion stop |
| `fms_velocity_shaper.static_obstacle_stop` | stop | a static obstacle footprint intrusion stop |
| `fms_velocity_shaper.dynamic_obstacle_stop` | stop | a predicted-collision stop |

`StopReason` (`OBSTACLE_STOP`) is published for occupancy, static-obstacle, and
dynamic-obstacle hard stops. The specific reason is carried in
`VelocityFactor.detail` and in the `~/output/stop_markers` text. RViz stop
markers lead each cycle with `DELETEALL` so stale stops clear, then publish a
red semi-transparent stop wall, red stop sphere, and text label with `STOP`, the
specific reason, stop distance, and obstacle gap when an anchor point exists.
Anchored stops also include a yellow connector from the inserted stop pose to
the obstacle/cell anchor.
The occupancy debug topics use reliable transient-local QoS so RViz can replay
the latest payload when the displays are toggled on. On each path cycle with a
valid occupancy grid, the node builds or reuses the cached EDT for the current
grid and publishes it as a grayscale `OccupancyGrid`; this debug publication no
longer depends on a prior occupancy constraint being generated or on an active
subscriber. The corridor marker additionally requires a grid-frame path and
shows the sampled centerline and ego-half-width boundaries in the occupancy-grid
frame, plus `TEXT_VIEW_FACING` labels for the closest sampled distance and
sparse in-band distances in metres.

## Tests

`colcon test --packages-select igv_velocity_planner`. Unit suites (gtest,
linking `velocity_shaper_lib`): `constraint_merger_test` (priority/min, monotonic
guard, multi-stop sparse-path insertion, the worked example),
`occupancy_distance_map_test` (EDT geometry, corridor hard stop, graded
slow-down, benign no-op), `generators_test` (each generator's emitted
constraints + hysteresis), and `fms_velocity_shaper_test` (the preserved
static / dynamic / ACC / slow-down / projection pathway tests).
