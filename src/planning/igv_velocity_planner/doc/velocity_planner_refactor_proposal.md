# igv_velocity_planner — Constraint-Based Refactor Proposal

**Status:** proposal / for review
**Branch:** `feature/velocity_planner_refactor`
**Scope:** the `igv_velocity_shaper` node and its `velocity_shaper` library inside
`src/igv/igv_velocity_planner`. The `igv_kinematic_smoother` stage and the node/topic
boundaries (`/planning/igv_velocity_planner/trajectory` out) are **kept stable**.
**Continues:** `docs/vehicle_agent/velocity_planner_simplification_plan.md` (which
reduced the stock Autoware velocity stack to this single shaper + smoother). This
document is the next step: re-architecting the shaper internals.

---

## 1. Goal

Three changes, in priority order:

1. **Replace the `if + min` velocity clipper** in `FmsVelocityShaperNode` with a
   mature, industry-standard **constraint-based** architecture
   (segment `VelocityConstraint`s → merge → raw velocity profile → smoother).
   This is the architecture recommended in `velocity_planner_optimizer.TXT`.
2. **Source free-space occupancy directly from `/perception/occupancy_grid_map/map`**
   (`nav_msgs/OccupancyGrid`) using a **distance-map + corridor** check, and
   **delete the `detected_objects_to_grid` conversion node** that currently
   manufactures a grid from detected objects.
3. **Remove all point-cloud pathways** (`pointcloud_limiter`,
   `slow_down_pointcloud`, the `~/input/no_ground_pointcloud` subscription and its
   TF handling). Occupancy + objects are the only perception inputs going forward.

Non-goals: changing the `igv_kinematic_smoother`, changing the FMS→Path stage
(`igv_path_planner`), or changing what `motion_control` downstream consumes.

---

## 2. Current architecture (what we are replacing)

`FmsVelocityShaperNode::shapeTrajectory()`
(`src/fms_velocity_shaper_node.cpp:277`) converts the incoming `Path` to a
`std::vector<TrajectoryPoint>` and then runs the raw velocity-shaping pathways
in sequence, each
**directly mutating** the per-point velocity:

| # | Pathway (lib file) | Input | Mutation primitive |
|---|---|---|---|
| 1b | `pointcloud_limiter` | `no_ground_pointcloud` | `capVelocityFrom` |
| 1c | `slow_down_pointcloud` | `no_ground_pointcloud` | `capVelocityRange` |
| 2 | `dynamic_obstacle_stop` | objects (predicted paths) | `insertStopAtArcLength` + `capVelocityFrom(…, 0)` |
| 2b | `occupancy_grid_check` | grid (from detection rasterizer) | `capVelocityFrom(…, 0)` |
| 2c | `slow_down` | objects (outer band) | `capVelocityRange` |
| 3 | `adaptive_cruise` | objects + odometry | `capVelocityFrom(0, v_cap)` |

This is a **scalar velocity clipper**: the final velocity is
`min(fms, acc, slow_down, dynamic_stop, occupancy_stop, …)` applied imperatively,
pathway by pathway. Per `velocity_planner_optimizer.TXT`, in the single-frame,
no-conflict case this is mathematically equivalent to a clean constraint merge —
but in a real IGV it degrades along five axes:

1. **Spatial expressiveness.** `capVelocityFrom`/`capVelocityRange` bake the
   *segment* (`[start_s, end_s)`) into imperative loops. The "what" (a velocity
   limit over an arc-length interval) and the "how" (the mutation) are fused, so
   the segment is never a first-class, inspectable object.
2. **Ordering coupling.** The pathways run in a fixed code order. `min` makes the
   *result* order-independent, but intermediate state, diagnostics, and which
   pathway "won" all depend on write order. Re-ordering the `if` blocks silently
   changes debug behavior.
3. **Time consistency.** Each pathway reads its own cached input
   (`objects_`, `pointcloud_`, `occupancy_grid_`) latched at different callback
   times. There is no single "tick" snapshot.
4. **Explainability / traceability.** After the fact, `v = 0` at some `s` carries
   no record of *which* pathway set it. Industrial deployments need
   "who stopped the vehicle, and where" for safety audit and field debugging.
   Today this is reconstructed heuristically from the side-channel
   `VelocityFactor`/`StopReason` messages.
5. **Non-monotonic behavior.** `if + min` assumes every pathway only ever *lowers*
   velocity. Recovery (obstacle clears → speed may rise), gap-based behavior, and
   curve limits do not fit the "stick at the running minimum" model, which is why
   hysteresis is bolted on per-pathway today (`stop_decisions_`,
   `slow_down_decisions_`).

---

## 3. Target architecture

```
                 single tick snapshot (path + objects + grid + odom)
                                   │
        ┌──────────────────────────┴───────────────────────────┐
        │                  Constraint Generators                │
        │   ┌────────────────┐  emit 0..N VelocityConstraint    │
        │   │ StaticObstacle │──┐                               │
        │   │ DynamicStop    │──┤   each = {start_s, end_s,     │
        │   │ OccupancyStop  │──┤          target_v, type,      │
        │   │ SlowDown       │──┤          priority, source}    │
        │   │ AdaptiveCruise │──┘                               │
        │   └────────────────┘                                  │
        └──────────────────────────┬───────────────────────────┘
                                    │  std::vector<VelocityConstraint>
                                    ▼
                          ┌──────────────────┐
                          │ ConstraintMerger │  project onto s-axis,
                          │                  │  resolve (priority + min)
                          └────────┬─────────┘
                                   │  raw velocity profile v_limit[i]  (+ winning source per i)
                                   ▼
                       /planning/igv_velocity_planner/trajectory
                                   │
                                   ▼
                         igv_kinematic_smoother      ← already exists; this is the
                       (jerk/accel/lat-acc feasible)   notes' "Velocity Smoother"
                                   │
                                   ▼
                       stock velocity_smoother → planning_validator → motion_control
```

The shaper node becomes a thin orchestrator:

1. **Snapshot.** On `onPath`, latch the current `objects_`, `occupancy_grid_`,
   `odometry_` into one immutable `TickInput`. Every generator reads the same tick.
2. **Generate.** Run each enabled generator; collect all `VelocityConstraint`s into
   one list. Generators are **pure and stateless** w.r.t. the trajectory — they
   *describe* limits, they never write velocities.
3. **Merge.** `ConstraintMerger` rasterizes the constraints onto the arc-length
   samples of the path and produces `v_limit[i]` plus, for each `i`, the
   `source` that won. This is the single place where resolution policy lives.
4. **Emit.** Write `v_limit[i]` into the outgoing `Trajectory`; publish the
   `VelocityFactor`/`StopReason`/marker diagnostics *from the merge result* (so the
   reported reason is exactly the one that shaped the velocity — no reconstruction).

The downstream `igv_kinematic_smoother` is unchanged: it already performs the
forward/backward jerk-, accel-, and lateral-accel-limited pass that the notes call
the "Velocity Smoother." We are only changing how the *raw* `v_limit` profile that
feeds it is produced.

### 3.1 Core data structures

```cpp
namespace igv_velocity_planner::velocity_shaper {

enum class ConstraintType {
  kFmsBase,          // the path's own per-point speed (baseline, lowest priority)
  kAdaptiveCruise,   // follow-leader cap
  kSlowDown,         // outer-band localized dip
  kStaticObstacleStop, // static obstacle hard stop
  kDynamicStop,      // predicted-collision stop
  kOccupancyStop,    // free-space corridor intrusion stop
  // future: kTrafficLight, kStation, kManualOverride, kSpeedZone, kAeb
};

// Priorities decide *resolution policy*, not just ordering. Higher = wins by
// overwrite; equal/lower compose by min. (Mirrors the notes' Priority enum.)
enum Priority : int {
  kPrioFmsBase        = 0,
  kPrioSlowDown       = 20,
  kPrioAdaptiveCruise = 50,
  kPrioDynamicStop    = 80,
  kPrioOccupancyStop  = 90,
  kPrioAeb            = 100,   // reserved
};

// A speed limit over an arc-length interval. This is the single vocabulary every
// generator speaks. end_s == +inf means "to the end of the trajectory".
struct VelocityConstraint {
  double        start_s{0.0};
  double        end_s{std::numeric_limits<double>::infinity()};
  double        target_velocity{0.0};   // [m/s] cap applied inside [start_s, end_s)
  ConstraintType type{ConstraintType::kFmsBase};
  int           priority{kPrioFmsBase};
  std::string   source;                 // human-readable origin, e.g. "obj:<uuid>"
  // Optional diagnostic anchor (the obstacle/cell that caused this constraint).
  std::optional<geometry_msgs::msg::Point> anchor;
};
```

> The notes also sketch a richer `VelocityZone` carrying `max_acceleration` /
> `max_deceleration`. We deliberately **defer** that: the `igv_kinematic_smoother`
> already owns global accel/jerk limits, so per-zone dynamic limits add coupling
> with no current consumer. `VelocityConstraint` is the Stage-3 "industrial
> standard" target in the notes; `VelocityZone` is a later, optional extension and
> is called out in §9.

### 3.2 Constraint generator interface

```cpp
// Pure function family. Each takes the tick snapshot and appends constraints.
// No trajectory mutation, no hysteresis side effects on the points.
struct TickInput {
  const std::vector<TrajectoryPoint> & points;   // path → trajectory points
  const PredictedObjects *             objects;   // nullable
  const nav_msgs::msg::OccupancyGrid * grid;      // nullable, in its own frame
  const Odometry *                     odometry;  // nullable
  double ego_front_clearance;
  double ego_half_width;
  rclcpp::Time stamp;
};

class IConstraintGenerator {
public:
  virtual ~IConstraintGenerator() = default;
  virtual const char * name() const = 0;
  virtual bool enabled() const = 0;
  // Append zero or more constraints derived from `in`.
  virtual void generate(const TickInput & in, std::vector<VelocityConstraint> & out) = 0;
};
```

Generators that need temporal hysteresis (dynamic stop, slow-down) keep their
per-UUID state *inside the generator object* (where `stop_decisions_` /
`slow_down_decisions_` live today), but they emit/withhold a **constraint** rather
than mutating the path. This isolates the only stateful logic and keeps the merge
deterministic.

### 3.3 Constraint merge

```cpp
struct MergeResult {
  std::vector<double>         v_limit;   // one per trajectory point
  std::vector<ConstraintType> winner;    // which type set each v_limit[i]
  std::vector<std::string>    source;    // which concrete source won at i
};

MergeResult mergeConstraints(
  const std::vector<TrajectoryPoint> & points,
  const std::vector<VelocityConstraint> & constraints);
```

Algorithm (the notes' "project onto one S-axis, take the most conservative"):

```
for i in points:                       v_limit[i] = points[i].v   (FMS base)
for c in constraints sorted by priority asc:
    for i where arc_s(i) ∈ [c.start_s, c.end_s):
        if c.priority > winner_priority[i]:        # higher priority overwrites
            v_limit[i] = c.target_velocity; record winner
        elif c.priority == winner_priority[i]:     # same priority composes by min
            if c.target_velocity < v_limit[i]: v_limit[i] = c.target_velocity; record
        else:                                       # lower priority still clips
            if c.target_velocity < v_limit[i]: v_limit[i] = c.target_velocity; record
```

The "overwrite if higher priority" branch is what lets a future **AEB** or an
**occupancy stop** force `0` even if a lower-priority generator wanted to *recover*
speed — the exact non-monotonic case `if + min` cannot express. For all the
generators we have today (all of which only lower speed), the result is identical
to the current `min`, so **this refactor is behavior-preserving by construction**
for the existing pathways. The win is structural: traceable source per point,
single resolution site, trivial extensibility.

Stop insertion (today's `insertStopAtArcLength`) is handled once, centrally: after
merge, wherever `v_limit` first reaches `0` on a coarse segment, the merger inserts
an exact-arc-length stop point. No generator does its own point insertion.

---

## 4. Occupancy from `/perception/occupancy_grid_map/map` (distance-map + corridor)

### 4.1 What changes at the boundary

| | Today | After |
|---|---|---|
| Grid producer | `detected_objects_to_grid` node (local, manufactured from objects) | `autoware_probabilistic_occupancy_grid_map` (real) / sim publisher — **already runs** |
| Topic into shaper | `~/input/occupancy_grid` ← `/planning/igv_velocity_planner/detections_occupancy_grid` | `~/input/occupancy_grid` ← `/perception/occupancy_grid_map/map` |
| Gate | `perception_inputs.use_occupancy_grid` = `enable_detection_grid` | `perception_inputs.use_occupancy_grid` (own arg) |
| Semantics | first trajectory *point* in an occupied cell → stop | first **corridor sample** whose **distance-to-nearest-obstacle < required_clearance** → stop |

`/perception/occupancy_grid_map/map` is `nav_msgs/OccupancyGrid` in the `map`
frame, cells `{-1 unknown, 0 free, 100 occupied}` (confirmed:
`launch/scripts/publish_occupancy_grid.py`, and
`tier4_perception_launch/.../probabilistic_occupancy_grid_map.launch.xml`
`output:=/perception/occupancy_grid_map/map`). The shaper already TF-transforms
trajectory points into the grid frame (`occupancy_grid_check.cpp`), so no new TF
wiring is needed.

### 4.2 Why a distance map instead of point-in-cell

The current check (`applyOccupancyGridStopCheck`) samples the **path centerline
only** and asks the binary question "is this one cell occupied?". The notes argue —
correctly — for upgrading to a **distance field** (Euclidean Distance Transform,
EDT) queried over a **corridor**:

- A distance field `d(x,y)` = metric distance to the nearest occupied cell answers
  a *continuous* question, enabling graded response (slow as `d` shrinks, stop when
  `d < clearance`) instead of binary stop.
- Sampling a **corridor** (the path inflated to `ego_half_width + margin`) instead
  of the centerline naturally accounts for **vehicle width, localization error,
  control error, grid resolution, and trajectory sampling gaps** — the failure
  modes the centerline check misses (a cell adjacent to, but not under, a sample).

### 4.3 Algorithm

```
OccupancyStopGenerator(grid, points):
    # 1. Bound work: ROI = bbox of the path corridor clipped to max look-ahead.
    roi = inflate(path_bbox, ego_half_width + clearance) ∩ grid, s ≤ max_distance
    # 2. EDT over the ROI only (Felzenszwalb 2-pass, O(cells), few ms on a 200×200 ROI).
    dist = distanceTransform(grid ≥ occupancy_threshold, roi)   # metres
    # 3. Walk the trajectory; query d at each sample (optionally a few lateral
    #    offsets across the corridor half-width).
    required = ego_half_width + clearance_margin
    for i, p in points (s(i) ≤ max_distance):
        d_min = min over corridor offsets of dist.query(p ⊕ offset)
        if d_min < required:                       # intrusion
            emit { start_s = max(0, s(i) - stop_buffer), end_s = inf,
                   target_v = 0, type = kOccupancyStop, prio = kPrioOccupancyStop,
                   source = "occupancy@(cx,cy)" }
            break
        elif enable_slow_down and d_min < required + slow_band:
            emit { [s(i), s(i)+window], v = ramp(d_min), type = kSlowDown, … }
```

First cut may skip the lateral corridor offsets and the graded slow-down (emit only
the hard stop) to land the topic switch quickly, then add them. Compute is bounded
by the ROI; the EDT is cached per grid message (rebuilt only when a new grid
arrives), so per-`onPath` cost is just the corridor query.

### 4.4 New module

- `velocity_shaper/distance_map.{hpp,cpp}` — ROI EDT builder + bilinear query, in
  grid-local coordinates. Pure, unit-testable, no ROS types beyond `OccupancyGrid`.
- `velocity_shaper/occupancy_stop.{hpp,cpp}` — generator wrapping the distance map;
  replaces `occupancy_grid_check.{hpp,cpp}`.

---

## 5. Point-cloud removal

Delete entirely — no replacement:

**Source / headers**
- `src/velocity_shaper/pointcloud_limiter.cpp`, `include/.../pointcloud_limiter.hpp`
- `applyPointCloudSlowDownLimiter` + `PointCloudSlowDownResult` in `slow_down.{hpp,cpp}`

**Node (`fms_velocity_shaper_node.*`)**
- `onPointCloud`, `pointcloud_sub_`, `pointcloud_` cache, the `PointCloud2` using-alias
- Pathways **1b** and **1c** (the entire `use_no_ground_pointcloud` block,
  `fms_velocity_shaper_node.cpp:307-379`), including its TF lookup and
  `sensor_msgs/point_cloud2_iterator` include
- `params_.pointcloud_limiter`, `params_.slow_down_pointcloud`, and
  `perception_inputs.use_no_ground_pointcloud`

**Params / config**
- `pointcloud_limiter:` and `slow_down_pointcloud:` blocks and
  `perception_inputs.use_no_ground_pointcloud` in
  `config/igv_velocity_shaper.param.yaml`
- The `PointCloudLimiter` and `SlowDownPointCloud` structs in `params.hpp`

**CMake / package.xml**
- Drop `pointcloud_limiter.cpp` from `velocity_shaper_lib`
- `sensor_msgs` dep can be dropped if nothing else uses it (verify first)

**Launch**
- Remove `<remap from="~/input/no_ground_pointcloud" to="/perception/obstacle_segmentation/pointcloud"/>`
  in `igv_launch/launch/components/tier4_planning_component_fms.launch.xml`

**Tests**
- Remove `PointCloud*` cases from `test/fms_velocity_shaper_test.cpp`
  (`PointCloudInBandSlowsTrajectory`, `PointCloudSlowDown*`, etc.)

---

## 6. Detection-grid (`detected_objects_to_grid`) removal

The occupancy grid now comes from perception directly, so the entire rasterizer is
dead weight:

**Delete**
- `src/detected_objects_grid.cpp`, `src/detected_objects_grid_node.cpp`
- `include/.../detected_objects_grid.hpp`, `include/.../detected_objects_grid_node.hpp`
- `config/detected_objects_grid.param.yaml`
- `launch/detected_objects_grid.launch.xml`
- `test/detected_objects_grid_test.cpp`
- CMake: `detected_objects_grid_lib`, `detected_objects_grid_node`,
  `detected_objects_to_grid` executable registration, and the
  `detected_objects_grid_test` block

**Launch (`tier4_planning_component_fms.launch.xml`)**
- Remove the `<group if="$(var enable_detection_grid)">` node block (lines ~131-141)
- Repoint the shaper input:
  `~/input/occupancy_grid` → `/perception/occupancy_grid_map/map`
- Replace `perception_inputs.use_occupancy_grid = $(var enable_detection_grid)`
  with a dedicated `use_occupancy_grid` arg (default `true`)
- In `igv_launch/launch/igv_planning.launch.xml`, retire `enable_detection_grid`
  (or repurpose it to `use_occupancy_grid`)

> **Verify before deleting:** confirm no other consumer subscribes to
> `/planning/igv_velocity_planner/detections_occupancy_grid` (grep shows only the
> shaper today). The `simple_object_predictor` and `trajectory_switch` utilities are
> independent of the detection grid and are unaffected.

---

## 7. Per-pathway migration map

| Current pathway | Becomes generator | `ConstraintType` | Notes |
|---|---|---|---|
| (Path per-point v) | baseline in merger | `kFmsBase` | seeded as `v_limit[i] = path.v` |
| `dynamic_obstacle_stop` (+hysteresis) | `DynamicStopGenerator` | `kDynamicStop` | keeps per-UUID `stop_decisions_` *inside* the generator; emits `[s_stop, inf)` v=0 when committed |
| `static_obstacle_stop` (+hysteresis) | `StaticObstacleStopGenerator` | `kStaticObstacleStop` | emits `[s_stop, inf)` v=0 when a near-static footprint intrudes the ego corridor |
| `occupancy_grid_check` | `OccupancyStopGenerator` (new, §4) | `kOccupancyStop` | distance-map + corridor; consumes `/perception/occupancy_grid_map/map` |
| `slow_down` (+hysteresis) | `SlowDownGenerator` | `kSlowDown` | emits `[s_start, s_end)` windowed cap; keeps `slow_down_decisions_` |
| `adaptive_cruise` | `AdaptiveCruiseGenerator` | `kAdaptiveCruise` | emits `[0, inf)` cap = leader-derived v |
| `pointcloud_limiter` / `slow_down_pointcloud` | **deleted** | — | §5 |

Diagnostics (`VelocityFactorArray`, `StopReasonArray`, `MarkerArray`) move to a
single post-merge emitter driven by `MergeResult.winner/source`, replacing the
per-pathway emission scattered through `shapeTrajectory()`.

---

## 8. Proposed file layout

```
include/igv_velocity_planner/velocity_shaper/
  constraint.hpp              # NEW  VelocityConstraint, ConstraintType, Priority
  constraint_merger.hpp       # NEW  mergeConstraints + MergeResult
  constraint_generator.hpp    # NEW  IConstraintGenerator, TickInput
  distance_map.hpp            # NEW  ROI EDT + query
  occupancy_stop.hpp          # NEW  replaces occupancy_grid_check.hpp
  static_obstacle_stop.hpp    # footprint-aware static hard stop
  dynamic_stop.hpp            # was dynamic_obstacle_stop.hpp
  slow_down.hpp               # trimmed: drop pointcloud slow-down
  adaptive_cruise.hpp         # unchanged signature, wrapped as generator
  geometry_utils.hpp          # unchanged (shared math)
  params.hpp                  # drop PointCloud* structs; add merge/occupancy params
  # DELETED: pointcloud_limiter.hpp, occupancy_grid_check.hpp
src/velocity_shaper/   … mirrors the above
src/fms_velocity_shaper_node.cpp  # thin orchestrator: snapshot → generate → merge → emit
# DELETED: detected_objects_grid*.{hpp,cpp}, pointcloud_limiter.cpp
```

`geometry_utils` keeps `capVelocityFrom`/`capVelocityRange`/`insertStopAtArcLength`
only if still useful inside the merger; otherwise they fold into
`constraint_merger.cpp`.

---

## 9. Phased migration (each phase builds + keeps tests green)

**Phase 0 — Introduce the vocabulary (no behavior change).**
Add `constraint.hpp`, `constraint_merger.{hpp,cpp}`, `constraint_generator.hpp`.
Unit-test the merger against hand-built constraint lists (incl. the notes'
worked example: FMS 8, ACC 4, slow-down 3 over [55,65], dynamic stop @32,
occupancy stop @17 → expected profile). Nothing wired yet.

**Phase 1 — Port generators behind a flag.**
Wrap each existing pathway as an `IConstraintGenerator` that produces constraints
instead of mutating points. Add a `use_constraint_pipeline` param: when true, run
generators → merge → write profile; when false, keep the legacy `if+min` path.
Add a parity test asserting identical `v_limit` for representative scenes. Flip the
default once parity holds, then delete the legacy path.

**Phase 2 — Occupancy source switch + distance map.**
Add `distance_map` + `OccupancyStopGenerator`; repoint the launch remap to
`/perception/occupancy_grid_map/map`; delete `occupancy_grid_check`. Validate in
sim with the `publish_occupancy_grid.py` empty grid (no false stops) and with a
synthetic occupied block (stop lands `stop_buffer` ahead of intrusion).

**Phase 3 — Delete point-cloud + detection grid.**
Remove the files/params/launch/tests in §5 and §6. Drop now-unused deps.

**Phase 4 — Diagnostics + docs.**
Single post-merge diagnostic emitter from `MergeResult`. Update `README.md`,
`doc/fms_velocity_shaper_node.md`, and CLAUDE.md's FMS-chain section. Optionally
prototype `VelocityZone` (per-constraint accel/decel) if a consumer appears.

---

## 10. Testing strategy

- **Merger unit tests** (new): priority overwrite vs. min, segment projection on
  sparse paths, central stop insertion, the notes' worked example.
- **Distance-map unit tests** (new): known-geometry EDT values, corridor `d_min`,
  intrusion → stop arc-length, empty/unknown-cell grids are benign.
- **Generator parity tests**: each ported generator reproduces the legacy
  pathway's `v_limit` on its existing scenarios (reuse the current
  `fms_velocity_shaper_test.cpp` cases for static, dynamic, ACC, slow-down).
- **Preserved behaviors** (must stay green, possibly relocated): static-obstacle
  ego-half-width inclusion, dynamic-stop hysteresis + sparse-path stop placement,
  ACC leader gating, occupancy stop with buffer.
- **Removed**: all `PointCloud*` and `detected_objects_grid` tests.

---

## 11. Risks & mitigations

| Risk | Mitigation |
|---|---|
| `/perception/occupancy_grid_map/map` not populated on the real vehicle yet | Gate with `use_occupancy_grid` (default off until perception confirms publish); generator is a no-op when grid is null/empty — same tolerance as today |
| Full-grid EDT too costly (500×500 @ high rate) | ROI-bounded EDT + per-message caching; query only the corridor; cap by `max_distance` |
| Behavior drift during the if+min → constraint swap | Phase 1 parity flag + parity tests before deleting legacy path |
| Probabilistic grid is noisier than the manufactured detection grid (false stops) | `occupancy_threshold` tuning + required-clearance margin + optional temporal/area filtering; graded slow-down softens single-cell flicker |
| Loss of the close-range, identity-free stop the detection grid provided | The perception OGM already encodes close-range returns; verify coverage in sim/field before removing the rasterizer in Phase 3 |

---

## 12. Summary of changes by file

**Add**
`velocity_shaper/constraint.hpp`, `constraint_merger.{hpp,cpp}`,
`constraint_generator.hpp`, `distance_map.{hpp,cpp}`, `occupancy_stop.{hpp,cpp}`;
merger + distance-map tests.

**Modify**
`fms_velocity_shaper_node.{hpp,cpp}` (orchestrator), `params.hpp` +
`igv_velocity_shaper.param.yaml` (drop pointcloud, add merge/occupancy),
`dynamic_obstacle_stop`/`static_obstacle_stop`/`slow_down`/`adaptive_cruise`
(generator-shaped), `CMakeLists.txt`, `package.xml`, `README.md`,
`doc/fms_velocity_shaper_node.md`,
`igv_launch/launch/components/tier4_planning_component_fms.launch.xml`,
`igv_launch/launch/igv_planning.launch.xml`.

**Delete**
`detected_objects_grid*.{hpp,cpp}`, `pointcloud_limiter.{hpp,cpp}`,
`occupancy_grid_check.{hpp,cpp}`, `config/detected_objects_grid.param.yaml`,
`launch/detected_objects_grid.launch.xml`, `test/detected_objects_grid_test.cpp`,
pointcloud pieces of `slow_down.{hpp,cpp}` and `fms_velocity_shaper_test.cpp`.

**Unchanged**
`igv_kinematic_smoother` (the downstream "Velocity Smoother"),
`simple_object_predictor`, `trajectory_switch`, and the output boundary
`/planning/igv_velocity_planner/trajectory`.

---

## 13. Open decisions for the team

1. **Graded occupancy response** in v1, or hard stop only first? (Recommend: ship
   hard stop, add graded slow-down in a follow-up.)
2. **`VelocityZone`** (per-constraint accel/decel) — adopt now or defer? (Recommend:
   defer until a consumer needs it; smoother owns global limits today.)
3. **Keep the detection grid as a fallback** behind a flag during field validation,
   or delete outright in Phase 3? (Recommend: keep one release as an off-by-default
   fallback, then delete.)
4. **`enable_detection_grid` arg** — rename to `use_occupancy_grid` or keep the old
   name for launch-script compatibility?
```
