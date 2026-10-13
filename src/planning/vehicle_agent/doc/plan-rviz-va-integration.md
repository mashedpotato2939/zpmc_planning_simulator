# Plan — IR↔FMS bridge + RViz "Publish via VA" toggle

Follow-up to Sprint 6. Closes the cloud→VA→converter loop and gives the
RViz panel an end-to-end test path that exercises VA without disturbing
the existing manual-injection workflow.

## 1. Motivation

Today:

```
RViz panel  ──FMSRouteWindow──▶ /fms/route_window  ──▶ igv_path_converter ──▶ planning
Cloud FMS   ──▶ VA adapter  ──ir::Mission──▶ /vehicle_agent/mission   ⟂  (dead end)
```

VA's downlink path is exercised by tests but never feeds the planner.
The plugin's injection path is exercised by humans but never touches VA.
This plan adds:

```
Cloud FMS / RViz inject  ──▶ VA  ──ir::Mission──▶ /vehicle_agent/mission
                                                  │
                                                  ▼
                              va_route_bridge ──FMSRouteWindow──▶ igv_path_converter
```

Two artefacts, designed so they can ship in either order. Most value is
unlocked by (1) alone; (2) just makes the operator panel a real testbed.

---

## 2. Artefact 1 — `va_route_bridge` node

A small, single-process ROS node that subscribes to VA's IR topics and
republishes onto the topics `igv_path_converter` already consumes. It
also owns the ego-centric **route windowing** that keeps the
converter's cached path alive between FMS pushes — see §2.8. Lives in
the `vehicle_agent` package so it ships alongside VA.

### 2.1 Subscribed → Published

| Sub | Type | Pub | Type | Notes |
|---|---|---|---|---|
| `/vehicle_agent/mission` | `IrMission` | `/fms/route_window` | `FMSRouteWindow` | latched; republished at `window_rate_hz` (default 1.0) as an ego-centric slice |
| `/vehicle_agent/command` | `IrCommand` | `/fms/stop_point` | `FMSStopPoint` | PAUSE/RESUME only; ESTOP/ABORT pass through other channels |
| `/localization/kinematic_state` | `Odometry` | (none) | — | latched ego pose for both `stop_point.x/y` and the window cropping |

### 2.2 Mapping — `IrMission` → `FMSRouteWindow`

The codec convention from `ahtfms_ns/codec.cpp` already records segment
orientation in the waypoint's `segment_id` as `"segN:fwd"` / `"segN:rev"`.
The bridge inverts this:

```
group IrMission.waypoints by parsed segment_id ─▶ FMSPathSegment
  prefix "segN"      ─▶ segment index (just used to preserve order)
  suffix ":fwd"      ─▶ ORIENT_FORWARD (0)
  suffix ":rev"      ─▶ ORIENT_REVERSE (1)
  segment_id missing ─▶ single segment, ORIENT_FORWARD
```

Per-field mapping:

| `IrMission` / `IrWaypoint` | `FMSRouteWindow` / `SegmentWaypoint` |
|---|---|
| `id` | `route_id` |
| (no field) | `vehicle_id` = node param `vehicle_id` |
| (no field) | `job_id`, `job_task_id` = `""` |
| `issued_at_ms` | `header.stamp` (ns conversion) |
| `kind == ABORT` | `status = STATUS_CANCELLED`, segments empty |
| `kind == ROUTE` (any other) | `status = STATUS_ACTIVE` |
| `waypoints[k].x/y/z` | `points[k].x/y/z` |
| `waypoints[k].yaw` | `points[k].heading` |
| `waypoints[k].target_speed` | `points[k].speed_limit` |
| (no field) | `points[k].point_type = POINT_NORMAL` (last point in last segment → POINT_DESTINATION) |

`STOP_AT`/`RESUME`/`PARK` mission kinds are uncommon in IR today — emit
a `STATUS_ACTIVE` route window for `STOP_AT`/`PARK` (treat as a route to
the stop pose), `STATUS_CANCELLED` for `RESUME`. The bridge logs an
`RCLCPP_WARN` whenever a mission kind doesn't have a one-to-one mapping
so we can revisit if it becomes common.

### 2.3 Mapping — `IrCommand` → `FMSStopPoint`

`FMSStopPoint` has only `{vehicle_id, x, y, clear}`. There's no PAUSE
vs. ABORT distinction at the converter; the only knob is `clear`.

| `IrCommand.kind` | Action |
|---|---|
| `PAUSE` | publish `FMSStopPoint{clear=false, x/y=last ego}` |
| `RESUME` | publish `FMSStopPoint{clear=true, x/y=0}` |
| `ABORT` | publish `FMSStopPoint{clear=false}` AND log warn ("abort handled upstream") |
| `ESTOP` | do not publish — ESTOP rides `EstopChannel`, not the converter |
| `CLEAR_FAULT` | no-op (operator concern, not the planner's) |

This is "lossy" — the converter can't tell PAUSE from ABORT — but it
preserves the only semantic the converter currently honours.

### 2.4 Node shape

```cpp
// vehicle_agent/include/vehicle_agent/ros_bridge/route_bridge.hpp
class RouteBridgeNode : public rclcpp::Node {
public:
  explicit RouteBridgeNode(const rclcpp::NodeOptions &);

  static FMSRouteWindow translateMission(
    const IrMission &, const std::string & vehicle_id,
    const rclcpp::Time & stamp);
  static std::optional<FMSStopPoint> translateCommand(
    const IrCommand &, const std::string & vehicle_id,
    double ego_x, double ego_y, bool ego_received,
    const rclcpp::Time & stamp);
  // Crop a full FMSRouteWindow to an ego-centric slice. See §2.8.
  static FMSRouteWindow sliceAroundEgo(
    const FMSRouteWindow & full, double ego_x, double ego_y,
    bool ego_received, double behind_m, double ahead_m);

private:
  void onMission(IrMission::ConstSharedPtr);
  void onCommand(IrCommand::ConstSharedPtr);
  void onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr);
  void onWindowTimer();

  std::string vehicle_id_;
  double window_ahead_m_{100.0};
  double window_lookback_m_{10.0};
  double window_rate_hz_{1.0};

  rclcpp::Subscription<IrMission>::SharedPtr sub_mission_;
  rclcpp::Subscription<IrCommand>::SharedPtr sub_command_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<FMSRouteWindow>::SharedPtr pub_route_;
  rclcpp::Publisher<FMSStopPoint>::SharedPtr pub_stop_;
  rclcpp::TimerBase::SharedPtr window_timer_;

  mutable std::mutex ego_mtx_;
  double ego_x_{0.0}, ego_y_{0.0};
  bool ego_received_{false};

  mutable std::mutex route_mtx_;
  IrMission::ConstSharedPtr latched_;
  uint64_t latched_seq_{0};
};
```

Ship as a composable node next to `VehicleAgentNode` so launches can
load both into the same container. Standalone executable + composition
plugin registration via `RCLCPP_COMPONENTS_REGISTER_NODE`.

### 2.5 CMake / launch

- Add `src/ros_bridge/route_bridge.cpp` to `vehicle_agent_core`.
- New executable `va_route_bridge`, composable node
  `vehicle_agent::RouteBridgeNode`.
- New launch file `launch/route_bridge.launch.py` (small — just declares
  `vehicle_id`, source topic params, and starts the node).
- `complete_autoware_system_fms.launch.xml`: when `use_vehicle_agent`
  is on, include `route_bridge.launch.py` immediately after the VA
  include. Behind a separate `use_va_route_bridge` arg (default true)
  so a deployment can disable it while iterating on the mapping.

### 2.6 Tests

- `test/unit/test_route_bridge_mapping.cpp` — pure function tests on
  `translateMission` / `translateCommand`. Pinned cases:
  - single-segment forward route → one FMSPathSegment with
    `ORIENT_FORWARD`
  - mixed `seg0:fwd` + `seg1:rev` → two segments with correct
    orientations and the right point counts
  - empty waypoints + `kind=ABORT` → empty segments,
    `STATUS_CANCELLED`
  - missing segment_id (legacy producer) → single forward segment
  - last point of last segment marked `POINT_DESTINATION`
- `test/unit/test_route_bridge_window.cpp` — pure function tests on
  `sliceAroundEgo`. Pinned cases: no-ego fallback returns full route;
  ego mid-route centers window; ego before start includes start; ego
  past end fires the ≥1-preceding guard; window cutting an LC zone
  extends to enclose the LC pair; orientation flip preserves segment
  boundary with the boundary point duplicated; header/id fields
  passthrough.
- `test/integration/test_route_bridge_flow.cpp` — spin a RouteBridgeNode
  + a SensorMsgs-style test publisher. Cases: single mission gives the
  first translated window; PAUSE uses latched ego; ESTOP does not
  publish; **`OneMissionGivesRepeatedWindows`** — the regression guard
  for the halt-midway failure (one IrMission in must produce a stream
  of windows at `window_rate_hz`); **`SameIdRefreshReplacesGeometry`**
  — re-pushing the same `route_id` with extended waypoints reflects in
  the downstream stream without an intervening `STATUS_CANCELLED`.
- Smoke: drive a scenario through `vehicle_agent_sim.launch.py` +
  `route_bridge.launch.py` and confirm `ros2 topic echo /fms/route_window`
  shows the converted form arriving at ~1 Hz.

### 2.7 Out of scope (deliberately)

- Drivable-area generation. That's the converter's job.
- Inverting back to IR for telemetry uplink. Out of scope; the existing
  `RosSubscriber` already produces telemetry directly from Odometry.

### 2.8 Data plane vs. control plane

The bridge is the seam between two concerns that used to be conflated
on the single `IrMission` topic:

| Plane | Trigger | Owner | What it changes |
|---|---|---|---|
| Control / lifecycle | new `route_id`, `KIND_ABORT`, completion | `MissionStateMachine` | FSM state (IDLE → EXECUTING → DONE / ABORTING / FAILED) |
| Data / geometry | same `route_id` re-push, ego progress | `RouteBridgeNode` (latched mission + window timer) | The polyline `igv_path_converter` consumes |

The FSM short-circuits same-`id` re-pushes as silent data-plane
refreshes (see §4.7 in `vehicle_agent.md`); the bridge latches the
latest snapshot and slides an ego-centric window over it every
`1/window_rate_hz` seconds. Effects:

- A single push from any producer (RViz panel, scenario_loopback) gives
  the converter a continuous stream — its `stale_threshold_` (~5 s
  default) never trips.
- A real FMS that pushes `RouteNotify` on-change (ICD §8.63) gets its
  extended/truncated geometry picked up on the next window tick, with
  no FSM churn.
- Out-of-order pushes are skipped when `IrMission.seq` is non-zero
  (`ros_injector` and `scenario_loopback` set it; the AHTFMS-NS codec
  leaves it 0 today, which simply disables the check).

`KIND_ABORT` clears the latch and publishes one `STATUS_CANCELLED`
immediately so the converter releases its cached path without waiting
for the next tick.

#### Window parameters

| Parameter | Default | Notes |
|---|---|---|
| `window_ahead_m` | `100.0` | Meters ahead of ego included in each window |
| `window_lookback_m` | `10.0` | Meters behind ego included in each window |
| `window_rate_hz` | `1.0` | Republish rate. Must stay above `igv_path_converter::stale_threshold_` (~5 s) to keep the cached path alive when the FMS is quiet. |

`sliceAroundEgo` preserves the upstream segment grouping: LC pairs are
never split across the window edge, orientation flips always sit on
segment boundaries (with the boundary point duplicated on both sides),
and the `≥1-preceding-waypoint` guard prevents degenerate single-point
windows that would collapse the path generator.

---

## 3. Artefact 2 — `ros_injector` adapter + RViz toggle

Lets the existing `FmsBridgePanel` exercise the full path
RViz → VA → bridge → converter, instead of bypassing VA. The pattern
matches every other adapter: a new `ICloudProtocolAdapter`
implementation that happens to listen on ROS topics instead of a
network socket.

### 3.1 The adapter

```
class RosInjectorAdapter : public ICloudProtocolAdapter {
  // name() = "ros_injector"; priority = 50 (above loopback=10, below ahtfms_ns=200)
  //
  // start(cb):
  //   sub_mission_inject_ <- /vehicle_agent/inject_mission  (IrMission)
  //   sub_command_inject_ <- /vehicle_agent/inject_command  (IrCommand)
  //   on_connection_state_changed(name(), true)  — always "connected"
  //
  // on each IrMission: build ir::Mission with source_adapter="ros_injector"
  //                    fire callbacks.on_mission(m)
};
```

Lives in `src/adapters/ros_injector_adapter.cpp`. Implements only the
downlink half — uplink publishMethods are no-ops because there's
nowhere meaningful to "send" telemetry on the operator side.

Why a new adapter rather than extending `RosSubscriber`:
- The adapter contract is the right shape (downlink emits IR through
  AdapterCallbacks, the FSM and OfflineQueue route it like any other
  cloud-delivered mission).
- It composes — the operator can run `adapters=[ros_injector,ahtfms_ns]`
  and exercise both paths in the same node.
- It doesn't muddy `RosSubscriber`, whose only job is Odometry→Telemetry.

### 3.2 Node wiring

- New `else if (name == "ros_injector")` branch in
  `vehicle_agent_node.cpp`'s adapter dispatch.
- Declare `ros_injector.mission_topic` (default
  `/vehicle_agent/inject_mission`) and
  `ros_injector.command_topic` (default `/vehicle_agent/inject_command`)
  ROS params up front.
- No new config YAML needed (defaults are sane).

### 3.3 The RViz panel toggle

- Add a `QComboBox` "Publish via" at the top of `FmsBridgePanel` with
  two entries: `igv_path_converter` (default, today's behaviour),
  `vehicle_agent`.
- On `onSendRoute()`:
  - if combo == `igv_path_converter`: publish `FMSRouteWindow` to
    `/fms/route_window` (unchanged from today).
  - if combo == `vehicle_agent`: build an `IrMission` from the same
    waypoint table; publish to `/vehicle_agent/inject_mission`.
- On `onSendStop()` / `onSendResume()`:
  - if combo == `vehicle_agent`: publish an `IrCommand` with
    `kind=PAUSE` or `kind=RESUME` to `/vehicle_agent/inject_command`.
- Persist the combo choice as a panel setting (load/save in
  `rviz_common::Panel::{load,save}`).

The IrMission build mirrors the FMSRouteWindow build the panel already
does, just with the inverse mapping from §2.2:

| Panel field | `IrMission` |
|---|---|
| `route_id_input_` text | `id` |
| (auto) | `kind = KIND_ROUTE` |
| (auto, monotonic) | `seq` |
| now() in ms | `issued_at_ms` |
| each table row | one `IrWaypoint` |
| orientation combo | encoded into each waypoint's `segment_id` as `seg0:fwd`/`seg0:rev` |
| (constant) | `source_adapter = "ros_injector"` |

The panel doesn't need to know `ros_injector` exists conceptually — the
topic name `/vehicle_agent/inject_mission` is the contract. But the
adapter must be in the running VA's `adapters` list, so the panel's
status section should subscribe to `/diagnostics` and surface a warning
when `vehicle_agent` is selected but `ros_injector` isn't connected.

### 3.4 Tests

- `test/unit/test_ros_injector_adapter.cpp` — publish an `IrMission` on
  the injection topic via a test publisher, assert the adapter calls
  `on_mission` with matching contents and `source_adapter="ros_injector"`.
- `test/integration/test_panel_to_va_to_bridge.cpp` (optional but high
  value) — chain panel-emitted IR → `RosInjectorAdapter` → FSM →
  `RosPublisher` → `RouteBridgeNode` → `FMSRouteWindow`. Asserts that
  flipping the panel toggle gives byte-equivalent FMSRouteWindow output
  to the legacy path for a fixed waypoint list.

### 3.5 Out of scope (deliberately)

- Replacing the legacy converter-injection path. Operators sometimes
  need to bypass VA (e.g. when debugging the converter itself in
  isolation). The toggle keeps both paths usable.
- Bi-directional sync — the panel doesn't subscribe to VA status. The
  `AdapterStatus` section already surfaces the converter's view; the
  /diagnostics side mentioned in §3.3 is an *additive* indicator only.

---

## 4. Phasing

A single PR is fine for both halves — they're small and the integration
test in §3.4 covers both. If we want to land them separately:

**Phase 1 — `va_route_bridge` only (≈1 day):**
- Lets the cloud→VA→converter path light up end-to-end without
  touching the RViz panel.
- Risk: low. Pure additive: a new subscriber on `/vehicle_agent/mission`,
  a new publisher on `/fms/route_window` (no other process publishes
  there *except* the panel, and the bridge running with no VA-published
  mission is silent).
- Exit gate: with the loopback adapter and a scenario file, `ros2 topic
  echo /fms/route_window` shows the converted form within
  `delay_ms + ε` of the scenario step.

**Phase 2 — `ros_injector` adapter + panel toggle (≈1 day):**
- Pure addition to VA + a localized change to the RViz plugin.
- Exit gate: in RViz, with the toggle set to `vehicle_agent`, clicking
  "Send Route" produces the same FMSRouteWindow on
  `/fms/route_window` that the panel's `igv_path_converter` mode does,
  modulo the canonical mission_id format.

---

## 5. File-by-file change list

### Phase 1 — `va_route_bridge`

```
src/igv/vehicle_agent/
  include/vehicle_agent/ros_bridge/
    route_bridge.hpp                                    (NEW)
  src/ros_bridge/
    route_bridge.cpp                                    (NEW; latch + window timer + sliceAroundEgo)
  launch/
    route_bridge.launch.py                              (NEW; declares window_ahead_m / window_lookback_m / window_rate_hz)
  test/unit/
    test_route_bridge_mapping.cpp                       (NEW)
    test_route_bridge_window.cpp                        (NEW; sliceAroundEgo cases)
  test/integration/
    test_route_bridge_flow.cpp                          (NEW; includes OneMissionGivesRepeatedWindows + SameIdRefreshReplacesGeometry)
  CMakeLists.txt                                        (MOD: add sources, executable, gtest targets)
  README.md                                             (MOD: section 4.12 + new "deferred item closed")
  doc/vehicle_agent.md                                  (MOD: §12 closes item 1)

src/launcher/autoware_launch/autoware_launch/launch/
  complete_autoware_system_fms.launch.xml               (MOD: add use_va_route_bridge arg + include)
```

### Phase 2 — `ros_injector` + RViz toggle

```
src/igv/vehicle_agent/
  include/vehicle_agent/adapters/
    ros_injector_adapter.hpp                            (NEW)
  src/adapters/
    ros_injector_adapter.cpp                            (NEW)
  src/vehicle_agent_node.cpp                            (MOD: dispatch branch + new params)
  test/unit/
    test_ros_injector_adapter.cpp                       (NEW)
  test/integration/
    test_panel_to_va_to_bridge.cpp                      (NEW, optional)
  CMakeLists.txt                                        (MOD: add sources + gtests)
  doc/vehicle_agent.md                                  (MOD: adapter table, priority table)

src/igv/fms_bridge_rviz_plugin/
  src/fms_bridge_panel.hpp                              (MOD: combo box + IR publishers)
  src/fms_bridge_panel.cpp                              (MOD: ~80 lines for toggle + IrMission build)
  package.xml                                           (MOD: depend on vehicle_agent if needed — but IR msgs live in igv_planning_msgs so no new dep)
  README.md                                             (MOD: brief note on the toggle)
```

---

## 6. Risks & open questions

1. **Segment grouping correctness.** The codec's
   `"segN:fwd"`/`"segN:rev"` convention is the only segment-orientation
   carrier in IR today. If a future adapter forgets to set it, the
   bridge fans all waypoints into one forward segment — which is
   wrong for any reverse leg. Mitigation: bridge logs a `WARN` once
   per mission when fewer segments are inferred than the codec
   would have produced for the same path. Long-term fix: lift
   orientation onto `ir::Waypoint` as a proper enum field.
2. **`FMSStopPoint` is too coarse.** PAUSE and ABORT collapse to the
   same wire form. Acceptable for now (matches the converter's
   capability) but should be documented as a known degradation.
3. **Two publishers on `/fms/route_window`.** With both the panel
   (legacy mode) and the bridge active, both can publish. Mitigation:
   the panel guards its publish behind the toggle, so if the operator
   selects `vehicle_agent` the panel goes silent on that topic. With
   the toggle set to `igv_path_converter`, the bridge is still emitting
   whenever VA has a latched mission — which is the *intent*. Note
   that the bridge now publishes continuously at `window_rate_hz` (see
   §2.8), so the panel's legacy mode and the bridge will *both* be
   talking on the topic if both have an active route — the converter
   will see interleaved windows. Acceptable: the panel's converter-direct
   path is for debugging the converter in isolation; in that workflow
   VA should have no latched mission. Document in the README.
4. **`source_adapter` round-trip.** The plugin sets
   `source_adapter="ros_injector"`. If the `ros_injector` adapter
   overrides that (it does, in the current adapter pattern), the
   panel-provided value is lost. Decide: the adapter sets it
   unconditionally (consistent) or only when the inbound IR doesn't
   provide one (lets the panel be honest about origin).
   Recommendation: adapter sets it unconditionally. The panel-side
   "publish via VA" toggle is already proof of origin.
5. **Composable vs. standalone node.** The bridge is small enough that
   shipping both forms costs little; default to standalone in the
   launch file so failures show up obviously, leave the composable
   plugin as an option for production where node count matters.
