# Vehicle Agent — Engineering Reference

This document is the implementation-level reference for the `vehicle_agent`
package. It complements the high-level v1.2 design in
`docs/vehicle_agent/Vehicle_Agent_Design_v1.2.docx` and the user-facing
`src/igv/vehicle_agent/README.md`. Where the design doc tells you *what* the
agent does, this document tells you *how the code is laid out, threaded,
and tested*.

Audience: engineers extending VA, debugging a live deployment, or
reviewing a change. Assumes ROS 2 Galactic, C++17, Autoware Universe.

---

## 1. Overview

`vehicle_agent` is the edge middleware between the cloud fleet-management
system (FMS) and the on-vehicle Autoware planning/control stack. Before
v1.2, cloud↔vehicle integration lived inside `igv_path_planner`,
which mixed three concerns: protocol parsing, simulation, and planning.
VA extracts the first two into a dedicated package with a portable,
ROS-free internal representation and a pluggable adapter layer.

Design goals (from v1.2 §2):

| Goal | How VA delivers it |
|---|---|
| Single edge entrypoint for any cloud protocol | `ICloudProtocolAdapter` + `AdapterMultiplexer` |
| ROS-free, protocol-free core | `ir::*` canonical types in `include/vehicle_agent/ir/` |
| Survive a crash with no lost commands | `OfflineQueue` (SQLite WAL) + startup replay |
| Bounded E-stop latency | `EstopChannel` bypasses `EventBus` (p95 < 50 ms) |
| Auth + anti-replay against tampered messages | `HmacSigner` + `NonceLRU` + `ClockSkewWindow` + `MtlsContext` |
| Concurrent adapters with deterministic primary | `AdapterMultiplexer::connectedPrimaryName()` |
| Standalone simulation without a broker | `ScenarioLoopbackAdapter` reading YAML |

---

## 2. Architecture

```
                     ┌──────────────────────────────────────────────┐
                     │              Cloud / FMS                      │
                     └──────────────────────────────────────────────┘
                          │           │              │
                          ▼           ▼              ▼
                 ┌────────────┐ ┌─────────────┐ ┌─────────────────┐
                 │ ahtfms_ns  │ │ internal_   │ │ scenario_       │
                 │ (TCP)      │ │ zpmc(MQTT5+ │ │ loopback (YAML) │
                 │            │ │ HMAC+mTLS)  │ │                 │
                 └─────┬──────┘ └──────┬──────┘ └────────┬────────┘
                       │               │                 │
                       └──────┬────────┴─────────┬───────┘
                              ▼                  ▼
                    ┌─────────────────────────────────────┐
                    │      AdapterMultiplexer             │
                    │  • downlink union → AdapterCallbacks│
                    │  • uplink mirror to all (per cap)   │
                    │  • connectedPrimaryName() failover  │
                    └─────────────────┬───────────────────┘
                                      │ on_mission / on_command
                                      │ on_connection_state_changed
                                      ▼
   ┌───────────────────────────────────────────────────────────────┐
   │                  VehicleAgentNode (rclcpp)                    │
   │                                                               │
   │   ┌────────────┐    ┌─────────────┐    ┌─────────────────┐    │
   │   │ Offline    │◀───│   Adapter   │───▶│   EventBus      │    │
   │   │ Queue      │    │ callbacks   │    │ (FIFO, capped)  │    │
   │   │ (SQLite)   │    │ (node)      │    └────────┬────────┘    │
   │   └─────┬──────┘    └─────────────┘             │ pop          │
   │         │ replay on start                       ▼              │
   │         │                              ┌──────────────────┐   │
   │         │                              │ Mission State    │   │
   │         │                              │ Machine (9 st.)  │   │
   │         │           ack on terminal───▶│ single writer    │   │
   │         │                              └────────┬─────────┘   │
   │         │                                       │ on_status   │
   │         │                                       │ on_ack      │
   │         │                                       ▼             │
   │   ┌─────▼─────────┐    ┌──────────────┐  ┌──────────────────┐ │
   │   │ EstopChannel  │    │ RosSubscriber│  │ RosPublisher     │ │
   │   │ (fast path)   │    │ (telemetry)  │  │ (IR → ROS msgs)  │ │
   │   └───────┬───────┘    └──────┬───────┘  └────────┬─────────┘ │
   │           │                   │                   │           │
   │   ┌───────▼───────────────────▼───────────────────▼─────────┐ │
   │   │             DiagnosticPublisher  (/diagnostics @ 1 Hz)  │ │
   │   └─────────────────────────────────────────────────────────┘ │
   └───────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
                  /vehicle_agent/{mission,command,mission_status},
                  /diagnostics, → igv_path_planner → planning
```

### Source layout

```
src/igv/vehicle_agent/
├── include/vehicle_agent/
│   ├── adapters/                 # ICloudProtocolAdapter + concrete adapters
│   │   ├── ahtfms_ns/            # ICD framing + codec + transport
│   │   ├── internal_zpmc/        # MQTT-bus interface + FakeMqttBus
│   │   └── …
│   ├── app/                      # FSM, EventBus, EstopChannel, structured_log
│   ├── ir/                       # Canonical types (ROS-free, MQTT-free)
│   ├── ros_bridge/               # IR → ROS msgs, Odometry → IR
│   ├── security/                 # HMAC, NonceLRU, ClockSkewWindow, MtlsContext
│   └── storage/                  # OfflineQueue (SQLite WAL)
├── src/                          # mirror of include/, with .cpp impls
├── config/
│   ├── vehicle_agent.yaml        # node defaults
│   └── adapters/                 # per-adapter override files
├── launch/
│   ├── vehicle_agent.launch.py     # production
│   └── vehicle_agent_sim.launch.py # scenario_loopback only
├── scenarios/                    # YAML scenarios consumed by loopback
└── test/
    ├── unit/                     # gtest, no ROS context
    ├── integration/              # gtest with composable nodes / fake transports
    └── fixtures/                 # JSON fixtures for ICD codec tests
```

---

## 3. Build & run

```bash
# Build only this package (and rebuild its dependent if msgs change):
colcon build --packages-up-to vehicle_agent --symlink-install

# Source the workspace:
source install/setup.bash

# Standalone simulation (no broker, no Autoware needed):
ros2 launch vehicle_agent vehicle_agent_sim.launch.py \
  scenario:=$(ros2 pkg prefix vehicle_agent)/share/vehicle_agent/scenarios/basic_pickup.yaml

# Production (AHTFMS-NS via vehicle_agent.launch.py):
ros2 launch vehicle_agent vehicle_agent.launch.py \
  adapters:=ahtfms_ns \
  vehicle_id:=IGV2001 \
  ahtfms_ns.broker_host:=10.0.0.42 ahtfms_ns.broker_port:=8801

# Concurrent adapters (Sprint 5):
ros2 launch vehicle_agent vehicle_agent.launch.py \
  adapters:="ahtfms_ns,scenario_loopback" \
  vehicle_agent_scenario_path:=/path/to/scenario.yaml
```

The `adapters` parameter is declared as `std::vector<std::string>` on the
ROS side. Both launch files split the CSV form via `OpaqueFunction`
before passing it to the node — pass a Python list directly if you're
writing your own launch script.

---

## 4. Component reference

### 4.1 Canonical types (`include/vehicle_agent/ir/`)

| Type | Purpose | nlohmann JSON `to/from_json` |
|---|---|---|
| `Mission` | Downlink: route, stop_at, resume, abort, park | yes |
| `Waypoint` | Pose + target_speed + segment_id | yes |
| `Command` | PAUSE, RESUME, ABORT, ESTOP, CLEAR_FAULT | yes |
| `Telemetry` | Pose, speed, steering, mission ctx, e_stop | yes |
| `MissionStatus` | One of 9 `MissionState` values + progress + detail | yes |
| `Ack` | status (OK/REJECTED/PREEMPTED/FAILED), stamp, detail | yes |
| `MissionCheckResult` | Validation outcome between RECEIVED and EXECUTING | yes |
| `Event` | `variant<Mission,Command,…,Tick>` for the bus | n/a |

All IR types are header-only. Adapters are expected to translate their
wire format into IR before handing off to the rest of the system. The
`source_adapter` field on `Mission`/`Command` records which adapter
delivered the object — used by diagnostics and for cycle-detection if a
future feature mirrors downlink between adapters.

### 4.2 Adapter contract (`adapters/i_cloud_protocol_adapter.hpp`)

```cpp
class ICloudProtocolAdapter {
public:
  virtual std::string name() const = 0;
  virtual AdapterCapabilities capabilities() const = 0;

  virtual void start(AdapterCallbacks cb) = 0;
  virtual void stop() = 0;
  virtual bool connected() const { return true; }

  virtual void publishTelemetry(const ir::Telemetry &)        {}
  virtual void publishMissionStatus(const ir::MissionStatus &){}
  virtual void publishAck(const ir::Ack &)                    {}
};
```

`AdapterCapabilities::priority` orders failover candidates. Reserved
values:

| Adapter | Priority |
|---|---|
| `internal_zpmc` | 300 |
| `ahtfms_ns` | 200 |
| `scenario_loopback` | 10 |

Adapters MUST be safe to call from the node's main thread (start/stop)
and from their own I/O thread (callbacks). The lifecycle contract is:
`start()` may spawn threads and open sockets; `stop()` joins those
threads before returning; the destructor calls `stop()` defensively.

`AdapterCallbacks` carries:

| Field | Direction | Fired by |
|---|---|---|
| `on_mission` | downlink | adapter on decoded Mission |
| `on_command` | downlink | adapter on decoded Command |
| `on_execution_complete` | sync from "execution side" | loopback (Sprint 1), ROS subscriber (later) |
| `on_connection_state_changed` | health | adapter on every connect/disconnect flip |

### 4.3 AHTFMS-NS adapter (`adapters/ahtfms_ns_adapter.*`)

Concrete implementation of `ICloudProtocolAdapter` for the AHTFMS-NS
ICD v1.1.1. The adapter is split into three layers so each can be
tested in isolation:

```
  ┌─────────────────────────────────────────────────────────┐
  │  AhtfmsNsAdapter  (lifecycle, stream reassembly,        │
  │                    callbacks → IR)                       │
  └──────────────┬──────────────────────────────────┬───────┘
                 │ ITransport::send / on_bytes      │ codec
                 ▼                                  ▼
  ┌──────────────────────────┐        ┌────────────────────────┐
  │ TcpTransport (prod)      │        │ codec: PB-placeholder  │
  │ FakeTransport (tests)    │        │ (JSON) ↔ ir::Mission / │
  └──────────────────────────┘        │ ir::Command / …        │
                 │                    └────────────────────────┘
                 ▼
  ┌──────────────────────────┐
  │ framing: encode/decode   │
  │ CM Header + Msg Header   │
  │ (FMS1 magic, LE ints)    │
  └──────────────────────────┘
```

**Framing** (`ahtfms_ns/framing.hpp`, 51 B fixed header):

```
+------+-----------+-----------+-------------------+--------+--------+----------+
| FMS1 | sender_sys| target_sys|     event_dt      | type   | length |  PB body |
| (4)  |   (9)     |   (9)     |        (23)       | (u16le)| (u32le)|  (var)   |
+------+-----------+-----------+-------------------+--------+--------+----------+
                  ── CM Header (45) ──             ── Msg Header (6) ──
```

`encodeFrame` throws `std::invalid_argument` if a CHAR[] field overflows
its slot (programmer error); `decodeFrame` returns one of
`{OK, TRUNCATED_HEADER, BAD_MAGIC, TRUNCATED_PAYLOAD, OVERSIZED_PAYLOAD}`
and leaves `pos` untouched on failure so the caller can wait for more
bytes.

**Message types handled** (matched against ICD §5/§6/§7):

| ID | Direction | Decoded as |
|---|---|---|
| 1001 REMOTE_COMMAND | FMS→NS | `ir::Command` (PAUSE/RESUME/ABORT/CLEAR_FAULT; SUSPEND+EMERGENCY_BRAKE → ESTOP) |
| 2001 REMOTE_COMMAND_RESPONSE | NS→FMS | from `ir::Ack` |
| 3001 JOB_NOTIFY | FMS→NS | `ir::Mission` (empty `job_id` → `ABORT`) |
| 3002 ROUTE_NOTIFY | FMS→NS | `ir::Mission` (segment-flattened, "segN:fwd"/"segN:rev") |
| 3003 STOP_POINT_NOTIFY | FMS→NS | `ir::Command` (activate=true → PAUSE, false → RESUME) |
| 4001 VEHICLE_KEY_EVENT_STATUS | NS→FMS | SYNC_REQUEST emitted once on connect (§8.8) |
| 4002 VEHICLE_BASIC_STATUS | NS→FMS | from `ir::Telemetry` |

Unsupported message types are decoded and discarded (the counter
`unsupported_msg_type` is bumped so they appear in diagnostics).

**Note on PB**: the ICD specifies Protobuf bodies, but this sprint uses
nlohmann JSON with field names matching the proto identifiers as a
placeholder. The codec layer is the single seam — when real `.proto`
files arrive, only `codec.cpp` changes.

**Inbound stream reassembly** lives in `AhtfmsNsAdapter::onBytes`. The
adapter keeps a `rx_tail_` buffer; on each chunk it appends and then
pulls whole frames off the front until `decodeFrame` returns
`TRUNCATED_HEADER` or `TRUNCATED_PAYLOAD`, then waits for more bytes.

**`target_sys` gate**: frames whose `target_sys != vehicle_id` are
silently dropped (counter `target_mismatches` bumped). This is how a
single broker can address multiple IGVs without IPC fan-out.

### 4.4 Internal ZPMC adapter (`adapters/internal_zpmc_adapter.*`)

MQTT 5 transport with HMAC-SHA256 envelopes:

```
publish/subscribe payload:
  {
    "payload": <ir_json>,        # the inner IR object
    "ts":      <epoch_ms>,
    "nonce":   <hex string>,
    "sig":     <hex hmac-sha256 over `<payload>\n<ts>\n<nonce>` >
  }
```

Inbound verification chain (`openEnvelope`):

1. Parse outer JSON. Bad parse → `decode_errors++`, drop.
2. If a signing key is configured: verify `sig`. Mismatch →
   `bad_signature++`, drop.
3. Check `ts` within `±skew_tolerance_ms` of local clock. Outside →
   `skew_rejections++`, drop.
4. Check `nonce` not in `NonceLRU`. Seen → `replay_rejections++`, drop.
5. Decode inner `payload` as `ir::Mission` (on mission topic) or
   `ir::Command` (on command topic). Decode failure →
   `decode_errors++`.

Topics are exact-match (no wildcards), formed as
`<prefix><vehicle_id>` — defaults documented in
`config/adapters/internal_zpmc.yaml`.

**Transport implementations**: `IMqttBus` has two implementations.
`FakeMqttBus` (test) is an in-process broker shared by paired client
handles, used by the unit and integration tests. `PahoMqttBus`
(`paho_bus.{hpp,cpp}`) is the production wire — MQTT 5 over TLS,
built on Eclipse Paho's `mqtt::async_client`. The production
constructor `InternalZpmcAdapter(Config)` instantiates `PahoMqttBus`
from `Config::{broker_host, broker_port, mtls}`; TLS is selected when
the broker port is the conventional `:8883` or when any mTLS path is
populated. The test constructor
`InternalZpmcAdapter(Config, unique_ptr<IMqttBus>)` accepts an
injected bus so tests can run without a network broker. `start()`
still throws `std::runtime_error` if the injected bus is null — that
path is reserved for tests that want to verify the never-started
adapter shape.

`PahoMqttBus` is non-blocking on `start()` — the connect is async, so
the agent comes up cleanly even when the broker is unreachable and
flips to `connected()==true` only when the broker accepts.
Subscriptions issued pre-connect are buffered and re-issued by paho's
`connected` callback after each successful (re)connect. The vendored
paho lives under `src/others/paho.mqtt.{c,cpp}` and is built by
colcon ahead of `vehicle_agent`.

### 4.5 Scenario loopback adapter (`adapters/scenario_loopback_adapter.*`)

Reads a YAML scenario at start, sleeps `delay_ms` between steps, fires
`on_mission` / `on_command` / `on_execution_complete` callbacks. No
network deps. Used for unit tests and `vehicle_agent_sim.launch.py`.

```yaml
# scenarios/basic_pickup.yaml
steps:
  - delay_ms: 3000
    type: mission
    id: scn-001
    waypoints:
      - { x: 80, y: 20, yaw: 0.0, target_speed: 2.0, segment_id: "s0" }
  - delay_ms: 1000
    type: command
    mission_id: scn-001
    kind: PAUSE
  - delay_ms: 500
    type: complete
    mission_id: scn-001
```

### 4.6 Multiplexer (`adapters/adapter_multiplexer.*`)

Owns the adapter list and applies a uniform routing policy:

| Direction | Policy |
|---|---|
| Downlink (`Mission`/`Command`) | Unioned — any adapter may deliver. The FSM dedups (a stale mission whose `id` matches the active one is filtered later). |
| Uplink (`Telemetry`/`Status`/`Ack`) | Mirrored to every adapter whose capability bit is set. Not gated on `connected()` — adapters are responsible for their own offline queueing. |
| Primary | `primaryName()` = highest `priority` regardless of state; `connectedPrimaryName()` = highest-priority *connected* adapter. Diagnostics surface the latter. |

The multiplexer holds a single `std::mutex` around its adapter vector;
callbacks fire on adapter-owned threads. Adding/removing adapters at
runtime is not currently supported.

### 4.7 Mission state machine (`app/mission_state_machine.*`)

The 9-state machine per v1.2 §7. Single-writer: only the FSM thread
inside `VehicleAgentNode` calls `handle(Event)`. The `mtx_` member
serializes against E-stop, which is also synchronous but enters
through `EstopChannel::trigger`.

```
              ┌───────────────────────────────────────────────────┐
              │                                                   │
              ▼                                                   │
 ┌──────────┐  mission   ┌──────────┐ validate  ┌──────────┐  done│
 │  IDLE    │───────────▶│ RECEIVED │──────────▶│VALIDATING│──────┤
 └──────────┘            └────┬─────┘   ok      └────┬─────┘  ┌───▼───┐
       ▲                      │                      │        │ DONE  │
       │                      │ validate bad         │ accept └───────┘
       │                  ┌───▼────┐             ┌───▼────┐
       │                  │ FAILED │◀────┐       │EXECUTNG│◀──────┐
       │                  └────────┘     │       └───┬────┘       │
       │ next mission                    │           │  PAUSE     │
       │                                 │           ▼            │
       │                ABORT/ESTOP ┌────┴─────┐  ┌────────┐ RESUME│
       └────────────────────────────│ABORTING  │  │ PAUSED │───────┘
                                    └──────────┘  └───┬────┘
                                                      │ new mission
                                                      ▼
                                                 ┌──────────┐
                                                 │PREEMPTED │
                                                 └──────────┘
```

Terminal states (`DONE`, `FAILED`, `PREEMPTED`) emit a final
`MissionStatus`; the node's `onMissionStatus` callback acks the
corresponding offline-queue row.

**Same-id refresh.** A new `IrMission` arriving while
`state_ ∈ {EXECUTING, PAUSED}` with `m.id == active_->id` is treated
as a silent data-plane refresh: `active_->waypoints / seq /
issued_at_ms` are updated in place; no FSM transition fires; no ack
emits. This is what makes an FMS that pushes `RouteNotify` on-change
for the same `route_id` (AHTFMS-NS ICD §8.63) usable end-to-end —
without the short-circuit, every same-id push would emit a
`PREEMPTED` ack and cycle the FSM through `RECEIVED → VALIDATING →
EXECUTING`. Different `id` keeps the existing preempt-and-promote
path. The geometry refresh is picked up downstream by
`RouteBridgeNode`'s window timer on its next tick — see
`doc/plan-rviz-va-integration.md` §2.8.

### 4.8 Event bus (`app/event_bus.*`)

Bounded FIFO with a single condvar. Multiple producers (each adapter
callback, the ROS subscriber on every odometry-derived telemetry
sample), one consumer (the FSM thread). Capacity is configurable via
the `event_bus_capacity` ROS parameter; on overflow `push` blocks
briefly — telemetry-side producers should drop oldest rather than
block, which is enforced at the call site.

`close()` is called from the node destructor; the consumer breaks out
when the bus is closed and drained.

### 4.9 E-stop channel (`app/estop_channel.*`)

Direct, synchronous call into `MissionStateMachine::handle`. Bypasses
`EventBus` so a flooded queue can't delay the stop. Measured latency
is exposed via `lastTriggerLatencyMicros()` for test assertions
(p95 < 50 ms is the design budget).

Idempotent: re-triggering when `active()` is already true is a no-op
inside the FSM (the state remains `ABORTING`/`FAILED`).

### 4.10 Offline queue (`storage/offline_queue.*`)

SQLite WAL with a single append-only table:

```sql
CREATE TABLE inbound (
  id          INTEGER PRIMARY KEY AUTOINCREMENT,
  kind        TEXT NOT NULL,            -- 'mission' | 'command'
  payload     TEXT NOT NULL,            -- nlohmann JSON
  received_ms INTEGER NOT NULL,
  acked       INTEGER NOT NULL DEFAULT 0
);
```

Lifecycle:

1. Adapter callback `on_mission` / `on_command` enqueues a row and
   then pushes the event onto the bus, carrying `queue_row_id`.
2. FSM consumes the event. For commands, the node acks the row as
   soon as `handle()` returns. For missions, the row is acked only
   when the FSM emits a terminal status (`DONE` or `FAILED`).
3. At startup, `replayPending()` reads unacked rows in receive order,
   reconstructs the IR via `nlohmann::json::from_json`, and pushes
   them back onto the bus before the adapters' fresh events arrive.

Crash-safe by virtue of SQLite WAL. `purgeAckedOlderThan` is exposed
for an ops-side housekeeping cron; the node itself does not call it.

### 4.11 Security stack (`security/*`)

| Primitive | File | Used by |
|---|---|---|
| `HmacSigner` | `hmac_signer.*` | `internal_zpmc` (sign outbound, verify inbound) |
| `NonceLRU` | `nonce_lru.*` | `internal_zpmc` (drop replays) |
| `ClockSkewWindow` | `clock_skew_window.*` | `internal_zpmc` (drop out-of-window timestamps) |
| `MtlsContext` | `mtls_context.*` | `internal_zpmc` (validates cert/key/CA file paths exist; `PahoMqttBus` projects the `MtlsConfig` into `mqtt::ssl_options`) |

Canonical signing input is `<payload>\n<ts>\n<nonce>`. Newlines bind
the three fields so tampering with any one produces a different MAC.
`HmacSigner::verify` is constant-time.

`NonceLRU` evicts on two axes: capacity (size > `max_entries` →
oldest dropped) and TTL (entries older than `ttl_ms` dropped lazily
on access).

`ClockSkewWindow` is symmetric — `|ts - now| > tolerance` is rejected.
Both sides must NTP-sync; document this in the ops runbook.

### 4.12 ROS bridge (`ros_bridge/*`)

| Component | Publishes / subscribes | Topic |
|---|---|---|
| `RosPublisher` | `igv_planning_msgs/msg/IrMission` | `<ns>/mission` |
|  | `igv_planning_msgs/msg/IrCommand` | `<ns>/command` |
|  | `igv_planning_msgs/msg/IrMissionStatus` | `<ns>/mission_status` (transient_local) |
| `RosSubscriber` | `nav_msgs/msg/Odometry` | `/localization/kinematic_state` |
| `RouteBridgeNode` (`va_route_bridge` executable) | subscribes IrMission, IrCommand, Odometry; publishes `FMSRouteWindow`, `FMSStopPoint` | `/fms/route_window`, `/fms/stop_point` |
| `DiagnosticPublisher` | `diagnostic_msgs/msg/DiagnosticArray` | `/diagnostics` |

The default namespace prefix is `/vehicle_agent` (overridable via
the `publish_ns` ROS parameter). Mission/command use `QoS(10)`;
mission_status uses `transient_local` so a late-joining subscriber
gets the latest state.

`RosSubscriber::updateMissionContext` is called by the FSM status
sink so the next outbound telemetry packet carries the current
mission_id, mission_state, and e_stop flag without a round trip.

`RouteBridgeNode` is a separate executable (`va_route_bridge`,
launched via `launch/route_bridge.launch.py`) but ships in the same
package because the IR↔FMS mapping is VA-internal vocabulary. It
latches the last `IrMission` and republishes an ego-centric windowed
`FMSRouteWindow` at `window_rate_hz` (default 1.0). Single-push
producers (RViz panel, scenario_loopback) and chatty FMS producers
both get a continuous stream downstream — the converter's
`stale_threshold_` never trips. Full data-plane/control-plane split,
windowing parameters, and the `sliceAroundEgo` contract are in
`doc/plan-rviz-va-integration.md` §2.8.

### 4.13 Diagnostic publisher

Runs at 1 Hz (configurable). Emits one `DiagnosticStatus` per
registered adapter plus a top-level `vehicle_agent` status:

| Level | Condition |
|---|---|
| OK | at least one adapter connected AND FSM not FAILED |
| WARN | all adapters disconnected, OR queue depth > `queue_warn_threshold` |
| ERROR | FSM is FAILED |

Each `DiagnosticStatus` carries key/value pairs: `connected`,
`last_error`, `connected_since_ms`. The node wires
`setAdapterConnected` to the multiplexer's
`on_connection_state_changed` callback.

### 4.14 Structured logging

`app/structured_log.hpp` defines `VA_LOG_{INFO,WARN,ERROR,DEBUG}`
macros that prepend `[mid=<mission_id>]` to every line. Grepping a
mission's full lifecycle:

```bash
ros2 launch vehicle_agent vehicle_agent.launch.py … 2>&1 | grep 'mid=R-2025-0001'
```

---

## 5. Threading model

| Thread | Origin | What runs on it |
|---|---|---|
| `main` (rclcpp executor) | `VehicleAgentNode` ctor | Parameter declaration, multiplexer start, subscription callbacks (`Odometry`), diagnostic timer |
| FSM thread | spawned by node | `EventBus::popBlocking` loop → `MissionStateMachine::handle` → publishers/sinks |
| One I/O thread per adapter | spawned by adapter `start()` | Socket/MQTT read loop → `on_bytes` / `on_message` → IR decode → `AdapterCallbacks` (which enqueue onto the bus) |
| Scenario loopback worker | inside `ScenarioLoopbackAdapter` | YAML iteration + sleep + callbacks |

Synchronization contracts:

- The FSM mutex (`MissionStateMachine::mtx_`) serializes every state
  read/write. Both the FSM thread and `EstopChannel` acquire it.
- `EventBus` and `OfflineQueue` are internally thread-safe.
- `RosPublisher` is single-threaded (called only from the FSM thread).
- Adapter callbacks may fire concurrently — the node-side handlers
  use the offline queue's lock for serialization where needed
  (`mission_rows_` map under `map_mtx_`).

---

## 6. End-to-end data flows

### 6.1 Downlink mission

```
Cloud → adapter onBytes → codec → ir::Mission
       → AdapterCallbacks.on_mission (in adapter I/O thread)
         → OfflineQueue.enqueueMission (commit to SQLite)
         → RosPublisher.publishMission                ─┐
         → EventBus.push(MISSION_RECEIVED, row_id)     │
                                                       │
       FSM thread: pop → handle:                       │
         IDLE       → RECEIVED → VALIDATING (no-op)    │
                    → EXECUTING (on_status fires)      │
                                                       │
       on_status sink (in FSM thread):                 │
         RosPublisher.publishMissionStatus  ───────────┴─▶ Autoware
         AdapterMultiplexer.publishMissionStatus → uplink
         RosSubscriber.updateMissionContext (enriches future telemetry)
```

When mission reaches `DONE` or `FAILED`, the node's
`onMissionStatus` callback acks the offline-queue row.

### 6.2 Uplink telemetry

```
/localization/kinematic_state ─▶ RosSubscriber.onOdom (rate-limited)
                              ─▶ ir::Telemetry { vehicle_id, pose, speed,
                                                  mission_id, mission_state, e_stop }
                              ─▶ AdapterMultiplexer.publishTelemetry
                                  ▶ ahtfms_ns      → VehicleBasicStatus frame
                                  ▶ internal_zpmc  → MQTT publish (signed envelope)
                                  ▶ scenario_loopback → /dev/null
```

`max_rate_hz` (default 10 Hz) is enforced inside `RosSubscriber`; the
adapter side does no further throttling.

### 6.3 E-stop fast path

```
ESTOP command (any adapter) → on_command:
   if kind == ESTOP:
     RosPublisher.publishCommand (immediate)
     EstopChannel.trigger:
        timer ← now
        fsm.handle(Event{COMMAND_RECEIVED, c})
        last_latency_us = now - timer; active = true
   else:
     OfflineQueue.enqueueCommand
     RosPublisher.publishCommand
     EventBus.push(COMMAND_RECEIVED)
```

E-stop bypasses both the queue and the bus. The FSM transitions
`<active> → ABORTING → FAILED`; the status sink mirrors the new state
to uplink and Autoware.

### 6.4 Crash recovery

```
node ctor:
  OfflineQueue.open (WAL replays partial transactions)
  start adapter threads
  replayPending():
    for each unacked row (oldest first):
      decode IR
      if command and kind == ESTOP:
        EstopChannel.trigger; ack immediately
      else:
        EventBus.push (carrying row_id)
```

The replay path runs AFTER adapters start so a fresh inbound message
arriving during boot doesn't reorder ahead of the queue tail.

---

## 7. Configuration reference

### 7.1 Node parameters (declared in `vehicle_agent_node.cpp`)

| Parameter | Type | Default | Notes |
|---|---|---|---|
| `adapters` | string[] | `["scenario_loopback"]` | Names dispatched in node ctor |
| `publish_ns` | string | `/vehicle_agent` | Prefix for `RosPublisher` topics |
| `queue_db_path` | string | `~/.ros/vehicle_agent/queue.db` | `~` is expanded |
| `vehicle_id` | string | `""` | Falls back to per-adapter `vehicle_id` |
| `odometry_topic` | string | `/localization/kinematic_state` | |
| `telemetry_rate_hz` | double | `10.0` | Subscriber-side throttle |
| `diagnostic_rate_hz` | double | `1.0` | DiagnosticPublisher timer |
| `queue_warn_threshold` | int | `32` | Above this, diagnostics go WARN |
| `event_bus_capacity` | int | `1024` | |
| `scenario_path` | string | `""` | Required when `scenario_loopback` is in `adapters` |
| `ahtfms_ns.{vehicle_id,fms_sys,broker_host,broker_port,max_payload_bytes,reconnect_min_ms,reconnect_max_ms}` | … | see `config/adapters/ahtfms_ns.yaml` | |
| `internal_zpmc.{vehicle_id,broker_host,broker_port,hmac_key,skew_tolerance_ms,nonce_capacity,nonce_ttl_ms,mtls.*}` | … | see `config/adapters/internal_zpmc.yaml` | |

### 7.2 Per-adapter config files

`config/adapters/ahtfms_ns.yaml`, `config/adapters/internal_zpmc.yaml`
ship default values and inline documentation. Mount them via
`--params-file` or compose into a higher-level launch.

---

## 8. ROS topic reference

### 8.1 Published

| Topic | Type | QoS | Notes |
|---|---|---|---|
| `/vehicle_agent/mission` | `igv_planning_msgs/msg/IrMission` | `QoS(10)` | Every downlink mission |
| `/vehicle_agent/command` | `igv_planning_msgs/msg/IrCommand` | `QoS(10)` | Every downlink command (including ESTOP) |
| `/vehicle_agent/mission_status` | `igv_planning_msgs/msg/IrMissionStatus` | `QoS(10).transient_local()` | FSM state transitions |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | default | 1 Hz |

### 8.2 Subscribed

| Topic | Type | Used for |
|---|---|---|
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | `ir::Telemetry` uplink |

When debugging in this workspace, remember to specify QoS overrides
when echoing topics (see `CLAUDE.md`):

```bash
ros2 topic echo /diagnostics --qos-reliability reliable
ros2 topic echo /vehicle_agent/mission_status \
  --qos-reliability reliable --qos-durability transient_local
```

---

## 9. Testing

```bash
colcon test --packages-select vehicle_agent
colcon test-result --test-result-base build/vehicle_agent --verbose
```

Sixteen test binaries:

| Binary | Tests | Covers |
|---|---|---|
| `test_event_bus` | push/pop, capacity, close-drains | bus invariants |
| `test_mission_state_machine` | every transition in the 9-state table | FSM correctness |
| `test_scenario_loopback` | YAML → IR → callback sequence | loopback adapter |
| `test_offline_queue` | enqueue/peek/ack/replay, kill-restart | SQLite durability |
| `test_estop_channel` | direct trigger, idempotence, latency budget | fast path |
| `test_hmac_signer` | sign, verify, empty-key rejection | security |
| `test_nonce_lru` | replay rejection, capacity eviction, TTL eviction | security |
| `test_clock_skew_window` | accept/reject inside/outside tolerance | security |
| `test_ahtfms_ns_codec` | 20 cases: framing round-trip, magic/truncation, ICD fixtures | adapter codec |
| `test_loopback_replay` | end-to-end: kill mid-mission → restart → replay | crash safety |
| `test_ahtfms_ns_flow` | 9 cases: connect-sync, frame reassembly, target-mismatch, telemetry uplink, disconnect | adapter wiring |
| `test_multiplexer_dual` | 5 cases: both adapters up, mirroring, IR delivery, failover, nonce replay | multiplex policy |

Fixtures for the ICD codec live in `test/fixtures/ahtfms_ns/` —
golden JSON shapes drawn from the spec's examples plus a few
designed-to-fail cases.

---

## 10. Adding a new cloud adapter

1. Pick a name (lowercase snake) and create
   `include/vehicle_agent/adapters/<name>_adapter.hpp` +
   `src/adapters/<name>_adapter.cpp` implementing
   `ICloudProtocolAdapter`.
2. If the wire protocol is non-trivial, split the codec and transport
   into `adapters/<name>/` (see `ahtfms_ns/` for the pattern). Inject
   the transport via a unique_ptr so tests can swap in a fake.
3. Pick a `priority` distinct from existing adapters. Higher = more
   preferred as primary.
4. Add the source files to `CMakeLists.txt` (`vehicle_agent_core`
   target).
5. Add a `name == "<name>"` branch in
   `vehicle_agent_node.cpp`'s adapter dispatch loop. Declare any new
   ROS parameters up front (so launches can override them even when
   the adapter is off).
6. Add `config/adapters/<name>.yaml` with broker/security/topic
   defaults and inline documentation.
7. Write unit tests under `test/unit/test_<name>_*.cpp` and, if the
   adapter talks to a stateful peer, an integration test under
   `test/integration/`.
8. Add the test binary to the `if(BUILD_TESTING)` block in
   `CMakeLists.txt`.

The multiplexer requires no changes: it picks up the new adapter
through `addAdapter` and applies the standard mirroring/failover
policy.

---

## 11. Operations & failure modes

| Failure | Symptom | Recovery |
|---|---|---|
| Cloud broker drops | `connected()` flips false; multiplexer's `connectedPrimaryName()` falls back to the next-priority adapter | Adapter's I/O thread reconnects with exponential backoff (`reconnect_min_ms` → `reconnect_max_ms`). Diagnostic flips to WARN. |
| FMS re-pushes the same `route_id` (RouteNotify on-change per ICD §8.63) | New mission's `id` matches active; FSM in `EXECUTING` / `PAUSED` | Silent data-plane refresh: `active_->waypoints/seq/issued_at_ms` updated in place, no state transition, no ack. `RouteBridgeNode` picks the new geometry up on its next window tick. See §4.7. |
| FMS sends a genuinely new mission while active | New mission's `id` differs; FSM in `EXECUTING` / `PAUSED` | `PREEMPTED` ack emitted for the active mission; the new mission is promoted via `RECEIVED → VALIDATING → EXECUTING`. |
| Tampered MAC on internal_zpmc | `bad_signature` counter increments; message dropped | None — counter visible via adapter `counters()`, surface in diagnostics if needed. |
| Clock skewed > tolerance | `skew_rejections` counter increments; messages dropped on both directions | Fix NTP; the window is symmetric so resyncing recovers automatically. |
| Crash mid-mission | unacked rows in `queue.db` | Restart: `replayPending` re-injects them in receive order. |
| `queue.db` directory missing | ctor throws, node fails to start | Create the directory (`~/.ros/vehicle_agent/`) or override `queue_db_path`. |
| Scenario YAML missing | loopback adapter logs and runs nothing | Pass `vehicle_agent_scenario_path:=…`; the launch file does not gate on this. |
| paho-cpp build missing from workspace | `colcon build --packages-select vehicle_agent` fails at link time | Build the vendored libs first: `colcon build --packages-select paho-mqtt-c paho-mqtt-cpp` (sources at `src/others/paho.mqtt.{c,cpp}`). |
| Broker unreachable at startup | `connected()` stays false; `start()` does NOT throw | Vehicle agent comes up; paho's `automatic_reconnect` retries between `min_retry_sec`/`max_retry_sec`. Diagnostic stays WARN until the broker is back. |

E-stop latency monitoring: assert `EstopChannel::lastTriggerLatencyMicros()`
in test rigs. The budget is 50 ms p95; current measured median on
the dual-multiplex test is in the sub-millisecond range.

---

## 12. Deferred / open work

Tracked items from Sprint 6's exit checklist:

1. ~~**`IrMission → FMSRouteWindow` bridge node.**~~ — **DONE**.
   `RouteBridgeNode` (`va_route_bridge` executable) maps IR topics to
   the legacy FMS shapes `igv_path_converter` consumes, and owns the
   ego-centric route windowing that keeps downstream liveness
   decoupled from FMS push cadence. See §4.12 and
   `doc/plan-rviz-va-integration.md` §2.
2. ~~**Real paho-cpp `IMqttBus` impl.**~~ — **DONE**. Vendored paho
   restored to `src/others/paho.mqtt.{c,cpp}` (built via colcon as
   plain CMake packages). `PahoMqttBus` (`paho_bus.{hpp,cpp}`)
   implements `IMqttBus` over `mqtt::async_client` with MQTT 5 + TLS
   and `MtlsConfig`-driven `ssl_options`. The production constructor
   `InternalZpmcAdapter(Config)` now builds it from
   `Config::{broker_host, broker_port, mtls}`.
3. **AHTFMS-NS uncovered message types.** RestrictedArea / Vessel
   / QC / YC / Container / Config notifications (3004–3010) decode
   today but are discarded.
4. **Trajectory uplink.** ICD message 4003 VEHICLE_TRAJECTORY_STATUS
   is not emitted yet.
5. **Live two-terminal smoke test.** Per Sprint 6 verification,
   running `./launch_fms_test_components.sh` + `./launch_with_fms_real_vehicle.sh`
   and confirming `/vehicle_agent/mission` populates plus the IGV
   moves in RViz is the operator-side gate.

---

## 13. References

- `docs/vehicle_agent/Vehicle_Agent_Design_v1.2.docx` — high-level
  v1.2 design (sections referenced throughout this doc, e.g. §7 for
  the state machine).
- `docs/vehicle_agent/AHTFMS-NS ICD_v1.1.1_draft_1124.pdf` —
  wire-format source of truth for the AHTFMS-NS adapter.
- `src/igv/vehicle_agent/README.md` — user-facing overview.
- `src/igv/igv_path_planner/README.md` — downstream consumer of
  VA's IR topics (post-`IrMission`-bridge work).
- `/root/.claude/plans/virtual-hatching-nygaard.md` — the v1.2
  implementation plan that drove Sprints 1–6.
