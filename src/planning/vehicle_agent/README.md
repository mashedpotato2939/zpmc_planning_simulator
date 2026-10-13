# vehicle_agent

Edge middleware between cloud fleet-management systems and the on-vehicle
ROS 2 Autoware stack. Implements the v1.2 architecture in
`docs/vehicle_agent/Vehicle_Agent_Design_v1.2.docx`.

## Responsibilities

| Layer | Module |
|---|---|
| Cloud protocol adapters | `ahtfms_ns`, `internal_zpmc`, `scenario_loopback` |
| Mission state machine | `app/mission_state_machine.cpp` (9 states) |
| Security envelope | `security/{hmac_signer,nonce_lru,clock_skew_window,mtls_context}.cpp` |
| Durable offline queue | `storage/offline_queue.cpp` (SQLite WAL) |
| ROS bridge | `ros_bridge/{ros_publisher,ros_subscriber,diagnostic_publisher}.cpp` |
| E-stop fast path | `app/estop_channel.cpp` (bypasses the event bus) |

## Adapters

| Name | Wire protocol | Status | When to use |
|---|---|---|---|
| `ahtfms_ns` | TCP, ICD v1.1.1 framing (FMS1 magic, CM header, msg header) | Production | Default — paired with the AHTFMS-NS CM gateway |
| `internal_zpmc` | MQTT 5 + HMAC-SHA256 envelope + nonce LRU + mTLS | Test-only until paho-cpp lands | ZPMC internal fleet broker |
| `scenario_loopback` | YAML scenario file | Production (for sim) | Standalone simulation; no broker required |

Adapters share `ICloudProtocolAdapter`. The `AdapterMultiplexer` mirrors
uplink (telemetry/status/ack) to every adapter that advertises support;
downlink (mission/command) flows from any adapter to the FSM.
`connectedPrimaryName()` is the failover-aware primary picker.

## Launch

```bash
# Standalone simulation (loopback adapter):
ros2 launch vehicle_agent vehicle_agent_sim.launch.py \
  scenario:=$(ros2 pkg prefix vehicle_agent)/share/vehicle_agent/scenarios/basic_pickup.yaml

# Production — chosen by complete_autoware_system_fms.launch.xml, e.g.:
ros2 launch vehicle_agent vehicle_agent.launch.py \
  adapters:=ahtfms_ns vehicle_id:=IGV2001 \
  ahtfms_ns.broker_host:=10.0.0.42 ahtfms_ns.broker_port:=8801

# Concurrent adapters (Sprint 5 dual-multiplex):
ros2 launch vehicle_agent vehicle_agent.launch.py \
  adapters:="ahtfms_ns,scenario_loopback" \
  vehicle_agent_scenario_path:=/path/to/scenario.yaml
```

## Topics

Published (default namespace `/vehicle_agent`):

| Topic | Type (in `igv_planning_msgs/msg/`) | Notes |
|---|---|---|
| `~/mission` | `IrMission` | Downlink mission |
| `~/command` | `IrCommand` | Downlink command (PAUSE/RESUME/ABORT/ESTOP/CLEAR_FAULT) |
| `~/mission_status` | `IrMissionStatus` | FSM state transitions; transient_local |

Subscribed:

| Topic | Type | Notes |
|---|---|---|
| `/localization/kinematic_state` | `nav_msgs/Odometry` | feeds `ir::Telemetry` uplink |

Diagnostics published on `/diagnostics` (per-adapter
connected/queue-depth/last-error).

## How it fits with `igv_path_planner`

`igv_path_converter` (in `igv_path_planner`) still owns the path-
to-trajectory conversion. VA produces `IrMission` on `/vehicle_agent/mission`;
the legacy `FMSRouteWindow` / `FMSStopPoint` topics that the converter
subscribes to today are not yet emitted by VA. An IR→`FMSRouteWindow`
bridge node is the next integration step — until it lands, the legacy
`fms_protocol` plugin keeps producing those topics from whatever the
operator/FMS publishes upstream.

## Tests

```bash
colcon test --packages-select vehicle_agent
colcon test-result --test-result-base build/vehicle_agent --verbose
```

16 test binaries currently green. Coverage includes the 9-state machine,
offline-queue durability, E-stop fast path, AHTFMS-NS codec/framing
(20 cases + 9 integration), internal_zpmc multiplex+failover (5 cases).

## Extending

To add a new cloud adapter:

1. Implement `ICloudProtocolAdapter` under `src/adapters/<name>/`.
2. Register a `<name>` branch in `vehicle_agent_node.cpp`'s adapter loop.
3. Add `config/adapters/<name>.yaml` with the broker / topic / security
   knobs documented inline.
4. Add a fixture-driven unit test under `test/unit/`.

Each adapter advertises an `AdapterCapabilities::priority`. The multiplexer
selects the highest-priority *connected* adapter as primary, so failover
to a lower-priority adapter is automatic when the preferred link drops.
