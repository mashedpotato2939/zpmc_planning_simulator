// Copyright 2026 ZPMC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "vehicle_agent/adapters/ahtfms_ns/codec.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <exception>

namespace vehicle_agent::ahtfms_ns
{

namespace
{
using nlohmann::json;

std::vector<uint8_t> jsonToBytes(const json & j)
{
  // ICD payload is opaque to the framing layer; we serialize as compact UTF-8
  // JSON. dump() throws on non-UTF8, which is fine — that's a programming bug.
  const auto s = j.dump();
  return std::vector<uint8_t>(s.begin(), s.end());
}

std::optional<json> bytesToJson(const std::vector<uint8_t> & bytes)
{
  try {
    return json::parse(bytes.begin(), bytes.end());
  } catch (const json::parse_error &) {
    return std::nullopt;
  }
}

uint64_t headerStamp(const json & j)
{
  if (j.contains("header") && j["header"].is_object()) {
    const auto & h = j["header"];
    if (h.contains("message_time")) {
      return h["message_time"].get<uint64_t>();
    }
  }
  return 0;
}
}  // namespace

// ---------- inbound ----------

std::optional<ir::Mission> decodeRouteNotify(const json & j)
{
  // ICD §8.63: RouteNotify { route_id, job_id, job_task_id, route_cut_time,
  //   route_segments[], destination_point, destination_stop_precision, ... }
  if (!j.contains("route_id")) return std::nullopt;
  ir::Mission m;
  m.id = j.value("route_id", "");
  m.kind = ir::MissionKind::ROUTE;
  m.issued_at_ms = headerStamp(j);
  if (m.issued_at_ms == 0) {
    // Some FMS implementations put the stamp at the top level.
    m.issued_at_ms = j.value("route_cut_time", 0ULL);
  }
  m.source_adapter = "ahtfms_ns";

  // route_segments[].points[] -> Waypoint
  // We flatten across all segments. The route's segment-level orientation
  // (FORWARD/REVERSE) is encoded into Waypoint.segment_id for now so it can be
  // recovered downstream; a richer ir::Mission can lift this into a proper
  // field in a later sprint.
  if (j.contains("route_segments") && j["route_segments"].is_array()) {
    int seg_idx = 0;
    for (const auto & seg : j["route_segments"]) {
      const int orient = seg.value("orientation", 1);
      const std::string seg_id = std::string("seg") + std::to_string(seg_idx++) +
                                 (orient == 2 ? ":rev" : ":fwd");
      if (!seg.contains("points") || !seg["points"].is_array()) continue;
      for (const auto & p : seg["points"]) {
        ir::Waypoint w;
        w.x = p.value("x", 0.0);
        w.y = p.value("y", 0.0);
        w.z = 0.0;
        w.yaw = p.value("heading", 0.0);
        w.target_speed = p.value("speed_upper_limit", 0.0);
        w.segment_id = seg_id;
        // Wire field interpreted directly in the SegmentWaypoint POINT_* enum
        // (NORMAL=0, DESTINATION=1, LC_START=2, LC_END=3, POI=4,
        // LOCAL_PLAN_START=5, LOCAL_PLAN_END=6). Producers that emit other
        // numbering must translate before publishing.
        w.point_type = static_cast<uint8_t>(p.value("point_type", 0));
        m.waypoints.push_back(w);
      }
    }
  }
  return m;
}

std::optional<ir::Mission> decodeJobNotify(const json & j)
{
  // ICD §8.58: JobNotify { job_id, job_type, job_task, ... }
  // If `job_id` is empty the spec says "the vehicle's planned path will be
  // canceled" — translate that to an ABORT mission.
  if (!j.contains("job_id")) return std::nullopt;
  ir::Mission m;
  m.id = j.value("job_id", "");
  m.issued_at_ms = headerStamp(j);
  m.source_adapter = "ahtfms_ns";
  if (m.id.empty()) {
    m.kind = ir::MissionKind::ABORT;
  } else {
    m.kind = ir::MissionKind::ROUTE;
  }
  // JobNotify by itself carries no waypoints — the route is delivered by a
  // matching RouteNotify. We still emit the Mission so the FSM can transition
  // RECEIVED → VALIDATING and update mission_context.
  return m;
}

std::optional<ir::Command> decodeStopPointNotify(const json & j)
{
  // ICD §8.64: StopPointNotify { stop_point, activate }
  // activate=true → PAUSE-at-point; activate=false → RESUME.
  ir::Command c;
  c.issued_at_ms = headerStamp(j);
  c.source_adapter = "ahtfms_ns";
  const bool activate = j.value("activate", true);
  c.kind = activate ? ir::CommandKind::PAUSE : ir::CommandKind::RESUME;
  return c;
}

std::optional<ir::Command> decodeRemoteCommand(const json & j)
{
  // ICD §8.46: RemoteCommand { command_id, command_type, oneof command {...} }
  if (!j.contains("command_id")) return std::nullopt;
  ir::Command c;
  c.mission_id = j.value("command_id", "");
  c.issued_at_ms = headerStamp(j);
  c.source_adapter = "ahtfms_ns";
  // Map CommandType to ir::CommandKind. Values match the enum order in §8.46.
  //   1 SUSPEND      → PAUSE
  //   2 RESUME       → RESUME
  //   3 RESET        → CLEAR_FAULT
  //   4 BYPASS       → CLEAR_FAULT (treated as a maintenance-style ack)
  //   5 POWER_RESET  → ABORT       (anything reseting power is terminal)
  //   6/7 modes      → PAUSE       (no IR-side handler yet; safe default)
  const int t = j.value("command_type", 0);
  switch (t) {
    case 1: c.kind = ir::CommandKind::PAUSE; break;
    case 2: c.kind = ir::CommandKind::RESUME; break;
    case 3: c.kind = ir::CommandKind::CLEAR_FAULT; break;
    case 4: c.kind = ir::CommandKind::CLEAR_FAULT; break;
    case 5: c.kind = ir::CommandKind::ABORT; break;
    default: c.kind = ir::CommandKind::PAUSE; break;
  }
  // SuspendCommand.stopping_type == EMERGENCY_BRAKE -> escalate to ESTOP.
  if (t == 1 && j.contains("suspend_command")) {
    const auto & sc = j["suspend_command"];
    if (sc.value("stopping_type", 0) == 2) {
      c.kind = ir::CommandKind::ESTOP;
    }
  }
  return c;
}

std::optional<DecodedInbound> decodePayload(MsgType msg_type,
                                            const std::vector<uint8_t> & payload)
{
  auto maybe = bytesToJson(payload);
  if (!maybe) return std::nullopt;
  const json & j = *maybe;
  switch (msg_type) {
    case MsgType::ROUTE_NOTIFY: {
      if (auto m = decodeRouteNotify(j)) return DecodedInbound{std::move(*m)};
      return std::nullopt;
    }
    case MsgType::JOB_NOTIFY: {
      if (auto m = decodeJobNotify(j)) return DecodedInbound{std::move(*m)};
      return std::nullopt;
    }
    case MsgType::STOP_POINT_NOTIFY: {
      if (auto c = decodeStopPointNotify(j)) return DecodedInbound{std::move(*c)};
      return std::nullopt;
    }
    case MsgType::REMOTE_COMMAND: {
      if (auto c = decodeRemoteCommand(j)) return DecodedInbound{std::move(*c)};
      return std::nullopt;
    }
    // Notify messages we accept and silently drop until later sprints map them.
    case MsgType::RESTRICTED_AREA_NOTIFY:
    case MsgType::VESSEL_INFO_NOTIFY:
    case MsgType::QUAY_CRANE_INFO_NOTIFY:
    case MsgType::YARD_CRANE_INFO_NOTIFY:
    case MsgType::CONTAINER_INFO_NOTIFY:
    case MsgType::CONFIG_NOTIFY:
      return DecodedInbound{DecodedNothing{}};
    default:
      return std::nullopt;
  }
}

std::optional<DecodedInbound> decode(const Frame & f, const DecodeContext & ctx)
{
  // §4.3: NS drops frames whose target_sys doesn't match its own name.
  if (!ctx.vehicle_sys.empty() && f.cm.target_sys != ctx.vehicle_sys) {
    return DecodedInbound{DecodedNothing{}};
  }
  return decodePayload(static_cast<MsgType>(f.msg.msg_type_id), f.payload);
}

// ---------- outbound ----------

std::vector<uint8_t> encodeRemoteCommandResponse(const ir::Ack & ack)
{
  // ICD §8.47: RemoteCommandResponse { header, command_id, command_state }
  // ir::AckStatus::OK -> ExecuteState 1, anything else -> 2.
  json j;
  j["header"]["message_time"] = ack.stamp_ms;
  j["command_id"] = ack.target_id;
  j["command_state"] = (ack.status == ir::AckStatus::OK) ? 1 : 2;
  if (!ack.detail.empty()) {
    j["detail"] = ack.detail;   // not in the .proto but we keep it for diagnostics
  }
  return jsonToBytes(j);
}

std::vector<uint8_t> encodeVehicleBasicStatus(const ir::Telemetry & t)
{
  // ICD §8.19 VehicleBasicStatus carries pose + speed + mission ids. The .proto
  // has many more fields; we populate the subset that ir::Telemetry actually
  // tracks today.
  json j;
  j["header"]["message_time"] = t.stamp_ms;
  j["vehicle_id"] = t.vehicle_id;
  j["position"]["x"] = t.pose.x;
  j["position"]["y"] = t.pose.y;
  j["heading"] = t.pose.yaw;
  j["speed"] = t.speed_mps;
  j["steering"] = t.steering_rad;
  j["current_mission_id"] = t.current_mission_id;
  j["mission_state"] = t.mission_state;
  j["op_mode"] = t.op_mode;
  j["e_stop_active"] = t.e_stop_active;
  return jsonToBytes(j);
}

std::vector<uint8_t> encodeVehicleTrajectoryStatus(const ir::Telemetry & t,
                                                   const std::vector<ir::Pose2D> & path)
{
  // ICD §8.22 VehicleTrajectoryStatus { header, points[] (TrajectoryPoint) }
  json j;
  j["header"]["message_time"] = t.stamp_ms;
  j["vehicle_id"] = t.vehicle_id;
  json arr = json::array();
  for (const auto & p : path) {
    arr.push_back({{"x", p.x}, {"y", p.y}, {"heading", p.yaw}});
  }
  j["points"] = std::move(arr);
  return jsonToBytes(j);
}

std::vector<uint8_t> encodeVehicleKeyEventSyncRequest(uint64_t stamp_ms)
{
  // ICD §8.8 VehicleKeyEventStatus, event_type = SYNC_REQUEST (1). Sent on
  // startup + on any internal restart so FMS re-pushes Job/Route/etc.
  json j;
  j["header"]["message_time"] = stamp_ms;
  j["event_type"] = 1;
  return jsonToBytes(j);
}

}  // namespace vehicle_agent::ahtfms_ns
