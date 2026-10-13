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

#ifndef VEHICLE_AGENT__ADAPTERS__AHTFMS_NS__CODEC_HPP_
#define VEHICLE_AGENT__ADAPTERS__AHTFMS_NS__CODEC_HPP_

#include "vehicle_agent/adapters/ahtfms_ns/framing.hpp"
#include "vehicle_agent/ir/ack.hpp"
#include "vehicle_agent/ir/command.hpp"
#include "vehicle_agent/ir/mission.hpp"
#include "vehicle_agent/ir/mission_status.hpp"
#include "vehicle_agent/ir/telemetry.hpp"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <variant>
#include <vector>

// Sprint 4 ICD codec.
//
// ICD §4.2 says payloads are Google Protocol Buffers. We don't have the .proto
// files compiled in tree, so this layer serializes the section-8 message
// shapes as nlohmann::json (field names match the .proto identifiers verbatim,
// e.g. `route_id`, `job_task`, `command_id`). A future sprint can swap the
// payload codec to real protobuf without touching framing or `ir::*`.
//
// The codec is direction-aware:
//   - Inbound (FMS → NS):    RouteNotify, JobNotify, StopPointNotify,
//                            RemoteCommand          → ir::Mission / ir::Command
//   - Outbound (NS → FMS):   RemoteCommandResponse  ← ir::Ack
//                            VehicleBasicStatus     ← ir::Telemetry
//                            VehicleKeyEventStatus  ← startup / re-sync
//
// Decode is a free function returning a variant — easier to dispatch than
// virtual visitor in the adapter.

namespace vehicle_agent::ahtfms_ns
{

// Decoded inbound payloads. Each is the `ir::*` projection of the ICD message.
struct DecodedNothing {};   // received but no IR mapping (e.g. Vessel info)

using DecodedInbound = std::variant<
  DecodedNothing,
  ir::Mission,        // from RouteNotify or JobNotify-with-route
  ir::Command         // from RemoteCommand or StopPointNotify
>;

struct DecodeContext
{
  // Vehicle's own system name. Frames where `target_sys` != this are dropped
  // per ICD §4.3 ("NS will ignore the message").
  std::string vehicle_sys;
};

// Encode/Decode operations.

// Inbound: parse PB payload bytes against the schema implied by msg_type.
// Returns nullopt if the JSON is malformed or msg_type is unsupported.
std::optional<DecodedInbound> decodePayload(MsgType msg_type,
                                            const std::vector<uint8_t> & payload);

// One-step: pass an entire frame through the dispatcher. Performs target_sys
// gating against ctx.vehicle_sys (drops mismatched frames as DecodedNothing).
std::optional<DecodedInbound> decode(const Frame & f, const DecodeContext & ctx);

// Outbound payload encoders. The adapter wraps each with framing.
std::vector<uint8_t> encodeRemoteCommandResponse(const ir::Ack & ack);
std::vector<uint8_t> encodeVehicleBasicStatus(const ir::Telemetry & t);
std::vector<uint8_t> encodeVehicleTrajectoryStatus(const ir::Telemetry & t,
                                                   const std::vector<ir::Pose2D> & path);
std::vector<uint8_t> encodeVehicleKeyEventSyncRequest(uint64_t stamp_ms);

// Inbound→ir helpers exposed for fine-grained tests / golden vector checks.
std::optional<ir::Mission> decodeRouteNotify(const nlohmann::json & j);
std::optional<ir::Mission> decodeJobNotify(const nlohmann::json & j);
std::optional<ir::Command> decodeStopPointNotify(const nlohmann::json & j);
std::optional<ir::Command> decodeRemoteCommand(const nlohmann::json & j);

}  // namespace vehicle_agent::ahtfms_ns
#endif  // VEHICLE_AGENT__ADAPTERS__AHTFMS_NS__CODEC_HPP_
