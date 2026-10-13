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

#ifndef VEHICLE_AGENT__IR__EVENT_HPP_
#define VEHICLE_AGENT__IR__EVENT_HPP_

#include "vehicle_agent/ir/command.hpp"
#include "vehicle_agent/ir/mission.hpp"
#include "vehicle_agent/ir/mission_check_result.hpp"
#include "vehicle_agent/ir/mission_status.hpp"
#include "vehicle_agent/ir/telemetry.hpp"

#include <cstdint>
#include <string>
#include <variant>

namespace vehicle_agent::ir
{

enum class EventKind : uint8_t
{
  MISSION_RECEIVED,
  COMMAND_RECEIVED,
  TELEMETRY_UPDATE,
  CHECK_COMPLETE,
  EXECUTION_PROGRESS,
  EXECUTION_COMPLETE,
  EXECUTION_FAILED,
  TICK,
};

struct ExecutionProgress
{
  std::string mission_id;
  double progress{0.0};
};

struct ExecutionComplete
{
  std::string mission_id;
};

struct ExecutionFailed
{
  std::string mission_id;
  std::string reason;
};

struct Tick {};

using EventPayload = std::variant<
  Mission, Command, Telemetry, MissionCheckResult,
  ExecutionProgress, ExecutionComplete, ExecutionFailed, Tick>;

struct Event
{
  EventKind kind;
  EventPayload payload;
  // Non-zero when this event originated from a persisted OfflineQueue row.
  // The FSM-consume loop acks the row after handle() returns.
  int64_t queue_row_id{0};
};

}  // namespace vehicle_agent::ir
#endif  // VEHICLE_AGENT__IR__EVENT_HPP_
