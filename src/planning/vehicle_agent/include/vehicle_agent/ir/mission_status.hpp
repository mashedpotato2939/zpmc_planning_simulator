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

#ifndef VEHICLE_AGENT__IR__MISSION_STATUS_HPP_
#define VEHICLE_AGENT__IR__MISSION_STATUS_HPP_

#include <nlohmann/json.hpp>
#include <cstdint>
#include <string>

namespace vehicle_agent::ir
{

// 9-state machine (v1.2 §7)
enum class MissionState : uint8_t
{
  IDLE = 0,
  RECEIVED = 1,
  VALIDATING = 2,
  EXECUTING = 3,
  PAUSED = 4,
  ABORTING = 5,
  PREEMPTED = 6,
  DONE = 7,
  FAILED = 8,
};

inline const char * to_string(MissionState s)
{
  switch (s) {
    case MissionState::IDLE: return "IDLE";
    case MissionState::RECEIVED: return "RECEIVED";
    case MissionState::VALIDATING: return "VALIDATING";
    case MissionState::EXECUTING: return "EXECUTING";
    case MissionState::PAUSED: return "PAUSED";
    case MissionState::ABORTING: return "ABORTING";
    case MissionState::PREEMPTED: return "PREEMPTED";
    case MissionState::DONE: return "DONE";
    case MissionState::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

struct MissionStatus
{
  std::string mission_id;
  MissionState state{MissionState::IDLE};
  uint64_t stamp_ms{0};
  double progress{0.0};   // 0..1
  std::string detail;
};

inline void to_json(nlohmann::json & j, const MissionStatus & s)
{
  j = {{"mission_id", s.mission_id}, {"state", static_cast<uint8_t>(s.state)},
       {"stamp_ms", s.stamp_ms}, {"progress", s.progress}, {"detail", s.detail}};
}

inline void from_json(const nlohmann::json & j, MissionStatus & s)
{
  s.mission_id = j.value("mission_id", "");
  s.state = static_cast<MissionState>(j.value("state", 0));
  s.stamp_ms = j.value("stamp_ms", 0ULL);
  s.progress = j.value("progress", 0.0);
  s.detail = j.value("detail", "");
}

}  // namespace vehicle_agent::ir
#endif  // VEHICLE_AGENT__IR__MISSION_STATUS_HPP_
