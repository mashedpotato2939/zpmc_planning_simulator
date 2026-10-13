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

#ifndef VEHICLE_AGENT__APP__MISSION_STATE_MACHINE_HPP_
#define VEHICLE_AGENT__APP__MISSION_STATE_MACHINE_HPP_

#include "vehicle_agent/ir/ack.hpp"
#include "vehicle_agent/ir/command.hpp"
#include "vehicle_agent/ir/event.hpp"
#include "vehicle_agent/ir/mission.hpp"
#include "vehicle_agent/ir/mission_status.hpp"

#include <functional>
#include <mutex>
#include <optional>
#include <string>

namespace vehicle_agent
{

// Single-writer 9-state machine. Called only from the state-machine thread.
// Transition table (v1.2 §7):
//   IDLE       -- Mission        --> RECEIVED
//   RECEIVED   -- (validate ok)  --> VALIDATING -> EXECUTING
//   RECEIVED   -- (validate bad) --> FAILED
//   EXECUTING  -- PAUSE          --> PAUSED
//   PAUSED     -- RESUME         --> EXECUTING
//   EXECUTING  -- new Mission    --> PREEMPTED -> RECEIVED (next)
//   PAUSED     -- new Mission    --> PREEMPTED -> RECEIVED (next)
//   any-active -- ABORT|ESTOP    --> ABORTING -> FAILED
//   EXECUTING  -- complete       --> DONE
//   DONE|FAILED|PREEMPTED -- (Mission cleared) -> IDLE
class MissionStateMachine
{
public:
  struct Sinks
  {
    // Called when the canonical (ir) mission status changes.
    std::function<void(const ir::MissionStatus &)> on_status;
    // Called to ack mission acceptance/rejection back to the cloud.
    std::function<void(const ir::Ack &)> on_ack;
  };

  explicit MissionStateMachine(Sinks sinks);

  void handle(const ir::Event & ev);

  ir::MissionState state() const;
  std::optional<ir::Mission> activeMission() const;

private:
  void transition(ir::MissionState next, const std::string & detail = {});
  void onMissionReceived(const ir::Mission & m);
  void onCommand(const ir::Command & c);
  void onCheckComplete(const ir::MissionCheckResult & r);
  void onExecutionComplete(const std::string & mission_id);
  void onExecutionFailed(const std::string & mission_id, const std::string & reason);

  void emitStatus(const std::string & detail);

  Sinks sinks_;
  mutable std::mutex mtx_;
  ir::MissionState state_{ir::MissionState::IDLE};
  std::optional<ir::Mission> active_;
  std::optional<ir::Mission> pending_;   // queued when active is preempted
  double progress_{0.0};
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__APP__MISSION_STATE_MACHINE_HPP_
