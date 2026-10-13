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

#ifndef VEHICLE_AGENT__APP__ESTOP_CHANNEL_HPP_
#define VEHICLE_AGENT__APP__ESTOP_CHANNEL_HPP_

#include "vehicle_agent/app/mission_state_machine.hpp"
#include "vehicle_agent/ir/command.hpp"

#include <atomic>
#include <cstdint>

namespace vehicle_agent
{

// Fast-path E-stop. Bypasses EventBus so a flooded queue can't delay the
// stop. Calls MissionStateMachine::handle directly; the FSM mutex serializes
// the transition against the consume thread (a single transition is O(us)).
//
// Latency budget: p95 < 50 ms end-to-end (from trigger() entry to FSM
// emitting the FAILED status). Measured by lastTriggerLatencyMicros().
class EstopChannel
{
public:
  explicit EstopChannel(MissionStateMachine & fsm);

  // Synchronously route the ESTOP command into the FSM. Safe from any thread.
  // Other CommandKinds are accepted but treated as no-ops here (use the bus).
  void trigger(const ir::Command & c);

  // Whether the most recent trigger() left the FSM in a stop state.
  bool active() const { return active_.load(); }

  // Microseconds spent inside the most recent trigger() call. Useful for
  // latency assertions in tests.
  uint64_t lastTriggerLatencyMicros() const { return last_latency_us_.load(); }

  // Total trigger() invocations.
  uint64_t triggerCount() const { return trigger_count_.load(); }

private:
  MissionStateMachine & fsm_;
  std::atomic<bool> active_{false};
  std::atomic<uint64_t> last_latency_us_{0};
  std::atomic<uint64_t> trigger_count_{0};
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__APP__ESTOP_CHANNEL_HPP_
