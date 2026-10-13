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

#include "vehicle_agent/app/estop_channel.hpp"

#include "vehicle_agent/ir/event.hpp"

#include <chrono>

namespace vehicle_agent
{

EstopChannel::EstopChannel(MissionStateMachine & fsm) : fsm_(fsm) {}

void EstopChannel::trigger(const ir::Command & c)
{
  using clock = std::chrono::steady_clock;
  const auto t0 = clock::now();

  ir::Event ev{ir::EventKind::COMMAND_RECEIVED, c, 0};
  fsm_.handle(ev);

  const auto t1 = clock::now();
  last_latency_us_.store(
    static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count()));
  trigger_count_.fetch_add(1);
  if (c.kind == ir::CommandKind::ESTOP) {
    active_.store(fsm_.state() == ir::MissionState::FAILED ||
                  fsm_.state() == ir::MissionState::ABORTING);
  }
}

}  // namespace vehicle_agent
