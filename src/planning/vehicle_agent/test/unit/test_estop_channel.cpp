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
#include "vehicle_agent/app/mission_state_machine.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

using vehicle_agent::EstopChannel;
using vehicle_agent::MissionStateMachine;
namespace ir = vehicle_agent::ir;

namespace
{
MissionStateMachine make_fsm(std::vector<ir::MissionState> * states)
{
  MissionStateMachine::Sinks s;
  s.on_status = [states](const ir::MissionStatus & st) {
    if (states) states->push_back(st.state);
  };
  return MissionStateMachine(std::move(s));
}

ir::Event mission(const std::string & id)
{
  ir::Mission m;
  m.id = id;
  return ir::Event{ir::EventKind::MISSION_RECEIVED, m};
}

ir::Command estop_cmd(const std::string & mid)
{
  ir::Command c;
  c.mission_id = mid;
  c.kind = ir::CommandKind::ESTOP;
  return c;
}
}  // namespace

TEST(EstopChannel, TriggerDrivesFsmToFailed)
{
  std::vector<ir::MissionState> states;
  auto fsm = make_fsm(&states);
  EstopChannel chan(fsm);

  fsm.handle(mission("m1"));
  ASSERT_EQ(fsm.state(), ir::MissionState::EXECUTING);

  EXPECT_FALSE(chan.active());
  EXPECT_EQ(chan.triggerCount(), 0u);

  chan.trigger(estop_cmd("m1"));
  EXPECT_EQ(fsm.state(), ir::MissionState::FAILED);
  EXPECT_TRUE(chan.active());
  EXPECT_EQ(chan.triggerCount(), 1u);
}

TEST(EstopChannel, LatencyUnder50MillisecondsP95)
{
  auto fsm = make_fsm(nullptr);
  EstopChannel chan(fsm);

  // Reach EXECUTING so trigger drives a real transition.
  fsm.handle(mission("m1"));

  std::vector<uint64_t> latencies_us;
  // 20 trigger cycles. After each ESTOP the FSM is in FAILED, so we use
  // CLEAR_FAULT to return to IDLE and re-accept a mission. We measure ESTOP
  // latency only.
  for (int i = 0; i < 20; ++i) {
    chan.trigger(estop_cmd("m1"));
    latencies_us.push_back(chan.lastTriggerLatencyMicros());

    ir::Command clear;
    clear.mission_id = "m1";
    clear.kind = ir::CommandKind::CLEAR_FAULT;
    fsm.handle(ir::Event{ir::EventKind::COMMAND_RECEIVED, clear});
    fsm.handle(mission("m1"));
  }

  std::sort(latencies_us.begin(), latencies_us.end());
  const auto p95 = latencies_us[(latencies_us.size() * 95) / 100];
  EXPECT_LT(p95, 50'000u) << "ESTOP p95 latency exceeded 50ms: " << p95 << "us";
}

TEST(EstopChannel, NonEstopCommandLeavesActiveUnchanged)
{
  auto fsm = make_fsm(nullptr);
  EstopChannel chan(fsm);
  fsm.handle(mission("m1"));

  ir::Command pause;
  pause.mission_id = "m1";
  pause.kind = ir::CommandKind::PAUSE;
  chan.trigger(pause);

  // Trigger count still increments (every trigger counts), but `active` only
  // tracks ESTOP outcomes.
  EXPECT_EQ(chan.triggerCount(), 1u);
  EXPECT_FALSE(chan.active());
}
