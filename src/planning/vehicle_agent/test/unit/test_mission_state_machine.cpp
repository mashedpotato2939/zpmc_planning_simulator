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

#include "vehicle_agent/app/mission_state_machine.hpp"

#include <gtest/gtest.h>

#include <vector>

using namespace vehicle_agent;
using ir::MissionState;

namespace
{
struct Recorder
{
  std::vector<MissionState> states;
  std::vector<ir::Ack> acks;
};

MissionStateMachine make(Recorder & rec)
{
  MissionStateMachine::Sinks s;
  s.on_status = [&](const ir::MissionStatus & st) { rec.states.push_back(st.state); };
  s.on_ack = [&](const ir::Ack & a) { rec.acks.push_back(a); };
  return MissionStateMachine(std::move(s));
}

ir::Event mission_event(const std::string & id)
{
  ir::Mission m;
  m.id = id;
  return ir::Event{ir::EventKind::MISSION_RECEIVED, m};
}

ir::Event mission_event_with_waypoints(
  const std::string & id, std::size_t n, double base_x = 0.0)
{
  ir::Mission m;
  m.id = id;
  for (std::size_t i = 0; i < n; ++i) {
    ir::Waypoint w;
    w.x = base_x + static_cast<double>(i);
    w.y = 0.0;
    m.waypoints.push_back(w);
  }
  return ir::Event{ir::EventKind::MISSION_RECEIVED, m};
}

ir::Event command_event(const std::string & id, ir::CommandKind k)
{
  ir::Command c;
  c.mission_id = id;
  c.kind = k;
  return ir::Event{ir::EventKind::COMMAND_RECEIVED, c};
}

ir::Event complete_event(const std::string & id)
{
  ir::ExecutionComplete e{id};
  return ir::Event{ir::EventKind::EXECUTION_COMPLETE, e};
}
}  // namespace

TEST(MissionStateMachine, HappyPathIdleToDone)
{
  Recorder r;
  auto fsm = make(r);

  fsm.handle(mission_event("m1"));
  // On receive: RECEIVED -> VALIDATING -> EXECUTING
  ASSERT_GE(r.states.size(), 3u);
  EXPECT_EQ(r.states[0], MissionState::RECEIVED);
  EXPECT_EQ(r.states[1], MissionState::VALIDATING);
  EXPECT_EQ(r.states[2], MissionState::EXECUTING);
  EXPECT_EQ(fsm.state(), MissionState::EXECUTING);

  // One OK ack should have fired.
  ASSERT_EQ(r.acks.size(), 1u);
  EXPECT_EQ(r.acks[0].target_id, "m1");
  EXPECT_EQ(r.acks[0].status, ir::AckStatus::OK);

  fsm.handle(complete_event("m1"));
  EXPECT_EQ(fsm.state(), MissionState::DONE);
}

TEST(MissionStateMachine, PauseResume)
{
  Recorder r;
  auto fsm = make(r);
  fsm.handle(mission_event("m1"));
  fsm.handle(command_event("m1", ir::CommandKind::PAUSE));
  EXPECT_EQ(fsm.state(), MissionState::PAUSED);
  fsm.handle(command_event("m1", ir::CommandKind::RESUME));
  EXPECT_EQ(fsm.state(), MissionState::EXECUTING);
}

TEST(MissionStateMachine, EstopFromExecutingGoesToFailed)
{
  Recorder r;
  auto fsm = make(r);
  fsm.handle(mission_event("m1"));
  fsm.handle(command_event("m1", ir::CommandKind::ESTOP));
  EXPECT_EQ(fsm.state(), MissionState::FAILED);
  // Two acks: OK on accept, FAILED on estop.
  ASSERT_EQ(r.acks.size(), 2u);
  EXPECT_EQ(r.acks[1].status, ir::AckStatus::FAILED);
}

TEST(MissionStateMachine, PreemptionPromotesNewMission)
{
  Recorder r;
  auto fsm = make(r);
  fsm.handle(mission_event("m1"));
  fsm.handle(mission_event("m2"));
  // After preemption: m2 must be active and executing.
  EXPECT_EQ(fsm.state(), MissionState::EXECUTING);
  auto active = fsm.activeMission();
  ASSERT_TRUE(active.has_value());
  EXPECT_EQ(active->id, "m2");

  // Acks: OK(m1), PREEMPTED(m1), OK(m2).
  ASSERT_EQ(r.acks.size(), 3u);
  EXPECT_EQ(r.acks[0].target_id, "m1");
  EXPECT_EQ(r.acks[0].status, ir::AckStatus::OK);
  EXPECT_EQ(r.acks[1].target_id, "m1");
  EXPECT_EQ(r.acks[1].status, ir::AckStatus::PREEMPTED);
  EXPECT_EQ(r.acks[2].target_id, "m2");
  EXPECT_EQ(r.acks[2].status, ir::AckStatus::OK);
}

TEST(MissionStateMachine, SameIdRefreshUpdatesWaypointsWithoutTransition)
{
  Recorder r;
  auto fsm = make(r);
  fsm.handle(mission_event_with_waypoints("m1", 3, 0.0));
  ASSERT_EQ(fsm.state(), MissionState::EXECUTING);
  const auto status_count_before = r.states.size();
  ASSERT_EQ(r.acks.size(), 1u);  // initial OK accept

  // Same id with a different waypoint set — should be a silent data-plane
  // refresh: no state transition, no ack, but active waypoints update.
  fsm.handle(mission_event_with_waypoints("m1", 5, 100.0));

  EXPECT_EQ(fsm.state(), MissionState::EXECUTING);
  EXPECT_EQ(r.states.size(), status_count_before);
  EXPECT_EQ(r.acks.size(), 1u);

  auto active = fsm.activeMission();
  ASSERT_TRUE(active.has_value());
  ASSERT_EQ(active->waypoints.size(), 5u);
  EXPECT_DOUBLE_EQ(active->waypoints.front().x, 100.0);
  EXPECT_DOUBLE_EQ(active->waypoints.back().x, 104.0);
}

TEST(MissionStateMachine, SameIdRefreshDuringPausedKeepsPausedState)
{
  Recorder r;
  auto fsm = make(r);
  fsm.handle(mission_event_with_waypoints("m1", 3, 0.0));
  fsm.handle(command_event("m1", ir::CommandKind::PAUSE));
  ASSERT_EQ(fsm.state(), MissionState::PAUSED);
  const auto acks_before = r.acks.size();

  fsm.handle(mission_event_with_waypoints("m1", 4, 10.0));

  EXPECT_EQ(fsm.state(), MissionState::PAUSED);
  EXPECT_EQ(r.acks.size(), acks_before);
  auto active = fsm.activeMission();
  ASSERT_TRUE(active.has_value());
  EXPECT_EQ(active->waypoints.size(), 4u);
}

TEST(MissionStateMachine, ClearFaultReturnsToIdle)
{
  Recorder r;
  auto fsm = make(r);
  fsm.handle(mission_event("m1"));
  fsm.handle(command_event("m1", ir::CommandKind::ABORT));
  ASSERT_EQ(fsm.state(), MissionState::FAILED);
  fsm.handle(command_event("m1", ir::CommandKind::CLEAR_FAULT));
  EXPECT_EQ(fsm.state(), MissionState::IDLE);
  EXPECT_FALSE(fsm.activeMission().has_value());
}
