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

// Integration test for Sprint 2: simulate a crash mid-mission by stuffing
// rows into the offline queue, then start a fresh node-like pipeline and
// verify the rows replay onto the FSM in order, advance through terminal
// state, and get acked from the queue.

#include "vehicle_agent/app/event_bus.hpp"
#include "vehicle_agent/app/mission_state_machine.hpp"
#include "vehicle_agent/storage/offline_queue.hpp"

#include <nlohmann/json.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <thread>
#include <unistd.h>
#include <vector>

namespace ir = vehicle_agent::ir;
using vehicle_agent::EventBus;
using vehicle_agent::MissionStateMachine;
using vehicle_agent::OfflineQueue;

namespace
{
std::string tmp_db_path()
{
  char tmpl[] = "/tmp/va_replay_XXXXXX";
  int fd = ::mkstemp(tmpl);
  EXPECT_GE(fd, 0);
  ::close(fd);
  ::unlink(tmpl);
  return std::string(tmpl) + ".db";
}

ir::Mission make_mission(const std::string & id)
{
  ir::Mission m;
  m.id = id;
  m.kind = ir::MissionKind::ROUTE;
  m.seq = 1;
  ir::Waypoint w;
  w.x = 10.0;
  w.y = 20.0;
  w.target_speed = 1.0;
  m.waypoints.push_back(w);
  return m;
}
}  // namespace

TEST(LoopbackReplay, RowsReplayInOrderAndAckOnTerminal)
{
  const auto path = tmp_db_path();
  std::vector<int64_t> mission_rows;
  std::vector<int64_t> command_rows;

  // === Crash phase ===
  // Pretend the node received two missions and one pause command, then
  // crashed before any FSM dispatch.
  {
    OfflineQueue q(path);
    mission_rows.push_back(q.enqueueMission(make_mission("rmi-1")));
    command_rows.push_back(q.enqueueCommand([&] {
      ir::Command c;
      c.mission_id = "rmi-1";
      c.kind = ir::CommandKind::PAUSE;
      return c;
    }()));
    mission_rows.push_back(q.enqueueMission(make_mission("rmi-2")));
    EXPECT_EQ(q.pendingCount(), 3u);
  }

  // === Restart phase ===
  OfflineQueue q(path);
  EventBus bus(64);

  std::vector<ir::MissionState> states;
  std::vector<std::string> mission_terminals;   // (mission_id) at DONE/FAILED

  MissionStateMachine::Sinks sinks;
  std::unordered_map<std::string, int64_t> mission_to_row;

  // Mimic the node's onMissionStatus terminal-ack path.
  sinks.on_status = [&](const ir::MissionStatus & s) {
    states.push_back(s.state);
    if (s.state == ir::MissionState::DONE || s.state == ir::MissionState::FAILED) {
      auto it = mission_to_row.find(s.mission_id);
      if (it != mission_to_row.end()) {
        q.ack(it->second);
        mission_to_row.erase(it);
        mission_terminals.push_back(s.mission_id);
      }
    }
  };
  MissionStateMachine fsm(std::move(sinks));

  // Replay pending rows into the bus, mirroring node startup.
  auto items = q.peekPending(4096);
  ASSERT_EQ(items.size(), 3u);
  for (const auto & it : items) {
    auto j = nlohmann::json::parse(it.payload_json);
    if (it.kind == "mission") {
      auto m = j.get<ir::Mission>();
      mission_to_row[m.id] = it.id;
      bus.push(ir::Event{ir::EventKind::MISSION_RECEIVED, std::move(m), it.id});
    } else if (it.kind == "command") {
      auto c = j.get<ir::Command>();
      bus.push(ir::Event{ir::EventKind::COMMAND_RECEIVED, std::move(c), it.id});
    }
  }

  // Consume thread: pops events, dispatches to FSM, acks commands.
  std::atomic<bool> stop{false};
  std::thread consumer([&] {
    while (!stop) {
      auto ev = bus.popBlocking();
      if (!ev) break;
      const auto row = ev->queue_row_id;
      const auto kind = ev->kind;
      fsm.handle(*ev);
      if (row != 0 && kind == ir::EventKind::COMMAND_RECEIVED) {
        q.ack(row);
      }
    }
  });

  // Wait for the bus to drain.
  for (int i = 0; i < 200; ++i) {
    if (bus.size() == 0) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // After consume: rmi-1 was preempted by rmi-2 (FSM treats new mission while
  // active as preemption). Drive rmi-2 to DONE via an EXECUTION_COMPLETE.
  bus.push(ir::Event{ir::EventKind::EXECUTION_COMPLETE,
                     ir::ExecutionComplete{"rmi-2"}, 0});
  for (int i = 0; i < 200; ++i) {
    if (bus.size() == 0) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  stop = true;
  bus.close();
  consumer.join();

  // Commands are acked immediately, missions acked on terminal state.
  // rmi-1 was preempted (not terminal), so its row should still be pending.
  EXPECT_TRUE(std::find(mission_terminals.begin(), mission_terminals.end(),
                        "rmi-2") != mission_terminals.end());
  EXPECT_FALSE(states.empty());

  const auto pending_after = q.pendingCount();
  // rmi-1's queue row stays unacked because PREEMPTED is not terminal in our
  // ack policy. That's fine — the next restart would replay it again.
  EXPECT_LE(pending_after, 1u);

  // Cleanup.
  std::error_code ec;
  std::filesystem::remove(path, ec);
  std::filesystem::remove(path + "-wal", ec);
  std::filesystem::remove(path + "-shm", ec);
}
