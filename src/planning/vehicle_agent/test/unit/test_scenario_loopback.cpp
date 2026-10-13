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

#include "vehicle_agent/adapters/scenario_loopback_adapter.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

using vehicle_agent::AdapterCallbacks;
using vehicle_agent::ScenarioLoopbackAdapter;

namespace
{
std::string write_temp(const std::string & body)
{
  char tmpl[] = "/tmp/va_scn_XXXXXX";
  int fd = ::mkstemp(tmpl);
  EXPECT_GE(fd, 0);
  ::close(fd);
  std::string path = std::string(tmpl) + ".yaml";
  std::ofstream f(path);
  f << body;
  return path;
}
}  // namespace

TEST(ScenarioLoopback, EmitsMissionsAndCommandsInOrder)
{
  const std::string yaml = R"(
steps:
  - delay_ms: 0
    type: mission
    id: scn-1
    waypoints:
      - { x: 1.0, y: 2.0, target_speed: 1.5 }
  - delay_ms: 0
    type: command
    mission_id: scn-1
    kind: PAUSE
  - delay_ms: 0
    type: command
    mission_id: scn-1
    kind: RESUME
)";
  const auto path = write_temp(yaml);

  std::mutex mtx;
  std::vector<std::string> events;
  AdapterCallbacks cb;
  cb.on_mission = [&](vehicle_agent::ir::Mission m) {
    std::lock_guard<std::mutex> lock(mtx);
    events.push_back("mission:" + m.id);
  };
  cb.on_command = [&](vehicle_agent::ir::Command c) {
    std::lock_guard<std::mutex> lock(mtx);
    events.push_back("command:" + std::to_string(static_cast<int>(c.kind)));
  };

  ScenarioLoopbackAdapter adapter(path);
  adapter.start(cb);

  // Wait up to 2s for all 3 events.
  for (int i = 0; i < 200; ++i) {
    {
      std::lock_guard<std::mutex> lock(mtx);
      if (events.size() >= 3u) break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  adapter.stop();

  ASSERT_EQ(events.size(), 3u);
  EXPECT_EQ(events[0], "mission:scn-1");
  EXPECT_EQ(events[1], "command:0");   // PAUSE
  EXPECT_EQ(events[2], "command:1");   // RESUME
  EXPECT_EQ(adapter.injectedCount(), 3u);
}

TEST(ScenarioLoopback, MissingFileDoesNotCrash)
{
  ScenarioLoopbackAdapter adapter("/nonexistent/scenario.yaml");
  AdapterCallbacks cb;
  adapter.start(cb);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  adapter.stop();
  EXPECT_EQ(adapter.injectedCount(), 0u);
}
