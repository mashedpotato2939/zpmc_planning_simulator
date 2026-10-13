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

#include "vehicle_agent/ir/mission_status.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <iostream>
#include <thread>
#include <utility>

namespace vehicle_agent
{

namespace
{
uint64_t now_ms()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}
}  // namespace

ScenarioLoopbackAdapter::ScenarioLoopbackAdapter(std::string scenario_path)
: scenario_path_(std::move(scenario_path)) {}

ScenarioLoopbackAdapter::~ScenarioLoopbackAdapter()
{
  stop();
}

AdapterCapabilities ScenarioLoopbackAdapter::capabilities() const
{
  AdapterCapabilities c;
  c.supports_estop = true;
  c.priority = 10;   // low — real adapters take precedence when present
  return c;
}

void ScenarioLoopbackAdapter::start(AdapterCallbacks callbacks)
{
  stop_ = false;
  if (callbacks.on_connection_state_changed) {
    callbacks.on_connection_state_changed(name(), true);
  }
  worker_ = std::thread([this, cb = std::move(callbacks)]() mutable { run(cb); });
}

void ScenarioLoopbackAdapter::stop()
{
  stop_ = true;
  if (worker_.joinable()) worker_.join();
}

void ScenarioLoopbackAdapter::publishMissionStatus(const ir::MissionStatus & s)
{
  std::cout << "[scenario_loopback] status mission=" << s.mission_id
            << " state=" << ir::to_string(s.state)
            << " progress=" << s.progress
            << " detail=\"" << s.detail << "\"\n";
}

void ScenarioLoopbackAdapter::publishAck(const ir::Ack & a)
{
  std::cout << "[scenario_loopback] ack target=" << a.target_id
            << " status=" << static_cast<int>(a.status)
            << " detail=\"" << a.detail << "\"\n";
}

void ScenarioLoopbackAdapter::run(AdapterCallbacks cb)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(scenario_path_);
  } catch (const std::exception & e) {
    std::cerr << "[scenario_loopback] failed to load " << scenario_path_ << ": " << e.what() << "\n";
    return;
  }

  const auto steps = root["steps"];
  if (!steps || !steps.IsSequence()) {
    std::cerr << "[scenario_loopback] scenario must contain a `steps:` sequence\n";
    return;
  }

  for (const auto & step : steps) {
    if (stop_) return;
    const int delay_ms = step["delay_ms"] ? step["delay_ms"].as<int>() : 0;
    if (delay_ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    if (stop_) return;

    const std::string type = step["type"] ? step["type"].as<std::string>() : "";
    if (type == "mission") {
      ir::Mission m;
      m.id = step["id"] ? step["id"].as<std::string>() : "loopback-mission";
      m.seq = step["seq"] ? step["seq"].as<uint64_t>() : 1;
      m.issued_at_ms = now_ms();
      m.source_adapter = name();
      if (step["waypoints"] && step["waypoints"].IsSequence()) {
        for (const auto & w : step["waypoints"]) {
          ir::Waypoint wp;
          wp.x = w["x"] ? w["x"].as<double>() : 0.0;
          wp.y = w["y"] ? w["y"].as<double>() : 0.0;
          wp.z = w["z"] ? w["z"].as<double>() : 0.0;
          wp.yaw = w["yaw"] ? w["yaw"].as<double>() : 0.0;
          wp.target_speed = w["target_speed"] ? w["target_speed"].as<double>() : 0.0;
          wp.segment_id = w["segment_id"] ? w["segment_id"].as<std::string>() : "";
          wp.point_type = w["point_type"]
            ? static_cast<uint8_t>(w["point_type"].as<int>()) : 0;
          m.waypoints.push_back(wp);
        }
      }
      if (cb.on_mission) cb.on_mission(std::move(m));
      injected_++;
    } else if (type == "command") {
      ir::Command c;
      c.mission_id = step["mission_id"] ? step["mission_id"].as<std::string>() : "";
      const std::string kind = step["kind"] ? step["kind"].as<std::string>() : "PAUSE";
      if (kind == "PAUSE") c.kind = ir::CommandKind::PAUSE;
      else if (kind == "RESUME") c.kind = ir::CommandKind::RESUME;
      else if (kind == "ABORT") c.kind = ir::CommandKind::ABORT;
      else if (kind == "ESTOP") c.kind = ir::CommandKind::ESTOP;
      else if (kind == "CLEAR_FAULT") c.kind = ir::CommandKind::CLEAR_FAULT;
      c.issued_at_ms = now_ms();
      c.source_adapter = name();
      if (cb.on_command) cb.on_command(std::move(c));
      injected_++;
    } else if (type == "complete") {
      const std::string mission_id =
        step["mission_id"] ? step["mission_id"].as<std::string>() : "";
      if (cb.on_execution_complete) cb.on_execution_complete(mission_id);
      injected_++;
    }
  }
}

}  // namespace vehicle_agent
