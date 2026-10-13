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

#ifndef VEHICLE_AGENT__ADAPTERS__SCENARIO_LOOPBACK_ADAPTER_HPP_
#define VEHICLE_AGENT__ADAPTERS__SCENARIO_LOOPBACK_ADAPTER_HPP_

#include "vehicle_agent/adapters/i_cloud_protocol_adapter.hpp"

#include <atomic>
#include <string>
#include <thread>

namespace vehicle_agent
{

// File-driven adapter: reads a YAML scenario and injects ir::Mission / ir::Command
// events at configurable delays. No network deps.
class ScenarioLoopbackAdapter : public ICloudProtocolAdapter
{
public:
  explicit ScenarioLoopbackAdapter(std::string scenario_path);
  ~ScenarioLoopbackAdapter() override;

  std::string name() const override { return "scenario_loopback"; }
  AdapterCapabilities capabilities() const override;

  void start(AdapterCallbacks callbacks) override;
  void stop() override;

  // Uplink sinks: log only (no real upstream).
  void publishTelemetry(const ir::Telemetry &) override {}
  void publishMissionStatus(const ir::MissionStatus &) override;
  void publishAck(const ir::Ack &) override;

  // Test hook: returns number of events injected so far.
  size_t injectedCount() const { return injected_.load(); }

private:
  void run(AdapterCallbacks cb);

  std::string scenario_path_;
  std::thread worker_;
  std::atomic<bool> stop_{false};
  std::atomic<size_t> injected_{0};
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__ADAPTERS__SCENARIO_LOOPBACK_ADAPTER_HPP_
