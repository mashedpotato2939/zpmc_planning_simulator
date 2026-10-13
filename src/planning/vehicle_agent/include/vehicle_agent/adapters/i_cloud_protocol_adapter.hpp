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

#ifndef VEHICLE_AGENT__ADAPTERS__I_CLOUD_PROTOCOL_ADAPTER_HPP_
#define VEHICLE_AGENT__ADAPTERS__I_CLOUD_PROTOCOL_ADAPTER_HPP_

#include "vehicle_agent/ir/ack.hpp"
#include "vehicle_agent/ir/command.hpp"
#include "vehicle_agent/ir/mission.hpp"
#include "vehicle_agent/ir/mission_status.hpp"
#include "vehicle_agent/ir/telemetry.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace vehicle_agent
{

struct AdapterCapabilities
{
  bool supports_mission{true};
  bool supports_command{true};
  bool supports_telemetry_uplink{true};
  bool supports_ack{true};
  bool supports_estop{false};
  uint32_t priority{100};   // higher = preferred as primary
};

struct AdapterCallbacks
{
  std::function<void(ir::Mission)> on_mission;
  std::function<void(ir::Command)> on_command;
  // Synthetic execution-complete hook. In Sprint 1 the loopback adapter
  // emits this; in Sprint 2 the ROS subscriber will emit it from the
  // planning/control side. Keeping it on AdapterCallbacks avoids a
  // separate plumbing path for tests.
  std::function<void(std::string /*mission_id*/)> on_execution_complete;
  // Fired when an adapter's connection state flips. On true->false, queued
  // uplink should be paused; on false->true, the node drains pending offline
  // queue rows to this adapter. Sprint-2 stubbed: loopback adapter is always
  // connected, so this only fires once on start.
  std::function<void(std::string /*adapter_name*/, bool /*connected*/)>
    on_connection_state_changed;
};

class ICloudProtocolAdapter
{
public:
  virtual ~ICloudProtocolAdapter() = default;

  virtual std::string name() const = 0;
  virtual AdapterCapabilities capabilities() const = 0;

  // Lifecycle. start() may spawn threads, open sockets, load files.
  virtual void start(AdapterCallbacks callbacks) = 0;
  virtual void stop() = 0;

  // Transport health. Default true (in-process adapters like loopback are
  // always "connected"). Real network adapters override.
  virtual bool connected() const { return true; }

  // Uplink (called by app layer).
  virtual void publishTelemetry(const ir::Telemetry &) {}
  virtual void publishMissionStatus(const ir::MissionStatus &) {}
  virtual void publishAck(const ir::Ack &) {}
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__ADAPTERS__I_CLOUD_PROTOCOL_ADAPTER_HPP_
