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

#ifndef VEHICLE_AGENT__ADAPTERS__ADAPTER_MULTIPLEXER_HPP_
#define VEHICLE_AGENT__ADAPTERS__ADAPTER_MULTIPLEXER_HPP_

#include "vehicle_agent/adapters/i_cloud_protocol_adapter.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace vehicle_agent
{

// Owns a set of adapters and applies a uniform routing policy:
//
//  - Downlink (mission/command): unioned via AdapterCallbacks. Any adapter
//    may deliver an inbound IR object; the multiplexer does not dedup —
//    conflict resolution is the FSM's job (e.g. dropping a stale mission
//    that the FSM already has under execution).
//
//  - Uplink (telemetry/status/ack): mirrored to every adapter that
//    advertises support for the respective capability. We deliberately
//    do not gate mirroring on `connected()` — the adapters are responsible
//    for queuing their own offline backlog (the OfflineQueue holds the
//    canonical IR copy, so a slow adapter cannot lose state).
//
//  - Primary selection: `primaryName()` returns the highest-priority
//    adapter known. `connectedPrimaryName()` is the failover-aware form:
//    it returns the highest-priority adapter that is *currently* connected
//    (falling back to a lower-priority connected adapter when the preferred
//    primary is down). This is what diagnostics should display to the user.
class AdapterMultiplexer
{
public:
  AdapterMultiplexer() = default;
  AdapterMultiplexer(const AdapterMultiplexer &) = delete;
  AdapterMultiplexer & operator=(const AdapterMultiplexer &) = delete;

  void addAdapter(std::unique_ptr<ICloudProtocolAdapter> adapter);
  void startAll(AdapterCallbacks callbacks);
  void stopAll();

  void publishTelemetry(const ir::Telemetry & t);
  void publishMissionStatus(const ir::MissionStatus & s);
  void publishAck(const ir::Ack & a);

  // Highest-priority adapter known (regardless of connection state).
  std::string primaryName() const;

  // Highest-priority *connected* adapter; falls back when the preferred
  // primary is down. Empty if no adapter is currently connected.
  std::string connectedPrimaryName() const;

  size_t size() const;

private:
  mutable std::mutex mtx_;
  std::vector<std::unique_ptr<ICloudProtocolAdapter>> adapters_;
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__ADAPTERS__ADAPTER_MULTIPLEXER_HPP_
