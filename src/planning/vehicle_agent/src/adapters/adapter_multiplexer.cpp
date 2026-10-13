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

#include "vehicle_agent/adapters/adapter_multiplexer.hpp"

#include <algorithm>
#include <utility>

namespace vehicle_agent
{

void AdapterMultiplexer::addAdapter(std::unique_ptr<ICloudProtocolAdapter> adapter)
{
  std::lock_guard<std::mutex> lock(mtx_);
  adapters_.push_back(std::move(adapter));
}

void AdapterMultiplexer::startAll(AdapterCallbacks callbacks)
{
  std::lock_guard<std::mutex> lock(mtx_);
  for (auto & a : adapters_) {
    a->start(callbacks);
  }
}

void AdapterMultiplexer::stopAll()
{
  std::lock_guard<std::mutex> lock(mtx_);
  for (auto & a : adapters_) {
    a->stop();
  }
}

void AdapterMultiplexer::publishTelemetry(const ir::Telemetry & t)
{
  std::lock_guard<std::mutex> lock(mtx_);
  for (auto & a : adapters_) {
    if (a->capabilities().supports_telemetry_uplink) a->publishTelemetry(t);
  }
}

void AdapterMultiplexer::publishMissionStatus(const ir::MissionStatus & s)
{
  std::lock_guard<std::mutex> lock(mtx_);
  for (auto & a : adapters_) {
    a->publishMissionStatus(s);
  }
}

void AdapterMultiplexer::publishAck(const ir::Ack & a)
{
  std::lock_guard<std::mutex> lock(mtx_);
  for (auto & ad : adapters_) {
    if (ad->capabilities().supports_ack) ad->publishAck(a);
  }
}

std::string AdapterMultiplexer::primaryName() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  ICloudProtocolAdapter * best = nullptr;
  uint32_t best_prio = 0;
  for (const auto & a : adapters_) {
    auto p = a->capabilities().priority;
    if (!best || p > best_prio) {
      best = a.get();
      best_prio = p;
    }
  }
  return best ? best->name() : std::string{};
}

std::string AdapterMultiplexer::connectedPrimaryName() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  ICloudProtocolAdapter * best = nullptr;
  uint32_t best_prio = 0;
  for (const auto & a : adapters_) {
    if (!a->connected()) continue;
    auto p = a->capabilities().priority;
    if (!best || p > best_prio) {
      best = a.get();
      best_prio = p;
    }
  }
  return best ? best->name() : std::string{};
}

size_t AdapterMultiplexer::size() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return adapters_.size();
}

}  // namespace vehicle_agent
