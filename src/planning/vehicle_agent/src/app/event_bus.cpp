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

#include "vehicle_agent/app/event_bus.hpp"

#include <utility>

namespace vehicle_agent
{

EventBus::EventBus(size_t capacity) : capacity_(capacity) {}

void EventBus::push(ir::Event ev)
{
  std::unique_lock<std::mutex> lock(mtx_);
  // Backpressure: drop oldest TICK/TELEMETRY first; never drop commands/missions.
  if (q_.size() >= capacity_) {
    for (auto it = q_.begin(); it != q_.end(); ++it) {
      if (it->kind == ir::EventKind::TICK || it->kind == ir::EventKind::TELEMETRY_UPDATE) {
        q_.erase(it);
        break;
      }
    }
  }
  q_.push_back(std::move(ev));
  cv_.notify_one();
}

std::optional<ir::Event> EventBus::popBlocking()
{
  std::unique_lock<std::mutex> lock(mtx_);
  cv_.wait(lock, [this] { return !q_.empty() || closed_; });
  if (q_.empty()) return std::nullopt;
  auto ev = std::move(q_.front());
  q_.pop_front();
  return ev;
}

std::optional<ir::Event> EventBus::tryPop()
{
  std::unique_lock<std::mutex> lock(mtx_);
  if (q_.empty()) return std::nullopt;
  auto ev = std::move(q_.front());
  q_.pop_front();
  return ev;
}

void EventBus::close()
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    closed_ = true;
  }
  cv_.notify_all();
}

size_t EventBus::size() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return q_.size();
}

}  // namespace vehicle_agent
