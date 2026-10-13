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

#ifndef VEHICLE_AGENT__APP__EVENT_BUS_HPP_
#define VEHICLE_AGENT__APP__EVENT_BUS_HPP_

#include "vehicle_agent/ir/event.hpp"

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>

namespace vehicle_agent
{

// FIFO event queue. Multiple producers (adapters, ROS subscriber timers),
// single logical consumer (the state machine thread).
class EventBus
{
public:
  explicit EventBus(size_t capacity = 1024);

  // Producer-side. Blocks briefly if capacity reached.
  void push(ir::Event ev);

  // Consumer-side. Blocks until an event is available or close() is called.
  // Returns nullopt iff bus closed and queue drained.
  std::optional<ir::Event> popBlocking();

  // Non-blocking pop. Returns nullopt if empty.
  std::optional<ir::Event> tryPop();

  void close();
  size_t size() const;

private:
  mutable std::mutex mtx_;
  std::condition_variable cv_;
  std::deque<ir::Event> q_;
  size_t capacity_;
  bool closed_{false};
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__APP__EVENT_BUS_HPP_
