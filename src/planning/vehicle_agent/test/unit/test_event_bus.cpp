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
#include "vehicle_agent/ir/event.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

using vehicle_agent::EventBus;
using vehicle_agent::ir::Event;
using vehicle_agent::ir::EventKind;
using vehicle_agent::ir::Mission;
using vehicle_agent::ir::Telemetry;
using vehicle_agent::ir::Tick;

TEST(EventBus, FifoOrdering)
{
  EventBus bus(64);
  for (int i = 0; i < 5; ++i) {
    Mission m;
    m.id = "m" + std::to_string(i);
    bus.push(Event{EventKind::MISSION_RECEIVED, m});
  }
  ASSERT_EQ(bus.size(), 5u);

  for (int i = 0; i < 5; ++i) {
    auto ev = bus.tryPop();
    ASSERT_TRUE(ev.has_value());
    EXPECT_EQ(std::get<Mission>(ev->payload).id, "m" + std::to_string(i));
  }
  EXPECT_FALSE(bus.tryPop().has_value());
}

TEST(EventBus, PopBlockingUnblocksOnClose)
{
  EventBus bus(8);
  std::thread reader([&] {
    auto ev = bus.popBlocking();
    EXPECT_FALSE(ev.has_value());
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  bus.close();
  reader.join();
}

TEST(EventBus, BackpressureDropsTickFirst)
{
  EventBus bus(2);
  // Fill with non-droppable events.
  Mission m1; m1.id = "keep-1";
  Mission m2; m2.id = "keep-2";
  bus.push(Event{EventKind::MISSION_RECEIVED, m1});
  bus.push(Event{EventKind::MISSION_RECEIVED, m2});
  ASSERT_EQ(bus.size(), 2u);

  // Pushing TICK at cap: nothing to drop (no TICK/TELEMETRY present),
  // so we just append and size grows by 1 beyond cap.
  bus.push(Event{EventKind::TICK, Tick{}});
  EXPECT_EQ(bus.size(), 3u);

  // Now there is a TICK in the queue. A new MISSION push hits capacity logic
  // (size>=cap which it already is). It should drop the TICK first.
  Mission m3; m3.id = "keep-3";
  bus.push(Event{EventKind::MISSION_RECEIVED, m3});

  // After: m1, m2, m3 — TICK gone.
  ASSERT_EQ(bus.size(), 3u);
  for (const char * id : {"keep-1", "keep-2", "keep-3"}) {
    auto ev = bus.tryPop();
    ASSERT_TRUE(ev.has_value());
    EXPECT_EQ(ev->kind, EventKind::MISSION_RECEIVED);
    EXPECT_EQ(std::get<Mission>(ev->payload).id, id);
  }
}
