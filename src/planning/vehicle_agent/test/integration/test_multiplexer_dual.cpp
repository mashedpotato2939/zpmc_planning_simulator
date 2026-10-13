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
#include "vehicle_agent/adapters/internal_zpmc_adapter.hpp"
#include "vehicle_agent/adapters/internal_zpmc/bus.hpp"
#include "vehicle_agent/adapters/scenario_loopback_adapter.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using nlohmann::json;
using namespace vehicle_agent;
using namespace vehicle_agent::internal_zpmc;

namespace
{
// A tiny scenario the loopback adapter can run on its own thread. We point
// the adapter at a temp file rather than the shipped scenarios/*.yaml so the
// test doesn't depend on install-time data.
fs::path writeTempScenario(const std::string & body)
{
  static int counter = 0;
  auto p = fs::temp_directory_path() /
           ("va_sprint5_scenario_" + std::to_string(++counter) + ".yaml");
  std::ofstream f(p);
  f << body;
  return p;
}

// Capture everything an "app layer" cares about.
struct Inbox
{
  std::mutex mtx;
  std::condition_variable cv;
  std::vector<ir::Mission> missions;
  std::vector<ir::Command> commands;
  std::vector<std::pair<std::string, bool>> conn_events;

  AdapterCallbacks cb()
  {
    AdapterCallbacks c;
    c.on_mission = [this](ir::Mission m) {
      std::lock_guard<std::mutex> lk(mtx);
      missions.push_back(std::move(m));
      cv.notify_all();
    };
    c.on_command = [this](ir::Command cmd) {
      std::lock_guard<std::mutex> lk(mtx);
      commands.push_back(std::move(cmd));
      cv.notify_all();
    };
    c.on_connection_state_changed = [this](std::string n, bool up) {
      std::lock_guard<std::mutex> lk(mtx);
      conn_events.emplace_back(std::move(n), up);
      cv.notify_all();
    };
    return c;
  }

  template <typename Pred>
  bool waitFor(Pred p, int ms = 1500)
  {
    std::unique_lock<std::mutex> lk(mtx);
    return cv.wait_for(lk, std::chrono::milliseconds(ms), p);
  }
};
}  // namespace

class MultiplexerDualTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // FMS-side bus subscribes to the topics the adapter publishes on, so the
    // test can observe uplink traffic. Both buses share the broker.
    fms_bus_ = std::make_unique<FakeMqttBus>(broker_, "fms-client");
    fms_bus_->setOnMessage([this](const std::string & t, const std::vector<uint8_t> & p) {
      std::lock_guard<std::mutex> lk(uplink_mtx_);
      uplink_.emplace_back(t, p);
    });
    fms_bus_->start();
    fms_bus_->subscribe("zpmc/v1/telemetry/IGV2001");
    fms_bus_->subscribe("zpmc/v1/status/IGV2001");
    fms_bus_->subscribe("zpmc/v1/ack/IGV2001");

    // Vehicle-side bus the adapter owns.
    auto vehicle_bus = std::make_unique<FakeMqttBus>(broker_, "vehicle-client");
    vehicle_bus_ = vehicle_bus.get();

    InternalZpmcAdapter::Config cfg;
    cfg.vehicle_id = "IGV2001";
    cfg.hmac_key = "";   // signing off so the test bypasses key provisioning
    auto zpmc = std::make_unique<InternalZpmcAdapter>(cfg, std::move(vehicle_bus));
    zpmc_ptr_ = zpmc.get();

    scenario_path_ = writeTempScenario(R"YAML(
steps:
  - delay_ms: 50
    type: mission
    id: mux-001
    seq: 1
    waypoints:
      - { x: 1.0, y: 1.0, yaw: 0.0, target_speed: 1.0, segment_id: "s0" }
  - delay_ms: 50
    type: complete
    mission_id: mux-001
)YAML");
    auto loopback = std::make_unique<ScenarioLoopbackAdapter>(scenario_path_.string());

    mux_.addAdapter(std::move(zpmc));
    mux_.addAdapter(std::move(loopback));
  }

  void TearDown() override
  {
    mux_.stopAll();
    fms_bus_->stop();
    std::error_code ec;
    fs::remove(scenario_path_, ec);
  }

  // Hoisted into the fixture so it outlives TearDown(). When the multiplexer
  // stops, internal_zpmc emits a final connection-state(false) callback —
  // that callback captures &inbox_, so inbox_ must still be alive at that
  // point. A local inside the TEST_F body would be gone by then.
  Inbox inbox_;

  // Publish a signed-envelope message from the FMS side to a topic the
  // adapter subscribes to. cfg.hmac_key is empty so we omit the sig.
  void fmsPublish(const std::string & topic, const json & payload)
  {
    json env;
    env["payload"] = payload;
    env["ts"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::system_clock::now().time_since_epoch()).count();
    env["nonce"] = "nonce-" + std::to_string(nonce_counter_++);
    const auto s = env.dump();
    fms_bus_->publish(topic, std::vector<uint8_t>(s.begin(), s.end()));
  }

  FakeMqttBroker broker_;
  std::unique_ptr<FakeMqttBus> fms_bus_;
  FakeMqttBus * vehicle_bus_{nullptr};     // owned by adapter
  AdapterMultiplexer mux_;
  InternalZpmcAdapter * zpmc_ptr_{nullptr};
  fs::path scenario_path_;
  int nonce_counter_{0};

  std::mutex uplink_mtx_;
  std::vector<std::pair<std::string, std::vector<uint8_t>>> uplink_;
};

TEST_F(MultiplexerDualTest, BothAdaptersComeUpAndLoopbackDeliversMission)
{
  
  mux_.startAll(inbox_.cb());

  // Loopback fires the mission ~50 ms in.
  ASSERT_TRUE(inbox_.waitFor([&]() { return !inbox_.missions.empty(); }))
    << "loopback never delivered its scripted mission";
  EXPECT_EQ(inbox_.missions.front().id, "mux-001");
  EXPECT_EQ(inbox_.missions.front().source_adapter, "scenario_loopback");

  // Both adapters should now be marked connected at the multiplexer.
  EXPECT_TRUE(vehicle_bus_->connected());
  EXPECT_EQ(mux_.size(), 2u);
  EXPECT_EQ(mux_.primaryName(), "internal_zpmc");        // highest priority
  EXPECT_EQ(mux_.connectedPrimaryName(), "internal_zpmc");
}

TEST_F(MultiplexerDualTest, TelemetryIsMirroredToInternalZpmcOverMqtt)
{
  
  mux_.startAll(inbox_.cb());

  ir::Telemetry t;
  t.vehicle_id = "IGV2001";
  t.pose.x = 5.5;
  t.pose.y = -1.25;
  t.speed_mps = 2.0;
  t.stamp_ms = 1763971200500;
  mux_.publishTelemetry(t);

  // Spin briefly to let the broker route the publish.
  std::vector<std::pair<std::string, std::vector<uint8_t>>> seen;
  for (int i = 0; i < 50 && seen.empty(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::lock_guard<std::mutex> lk(uplink_mtx_);
    seen = uplink_;
  }
  ASSERT_FALSE(seen.empty());

  bool found_telemetry = false;
  for (const auto & [topic, bytes] : seen) {
    if (topic != "zpmc/v1/telemetry/IGV2001") continue;
    found_telemetry = true;
    const auto env = json::parse(bytes.begin(), bytes.end());
    EXPECT_DOUBLE_EQ(env["payload"]["x"].get<double>(), 5.5);
    EXPECT_EQ(env["payload"]["vehicle_id"], "IGV2001");
    break;
  }
  EXPECT_TRUE(found_telemetry) << "telemetry wasn't mirrored to internal_zpmc";
}

TEST_F(MultiplexerDualTest, MissionOnZpmcTopicReachesAppLayer)
{
  
  mux_.startAll(inbox_.cb());

  json mission;
  mission["id"] = "zpmc-007";
  mission["kind"] = 0;
  mission["seq"] = 42;
  mission["issued_at_ms"] = 1700000000000ULL;
  mission["waypoints"] = json::array();
  fmsPublish("zpmc/v1/mission/IGV2001", mission);

  ASSERT_TRUE(inbox_.waitFor([&]() {
    for (const auto & m : inbox_.missions) {
      if (m.id == "zpmc-007") return true;
    }
    return false;
  }));

  // Adapter should also be counting the inbound.
  const auto cnt = zpmc_ptr_->counters();
  EXPECT_GE(cnt.messages_in, 1u);
}

TEST_F(MultiplexerDualTest, FailoverWhenInternalZpmcDisconnects)
{
  
  mux_.startAll(inbox_.cb());

  // Preferred primary is internal_zpmc (priority 300).
  EXPECT_EQ(mux_.connectedPrimaryName(), "internal_zpmc");

  // Pull the MQTT link out from under it.
  vehicle_bus_->disconnect();

  // Failover: with internal_zpmc disconnected, the multiplexer reports the
  // loopback as the connected primary. Loopback is always "connected" by
  // virtue of being in-process.
  EXPECT_EQ(mux_.connectedPrimaryName(), "scenario_loopback");
  EXPECT_EQ(mux_.primaryName(), "internal_zpmc")
    << "primaryName must remain priority-only; failover lives in "
       "connectedPrimaryName";
}

TEST_F(MultiplexerDualTest, ReplayedNonceIsRejected)
{
  
  mux_.startAll(inbox_.cb());

  // First delivery accepted.
  json env;
  env["payload"] = {{"id", "rep-001"}, {"kind", 0}, {"seq", 1},
                    {"issued_at_ms", 1700000000000ULL},
                    {"waypoints", json::array()}};
  env["ts"] = std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch()).count();
  env["nonce"] = "fixed-nonce";
  const auto s = env.dump();
  std::vector<uint8_t> bytes(s.begin(), s.end());

  fms_bus_->publish("zpmc/v1/mission/IGV2001", bytes);
  ASSERT_TRUE(inbox_.waitFor([&]() {
    for (const auto & m : inbox_.missions) {
      if (m.id == "rep-001") return true;
    }
    return false;
  }));

  const auto baseline = zpmc_ptr_->counters().replay_rejections;

  // Same envelope a second time: nonce reuse → reject.
  fms_bus_->publish("zpmc/v1/mission/IGV2001", bytes);
  // Give the broker time to deliver and the adapter time to reject.
  for (int i = 0; i < 20; ++i) {
    if (zpmc_ptr_->counters().replay_rejections > baseline) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_GT(zpmc_ptr_->counters().replay_rejections, baseline);
}
