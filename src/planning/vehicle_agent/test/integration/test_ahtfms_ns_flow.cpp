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

#include "vehicle_agent/adapters/ahtfms_ns/codec.hpp"
#include "vehicle_agent/adapters/ahtfms_ns/framing.hpp"
#include "vehicle_agent/adapters/ahtfms_ns/transport.hpp"
#include "vehicle_agent/adapters/ahtfms_ns_adapter.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using nlohmann::json;
using namespace vehicle_agent;
using namespace vehicle_agent::ahtfms_ns;

namespace
{
fs::path fixture(const std::string & name)
{
  return fs::path(__FILE__).parent_path().parent_path() / "fixtures" / "ahtfms_ns" / name;
}

json readJson(const std::string & name)
{
  std::ifstream f(fixture(name));
  EXPECT_TRUE(f.good()) << "missing fixture: " << name;
  return json::parse(f);
}

// Helper: simulate the CM pushing a frame to NS.
std::vector<uint8_t> buildIcdFrame(MsgType type,
                                   const std::string & target,
                                   const json & payload_json,
                                   const std::string & sender = "AHTFMS",
                                   const std::string & dt = "24-11-2025T08:00:00.000")
{
  const auto s = payload_json.dump();
  Frame f;
  f.cm.sender_sys = sender;
  f.cm.target_sys = target;
  f.cm.event_dt = dt;
  f.msg.msg_type_id = static_cast<uint16_t>(type);
  f.payload.assign(s.begin(), s.end());
  f.msg.msg_len_q = static_cast<uint32_t>(f.payload.size());
  return encodeFrame(f);
}

// Inbox that captures whatever the adapter dispatches into the app layer.
struct AppInbox
{
  std::mutex m;
  std::condition_variable cv;
  std::vector<ir::Mission> missions;
  std::vector<ir::Command> commands;
  std::vector<std::pair<std::string, bool>> conn_events;

  AdapterCallbacks asCallbacks()
  {
    AdapterCallbacks cb;
    cb.on_mission = [this](ir::Mission m) {
      std::lock_guard<std::mutex> lk(m_);
      missions.push_back(std::move(m));
      cv.notify_all();
    };
    cb.on_command = [this](ir::Command c) {
      std::lock_guard<std::mutex> lk(m_);
      commands.push_back(std::move(c));
      cv.notify_all();
    };
    cb.on_connection_state_changed = [this](std::string n, bool up) {
      std::lock_guard<std::mutex> lk(m_);
      conn_events.emplace_back(std::move(n), up);
      cv.notify_all();
    };
    return cb;
  }

  // Wait for `pred()` to be true under the lock, up to `dur_ms`. Returns
  // whether the predicate became true within the timeout.
  template <typename Pred>
  bool waitFor(Pred pred, int dur_ms = 1000)
  {
    std::unique_lock<std::mutex> lk(m_);
    return cv.wait_for(lk, std::chrono::milliseconds(dur_ms), pred);
  }

private:
  std::mutex & m_ {m};
};
}  // namespace

class AhtfmsNsFlow : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Two ends of a single TCP-like pipe.
    auto a = std::make_unique<FakeTransport>();
    auto b = std::make_unique<FakeTransport>();
    FakeTransport::pair(*a, *b);
    ns_end_ = a.get();
    cm_end_ = b.get();

    AhtfmsNsAdapter::Config cfg;
    cfg.vehicle_id = "IGV2001";
    cfg.fms_sys = "AHTFMS";
    adapter_ = std::make_unique<AhtfmsNsAdapter>(cfg, std::move(a));

    // Hook the CM end so we can both observe outbound and inject inbound.
    cm_end_->setOnBytes([this](const std::vector<uint8_t> & bytes) {
      std::lock_guard<std::mutex> lk(cm_mtx_);
      cm_received_.insert(cm_received_.end(), bytes.begin(), bytes.end());
    });

    // Keep b alive for the duration of the test.
    cm_owned_ = std::move(b);
  }

  void TearDown() override
  {
    adapter_->stop();
  }

  AppInbox inbox_;
  std::unique_ptr<AhtfmsNsAdapter> adapter_;
  FakeTransport * ns_end_{nullptr};
  FakeTransport * cm_end_{nullptr};
  std::unique_ptr<FakeTransport> cm_owned_;

  std::mutex cm_mtx_;
  std::vector<uint8_t> cm_received_;
};

TEST_F(AhtfmsNsFlow, ConnectEmitsSyncRequestUpstream)
{
  adapter_->start(inbox_.asCallbacks());
  ns_end_->connect();

  // Expect: adapter pushes one VehicleKeyEventStatus frame and the inbox
  // sees a single connection-up event.
  inbox_.waitFor([&]() {
    return !inbox_.conn_events.empty();
  });

  // Pull what CM saw.
  std::vector<uint8_t> seen;
  {
    std::lock_guard<std::mutex> lk(cm_mtx_);
    seen = cm_received_;
  }
  ASSERT_GE(seen.size(), kFrameHeaderSize);
  std::size_t pos = 0;
  Frame f;
  ASSERT_EQ(decodeFrame(seen, pos, f), FrameError::OK);
  EXPECT_EQ(f.cm.sender_sys, "IGV2001");
  EXPECT_EQ(f.cm.target_sys, "AHTFMS");
  EXPECT_EQ(f.msg.msg_type_id,
            static_cast<uint16_t>(MsgType::VEHICLE_KEY_EVENT_STATUS));
  const auto j = json::parse(f.payload.begin(), f.payload.end());
  EXPECT_EQ(j["event_type"], 1);
}

TEST_F(AhtfmsNsFlow, RouteNotifyDeliversMissionToApp)
{
  adapter_->start(inbox_.asCallbacks());
  ns_end_->connect();
  // Drain the sync request the adapter emits on connect so it doesn't
  // confuse counter assertions later.
  {
    std::lock_guard<std::mutex> lk(cm_mtx_);
    cm_received_.clear();
  }

  auto frame = buildIcdFrame(MsgType::ROUTE_NOTIFY, "IGV2001",
                             readJson("route_notify_basic.json"));
  // Send from the CM side → adapter sees it as inbound.
  ASSERT_TRUE(cm_end_->send(frame));

  ASSERT_TRUE(inbox_.waitFor([&]() { return !inbox_.missions.empty(); }))
    << "adapter never delivered the mission";
  ASSERT_EQ(inbox_.missions.size(), 1u);
  EXPECT_EQ(inbox_.missions[0].id, "R-2025-0001");
  EXPECT_EQ(inbox_.missions[0].source_adapter, "ahtfms_ns");
  EXPECT_EQ(inbox_.missions[0].waypoints.size(), 5u);

  const auto cnt = adapter_->counters();
  EXPECT_GE(cnt.frames_in, 1u);
}

TEST_F(AhtfmsNsFlow, RemoteCommandSuspendEmergencyEscalatesToEstop)
{
  adapter_->start(inbox_.asCallbacks());
  ns_end_->connect();

  auto frame = buildIcdFrame(MsgType::REMOTE_COMMAND, "IGV2001",
                             readJson("remote_command_suspend_emergency.json"));
  ASSERT_TRUE(cm_end_->send(frame));

  ASSERT_TRUE(inbox_.waitFor([&]() { return !inbox_.commands.empty(); }));
  EXPECT_EQ(inbox_.commands[0].kind, ir::CommandKind::ESTOP);
}

TEST_F(AhtfmsNsFlow, FrameAddressedToOtherVehicleIsDropped)
{
  adapter_->start(inbox_.asCallbacks());
  ns_end_->connect();

  auto frame = buildIcdFrame(MsgType::ROUTE_NOTIFY, "IGV9999",
                             readJson("route_notify_basic.json"));
  ASSERT_TRUE(cm_end_->send(frame));

  // The adapter should NOT deliver this. Wait a short time then assert silence.
  EXPECT_FALSE(inbox_.waitFor([&]() { return !inbox_.missions.empty(); }, 200));
  const auto cnt = adapter_->counters();
  EXPECT_GE(cnt.target_mismatches, 1u);
}

TEST_F(AhtfmsNsFlow, ReassemblesFrameAcrossChunks)
{
  adapter_->start(inbox_.asCallbacks());
  ns_end_->connect();

  // Build one frame, then send it in three slices.
  const auto bytes = buildIcdFrame(MsgType::ROUTE_NOTIFY, "IGV2001",
                                   readJson("route_notify_basic.json"));
  const std::size_t s1 = 20;
  const std::size_t s2 = bytes.size() / 2;
  std::vector<uint8_t> a(bytes.begin(), bytes.begin() + s1);
  std::vector<uint8_t> b(bytes.begin() + s1, bytes.begin() + s2);
  std::vector<uint8_t> c(bytes.begin() + s2, bytes.end());
  ASSERT_TRUE(cm_end_->send(a));
  ASSERT_TRUE(cm_end_->send(b));
  ASSERT_TRUE(cm_end_->send(c));

  ASSERT_TRUE(inbox_.waitFor([&]() { return !inbox_.missions.empty(); }));
  EXPECT_EQ(inbox_.missions.size(), 1u);
}

TEST_F(AhtfmsNsFlow, MultipleFramesInOneChunk)
{
  adapter_->start(inbox_.asCallbacks());
  ns_end_->connect();

  auto f1 = buildIcdFrame(MsgType::ROUTE_NOTIFY, "IGV2001",
                          readJson("route_notify_basic.json"));
  auto f2 = buildIcdFrame(MsgType::REMOTE_COMMAND, "IGV2001",
                          readJson("remote_command_resume.json"));
  std::vector<uint8_t> joined;
  joined.insert(joined.end(), f1.begin(), f1.end());
  joined.insert(joined.end(), f2.begin(), f2.end());
  ASSERT_TRUE(cm_end_->send(joined));

  ASSERT_TRUE(inbox_.waitFor([&]() {
    return !inbox_.missions.empty() && !inbox_.commands.empty();
  }));
  EXPECT_EQ(inbox_.missions.size(), 1u);
  EXPECT_EQ(inbox_.commands.size(), 1u);
  EXPECT_EQ(inbox_.commands[0].kind, ir::CommandKind::RESUME);
}

TEST_F(AhtfmsNsFlow, TelemetryUplinkProducesVehicleBasicStatusFrame)
{
  adapter_->start(inbox_.asCallbacks());
  ns_end_->connect();
  {
    std::lock_guard<std::mutex> lk(cm_mtx_);
    cm_received_.clear();   // drop the sync_request the adapter sends first
  }

  ir::Telemetry t;
  t.vehicle_id = "IGV2001";
  t.stamp_ms = 1732468800500;
  t.pose.x = 10.0;
  t.pose.y = 20.0;
  t.pose.yaw = 0.1;
  t.speed_mps = 3.0;
  t.current_mission_id = "R-77";
  t.mission_state = 3;
  adapter_->publishTelemetry(t);

  // Poll the CM-side capture until at least one frame appears.
  std::vector<uint8_t> seen;
  for (int i = 0; i < 50 && seen.empty(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    std::lock_guard<std::mutex> lk(cm_mtx_);
    seen = cm_received_;
  }
  ASSERT_FALSE(seen.empty());

  std::size_t pos = 0;
  Frame f;
  ASSERT_EQ(decodeFrame(seen, pos, f), FrameError::OK);
  EXPECT_EQ(f.cm.sender_sys, "IGV2001");
  EXPECT_EQ(f.cm.target_sys, "AHTFMS");
  EXPECT_EQ(f.msg.msg_type_id,
            static_cast<uint16_t>(MsgType::VEHICLE_BASIC_STATUS));
  const auto j = json::parse(f.payload.begin(), f.payload.end());
  EXPECT_EQ(j["current_mission_id"], "R-77");
}

TEST_F(AhtfmsNsFlow, AckProducesRemoteCommandResponseFrame)
{
  adapter_->start(inbox_.asCallbacks());
  ns_end_->connect();
  {
    std::lock_guard<std::mutex> lk(cm_mtx_);
    cm_received_.clear();
  }

  ir::Ack a;
  a.target_id = "C-321";
  a.status = ir::AckStatus::OK;
  a.stamp_ms = 1732468800999;
  adapter_->publishAck(a);

  std::vector<uint8_t> seen;
  for (int i = 0; i < 50 && seen.empty(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    std::lock_guard<std::mutex> lk(cm_mtx_);
    seen = cm_received_;
  }
  ASSERT_FALSE(seen.empty());

  std::size_t pos = 0;
  Frame f;
  ASSERT_EQ(decodeFrame(seen, pos, f), FrameError::OK);
  EXPECT_EQ(f.msg.msg_type_id,
            static_cast<uint16_t>(MsgType::REMOTE_COMMAND_RESPONSE));
  const auto j = json::parse(f.payload.begin(), f.payload.end());
  EXPECT_EQ(j["command_id"], "C-321");
  EXPECT_EQ(j["command_state"], 1);
}

TEST_F(AhtfmsNsFlow, DisconnectStopsSendsAndNotifiesApp)
{
  adapter_->start(inbox_.asCallbacks());
  ns_end_->connect();
  ASSERT_TRUE(inbox_.waitFor([&]() { return !inbox_.conn_events.empty(); }));

  ns_end_->disconnect();
  ASSERT_TRUE(inbox_.waitFor([&]() {
    // We expect at least one true→false flip.
    for (const auto & e : inbox_.conn_events) {
      if (!e.second) return true;
    }
    return false;
  }));

  ir::Telemetry t;
  EXPECT_FALSE(ns_end_->send({0, 1, 2})) << "send should fail while disconnected";
}
