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

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using nlohmann::json;
using namespace vehicle_agent;
using namespace vehicle_agent::ahtfms_ns;

namespace
{
// Resolve a fixture relative to this source file. Avoids depending on the
// runtime CWD (colcon test runs from the build tree).
fs::path fixture(const std::string & name)
{
  // __FILE__ → src/igv/vehicle_agent/test/unit/test_ahtfms_ns_codec.cpp
  return fs::path(__FILE__).parent_path().parent_path() / "fixtures" / "ahtfms_ns" / name;
}

json readJson(const std::string & name)
{
  std::ifstream f(fixture(name));
  EXPECT_TRUE(f.good()) << "missing fixture: " << name;
  return json::parse(f);
}
}  // namespace

// ---------- framing ----------

TEST(AhtfmsNsFraming, RoundTripSmallPayload)
{
  Frame in;
  in.cm.sender_sys = "IGV2001";
  in.cm.target_sys = "AHTFMS";
  in.cm.event_dt = "24-11-2025T08:00:00.123";
  in.msg.msg_type_id = static_cast<uint16_t>(MsgType::ROUTE_NOTIFY);
  in.payload = {0xDE, 0xAD, 0xBE, 0xEF};
  in.msg.msg_len_q = static_cast<uint32_t>(in.payload.size());

  const auto bytes = encodeFrame(in);
  ASSERT_EQ(bytes.size(), kFrameHeaderSize + 4);
  // magic
  EXPECT_EQ(bytes[0], 'F');
  EXPECT_EQ(bytes[1], 'M');
  EXPECT_EQ(bytes[2], 'S');
  EXPECT_EQ(bytes[3], '1');
  // little-endian msg_type_id 3002 → 0xBA 0x0B
  EXPECT_EQ(bytes[kCmHeaderSize + 0], 0xBA);
  EXPECT_EQ(bytes[kCmHeaderSize + 1], 0x0B);
  // little-endian msg_len_q 4 → 0x04 0x00 0x00 0x00
  EXPECT_EQ(bytes[kCmHeaderSize + 2], 0x04);
  EXPECT_EQ(bytes[kCmHeaderSize + 3], 0x00);

  std::size_t pos = 0;
  Frame out;
  const auto err = decodeFrame(bytes, pos, out);
  EXPECT_EQ(err, FrameError::OK);
  EXPECT_EQ(pos, bytes.size());
  EXPECT_EQ(out.cm.sender_sys, "IGV2001");
  EXPECT_EQ(out.cm.target_sys, "AHTFMS");
  EXPECT_EQ(out.cm.event_dt, "24-11-2025T08:00:00.123");
  EXPECT_EQ(out.msg.msg_type_id, in.msg.msg_type_id);
  EXPECT_EQ(out.payload, in.payload);
}

TEST(AhtfmsNsFraming, RejectsBadMagic)
{
  Frame f;
  f.cm.sender_sys = "X";
  f.cm.target_sys = "Y";
  f.cm.event_dt = "24-11-2025T08:00:00.000";
  f.msg.msg_type_id = 4001;
  f.msg.msg_len_q = 0;
  auto bytes = encodeFrame(f);
  bytes[1] = 'X';  // corrupt 'M' -> 'X'
  std::size_t pos = 0;
  Frame out;
  EXPECT_EQ(decodeFrame(bytes, pos, out), FrameError::BAD_MAGIC);
}

TEST(AhtfmsNsFraming, RejectsTruncatedPayload)
{
  Frame f;
  f.cm.sender_sys = "X";
  f.cm.target_sys = "Y";
  f.cm.event_dt = "24-11-2025T08:00:00.000";
  f.msg.msg_type_id = 3001;
  f.payload = {1, 2, 3, 4, 5};
  f.msg.msg_len_q = static_cast<uint32_t>(f.payload.size());
  auto bytes = encodeFrame(f);
  bytes.resize(bytes.size() - 2);   // chop two payload bytes
  std::size_t pos = 0;
  Frame out;
  EXPECT_EQ(decodeFrame(bytes, pos, out), FrameError::TRUNCATED_PAYLOAD);
}

TEST(AhtfmsNsFraming, RejectsOversizedPayload)
{
  Frame f;
  f.cm.sender_sys = "X";
  f.cm.target_sys = "Y";
  f.cm.event_dt = "24-11-2025T08:00:00.000";
  f.msg.msg_type_id = 3001;
  // Claim a huge payload but provide nothing.
  f.msg.msg_len_q = 100'000;
  f.payload.assign(100'000, 0xAA);
  auto bytes = encodeFrame(f);
  std::size_t pos = 0;
  Frame out;
  EXPECT_EQ(decodeFrame(bytes, pos, out, /*max_payload=*/1024),
            FrameError::OVERSIZED_PAYLOAD);
}

TEST(AhtfmsNsFraming, PadsShortSenderSys)
{
  Frame f;
  f.cm.sender_sys = "X";          // 1 char → 8 NULs pad
  f.cm.target_sys = "AHTFMS";     // 6 chars → 3 NULs pad
  f.cm.event_dt = "24-11-2025T08:00:00.000";
  f.msg.msg_type_id = 4001;
  f.msg.msg_len_q = 0;
  auto bytes = encodeFrame(f);
  // sender_sys slot starts at offset 4, length 9.
  for (std::size_t i = 1; i < 9; ++i) {
    EXPECT_EQ(bytes[4 + i], 0u) << "expected pad at offset 4+" << i;
  }
  std::size_t pos = 0;
  Frame out;
  ASSERT_EQ(decodeFrame(bytes, pos, out), FrameError::OK);
  EXPECT_EQ(out.cm.sender_sys, "X");
  EXPECT_EQ(out.cm.target_sys, "AHTFMS");
}

TEST(AhtfmsNsFraming, ThrowsOnOverlongField)
{
  Frame f;
  f.cm.sender_sys = "TEN_CHARS!";   // 10 > slot 9
  f.cm.target_sys = "AHTFMS";
  f.cm.event_dt = "24-11-2025T08:00:00.000";
  f.msg.msg_type_id = 1001;
  f.msg.msg_len_q = 0;
  EXPECT_THROW(encodeFrame(f), std::invalid_argument);
}

TEST(AhtfmsNsFraming, EventDtFormatRoundTrip)
{
  // 2025-11-24 08:00:00.123 UTC -> 1763971200123 ms.
  const std::string s = formatEventDt(1763971200123ULL);
  EXPECT_EQ(s.size(), kEventDtSize);
  EXPECT_EQ(s, "24-11-2025T08:00:00.123");
  auto back = parseEventDt(s);
  ASSERT_TRUE(back.has_value());
  EXPECT_EQ(*back, 1763971200123ULL);
}

TEST(AhtfmsNsFraming, ParseEventDtTolerantOfTrailingNul)
{
  std::string s = "24-11-2025T08:00:00.123";
  s.push_back('\0');                 // simulate the padding tail we'd see on-wire
  EXPECT_TRUE(parseEventDt(s).has_value());
  EXPECT_FALSE(parseEventDt("bogus").has_value());
}

// ---------- codec inbound ----------

TEST(AhtfmsNsCodec, DecodesRouteNotifyToMission)
{
  auto j = readJson("route_notify_basic.json");
  auto m = decodeRouteNotify(j);
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(m->id, "R-2025-0001");
  EXPECT_EQ(m->kind, ir::MissionKind::ROUTE);
  EXPECT_EQ(m->source_adapter, "ahtfms_ns");
  EXPECT_EQ(m->issued_at_ms, 1732468800000ULL);
  // 3 forward + 2 reverse points across two segments.
  ASSERT_EQ(m->waypoints.size(), 5u);
  EXPECT_EQ(m->waypoints[0].segment_id, "seg0:fwd");
  EXPECT_EQ(m->waypoints[3].segment_id, "seg1:rev");
  EXPECT_DOUBLE_EQ(m->waypoints[0].x, 100.0);
  EXPECT_DOUBLE_EQ(m->waypoints[2].x, 120.0);
  EXPECT_DOUBLE_EQ(m->waypoints[4].x, 110.0);
}

TEST(AhtfmsNsCodec, DecodesJobNotifyToMission)
{
  auto j = readJson("job_notify_basic.json");
  auto m = decodeJobNotify(j);
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(m->id, "J-9001");
  EXPECT_EQ(m->kind, ir::MissionKind::ROUTE);
  EXPECT_EQ(m->source_adapter, "ahtfms_ns");
  EXPECT_TRUE(m->waypoints.empty()) << "job notify carries no waypoints";
}

TEST(AhtfmsNsCodec, DecodesJobNotifyCancelAsAbort)
{
  auto j = readJson("job_notify_cancel.json");
  auto m = decodeJobNotify(j);
  ASSERT_TRUE(m.has_value());
  EXPECT_EQ(m->kind, ir::MissionKind::ABORT)
    << "empty job_id should cancel the planned path per ICD §8.58";
}

TEST(AhtfmsNsCodec, DecodesStopPointActivateAsPause)
{
  auto j = readJson("stop_point_notify_activate.json");
  auto c = decodeStopPointNotify(j);
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(c->kind, ir::CommandKind::PAUSE);
}

TEST(AhtfmsNsCodec, DecodesRemoteCommandSuspendEmergencyAsEstop)
{
  auto j = readJson("remote_command_suspend_emergency.json");
  auto c = decodeRemoteCommand(j);
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(c->kind, ir::CommandKind::ESTOP)
    << "EMERGENCY_BRAKE stopping_type must escalate to ESTOP";
  EXPECT_EQ(c->mission_id, "C-9999");
}

TEST(AhtfmsNsCodec, DecodesRemoteCommandResume)
{
  auto j = readJson("remote_command_resume.json");
  auto c = decodeRemoteCommand(j);
  ASSERT_TRUE(c.has_value());
  EXPECT_EQ(c->kind, ir::CommandKind::RESUME);
}

TEST(AhtfmsNsCodec, DecodePayloadDispatchesByType)
{
  auto j = readJson("route_notify_basic.json");
  const auto s = j.dump();
  std::vector<uint8_t> bytes(s.begin(), s.end());
  auto out = decodePayload(MsgType::ROUTE_NOTIFY, bytes);
  ASSERT_TRUE(out.has_value());
  EXPECT_TRUE(std::holds_alternative<ir::Mission>(*out));
}

TEST(AhtfmsNsCodec, DecodeDropsFrameWithMismatchedTarget)
{
  // Build a route_notify frame addressed to a different vehicle.
  auto j = readJson("route_notify_basic.json");
  const auto s = j.dump();
  Frame f;
  f.cm.sender_sys = "AHTFMS";
  f.cm.target_sys = "IGV9999";       // not us
  f.cm.event_dt = "24-11-2025T08:00:00.000";
  f.msg.msg_type_id = static_cast<uint16_t>(MsgType::ROUTE_NOTIFY);
  f.payload.assign(s.begin(), s.end());
  f.msg.msg_len_q = static_cast<uint32_t>(f.payload.size());

  DecodeContext ctx;
  ctx.vehicle_sys = "IGV2001";
  auto out = decode(f, ctx);
  ASSERT_TRUE(out.has_value());
  EXPECT_TRUE(std::holds_alternative<DecodedNothing>(*out));
}

// ---------- codec outbound ----------

TEST(AhtfmsNsCodec, EncodesRemoteCommandResponse)
{
  ir::Ack a;
  a.target_id = "C-1234";
  a.status = ir::AckStatus::OK;
  a.stamp_ms = 1732468800000;
  const auto bytes = encodeRemoteCommandResponse(a);
  const auto j = json::parse(bytes.begin(), bytes.end());
  EXPECT_EQ(j["command_id"], "C-1234");
  EXPECT_EQ(j["command_state"], 1);
  EXPECT_EQ(j["header"]["message_time"], 1732468800000ULL);
}

TEST(AhtfmsNsCodec, EncodesRemoteCommandResponseFailureMapsToTwo)
{
  ir::Ack a;
  a.target_id = "C-7";
  a.status = ir::AckStatus::FAILED;
  a.stamp_ms = 0;
  const auto bytes = encodeRemoteCommandResponse(a);
  const auto j = json::parse(bytes.begin(), bytes.end());
  EXPECT_EQ(j["command_state"], 2);
}

TEST(AhtfmsNsCodec, EncodesVehicleBasicStatus)
{
  ir::Telemetry t;
  t.vehicle_id = "IGV2001";
  t.stamp_ms = 1732468800500;
  t.pose.x = 100.5;
  t.pose.y = 200.25;
  t.pose.yaw = 1.57;
  t.speed_mps = 4.2;
  t.steering_rad = -0.05;
  t.current_mission_id = "R-77";
  t.mission_state = 3;
  t.e_stop_active = false;
  const auto bytes = encodeVehicleBasicStatus(t);
  const auto j = json::parse(bytes.begin(), bytes.end());
  EXPECT_EQ(j["vehicle_id"], "IGV2001");
  EXPECT_DOUBLE_EQ(j["position"]["x"].get<double>(), 100.5);
  EXPECT_DOUBLE_EQ(j["heading"].get<double>(), 1.57);
  EXPECT_DOUBLE_EQ(j["speed"].get<double>(), 4.2);
  EXPECT_EQ(j["current_mission_id"], "R-77");
  EXPECT_EQ(j["mission_state"], 3);
}

TEST(AhtfmsNsCodec, EncodesVehicleKeyEventSyncRequest)
{
  const auto bytes = encodeVehicleKeyEventSyncRequest(42);
  const auto j = json::parse(bytes.begin(), bytes.end());
  EXPECT_EQ(j["event_type"], 1);
  EXPECT_EQ(j["header"]["message_time"], 42u);
}
