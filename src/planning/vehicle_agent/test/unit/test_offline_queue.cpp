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

#include "vehicle_agent/storage/offline_queue.hpp"

#include <nlohmann/json.hpp>

#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <unistd.h>

using vehicle_agent::OfflineQueue;
namespace ir = vehicle_agent::ir;

namespace
{
std::string tmp_db_path(const std::string & tag)
{
  char tmpl[] = "/tmp/va_queue_XXXXXX";
  int fd = ::mkstemp(tmpl);
  EXPECT_GE(fd, 0);
  ::close(fd);
  ::unlink(tmpl);   // remove file mkstemp created; sqlite will recreate it
  return std::string(tmpl) + "_" + tag + ".db";
}

ir::Mission make_mission(const std::string & id, uint64_t seq = 1)
{
  ir::Mission m;
  m.id = id;
  m.kind = ir::MissionKind::ROUTE;
  m.seq = seq;
  m.issued_at_ms = 12345;
  m.source_adapter = "scenario_loopback";
  ir::Waypoint w;
  w.x = 1.5;
  w.y = 2.5;
  w.target_speed = 1.0;
  m.waypoints.push_back(w);
  return m;
}

ir::Command make_command(const std::string & mid, ir::CommandKind k)
{
  ir::Command c;
  c.mission_id = mid;
  c.kind = k;
  c.issued_at_ms = 99;
  return c;
}
}  // namespace

class OfflineQueueTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    for (const auto & p : created_) {
      std::error_code ec;
      std::filesystem::remove(p, ec);
      std::filesystem::remove(p + "-wal", ec);
      std::filesystem::remove(p + "-shm", ec);
    }
  }

  std::string newPath(const std::string & tag = "x")
  {
    auto p = tmp_db_path(tag);
    created_.push_back(p);
    return p;
  }

  std::vector<std::string> created_;
};

TEST_F(OfflineQueueTest, EnqueueAndPeekReturnsItemsInOrder)
{
  const auto path = newPath("order");
  OfflineQueue q(path);
  EXPECT_EQ(q.pendingCount(), 0u);

  const auto r1 = q.enqueueMission(make_mission("m1"));
  const auto r2 = q.enqueueCommand(make_command("m1", ir::CommandKind::PAUSE));
  const auto r3 = q.enqueueMission(make_mission("m2"));
  EXPECT_GT(r1, 0);
  EXPECT_GT(r2, r1);
  EXPECT_GT(r3, r2);
  EXPECT_EQ(q.pendingCount(), 3u);

  auto items = q.peekPending(10);
  ASSERT_EQ(items.size(), 3u);
  EXPECT_EQ(items[0].id, r1);
  EXPECT_EQ(items[0].kind, "mission");
  EXPECT_EQ(items[1].id, r2);
  EXPECT_EQ(items[1].kind, "command");
  EXPECT_EQ(items[2].id, r3);
  EXPECT_EQ(items[2].kind, "mission");
}

TEST_F(OfflineQueueTest, AckRemovesFromPending)
{
  const auto path = newPath("ack");
  OfflineQueue q(path);
  const auto r1 = q.enqueueMission(make_mission("m1"));
  const auto r2 = q.enqueueMission(make_mission("m2"));
  EXPECT_EQ(q.pendingCount(), 2u);

  q.ack(r1);
  EXPECT_EQ(q.pendingCount(), 1u);
  auto remaining = q.peekPending(10);
  ASSERT_EQ(remaining.size(), 1u);
  EXPECT_EQ(remaining[0].id, r2);

  q.ack(r2);
  EXPECT_EQ(q.pendingCount(), 0u);
}

TEST_F(OfflineQueueTest, DurabilityAcrossReopen)
{
  const auto path = newPath("durable");
  int64_t row_mission = 0;
  int64_t row_cmd = 0;
  {
    OfflineQueue q(path);
    row_mission = q.enqueueMission(make_mission("durable-1", 7));
    row_cmd = q.enqueueCommand(make_command("durable-1", ir::CommandKind::RESUME));
    q.ack(row_cmd);
    // q goes out of scope here -> destructor closes db (commits WAL).
  }

  OfflineQueue reopened(path);
  EXPECT_EQ(reopened.pendingCount(), 1u);
  auto items = reopened.peekPending(10);
  ASSERT_EQ(items.size(), 1u);
  EXPECT_EQ(items[0].id, row_mission);
  EXPECT_EQ(items[0].kind, "mission");

  auto j = nlohmann::json::parse(items[0].payload_json);
  auto m = j.get<ir::Mission>();
  EXPECT_EQ(m.id, "durable-1");
  EXPECT_EQ(m.seq, 7u);
  ASSERT_EQ(m.waypoints.size(), 1u);
  EXPECT_DOUBLE_EQ(m.waypoints[0].x, 1.5);
}

TEST_F(OfflineQueueTest, AckAllBatchTransaction)
{
  const auto path = newPath("ackall");
  OfflineQueue q(path);
  std::vector<int64_t> ids;
  for (int i = 0; i < 5; ++i) {
    ids.push_back(q.enqueueMission(make_mission("m" + std::to_string(i))));
  }
  EXPECT_EQ(q.pendingCount(), 5u);
  q.ackAll(ids);
  EXPECT_EQ(q.pendingCount(), 0u);
}

TEST_F(OfflineQueueTest, PurgeAckedOlderThan)
{
  const auto path = newPath("purge");
  OfflineQueue q(path);
  const auto r1 = q.enqueueMission(make_mission("m1"));
  q.enqueueMission(make_mission("m2"));
  q.ack(r1);
  // Acked rows older than "now+1 day" matches everything acked -> 1 row.
  const uint64_t way_future = 4'000'000'000'000ULL;
  EXPECT_EQ(q.purgeAckedOlderThan(way_future), 1u);
  EXPECT_EQ(q.pendingCount(), 1u);
}

TEST_F(OfflineQueueTest, PeekRespectsLimit)
{
  const auto path = newPath("limit");
  OfflineQueue q(path);
  for (int i = 0; i < 10; ++i) {
    q.enqueueMission(make_mission("m" + std::to_string(i)));
  }
  auto items = q.peekPending(3);
  EXPECT_EQ(items.size(), 3u);
}
