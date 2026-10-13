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

#ifndef VEHICLE_AGENT__STORAGE__OFFLINE_QUEUE_HPP_
#define VEHICLE_AGENT__STORAGE__OFFLINE_QUEUE_HPP_

#include "vehicle_agent/ir/command.hpp"
#include "vehicle_agent/ir/mission.hpp"

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

struct sqlite3;

namespace vehicle_agent
{

// Durable FIFO for incoming downlink IR (missions, commands). SQLite WAL
// journal so a crash mid-write still recovers a consistent file. Items are
// enqueued by adapter callbacks, replayed at startup, and acked when the
// state machine reaches a terminal state for the mission (or right after the
// FSM dispatches a command).
//
// Schema (single table, append-only with soft-delete via acked flag):
//   inbound (id INTEGER PRIMARY KEY AUTOINCREMENT,
//            kind TEXT NOT NULL,            -- 'mission' | 'command'
//            payload TEXT NOT NULL,         -- JSON
//            received_ms INTEGER NOT NULL,
//            acked INTEGER NOT NULL DEFAULT 0)
class OfflineQueue
{
public:
  struct Item
  {
    int64_t id{0};
    std::string kind;          // "mission" | "command"
    std::string payload_json;
    uint64_t received_ms{0};
  };

  // Opens (and creates if needed) the SQLite database at db_path. Sets WAL.
  // Throws std::runtime_error on failure.
  explicit OfflineQueue(const std::string & db_path);
  ~OfflineQueue();

  OfflineQueue(const OfflineQueue &) = delete;
  OfflineQueue & operator=(const OfflineQueue &) = delete;

  // Persist a mission; returns its row id (>0).
  int64_t enqueueMission(const ir::Mission & m);

  // Persist a command; returns its row id (>0).
  int64_t enqueueCommand(const ir::Command & c);

  // Returns up to `limit` unacked items in receive order (oldest first).
  std::vector<Item> peekPending(size_t limit = 1024) const;

  // Marks one row as acked.
  void ack(int64_t row_id);

  // Marks many rows as acked in a single transaction.
  void ackAll(const std::vector<int64_t> & ids);

  // Number of unacked rows.
  size_t pendingCount() const;

  // Vacuums acked rows older than `older_than_ms` epoch ms. Returns deleted count.
  size_t purgeAckedOlderThan(uint64_t older_than_ms);

  // Path the queue was opened against (for diagnostics).
  const std::string & path() const { return path_; }

private:
  void open();
  void exec(const char * sql);
  int64_t insertRow(const std::string & kind, const std::string & payload_json);

  mutable std::mutex mtx_;
  sqlite3 * db_{nullptr};
  std::string path_;
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__STORAGE__OFFLINE_QUEUE_HPP_
