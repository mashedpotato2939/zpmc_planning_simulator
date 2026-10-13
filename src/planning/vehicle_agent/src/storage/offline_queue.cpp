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
#include <sqlite3.h>

#include <chrono>
#include <filesystem>
#include <sstream>
#include <stdexcept>

namespace vehicle_agent
{

namespace
{
uint64_t now_ms()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

std::string err_text(sqlite3 * db, const std::string & op)
{
  std::ostringstream os;
  os << "OfflineQueue: " << op << " failed: " << (db ? sqlite3_errmsg(db) : "no db");
  return os.str();
}
}  // namespace

OfflineQueue::OfflineQueue(const std::string & db_path) : path_(db_path)
{
  open();
}

OfflineQueue::~OfflineQueue()
{
  if (db_) {
    sqlite3_close(db_);
    db_ = nullptr;
  }
}

void OfflineQueue::open()
{
  const auto parent = std::filesystem::path(path_).parent_path();
  if (!parent.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(parent, ec);
  }

  if (sqlite3_open(path_.c_str(), &db_) != SQLITE_OK) {
    auto msg = err_text(db_, "open");
    sqlite3_close(db_);
    db_ = nullptr;
    throw std::runtime_error(msg);
  }

  exec("PRAGMA journal_mode=WAL;");
  exec("PRAGMA synchronous=NORMAL;");
  exec("PRAGMA foreign_keys=ON;");
  exec(
    "CREATE TABLE IF NOT EXISTS inbound ("
    " id INTEGER PRIMARY KEY AUTOINCREMENT,"
    " kind TEXT NOT NULL,"
    " payload TEXT NOT NULL,"
    " received_ms INTEGER NOT NULL,"
    " acked INTEGER NOT NULL DEFAULT 0"
    ");");
  exec("CREATE INDEX IF NOT EXISTS idx_inbound_unacked ON inbound(acked, id);");
}

void OfflineQueue::exec(const char * sql)
{
  char * errmsg = nullptr;
  if (sqlite3_exec(db_, sql, nullptr, nullptr, &errmsg) != SQLITE_OK) {
    std::string msg = std::string("OfflineQueue: exec failed: ") +
      (errmsg ? errmsg : "(null)") + " sql=" + sql;
    if (errmsg) sqlite3_free(errmsg);
    throw std::runtime_error(msg);
  }
}

int64_t OfflineQueue::insertRow(const std::string & kind, const std::string & payload_json)
{
  std::lock_guard<std::mutex> lock(mtx_);
  const char * sql =
    "INSERT INTO inbound(kind, payload, received_ms, acked) VALUES(?, ?, ?, 0);";
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error(err_text(db_, "prepare insert"));
  }
  sqlite3_bind_text(stmt, 1, kind.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_text(stmt, 2, payload_json.c_str(), -1, SQLITE_TRANSIENT);
  sqlite3_bind_int64(stmt, 3, static_cast<sqlite3_int64>(now_ms()));
  if (sqlite3_step(stmt) != SQLITE_DONE) {
    sqlite3_finalize(stmt);
    throw std::runtime_error(err_text(db_, "step insert"));
  }
  const int64_t row_id = sqlite3_last_insert_rowid(db_);
  sqlite3_finalize(stmt);
  return row_id;
}

int64_t OfflineQueue::enqueueMission(const ir::Mission & m)
{
  return insertRow("mission", nlohmann::json(m).dump());
}

int64_t OfflineQueue::enqueueCommand(const ir::Command & c)
{
  return insertRow("command", nlohmann::json(c).dump());
}

std::vector<OfflineQueue::Item> OfflineQueue::peekPending(size_t limit) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  const char * sql =
    "SELECT id, kind, payload, received_ms FROM inbound WHERE acked = 0 "
    "ORDER BY id ASC LIMIT ?;";
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error(err_text(db_, "prepare peek"));
  }
  sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(limit));
  std::vector<Item> out;
  while (sqlite3_step(stmt) == SQLITE_ROW) {
    Item it;
    it.id = sqlite3_column_int64(stmt, 0);
    it.kind = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 1));
    it.payload_json = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 2));
    it.received_ms = static_cast<uint64_t>(sqlite3_column_int64(stmt, 3));
    out.push_back(std::move(it));
  }
  sqlite3_finalize(stmt);
  return out;
}

void OfflineQueue::ack(int64_t row_id)
{
  std::lock_guard<std::mutex> lock(mtx_);
  const char * sql = "UPDATE inbound SET acked = 1 WHERE id = ?;";
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error(err_text(db_, "prepare ack"));
  }
  sqlite3_bind_int64(stmt, 1, row_id);
  if (sqlite3_step(stmt) != SQLITE_DONE) {
    sqlite3_finalize(stmt);
    throw std::runtime_error(err_text(db_, "step ack"));
  }
  sqlite3_finalize(stmt);
}

void OfflineQueue::ackAll(const std::vector<int64_t> & ids)
{
  if (ids.empty()) return;
  std::lock_guard<std::mutex> lock(mtx_);
  char * errmsg = nullptr;
  if (sqlite3_exec(db_, "BEGIN;", nullptr, nullptr, &errmsg) != SQLITE_OK) {
    std::string m = errmsg ? errmsg : "begin failed";
    if (errmsg) sqlite3_free(errmsg);
    throw std::runtime_error("OfflineQueue: " + m);
  }
  const char * sql = "UPDATE inbound SET acked = 1 WHERE id = ?;";
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    sqlite3_exec(db_, "ROLLBACK;", nullptr, nullptr, nullptr);
    throw std::runtime_error(err_text(db_, "prepare ackAll"));
  }
  for (auto id : ids) {
    sqlite3_reset(stmt);
    sqlite3_bind_int64(stmt, 1, id);
    if (sqlite3_step(stmt) != SQLITE_DONE) {
      sqlite3_finalize(stmt);
      sqlite3_exec(db_, "ROLLBACK;", nullptr, nullptr, nullptr);
      throw std::runtime_error(err_text(db_, "step ackAll"));
    }
  }
  sqlite3_finalize(stmt);
  if (sqlite3_exec(db_, "COMMIT;", nullptr, nullptr, &errmsg) != SQLITE_OK) {
    std::string m = errmsg ? errmsg : "commit failed";
    if (errmsg) sqlite3_free(errmsg);
    throw std::runtime_error("OfflineQueue: " + m);
  }
}

size_t OfflineQueue::pendingCount() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  const char * sql = "SELECT COUNT(*) FROM inbound WHERE acked = 0;";
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error(err_text(db_, "prepare count"));
  }
  size_t n = 0;
  if (sqlite3_step(stmt) == SQLITE_ROW) {
    n = static_cast<size_t>(sqlite3_column_int64(stmt, 0));
  }
  sqlite3_finalize(stmt);
  return n;
}

size_t OfflineQueue::purgeAckedOlderThan(uint64_t older_than_ms)
{
  std::lock_guard<std::mutex> lock(mtx_);
  const char * sql = "DELETE FROM inbound WHERE acked = 1 AND received_ms < ?;";
  sqlite3_stmt * stmt = nullptr;
  if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
    throw std::runtime_error(err_text(db_, "prepare purge"));
  }
  sqlite3_bind_int64(stmt, 1, static_cast<sqlite3_int64>(older_than_ms));
  if (sqlite3_step(stmt) != SQLITE_DONE) {
    sqlite3_finalize(stmt);
    throw std::runtime_error(err_text(db_, "step purge"));
  }
  sqlite3_finalize(stmt);
  return static_cast<size_t>(sqlite3_changes(db_));
}

}  // namespace vehicle_agent
