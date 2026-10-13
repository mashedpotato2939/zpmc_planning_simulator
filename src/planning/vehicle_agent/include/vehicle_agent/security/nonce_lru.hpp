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

#ifndef VEHICLE_AGENT__SECURITY__NONCE_LRU_HPP_
#define VEHICLE_AGENT__SECURITY__NONCE_LRU_HPP_

#include <cstddef>
#include <cstdint>
#include <list>
#include <mutex>
#include <string>
#include <unordered_map>

namespace vehicle_agent
{

// Bounded LRU of recently-seen nonces with TTL eviction. Protects the
// inbound path from replays of valid-MAC'd messages. Two eviction policies
// act simultaneously:
//   - capacity: when size exceeds `max_entries`, oldest entry is dropped.
//   - TTL: entries older than `ttl_ms` are pruned lazily on each access.
//
// Thread-safe. seenAndRecord is the only mutating entry point: it returns
// true if the nonce was already in the store (i.e. a replay), false if it
// was new and has been recorded.
class NonceLRU
{
public:
  NonceLRU(size_t max_entries, uint64_t ttl_ms);

  // Returns true if `nonce` was seen before within the TTL window (replay).
  // Otherwise records (nonce, ts_ms) and returns false.
  bool seenAndRecord(const std::string & nonce, uint64_t ts_ms);

  // Read-only check (does not record). Mostly for tests/diagnostics.
  bool contains(const std::string & nonce) const;

  size_t size() const;

private:
  // Prune entries with timestamp older than (now_ms - ttl_ms). Caller holds
  // the lock.
  void pruneExpired(uint64_t now_ms);
  // Evict oldest entry. Caller holds the lock.
  void evictOldest();

  struct Entry
  {
    std::string nonce;
    uint64_t ts_ms;
  };

  mutable std::mutex mtx_;
  size_t max_entries_;
  uint64_t ttl_ms_;
  std::list<Entry> order_;   // front = oldest, back = newest
  std::unordered_map<std::string, std::list<Entry>::iterator> index_;
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__SECURITY__NONCE_LRU_HPP_
