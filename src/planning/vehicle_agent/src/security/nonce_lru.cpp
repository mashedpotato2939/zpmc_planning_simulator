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

#include "vehicle_agent/security/nonce_lru.hpp"

#include <algorithm>

namespace vehicle_agent
{

NonceLRU::NonceLRU(size_t max_entries, uint64_t ttl_ms)
: max_entries_(max_entries == 0 ? 1 : max_entries), ttl_ms_(ttl_ms)
{
}

bool NonceLRU::seenAndRecord(const std::string & nonce, uint64_t ts_ms)
{
  std::lock_guard<std::mutex> lock(mtx_);
  pruneExpired(ts_ms);

  auto it = index_.find(nonce);
  if (it != index_.end()) {
    // Replay: refresh the timestamp so a flood of replays does not let an
    // even-older replay sneak past TTL afterwards.
    it->second->ts_ms = ts_ms;
    order_.splice(order_.end(), order_, it->second);
    return true;
  }

  order_.push_back(Entry{nonce, ts_ms});
  auto last = std::prev(order_.end());
  index_.emplace(nonce, last);

  while (index_.size() > max_entries_) {
    evictOldest();
  }
  return false;
}

bool NonceLRU::contains(const std::string & nonce) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return index_.find(nonce) != index_.end();
}

size_t NonceLRU::size() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return index_.size();
}

void NonceLRU::pruneExpired(uint64_t now_ms)
{
  if (ttl_ms_ == 0) return;
  while (!order_.empty()) {
    const auto & front = order_.front();
    if (now_ms < front.ts_ms || (now_ms - front.ts_ms) <= ttl_ms_) break;
    index_.erase(front.nonce);
    order_.pop_front();
  }
}

void NonceLRU::evictOldest()
{
  if (order_.empty()) return;
  index_.erase(order_.front().nonce);
  order_.pop_front();
}

}  // namespace vehicle_agent
