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

#ifndef VEHICLE_AGENT__SECURITY__CLOCK_SKEW_WINDOW_HPP_
#define VEHICLE_AGENT__SECURITY__CLOCK_SKEW_WINDOW_HPP_

#include <cstdint>

namespace vehicle_agent
{

// Symmetric clock-skew gate. Accepts a message timestamp iff it lies within
// ±tolerance_ms of the receiver's local time. Used together with NonceLRU
// to bound how far back a replay can come from, and to reject messages
// from a node whose clock has drifted.
//
// Depends on NTP being healthy on both sides; document this in ops notes.
class ClockSkewWindow
{
public:
  enum class Reason : uint8_t
  {
    OK,
    TOO_OLD,
    TOO_NEW,
  };

  explicit ClockSkewWindow(uint64_t tolerance_ms);

  // True iff |msg_ts_ms - now_ms| <= tolerance_ms.
  bool accepts(uint64_t msg_ts_ms, uint64_t now_ms) const;

  // Same as accepts() but also returns a rejection reason for diagnostics.
  Reason classify(uint64_t msg_ts_ms, uint64_t now_ms) const;

  uint64_t toleranceMs() const { return tolerance_ms_; }

private:
  uint64_t tolerance_ms_;
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__SECURITY__CLOCK_SKEW_WINDOW_HPP_
