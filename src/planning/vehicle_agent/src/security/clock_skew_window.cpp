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

#include "vehicle_agent/security/clock_skew_window.hpp"

namespace vehicle_agent
{

ClockSkewWindow::ClockSkewWindow(uint64_t tolerance_ms) : tolerance_ms_(tolerance_ms) {}

bool ClockSkewWindow::accepts(uint64_t msg_ts_ms, uint64_t now_ms) const
{
  return classify(msg_ts_ms, now_ms) == Reason::OK;
}

ClockSkewWindow::Reason ClockSkewWindow::classify(uint64_t msg_ts_ms, uint64_t now_ms) const
{
  if (msg_ts_ms > now_ms) {
    return (msg_ts_ms - now_ms) <= tolerance_ms_ ? Reason::OK : Reason::TOO_NEW;
  }
  return (now_ms - msg_ts_ms) <= tolerance_ms_ ? Reason::OK : Reason::TOO_OLD;
}

}  // namespace vehicle_agent
