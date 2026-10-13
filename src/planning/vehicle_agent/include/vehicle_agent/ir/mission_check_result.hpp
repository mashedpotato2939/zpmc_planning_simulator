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

#ifndef VEHICLE_AGENT__IR__MISSION_CHECK_RESULT_HPP_
#define VEHICLE_AGENT__IR__MISSION_CHECK_RESULT_HPP_

#include <nlohmann/json.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace vehicle_agent::ir
{

struct MissionCheckResult
{
  std::string mission_id;
  bool accepted{false};
  std::vector<std::string> reasons;   // empty when accepted
  uint64_t stamp_ms{0};
};

inline void to_json(nlohmann::json & j, const MissionCheckResult & r)
{
  j = {{"mission_id", r.mission_id}, {"accepted", r.accepted},
       {"reasons", r.reasons}, {"stamp_ms", r.stamp_ms}};
}

inline void from_json(const nlohmann::json & j, MissionCheckResult & r)
{
  r.mission_id = j.value("mission_id", "");
  r.accepted = j.value("accepted", false);
  r.reasons = j.value("reasons", std::vector<std::string>{});
  r.stamp_ms = j.value("stamp_ms", 0ULL);
}

}  // namespace vehicle_agent::ir
#endif  // VEHICLE_AGENT__IR__MISSION_CHECK_RESULT_HPP_
