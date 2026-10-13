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

#ifndef VEHICLE_AGENT__IR__ACK_HPP_
#define VEHICLE_AGENT__IR__ACK_HPP_

#include <nlohmann/json.hpp>
#include <cstdint>
#include <string>

namespace vehicle_agent::ir
{

enum class AckStatus : uint8_t
{
  OK = 0,
  REJECTED = 1,
  PREEMPTED = 2,
  FAILED = 3,
};

struct Ack
{
  std::string target_id;        // mission_id or command id being acked
  AckStatus status{AckStatus::OK};
  std::string detail;
  uint64_t stamp_ms{0};
};

inline void to_json(nlohmann::json & j, const Ack & a)
{
  j = {{"target_id", a.target_id}, {"status", static_cast<uint8_t>(a.status)},
       {"detail", a.detail}, {"stamp_ms", a.stamp_ms}};
}

inline void from_json(const nlohmann::json & j, Ack & a)
{
  a.target_id = j.value("target_id", "");
  a.status = static_cast<AckStatus>(j.value("status", 0));
  a.detail = j.value("detail", "");
  a.stamp_ms = j.value("stamp_ms", 0ULL);
}

}  // namespace vehicle_agent::ir
#endif  // VEHICLE_AGENT__IR__ACK_HPP_
