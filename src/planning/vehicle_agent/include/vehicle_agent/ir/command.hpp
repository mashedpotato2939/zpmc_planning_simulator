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

#ifndef VEHICLE_AGENT__IR__COMMAND_HPP_
#define VEHICLE_AGENT__IR__COMMAND_HPP_

#include <nlohmann/json.hpp>
#include <cstdint>
#include <string>

namespace vehicle_agent::ir
{

enum class CommandKind : uint8_t
{
  PAUSE = 0,
  RESUME = 1,
  ABORT = 2,
  ESTOP = 3,
  CLEAR_FAULT = 4,
};

struct Command
{
  std::string mission_id;
  CommandKind kind{CommandKind::PAUSE};
  uint64_t issued_at_ms{0};
  std::string source_adapter;
};

inline void to_json(nlohmann::json & j, const Command & c)
{
  j = {{"mission_id", c.mission_id}, {"kind", static_cast<uint8_t>(c.kind)},
       {"issued_at_ms", c.issued_at_ms}, {"source_adapter", c.source_adapter}};
}

inline void from_json(const nlohmann::json & j, Command & c)
{
  c.mission_id = j.value("mission_id", "");
  c.kind = static_cast<CommandKind>(j.value("kind", 0));
  c.issued_at_ms = j.value("issued_at_ms", 0ULL);
  c.source_adapter = j.value("source_adapter", "");
}

}  // namespace vehicle_agent::ir
#endif  // VEHICLE_AGENT__IR__COMMAND_HPP_
