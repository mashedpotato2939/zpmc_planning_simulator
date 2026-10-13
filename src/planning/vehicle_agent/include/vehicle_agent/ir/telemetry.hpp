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

#ifndef VEHICLE_AGENT__IR__TELEMETRY_HPP_
#define VEHICLE_AGENT__IR__TELEMETRY_HPP_

#include <nlohmann/json.hpp>
#include <cstdint>
#include <string>

namespace vehicle_agent::ir
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Telemetry
{
  std::string vehicle_id;
  uint64_t stamp_ms{0};
  Pose2D pose;
  double speed_mps{0.0};
  double steering_rad{0.0};
  std::string current_mission_id;
  uint8_t mission_state{0};   // mirrors ir::MissionState
  uint8_t op_mode{0};         // 0=manual, 1=auto, 2=remote
  bool e_stop_active{false};
};

inline void to_json(nlohmann::json & j, const Telemetry & t)
{
  j = {{"vehicle_id", t.vehicle_id}, {"stamp_ms", t.stamp_ms},
       {"x", t.pose.x}, {"y", t.pose.y}, {"yaw", t.pose.yaw},
       {"speed_mps", t.speed_mps}, {"steering_rad", t.steering_rad},
       {"current_mission_id", t.current_mission_id},
       {"mission_state", t.mission_state}, {"op_mode", t.op_mode},
       {"e_stop_active", t.e_stop_active}};
}

inline void from_json(const nlohmann::json & j, Telemetry & t)
{
  t.vehicle_id = j.value("vehicle_id", "");
  t.stamp_ms = j.value("stamp_ms", 0ULL);
  t.pose.x = j.value("x", 0.0);
  t.pose.y = j.value("y", 0.0);
  t.pose.yaw = j.value("yaw", 0.0);
  t.speed_mps = j.value("speed_mps", 0.0);
  t.steering_rad = j.value("steering_rad", 0.0);
  t.current_mission_id = j.value("current_mission_id", "");
  t.mission_state = j.value("mission_state", 0);
  t.op_mode = j.value("op_mode", 0);
  t.e_stop_active = j.value("e_stop_active", false);
}

}  // namespace vehicle_agent::ir
#endif  // VEHICLE_AGENT__IR__TELEMETRY_HPP_
