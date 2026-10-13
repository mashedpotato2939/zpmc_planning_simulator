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

#ifndef VEHICLE_AGENT__IR__MISSION_HPP_
#define VEHICLE_AGENT__IR__MISSION_HPP_

#include <nlohmann/json.hpp>
#include <cstdint>
#include <string>
#include <vector>
#include <optional>

namespace vehicle_agent::ir
{

struct Waypoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  double target_speed{0.0};
  std::string segment_id;
  // Matches igv_planning_msgs::msg::SegmentWaypoint POINT_* enum
  // (NORMAL=0, DESTINATION=1, LC_START=2, LC_END=3, POI=4, LOCAL_PLAN_START=5,
  // LOCAL_PLAN_END=6). Carried end-to-end so fms_path_converter can locate
  // local-replan zones and lane-change markers regardless of which adapter
  // produced the mission.
  uint8_t point_type{0};
};

inline void to_json(nlohmann::json & j, const Waypoint & w)
{
  j = {{"x", w.x}, {"y", w.y}, {"z", w.z}, {"yaw", w.yaw},
       {"target_speed", w.target_speed}, {"segment_id", w.segment_id},
       {"point_type", w.point_type}};
}

inline void from_json(const nlohmann::json & j, Waypoint & w)
{
  j.at("x").get_to(w.x);
  j.at("y").get_to(w.y);
  w.z = j.value("z", 0.0);
  w.yaw = j.value("yaw", 0.0);
  w.target_speed = j.value("target_speed", 0.0);
  w.segment_id = j.value("segment_id", "");
  w.point_type = j.value("point_type", static_cast<uint8_t>(0));
}

enum class MissionKind : uint8_t
{
  ROUTE = 0,
  STOP_AT = 1,
  RESUME = 2,
  ABORT = 3,
  PARK = 4,
};

struct Mission
{
  std::string id;
  MissionKind kind{MissionKind::ROUTE};
  uint64_t seq{0};
  uint64_t issued_at_ms{0};
  std::vector<Waypoint> waypoints;
  std::optional<uint64_t> deadline_ms;
  std::string source_adapter;
};

inline void to_json(nlohmann::json & j, const Mission & m)
{
  j = {{"id", m.id}, {"kind", static_cast<uint8_t>(m.kind)}, {"seq", m.seq},
       {"issued_at_ms", m.issued_at_ms}, {"waypoints", m.waypoints},
       {"source_adapter", m.source_adapter}};
  if (m.deadline_ms) j["deadline_ms"] = *m.deadline_ms;
}

inline void from_json(const nlohmann::json & j, Mission & m)
{
  j.at("id").get_to(m.id);
  m.kind = static_cast<MissionKind>(j.value("kind", 0));
  m.seq = j.value("seq", 0ULL);
  m.issued_at_ms = j.value("issued_at_ms", 0ULL);
  m.waypoints = j.value("waypoints", std::vector<Waypoint>{});
  if (j.contains("deadline_ms")) m.deadline_ms = j.at("deadline_ms").get<uint64_t>();
  m.source_adapter = j.value("source_adapter", "");
}

}  // namespace vehicle_agent::ir
#endif  // VEHICLE_AGENT__IR__MISSION_HPP_
