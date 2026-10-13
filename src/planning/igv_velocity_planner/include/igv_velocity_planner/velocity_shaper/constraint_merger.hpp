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

#ifndef FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__CONSTRAINT_MERGER_HPP_
#define FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__CONSTRAINT_MERGER_HPP_

#include "igv_velocity_planner/velocity_shaper/constraint.hpp"

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <optional>
#include <string>
#include <vector>

namespace igv_velocity_planner::velocity_shaper
{

// Per-point provenance of the merged velocity profile. Every vector is
// aligned with the FINAL `points` vector (after any stop-point insertions),
// so `winner[i]`/`source[i]`/`anchor[i]` describe what shaped `points[i]`.
struct MergeResult
{
  std::vector<double> v_limit;                              // resolved cap per point
  std::vector<ConstraintType> winner;                       // type that won at i
  std::vector<int> winner_priority;                         // priority that won at i
  std::vector<std::string> source;                          // concrete source string at i
  std::vector<std::optional<geometry_msgs::msg::Point>> anchor;  // diagnostic anchor at i
};

// Resolve all constraints onto the trajectory and write the result back.
//
// Semantics (the single place velocity resolution lives):
//   1. v_limit seeded from each point's own longitudinal_velocity_mps (FMS base).
//   2. Constraints applied in priority order. A constraint *records provenance*
//      whenever its priority >= the current winner, but it only ever *lowers*
//      v_limit (monotonic guard) — so the numeric profile equals a pure min()
//      over all constraints, identical to the legacy if+min clipper, while the
//      winner/source reflect the highest-priority active limit.
//   3. Stop points (target_velocity == 0) are inserted at exact arc-length once,
//      back-to-front, so insertions never invalidate not-yet-processed indices;
//      every sample downstream of a stop is forced to 0.
//
// `points` is mutated: velocities lowered, stop points inserted. The returned
// MergeResult is aligned with the post-insertion `points`.
MergeResult mergeConstraints(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const std::vector<VelocityConstraint> & constraints);

}  // namespace igv_velocity_planner::velocity_shaper

#endif  // FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__CONSTRAINT_MERGER_HPP_
