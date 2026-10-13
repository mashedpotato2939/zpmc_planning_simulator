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

#ifndef FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__CONSTRAINT_HPP_
#define FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__CONSTRAINT_HPP_

#include <geometry_msgs/msg/point.hpp>

#include <limits>
#include <optional>
#include <string>

namespace igv_velocity_planner::velocity_shaper
{

// The kind of limit a constraint represents. Used for diagnostics (which
// pathway shaped the velocity) and to look up the canonical priority.
enum class ConstraintType {
  kFmsBase,            // the path's own per-point speed (baseline)
  kAdaptiveCruise,     // follow-leader cap
  kSlowDown,           // outer-band localized dip (object or graded occupancy)
  kStaticObstacleStop, // static obstacle whose footprint intrudes the ego corridor → stop
  kDynamicStop,        // predicted-collision stop
  kOccupancyStop,      // free-space corridor intrusion stop
  // future: kTrafficLight, kStation, kManualOverride, kSpeedZone, kAeb
};

// Resolution rank. Higher priority records provenance first, but the merger
// applies a *monotonic guard*: a constraint never raises velocity, it only
// lowers it (see constraint_merger.cpp). So a higher-priority constraint
// "wins" the reported source even when a lower-priority cap is also active,
// while the numeric result stays equal to a pure min() — identical to the
// legacy if+min clipper for the limit-only generators we have today.
enum Priority : int {
  kPrioFmsBase            = 0,
  kPrioSlowDown           = 20,
  kPrioAdaptiveCruise     = 50,
  kPrioStaticObstacleStop = 70,
  kPrioDynamicStop        = 80,
  kPrioOccupancyStop      = 90,
  kPrioAeb                = 100,  // reserved
};

// A speed limit over an arc-length interval [start_s, end_s). This is the
// single vocabulary every generator speaks; generators never mutate the
// trajectory, they only describe limits. end_s == +inf means "to the end".
struct VelocityConstraint
{
  double         start_s{0.0};
  double         end_s{std::numeric_limits<double>::infinity()};
  double         target_velocity{0.0};   // [m/s] cap applied inside [start_s, end_s)
  ConstraintType type{ConstraintType::kFmsBase};
  int            priority{kPrioFmsBase};
  std::string    source;                 // human-readable origin, e.g. "obj:<uuid>"
  // Optional diagnostic anchor: the obstacle/cell that caused this constraint.
  std::optional<geometry_msgs::msg::Point> anchor;
};

// Canonical priority for a type. Single source of truth so a constraint's
// priority is never set inconsistently with its type. (Graded occupancy
// slow-down deliberately emits type kSlowDown — not kOccupancyStop — so it
// composes by min like any other slow-down; only the hard occupancy stop
// carries kOccupancyStop. See occupancy_stop.cpp.)
inline int priorityOf(ConstraintType t)
{
  switch (t) {
    case ConstraintType::kFmsBase:        return kPrioFmsBase;
    case ConstraintType::kSlowDown:            return kPrioSlowDown;
    case ConstraintType::kAdaptiveCruise:      return kPrioAdaptiveCruise;
    case ConstraintType::kStaticObstacleStop:  return kPrioStaticObstacleStop;
    case ConstraintType::kDynamicStop:         return kPrioDynamicStop;
    case ConstraintType::kOccupancyStop:       return kPrioOccupancyStop;
  }
  return kPrioFmsBase;
}

}  // namespace igv_velocity_planner::velocity_shaper

#endif  // FMS_TRAJECTORY_BRIDGE__VELOCITY_SHAPER__CONSTRAINT_HPP_
