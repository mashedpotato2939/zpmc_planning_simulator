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

#include "igv_velocity_planner/velocity_shaper/constraint_merger.hpp"

#include "igv_velocity_planner/velocity_shaper/geometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace igv_velocity_planner::velocity_shaper
{

namespace
{
constexpr double kEps = 1e-6;

// A v=0, half-open-to-infinity constraint that becomes an exact stop point.
struct StopRequest
{
  double start_s;
  ConstraintType type;
  int priority;
  std::string source;
  std::optional<geometry_msgs::msg::Point> anchor;
};

// A genuine full stop: holds 0 from start_s to the end of the trajectory.
// Graded slow-downs that ramp to ~0 over a *finite* window are NOT stops —
// they are ordinary interval caps handled in the resolution pass.
bool isFullStop(const VelocityConstraint & c)
{
  return std::isinf(c.end_s) && std::abs(c.target_velocity) <= kEps;
}
}  // namespace

MergeResult mergeConstraints(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const std::vector<VelocityConstraint> & constraints)
{
  MergeResult mr;
  const size_t n = points.size();
  if (n == 0) return mr;

  // 1) Immutable arc-length axis for the resolution pass.
  std::vector<double> s(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    s[i] = s[i - 1] + distance2D(points[i].pose.position, points[i - 1].pose.position);
  }
  const double s_total = s[n - 1];

  // 2) Seed with the FMS base (each point's own velocity).
  mr.v_limit.assign(n, 0.0);
  mr.winner.assign(n, ConstraintType::kFmsBase);
  mr.winner_priority.assign(n, kPrioFmsBase);
  mr.source.assign(n, "fms_base");
  mr.anchor.assign(n, std::nullopt);
  for (size_t i = 0; i < n; ++i) {
    mr.v_limit[i] = static_cast<double>(points[i].longitudinal_velocity_mps);
  }

  // 3) Resolve on the existing samples. Process in ascending priority so that
  //    among constraints binding at the same cap, the highest-priority one
  //    names the result. Numerically this is a pure min() (monotonic guard:
  //    a constraint never raises velocity), identical to the legacy if+min.
  std::vector<const VelocityConstraint *> sorted;
  sorted.reserve(constraints.size());
  for (const auto & c : constraints) sorted.push_back(&c);
  std::stable_sort(
    sorted.begin(), sorted.end(),
    [](const VelocityConstraint * a, const VelocityConstraint * b) {
      return a->priority < b->priority;
    });

  for (const auto * c : sorted) {
    for (size_t i = 0; i < n; ++i) {
      if (s[i] < c->start_s || s[i] >= c->end_s) continue;
      const double cap = c->target_velocity;
      if (cap < mr.v_limit[i] - kEps) {
        // Strictly tighter cap: this constraint is now binding → it wins.
        mr.v_limit[i] = cap;
        mr.winner[i] = c->type;
        mr.winner_priority[i] = c->priority;
        mr.source[i] = c->source;
        mr.anchor[i] = c->anchor;
      } else if (cap <= mr.v_limit[i] + kEps && c->priority >= mr.winner_priority[i]) {
        // Ties at (near-)equal cap: higher-or-equal priority names the result.
        mr.v_limit[i] = std::min(mr.v_limit[i], cap);
        mr.winner[i] = c->type;
        mr.winner_priority[i] = c->priority;
        mr.source[i] = c->source;
        mr.anchor[i] = c->anchor;
      }
      // else: cap clearly higher → not binding, leave untouched (never raise).
    }
  }

  // 4) Insert exact stop points for full-stop constraints, back-to-front so
  //    earlier (smaller-s) insertions never shift not-yet-processed indices.
  //    The parallel provenance vectors are kept in lockstep with `points`.
  std::vector<StopRequest> stops;
  for (const auto & c : constraints) {
    if (!isFullStop(c)) continue;
    const double s_stop = std::clamp(c.start_s, 0.0, s_total);
    stops.push_back({s_stop, c.type, c.priority, c.source, c.anchor});
  }
  std::sort(stops.begin(), stops.end(), [](const StopRequest & a, const StopRequest & b) {
    return a.start_s > b.start_s;  // descending
  });

  for (const auto & req : stops) {
    const size_t before = points.size();
    const size_t idx = insertStopAtArcLength(points, req.start_s);
    if (points.size() > before) {
      // A new point was inserted at idx — mirror it into the provenance arrays.
      mr.v_limit.insert(mr.v_limit.begin() + idx, 0.0);
      mr.winner.insert(mr.winner.begin() + idx, req.type);
      mr.winner_priority.insert(mr.winner_priority.begin() + idx, req.priority);
      mr.source.insert(mr.source.begin() + idx, req.source);
      mr.anchor.insert(mr.anchor.begin() + idx, req.anchor);
    } else {
      // No insertion (s_stop<=0 → front, or past-end → back): update in place,
      // and let priority decide whether this stop names the existing point.
      if (req.priority >= mr.winner_priority[idx] || mr.v_limit[idx] > kEps) {
        mr.winner[idx] = req.type;
        mr.winner_priority[idx] = req.priority;
        mr.source[idx] = req.source;
        mr.anchor[idx] = req.anchor;
      }
      mr.v_limit[idx] = 0.0;
    }
    // Everything from the stop onward must be 0 (matches capVelocityFrom(...,0)).
    for (size_t j = idx; j < mr.v_limit.size(); ++j) mr.v_limit[j] = 0.0;
  }

  // 5) Write the resolved velocities back into the (possibly grown) trajectory.
  for (size_t i = 0; i < points.size(); ++i) {
    points[i].longitudinal_velocity_mps = static_cast<float>(mr.v_limit[i]);
  }
  return mr;
}

}  // namespace igv_velocity_planner::velocity_shaper
