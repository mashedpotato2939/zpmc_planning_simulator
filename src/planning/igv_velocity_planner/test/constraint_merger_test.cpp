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

#include <gtest/gtest.h>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <cmath>
#include <limits>
#include <vector>

namespace
{
using igv_velocity_planner::velocity_shaper::ConstraintType;
using igv_velocity_planner::velocity_shaper::mergeConstraints;
using igv_velocity_planner::velocity_shaper::Priority;
using igv_velocity_planner::velocity_shaper::VelocityConstraint;
using TrajectoryPoint = autoware_planning_msgs::msg::TrajectoryPoint;

constexpr double kInf = std::numeric_limits<double>::infinity();

// Straight path along +x, identity orientation, constant velocity.
std::vector<TrajectoryPoint> line(double length, double step, double vel)
{
  std::vector<TrajectoryPoint> pts;
  for (double x = 0.0; x <= length + 1e-9; x += step) {
    TrajectoryPoint p;
    p.pose.position.x = x;
    p.pose.orientation.w = 1.0;
    p.longitudinal_velocity_mps = static_cast<float>(vel);
    pts.push_back(p);
  }
  return pts;
}

VelocityConstraint mk(
  double s0, double s1, double v, ConstraintType type)
{
  VelocityConstraint c;
  c.start_s = s0;
  c.end_s = s1;
  c.target_velocity = v;
  c.type = type;
  c.priority = igv_velocity_planner::velocity_shaper::priorityOf(type);
  c.source = "test";
  return c;
}

// Velocity of the point whose x is nearest to `x`.
float vAtX(const std::vector<TrajectoryPoint> & pts, double x)
{
  size_t best = 0;
  double bd = 1e18;
  for (size_t i = 0; i < pts.size(); ++i) {
    const double d = std::abs(pts[i].pose.position.x - x);
    if (d < bd) {
      bd = d;
      best = i;
    }
  }
  return pts[best].longitudinal_velocity_mps;
}
}  // namespace

// A single cap lowers velocity by min; nothing is raised.
TEST(ConstraintMerger, SingleCapAppliesMin)
{
  auto pts = line(100.0, 1.0, 8.0);
  std::vector<VelocityConstraint> cs{mk(0.0, kInf, 4.0, ConstraintType::kAdaptiveCruise)};
  const auto mr = mergeConstraints(pts, cs);
  EXPECT_NEAR(vAtX(pts, 10.0), 4.0, 1e-5);
  EXPECT_NEAR(vAtX(pts, 90.0), 4.0, 1e-5);
  EXPECT_EQ(mr.winner[10], ConstraintType::kAdaptiveCruise);
}

// Monotonic guard: a HIGHER-priority constraint with a LARGER target must not
// raise velocity above a lower-priority smaller cap. The smaller cap is binding.
TEST(ConstraintMerger, HigherPriorityNeverRaisesVelocity)
{
  auto pts = line(50.0, 1.0, 8.0);
  std::vector<VelocityConstraint> cs{
    mk(0.0, kInf, 2.0, ConstraintType::kSlowDown),       // prio 20, cap 2
    mk(0.0, kInf, 6.0, ConstraintType::kOccupancyStop),  // prio 90, cap 6 (larger!)
  };
  const auto mr = mergeConstraints(pts, cs);
  EXPECT_NEAR(vAtX(pts, 25.0), 2.0, 1e-5);  // stays at the smaller cap
  EXPECT_EQ(mr.winner[25], ConstraintType::kSlowDown);  // binding constraint wins
}

// A windowed slow-down depresses velocity only inside its interval; the
// trajectory recovers afterward (the non-monotonic case if+min handles poorly).
TEST(ConstraintMerger, WindowedSlowDownRecovers)
{
  auto pts = line(100.0, 1.0, 8.0);
  std::vector<VelocityConstraint> cs{
    mk(0.0, kInf, 4.0, ConstraintType::kAdaptiveCruise),  // global 4
    mk(55.0, 65.0, 3.0, ConstraintType::kSlowDown),       // dip to 3 in [55,65]
  };
  const auto mr = mergeConstraints(pts, cs);
  EXPECT_NEAR(vAtX(pts, 40.0), 4.0, 1e-5);
  EXPECT_NEAR(vAtX(pts, 60.0), 3.0, 1e-5);
  EXPECT_NEAR(vAtX(pts, 80.0), 4.0, 1e-5);  // recovers after the window
  EXPECT_EQ(mr.winner[60], ConstraintType::kSlowDown);
}

// The worked example from velocity_planner_optimizer.TXT: FMS 8, ACC 4,
// slow-down 3 over [55,65], dynamic stop @32, occupancy stop @17. The occupancy
// stop dominates everything past 17; ACC owns 0..17.
TEST(ConstraintMerger, WorkedExampleOccupancyDominatesPast17)
{
  auto pts = line(100.0, 1.0, 8.0);
  std::vector<VelocityConstraint> cs{
    mk(0.0, kInf, 4.0, ConstraintType::kAdaptiveCruise),
    mk(55.0, 65.0, 3.0, ConstraintType::kSlowDown),
    mk(32.0, kInf, 0.0, ConstraintType::kDynamicStop),
    mk(17.0, kInf, 0.0, ConstraintType::kOccupancyStop),
  };
  const auto mr = mergeConstraints(pts, cs);
  EXPECT_NEAR(vAtX(pts, 10.0), 4.0, 1e-5);   // ACC owns the pre-stop region
  EXPECT_NEAR(vAtX(pts, 20.0), 0.0, 1e-5);   // occupancy stop from 17
  EXPECT_NEAR(vAtX(pts, 60.0), 0.0, 1e-5);   // still stopped (slow-down masked)
  // Past the dynamic stop both stops are active; the higher-priority occupancy
  // stop names the result.
  EXPECT_EQ(vAtX(pts, 40.0), 0.0f);
}

// Two stops on a sparse 2-point/30 m path land at exact arc-length and zero
// everything downstream — the index-shift regression guard.
TEST(ConstraintMerger, MultipleStopsSparsePathExactInsertion)
{
  std::vector<TrajectoryPoint> pts;
  for (double x : {0.0, 30.0}) {
    TrajectoryPoint p;
    p.pose.position.x = x;
    p.pose.orientation.w = 1.0;
    p.longitudinal_velocity_mps = 5.0f;
    pts.push_back(p);
  }
  std::vector<VelocityConstraint> cs{
    mk(11.0, kInf, 0.0, ConstraintType::kDynamicStop),
    mk(26.0, kInf, 0.0, ConstraintType::kOccupancyStop),
  };
  const auto mr = mergeConstraints(pts, cs);

  ASSERT_EQ(pts.size(), 4u);  // two exact stop points inserted
  EXPECT_NEAR(pts[0].pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(pts[1].pose.position.x, 11.0, 1e-6);
  EXPECT_NEAR(pts[2].pose.position.x, 26.0, 1e-6);
  EXPECT_NEAR(pts[3].pose.position.x, 30.0, 1e-6);
  EXPECT_NEAR(pts[0].longitudinal_velocity_mps, 5.0, 1e-5);  // before first stop
  EXPECT_NEAR(pts[1].longitudinal_velocity_mps, 0.0, 1e-5);
  EXPECT_NEAR(pts[2].longitudinal_velocity_mps, 0.0, 1e-5);
  EXPECT_NEAR(pts[3].longitudinal_velocity_mps, 0.0, 1e-5);
  EXPECT_EQ(mr.v_limit.size(), pts.size());  // provenance stays in lockstep
}

// No constraints → trajectory unchanged (FMS base passes through).
TEST(ConstraintMerger, NoConstraintsPassThrough)
{
  auto pts = line(20.0, 1.0, 6.0);
  const auto mr = mergeConstraints(pts, {});
  for (const auto & p : pts) EXPECT_NEAR(p.longitudinal_velocity_mps, 6.0, 1e-5);
  EXPECT_EQ(mr.winner[5], ConstraintType::kFmsBase);
}

// Stop at s<=0 zeroes the whole trajectory in place (no spurious insertion).
TEST(ConstraintMerger, StopAtOriginZeroesAll)
{
  auto pts = line(20.0, 1.0, 6.0);
  const size_t n0 = pts.size();
  std::vector<VelocityConstraint> cs{mk(0.0, kInf, 0.0, ConstraintType::kOccupancyStop)};
  mergeConstraints(pts, cs);
  EXPECT_EQ(pts.size(), n0);  // start_s==0 hits the front point, no insertion
  for (const auto & p : pts) EXPECT_NEAR(p.longitudinal_velocity_mps, 0.0, 1e-5);
}
