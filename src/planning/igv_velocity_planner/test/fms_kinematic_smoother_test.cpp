// Copyright 2026 ZPMC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unit tests for the kinematic-smoother trajectory utilities. Tests live at
// the trajectory_utils level because the higher-level smoothing pipeline
// (calcInitialMotion, smoothVelocity, …) is private to the node and needs
// full ROS state to exercise — the bag-replay regression plan in
// docs/.../velocity_planner_simplification_plan.md covers that level.

#include "igv_velocity_planner/kinematic_smoother/resample.hpp"
#include "igv_velocity_planner/kinematic_smoother/trajectory_utils.hpp"

#include <gtest/gtest.h>

#include <map>
#include <vector>

namespace
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
namespace rs = igv_velocity_planner::kinematic_smoother::resampling;
namespace tu = igv_velocity_planner::kinematic_smoother::trajectory_utils;

TrajectoryPoints makeStraight(double length, double step, double velocity)
{
  TrajectoryPoints out;
  for (double s = 0.0; s <= length + 1e-9; s += step) {
    TrajectoryPoint tp;
    tp.pose.position.x = s;
    tp.pose.position.y = 0.0;
    tp.pose.orientation.w = 1.0;
    tp.longitudinal_velocity_mps = static_cast<float>(velocity);
    tp.acceleration_mps2 = 0.0f;
    out.push_back(tp);
  }
  return out;
}
}  // namespace

// ---------------------------------------------------------------------------
// calcArclengthArray
// ---------------------------------------------------------------------------

TEST(KinematicSmootherTrajUtils, ArclengthArrayMonotonic)
{
  const auto pts = makeStraight(10.0, 1.0, 5.0);
  const auto arc = tu::calcArclengthArray(pts);
  ASSERT_EQ(arc.size(), pts.size());
  EXPECT_DOUBLE_EQ(arc.front(), 0.0);
  for (size_t i = 1; i < arc.size(); ++i) {
    EXPECT_GE(arc[i], arc[i - 1]);
  }
  EXPECT_NEAR(arc.back(), 10.0, 1e-6);
}

TEST(KinematicSmootherTrajUtils, ArclengthEmptyInput)
{
  TrajectoryPoints empty;
  EXPECT_TRUE(tu::calcArclengthArray(empty).empty());
}

// ---------------------------------------------------------------------------
// extractPathAroundIndex
// ---------------------------------------------------------------------------

TEST(KinematicSmootherTrajUtils, ExtractAroundIndexRespectsBounds)
{
  const auto pts = makeStraight(50.0, 1.0, 5.0);
  // Around index 25, take 5 m behind + 10 m ahead. The implementation
  // overshoots by one segment in each direction (loop terminates when
  // dist_sum *strictly* exceeds the budget), so the actual envelope is
  // roughly [25-5-1, 25+10+1] = [19, 36].
  const auto extracted = tu::extractPathAroundIndex(pts, /*index=*/25, /*ahead=*/10.0, /*behind=*/5.0);
  ASSERT_FALSE(extracted.empty());
  EXPECT_GE(extracted.front().pose.position.x, 25.0 - 5.0 - 1.0 - 1e-6);
  EXPECT_LE(extracted.back().pose.position.x, 25.0 + 10.0 + 1.0 + 1e-6);
  // The extracted range must include the around-index point.
  const double front_x = extracted.front().pose.position.x;
  const double back_x = extracted.back().pose.position.x;
  EXPECT_LE(front_x, 25.0);
  EXPECT_GE(back_x, 25.0);
}

TEST(KinematicSmootherTrajUtils, ExtractInvalidIndexReturnsEmpty)
{
  const auto pts = makeStraight(10.0, 1.0, 5.0);
  EXPECT_TRUE(tu::extractPathAroundIndex(pts, /*index=*/999, 1.0, 1.0).empty());
}

// ---------------------------------------------------------------------------
// applyMaximumVelocityLimit
// ---------------------------------------------------------------------------

TEST(KinematicSmootherTrajUtils, MaxVelocityLimitClipsRangeOnly)
{
  auto pts = makeStraight(10.0, 1.0, 5.0);
  // Clip points [3, 7) to 2 m/s. Outside that range, values stay 5.
  tu::applyMaximumVelocityLimit(/*from=*/3, /*to=*/7, /*max_vel=*/2.0, pts);
  for (size_t i = 0; i < pts.size(); ++i) {
    if (i >= 3 && i < 7) {
      EXPECT_FLOAT_EQ(pts[i].longitudinal_velocity_mps, 2.0f);
    } else {
      EXPECT_FLOAT_EQ(pts[i].longitudinal_velocity_mps, 5.0f);
    }
  }
}

TEST(KinematicSmootherTrajUtils, MaxVelocityLimitNeverIncreases)
{
  auto pts = makeStraight(5.0, 1.0, 1.0);
  // Asking for a cap of 10 (above current 1.0) must not raise any value.
  tu::applyMaximumVelocityLimit(0, pts.size(), 10.0, pts);
  for (const auto & p : pts) {
    EXPECT_FLOAT_EQ(p.longitudinal_velocity_mps, 1.0f);
  }
}

// ---------------------------------------------------------------------------
// calcStopDistance
// ---------------------------------------------------------------------------

TEST(KinematicSmootherTrajUtils, StopDistanceFindsZeroVelocityIndex)
{
  auto pts = makeStraight(10.0, 1.0, 5.0);
  pts[5].longitudinal_velocity_mps = 0.0f;  // stop at x=5
  pts.back().longitudinal_velocity_mps = 0.0f;
  const double dist = tu::calcStopDistance(pts, /*closest=*/0);
  EXPECT_NEAR(dist, 5.0, 1e-6);
}

TEST(KinematicSmootherTrajUtils, StopDistanceNoStopReturnsLarge)
{
  const auto pts = makeStraight(10.0, 1.0, 5.0);
  const double dist = tu::calcStopDistance(pts, /*closest=*/0);
  EXPECT_GT(dist, 1e6);  // returns std::numeric_limits<double>::max() when no stop
}

// ---------------------------------------------------------------------------
// calcStopDistWithJerkConstraints
//
// This is the safety-critical kinematic predicate: given (v0, a0, jerk_acc,
// jerk_dec, min_acc, target_vel), compute the minimum braking distance and
// the corresponding jerk profile. Asserts the predicate succeeds for a
// representative input and that the returned profile sums to a positive
// distance.
// ---------------------------------------------------------------------------

TEST(KinematicSmootherTrajUtils, CalcStopDistWithJerkConstraintsBrakeToZero)
{
  std::map<double, double> jerk_profile;
  double stop_dist = 0.0;
  // From 5 m/s, no initial accel, brake to 0 with limits matching the
  // shipped Analytical defaults.
  const bool ok = tu::calcStopDistWithJerkConstraints(
    /*v0=*/5.0, /*a0=*/0.0, /*jerk_acc=*/0.3, /*jerk_dec=*/-0.3,
    /*min_acc=*/-2.5, /*target_vel=*/0.0, jerk_profile, stop_dist);
  EXPECT_TRUE(ok);
  EXPECT_GT(stop_dist, 0.0);
  EXPECT_FALSE(jerk_profile.empty());
}

TEST(KinematicSmootherTrajUtils, CalcStopDistSmallVelocityIsSmall)
{
  // Note: v0 == target_vel produces a 0/0 in the LINEAR branch of the
  // upstream implementation. Tested with a small but non-zero v0 instead,
  // which exercises the real braking math.
  std::map<double, double> jerk_profile;
  double stop_dist = 0.0;
  const bool ok = tu::calcStopDistWithJerkConstraints(
    /*v0=*/0.1, /*a0=*/0.0, /*jerk_acc=*/0.3, /*jerk_dec=*/-0.3, /*min_acc=*/-2.5,
    /*target_vel=*/0.0, jerk_profile, stop_dist);
  EXPECT_TRUE(ok);
  EXPECT_GE(stop_dist, 0.0);
  EXPECT_LT(stop_dist, 1.0);  // braking from 0.1 m/s should be << 1 m
}

// ---------------------------------------------------------------------------
// calcVelocityProfileWithConstantJerkAndAccelerationLimit
// ---------------------------------------------------------------------------

TEST(KinematicSmootherTrajUtils, ConstantJerkProfileMonotonicWhenJerkPositive)
{
  const auto pts = makeStraight(10.0, 1.0, 0.0);
  const auto profile = tu::calcVelocityProfileWithConstantJerkAndAccelerationLimit(
    pts, /*v0=*/0.0, /*a0=*/0.0, /*jerk=*/0.3, /*acc_max=*/1.0, /*acc_min=*/-0.5);
  ASSERT_EQ(profile.size(), pts.size());
  for (size_t i = 1; i < profile.size(); ++i) {
    EXPECT_GE(profile[i] + 1e-6, profile[i - 1]);  // monotonically non-decreasing
  }
}


// ---------------------------------------------------------------------------
// endpoint velocity preservation through post-resampling
// ---------------------------------------------------------------------------

rs::ResampleParam makePostResampleParam()
{
  return rs::ResampleParam{
    300.0, 30.0, 10.0, 0.1, 0.1, 0.1, 1.0};
}

TEST(KinematicSmootherResample, PreservesNonzeroRollingHorizonEndpoint)
{
  const auto input = makeStraight(100.0, 1.0, 5.0);
  geometry_msgs::msg::Pose ego;
  ego.orientation.w = 1.0;

  const auto output = rs::resampleTrajectory(
    input, /*v_current=*/20.0, ego, /*nearest_dist_threshold=*/3.0,
    /*nearest_yaw_threshold=*/1.046, makePostResampleParam(), /*use_zoh_for_v=*/false);

  ASSERT_FALSE(output.empty());
  EXPECT_NEAR(output.back().pose.position.x, input.back().pose.position.x, 1e-6);
  EXPECT_FLOAT_EQ(output.back().longitudinal_velocity_mps, 5.0f);
}

TEST(KinematicSmootherResample, PreservesRealZeroVelocityEndpoint)
{
  auto input = makeStraight(100.0, 1.0, 5.0);
  input.back().longitudinal_velocity_mps = 0.0f;
  geometry_msgs::msg::Pose ego;
  ego.orientation.w = 1.0;

  const auto output = rs::resampleTrajectory(
    input, /*v_current=*/20.0, ego, /*nearest_dist_threshold=*/3.0,
    /*nearest_yaw_threshold=*/1.046, makePostResampleParam(), /*use_zoh_for_v=*/false);

  ASSERT_FALSE(output.empty());
  EXPECT_NEAR(output.back().pose.position.x, input.back().pose.position.x, 1e-6);
  EXPECT_FLOAT_EQ(output.back().longitudinal_velocity_mps, 0.0f);
}
