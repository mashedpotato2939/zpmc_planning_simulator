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

#include "igv_motion_utils/trajectory_resample.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace
{

geometry_msgs::msg::Quaternion yawToQuat(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double quatToYaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

autoware_planning_msgs::msg::TrajectoryPoint makePoint(
  const double x, const double y, const double yaw, const double velocity = 1.0)
{
  autoware_planning_msgs::msg::TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.orientation = yawToQuat(yaw);
  point.longitudinal_velocity_mps = static_cast<float>(velocity);
  return point;
}

autoware_planning_msgs::msg::PathPoint makePathPoint(
  const double x, const double y, const double yaw)
{
  autoware_planning_msgs::msg::PathPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.orientation = yawToQuat(yaw);
  return point;
}

}  // namespace

TEST(IgvMotionUtils, ResamplePreservesBodyYawInsteadOfPathTangent)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> input{
    makePoint(0.0, 0.0, M_PI / 6.0), makePoint(4.0, 0.0, M_PI / 6.0)};

  const auto output = igv_motion_utils::resampleTrajectoryPreserveOrientation(input, 1.0);
  ASSERT_GE(output.size(), 4u);
  for (const auto & point : output) {
    EXPECT_NEAR(normalizeAngle(quatToYaw(point.pose.orientation) - M_PI / 6.0), 0.0, 1e-6);
  }
}

TEST(IgvMotionUtils, ResampleInterpolatesYawAcrossWraparoundContinuously)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> input{
    makePoint(0.0, 0.0, 170.0 * M_PI / 180.0),
    makePoint(1.0, 0.0, -170.0 * M_PI / 180.0),
    makePoint(2.0, 0.0, -160.0 * M_PI / 180.0)};

  const auto output = igv_motion_utils::resampleTrajectoryPreserveOrientation(input, 0.5);
  ASSERT_GE(output.size(), 5u);
  EXPECT_NEAR(std::abs(quatToYaw(output[1].pose.orientation)), M_PI, 1e-6);
  EXPECT_NEAR(quatToYaw(output[2].pose.orientation), -170.0 * M_PI / 180.0, 1e-6);
}

TEST(IgvMotionUtils, StopPointInsertionKeepsBodyYawContinuous)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> input{
    makePoint(0.0, 0.0, 0.0, 1.0), makePoint(1.0, 0.0, M_PI / 2.0, 0.0),
    makePoint(2.0, 0.0, M_PI / 2.0, 0.0)};

  const auto output = igv_motion_utils::resampleTrajectoryPreserveOrientation(
    input, 0.75, true, true);

  const auto it = std::find_if(output.begin(), output.end(), [](const auto & point) {
    return std::abs(point.pose.position.x - 1.0) < 1e-6;
  });
  ASSERT_NE(it, output.end());
  EXPECT_NEAR(normalizeAngle(quatToYaw(it->pose.orientation) - M_PI / 2.0), 0.0, 1e-6);
}

TEST(IgvMotionUtils, PathProjectionInterpolatesBodyYaw)
{
  autoware_planning_msgs::msg::Path behavior_path;
  behavior_path.points.push_back(makePathPoint(0.0, 0.0, 0.0));
  behavior_path.points.push_back(makePathPoint(1.0, 0.0, 10.0 * M_PI / 180.0));
  behavior_path.points.push_back(makePathPoint(2.0, 0.0, 20.0 * M_PI / 180.0));

  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> trajectory{
    makePoint(0.25, 0.0, 0.0), makePoint(0.75, 0.0, 0.0), makePoint(1.25, 0.0, 0.0)};

  igv_motion_utils::overwriteOrientationFromPathProjection(trajectory, behavior_path);

  EXPECT_NEAR(quatToYaw(trajectory[0].pose.orientation), 2.5 * M_PI / 180.0, 1e-6);
  EXPECT_NEAR(quatToYaw(trajectory[1].pose.orientation), 7.5 * M_PI / 180.0, 1e-6);
  EXPECT_NEAR(quatToYaw(trajectory[2].pose.orientation), 12.5 * M_PI / 180.0, 1e-6);
}
