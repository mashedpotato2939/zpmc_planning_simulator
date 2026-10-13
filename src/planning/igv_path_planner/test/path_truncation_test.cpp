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

#include "igv_path_planner/utils/path_planning_utils.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace
{
geometry_msgs::msg::Quaternion yawToQuat(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(yaw / 2.0);
  q.z = std::sin(yaw / 2.0);
  return q;
}

autoware_planning_msgs::msg::Path makeStraightPath(
  const int n, const double spacing, const float velocity, const double yaw = 0.0)
{
  autoware_planning_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i < n; ++i) {
    autoware_planning_msgs::msg::PathPoint p;
    p.pose.position.x = spacing * i;
    p.pose.position.y = 0.0;
    p.pose.orientation = yawToQuat(yaw);
    p.longitudinal_velocity_mps = velocity;
    p.is_final = (i == n - 1);
    path.points.push_back(p);
  }
  return path;
}

geometry_msgs::msg::Pose makePose(const double x, const double y, const double yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = yawToQuat(yaw);
  return pose;
}

void addStraightBounds(autoware_planning_msgs::msg::Path & path)
{
  for (const auto & point : path.points) {
    auto left = point.pose.position;
    left.y += 1.0;
    path.left_bound.push_back(left);

    auto right = point.pose.position;
    right.y -= 1.0;
    path.right_bound.push_back(right);
  }
}
}  // namespace

TEST(PathTruncation, RetainsBackBufferBehindForwardEgo)
{
  auto path = makeStraightPath(11, 1.0, 5.0f);
  addStraightBounds(path);

  const bool changed =
    igv_path_planner::utils::truncatePathFromEgoPose(path, makePose(8.4, 0.0), 10.0, M_PI, 2.0);

  EXPECT_TRUE(changed);
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_NEAR(path.points.front().pose.position.x, 6.4, 1e-6);
  EXPECT_NEAR(path.points[1].pose.position.x, 7.0, 1e-6);
  EXPECT_NEAR(path.points.front().longitudinal_velocity_mps, 5.0f, 1e-6);
  ASSERT_EQ(path.left_bound.size(), path.points.size());
  ASSERT_EQ(path.right_bound.size(), path.points.size());
  EXPECT_NEAR(path.left_bound.front().x, 6.4, 1e-6);
  EXPECT_NEAR(path.left_bound.front().y, 1.0, 1e-6);
  EXPECT_NEAR(path.right_bound.front().x, 6.4, 1e-6);
  EXPECT_NEAR(path.right_bound.front().y, -1.0, 1e-6);
}

TEST(PathTruncation, AvoidsDuplicateBackBufferProjectionNearExistingPoint)
{
  auto path = makeStraightPath(11, 1.0, 5.0f);

  const bool changed =
    igv_path_planner::utils::truncatePathFromEgoPose(path, makePose(8.05, 0.0), 10.0, M_PI, 2.0);

  EXPECT_TRUE(changed);
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_NEAR(path.points.front().pose.position.x, 6.0, 1e-6);
  EXPECT_NEAR(path.points[1].pose.position.x, 7.0, 1e-6);
}

TEST(PathTruncation, ClampsBackBufferAtPathStart)
{
  auto path = makeStraightPath(11, 1.0, 5.0f);

  const bool changed =
    igv_path_planner::utils::truncatePathFromEgoPose(path, makePose(1.2, 0.0), 10.0, M_PI, 5.0);

  EXPECT_FALSE(changed);
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_NEAR(path.points.front().pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(path.points[1].pose.position.x, 1.0, 1e-6);
}

TEST(PathTruncation, UsesMotionOrderForReversePath)
{
  auto path = makeStraightPath(11, 1.0, -2.0f, M_PI);

  const bool changed =
    igv_path_planner::utils::truncatePathFromEgoPose(path, makePose(4.4, 0.0, M_PI), 10.0, M_PI, 2.0);

  EXPECT_TRUE(changed);
  ASSERT_GE(path.points.size(), 2u);
  EXPECT_NEAR(path.points.front().pose.position.x, 2.4, 1e-6);
  EXPECT_NEAR(path.points[1].pose.position.x, 3.0, 1e-6);
  EXPECT_LT(path.points.front().longitudinal_velocity_mps, 0.0f);
}

TEST(PathTruncation, DoesNotRepublishFullPathPastTerminalSegment)
{
  auto path = makeStraightPath(3, 1.0, 5.0f);

  const bool changed =
    igv_path_planner::utils::truncatePathFromEgoPose(path, makePose(2.2, 0.0), 10.0, M_PI, 0.5);

  EXPECT_TRUE(changed);
  ASSERT_FALSE(path.points.empty());
  EXPECT_NEAR(path.points.front().pose.position.x, 1.5, 1e-6);
  EXPECT_LT(path.points.size(), 3u);
}

TEST(LocalPlanRanges, ShiftsLaterResolvedRangesAfterEarlierExpansion)
{
  std::vector<std::pair<size_t, size_t>> ranges;

  // Reverse processing records the later LOCAL_PLAN zone first. When the
  // earlier zone expands, the already-recorded later range must move forward.
  ranges.emplace_back(6u, 36u);
  igv_path_planner::utils::shiftResolvedRangesAfterReplacement(
    ranges, /*replaced_end_idx=*/2u, /*delta=*/20);
  ranges.emplace_back(1u, 21u);

  std::sort(ranges.begin(), ranges.end());

  ASSERT_EQ(ranges.size(), 2u);
  EXPECT_EQ(ranges[0].first, 1u);
  EXPECT_EQ(ranges[0].second, 21u);
  EXPECT_EQ(ranges[1].first, 26u);
  EXPECT_EQ(ranges[1].second, 56u);
  EXPECT_LT(ranges[0].second, ranges[1].first);
}
