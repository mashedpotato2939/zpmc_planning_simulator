// Copyright 2024 ZPMC
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

#include <cmath>

#include <gtest/gtest.h>

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include "igv_path_planner/path_generator.hpp"

namespace
{

geometry_msgs::msg::Quaternion yawToQuat(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

autoware_planning_msgs::msg::PathPoint makePoint(
  const double x, const double y, const double yaw, const float velocity)
{
  autoware_planning_msgs::msg::PathPoint pt{};
  pt.pose.position.x = x;
  pt.pose.position.y = y;
  pt.pose.position.z = 0.0;
  pt.pose.orientation = yawToQuat(yaw);
  pt.longitudinal_velocity_mps = velocity;
  return pt;
}

igv_path_planner::PathGenerator::Parameters baseParams()
{
  igv_path_planner::PathGenerator::Parameters p;
  p.path_interval = 0.5;
  p.qturn_min_radius = 15.5;
  p.qturn_yaw_min_radius = 8.1;
  p.qturn_yaw_min_threshold = 1.3;
  p.qturn_yaw_max_threshold = 1.9;
  return p;
}

}  // namespace

TEST(PathGeneratorSpeedLimit, QTurnClampsForwardAndKeepsGoalStop)
{
  auto params = baseParams();
  params.qturn_speed_limit = 2.0;
  igv_path_planner::PathGenerator gen(params);

  const auto start = makePoint(0.0, 0.0, 0.0, 6.0f);
  const auto end = makePoint(16.0, 16.0, M_PI / 2.0, 0.0f);

  const auto points = gen.generateQTurnSegment(start, end);
  ASSERT_FALSE(gen.lastQTurnFellBack());
  ASSERT_GE(points.size(), 2u);

  for (size_t i = 0; i + 1 < points.size(); ++i) {
    EXPECT_NEAR(points[i].longitudinal_velocity_mps, 2.0f, 1e-5f);
  }
  EXPECT_NEAR(points.back().longitudinal_velocity_mps, 0.0f, 1e-5f);
}

TEST(PathGeneratorSpeedLimit, QTurnClampsReverse)
{
  auto params = baseParams();
  params.qturn_speed_limit = 2.5;
  igv_path_planner::PathGenerator gen(params);
  gen.setReverseState(true);

  const auto start = makePoint(0.0, 0.0, 0.0, -6.0f);
  const auto end = makePoint(-16.0, -16.0, M_PI / 2.0, -6.0f);

  const auto points = gen.generateQTurnSegment(start, end);
  ASSERT_FALSE(gen.lastQTurnFellBack());
  ASSERT_GE(points.size(), 2u);

  for (const auto & point : points) {
    EXPECT_NEAR(point.longitudinal_velocity_mps, -2.5f, 1e-5f);
  }
}

TEST(PathGeneratorSpeedLimit, ObliqueClampsForwardAndScalesLateralVelocity)
{
  auto limited_params = baseParams();
  limited_params.oblique_speed_limit = 2.0;
  igv_path_planner::PathGenerator limited_gen(limited_params);

  auto reference_params = limited_params;
  reference_params.oblique_speed_limit = 10.0;
  igv_path_planner::PathGenerator reference_gen(reference_params);

  const auto start = makePoint(0.0, 0.0, 0.0, 5.0f);
  const auto end = makePoint(20.0, 3.0, 0.0, 5.0f);

  const auto limited = limited_gen.generateObliqueLaneChangeSegment(start, end);
  const auto reference = reference_gen.generateObliqueLaneChangeSegment(start, end);
  ASSERT_EQ(limited.size(), reference.size());
  ASSERT_GE(limited.size(), 2u);

  bool checked_lateral_velocity = false;
  for (size_t i = 0; i < limited.size(); ++i) {
    EXPECT_NEAR(limited[i].longitudinal_velocity_mps, 2.0f, 1e-5f);

    if (!checked_lateral_velocity && std::abs(reference[i].lateral_velocity_mps) > 1e-4f) {
      EXPECT_NEAR(
        limited[i].lateral_velocity_mps,
        reference[i].lateral_velocity_mps * (2.0f / 5.0f), 1e-5f);
      checked_lateral_velocity = true;
    }
  }
  EXPECT_TRUE(checked_lateral_velocity);
}

TEST(PathGeneratorSpeedLimit, ObliqueClampsReverse)
{
  auto params = baseParams();
  params.oblique_speed_limit = 2.0;
  igv_path_planner::PathGenerator gen(params);
  gen.setReverseState(true);

  const auto start = makePoint(0.0, 0.0, 0.0, -5.0f);
  const auto end = makePoint(-20.0, -3.0, 0.0, -5.0f);

  const auto points = gen.generateObliqueLaneChangeSegment(start, end);
  ASSERT_GE(points.size(), 2u);

  for (const auto & point : points) {
    EXPECT_NEAR(point.longitudinal_velocity_mps, -2.0f, 1e-5f);
  }
}
