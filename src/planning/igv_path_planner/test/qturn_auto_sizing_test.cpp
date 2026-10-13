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

#include <algorithm>
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

double quatToYaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double wrapAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
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

igv_path_planner::PathGenerator::Parameters qturnParams(
  const double radius, const double yaw_radius)
{
  igv_path_planner::PathGenerator::Parameters params;
  params.path_interval = 1.0;
  params.qturn_min_radius = radius;
  params.qturn_yaw_min_radius = yaw_radius;
  params.qturn_speed_limit = 2.0;
  params.qturn_yaw_min_threshold = 1.3;
  params.qturn_yaw_max_threshold = 1.9;
  return params;
}

}  // namespace

TEST(QTurnAutoSizing, CapsCurveAndKeepsSideslipForScaleIgvGeometry)
{
  auto params = qturnParams(6.0, 4.0);
  igv_path_planner::PathGenerator gen(params);

  // Mirrors the live scale-IGV LOCAL_PLAN geometry that previously selected a
  // curve arc length just above pi * R_yaw and therefore produced zero sideslip.
  const auto start = makePoint(0.0, 0.0, 0.0, 5.0f);
  const auto end = makePoint(10.0, 7.647969, M_PI / 2.0, 5.0f);

  const auto points = gen.generateQTurnSegment(start, end);
  ASSERT_FALSE(gen.lastQTurnFellBack());
  ASSERT_GE(points.size(), 4u);

  EXPECT_NEAR(points.back().pose.position.x, end.pose.position.x, 1e-6);
  EXPECT_NEAR(points.back().pose.position.y, end.pose.position.y, 1e-6);
  EXPECT_NEAR(wrapAngle(quatToYaw(points.back().pose.orientation) - M_PI / 2.0), 0.0, 1e-6);
  EXPECT_TRUE(points.back().is_final);

  double max_abs_lateral_velocity = 0.0;
  double max_abs_heading_rate = 0.0;
  for (const auto & point : points) {
    max_abs_lateral_velocity =
      std::max(max_abs_lateral_velocity, std::abs(static_cast<double>(point.lateral_velocity_mps)));
    max_abs_heading_rate =
      std::max(max_abs_heading_rate, std::abs(static_cast<double>(point.heading_rate_rps)));
  }

  EXPECT_GT(max_abs_lateral_velocity, 1e-3);
  EXPECT_GT(max_abs_heading_rate, 1e-3);
}

TEST(QTurnAutoSizing, FallsBackWhenYawClothoidNeedsMoreTangentDistance)
{
  auto params = qturnParams(3.0, 4.0);
  igv_path_planner::PathGenerator gen(params);

  // R=3 m would fit geometrically, but R_yaw=4 m needs about 3.82 m of
  // tangent distance after applying the yaw-clothoid safety margin.
  const auto start = makePoint(0.0, 0.0, 0.0, 5.0f);
  const auto end = makePoint(3.5, 3.5, M_PI / 2.0, 5.0f);

  const auto points = gen.generateQTurnSegment(start, end);
  EXPECT_TRUE(gen.lastQTurnFellBack());
  EXPECT_GE(points.size(), 2u);
}
