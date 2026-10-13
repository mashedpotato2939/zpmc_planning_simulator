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

#include "igv_velocity_planner/velocity_shaper/generators.hpp"
#include "igv_velocity_planner/velocity_shaper/geometry_utils.hpp"

#include <gtest/gtest.h>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <rclcpp/time.hpp>

#include <cmath>
#include <vector>

namespace
{
namespace vs = igv_velocity_planner::velocity_shaper;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::PathPoint;
using TrajectoryPoint = autoware_planning_msgs::msg::TrajectoryPoint;

std::vector<TrajectoryPoint> straightTrajectoryPoints(double length, double step, double vel)
{
  Path path;
  for (double s = 0.0; s <= length + 1e-6; s += step) {
    PathPoint p;
    p.pose.position.x = s;
    p.pose.orientation.w = 1.0;
    p.longitudinal_velocity_mps = static_cast<float>(vel);
    path.points.push_back(p);
  }
  return vs::convertPathToTrajectoryPoints(path);
}

PredictedObject makeStaticObject(double x, double y, double width = 1.0)
{
  PredictedObject obj;
  obj.kinematics.initial_pose_with_covariance.pose.position.x = x;
  obj.kinematics.initial_pose_with_covariance.pose.position.y = y;
  obj.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  obj.shape.dimensions.x = 2.0;
  obj.shape.dimensions.y = width;
  obj.shape.dimensions.z = 1.0;
  return obj;
}

PredictedObject makeMovingObject(
  double start_x, double start_y, double vx, double vy, double dt = 0.2, size_t n_steps = 30)
{
  PredictedObject obj;
  obj.kinematics.initial_pose_with_covariance.pose.position.x = start_x;
  obj.kinematics.initial_pose_with_covariance.pose.position.y = start_y;
  obj.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  obj.kinematics.initial_twist_with_covariance.twist.linear.x = std::hypot(vx, vy);
  obj.shape.dimensions.x = 2.0;
  obj.shape.dimensions.y = 1.0;
  obj.shape.dimensions.z = 1.0;
  PredictedPath pp;
  pp.confidence = 1.0f;
  pp.time_step.sec = static_cast<int32_t>(dt);
  pp.time_step.nanosec = static_cast<uint32_t>((dt - static_cast<int32_t>(dt)) * 1e9);
  for (size_t i = 0; i < n_steps; ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = start_x + vx * i * dt;
    pose.position.y = start_y + vy * i * dt;
    pose.orientation.w = 1.0;
    pp.path.push_back(pose);
  }
  obj.kinematics.predicted_paths.push_back(pp);
  return obj;
}

PredictedObjects wrap(const std::vector<PredictedObject> & objs)
{
  PredictedObjects out;
  out.objects = objs;
  return out;
}
}  // namespace

// ACC generator caps at leader velocity minus margin.
TEST(Generators, AdaptiveCruiseEmitsLeaderCap)
{
  auto pts = straightTrajectoryPoints(60.0, 0.5, 8.0);
  vs::Params::AdaptiveCruise params;  // velocity_margin 0.5
  vs::AdaptiveCruiseGenerator gen(params);

  auto objs = wrap({makeMovingObject(25.0, 0.0, 5.0, 0.0)});
  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = 8.0;
  vs::TickInput in{pts, &objs, nullptr, &odom, 0.0, 0.0, rclcpp::Time()};
  std::vector<vs::VelocityConstraint> out;
  gen.generate(in, out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].type, vs::ConstraintType::kAdaptiveCruise);
  EXPECT_NEAR(out[0].start_s, 0.0, 1e-9);
  EXPECT_NEAR(out[0].target_velocity, 4.5, 1e-3);  // 5.0 - 0.5
}

// DynamicStopGenerator commits a v=0 constraint only after add_stop_duration_buffer
// of sustained collision, then releases after remove_stop_duration_buffer.
TEST(Generators, DynamicStopHysteresisCommitAndRelease)
{
  auto pts = straightTrajectoryPoints(40.0, 0.5, 5.0);
  vs::Params::DynamicObstacleStop params;
  params.add_stop_duration_buffer = 1.0;
  params.remove_stop_duration_buffer = 1.0;
  params.minimum_object_velocity = 0.5;
  double ego_fc = 0.0, ego_hw = 0.5;
  vs::DynamicStopGenerator gen(params, ego_fc, ego_hw);

  // Object crossing the path ahead.
  auto crossing = wrap({makeMovingObject(15.0, -8.0, 0.0, 4.0)});
  auto empty = wrap({});

  // t=10.0s: collision detected but not yet sustained → no constraint. (Use a
  // non-zero base stamp: nanoseconds()==0 is the "first collision" sentinel.)
  {
    vs::TickInput in{pts, &crossing, nullptr, nullptr, 0.0, ego_hw, rclcpp::Time(10, 0, RCL_ROS_TIME)};
    std::vector<vs::VelocityConstraint> out;
    gen.generate(in, out);
    EXPECT_TRUE(out.empty());
  }
  // t=11.5s: sustained beyond add buffer → committed stop.
  {
    vs::TickInput in{pts, &crossing, nullptr, nullptr, 0.0, ego_hw,
                     rclcpp::Time(11, 500000000, RCL_ROS_TIME)};
    std::vector<vs::VelocityConstraint> out;
    gen.generate(in, out);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].type, vs::ConstraintType::kDynamicStop);
    EXPECT_NEAR(out[0].target_velocity, 0.0, 1e-9);
  }
  // Object still tracked but no longer colliding, within remove buffer → held.
  {
    auto away = wrap({makeMovingObject(15.0, -40.0, 0.0, -1.0)});  // moving away, same uuid (default)
    vs::TickInput in{pts, &away, nullptr, nullptr, 0.0, ego_hw,
                     rclcpp::Time(12, 0, RCL_ROS_TIME)};
    std::vector<vs::VelocityConstraint> out;
    gen.generate(in, out);
    EXPECT_EQ(out.size(), 1u);  // still committed (0.5s < 1.0s remove buffer)
  }
  // Past the remove buffer → released.
  {
    auto away = wrap({makeMovingObject(15.0, -40.0, 0.0, -1.0)});
    vs::TickInput in{pts, &away, nullptr, nullptr, 0.0, ego_hw,
                     rclcpp::Time(13, 500000000, RCL_ROS_TIME)};
    std::vector<vs::VelocityConstraint> out;
    gen.generate(in, out);
    EXPECT_TRUE(out.empty());
  }
}

TEST(Generators, DynamicStopIgnoresObjectFootprintFullyBehindEgo)
{
  auto pts = straightTrajectoryPoints(40.0, 0.5, 5.0);
  vs::Params::DynamicObstacleStop params;
  params.add_stop_duration_buffer = 0.0;
  params.minimum_object_velocity = 0.5;
  params.minimum_object_distance_from_ego_trajectory = 0.0;
  double ego_fc = 0.0, ego_hw = 0.5;
  vs::DynamicStopGenerator gen(params, ego_fc, ego_hw);

  auto obj = makeMovingObject(4.0, 0.0, 3.0, 0.0);
  obj.shape.dimensions.x = 2.0;  // current footprint max x=5, behind ego rear x=8
  auto objs = wrap({obj});
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 10.0;
  odom.pose.pose.orientation.w = 1.0;

  vs::TickInput in{
    pts, &objs, nullptr, &odom, ego_fc, ego_hw, rclcpp::Time(10, 0, RCL_ROS_TIME),
    /*ego_s=*/10.0, /*ego_min_longitudinal_offset=*/-2.0};
  std::vector<vs::VelocityConstraint> out;
  gen.generate(in, out);

  EXPECT_TRUE(out.empty());
}

TEST(Generators, DynamicStopConsidersObjectOverlappingEgoRearBound)
{
  auto pts = straightTrajectoryPoints(40.0, 0.5, 5.0);
  vs::Params::DynamicObstacleStop params;
  params.add_stop_duration_buffer = 0.0;
  params.minimum_object_velocity = 0.5;
  params.minimum_object_distance_from_ego_trajectory = 0.0;
  double ego_fc = 0.0, ego_hw = 0.5;
  vs::DynamicStopGenerator gen(params, ego_fc, ego_hw);

  auto obj = makeMovingObject(7.5, 0.0, 3.0, 0.0);
  obj.shape.dimensions.x = 2.0;  // current footprint max x=8.5, overlapping ego rear x=8
  auto objs = wrap({obj});
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 10.0;
  odom.pose.pose.orientation.w = 1.0;

  vs::TickInput in{
    pts, &objs, nullptr, &odom, ego_fc, ego_hw, rclcpp::Time(10, 0, RCL_ROS_TIME),
    /*ego_s=*/10.0, /*ego_min_longitudinal_offset=*/-2.0};
  std::vector<vs::VelocityConstraint> out;
  gen.generate(in, out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].type, vs::ConstraintType::kDynamicStop);
}

// SlowDownGenerator commits an outer-band window after add_duration_buffer.
TEST(Generators, SlowDownHysteresisCommits)
{
  auto pts = straightTrajectoryPoints(40.0, 0.5, 8.0);
  vs::Params::SlowDown params;  // inner 0.5, outer 2.0
  params.add_duration_buffer = 0.3;
  double ego_hw = 0.0;
  vs::SlowDownGenerator gen(params, ego_hw);

  auto objs = wrap({makeStaticObject(20.0, 1.0)});  // outer band

  {
    vs::TickInput in{pts, &objs, nullptr, nullptr, 0.0, 0.0, rclcpp::Time(10, 0, RCL_ROS_TIME)};
    std::vector<vs::VelocityConstraint> out;
    gen.generate(in, out);
    EXPECT_TRUE(out.empty());  // not yet sustained
  }
  {
    vs::TickInput in{pts, &objs, nullptr, nullptr, 0.0, 0.0,
                     rclcpp::Time(10, 500000000, RCL_ROS_TIME)};
    std::vector<vs::VelocityConstraint> out;
    gen.generate(in, out);
    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].type, vs::ConstraintType::kSlowDown);
    EXPECT_LT(out[0].start_s, out[0].end_s);
    EXPECT_GT(out[0].target_velocity, 0.0);
  }
}

// Footprint-aware static stop: an elongated static object whose CENTER projects
// 1.4 m off the path (outside a naive center band) but whose near edge intrudes
// the ego corridor must produce a v=0 stop backed off by the buffer. Mirrors the
// field case where center-based reasoning under-reacted.
TEST(Generators, StaticObstacleStopFootprintIntrusionStops)
{
  auto pts = straightTrajectoryPoints(40.0, 0.5, 5.0);
  vs::Params::StaticObstacleStop params;
  params.lateral_clearance = 0.5;          // ego_half_width(0.5) + 0.5 => 1.0 m corridor
  params.stop_distance_buffer = 2.0;
  params.add_stop_duration_buffer = 0.0;   // commit immediately
  double ego_fc = 0.0, ego_hw = 0.5;
  vs::StaticObstacleStopGenerator gen(params, ego_fc, ego_hw);

  // width 2.0 -> half-width 1.0; center at y=1.4 -> near edge at 0.4 m < 1.0 m.
  auto objs = wrap({makeStaticObject(20.0, 1.4, 2.0)});
  vs::TickInput in{pts, &objs, nullptr, nullptr, ego_fc, ego_hw, rclcpp::Time(10, 0, RCL_ROS_TIME)};
  std::vector<vs::VelocityConstraint> out;
  gen.generate(in, out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].type, vs::ConstraintType::kStaticObstacleStop);
  EXPECT_NEAR(out[0].target_velocity, 0.0, 1e-9);
  EXPECT_TRUE(std::isinf(out[0].end_s));
  EXPECT_GT(out[0].start_s, 0.0);
  EXPECT_LT(out[0].start_s, 20.0);  // backed off before the object
}

TEST(Generators, StaticObstacleStopIgnoresFootprintFullyBehindEgoArcLength)
{
  auto pts = straightTrajectoryPoints(40.0, 0.5, 5.0);
  vs::Params::StaticObstacleStop params;
  params.lateral_clearance = 0.5;
  params.add_stop_duration_buffer = 0.0;
  double ego_fc = 0.0, ego_hw = 0.5;
  vs::StaticObstacleStopGenerator gen(params, ego_fc, ego_hw);

  auto obj = makeStaticObject(8.0, 0.0);
  obj.shape.dimensions.x = 1.0;  // footprint interval [7.5, 8.5] is behind ego_s=10
  auto objs = wrap({obj});
  vs::TickInput in{
    pts, &objs, nullptr, nullptr, ego_fc, ego_hw, rclcpp::Time(10, 0, RCL_ROS_TIME),
    /*ego_s=*/10.0};
  std::vector<vs::VelocityConstraint> out;
  gen.generate(in, out);

  EXPECT_TRUE(out.empty());
}

TEST(Generators, StaticObstacleStopOverlappingEgoClampsStopToEgoArcLength)
{
  auto pts = straightTrajectoryPoints(40.0, 0.5, 5.0);
  vs::Params::StaticObstacleStop params;
  params.lateral_clearance = 0.5;
  params.stop_distance_buffer = 2.0;
  params.add_stop_duration_buffer = 0.0;
  double ego_fc = 0.0, ego_hw = 0.5;
  vs::StaticObstacleStopGenerator gen(params, ego_fc, ego_hw);

  auto obj = makeStaticObject(9.8, 0.0);
  obj.shape.dimensions.x = 1.0;  // footprint interval [9.3, 10.3] overlaps ego_s=10
  auto objs = wrap({obj});
  vs::TickInput in{
    pts, &objs, nullptr, nullptr, ego_fc, ego_hw, rclcpp::Time(10, 0, RCL_ROS_TIME),
    /*ego_s=*/10.0};
  std::vector<vs::VelocityConstraint> out;
  gen.generate(in, out);

  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].type, vs::ConstraintType::kStaticObstacleStop);
  EXPECT_NEAR(out[0].start_s, 10.0, 1e-9);
}

// A static object safely beside the path (near edge beyond the corridor) is not
// stopped for (the graded slow-down owns that band).
TEST(Generators, StaticObstacleStopBesidePathNoStop)
{
  auto pts = straightTrajectoryPoints(40.0, 0.5, 5.0);
  vs::Params::StaticObstacleStop params;
  params.lateral_clearance = 0.5;
  params.add_stop_duration_buffer = 0.0;
  double ego_fc = 0.0, ego_hw = 0.5;
  vs::StaticObstacleStopGenerator gen(params, ego_fc, ego_hw);
  // width 1.0 -> half 0.5; center y=3.0 -> near edge 2.5 m > 1.0 m corridor.
  auto objs = wrap({makeStaticObject(20.0, 3.0, 1.0)});
  vs::TickInput in{pts, &objs, nullptr, nullptr, ego_fc, ego_hw, rclcpp::Time(10, 0, RCL_ROS_TIME)};
  std::vector<vs::VelocityConstraint> out;
  gen.generate(in, out);
  EXPECT_TRUE(out.empty());
}

// Moving objects belong to dynamic_obstacle_stop, not this generator.
TEST(Generators, StaticObstacleStopIgnoresMovingObject)
{
  auto pts = straightTrajectoryPoints(40.0, 0.5, 5.0);
  vs::Params::StaticObstacleStop params;
  params.add_stop_duration_buffer = 0.0;
  double ego_fc = 0.0, ego_hw = 0.5;
  vs::StaticObstacleStopGenerator gen(params, ego_fc, ego_hw);
  auto objs = wrap({makeMovingObject(20.0, 0.4, 3.0, 0.0)});  // in-path but moving
  vs::TickInput in{pts, &objs, nullptr, nullptr, ego_fc, ego_hw, rclcpp::Time(10, 0, RCL_ROS_TIME)};
  std::vector<vs::VelocityConstraint> out;
  gen.generate(in, out);
  EXPECT_TRUE(out.empty());
}
