#include <gtest/gtest.h>

#include "igv_path_planner/avoidance_planner/crab_avoidance_planner.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace
{
using igv_path_planner::cb_planning::CB_Path_Planning;
using igv_path_planner::cb_planning::Planning_Para;

std::shared_ptr<Planning_Para> parameters(double lane_end_x, double target_y)
{
  auto p = std::make_shared<Planning_Para>();
  p->ego_length = 4.0;
  p->ego_width = 2.0;
  p->steer_angle_max = 30.0;
  p->steer_angular_vel_max = 20.0;
  p->Min_Collision_free_distance = 20.0;
  p->start_point_mindis_threshold = 1.0;
  p->lane_sampling_begin_y = target_y;
  p->lane_sampling_end_y = target_y;
  p->lane_y_sampling_interval = 0.25;
  p->lane_begin_y = -5.25;
  p->lane_width = 3.5;
  p->lane_num = 3;
  p->lane_end_x = lane_end_x;
  p->local_map_forward_x = 50.0;
  p->local_map_backward_x = 5.0;
  p->lane_x_sampling_interval = 1.0;
  return p;
}

autoware_planning_msgs::msg::Path::SharedPtr straightPath(int end_x, bool duplicate)
{
  auto path = std::make_shared<autoware_planning_msgs::msg::Path>();
  for (int x = 0; x <= end_x; ++x) {
    autoware_planning_msgs::msg::PathPoint point;
    point.pose.position.x = static_cast<double>(x);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 3.0f;
    path->points.push_back(point);
    if (duplicate && x == 50) path->points.push_back(point);
  }
  return path;
}

nav_msgs::msg::Odometry::SharedPtr egoAtOrigin()
{
  auto ego = std::make_shared<nav_msgs::msg::Odometry>();
  ego->pose.pose.orientation.w = 1.0;
  ego->twist.twist.linear.x = 3.0;
  return ego;
}
}  // namespace

TEST(CrabAvoidancePlanner, OutputMatchesReferencePointCountAndReturnsToReference)
{
  auto reference = straightPath(100, true);
  CB_Path_Planning planner;
  planner.setParameter(parameters(100.0, 3.5));
  planner.setReferencePath(reference);
  planner.setOdometry(egoAtOrigin());

  ASSERT_TRUE(planner.run());
  const auto & output = planner.outputPath();
  ASSERT_EQ(output.points.size(), reference->points.size());
  ASSERT_EQ(planner.sampledTrajectory().size(), reference->points.size());
  double max_y = 0.0;
  for (size_t i = 0; i < output.points.size(); ++i) {
    EXPECT_NEAR(output.points[i].pose.position.x, reference->points[i].pose.position.x, 1e-6);
    max_y = std::max(max_y, output.points[i].pose.position.y);
  }
  EXPECT_GT(max_y, 3.0);
  EXPECT_NEAR(output.points.back().pose.position.y, 0.0, 1e-6);
}

TEST(CrabAvoidancePlanner, RejectsCandidateWithoutDistanceToReturn)
{
  CB_Path_Planning planner;
  planner.setParameter(parameters(25.0, 3.5));
  planner.setReferencePath(straightPath(25, false));
  planner.setOdometry(egoAtOrigin());

  EXPECT_FALSE(planner.run());
  EXPECT_TRUE(planner.outputPath().points.empty());
}

TEST(CrabAvoidancePlanner, FastObjectIsStillStaticWhenDynamicModeDisabled)
{
  auto p = parameters(100.0, 0.0);
  p->enable_dynamic_obstacle_avoidance = false;
  CB_Path_Planning planner;
  planner.setParameter(p);
  planner.setReferencePath(straightPath(100, false));
  planner.setOdometry(egoAtOrigin());

  autoware_perception_msgs::msg::PredictedObjects objects;
  autoware_perception_msgs::msg::PredictedObject object;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 5.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.kinematics.initial_twist_with_covariance.twist.linear.x = 5.0;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  objects.objects.push_back(object);
  planner.setObjectsInfo(objects);

  EXPECT_FALSE(planner.run());
}

TEST(CrabAvoidancePlanner, DynamicModeExpandsCrossingObjectIntoLane)
{
  autoware_perception_msgs::msg::PredictedObjects objects;
  autoware_perception_msgs::msg::PredictedObject object;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 5.0;
  object.kinematics.initial_pose_with_covariance.pose.position.y = 5.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.kinematics.initial_twist_with_covariance.twist.linear.y = -2.0;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  objects.objects.push_back(object);

  auto parameter = parameters(100.0, 0.0);
  CB_Path_Planning planner;
  planner.setParameter(parameter);
  planner.setReferencePath(straightPath(100, false));
  planner.setOdometry(egoAtOrigin());
  planner.setObjectsInfo(objects);
  EXPECT_TRUE(planner.run());

  parameter->enable_dynamic_obstacle_avoidance = true;
  CB_Path_Planning dynamic_planner;
  dynamic_planner.setParameter(parameter);
  dynamic_planner.setReferencePath(straightPath(100, false));
  dynamic_planner.setOdometry(egoAtOrigin());
  dynamic_planner.setObjectsInfo(objects);
  EXPECT_FALSE(dynamic_planner.run());

  // 上一帧静态规划成功后切换模式，也不能沿用已被目标扫掠区域占据的前段。
  planner.setParameter(parameter);
  EXPECT_FALSE(planner.run());
}
