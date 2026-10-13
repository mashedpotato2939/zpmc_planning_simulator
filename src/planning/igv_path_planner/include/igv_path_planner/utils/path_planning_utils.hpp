#ifndef IGV_PATH_PLANNER__UTILS__PATH_PLANNING_UTILS_HPP_
#define IGV_PATH_PLANNER__UTILS__PATH_PLANNING_UTILS_HPP_

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace igv_path_planner::utils
{

using PathPoints = std::vector<autoware_planning_msgs::msg::PathPoint>;

double distance2D(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b);

std::string baseRouteId(const std::string & route_id);

geometry_msgs::msg::Quaternion createQuaternionFromYaw(double yaw);

double getYaw(const geometry_msgs::msg::Quaternion & q);

geometry_msgs::msg::Point interpolatePoint(
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & end,
  double ratio);

autoware_planning_msgs::msg::PathPoint interpolatePathPoint(
  const autoware_planning_msgs::msg::PathPoint & start,
  const autoware_planning_msgs::msg::PathPoint & end,
  double ratio);

void shiftResolvedRangesAfterReplacement(
  std::vector<std::pair<size_t, size_t>> & ranges,
  size_t replaced_end_idx,
  std::ptrdiff_t delta);

bool truncatePathFromEgoPose(
  autoware_planning_msgs::msg::Path & path,
  const geometry_msgs::msg::Pose & ego_pose,
  double nearest_dist_threshold,
  double nearest_yaw_threshold,
  double back_buffer_length = 0.0,
  double min_projection_distance = 0.3);

size_t findNearestIndex(
  const autoware_planning_msgs::msg::Path & path,
  const geometry_msgs::msg::Point & point);

double computeCurvatureAtIndex(
  const PathPoints & points,
  size_t index);

double calculateMinimumLaneWidth(
  double curvature,
  double vehicle_length,
  double vehicle_width,
  double vehicle_front_overhang,
  double vehicle_wheel_base,
  double min_bound_margin,
  double max_width = 12.0);

void smoothDrivableBounds(
  autoware_planning_msgs::msg::Path & path,
  double max_width_change_rate = 0.5);

size_t findFirstNearestSegmentIndexWithSoftConstraints(
  const PathPoints & points,
  const geometry_msgs::msg::Pose & pose,
  double dist_threshold,
  double yaw_threshold);

double calcLateralOffset(
  const PathPoints & points,
  const geometry_msgs::msg::Point & target_point);

PathPoints cropPoints(
  const PathPoints & points,
  const geometry_msgs::msg::Point & target_point,
  size_t target_seg_idx,
  double forward_length,
  double backward_length);

}  // namespace igv_path_planner::utils

#endif  // IGV_PATH_PLANNER__UTILS__PATH_PLANNING_UTILS_HPP_
