#include "trajectory.hpp"
#include "geometry.hpp"
#include <iostream>

namespace control_utils::trajectory{

double calcLateralOffset(const std::vector<TrajectoryPoint> & points, const geometry_msgs::msg::Point & p_target){
    
    const auto overlap_removed_points = removeOverlapPoints(points, 0);
    const size_t seg_idx = findNearestSegmentIndex(overlap_removed_points, p_target);

    const auto p_indices = overlap_removed_points.size() - 2;
    const auto p_front_idx = (p_indices > seg_idx) ? seg_idx : p_indices;
    const auto p_back_idx = p_front_idx + 1;

    const auto p_front = overlap_removed_points.at(p_front_idx).pose.position;
    const auto p_back = overlap_removed_points.at(p_back_idx).pose.position;

    const Eigen::Vector3d segment_vec{p_back.x - p_front.x, p_back.y - p_front.y, 0.0};
    const Eigen::Vector3d target_vec{p_target.x - p_front.x, p_target.y - p_front.y, 0.0};
    
    const Eigen::Vector3d cross_vec = segment_vec.cross(target_vec);
    return cross_vec(2) / segment_vec.norm();
}


std::vector<TrajectoryPoint> removeOverlapPoints(const std::vector<TrajectoryPoint> & points, const size_t start_idx)
{
  if (points.size() < start_idx + 1) {
    return points;
  }

  std::vector<TrajectoryPoint> dst;
  dst.reserve(points.size());

  for (size_t i = 0; i <= start_idx; ++i) {
    dst.push_back(points.at(i));
  }

  constexpr double eps = 1.0E-08;
  for (size_t i = start_idx + 1; i < points.size(); ++i) {
    const auto prev_p = dst.back().pose.position;
    const auto curr_p = points.at(i).pose.position;
    if (std::abs(prev_p.x - curr_p.x) < eps && std::abs(prev_p.y - curr_p.y) < eps) {
      continue;
    }
    dst.push_back(points.at(i));
  }

  return dst;
}

size_t findNearestSegmentIndex(const std::vector<TrajectoryPoint> & points, const geometry_msgs::msg::Point & point)
{
  const size_t nearest_idx = findNearestIndex(points, point);

  if (nearest_idx == 0) {
    return 0;
  }
  if (nearest_idx == points.size() - 1) {
    return points.size() - 2;
  }

  const auto p_front = points.at(nearest_idx).pose.position;
  const auto p_back = points.at(nearest_idx + 1).pose.position;

  const Eigen::Vector3d segment_vec{p_back.x - p_front.x, p_back.y - p_front.y, 0};
  const Eigen::Vector3d target_vec{point.x - p_front.x, point.y - p_front.y, 0};
  
  //由最近点和轨迹点中下一个点组成的线段与目标点和最近点的线段点积，如果为负，就是前一个线段
  const double signed_length =  segment_vec.dot(target_vec) / segment_vec.norm();

  if (signed_length <= 0) {
    return nearest_idx - 1;
  }

  return nearest_idx;
}

size_t findNearestIndex(const std::vector<TrajectoryPoint> &points, const geometry_msgs::msg::Point &point)
{
  if (points.empty()) {
    throw std::invalid_argument("validateNonEmpty(): Points is empty.");
  }

  double min_dist = std::numeric_limits<double>::max();
  size_t min_idx = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto dist = control_utils::geometry::calcSquaredDistance2d(points.at(i), point);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return min_idx;
}

std::optional<size_t> findNearestIndex(const std::vector<TrajectoryPoint> & points, const geometry_msgs::msg::Pose & pose,
    const double max_dist, const double max_yaw){
  
  const double max_squared_dist = max_dist * max_dist;
  double min_squared_dist = std::numeric_limits<double>::max();
  bool is_nearest_found = false;
  size_t min_idx = 0;
  
  for (size_t i = 0; i < points.size(); ++i) {
    const auto squared_dist = control_utils::geometry::calcSquaredDistance2d(points.at(i), pose.position);
    if (squared_dist > max_squared_dist || squared_dist >= min_squared_dist) {
      continue;
    }

    const auto base_yaw = tf2::getYaw(points.at(i).pose.orientation);
    const auto target_yaw = tf2::getYaw(pose.orientation);
    auto yaw = control_utils::geometry::normalizeRadian(target_yaw - base_yaw);
    if (std::fabs(yaw) > max_yaw) {
      continue;
    }

    min_squared_dist = squared_dist;
    min_idx = i;
    is_nearest_found = true;
  }

  if (is_nearest_found) {
    return min_idx;
  }
  return std::nullopt;
}

}