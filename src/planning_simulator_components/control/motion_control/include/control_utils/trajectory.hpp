#ifndef CONTROL_UTILS_TRAJECTORY_HPP_
#define CONTROL_UTILS_TRAJECTORY_HPP_

#include <Eigen/Geometry>
#include <rclcpp/logging.hpp>

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <algorithm>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace control_utils::trajectory{

using autoware_planning_msgs::msg::TrajectoryPoint;

/*求目标点到轨迹的横向偏移误差*/
double calcLateralOffset(const std::vector<TrajectoryPoint> & points, const geometry_msgs::msg::Point & p_target);

/*将轨迹中距离很近的点消除掉*/
std::vector<TrajectoryPoint> removeOverlapPoints(const std::vector<TrajectoryPoint> & points, const size_t start_idx = 0);

/*找到轨迹中到目标点最近的线段*/
size_t findNearestSegmentIndex(const std::vector<TrajectoryPoint> & points, const geometry_msgs::msg::Point & point);

/*找到轨迹中到目标点最近的点*/
size_t findNearestIndex(const std::vector<TrajectoryPoint> &points, const geometry_msgs::msg::Point &point);

std::optional<size_t> findNearestIndex(const std::vector<TrajectoryPoint> & points, const geometry_msgs::msg::Pose & pose,
    const double max_dist = std::numeric_limits<double>::max(), const double max_yaw = std::numeric_limits<double>::max());

}

#endif