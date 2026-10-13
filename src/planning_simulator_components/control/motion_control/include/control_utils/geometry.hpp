#ifndef CONTROL_UTILS_GEOMETRY_HPP_
#define CONTROL_UTILS_GEOMETRY_HPP_

#include <exception>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <tf2/utils.h>

namespace control_utils::geometry{

using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Twist;

/*获取x,y,z坐标*/
inline geometry_msgs::msg::Point getPoint(const TrajectoryPoint & p){
  return p.pose.position;
}

inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::Point & p){
  return p;
}

/*计算2D欧式距离*/
double calcDistance2d(const Point & point1, const Point & point2);

/*计算2D欧式距离的平方*/
double calcSquaredDistance2d(const geometry_msgs::msg::Point & point1, const geometry_msgs::msg::Point & point2);
double calcSquaredDistance2d(const TrajectoryPoint & point1, const geometry_msgs::msg::Point & point2);

/*将弧度限制在-PI到PI之间*/
double normalizeRadian(const double rad, const double min_rad = -M_PI);

/*求航向角，arctan(y/x)*/
double calcAzimuthAngle(
    const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to);

/*求俯仰角，arctan(z/sqrt(x^2+y^2))*/
double calcElevationAngle(
    const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to);

/*根据两点线段的矢量方向和初始点航向角的插值，判断车是前进还是倒车*/
bool isDrivingForward(const TrajectoryPoint & src_pose, const TrajectoryPoint & dst_pose);

/*从欧拉角生成四元数*/
geometry_msgs::msg::Quaternion createQuaternionFromRPY(
    const double roll, const double pitch, const double yaw);

void setOrientation(const geometry_msgs::msg::Quaternion & orientation, geometry_msgs::msg::Pose & p);

/*根据3点计算曲率*/
double calcCurvature(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
    const geometry_msgs::msg::Point & p3);

/*根据车体坐标系计算目标点的相对坐标*/
geometry_msgs::msg::Point transformToRelativeCoordinate2D(
    const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & origin);
}

#endif