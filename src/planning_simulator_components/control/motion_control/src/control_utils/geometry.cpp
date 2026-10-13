#include "geometry.hpp"

namespace control_utils::geometry{

double calcDistance2d(const Point & point1, const Point & point2)
{
    const auto p1 = getPoint(point1);
    const auto p2 = getPoint(point2);
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

double calcSquaredDistance2d(const geometry_msgs::msg::Point & point1, const geometry_msgs::msg::Point & point2)
{
    const auto p1 = point1;
    const auto p2 = point2;
    const auto dx = p1.x - p2.x;
    const auto dy = p1.y - p2.y;
    return dx * dx + dy * dy;
}

double calcSquaredDistance2d(const TrajectoryPoint & point1, const geometry_msgs::msg::Point & point2)
{
    const auto p1 = point1.pose.position;
    const auto p2 = point2;
    const auto dx = p1.x - p2.x;
    const auto dy = p1.y - p2.y;
    return dx * dx + dy * dy;
}

double normalizeRadian(const double rad, const double min_rad)
{
  const auto max_rad = min_rad + 2 * M_PI;

  const auto value = std::fmod(rad, 2 * M_PI);
  if (min_rad <= value && value < max_rad) {
    return value;
  }

  return value - std::copysign(2 * M_PI, value);
}

double calcAzimuthAngle(
    const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to)
{
  const double dx = p_to.x - p_from.x;
  const double dy = p_to.y - p_from.y;
  return std::atan2(dy, dx);
}

double calcElevationAngle(
    const geometry_msgs::msg::Point & p_from, const geometry_msgs::msg::Point & p_to)
{
  const double dz = p_to.z - p_from.z;
  const double dist_2d = calcDistance2d(p_from, p_to);
  return std::atan2(dz, dist_2d);
}

bool isDrivingForward(const TrajectoryPoint & src_pose, const TrajectoryPoint & dst_pose)
{
  // check the first point direction
  const double src_yaw = tf2::getYaw(src_pose.pose.orientation);
  const double pose_direction_yaw = calcAzimuthAngle(getPoint(src_pose), getPoint(dst_pose));
  return std::fabs(normalizeRadian(src_yaw - pose_direction_yaw)) < M_PI / 2.0;
}

geometry_msgs::msg::Quaternion createQuaternionFromRPY(
    const double roll, const double pitch, const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}

void setOrientation(const geometry_msgs::msg::Quaternion & orientation, geometry_msgs::msg::Pose & p)
{
  auto pose = p;
  pose.orientation = orientation;
  p = pose;
}

double calcCurvature(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
    const geometry_msgs::msg::Point & p3)
{
    // Calculation details are described in the following page
    // https://en.wikipedia.org/wiki/Menger_curvature
    const double denominator =
        calcDistance2d(p1, p2) * calcDistance2d(p2, p3) * calcDistance2d(p3, p1);
    if (std::fabs(denominator) < 1e-10) {
        throw std::runtime_error("points are too close for curvature calculation.");
    }
    return 2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / denominator;
}

geometry_msgs::msg::Point transformToRelativeCoordinate2D(
    const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Pose & origin)
{
  // translation
  geometry_msgs::msg::Point trans_p;
  trans_p.x = point.x - origin.position.x;
  trans_p.y = point.y - origin.position.y;

  // rotation (use inverse matrix of rotation)
  double yaw = tf2::getYaw(origin.orientation);

  geometry_msgs::msg::Point res;
  res.x = (cos(yaw) * trans_p.x) + (sin(yaw) * trans_p.y);
  res.y = ((-1) * sin(yaw) * trans_p.x) + (cos(yaw) * trans_p.y);
  res.z = origin.position.z;

  return res;
}

}