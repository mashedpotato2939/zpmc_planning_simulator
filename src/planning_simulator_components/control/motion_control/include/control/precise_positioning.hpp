#ifndef PRECISE_POSITIONING_HPP_
#define PRECISE_POSITIONING_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "interpolation.hpp"
#include "geometry.hpp"
#include "trajectory.hpp"

namespace motion_controller
{
using autoware_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;

class PrecisePositioning{
public:
    explicit PrecisePositioning();
    ~PrecisePositioning() = default;

    bool run();

private:
    Trajectory::ConstSharedPtr trajectory_;
    Odometry::ConstSharedPtr odometry_;
};
}


#endif