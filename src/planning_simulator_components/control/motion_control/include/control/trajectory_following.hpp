#ifndef TRAJECTORY_FOLLOWING_HPP_
#define TRAJECTORY_FOLLOWING_HPP_

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

#include <tf2/utils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define _USE_MATH_DEFINES      //使用PI
#include <cmath>
#include <algorithm>

#include "interpolation.hpp"
#include "geometry.hpp"
#include "trajectory.hpp"

namespace motion_controller
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

struct TF_Cmd
{
  double steer_cmd;
  double velocity_cmd;
};

struct Param
{
  // Global Parameters
  double wheel_base;
  double max_steering_angle;  // [rad]

  // Algorithm Parameters
  double resampling_ds;
  double ld_velocity_ratio;
  double ld_curvature_ratio;
  double long_ld_lateral_error_threshold;
  double ld_lateral_error_ratio;
  double max_lookahead_distance;
  double min_lookahead_distance;
  double curvature_calculation_distance;   //计算曲率在最近点前后按照该距离取两个点
  uint32_t min_traj_points_size;
  double max_traj_points_interval;
  double front_to_rear_steering_ratio;    // 前轮转向角与后轮转向角的比值
};

class TrajectoryFollowing{
public:
    explicit TrajectoryFollowing();
    ~TrajectoryFollowing() = default;

    inline void setTrajectory(Trajectory::ConstSharedPtr traj_){
        trajectory_ = traj_;
        is_trajectory_new = true;
    }

    inline void setOdometry(Odometry::ConstSharedPtr odom_){
        odometry_ = odom_;
        is_odometry_new = true;
    }

    inline void setParameter(std::shared_ptr<Param> param_) {param = *param_;}
    inline double get_lateral_error() {return lateral_error_d;}
    double get_longtitude_error();
    bool run();

    Param param;
    TF_Cmd tf_cmd;

private:

    Trajectory::ConstSharedPtr trajectory_;
    Odometry::ConstSharedPtr odometry_;

    bool is_trajectory_new = false;
    bool is_odometry_new = false;

    Trajectory resampled_trajectory;
    std::vector<TrajectoryPoint> output_tp_array_;
    geometry_msgs::msg::Point preview_point;
    std::optional<size_t> closest_idx_result;
    
    bool use_wp_replace_tgt_pose;
    double lookahead_distance;

    double lateral_error_d;
    double longtitude_error;

    bool checkData();
    Trajectory resampleTrajectory(
        const autoware_planning_msgs::msg::Trajectory & input_trajectory, const double resample_interval);
    double CalcLookaheadDistance(const double lateral_error, const double curvature, const double velocity, const double min_ld);
    std::pair<bool, geometry_msgs::msg::Point> FindLookaheadPoint(geometry_msgs::msg::Pose pose);
    float calcSteerCmd(const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Pose & current_pose);

    //utils 函数
    void insertOrientation(std::vector<geometry_msgs::msg::Pose> & points, const bool is_driving_forward);
    double calcCurvature(const size_t closest_idx);
    int32_t findNextPointIdx(int32_t search_start_idx, geometry_msgs::msg::Pose pose);
    std::pair<bool, geometry_msgs::msg::Point> lerpNextTarget(int32_t next_wp_idx, geometry_msgs::msg::Pose pose);

};
}


#endif