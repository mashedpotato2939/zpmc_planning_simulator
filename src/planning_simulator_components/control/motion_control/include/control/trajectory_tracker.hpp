#ifndef TRAJECTORY_TRACKER_HPP_
#define TRAJECTORY_TRACKER_HPP_

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
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;

struct Point2D{
    double x;
    double y;
};

struct TrackerParam{
    double omega;               // 特征频率
    double dt;                  // 采样时间间隔
    double wheel_base;          // 轴距
    double wheel_base_ratio;    // 二三轴轴距与一四轴轴距的比值
    double max_steering_angle;  // 最大转向角，按照二三轴最大幅度计算
    double delta_limit;         // 最大转向角增量，防止控制指令突变
    double d_delta_limit;       // 最大转向角增量的增量，防止控制指令突变
    double predicted_ts;        // 预测时间，用于计算预瞄点位置
    double ky;                  // 预瞄点位置横向误差的增益
    double ky_reverse;
    double kvy;                 // 预瞄点位置横向速度误差的增益
    double k_com;
    double kyaw;                // 预瞄点位置航向误差的增益
    double kyaw_reverse;
    double vx_min;              // 比例系数中的最小线速度，防止在低速时控制指令过大
    bool is_forward;            // 是否为前向行驶
};

// 估计器增益
struct EstimatorGains {
    double k1;      // ω^3
    double k2;      // 3ω^2
    double k3;      // 3ω
};

// 三阶微分器状态
struct ThirdOrderState {
    double x1;  // 位置估计
    double x2;  // 速度估计
    double x3;  // 加速度估计
    double u;   // 控制输入 (用于计算jerk)
};

struct TrackerCmd{
  double axis1_steer_cmd;
  double axis2_steer_cmd;
  double axis3_steer_cmd;
  double axis4_steer_cmd;
  double velocity_cmd;
};

class TrajectoryTracker{
public:
    explicit TrajectoryTracker();
    ~TrajectoryTracker() = default;

    inline void setTrajectory(Trajectory::ConstSharedPtr traj_){
        trajectory_ = traj_;
        is_trajectory_new = true;
    }

    inline void setOdometry(Odometry::ConstSharedPtr odom_){
        odometry_ = odom_;
        is_odometry_new = true;
    }

    inline void setParameter(std::shared_ptr<TrackerParam> param_) {
        param = *param_;
        gains_.k1 = std::pow(param.omega, 3);
        gains_.k2 = 3.0 * std::pow(param.omega, 2);
        gains_.k3 = 3.0 * param.omega;
    }

    inline void setReverseCmd(bool is_forward) {
        param.is_forward = is_forward;
    }

    bool run();

    TrackerParam param;
    TrackerCmd cmd;

    double vx_debug;
    double xr_debug;
    double yr_debug;
    double yawr_debug, yaw_debug;
    double ey_debug, eyaw_debug, delta_debug, deltal_debug, d_delta_debug, d_deltal_debug;
    double kappar_debug, betar_debug;

private:
    Trajectory::ConstSharedPtr trajectory_;
    Odometry::ConstSharedPtr odometry_;

    bool is_odometry_new;
    bool is_trajectory_new;

    std::vector<TrajectoryPoint> output_tp_array_;
    std::optional<size_t> closest_idx_result;

    EstimatorGains gains_;
    ThirdOrderState state_x_;
    ThirdOrderState state_y_;
    ThirdOrderState state_yaw_;

    EstimatorGains gains_ey_;
    ThirdOrderState state_ey_;

    bool has_yaw_measurement_;
    double last_yaw_measurement_;
    double unwrapped_yaw_measurement_;

    bool checkData();
    Trajectory traj_preprocess(Trajectory traj, bool is_forward);
    Odometry calc_preview_point(double predicted_ts, Odometry current_odom);
    TrackerCmd calcSteerCmd(double ey, double ey_dot, double e_yaw, double betar, double kappar, double wr, double vx);
    std::optional<size_t> findNearestIndex(const std::vector<TrajectoryPoint>& points, Odometry preview_point, double max_dist, double max_yaw);
    bool errorCalc(const std::vector<TrajectoryPoint>& points, Odometry preview_point, double & ey, double & eyaw, double & beta_t, double & kappa);

    // 三阶线性跟踪微分器函数
    void update_differentiator(const nav_msgs::msg::Odometry& odom);
    void third_order_tracking_differentiator( double measured_value, ThirdOrderState& state, const EstimatorGains& gains);

    // utils 函数
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quat);
    double normalize_angle(double angle);
};
}


#endif
