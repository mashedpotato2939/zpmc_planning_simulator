#ifndef MOTION_CONTROLLER_HPP_
#define MOTION_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_control_msgs/srv/map_creator_service.hpp>
#include <autoware_control_msgs/srv/traj_creator_service.hpp>

#include <pacmod_msgs/msg/system_cmd_float.hpp>
#include <pacmod_msgs/msg/system_cmd_int.hpp>
#include <pacmod_msgs/msg/steering_cmd.hpp>
#include <pacmod_msgs/srv/upper_test.hpp>

#include <reverse_vehicle_msg/msg/vehicle_reversed.hpp>

#include <memory>

#include "trajectory_following.hpp"
#include "trajectory_tracker.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

// 顺序按闭合多边形排列：主路 → 横臂 → 闭合
const double map_edge_points[8][3] = {
  { 60.0, 30.0, 0.0 },  // 左下
  { 60.0, 15.0, 0.0 },  // 左上
  { 5.0, 15.0, 0.0 },  // 右上
  { 5.0, 30.0, 0.0 },  // 横臂右上
  { 30.0, 30.0, 0.0 },  // 横臂右上端
  { 30.0, 50.0, 0.0 },  // 横臂右下端
  { 45.0, 50.0, 0.0 },  // 横臂右下
  { 45.0, 30.0, 0.0 }   // 右下
};

namespace motion_controller
{
using autoware_control_msgs::msg::Control;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using autoware_control_msgs::srv::MapCreatorService;
using autoware_control_msgs::srv::TrajCreatorService;

using pacmod_msgs::msg::SteeringCmd;
using pacmod_msgs::msg::SystemCmdInt;
using pacmod_msgs::msg::SystemCmdFloat;
using srvUpperTest = pacmod_msgs::srv::UpperTest;

struct UpperTest{
  uint32_t  index;
  bool      is_new;
  bool      enable;
  bool      bool_cmd;
  int32_t   int_cmd;
  double    float_cmd;
};

class MotionController : public rclcpp::Node
{
public:
  explicit MotionController(const rclcpp::NodeOptions & options);
  ~MotionController() = default;

  Trajectory::ConstSharedPtr trajectory_ ;
  Trajectory::ConstSharedPtr trajectory_reverse_ ;
  Odometry::ConstSharedPtr odometry_ = nullptr;
  Odometry odometry_final;

  bool is_odometry_new = false;
  bool is_trajectory_new = false;

private:
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<reverse_vehicle_msg::msg::VehicleReversed>::SharedPtr sub_reverse_state_;

  rclcpp::Publisher<Control>::SharedPtr pub_cmd_;
  rclcpp::Publisher<SystemCmdInt>::SharedPtr pub_shift_cmd_;
  rclcpp::Publisher<SystemCmdFloat>::SharedPtr pub_brake_cmd_;
  rclcpp::Publisher<SystemCmdFloat>::SharedPtr pub_motor_vel_cmd_;
  rclcpp::Publisher<SteeringCmd>::SharedPtr pub_steering_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr dbg_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr traj_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr traj_yaw_vs_s_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr traj_kappa_vs_s_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr traj_beta_vs_s_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr traj_debug_timer_;

  rclcpp::Client<MapCreatorService>::SharedPtr map_client_;           // 地图请求客户端
  rclcpp::Client<TrajCreatorService>::SharedPtr traj_client_;         // 全局轨迹请求客户端

  rclcpp::Service<srvUpperTest>::SharedPtr srv_upper_test_;
  UpperTest upper_test;

  std::shared_ptr<TrajectoryFollowing> traj_follower_;
  std::shared_ptr<TrajectoryTracker> tracker_;
  std::shared_ptr<Param> param_;
  std::shared_ptr<TrackerParam> tracker_param_;
  bool use_maptraj_server;
  bool publish_trajectory_debug;

  void Vehicle_Init();

  void onTimer();
  void publishTrajectoryDebug();
  bool isDataReady();

  template <typename T>
  bool connect_server(const T &client); // 链接服务器

  bool map_request();                   // 地图请求
  bool is_map_ready;

  void srv_upper_test_callback(const srvUpperTest::Request::SharedPtr request,
    srvUpperTest::Response::SharedPtr response);
};

}  // namespace motion_controller

#endif
