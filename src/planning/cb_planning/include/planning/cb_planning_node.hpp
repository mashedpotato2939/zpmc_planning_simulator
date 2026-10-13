#ifndef CRABWALKING_PLANNING__NODE_HPP_
#define CRABWALKING_PLANNING__NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "cb_velocity_planning.hpp"
#include "cb_path_planning.hpp"
#include "struct_types.hpp"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <config_reader.hpp>
#include "vehicle_sim_node.hpp"

namespace cb_planning
{
using nav_msgs::msg::Odometry; 
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Point;


class CrabWalking_Planning : public rclcpp::Node
{
public:
  explicit CrabWalking_Planning(const rclcpp::NodeOptions & node_options);

protected: 

  // 可视化相关的变量
  rclcpp::Publisher<MarkerArray>::SharedPtr lane_rviz_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr obstacle_rviz_pub_;
  rclcpp::Publisher<Marker>::SharedPtr vehicle_rviz_pub_;
  rclcpp::Publisher<Marker>::SharedPtr dynamic_obs_rviz_pub_;
  rclcpp::Publisher<Marker>::SharedPtr traj_rviz_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr dbg_pub_;
  // 仿真节点同步话题
  rclcpp::Publisher<msgUpperTest>::SharedPtr sim_sync_pub_;

  // 输出轨迹相关变量
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;

  // 定位相关变量
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  Odometry::SharedPtr ego_state_ptr_;
  bool is_odom_new;
  inline void odom_callback(const Odometry::SharedPtr msg){
    ego_state_ptr_ = msg;
    is_odom_new = true;
  };

  // 服务回调函数
  rclcpp::Service<srvUpperTest>::SharedPtr srv_upper_test_;
  UpperTest upper_test;
  void srv_upper_test_callback(const srvUpperTest::Request::SharedPtr request, 
                              srvUpperTest::Response::SharedPtr response);

  // 配置文件读取的接口
  std::unique_ptr<ConfigReader> planning_config_;

  // 定时器与回调函数
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();
  
  // 可视化函数
  void lane_vis_update();
  void vehicle_vis_update();
  void obstacle_vis_update();
  void moving_obstacle_vis_update();
  std::vector<Point2D> orderPolygon(std::vector<Point2D> pts);
  void traj_vis_update(std::vector<Point2D> traj_plot_xy, std::vector<double> x_local_map_set, 
    std::vector<double> Ll_set, std::vector<double> Lr_set);
  std::vector<Point2D> calc_XYplot(std::vector<double> opt_traj, double x_start, double x_end, double sample_point_num_pm);

  // 初始化函数
  void lane_info_init();
  void static_obstacles_info_init();
  std::vector<Point2D> calc_static_obs_edge_points(const Object_Info &object_info);
  void moving_obstacle_info_init();
  std::vector<Point2D> moving_obs_edge_points_update(const Object_Info &object_info);
  void ego_info_init();
  void planning_para_init();
  
  void generate_reference_path();
  std::vector<Point2D> global_reference_path;

  // 规划算法相关变量
  Lane_Info lane_info;
  Ego_Info ego_info;
  std::vector<Object_Info> objects_info;
  Object_Info moving_object_info;
  Planning_Para planning_para;

  std::unique_ptr<CB_Velocity_Planning> vel_planning;
  std::unique_ptr<CB_Path_Planning> path_planning;

  double start_point_x;
  double start_point_y;
  double start_point_dy;

  bool isbraking;
  double t_interval;
  double vx_ref;
  double vx_plan;
  double ax_drive;
  double ax_brake;
  uint32_t plot_cnt;

  std::vector<SpeedPoint> speed_points_prev;

  std::vector<double> opt_traj_prev;  //记录上一次规划的轨迹，用于再规划
  std::vector<std::array<double, 2>> opt_traj_inreplanning;
  bool is_replanning = false;
  std::vector<double> Yc_set_prev;
  std::vector<int> NoObs_set_prev;
  std::vector<double> Ll_set_prev;
  std::vector<double> Lr_set_prev;

  bool Enable_static_obstacle_avoidance_scenario;
  bool Enable_overtaking_scenario;
  
  // 超车场景下，移动障碍物相关变量
  double dynamic_obstacle_pos_x;
  double dynamic_obstacle_pos_y;
  bool is_moving;
  bool is_overtaking = false;

  bool is_first_plan;

};

}

#endif  // AUTOWARE_PATH_SAMPLER__NODE_HPP_