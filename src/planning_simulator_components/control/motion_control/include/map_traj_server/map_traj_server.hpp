#ifndef MAP_TRAJ_SERVER_HPP_
#define MAP_TRAJ_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "autoware_control_msgs/srv/map_creator_service.hpp"
#include "autoware_control_msgs/srv/traj_creator_service.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
//20250912,zhangyue
#include <std_msgs/msg/bool.hpp>

#include <fstream>
#include <sstream>
#include <vector>

namespace map_traj_server
{
using nav_msgs::msg::Odometry;
using autoware_control_msgs::srv::MapCreatorService;
using autoware_control_msgs::srv::TrajCreatorService;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using geometry_msgs::msg::Point;

using std::placeholders::_1;
using std::placeholders::_2;

struct CrabResult{
  int    state = 1;   // 1 或 5 表示失败
  double ddy   = 0.0;
  double dx1   = 0.0;
  double dx2   = 0.0;
  double dx3   = 0.0;
};

class MapTrajServer : public rclcpp::Node
{
public:
  explicit MapTrajServer(const rclcpp::NodeOptions & options);
  ~MapTrajServer() = default;

private:
  //定时器，在定时函数中读取里程计的位置，将车的位置显示在rviz中
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();

  //订阅车辆位置，并显示车的信息
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Publisher<Marker>::SharedPtr vehicle_rviz_pub_;
  Odometry::ConstSharedPtr odometry_ = nullptr;

  //地图请求服务回调函数：创建地图范围，并且显示在rviz中
  bool is_map_create_success;
  rclcpp::Publisher<MarkerArray>::SharedPtr map_rviz_pub_;
  rclcpp::Service<MapCreatorService>::SharedPtr map_server_;
  void srv_map_callback(const MapCreatorService::Request::SharedPtr request,
    MapCreatorService::Response::SharedPtr response);

  //轨迹请求服务回调函数：根据服务的选项，生成一条轨迹返回，并且显示在rviz中
  bool is_traj_create_success;
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<Marker>::SharedPtr traj_rviz_pub_;
  rclcpp::Service<TrajCreatorService>::SharedPtr traj_server_;
  void srv_traj_callback(const TrajCreatorService::Request::SharedPtr request,
    TrajCreatorService::Response::SharedPtr response);
  bool traj_creator(size_t type, size_t is_reverse);
  void traj_check(std::vector<Point> edge_points, double vehicle_length, double vehicle_width);

  /* 斜行轨迹生成 */
  CrabResult crabwalking_traj_generate(double y0, double y1, double dy0, double dx, double dymax, double ddymax, double ddymin);
  void calcPosy_Vy(std::vector<double> traj, double ego_pos_x, double & ego_pos_y, double & ego_pos_vy);

  Trajectory traj_planned;
  bool traj_creator_straight(Point & sp, Point & dp, double vel_max, double acc, size_t is_reverse);
  bool traj_creator_elbow_bend(Point & sp, Point & dp, double vel_max, double acc, double radius, size_t is_reverse);
  bool traj_creator_lane_change(Point & sp, Point & dp, double vel_target, double acc, size_t is_reverse);
  bool traj_creator_crabwalking(Point & sp, Point & dp, double vel_target, double acc, double rotate_angle, double bias_x, double bias_y, size_t is_reverse);
  bool traj_creator_from_txt(const std::string & filename);
  bool traj_creator_turn_optimize(Point &sp, Point &dp, double vx, double acc, double R, double rotate_angle, double bias_x, double bias_y, size_t is_reverse);

  void sampling_points_xy(int pts_num, double ds_1, double ds_2, double dk_ds, double kappa_max, std::vector<double>& x_pts, std::vector<double>& y_pts);
  void sampling_points_yaw(int pts_num, double ds_1, double ds_2, double dk_ds, double kappa_max, std::vector<double>& theta_pts, std::vector<double>& kappa_pts);
};

}  // namespace map_traj_server

#endif  