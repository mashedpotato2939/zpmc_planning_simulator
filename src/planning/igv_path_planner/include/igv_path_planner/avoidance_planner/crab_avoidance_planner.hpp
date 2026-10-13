// Copyright 2026 ZPMC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IGV_PATH_PLANNER__CRAB_AVOIDANCE_PLANNER_HPP_
#define IGV_PATH_PLANNER__CRAB_AVOIDANCE_PLANNER_HPP_

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/utils.h>
#include <cmath>

#include <cstddef>
#include <cstdint>
#include <array>
#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace igv_path_planner
{
namespace cb_planning
{
struct Point2D{
  double x; 
  double y;

  Point2D() = default;
  Point2D(double x_, double y_) : x(x_), y(y_) {}
};

struct CrabTrajectorySample{
  double x;   // 车道坐标系下的纵向坐标
  double y;   // 车道坐标系下的横向坐标
  double dy;  // dy/dx，横向轨迹斜率
};

struct Edges{
  double left_edge;
  double right_edge;
};

struct OneObsResult{
  bool isCollision = false;   // 是否碰撞
  bool isLoE       = false;   // 全部在左侧
  bool isRoE       = false;   // 全部在右侧
  std::vector<Point2D> obsEdgePts; // 有效边界点（调试用）
};

using Vec2V = std::vector<Point2D>; 
struct BatchObsResult{
  bool isCollision = false;
  std::vector<bool> isLoE;      // 每个障碍物的结果
  std::vector<bool> isRoE;
  std::vector<Vec2V> obsEps;  // 每个障碍物的有效边界点
};

struct DrivableRange
{
  double yc;     // 中心线
  double Ll;     // 左边界
  double Lr;     // 右边界
  int    NoObs;  // 0: 无障碍  1: 有障碍
};

struct CrabResult{
  int    state = 1;   // 1 或 5 表示失败
  double ddy   = 0.0;
  double dx1   = 0.0;
  double dx2   = 0.0;
  double dx3   = 0.0;
};

struct CostResult{
  double cost;
  std::vector<double> Disafemin_set;
  double y_longitude_cost;
};

struct LocalPlanResult{
  int                isSuccess;          // 0/1
  int                collision_free_num;
  std::vector<double> opt_traj;          // 11×1，与 MATLAB 顺序一致
  std::vector<double> return_traj;       // 避障段之后接回参考线的轨迹；为空表示已经在参考线上
};

struct Object_Info{
  size_t obs_id;
  uint8_t classification = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
  double obs_global_pos_x;
  double obs_global_pos_y;
  double obs_global_heading_angle;
  double obs_length;
  double obs_width;
  double obs_velocity;
  double obs_body_vx = 0.0;  // 障碍物自身坐标系下的有符号速度
  double obs_body_vy = 0.0;
  double obs_yaw_rate = 0.0;
  std::vector<Point2D> edge_points;
};

struct Ego_Info{
  double ego_pos_x;
  double ego_pos_y;
  double ego_pos_vy;
  double ego_pos_vx;
};

struct Planning_Para{
  // 车辆运动学参数
  double ego_length;
  double ego_width;
  double steer_angle_max;
  double steer_angular_vel_max;
  double dymax;
  double ddymax;
  double ddymin;

  double Min_Collision_free_distance;
  double start_point_mindis_threshold;
  
  // 在直线车道上寻找一个参考点作为原点，统一路径坐标，沿车道方向为x轴正方向
  double lane_x_bias;                  
  double lane_y_bias;
  double lane_yaw_bias;

  // 横向采样参数
  double lane_sampling_begin_y;       
  double lane_sampling_end_y;
  double lane_y_sampling_interval;
  double lane_begin_y;
  double lane_width;
  int lane_num;

  // 纵向采样参数，lane_end_x为车道纵向长度
  double lane_end_x;
  double local_map_forward_x;
  double local_map_backward_x;
  double lane_x_sampling_interval;
  double return_length_lateral_ratio = 5.0;  // 回归段纵向长度 / 横向偏移量
  double reference_merge_slope_tolerance = 0.02;  // 接回参考线时允许的 dy/dx 偏差
  bool enable_dynamic_obstacle_avoidance = false;
  double obstacle_clearance = 0.0;
};

using nav_msgs::msg::Odometry; 
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_perception_msgs::msg::PredictedObjects;

class CB_Path_Planning{
public:
    explicit CB_Path_Planning();
    ~CB_Path_Planning() = default;

    // Param param;
    inline void setParameter(std::shared_ptr<Planning_Para> param_) {
        if (std::abs(planning_para.lane_x_bias - param_->lane_x_bias) > 0.1 ||
            std::abs(planning_para.lane_y_bias - param_->lane_y_bias) > 0.1 ||
            std::abs(std::remainder(
              planning_para.lane_yaw_bias - param_->lane_yaw_bias, 2.0 * M_PI)) > 0.05) {
            opt_traj_prev.clear();
            opt_traj_inreplanning.clear();
            opt_return_inreplanning.clear();
            is_replanning = false;
        }
        planning_para = *param_;
    }

    // 上层传入 Q-turn 之前的连续直行/变道参考线，末端须可平顺接入。
    inline void setReferencePath(const Path::SharedPtr reference_path) {
        reference_path_ptr_ = reference_path;
    }

    inline void setOdometry(const Odometry::SharedPtr ego_state_ptr) {
        ego_state_ptr_ = ego_state_ptr;
    }

    inline void setPrevTrajectory(const Trajectory::SharedPtr prev_trajectory_msg) {
        prev_trajectory_msg_ = prev_trajectory_msg;
    }

    inline void setObjectsInfo(const PredictedObjects & objects) {
      objects_info.clear();
      objects_info.reserve(objects.objects.size());
      for(size_t i = 0; i < objects.objects.size(); ++i){
        Object_Info obs_info;
        obs_info.obs_id = i;
        if (!objects.objects[i].classification.empty()) {
          const auto best = std::max_element(
            objects.objects[i].classification.begin(), objects.objects[i].classification.end(),
            [](const auto & a, const auto & b) { return a.probability < b.probability; });
          obs_info.classification = best->label;
        }
        obs_info.obs_global_pos_x = objects.objects[i].kinematics.initial_pose_with_covariance.pose.position.x;
        obs_info.obs_global_pos_y = objects.objects[i].kinematics.initial_pose_with_covariance.pose.position.y;
        obs_info.obs_global_heading_angle = tf2::getYaw(objects.objects[i].kinematics.initial_pose_with_covariance.pose.orientation);
        obs_info.obs_length = objects.objects[i].shape.dimensions.x;
        obs_info.obs_width = objects.objects[i].shape.dimensions.y;
        obs_info.obs_body_vx = objects.objects[i].kinematics.initial_twist_with_covariance.twist.linear.x;
        obs_info.obs_body_vy = objects.objects[i].kinematics.initial_twist_with_covariance.twist.linear.y;
        obs_info.obs_yaw_rate = objects.objects[i].kinematics.initial_twist_with_covariance.twist.angular.z;
        obs_info.obs_velocity = std::hypot(obs_info.obs_body_vx, obs_info.obs_body_vy);
        objects_info.push_back(obs_info);
      }
    }

    static inline bool cmpPt(const Point2D& a, const Point2D& b){
        return a.x < b.x || (a.x == b.x && a.y < b.y);}

    bool run();

    const Path & outputPath() const { return output_path; }

    const std::vector<CrabTrajectorySample> & sampledTrajectory() const {
        return sampled_trajectory_;
    }
    
    std::vector<Point2D> calcObsEdgePoints(std::vector<Point2D> eps_for_OneObstacle, double lower, double upper);

    /* 计算可行驶范围 */
    DrivableRange calcDrivableRange(Vec2V obs_edge_points, double ego_pos_x, int ref_lane);
    DrivableRange calcDrivableRange(Vec2V obs_edge_points, double ego_pos_x, double ref_points);

    LocalPlanResult local_plan(std::vector<double> X_local_set, double start_point_x, double start_point_y, double start_point_dy, Vec2V edge_local_points,
        double Min_Collision_free_dis, std::vector<double> Yc_set, std::vector<int> NoObs_set, double dymax, double ddymax, double ddymin);

    LocalPlanResult local_replan(std::vector<double> X_local_set, double start_point_x, double start_lane_x, double start_point_y, double start_point_dy, Vec2V edge_local_points,
        double Min_Collision_free_dis, std::vector<double> Yc_set, std::vector<int> NoObs_set, double dymax, double ddymax, double ddymin);

    void calcPosy_Vy(const std::vector<double> & traj, double ego_pos_x, double & ego_pos_y, double & ego_pos_vy);

private:
    std::optional<CrabTrajectorySample> referencePointAt(double lane_x) const;
    std::optional<std::vector<double>> buildReturnTrajectory(
        const std::vector<double> & avoidance_traj);

    /* 将动态障碍物在预测时间窗内的占据区域转换为车道坐标系下的四点包围框 */
    Vec2V calcDynamicObstacleEdgePoints(const Object_Info & obstacle, double local_start_x) const;

    /* 投影到轨迹点，更新规划起点位置 */
    void updateStartPoint(Trajectory::SharedPtr prev_trajectory_msg, double ego_pos_x, double ego_pos_y, double ego_pos_vx, double ego_pos_vy,
                          double & start_point_x, double & start_point_y, double & start_point_dy, double & vx_plan);

    /* 斜行轨迹生成 */
    CrabResult crabwalking_traj_generate(double y0, double y1, double dy0, double dx, double dymax, double ddymax, double ddymin);

    /* 计算车辆中心点轨迹的坐标 */
    double calcTrajPoint(int state, double y0, double y1, double dy0, double ddy, double dx, double dx1, double dx3, double ego_pos_x);
    
    /* AGV斜行运动轨迹的边界计算 */
    void calcEdges(int state, double y0, double y1, double dy0, double ddy, double dx, double dx1, double dx2, double dx3, double dx4, 
                    double ego_pos_x, double& leftEdge, double& rightEdge); 

    /* 斜行运动的碰撞检测 */
    BatchObsResult CollisionCheck(int state, double y0, double y1, double dy0, double ddy,
                                double dx, double dx1, double dx2, double dx3, double dx4, const Vec2V& edge_points);
    OneObsResult CollisionCheck_for_OneObstacle(int state, double y0, double y1, double dy0, double ddy,
                                double dx, double dx1, double dx2, double dx3, double dx4, const Vec2V& edge_points);

    /* 代价计算 */
    double calc_LongiCollisionDis(std::vector<Point2D> obs_edge_points, double ego_pos_x, double ego_pos_y, double longdis_safe);
    CostResult calcCost(int state, double y0, double y1, double dy0, double ddy, double dx, double dx1, double dx3, double dx4, 
                    std::vector<double> X_local_set, std::vector<double> Yc_set, std::vector<int> NoObs_set);
    
    std::vector<Point2D> edge_points;
    std::vector<Point2D> edge_points_for_OneObstacle;
    std::vector<Edges> edges;

    std::vector<double> opt_traj_prev;  //记录上一次规划的轨迹，用于再规划
    std::vector<std::array<double, 2>> opt_traj_inreplanning;
    std::vector<double> opt_return_inreplanning;
    std::vector<CrabTrajectorySample> sampled_trajectory_;
    bool is_replanning = false;
    std::vector<double> Yc_set_prev;
    std::vector<int> NoObs_set_prev;
    std::vector<double> Ll_set_prev;
    std::vector<double> Lr_set_prev;
    
    // 由上层模块订阅速度规划结果，利用上一步轨迹速度信息作为横向规划的约束条件
    Trajectory::SharedPtr prev_trajectory_msg_;

    // 保存路径规划模块上一步的规划结果，投影在当前步规划的局部地图上
    Path::SharedPtr reference_path_ptr_;
    std::vector<Point2D> global_reference_path;
    std::vector<Point2D> reference_lookup_path_;

    Path output_path;

    // 由上层模块订阅车辆状态信息，获取车辆当前的位姿、速度等信息
    Odometry::SharedPtr ego_state_ptr_;

    Ego_Info ego_info;
    std::vector<Object_Info> objects_info;
    Planning_Para planning_para{};

    double start_point_x;
    double start_point_y;
    double start_point_dy;
    double vx_plan;

    bool is_first_plan;
};
} // namespace cb_planning

}  // namespace igv_path_planner

#endif  // IGV_PATH_PLANNER__CRAB_AVOIDANCE_PLANNER_HPP_
