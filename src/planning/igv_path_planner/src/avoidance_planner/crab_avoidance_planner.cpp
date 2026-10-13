// Copyright 2026 ZPMC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "igv_path_planner/avoidance_planner/crab_avoidance_planner.hpp"

#include <autoware_perception_msgs/msg/shape.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <optional>
#include <utility>
#include <iostream>

namespace igv_path_planner
{
namespace cb_planning
{
namespace
{
Vec2V obstacleCornersInLane(
  const Object_Info & obstacle, double origin_x, double origin_y,
  double lane_yaw, double local_start_x, double clearance)
{
  const double c_lane = std::cos(lane_yaw);
  const double s_lane = std::sin(lane_yaw);
  const double dx = obstacle.obs_global_pos_x - origin_x;
  const double dy = obstacle.obs_global_pos_y - origin_y;
  const double center_x = dx * c_lane + dy * s_lane - local_start_x;
  const double center_y = dy * c_lane - dx * s_lane;
  const double relative_yaw = obstacle.obs_global_heading_angle - lane_yaw;
  const double c_obs = std::cos(relative_yaw);
  const double s_obs = std::sin(relative_yaw);
  const double half_length = std::max(obstacle.obs_length, 0.1) / 2.0 + clearance;
  const double half_width = std::max(obstacle.obs_width, 0.1) / 2.0 + clearance;

  Vec2V corners;
  corners.reserve(4);
  const auto add_corner = [&](double longitudinal, double lateral) {
    corners.emplace_back(
      center_x + longitudinal * c_obs - lateral * s_obs,
      center_y + longitudinal * s_obs + lateral * c_obs);
  };
  add_corner(-half_length, -half_width);
  add_corner(half_length, -half_width);
  add_corner(half_length, half_width);
  add_corner(-half_length, half_width);
  return corners;
}
}  // namespace

CB_Path_Planning::CB_Path_Planning()
{
}

std::optional<CrabTrajectorySample> CB_Path_Planning::referencePointAt(double lane_x) const
{
  if (reference_lookup_path_.size() < 2 ||
      lane_x < reference_lookup_path_.front().x - 1e-9 ||
      lane_x > reference_lookup_path_.back().x + 1e-9) {
    return std::nullopt;
  }

  const auto next = std::lower_bound(
    reference_lookup_path_.begin() + 1, reference_lookup_path_.end(), lane_x,
    [](const Point2D & point, double x) { return point.x < x; });
  const auto right = next == reference_lookup_path_.end() ? next - 1 : next;
  const auto left = right - 1;
  const double dx = right->x - left->x;
  if (dx <= 1e-9) {
    return std::nullopt;
  }
  const double slope = (right->y - left->y) / dx;
  const double t = std::clamp((lane_x - left->x) / dx, 0.0, 1.0);
  return CrabTrajectorySample{lane_x, left->y + t * (right->y - left->y), slope};
}

std::optional<std::vector<double>> CB_Path_Planning::buildReturnTrajectory(
  const std::vector<double> & avoidance_traj)
{
  if (avoidance_traj.size() < 11 || reference_lookup_path_.size() < 2) {
    return std::nullopt;
  }
  const double start_x = avoidance_traj[1] + avoidance_traj[9] + avoidance_traj[8];
  const double start_y = avoidance_traj[10];
  const double reference_end_x = reference_lookup_path_.back().x;
  const auto reference_at_start = referencePointAt(start_x);
  const auto reference_at_end = referencePointAt(reference_end_x);
  if (!reference_at_start || !reference_at_end ||
      !std::isfinite(start_y) || reference_end_x < start_x - 1e-9) {
    return std::nullopt;
  }

  // 斜行生成器以零横向斜率结束；只在参考线趋于直行的位置并入，避免切线跳变。
  const double slope_tolerance = std::max(0.0, planning_para.reference_merge_slope_tolerance);
  const auto generate_to = [&](double target_x) -> std::optional<std::vector<double>> {
    const auto target = referencePointAt(target_x);
    if (!target) {
      return std::nullopt;
    }
    double joining_slope = target->dy;
    if (target_x < reference_end_x - 1e-9) {
      const auto after = referencePointAt(
        std::nextafter(target_x, std::numeric_limits<double>::infinity()));
      if (!after) return std::nullopt;
      joining_slope = after->dy;
    }
    if (std::abs(joining_slope) > slope_tolerance) return std::nullopt;
    const double dx = target_x - start_x;
    if (dx <= 1e-9) {
      if (std::abs(start_y - target->y) <= 1e-6) {
        return std::vector<double>{};
      }
      return std::nullopt;
    }
    const auto crab = crabwalking_traj_generate(
      start_y, target->y, 0.0, dx,
      planning_para.dymax, planning_para.ddymax, planning_para.ddymin);
    if (crab.state == 1 || crab.state == 5 ||
        !std::isfinite(crab.ddy) || !std::isfinite(crab.dx1)) {
      return std::nullopt;
    }
    return std::vector<double>{
      static_cast<double>(crab.state), start_x, start_y, 0.0, crab.ddy,
      crab.dx1, crab.dx2, crab.dx3, 0.0, dx, target->y};
  };

  // 候选筛选的硬条件：即使放宽到整个参考线末端，也必须能接回去。
  const auto full_return = generate_to(reference_end_x);
  if (!full_return) {
    return std::nullopt;
  }
  const double ratio = std::max(0.0, planning_para.return_length_lateral_ratio);
  const double preferred_x = std::min(
    start_x + ratio * std::abs(start_y - reference_at_start->y), reference_end_x);
  const auto preferred_return = generate_to(preferred_x);
  if (preferred_return) return preferred_return;
  // 5 倍距离处可能仍在参考线变道段，或长度不足；向前选第一个可行的接入点。
  auto next = std::upper_bound(
    reference_lookup_path_.begin(), reference_lookup_path_.end(), preferred_x,
    [](double x, const Point2D & point) { return x < point.x; });
  for (; next != reference_lookup_path_.end(); ++next) {
    const auto return_traj = generate_to(next->x);
    if (return_traj) return return_traj;
  }
  return full_return;
}

Vec2V CB_Path_Planning::calcDynamicObstacleEdgePoints(
  const Object_Info & obstacle, double local_start_x) const
{
  const double lane_yaw = planning_para.lane_yaw_bias;
  const auto corners = obstacleCornersInLane(
    obstacle, planning_para.lane_x_bias, planning_para.lane_y_bias,
    lane_yaw, local_start_x, planning_para.obstacle_clearance);

  // PredictedObject 的线速度按障碍物自身坐标系处理，保留前进/倒车和横移的符号。
  const double relative_yaw = obstacle.obs_global_heading_angle - lane_yaw;
  const double obstacle_vx =
    obstacle.obs_body_vx * std::cos(relative_yaw) -
    obstacle.obs_body_vy * std::sin(relative_yaw);
  const double obstacle_vy =
    obstacle.obs_body_vx * std::sin(relative_yaw) +
    obstacle.obs_body_vy * std::cos(relative_yaw);
  using Classification = autoware_perception_msgs::msg::ObjectClassification;
  const bool vulnerable =
    obstacle.classification == Classification::PEDESTRIAN ||
    obstacle.classification == Classification::BICYCLE ||
    obstacle.classification == Classification::MOTORCYCLE;
  const bool large_vehicle =
    obstacle.classification == Classification::TRUCK ||
    obstacle.classification == Classification::BUS ||
    obstacle.classification == Classification::TRAILER;
  const bool crossing = std::abs(obstacle_vy) > 0.5 * std::abs(obstacle_vx);
  const bool oncoming = obstacle_vx < -0.1;

  const double ego_yaw = tf2::getYaw(ego_state_ptr_->pose.pose.orientation) - lane_yaw;
  const double ego_vx =
    ego_info.ego_pos_vx * std::cos(ego_yaw) -
    ego_info.ego_pos_vy * std::sin(ego_yaw);
  const double ego_dx = ego_info.ego_pos_x - planning_para.lane_x_bias;
  const double ego_dy = ego_info.ego_pos_y - planning_para.lane_y_bias;
  const double ego_x = ego_dx * std::cos(lane_yaw) +
    ego_dy * std::sin(lane_yaw) - local_start_x;
  const double ego_half_x = 0.5 * (
    std::abs(std::cos(ego_yaw)) * planning_para.ego_length +
    std::abs(std::sin(ego_yaw)) * planning_para.ego_width);

  double obstacle_min_x = std::numeric_limits<double>::max();
  double obstacle_max_x = std::numeric_limits<double>::lowest();
  double obstacle_min_y = std::numeric_limits<double>::max();
  double obstacle_max_y = std::numeric_limits<double>::lowest();
  for (const auto & corner : corners) {
    obstacle_min_x = std::min(obstacle_min_x, corner.x);
    obstacle_max_x = std::max(obstacle_max_x, corner.x);
    obstacle_min_y = std::min(obstacle_min_y, corner.y);
    obstacle_max_y = std::max(obstacle_max_y, corner.y);
  }

  // 自车后端超过障碍物前端表示完成交互；尺寸已经包含在两个端点中。
  constexpr double max_prediction_time = 6.0;
  double prediction_time = std::clamp(
    planning_para.local_map_forward_x / std::max(std::abs(ego_vx), 1.0),
    1.0, max_prediction_time);
  const double closing_speed = ego_vx - obstacle_vx;
  if (closing_speed > 0.1 && obstacle_max_x > ego_x - ego_half_x) {
    const double clear_time =
      (obstacle_max_x - (ego_x - ego_half_x)) / closing_speed;
    prediction_time = std::min(
      max_prediction_time, std::max(prediction_time, clear_time + 0.5));
  }
  if (crossing || vulnerable) prediction_time = max_prediction_time;

  // 首尾矩形的包围框覆盖匀速平移期间的全部位置；角速度引起的位移另行保守放大。
  const double yaw_change = std::abs(obstacle.obs_yaw_rate) * prediction_time;
  const double corner_radius = 0.5 * std::hypot(obstacle.obs_length, obstacle.obs_width);
  const double turn_margin =
    corner_radius * std::min(yaw_change, 2.0) +
    0.5 * obstacle.obs_velocity * std::abs(obstacle.obs_yaw_rate) *
    prediction_time * prediction_time;
  // 其余余量覆盖定位、速度及未建模加速度误差；需要按感知误差上界标定。
  const double longitudinal_margin =
    (large_vehicle ? 1.0 : vulnerable ? 0.7 : 0.5) +
    (oncoming ? 0.4 : 0.2) * prediction_time +
    0.1 * prediction_time * prediction_time + turn_margin;
  const double lateral_margin =
    (vulnerable || crossing ? 0.7 : 0.3) +
    (vulnerable || crossing ? 0.2 : 0.1) * prediction_time +
    0.05 * prediction_time * prediction_time + turn_margin;
  const double min_x = obstacle_min_x + std::min(0.0, obstacle_vx * prediction_time) - longitudinal_margin;
  const double max_x = obstacle_max_x + std::max(0.0, obstacle_vx * prediction_time) + longitudinal_margin;
  const double min_y = obstacle_min_y + std::min(0.0, obstacle_vy * prediction_time) - lateral_margin;
  const double max_y = obstacle_max_y + std::max(0.0, obstacle_vy * prediction_time) + lateral_margin;

  return {{min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}};
}

bool CB_Path_Planning::run()
{
  sampled_trajectory_.clear();
  output_path = Path{};
  // 更新车辆当前位置
  if(!ego_state_ptr_)
  {
      std::cerr << "Ego state is not set. Please set the ego state before running the planner." << std::endl;
      return false;
  }
  ego_info.ego_pos_x = ego_state_ptr_->pose.pose.position.x;
  ego_info.ego_pos_y = ego_state_ptr_->pose.pose.position.y;
  ego_info.ego_pos_vx = ego_state_ptr_->twist.twist.linear.x;
  ego_info.ego_pos_vy = ego_state_ptr_->twist.twist.linear.y;

  // 更新规划起点状态，及动力学约束参数
  double vx_max;
  if(!prev_trajectory_msg_ || prev_trajectory_msg_->points.empty())
  {
      start_point_x = ego_info.ego_pos_x;
      start_point_y = ego_info.ego_pos_y;
      start_point_dy = ego_info.ego_pos_vy/std::max(ego_info.ego_pos_vx, 0.1);
      vx_plan = ego_info.ego_pos_vx;
      vx_max = std::max(vx_plan, 0.1);
  }
  else
  {
      updateStartPoint(prev_trajectory_msg_, ego_info.ego_pos_x, ego_info.ego_pos_y, ego_info.ego_pos_vx, ego_info.ego_pos_vy,
        start_point_x, start_point_y, start_point_dy, vx_plan);
      
      auto it = std::max_element(prev_trajectory_msg_->points.begin(), prev_trajectory_msg_->points.end(), 
        [](const TrajectoryPoint& a, const TrajectoryPoint& b) { return a.longitudinal_velocity_mps < b.longitudinal_velocity_mps; });
      vx_max = it->longitudinal_velocity_mps;
      vx_max = std::max(vx_max, 0.1);
  }

  planning_para.ddymax = planning_para.steer_angular_vel_max/180.0 * M_PI/vx_max; // 转向角速度对应的横向加速度
  planning_para.ddymin = 0.0;
  planning_para.dymax = planning_para.steer_angle_max/180.0 * M_PI; // 最大转向角对应的横向速度

  /*********横向规划*********/ 
  // 将规划起点的全局坐标投影在车道坐标系下，车道坐标系原点为车道参考点，沿车道方向为x轴正方向，垂直车道方向为y轴正方向
  double x_origin = planning_para.lane_x_bias;
  double y_origin = planning_para.lane_y_bias;
  double yaw_lane = planning_para.lane_yaw_bias;
  double local_start_x = (start_point_x - x_origin) * std::cos(yaw_lane) + (start_point_y - y_origin) * std::sin(yaw_lane);
  double local_start_y = (start_point_y - y_origin) * std::cos(yaw_lane) - (start_point_x - x_origin) * std::sin(yaw_lane);
  double local_start_dy = start_point_dy;

  // 填充参考路径，将其转换到车道坐标系下
  if (!reference_path_ptr_ || reference_path_ptr_->points.size() < 2)
  {
      std::cerr << "Reference path is empty." << std::endl;
      return false;
  }
  global_reference_path.clear();
  reference_lookup_path_.clear();
  for(size_t i=0; i<reference_path_ptr_->points.size(); i++)
  {
      Point2D ref_point;
      ref_point.x = (reference_path_ptr_->points.at(i).pose.position.x - x_origin) * std::cos(yaw_lane) + (reference_path_ptr_->points.at(i).pose.position.y - y_origin) * std::sin(yaw_lane);
      ref_point.y = (reference_path_ptr_->points.at(i).pose.position.y - y_origin) * std::cos(yaw_lane) - (reference_path_ptr_->points.at(i).pose.position.x - x_origin) * std::sin(yaw_lane);
      if (!std::isfinite(ref_point.x) || !std::isfinite(ref_point.y)) {
          std::cerr << "Reference path contains non-finite coordinates." << std::endl;
          return false;
      }
      global_reference_path.push_back(ref_point);
      if (!reference_lookup_path_.empty()) {
          const auto & prev = reference_lookup_path_.back();
          if (std::abs(ref_point.x - prev.x) <= 1e-9 &&
              std::abs(ref_point.y - prev.y) <= 1e-6) {
              continue;  // 上层拼接直行/变道分段时可能重复边界点。
          }
          if (ref_point.x <= prev.x + 1e-9) {
              std::cerr << "Reference path x must increase in lane coordinates." << std::endl;
              return false;
          }
      }
      reference_lookup_path_.push_back(ref_point);
  }
  if (reference_lookup_path_.size() < 2) {
      std::cerr << "Reference path has fewer than two distinct points." << std::endl;
      return false;
  }

  // 参考线点的间距和起始 x 不固定，逐点搜索车道坐标系下最近的 x。
  const auto nearest_reference_index = [this](double x) -> size_t {
      const auto it = std::min_element(
          global_reference_path.begin(), global_reference_path.end(),
          [x](const Point2D & lhs, const Point2D & rhs) {
              return std::abs(lhs.x - x) < std::abs(rhs.x - x);
          });
      return static_cast<size_t>(std::distance(global_reference_path.begin(), it));
  };

  // 初始化局部地图网格
  double lane_end_x = planning_para.lane_end_x;
  double x_grid_end = std::min(planning_para.local_map_forward_x, static_cast<double>(lane_end_x - local_start_x));
  std::vector<double> x_global_set;
  for(double x = planning_para.lane_x_sampling_interval; x <= x_grid_end; x += planning_para.lane_x_sampling_interval)
      x_global_set.push_back(x);
  
  std::vector<double> x_local_map_set;
  double x_local_set_bias = local_start_x - std::floor(local_start_x);
  for(size_t i=0; i<x_global_set.size(); i++)
      x_local_map_set.push_back(x_global_set.at(i) - x_local_set_bias);

  // 计算局部地图内障碍物位置, 将障碍物的边缘点转换到局部坐标系下
  Vec2V edge_local_points;
  for(size_t i=0; i<objects_info.size(); i++)
  {
      const auto & obstacle = objects_info.at(i);
      const double obs_velocity = obstacle.obs_velocity;

      // 动态功能关闭时所有障碍物均按当前占据区域处理；开启后按类型和运动方向预测扫掠区域。
      if(!planning_para.enable_dynamic_obstacle_avoidance || obs_velocity < 0.1)
      {
        const auto corners = obstacleCornersInLane(
          obstacle, x_origin, y_origin, yaw_lane, local_start_x,
          planning_para.obstacle_clearance);
        edge_local_points.insert(edge_local_points.end(), corners.begin(), corners.end());
      } else
      {
        // 动态障碍物的处理逻辑，根据自车与障碍物的交互点位置，计算障碍物在未来一段时间内的边缘点
        const auto swept_box = calcDynamicObstacleEdgePoints(obstacle, local_start_x);
        edge_local_points.insert(edge_local_points.end(), swept_box.begin(), swept_box.end());
      }

  }

  // 计算车道中的可行驶区域
  std::vector <double> Yc_set;
  std::vector <int> NoObs_set;
  std::vector<double> Ll_set;
  std::vector<double> Lr_set;
  Yc_set.reserve(x_local_map_set.size());
  Ll_set.reserve(x_local_map_set.size());
  Lr_set.reserve(x_local_map_set.size());
  NoObs_set.reserve(x_local_map_set.size());

  for(size_t i=0; i<x_local_map_set.size(); i++)
  {
      DrivableRange drivable_range;
      double ref_path_x = local_start_x + x_local_map_set[i];
      size_t ind = nearest_reference_index(ref_path_x);
      double ref_path_y = global_reference_path.at(ind).y;
      drivable_range = calcDrivableRange(edge_local_points, x_local_map_set[i], ref_path_y);
      Yc_set.push_back(drivable_range.yc);
      Ll_set.push_back(drivable_range.Ll);
      Lr_set.push_back(drivable_range.Lr);
      NoObs_set.push_back(drivable_range.NoObs);
  }

  LocalPlanResult res_path;
  // 在正常状态下，根据当前位置进行局部规划；如果无法正常规划出轨迹，执行重规划，重规划成功进入重规划状态
  if(!is_replanning)
  {
    // 进行横向规划
    double L_min = planning_para.Min_Collision_free_distance;
    res_path = local_plan(x_local_map_set, local_start_x, local_start_y, local_start_dy, edge_local_points,
        L_min, Yc_set, NoObs_set, planning_para.dymax, planning_para.ddymax, planning_para.ddymin);

    // 调试信息：将res_path.opt_traj 的11个参数打印出来
    // std::cout << "Optimal trajectory parameters: ";
    // for (const auto& param : res_path.opt_traj) {
    //     std::cout << param << " "; 
    // }  
    // std::cout << std::endl;

    bool is_success = res_path.isSuccess;
    if(!is_success)
    {
        /*如果局部规划失败，则进行再规划*/
        if(opt_traj_prev.size() < 11 || std::isnan(opt_traj_prev.at(0)))
        {
            // 初始化后第一次就无法输出可行轨迹，输出停止轨迹，返回失败0
            std::cerr << "The first planning attempt failed, so the system exited directly" << std::endl;
            return false;
        }else
        {
            // 初始化上一步的局部地图网格
            double opt_x_prev = opt_traj_prev.at(1);
            double x_grid_end_prev = std::min({
                planning_para.local_map_forward_x, lane_end_x - opt_x_prev,
                global_reference_path.back().x - opt_x_prev});
            std::vector<double> x_global_set_prev;
            for(double x = planning_para.lane_x_sampling_interval; x <= x_grid_end_prev; x += planning_para.lane_x_sampling_interval)
                x_global_set_prev.push_back(x);
            
            double x_local_set_bias_prev = opt_x_prev - std::floor(opt_x_prev);
            std::vector<double> x_local_set_prev;
            for(size_t i=0; i<x_global_set_prev.size(); i++)
                x_local_set_prev.push_back(x_global_set_prev.at(i) - x_local_set_bias_prev);
            
            Vec2V edge_local_points_prev = edge_local_points;
            for (auto & point : edge_local_points_prev) {
                point.x += local_start_x - opt_x_prev;
            }
            std::vector<double> Yc_set_replan;
            std::vector<int> NoObs_set_replan;
            Yc_set_replan.reserve(x_local_set_prev.size());
            NoObs_set_replan.reserve(x_local_set_prev.size());
            for (const double x : x_local_set_prev) {
                const double ref_x = std::clamp(
                    opt_x_prev + x, global_reference_path.front().x,
                    global_reference_path.back().x);
                const auto ref = referencePointAt(ref_x);
                const auto range = calcDrivableRange(edge_local_points_prev, x, ref->y);
                Yc_set_replan.push_back(range.yc);
                NoObs_set_replan.push_back(range.NoObs);
            }

            size_t replan_num = static_cast<int>(L_min)/2 - 1;
            std::vector <bool> isSuccess_replan(replan_num, false);
            for(size_t i=0; i<replan_num; i++)
            {
                // 由于opt_traj_prev的长度是超过20的，进行9次遍历搜索不会超过轨迹总长度
                double start_pos_x_replan = 2 * i;
                double start_pos_y_replan;
                double start_pos_dy_replan;
                calcPosy_Vy(opt_traj_prev, start_pos_x_replan, start_pos_y_replan, start_pos_dy_replan);

                LocalPlanResult res_path_replan;
                res_path_replan = local_replan(x_local_set_prev, start_pos_x_replan,
                    opt_x_prev + start_pos_x_replan, start_pos_y_replan, start_pos_dy_replan,
                    edge_local_points_prev, L_min, Yc_set_replan, NoObs_set_replan,
                    planning_para.dymax, planning_para.ddymax, planning_para.ddymin);
                
                // 将res_path.opt_traj 的11个参数打印出来
                std::cout << "Optimal trajectory parameters: ";
                for (const auto& param : res_path_replan.opt_traj) {
                    std::cout << param << " "; 
                }  
                std::cout << std::endl;
                
                // 如果重规划成功，记录两段轨迹，并退出循环
                if(res_path_replan.isSuccess)
                {
                    isSuccess_replan.at(i) = true;
                    opt_traj_inreplanning.resize(res_path_replan.opt_traj.size());
                    for(size_t j=0; j<res_path_replan.opt_traj.size(); j++)
                    {
                        opt_traj_inreplanning.at(j)[0] = opt_traj_prev.at(j);
                        opt_traj_inreplanning.at(j)[1] = res_path_replan.opt_traj.at(j);
                    }
                    opt_return_inreplanning = res_path_replan.return_traj;
                    is_replanning = true;
                    std::cout << "x of Initial planning: " << opt_x_prev << ", x of replanning: " << start_pos_x_replan 
                        << "the number of collision free trajectories of replanning: " << res_path_replan.collision_free_num << std::endl;
                    break;
                }
            }

            // 如果9次重规划都失败，则输出停止轨迹，直接退出
            if(!std::any_of(isSuccess_replan.begin(), isSuccess_replan.end(), [](int v){ return v==1;}))
            {
                std::cout << "Replanning failed! " << std::endl;
                is_first_plan = true;
                return false;
            }
        }
    }else
    {
        // 局部规划成功，保存当前时刻的参量
        opt_traj_prev = res_path.opt_traj;
        Yc_set_prev = Yc_set;
        NoObs_set_prev = NoObs_set;
        Ll_set_prev = Ll_set;
        Lr_set_prev = Lr_set;
        // RCLCPP_INFO(this->get_logger(), "the number of collision free trajectories of planning: %d", res_path.collision_free_num);
    }
  }

  // 在参考线原有的每个 x 上采样，保持输出点数、顺序与输入参考线完全一致。
  std::vector<double> first_traj;
  std::vector<double> replan_traj;
  const auto & return_traj = is_replanning ? opt_return_inreplanning : res_path.return_traj;
  double sample_start_x = std::max(local_start_x, reference_lookup_path_.front().x);
  double replan_start_x = std::numeric_limits<double>::infinity();
  if (is_replanning) {
      if (opt_traj_inreplanning.size() < 11) {
          return false;
      }
      first_traj.reserve(opt_traj_inreplanning.size());
      replan_traj.reserve(opt_traj_inreplanning.size());
      for (const auto & params : opt_traj_inreplanning) {
          first_traj.push_back(params[0]);
          replan_traj.push_back(params[1]);
      }
      sample_start_x = std::max(sample_start_x, first_traj[1]);
      replan_start_x = replan_traj[1];
  } else {
      if (res_path.opt_traj.size() < 11) {
          return false;
      }
      first_traj = res_path.opt_traj;
  }
  if (is_replanning &&
      replan_start_x > first_traj[1] + first_traj[9] + first_traj[8] + 1e-9) {
      return false;
  }

  if (!std::isfinite(sample_start_x) ||
      sample_start_x > reference_lookup_path_.back().x) {
      return false;
  }

  // 重规划可能保留上一帧的前半段；障碍物状态变化后必须重新检查这段路径。
  // 这里使用内部密采样，输出点仍严格与参考线一一对应。
  const auto curve_is_clear = [&](const std::vector<double> & traj,
                                  double begin_x, double end_x) {
      if (end_x < begin_x) return true;
      const auto collision_at = [&](double lane_x) {
          double y = 0.0;
          double dy = 0.0;
          calcPosy_Vy(traj, lane_x - traj[1], y, dy);
          const double local_x = lane_x - local_start_x;
          for (size_t j = 0; j + 3 < edge_local_points.size(); j += 4) {
              double min_x = edge_local_points[j].x;
              double max_x = min_x;
              double min_y = edge_local_points[j].y;
              double max_y = min_y;
              for (size_t k = j + 1; k < j + 4; ++k) {
                  min_x = std::min(min_x, edge_local_points[k].x);
                  max_x = std::max(max_x, edge_local_points[k].x);
                  min_y = std::min(min_y, edge_local_points[k].y);
                  max_y = std::max(max_y, edge_local_points[k].y);
              }
              if (local_x + planning_para.ego_length / 2.0 >= min_x &&
                  local_x - planning_para.ego_length / 2.0 <= max_x &&
                  y + planning_para.ego_width / 2.0 >= min_y &&
                  y - planning_para.ego_width / 2.0 <= max_y) {
                  return true;
              }
          }
          return false;
      };
      constexpr double collision_sample_step = 0.25;
      for (double x = begin_x; x < end_x; x += collision_sample_step) {
          if (collision_at(x)) return false;
      }
      return !collision_at(end_x);
  };
  const double first_end_x = std::min(
      first_traj[1] + first_traj[9] + first_traj[8],
      is_replanning ? replan_start_x : std::numeric_limits<double>::infinity());
  if (!curve_is_clear(first_traj, sample_start_x, first_end_x) ||
      (is_replanning && !curve_is_clear(
        replan_traj, replan_start_x,
        replan_traj[1] + replan_traj[9] + replan_traj[8]))) {
      return false;
  }

  output_path = *reference_path_ptr_;
  sampled_trajectory_.reserve(global_reference_path.size());
  constexpr double endpoint_epsilon = 1e-9;
  const double c = std::cos(yaw_lane);
  const double s = std::sin(yaw_lane);
  for (size_t i = 0; i < global_reference_path.size(); ++i) {
      const double lane_x = global_reference_path[i].x;
      if (lane_x < sample_start_x - endpoint_epsilon) {
          const auto ref = referencePointAt(lane_x);
          sampled_trajectory_.push_back(*ref);
          continue;
      }
      const bool use_replan = is_replanning && lane_x >= replan_start_x;
      const auto & traj = use_replan ? replan_traj : first_traj;
      const double avoidance_end_x = traj[1] + traj[9] + traj[8];
      double y = 0.0;
      double dy = 0.0;
      bool on_generated_curve = false;
      if (lane_x < avoidance_end_x - endpoint_epsilon) {
          calcPosy_Vy(traj, lane_x - traj[1], y, dy);
          on_generated_curve = true;
      } else if ((!is_replanning || use_replan) && !return_traj.empty() &&
                 lane_x >= return_traj[1] - endpoint_epsilon &&
                 lane_x < return_traj[1] + return_traj[9] - endpoint_epsilon) {
          calcPosy_Vy(return_traj, lane_x - return_traj[1], y, dy);
          on_generated_curve = true;
      } else {
          const auto ref = referencePointAt(lane_x);
          y = ref->y;
          dy = ref->dy;
      }
      sampled_trajectory_.push_back({lane_x, y, dy});
      if (on_generated_curve) {
          auto & point = output_path.points[i];
          point.pose.position.x = x_origin + lane_x * c - y * s;
          point.pose.position.y = y_origin + lane_x * s + y * c;
          point.pose.orientation.x = 0.0;
          point.pose.orientation.y = 0.0;
          point.pose.orientation.z = std::sin(yaw_lane / 2.0);
          point.pose.orientation.w = std::cos(yaw_lane / 2.0);
          point.lateral_velocity_mps =
              static_cast<float>(dy * point.longitudinal_velocity_mps);
          point.heading_rate_rps = 0.0f;
      }
  }

  if (is_replanning) {
      const double ego_lane_x =
          (ego_info.ego_pos_x - x_origin) * std::cos(yaw_lane) +
          (ego_info.ego_pos_y - y_origin) * std::sin(yaw_lane);
      if (ego_lane_x >= replan_start_x) {
          is_replanning = false;
      }
  }

  return true;
}

LocalPlanResult CB_Path_Planning::local_plan(std::vector<double> X_set, double start_point_x, double start_point_y, double start_point_dy, Vec2V edge_points,
                    double L_min, std::vector<double> Yc_set, std::vector<int> NoObs_set, double dymax, double ddymax, double ddymin)
{
    std::vector<double> X_local_set = X_set;
    std::vector<double> Y_set;
    for(double y = planning_para.lane_sampling_begin_y; y<=planning_para.lane_sampling_end_y; y += planning_para.lane_y_sampling_interval)
        Y_set.push_back(y);
    
    size_t X_len = X_local_set.size();
    size_t Y_len = Y_set.size(); 

    std::vector<std::vector<int>>    state_set(X_len, std::vector<int>(Y_len, 0));
    std::vector<std::vector<double>> ddy_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> dx1_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> dx2_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> dx3_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> dx4_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> Cost_set(X_len, std::vector<double>(Y_len, 0.0));

    using CostElem = std::array<double, 3>;  //{cost, m, n}
    std::vector<CostElem> Cost_priority(X_len*Y_len, {std::numeric_limits<double>::infinity(), 0, 0});
    int collision_free_num = 0;

    Vec2V edge_local_points = edge_points;

    /* 2. 双重循环：遍历所有 (m,n) 网格 */
    for (size_t m = 0; m < X_len; ++m){
        double dx = X_local_set[m];
        for (size_t n = 0; n < Y_len; ++n){
            double y0 = start_point_y;
            double y1 = Y_set[n];
            double dy0 = start_point_dy;

            /* 2.1 生成斜行轨迹参数 */
            auto crab = crabwalking_traj_generate(y0, y1, dy0, dx, dymax, ddymax, ddymin);
            
            int state = crab.state;
            state_set[m][n] = state;
            if (state == 1 || state == 5) continue;

            ddy_set[m][n] = crab.ddy;
            dx1_set[m][n] = crab.dx1;
            dx2_set[m][n] = crab.dx2;
            dx3_set[m][n] = crab.dx3;

            /* 2.2 计算 dx4 */
            double dx4 = (dx < L_min) ? (L_min - dx) : 0.0;
            dx4_set[m][n] = dx4;

            /* 2.3 碰撞检测 */
            auto col = CollisionCheck(state, y0, y1, dy0, crab.ddy, dx, crab.dx1, crab.dx2, crab.dx3, dx4, edge_local_points);
            if (col.isCollision) continue;

            // 仅评价能在 Q-turn 前接回整段参考线的无碰撞候选。
            const std::vector<double> candidate{
                static_cast<double>(state), start_point_x, y0, dy0, crab.ddy,
                crab.dx1, crab.dx2, crab.dx3, dx4, dx, y1};
            if (!buildReturnTrajectory(candidate)) continue;

            /* 2.4 计算代价 */
            auto cost_res = calcCost(state, y0, y1, dy0, crab.ddy, dx, crab.dx1, crab.dx3, dx4,
                                      X_local_set, Yc_set, NoObs_set);
            double cost_val = cost_res.cost;
            Cost_set[m][n] = cost_val;

            /* 2.5 插入排序（升序） */
            CostElem elem{cost_val, static_cast<double>(m), static_cast<double>(n)};
            auto it = std::lower_bound(Cost_priority.begin(), Cost_priority.end(), elem,
                                        [](const CostElem& a, const CostElem& b){
                                            return a[0] < b[0];
                                        });
            if (it != Cost_priority.end()) *it = elem;

            collision_free_num++;
        }
    }

    /* 3. 输出结果 */
    LocalPlanResult res;
    res.collision_free_num = collision_free_num;
    if (collision_free_num == 0){
        res.isSuccess = 0;
        res.opt_traj.resize(11);
        std::fill(res.opt_traj.begin(), res.opt_traj.end(),
                  std::numeric_limits<double>::quiet_NaN());
    }
    else{
        res.isSuccess = 1;
        size_t opt_m = static_cast<size_t>(Cost_priority[0][1]);
        size_t opt_n = static_cast<size_t>(Cost_priority[0][2]);
        int    opt_state = state_set[opt_m][opt_n];
        double opt_ddy   = ddy_set[opt_m][opt_n];
        double opt_dx1   = dx1_set[opt_m][opt_n];
        double opt_dx2   = dx2_set[opt_m][opt_n];
        double opt_dx3   = dx3_set[opt_m][opt_n];
        double opt_dx4   = dx4_set[opt_m][opt_n];
        double opt_dx    = X_local_set[opt_m];
        double opt_y1    = Y_set[opt_n];

        res.opt_traj = { static_cast<double>(opt_state),
                          start_point_x,
                          start_point_y,
                          start_point_dy,
                          opt_ddy,
                          opt_dx1,
                          opt_dx2,
                          opt_dx3,
                          opt_dx4,
                          opt_dx,
                          opt_y1 };
        res.return_traj = *buildReturnTrajectory(res.opt_traj);
    }
    return res;
}

LocalPlanResult CB_Path_Planning::local_replan(std::vector<double> X_set, double start_point_x, double start_lane_x, double start_point_y, double start_point_dy, Vec2V edge_points, 
    double L_min, std::vector<double> Yc_set, std::vector<int> NoObs_set, double dymax, double ddymax, double ddymin)
{
    /* 1. 构造以重规划起点为 x=0 的局部地图；按 x 搜索，不假设网格间距为 1 m。 */
    std::vector<double> X_local_set, Yc_local_set;
    std::vector<int> NoObs_local_set;
    const size_t grid_size = std::min({X_set.size(), Yc_set.size(), NoObs_set.size()});
    for (size_t k = 0; k < grid_size; ++k) {
        if (X_set[k] <= start_point_x || X_set[k] > start_point_x + 30.0) continue;
        X_local_set.push_back(X_set[k] - start_point_x);
        Yc_local_set.push_back(Yc_set[k]);
        NoObs_local_set.push_back(NoObs_set[k]);
    }

      /* 2. 构造 Y 搜索空间 1.5:0.25:6.5 */
    std::vector<double> Y_set;
    for(double y = planning_para.lane_sampling_begin_y; y<=planning_para.lane_sampling_end_y; y += planning_para.lane_y_sampling_interval)
        Y_set.push_back(y);
    
    size_t X_len = X_local_set.size();
    size_t Y_len = Y_set.size(); 

    std::vector<std::vector<int>>    state_set(X_len, std::vector<int>(Y_len, 0));
    std::vector<std::vector<double>> ddy_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> dx1_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> dx2_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> dx3_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> dx4_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> Cost_set(X_len, std::vector<double>(Y_len, 0.0));

    using CostElem = std::array<double, 3>;  //{cost, m, n}
    std::vector<CostElem> Cost_priority(X_len*Y_len, {std::numeric_limits<double>::infinity(), 0, 0});
    int collision_free_num = 0;

    /* 4. 障碍物坐标平移 */
    Vec2V edge_local_points = edge_points;
    for (size_t i = 0; i < edge_points.size(); ++i) {
        edge_local_points.at(i).x = edge_points.at(i).x - start_point_x;
    }

    /* 5. 遍历网格 */
    /* 2. 双重循环：遍历所有 (m,n) 网格 */
    for (size_t m = 0; m < X_len; ++m){
        double dx = X_local_set[m];
        for (size_t n = 0; n < Y_len; ++n){
            double y0 = start_point_y;
            double y1 = Y_set[n];
            double dy0 = start_point_dy;

            /* 2.1 生成斜行轨迹参数 */
            auto crab = crabwalking_traj_generate(y0, y1, dy0, dx, dymax, ddymax, ddymin);
            
            int state = crab.state;
            state_set[m][n] = state;
            if (state == 1 || state == 5) continue;

            ddy_set[m][n] = crab.ddy;
            dx1_set[m][n] = crab.dx1;
            dx2_set[m][n] = crab.dx2;
            dx3_set[m][n] = crab.dx3;

            /* 2.2 计算 dx4 */
            double dx4 = (dx < (L_min - start_point_x +1.0)) ? (L_min - start_point_x +1.0 - dx) : 0.0;
            dx4_set[m][n] = dx4;

            /* 2.3 碰撞检测 */
            auto col = CollisionCheck(state, y0, y1, dy0, crab.ddy, dx, crab.dx1, crab.dx2, crab.dx3, dx4, edge_local_points);
            if (col.isCollision) continue;

            const std::vector<double> candidate{
                static_cast<double>(state), start_lane_x, y0, dy0, crab.ddy,
                crab.dx1, crab.dx2, crab.dx3, dx4, dx, y1};
            if (!buildReturnTrajectory(candidate)) continue;

            /* 2.4 计算代价 */
            auto cost_res = calcCost(state, y0, y1, dy0, crab.ddy, dx, crab.dx1, crab.dx3, dx4,
                                      X_local_set, Yc_local_set, NoObs_local_set);
            double cost_val = cost_res.cost;
            Cost_set[m][n] = cost_val;

            /* 2.5 插入排序（升序） */
            CostElem elem{cost_val, static_cast<double>(m), static_cast<double>(n)};
            auto it = std::lower_bound(Cost_priority.begin(), Cost_priority.end(), elem,
                                        [](const CostElem& a, const CostElem& b){
                                            return a[0] < b[0];
                                        });
            if (it != Cost_priority.end()) *it = elem;

            collision_free_num++;
        }
    }

    /* 3. 输出结果 */
    LocalPlanResult res;
    res.collision_free_num = collision_free_num;
    if (collision_free_num == 0){
        res.isSuccess = 0;
        res.opt_traj.resize(11);
        std::fill(res.opt_traj.begin(), res.opt_traj.end(),
                  std::numeric_limits<double>::quiet_NaN());
    }
    else{
        res.isSuccess = 1;
        size_t opt_m = static_cast<size_t>(Cost_priority[0][1]);
        size_t opt_n = static_cast<size_t>(Cost_priority[0][2]);
        int    opt_state = state_set[opt_m][opt_n];
        double opt_ddy   = ddy_set[opt_m][opt_n];
        double opt_dx1   = dx1_set[opt_m][opt_n];
        double opt_dx2   = dx2_set[opt_m][opt_n];
        double opt_dx3   = dx3_set[opt_m][opt_n];
        double opt_dx4   = dx4_set[opt_m][opt_n];
        double opt_dx    = X_local_set[opt_m];
        double opt_y1    = Y_set[opt_n];

        res.opt_traj = { static_cast<double>(opt_state),
                          start_lane_x,
                          start_point_y,
                          start_point_dy,
                          opt_ddy,
                          opt_dx1,
                          opt_dx2,
                          opt_dx3,
                          opt_dx4,
                          opt_dx,
                          opt_y1 };
        res.return_traj = *buildReturnTrajectory(res.opt_traj);
    }
    return res;
}

void CB_Path_Planning::calcPosy_Vy(const std::vector<double> & traj, double ego_pos_x, double &ego_pos_y, double &ego_pos_dy)
{
    int state = static_cast<int>(traj[0]);
    double y0 = traj[2];
    double y1 = traj[10];
    double dy0 = traj[3];
    double ddy = traj[4];
    double dx1 = traj[5];
    double dx3 = traj[7];
    double dxsum = traj[9];

    double x = ego_pos_x;

    ego_pos_y = 0.0;
    ego_pos_dy = 0.0;
    if (state == 0)
    {
        ego_pos_y = y0;
        ego_pos_dy = dy0;
    }
    else if (state == 2)
    {
        if (x>=0.0 && x<dx1)
        {
            ego_pos_y = y0 + dy0 * x;
            ego_pos_dy = dy0;
        }
        else if (x >= dx1 && x <= dxsum)
        {
            ego_pos_y = y0 + dy0*x - 0.5*ddy*(x - dx1)*(x - dx1);
            ego_pos_dy = dy0 - ddy * (x - dx1);
        }
    }
    else if (state == 3)
    {
        if (x>=0.0 && x < dx1)
        {
            ego_pos_y = y0 + dy0 * x + 0.5 * ddy * x * x;
            ego_pos_dy = dy0 + ddy * x;
        }
        else if (x >= dx1 && x <= dxsum)
        {
            double y_t1 = y0 + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
            double v_t1 = dy0 + ddy * dx1;
            ego_pos_y = y_t1 + v_t1 * (x - dx1) - 0.5*ddy*(x - dx1)*(x - dx1);
            ego_pos_dy = v_t1 - ddy * (x - dx1);
        }
    }
    else if (state == 4)
    {
        if (x>=0.0 && x < dx1)
        {
            ego_pos_y = y0 + dy0 * x + 0.5 * ddy * x * x;
            ego_pos_dy = dy0 + ddy * x;
        }
        else if (x >= dx1 && x < (dx1 +dx3))
        {
            double y_t1 = y0 + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
            double v_t1 = dy0 + ddy * dx1;
            ego_pos_y = y_t1 + v_t1 * (x - dx1);
            ego_pos_dy = v_t1;
        }
        else if (x >= (dx1 + dx3) && x <= dxsum)
        {
            double y_t1 = y0 + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
            double v_t1 = dy0 + ddy * dx1;
            double y_t2 = y_t1 + v_t1 * dx3;
            double v_t2 = v_t1;
            ego_pos_y = y_t2 + v_t2*(x-(dx1+dx3)) - 0.5*ddy*(x-(dx1+dx3))*(x-(dx1+dx3));
            ego_pos_dy = v_t2 - ddy * (x-(dx1+dx3));
        }
    }

    if(x < 0) 
        ego_pos_y = y0;
    else if (x > dxsum)
        ego_pos_y = y1;

}

void CB_Path_Planning::updateStartPoint(Trajectory::SharedPtr prev_trajectory_msg, double ego_pos_x, double ego_pos_y, double ego_pos_vx, double ego_pos_vy, 
  double &start_point_x, double &start_point_y, double &start_point_dy, double &vx_plan)
{
    // 根据轨迹点的航向角将当前位置投影在上一帧轨迹上，计算起点位置，再根据前后轨迹点速度插值得到当前速度
    const auto& points = prev_trajectory_msg->points;

    // 如果车辆当前位置与轨迹点的最小投影距离大于阈值，则使用车辆当前位置作为起点
    double min_dist2 = planning_para.start_point_mindis_threshold * planning_para.start_point_mindis_threshold;

    start_point_x = ego_pos_x;
    start_point_y = ego_pos_y;
    start_point_dy = ego_pos_vy / std::max(std::fabs(ego_pos_vx), 0.1);
    vx_plan = ego_pos_vx;

    for(size_t i=0; i+1<points.size(); i++)
    {
      const auto& p0 = points[i];
      const auto& p1 = points[i + 1];

      const double x0 = p0.pose.position.x;
      const double y0 = p0.pose.position.y;
      const double x1 = p1.pose.position.x;
      const double y1 = p1.pose.position.y;
      const double dy0 = p0.lateral_velocity_mps/std::max(p0.longitudinal_velocity_mps, 0.1f);
      const double dy1 = p1.lateral_velocity_mps/std::max(p1.longitudinal_velocity_mps, 0.1f);
      const double vx0 = p0.longitudinal_velocity_mps;
      const double vx1 = p1.longitudinal_velocity_mps;

      const double dx = x1 - x0;
      const double dy = y1 - y0;
      const double len2 = dx * dx + dy * dy;

      // 防止两个轨迹点重合
      if (len2 < 1e-6) continue;

      // -------------------------------------------------
      // ego 在当前轨迹段上的投影比例
      // P = P0 + t * (P1 - P0)
      // t = 0 -> P0
      // t = 1 -> P1
      // -------------------------------------------------
      double t =((ego_info.ego_pos_x - x0) * dx + (ego_info.ego_pos_y - y0) * dy) / len2;
      t = std::clamp(t, 0.0, 1.0); // 投影限制在线段内部

      // 投影点
      const double proj_x = x0 + t * dx;
      const double proj_y = y0 + t * dy;
      const double proj_dy = dy0 + t * (dy1 - dy0);
      const double proj_vx = vx0 + t * (vx1 - vx0);

      // ego 到投影点距离
      const double diff_x = ego_info.ego_pos_x - proj_x;
      const double diff_y = ego_info.ego_pos_y - proj_y;
      const double dist2 = diff_x * diff_x + diff_y * diff_y;

      // 找最近投影点
      if (dist2 < min_dist2)
      {
          min_dist2 = dist2;
          start_point_x = proj_x;
          start_point_y = proj_y;
          start_point_dy = proj_dy;
          vx_plan = proj_vx;
      }
  }
}

CrabResult CB_Path_Planning::crabwalking_traj_generate(double y0, double y1, double dy0, double dx, double dymax, double ddymax, double ddymin)
{
    // 初始化临时变量
    double delta_y = y1 - y0;
    double ddy, dx1, dx2, dx3, dyPeak;
    int state;

    state = 0;
    ddy = dx1 = dx2 = 0.0;
    dx3 = std::numeric_limits<double>::quiet_NaN();
    
    if(delta_y == 0.0)
    {
        if(dy0 == 0.0){
            state = 0;
            ddy = 0.0;
            dx1 = dx;
            dx2 = std::numeric_limits<double>::quiet_NaN();
            return {state, ddy, dx1, dx2, dx3};
        }else
        {
            state = 3;
            ddy = -(std::sqrt(2)+1)*dy0/dx;
            dx1 = sqrt(2)/2*dx;
            dx2 = dx - dx1;
            return {state, ddy, dx1, dx2, dx3};
        }
    }
    
    if(delta_y > 0.0)
    {
        if(dy0*dx > 2*delta_y)
        {
            state = 1;
            ddy = std::numeric_limits<double>::quiet_NaN();
            dx1 = std::numeric_limits<double>::quiet_NaN();
            dx2 = std::numeric_limits<double>::quiet_NaN();
            return {state, ddy, dx1, dx2, dx3};
        }

        if(dy0*dx > delta_y && dy0*dx <= 2*delta_y)
        {
            //第一种情况，先匀速，再减速
            ddy = dy0*dy0/(y0-y1+dy0*dx)/2;
            dx2 = 2*(y0 - y1 + dy0*dx)/dy0;
            dx1 = dx - dx2;

            if(ddy<ddymin)
            {
                state = 1;
                ddy = std::numeric_limits<double>::quiet_NaN();
                dx1 = std::numeric_limits<double>::quiet_NaN();
                dx2 = std::numeric_limits<double>::quiet_NaN();
                return {state, ddy, dx1, dx2, dx3};
            }

            if(ddy>ddymin && ddy<=ddymax)
            {
                state = 2;
                return {state, ddy, dx1, dx2, dx3};
            }
        }

        // 第二种情况，先加速再减速
        double ddy_tmp1 = -dx*dy0 + 2*delta_y;
        double ddy_tmp2 = std::sqrt(2*(dx*dx*dy0*dy0 - 2*dx*dy0*delta_y + 2*delta_y*delta_y));
        ddy = (ddy_tmp1 + ddy_tmp2)/dx/dx;
        dx1 = dx/2 - dy0/2/ddy;
        dx2 = dx/2 + dy0/2/ddy;

        if(ddy < ddymin){
            state = 1;
            ddy = std::numeric_limits<double>::quiet_NaN();
            dx1 = std::numeric_limits<double>::quiet_NaN();
            dx2 = std::numeric_limits<double>::quiet_NaN();
            return {state, ddy, dx1, dx2, dx3};                   
        }
        else if(ddy > ddymax){
            state = 5;
            ddy = std::numeric_limits<double>::quiet_NaN();
            dx1 = std::numeric_limits<double>::quiet_NaN();
            dx2 = std::numeric_limits<double>::quiet_NaN();
            return {state, ddy, dx1, dx2, dx3};
        }else
        {
            dyPeak = dy0 + ddy*dx1;
            if(dyPeak <= dymax){
                state = 3;
                return {state, ddy, dx1, dx2, dx3};
            } 
        }

        // 第三种情况，先加速、再匀速、再减速
        if(dyPeak > dymax)
        {
            ddy = ddymax;
            double dx1_tmp1 = ddy * dx - dy0;
            double dx1_tmp2 = sqrt(ddy*ddy*dx*dx + 2*ddy*dy0*dx - dy0*dy0 - 4*ddy*delta_y);
            dx1 = (dx1_tmp1 - dx1_tmp2)/2/ddy;
            dx2 = dx1 + dy0/ddy;
            dx3 = dx - 2*dx1 - dy0/ddy;
            dyPeak = dy0 + ddy*dx1;
            if(dyPeak <= dymax){
                state = 4;
                return {state, ddy, dx1, dx2, dx3};
            }else
            {
                ddy = std::numeric_limits<double>::quiet_NaN();
                dx1 = std::numeric_limits<double>::quiet_NaN();
                dx2 = std::numeric_limits<double>::quiet_NaN();
                state = 5;
                return {state, ddy, dx1, dx2, dx3};
            }
        }
    }

    if(delta_y < 0.0)
    {
        if(dy0*dx < 2*delta_y)
        {
            state = 1;
            ddy = std::numeric_limits<double>::quiet_NaN();
            dx1 = std::numeric_limits<double>::quiet_NaN();
            dx2 = std::numeric_limits<double>::quiet_NaN();
            return {state, ddy, dx1, dx2, dx3};
        }

        if(dy0*dx < delta_y && dy0*dx >= 2*delta_y)
        {
            //第一种情况，先匀速，再减速
            ddy = dy0*dy0/(y0-y1+dy0*dx)/2;
            dx2 = 2*(y0 - y1 + dy0*dx)/dy0;
            dx1 = dx - dx2;

            if(ddy>-ddymin)
            {
                state = 1;
                ddy = std::numeric_limits<double>::quiet_NaN();
                dx1 = std::numeric_limits<double>::quiet_NaN();
                dx2 = std::numeric_limits<double>::quiet_NaN();
                return {state, ddy, dx1, dx2, dx3};
            }

            if(ddy<=-ddymin && ddy>=-ddymax)
            {
                state = 2;
                return {state, ddy, dx1, dx2, dx3};
            }
        }

        // 第二种情况，先加速再减速
        double ddy_tmp1 = -dx*dy0 + 2*delta_y;
        double ddy_tmp2 = std::sqrt(2*(dx*dx*dy0*dy0 - 2*dx*dy0*delta_y + 2*delta_y*delta_y));
        ddy = (ddy_tmp1 - ddy_tmp2)/dx/dx;
        dx1 = dx/2 - dy0/2/ddy;
        dx2 = dx/2 + dy0/2/ddy;

        if(ddy > -ddymin){
            state = 1;
            ddy = std::numeric_limits<double>::quiet_NaN();
            dx1 = std::numeric_limits<double>::quiet_NaN();
            dx2 = std::numeric_limits<double>::quiet_NaN();
            return {state, ddy, dx1, dx2, dx3};                   
        }
        else if(ddy < -ddymax){
            state = 5;
            ddy = std::numeric_limits<double>::quiet_NaN();
            dx1 = std::numeric_limits<double>::quiet_NaN();
            dx2 = std::numeric_limits<double>::quiet_NaN();
            return {state, ddy, dx1, dx2, dx3};
        }else
        {
            dyPeak = dy0 + ddy*dx1;
            if(dyPeak >= -dymax){
                state = 3;
                return {state, ddy, dx1, dx2, dx3};
            } 
        }

        // 第三种情况，先加速、再匀速、再减速
        if(dyPeak < -dymax)
        {
            ddy = -ddymax;
            double dx1_tmp1 = ddy * dx - dy0;
            double dx1_tmp2 = sqrt(ddy*ddy*dx*dx + 2*ddy*dy0*dx - dy0*dy0 - 4*ddy*delta_y);
            dx1 = (dx1_tmp1 + dx1_tmp2)/2/ddy;
            dx2 = dx1 + dy0/ddy;
            dx3 = dx - 2*dx1 - dy0/ddy;
            dyPeak = dy0 + ddy*dx1;
            if(dyPeak >= -dymax){
                state = 4;
                return {state, ddy, dx1, dx2, dx3};
            }else
            {
                ddy = std::numeric_limits<double>::quiet_NaN();
                dx1 = std::numeric_limits<double>::quiet_NaN();
                dx2 = std::numeric_limits<double>::quiet_NaN();
                state = 5;
                return {state, ddy, dx1, dx2, dx3};
            }
        }
    }

    return {state, ddy, dx1, dx2, dx3};
}

double CB_Path_Planning::calcTrajPoint(int state, double y0, double y1, double dy0, double ddy, double dx, double dx1, double dx3, double ego_pos_x)
{
    //初始化变量
    double x = ego_pos_x;
    double y = 0;

    if(state == 0){
        y = y0;
    }else if(state == 2){
        if(x>=0 && x<dx1){
            y = y0 + dy0 * x;
        }else if(x>=dx1 && x<=dx){
            y = y0 + dy0 * x - 0.5*ddy*std::pow(x-dx1, 2);
        }
    }else if(state == 3){
        if(x>=0.0 && x<dx1){
            y = y0 + dy0*x + 0.5*ddy*x*x;
        }else if(x>=dx1 && x<=dx){
            double y_t1 = y0 + dy0*dx1 + 0.5*ddy*dx1*dx1;
            double v_t1 = dy0 + ddy*dx1;
            y = y_t1 + v_t1*(x - dx1) - 0.5*ddy*std::pow(x-dx1, 2);
        }
    }else if(state == 4){
        if(x>=0.0 && x<=dx1){
            y = y0 + dy0*x + 0.5*ddy*x*x;
        }else if(x>=dx1 && x<(dx1 + dx3)){
            double y_t1 = y0 + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
            double v_t1 = dy0 + ddy * dx1;
            y = y_t1 + v_t1*(x - dx1);
        }else if(x >= (dx1 + dx3) && x <= dx){
            double y_t1 = y0 + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
            double v_t1 = dy0 + ddy * dx1;
            double y_t2 = y_t1 + v_t1 * dx3;
            double v_t2 = v_t1;
            y = y_t2 + v_t2 * (x - (dx1 + dx3)) - 0.5 * ddy * std::pow(x - (dx1 + dx3), 2);
        }
    }

    /* 边界处理 */
    if(x < 0.0)
        y = y0;
    else if(x > dx)
        y = y1;
    
    return y;
}

void CB_Path_Planning::calcEdges(int state, double y0, double y1, double dy0, double ddy, double dx, double dx1, double dx2, double dx3, double dx4, 
                    double ego_pos_x, double& leftEdge, double& rightEdge){
    //初始化变量

    const double inf  =  std::numeric_limits<double>::infinity();
    leftEdge = inf;
    rightEdge = -inf;

    /* ---------- 提前退出区间 ---------- */
    if ((ego_pos_x > (dx+dx4+planning_para.ego_length/2)) && (ego_pos_x < -planning_para.ego_length/2))
        return;
    
    /* ---------- 匀速直线段 ---------- */
    if ((ego_pos_x > (dx + planning_para.ego_length/2)) && (ego_pos_x <= (dx+dx4+planning_para.ego_length/2)))
    {
        leftEdge = y1 + planning_para.ego_width/2;
        rightEdge = y1 - planning_para.ego_width/2;
        return;
    }

    /* ---------- 状态机 ---------- */
    if (state == 0)   // 直线
    {
        leftEdge  = y0 + planning_para.ego_width / 2;
        rightEdge = y0 - planning_para.ego_width / 2;
    }else if (state == 2)   // 简单换道（两段加速度）
    {
        if (dy0 < 0)
        {
            double x_left  = ego_pos_x - planning_para.ego_length / 2;
            double y_left  = y0 + planning_para.ego_width / 2;
            double x_right = ego_pos_x + planning_para.ego_length / 2;
            double y_right = y0 - planning_para.ego_width / 2;

            /* ---- 左边界 ---- */
            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < planning_para.ego_length / 2)
                leftEdge = y_left;
            else if (ego_pos_x >= planning_para.ego_length / 2 && ego_pos_x < dx1 + planning_para.ego_length / 2)
                leftEdge = y_left + dy0 * x_left;
            else if (ego_pos_x >= dx1 + planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
                leftEdge = y_left + dy0 * x_left - 0.5 * ddy * std::pow(x_left - dx1, 2);
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state2 left, dy0<0)\n";
                return;
            }

            /* ---- 右边界 ---- */
            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < dx1 - planning_para.ego_length / 2)
                rightEdge = y_right + dy0 * x_right;
            else if (ego_pos_x >= dx1 - planning_para.ego_length / 2 && ego_pos_x < dx - planning_para.ego_length / 2)
                rightEdge = y_right + dy0 * x_right - 0.5 * ddy * std::pow(x_right - dx1, 2);
            else if (ego_pos_x >= dx - planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
                rightEdge = y_right + dy0 * dx - 0.5 * ddy * dx2 * dx2;
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state2 right, dy0<0)\n";
                return;
            }
        }
        else if (dy0 > 0)
        {
            double x_left  = ego_pos_x + planning_para.ego_length / 2;
            double y_left  = y0 + planning_para.ego_width / 2;
            double x_right = ego_pos_x - planning_para.ego_length / 2;
            double y_right = y0 - planning_para.ego_width / 2;

            /* ---- 左边界 ---- */
            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < dx1 - planning_para.ego_length / 2)
                leftEdge = y_left + dy0 * x_left;
            else if (ego_pos_x >= dx1 - planning_para.ego_length / 2 && ego_pos_x < dx - planning_para.ego_length / 2)
                leftEdge = y_left + dy0 * x_left - 0.5 * ddy * std::pow(x_left - dx1, 2);
            else if (ego_pos_x >= dx - planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
                leftEdge = y_left + dy0 * dx - 0.5 * ddy * dx2 * dx2;
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state2 left, dy0>0)\n";
                return;
            }

            /* ---- 右边界 ---- */
            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < planning_para.ego_length / 2)
                rightEdge = y_right;
            else if (ego_pos_x >= planning_para.ego_length / 2 && ego_pos_x < dx1 + planning_para.ego_length / 2)
                rightEdge = y_right + dy0 * x_right;
            else if (ego_pos_x >= dx1 + planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
                rightEdge = y_right + dy0 * x_right - 0.5 * ddy * std::pow(x_right - dx1, 2);
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state2 right, dy0>0)\n";
                return;
            }
        }
        else
        {
            std::cerr << "warning: abnormal status (state2 dy0==0)\n";
            return;
        }
    }else if (state == 3)   // 三段加速度（先加后减）
    {
        /* 以下结构与 MATLAB 完全一致，仅做语法转换，不再逐行注释 */
        if (dy0 >= 0 && ddy > 0)
        {
            double x_left  = ego_pos_x + planning_para.ego_length / 2;
            double y_left  = y0 + planning_para.ego_width / 2;
            double x_right = ego_pos_x - planning_para.ego_length / 2;
            double y_right = y0 - planning_para.ego_width / 2;

            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < dx1 - planning_para.ego_length / 2)
                leftEdge = y_left + dy0 * x_left + 0.5 * ddy * x_left * x_left;
            else if (ego_pos_x >= dx1 - planning_para.ego_length / 2 && ego_pos_x < dx - planning_para.ego_length / 2)
            {
                double y_t1 = y_left + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                leftEdge = y_t1 + v_t1 * (x_left - dx1) - 0.5 * ddy * std::pow(x_left - dx1, 2);
            }
            else if (ego_pos_x >= dx - planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
            {
                double y_t1 = y_left + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                leftEdge = y_t1 + v_t1 * dx2 - 0.5 * ddy * dx2 * dx2;
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state3 left, dy0≥0, ddy>0)\n";
                return;
            }

            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < planning_para.ego_length / 2)
                rightEdge = y_right;
            else if (ego_pos_x >= planning_para.ego_length / 2 && ego_pos_x < dx1 + planning_para.ego_length / 2)
                rightEdge = y_right + dy0 * x_right + 0.5 * ddy * x_right * x_right;
            else if (ego_pos_x >= dx1 + planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
            {
                double y_t1 = y_right + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                rightEdge = y_t1 + v_t1 * (x_right - dx1) - 0.5 * ddy * std::pow(x_right - dx1, 2);
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state3 right, dy0≥0, ddy>0)\n";
                return;
            }
        }
        else if (dy0 <= 0 && ddy < 0)
        {
            double x_left  = ego_pos_x - planning_para.ego_length / 2;
            double y_left  = y0 + planning_para.ego_width / 2;
            double x_right = ego_pos_x + planning_para.ego_length / 2;
            double y_right = y0 - planning_para.ego_width / 2;

            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < planning_para.ego_length / 2)
                leftEdge = y_left;
            else if (ego_pos_x >= planning_para.ego_length / 2 && ego_pos_x < dx1 + planning_para.ego_length / 2)
                leftEdge = y_left + dy0 * x_left + 0.5 * ddy * x_left * x_left;
            else if (ego_pos_x >= dx1 + planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
            {
                double y_t1 = y_left + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                leftEdge = y_t1 + v_t1 * (x_left - dx1) - 0.5 * ddy * std::pow(x_left - dx1, 2);
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state3 left, dy0≤0, ddy<0)\n";
                return;
            }

            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < dx1 - planning_para.ego_length / 2)
                rightEdge = y_right + dy0 * x_right + 0.5 * ddy * x_right * x_right;
            else if (ego_pos_x >= dx1 - planning_para.ego_length / 2 && ego_pos_x < dx - planning_para.ego_length / 2)
            {
                double y_t1 = y_right + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                rightEdge = y_t1 + v_t1 * (x_right - dx1) - 0.5 * ddy * std::pow(x_right - dx1, 2);
            }
            else if (ego_pos_x >= dx - planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
            {
                double y_t1 = y_right + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                rightEdge = y_t1 + v_t1 * dx2 - 0.5 * ddy * dx2 * dx2;
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state3 right, dy0≤0, ddy<0)\n";
                return;
            }
        }
        else if (dy0 > 0 && ddy < 0)
        {
            double xlf = ego_pos_x - planning_para.ego_length / 2;
            double ylf = y0 + planning_para.ego_width / 2;
            double xlb = ego_pos_x + planning_para.ego_length / 2;
            double ylb = y0 + planning_para.ego_width / 2;
            double dx_01 = std::fabs(dy0 / ddy);

            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < dx_01 - planning_para.ego_length / 2)
                leftEdge = ylb + dy0 * xlb + 0.5 * ddy * xlb * xlb;
            else if (ego_pos_x >= dx_01 - planning_para.ego_length / 2 && ego_pos_x < dx_01 + planning_para.ego_length / 2)
                leftEdge = ylb + dy0 * dx_01 + 0.5 * ddy * dx_01 * dx_01;
            else if (ego_pos_x >= dx_01 + planning_para.ego_length / 2 && ego_pos_x < dx1 + planning_para.ego_length / 2)
                leftEdge = ylf + dy0 * xlf + 0.5 * ddy * xlf * xlf;
            else if (ego_pos_x >= dx1 + planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
            {
                double y_t1 = ylf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                leftEdge = y_t1 + v_t1 * (xlf - dx1) - 0.5 * ddy * std::pow(xlf - dx1, 2);
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state3 left, dy0>0, ddy<0)\n";
                return;
            }

            /* ---- 右边界：取最小 ---- */
            double xrf = ego_pos_x - planning_para.ego_length / 2;
            double yrf = y0 - planning_para.ego_width / 2;
            double xrb = ego_pos_x + planning_para.ego_length / 2;
            double yrb = y0 - planning_para.ego_width / 2;
            rightEdge = inf;

            auto update = [&](double y) { if (y < rightEdge) rightEdge = y; };

            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < planning_para.ego_length / 2)
                update(yrf);
            if (ego_pos_x >= planning_para.ego_length / 2 && ego_pos_x < dx_01 + planning_para.ego_length / 2)
                update(yrf + dy0 * xrf + 0.5 * ddy * xrf * xrf);
            if (ego_pos_x >= dx_01 - planning_para.ego_length / 2 && ego_pos_x < dx1 - planning_para.ego_length / 2)
                update(yrb + dy0 * xrb + 0.5 * ddy * xrb * xrb);
            if (ego_pos_x >= dx1 - planning_para.ego_length / 2 && ego_pos_x < dx - planning_para.ego_length / 2)
            {
                double y_t1 = yrb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                update(y_t1 + v_t1 * (xrb - dx1) - 0.5 * ddy * std::pow(xrb - dx1, 2));
            }
            if (ego_pos_x >= dx - planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
            {
                double y_t1 = yrb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx2;
                double v_t2 = v_t1;
                update(y_t2 + v_t2 * dx2 - 0.5 * ddy * dx2 * dx2);
            }
        }
        else if (dy0 < 0 && ddy > 0)
        {
            /* 与 dy0>0 && ddy<0 完全对称，左边界取最大，右边界正常分段 */
            double xlb = ego_pos_x + planning_para.ego_length / 2;
            double ylb = y0 + planning_para.ego_width / 2;
            double xlf = ego_pos_x - planning_para.ego_length / 2;
            double ylf = y0 + planning_para.ego_width / 2;
            double dx_01 = std::fabs(dy0 / ddy);
            leftEdge = -inf;

            auto updateL = [&](double y) { if (y > leftEdge) leftEdge = y; };

            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < planning_para.ego_length / 2)
                updateL(ylf);
            if (ego_pos_x >= planning_para.ego_length / 2 && ego_pos_x < dx_01 + planning_para.ego_length / 2)
                updateL(ylf + dy0 * xlf + 0.5 * ddy * xlf * xlf);
            if (ego_pos_x >= dx_01 - planning_para.ego_length / 2 && ego_pos_x < dx1 - planning_para.ego_length / 2)
                updateL(ylb + dy0 * xlb + 0.5 * ddy * xlb * xlb);
            if (ego_pos_x >= dx1 - planning_para.ego_length / 2 && ego_pos_x < dx - planning_para.ego_length / 2)
            {
                double y_t1 = ylb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                updateL(y_t1 + v_t1 * (xlb - dx1) - 0.5 * ddy * std::pow(xlb - dx1, 2));
            }
            if (ego_pos_x >= dx - planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
            {
                double y_t1 = ylb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx2;
                double v_t2 = v_t1;
                updateL(y_t2 + v_t2 * dx2 - 0.5 * ddy * dx2 * dx2);
            }

            /* ---- 右边界 ---- */
            double xrf = ego_pos_x - planning_para.ego_length / 2;
            double yrf = y0 - planning_para.ego_width / 2;
            double xrb = ego_pos_x + planning_para.ego_length / 2;
            double yrb = y0 - planning_para.ego_width / 2;

            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < dx_01 - planning_para.ego_length / 2)
                rightEdge = yrb + dy0 * xrb + 0.5 * ddy * xrb * xrb;
            else if (ego_pos_x >= dx_01 - planning_para.ego_length / 2 && ego_pos_x < dx_01 + planning_para.ego_length / 2)
                rightEdge = yrb + dy0 * dx_01 + 0.5 * ddy * dx_01 * dx_01;
            else if (ego_pos_x >= dx_01 + planning_para.ego_length / 2 && ego_pos_x < dx1 + planning_para.ego_length / 2)
                rightEdge = yrf + dy0 * xrf + 0.5 * ddy * xrf * xrf;
            else if (ego_pos_x >= dx1 + planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
            {
                double y_t1 = yrf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                rightEdge = y_t1 + v_t1 * (xrf - dx1) - 0.5 * ddy * std::pow(xrf - dx1, 2);
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state3 right, dy0<0, ddy>0)\n";
                return;
            }
        }
    }else if (state == 4)   // 四段加速度（加-匀-减）
    {
        /* 以下结构与 MATLAB 完全一致，仅做语法转换，不再逐行注释 */
        if (dy0 >= 0 && ddy > 0)
        {
            double xlb = ego_pos_x + planning_para.ego_length / 2;
            double ylb = y0 + planning_para.ego_width / 2;
            double xrf = ego_pos_x - planning_para.ego_length / 2;
            double yrf = y0 - planning_para.ego_width / 2;

            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < dx1 - planning_para.ego_length / 2)
                leftEdge = ylb + dy0 * xlb + 0.5 * ddy * xlb * xlb;
            else if (ego_pos_x >= dx1 - planning_para.ego_length / 2 && ego_pos_x < (dx1 + dx3) - planning_para.ego_length / 2)
            {
                double y_t1 = ylb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                leftEdge = y_t1 + v_t1 * (xlb - dx1);
            }
            else if (ego_pos_x >= (dx1 + dx3) - planning_para.ego_length / 2 && ego_pos_x < dx - planning_para.ego_length / 2)
            {
                double y_t1 = ylb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                leftEdge = y_t2 + v_t2 * (xlb - (dx1 + dx3)) - 0.5 * ddy * std::pow(xlb - (dx1 + dx3), 2);
            }
            else if (ego_pos_x >= dx - planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
            {
                double y_t1 = ylb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                leftEdge = y_t2 + v_t2 * dx2 - 0.5 * ddy * dx2 * dx2;
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state4 left, dy0≥0, ddy>0)\n";
                return;
            }

            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < planning_para.ego_length / 2)
                rightEdge = yrf;
            else if (ego_pos_x >= planning_para.ego_length / 2 && ego_pos_x < dx1 + planning_para.ego_length / 2)
                rightEdge = yrf + dy0 * xrf + 0.5 * ddy * xrf * xrf;
            else if (ego_pos_x >= dx1 + planning_para.ego_length / 2 && ego_pos_x < (dx1 + dx3) + planning_para.ego_length / 2)
            {
                double y_t1 = yrf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                rightEdge = y_t1 + v_t1 * (xrf - dx1);
            }
            else if (ego_pos_x >= (dx1 + dx3) + planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
            {
                double y_t1 = yrf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                rightEdge = y_t2 + v_t2 * (xrf - (dx1 + dx3)) - 0.5 * ddy * std::pow(xrf - (dx1 + dx3), 2);
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state4 right, dy0≥0, ddy>0)\n";
                return;
            }
        }
        else if (dy0 <= 0 && ddy < 0)
        {
            double xlf = ego_pos_x - planning_para.ego_length / 2;
            double ylf = y0 + planning_para.ego_width / 2;
            double xrb = ego_pos_x + planning_para.ego_length / 2;
            double yrb = y0 - planning_para.ego_width / 2;

            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < planning_para.ego_length / 2)
                leftEdge = ylf;
            else if (ego_pos_x >= planning_para.ego_length / 2 && ego_pos_x < dx1 + planning_para.ego_length / 2)
                leftEdge = ylf + dy0 * xlf + 0.5 * ddy * xlf * xlf;
            else if (ego_pos_x >= dx1 + planning_para.ego_length / 2 && ego_pos_x < (dx1 + dx3) + planning_para.ego_length / 2)
            {
                double y_t1 = ylf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                leftEdge = y_t1 + v_t1 * (xlf - dx1);
            }
            else if (ego_pos_x >= (dx1 + dx3) + planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
            {
                double y_t1 = ylf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                leftEdge = y_t2 + v_t2 * (xlf - (dx1 + dx3)) - 0.5 * ddy * std::pow(xlf - (dx1 + dx3), 2);
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state4 left, dy0≤0, ddy<0)\n";
                return;
            }

            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < dx1 - planning_para.ego_length / 2)
                rightEdge = yrb + dy0 * xrb + 0.5 * ddy * xrb * xrb;
            else if (ego_pos_x >= dx1 - planning_para.ego_length / 2 && ego_pos_x < (dx1 + dx3) - planning_para.ego_length / 2)
            {
                double y_t1 = yrb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                rightEdge = y_t1 + v_t1 * (xrb - dx1);
            }
            else if (ego_pos_x >= (dx1 + dx3) - planning_para.ego_length / 2 && ego_pos_x < dx - planning_para.ego_length / 2)
            {
                double y_t1 = yrb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                rightEdge = y_t2 + v_t2 * (xrb - (dx1 + dx3)) - 0.5 * ddy * std::pow(xrb - (dx1 + dx3), 2);
            }
            else if (ego_pos_x >= dx - planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
            {
                double y_t1 = yrb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                rightEdge = y_t2 + v_t2 * dx2 - 0.5 * ddy * dx2 * dx2;
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state4 right, dy0≤0, ddy<0)\n";
                return;
            }
        }
        else if (dy0 > 0 && ddy < 0)
        {
            double xlb = ego_pos_x + planning_para.ego_length / 2;
            double ylb = y0 + planning_para.ego_width / 2;
            double xlf = ego_pos_x - planning_para.ego_length / 2;
            double ylf = y0 + planning_para.ego_width / 2;
            double dx_01 = std::fabs(dy0 / ddy);

            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < dx_01 - planning_para.ego_length / 2)
                leftEdge = ylb + dy0 * xlb + 0.5 * ddy * xlb * xlb;
            else if (ego_pos_x >= dx_01 - planning_para.ego_length / 2 && ego_pos_x < dx_01 + planning_para.ego_length / 2)
                leftEdge = ylb + dy0 * dx_01 + 0.5 * ddy * dx_01 * dx_01;
            else if (ego_pos_x >= dx_01 + planning_para.ego_length / 2 && ego_pos_x < dx1 + planning_para.ego_length / 2)
                leftEdge = ylf + dy0 * xlf + 0.5 * ddy * xlf * xlf;
            else if (ego_pos_x >= dx1 + planning_para.ego_length / 2 && ego_pos_x < (dx1 + dx3) + planning_para.ego_length / 2)
            {
                double y_t1 = ylf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                leftEdge = y_t1 + v_t1 * (xlf - dx1);
            }
            else if (ego_pos_x >= (dx1 + dx3) + planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
            {
                double y_t1 = ylf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                leftEdge = y_t2 + v_t2 * (xlf - (dx1 + dx3)) - 0.5 * ddy * std::pow(xlf - (dx1 + dx3), 2);
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state4 left, dy0>0, ddy<0)\n";
                return;
            }

            double xrf = ego_pos_x - planning_para.ego_length / 2;
            double yrf = y0 - planning_para.ego_width / 2;
            double xrb = ego_pos_x + planning_para.ego_length / 2;
            double yrb = y0 - planning_para.ego_width / 2;
            rightEdge = inf;

            auto updateR = [&](double y) { if (y < rightEdge) rightEdge = y; };

            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < planning_para.ego_length / 2)
                updateR(yrf);
            if (ego_pos_x >= planning_para.ego_length / 2 && ego_pos_x < dx_01 + planning_para.ego_length / 2)
                updateR(yrf + dy0 * xrf + 0.5 * ddy * xrf * xrf);
            if (ego_pos_x >= dx_01 - planning_para.ego_length / 2 && ego_pos_x < dx1 - planning_para.ego_length / 2)
                updateR(yrb + dy0 * xrb + 0.5 * ddy * xrb * xrb);
            if (ego_pos_x >= dx1 - planning_para.ego_length / 2 && ego_pos_x < (dx1 + dx3) - planning_para.ego_length / 2)
            {
                double y_t1 = yrb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                updateR(y_t1 + v_t1 * (xrb - dx1));
            }
            if (ego_pos_x >= (dx1 + dx3) - planning_para.ego_length / 2 && ego_pos_x < dx - planning_para.ego_length / 2)
            {
                double y_t1 = yrb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                updateR(y_t2 + v_t2 * (xrb - (dx1 + dx3)) - 0.5 * ddy * std::pow(xrb - (dx1 + dx3), 2));
            }
            if (ego_pos_x >= dx - planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
            {
                double y_t1 = yrb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                updateR(y_t2 + v_t2 * dx2 - 0.5 * ddy * dx2 * dx2);
            }
        }
        else if (dy0 < 0 && ddy > 0)
        {
            double xlb = ego_pos_x + planning_para.ego_length / 2;
            double ylb = y0 + planning_para.ego_width / 2;
            double xlf = ego_pos_x - planning_para.ego_length / 2;
            double ylf = y0 + planning_para.ego_width / 2;
            double dx_01 = std::fabs(dy0 / ddy);
            leftEdge = -inf;

            auto updateL = [&](double y) { if (y > leftEdge) leftEdge = y; };

            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < planning_para.ego_length / 2)
                updateL(ylf);
            if (ego_pos_x >= planning_para.ego_length / 2 && ego_pos_x < dx_01 + planning_para.ego_length / 2)
                updateL(ylf + dy0 * xlf + 0.5 * ddy * xlf * xlf);
            if (ego_pos_x >= dx_01 - planning_para.ego_length / 2 && ego_pos_x < dx1 - planning_para.ego_length / 2)
                updateL(ylb + dy0 * xlb + 0.5 * ddy * xlb * xlb);
            if (ego_pos_x >= dx1 - planning_para.ego_length / 2 && ego_pos_x < (dx1 + dx3) - planning_para.ego_length / 2)
            {
                double y_t1 = ylb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                updateL(y_t1 + v_t1 * (xlb - dx1));
            }
            if (ego_pos_x >= (dx1 + dx3) - planning_para.ego_length / 2 && ego_pos_x < dx - planning_para.ego_length / 2)
            {
                double y_t1 = ylb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                updateL(y_t2 + v_t2 * (xlb - (dx1 + dx3)) - 0.5 * ddy * std::pow(xlb - (dx1 + dx3), 2));
            }
            if (ego_pos_x >= dx - planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
            {
                double y_t1 = ylb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                updateL(y_t2 + v_t2 * dx2 - 0.5 * ddy * dx2 * dx2);
            }

            double xrf = ego_pos_x - planning_para.ego_length / 2;
            double yrf = y0 - planning_para.ego_width / 2;
            double xrb = ego_pos_x + planning_para.ego_length / 2;
            double yrb = y0 - planning_para.ego_width / 2;

            if (ego_pos_x >= -planning_para.ego_length / 2 && ego_pos_x < dx_01 - planning_para.ego_length / 2)
                rightEdge = yrb + dy0 * xrb + 0.5 * ddy * xrb * xrb;
            else if (ego_pos_x >= dx_01 - planning_para.ego_length / 2 && ego_pos_x < dx_01 + planning_para.ego_length / 2)
                rightEdge = yrb + dy0 * dx_01 + 0.5 * ddy * dx_01 * dx_01;
            else if (ego_pos_x >= dx_01 + planning_para.ego_length / 2 && ego_pos_x < dx1 + planning_para.ego_length / 2)
                rightEdge = yrf + dy0 * xrf + 0.5 * ddy * xrf * xrf;
            else if (ego_pos_x >= dx1 + planning_para.ego_length / 2 && ego_pos_x < (dx1 + dx3) + planning_para.ego_length / 2)
            {
                double y_t1 = yrf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                rightEdge = y_t1 + v_t1 * (xrf - dx1);
            }
            else if (ego_pos_x >= (dx1 + dx3) + planning_para.ego_length / 2 && ego_pos_x <= dx + planning_para.ego_length / 2)
            {
                double y_t1 = yrf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                rightEdge = y_t2 + v_t2 * (xrf - (dx1 + dx3)) - 0.5 * ddy * std::pow(xrf - (dx1 + dx3), 2);
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state4 right, dy0<0, ddy>0)\n";
                return;
            }
        }
    }
}

std::vector<Point2D> CB_Path_Planning::calcObsEdgePoints(std::vector<Point2D> bbox, double lower, double upper)
{
    if(bbox.size() != 4){
        std::cerr << "warning: bbox size != 4\n";
        return {};
    }

    auto mmx = std::minmax_element(bbox.begin(), bbox.end(),
                                    [](const Point2D& a, const Point2D& b)
                                    { return a.x < b.x; });
    double min_x = mmx.first->x;
    double max_x = mmx.second->x;

    /* 快速排除 */
    if (max_x < lower || min_x > upper) return{};

    /* 刚好贴边 */
    if(max_x == lower){
        std::vector<Point2D> pts;
        for(const auto& p : bbox)
            if(p.x == max_x) pts.push_back(p);
        return pts;
    }
    if(min_x == upper){
        std::vector<Point2D> pts;
        for(const auto& p:bbox)
            if(p.x == min_x) pts.push_back(p);
        return pts;
    }

    /* 完全包含 */
    if(min_x >= lower && max_x <= upper) return bbox;

    /* 一般情况： 求四条边与x=lower / x=upper, 并补充内部顶点 */
    std::vector<Point2D> pts;
    std::vector<Point2D> tmp = bbox;
    tmp.push_back(bbox[0]);

    auto addIntersect = [&](double xcut)
    {
        for (size_t i = 0; i < 4; ++i) {
            double x1 = tmp[i].x, y1 = tmp[i].y;
            double x2 = tmp[i + 1].x, y2 = tmp[i + 1].y;

            if (y1 == y2) continue;            // 水平边
            if (x1 == x2) {                    // 垂直边
                if (xcut >= min_x && xcut <= max_x) {
                    pts.emplace_back(xcut, y1);
                    pts.emplace_back(xcut, y2);
                }
                break;
            }
            double t = (xcut - x1) / (x2 - x1);
            if (t < 0.0 || t > 1.0) continue;
            double y = y1 + t * (y2 - y1);
            pts.emplace_back(xcut, y);
        }
    };
    addIntersect(lower);
    addIntersect(upper);

    /* 补充位于 (lower,upper) 内的原始顶点 */
    for (const auto& p : bbox)
        if (p.x > lower && p.x < upper)
            pts.push_back(p);

    /* 排序 + 去重 */
    std::sort(pts.begin(), pts.end(), cmpPt);
    pts.erase(std::unique(pts.begin(), pts.end(),
                          [](const Point2D& a, const Point2D& b)
                          { return a.x == b.x && a.y == b.y; }),
              pts.end());
    return pts;
}

OneObsResult CB_Path_Planning::CollisionCheck_for_OneObstacle(int state, double y0, double y1, double dy0, double ddy,
                                double dxsum, double dx1, double dx2, double dx3, double dx4, const Vec2V& bbox)
{
    OneObsResult res;

    /* 1. 计算有效边界点 */
    res.obsEdgePts = calcObsEdgePoints(bbox, -planning_para.ego_length / 2.0,  (dxsum + dx4) + planning_para.ego_length / 2.0);
    std::size_t n = res.obsEdgePts.size();
    if(n == 0){
        res.isCollision = false;
        return res;
    }

    /* 2. 对每个点判断是否左右越界 */
    std::vector<bool> onLeft(n, false), onRight(n, false);
    for(std::size_t i=0; i<n; i++){
        double obs_x = res.obsEdgePts[i].x;
        double obs_y = res.obsEdgePts[i].y;

        double leftEdge, rightEgde;
        calcEdges(state, y0, y1, dy0, ddy, dxsum, dx1, dx2, dx3, dx4, obs_x, leftEdge,rightEgde);
        
        if(obs_y > leftEdge)        onLeft[i] = true;
        else if(obs_y < rightEgde)  onRight[i] = true;
        /* 介于区间内，两个标志均为false */
    }

    /* 3. 碰撞逻辑：只有“全部在左侧”或“全部在右侧”才算无碰撞 */
    bool allLeft = std::all_of(onLeft.begin(), onLeft.end(), [](bool v){ return v; });
    bool allRight = std::all_of(onRight.begin(), onRight.end(), [](bool v){ return v;});

    res.isLoE = allLeft;
    res.isRoE = allRight;
    res.isCollision = !(allLeft || allRight);
    return res;
}

BatchObsResult CB_Path_Planning::CollisionCheck(int state, double y0, double y1, double dy0, double ddy,
                                double dx, double dx1, double dx2, double dx3, double dx4, const Vec2V& edge_points)
{
    BatchObsResult res;
    size_t obs_num = edge_points.size() / 4;   // 4 个点一组
    res.isLoE.resize(obs_num, false);
    res.isRoE.resize(obs_num, false);
    res.obsEps.resize(obs_num);

    for (size_t m = 0; m < obs_num; ++m)
    {
        std::vector<Point2D> bbox(edge_points.begin() + m * 4,
                                    edge_points.begin() + (m + 1) * 4);

        auto r = CollisionCheck_for_OneObstacle(state, y0, y1, dy0, ddy, dx, dx1, dx2, dx3, dx4, bbox);
        res.isLoE[m] = r.isLoE;
        res.isRoE[m] = r.isRoE;
        res.obsEps[m] = std::move(r.obsEdgePts);

        if (r.isCollision)
        {
            res.isCollision = true;
            return res;               // 一旦碰撞立即退出，与 MATLAB 一致
        }
    }
    res.isCollision = false;
    return res;
}

DrivableRange CB_Path_Planning::calcDrivableRange(Vec2V obs_edge_points, double ego_pos_x, int ref_lane)
{
    /* 1. 检查输入障碍物点的数量 */
    Vec2V pts = obs_edge_points;
    if (pts.size() % 4 != 0){
        std::cerr << "顶点数必须是 4 的倍数\n";
        return {};
    }

    size_t obs_num = pts.size() / 4;

    /* 2. 先筛选在 [ego_pos_x-planning_para.ego_length/2, ego_pos_x+planning_para.ego_length/2] 范围内的障碍物 */
    Vec2V obs_eps_Vlength;                  // 存有效障碍物顶点（连续 4 个一组）
    std::vector<double> Yo_max_Vlength;
    std::vector<double> Yo_min_Vlength;
    size_t no = 0;                          // 有效障碍物计数

    double x_low = ego_pos_x - planning_para.ego_length/2.0;
    double x_high = ego_pos_x + planning_para.ego_length/2.0;

    for(size_t k=0; k<obs_num; k++){
        Vec2V bbox(pts.begin() + k * 4, pts.begin() + (k + 1) * 4);
        Vec2V eps = calcObsEdgePoints(bbox, x_low, x_high);
        if (!eps.empty()){
            /* 记录整个障碍物 4 顶点 */
            obs_eps_Vlength.insert(obs_eps_Vlength.end(), bbox.begin(), bbox.end());
            
            /* 记录该障碍物在纵向范围内的 Y 极值 */
            auto mmy = std::minmax_element(eps.begin(), eps.end(), [](const Point2D& a, const Point2D& b) {return a.y < b.y; });
            Yo_max_Vlength.push_back(mmy.second->y);
            Yo_min_Vlength.push_back(mmy.first ->y);
            no ++;
        }
    }

    /* 3.车身纵向范围内无障碍物 -> 走车道中心 */
    if(no == 0){
        double Ll = planning_para.lane_begin_y + ref_lane * planning_para.lane_width;
        double Lr = planning_para.lane_begin_y + (ref_lane - 1) * planning_para.lane_width;
        return {(Ll+Lr)/2.0, Ll, Lr, 0};
    }

    /* 4.再计算这些障碍物在局部地图 X 范围中的 Y 极值 */
    std::vector<double> Yo_max_localMap(no);
    std::vector<double> Yo_min_localMap(no);
    std::vector<int>    is_inRefLane(no, 0);

    double map_x_low = -planning_para.ego_length/2.0;
    double map_x_high = planning_para.local_map_forward_x + planning_para.ego_length/2.0;

    double Ll_lane = planning_para.lane_begin_y + std::min(ref_lane + 1, planning_para.lane_num) * planning_para.lane_width;
    double Lr_lane = planning_para.lane_begin_y + std::max(ref_lane - 2, 0) * planning_para.lane_width;

    for(size_t k=0; k<no; k++){
        Vec2V bbox(obs_eps_Vlength.begin() + k * 4, obs_eps_Vlength.begin() + (k + 1) * 4);
        Vec2V eps = calcObsEdgePoints(bbox, map_x_low, map_x_high);
        auto mmy = std::minmax_element(eps.begin(), eps.end(), [](const Point2D& a, const Point2D& b) {return a.y < b.y;});
        Yo_max_localMap[k] = mmy.second->y;
        Yo_min_localMap[k] = mmy.first->y;

        /* 判断是否侵入当前车道 */
        if(Yo_max_localMap[k] < (ref_lane - 1)*planning_para.lane_width ||
            Yo_min_localMap[k] > ref_lane * planning_para.lane_width)
            is_inRefLane[k] = 0;
        else
            is_inRefLane[k] = 1;
    }

    /* 5. 若局部地图内所有障碍物均未侵入车道 -> 走车道中心 */
    if(!std::any_of(is_inRefLane.begin(), is_inRefLane.end(), [](int v){ return v==1;}))
    {
        double Ll = planning_para.lane_begin_y + ref_lane * planning_para.lane_width;
        double Lr = planning_para.lane_begin_y + (ref_lane - 1) * planning_para.lane_width;
        return {(Ll+Lr)/2.0, Ll, Lr, 0};
    }

    /* 6. 有障碍物侵入，计算绕行边界 */
    std::vector<double> FreeSp_l(no), FreeSp_r(no);
    std::vector<double> Ll_obs(no), Lr_obs(no);

    for (size_t k = 0; k < no; ++k){
        FreeSp_l[k] = Ll_lane - Yo_max_Vlength[k];
        FreeSp_r[k] = Yo_min_Vlength[k] - Lr_lane;

        if (FreeSp_l[k] >= FreeSp_r[k]){
            Ll_obs[k] = std::numeric_limits<double>::infinity();
            Lr_obs[k] = Yo_max_Vlength[k];
        } else {
            Ll_obs[k] = Yo_min_Vlength[k];
            Lr_obs[k] = -std::numeric_limits<double>::infinity();
        }
    }

    /* 7. 合并车道边界与障碍物边界 */
    Ll_obs.push_back(Ll_lane);
    Lr_obs.push_back(Lr_lane);

    // for(size_t k=0; k<Ll_obs.size(); k++){
    //     std::cout << "Ll_obs[" << k << "] = " << Ll_obs[k] << ", Lr_obs[" << k << "] = " << Lr_obs[k] << "\n";
    // }

    double Ll = *std::min_element(Ll_obs.begin(), Ll_obs.end());
    double Lr = *std::max_element(Lr_obs.begin(), Lr_obs.end());
    double yc = (Ll + Lr) / 2.0;

    return { yc, Ll, Lr, 1 };
}

DrivableRange CB_Path_Planning::calcDrivableRange(Vec2V obs_edge_points, double ego_pos_x, double ref_points_y)
{
    double lane1_middle_y = planning_para.lane_begin_y + planning_para.lane_width * 0.5;
    double ref_lane_tmp = (ref_points_y - lane1_middle_y)/planning_para.lane_width + 1.0;
    int ref_lane = static_cast<int>(ref_lane_tmp);
    if(ref_lane_tmp == static_cast<double>(ref_lane))
    {
        return calcDrivableRange(obs_edge_points, ego_pos_x, ref_lane);
    }

    // 剩下的情况是参考路径不在车道中心线的情况（变道情况）

    /* 1. 检查输入障碍物点的数量 */
    Vec2V pts = obs_edge_points;
    if (pts.size() % 4 != 0){
        std::cerr << "顶点数必须是 4 的倍数\n";
        return {};
    }

    size_t obs_num = pts.size() / 4;

    /* 2. 先筛选在 [ego_pos_x-planning_para.ego_length/2, ego_pos_x+planning_para.ego_length/2] 范围内的障碍物 */
    Vec2V obs_eps_Vlength;                  // 存有效障碍物顶点（连续 4 个一组）
    std::vector<double> Yo_max_Vlength;
    std::vector<double> Yo_min_Vlength;
    size_t no = 0;                          // 有效障碍物计数

    double x_low = ego_pos_x - planning_para.ego_length/2.0;
    double x_high = ego_pos_x + planning_para.ego_length/2.0;

    for(size_t k=0; k<obs_num; k++){
        Vec2V bbox(pts.begin() + k * 4, pts.begin() + (k + 1) * 4);
        Vec2V eps = calcObsEdgePoints(bbox, x_low, x_high);
        if (!eps.empty()){
            /* 记录整个障碍物 4 顶点 */
            obs_eps_Vlength.insert(obs_eps_Vlength.end(), bbox.begin(), bbox.end());
            
            /* 记录该障碍物在纵向范围内的 Y 极值 */
            auto mmy = std::minmax_element(eps.begin(), eps.end(), [](const Point2D& a, const Point2D& b) {return a.y < b.y; });
            Yo_max_Vlength.push_back(mmy.second->y);
            Yo_min_Vlength.push_back(mmy.first ->y);
            no ++;
        }
    }

    /* 3.车身纵向范围内无障碍物 -> 沿着参考线走 */
    if(no == 0){
        double Ll = planning_para.lane_begin_y + (ref_lane + 1) * planning_para.lane_width;
        double Lr = planning_para.lane_begin_y + (ref_lane - 1) * planning_para.lane_width;
        return {ref_points_y, Ll, Lr, 0};
    }

    /* 4.再计算这些障碍物在局部地图 X 范围中的 Y 极值 */
    std::vector<double> Yo_max_localMap(no);
    std::vector<double> Yo_min_localMap(no);
    std::vector<int>    is_inRefLane(no, 0);

    double map_x_low = -planning_para.ego_length/2.0;
    double map_x_high = planning_para.local_map_forward_x + planning_para.ego_length/2.0;

    double Ll_lane = planning_para.lane_begin_y + std::min(ref_lane + 1, planning_para.lane_num) * planning_para.lane_width;
    double Lr_lane = planning_para.lane_begin_y + std::max(ref_lane - 2, 0) * planning_para.lane_width;

    for(size_t k=0; k<no; k++){
        Vec2V bbox(obs_eps_Vlength.begin() + k * 4, obs_eps_Vlength.begin() + (k + 1) * 4);
        Vec2V eps = calcObsEdgePoints(bbox, map_x_low, map_x_high);
        auto mmy = std::minmax_element(eps.begin(), eps.end(), [](const Point2D& a, const Point2D& b) {return a.y < b.y;});
        Yo_max_localMap[k] = mmy.second->y;
        Yo_min_localMap[k] = mmy.first->y;

        /* 判断是否侵入当前车道 */
        if(Yo_max_localMap[k] < (ref_lane - 1)*planning_para.lane_width ||
            Yo_min_localMap[k] > (ref_lane + 1) * planning_para.lane_width)
            is_inRefLane[k] = 0;
        else
            is_inRefLane[k] = 1;
    }

    /* 5. 若局部地图内所有障碍物均未侵入两个车道 -> 走车道中心 */
    if(!std::any_of(is_inRefLane.begin(), is_inRefLane.end(), [](int v){ return v==1;}))
    {
        double Ll = planning_para.lane_begin_y + (ref_lane + 1) * planning_para.lane_width;
        double Lr = planning_para.lane_begin_y + (ref_lane - 1) * planning_para.lane_width;
        return {ref_points_y, Ll, Lr, 0};
    }

    /* 6. 有障碍物侵入，计算绕行边界 */
    std::vector<double> FreeSp_l(no), FreeSp_r(no);
    std::vector<double> Ll_obs(no), Lr_obs(no);

    for (size_t k = 0; k < no; ++k){
        FreeSp_l[k] = Ll_lane - Yo_max_Vlength[k];
        FreeSp_r[k] = Yo_min_Vlength[k] - Lr_lane;

        if (FreeSp_l[k] >= FreeSp_r[k]){
            Ll_obs[k] = std::numeric_limits<double>::infinity();
            Lr_obs[k] = Yo_max_Vlength[k];
        } else {
            Ll_obs[k] = Yo_min_Vlength[k];
            Lr_obs[k] = -std::numeric_limits<double>::infinity();
        }
    }

    /* 7. 合并车道边界与障碍物边界 */
    Ll_obs.push_back(Ll_lane);
    Lr_obs.push_back(Lr_lane);

    double Ll = *std::min_element(Ll_obs.begin(), Ll_obs.end());
    double Lr = *std::max_element(Lr_obs.begin(), Lr_obs.end());
    double yc = (Ll + Lr) / 2.0;

    return { yc, Ll, Lr, 1 };
}

CostResult CB_Path_Planning::calcCost(int state, double y0, double y1, double dy0, double ddy, double dx, double dx1, double dx3, double dx4, 
                    std::vector<double> X_local_set, std::vector<double> Yc_set, std::vector<int> NoObs_set)
{
    //初始化变量
    // const double dis0 = 0.1;
    const double W11  = 10.0;
    const double W12  = 0.5;
    const double W2   = 0.0;
    // const double W3   = 1.0;

    double dis        = dx + dx4;
    size_t points_num = 0;
    double y_deviation_sum = 0.0;
    double y_longitude_sum = 0.0;
    size_t k_max = static_cast<size_t>(std::ceil(dis/2.0));
    std::vector<double> Disafemin_set(k_max, 0.0);

    for (size_t k = 0; k < k_max; ++k){
        double ego_pos_x = X_local_set[k];
        double ego_pos_y = calcTrajPoint(state, y0, y1, dy0, ddy, dx, dx1, dx3, ego_pos_x);
        points_num ++;

        double refY = Yc_set[k];
        int NoObs = NoObs_set[k];
        if(NoObs)
            y_deviation_sum += W12 * std::fabs(ego_pos_y - refY);
        else
            y_deviation_sum += W11 * std::fabs(ego_pos_y - refY);
    }

    double y_deviation_cost = y_deviation_sum / points_num;
    double y_longitude_cost = y_longitude_sum / points_num;
    double ay_cost = W2 * std::fabs(ddy);

    /* 总代价 */
    double cost = y_deviation_cost + 50.0 * y_longitude_cost + ay_cost;
    return {cost, Disafemin_set, y_longitude_cost};
}

}  // namespace cb_planning

}  // namespace igv_path_planner
