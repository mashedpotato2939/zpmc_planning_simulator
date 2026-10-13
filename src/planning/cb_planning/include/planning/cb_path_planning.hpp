#ifndef CRABWALKING_PATH_PLANNING_HPP_
#define CRABWALKING_PATH_PLANNING_HPP_

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

#include "struct_types.hpp"

#include <tf2/utils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#define _USE_MATH_DEFINES      //使用PI
#include <cmath>
#include <algorithm>
#include <limits>

namespace cb_planning
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

using Vec2V = std::vector<Point2D>; 


class CB_Path_Planning{
public:
    explicit CB_Path_Planning();
    ~CB_Path_Planning() = default;

    // Param param;
    inline void setParameter(std::shared_ptr<Param> param_) {
        ego_length = param_->ego_length;
        ego_width = param_->ego_width;
        local_map_length = param_->local_map_length;
        lane_para.lane_begin_x = param_->lane_begin_x;
        lane_para.lane_end_x = param_->lane_end_x;
        lane_para.lane_width = param_->lane_width;
        lane_para.lane_num = param_->lane_num;
        lane_para.lane_sampling_begin_y = param_->lane_sampling_begin_y;
        lane_para.lane_sampling_end_y = param_->lane_sampling_end_y;
        lane_para.lane_sampling_interval = param_->lane_sampling_interval;
    }
    static inline bool cmpPt(const Point2D& a, const Point2D& b){
        return a.x < b.x || (a.x == b.x && a.y < b.y);}

    std::vector<Point2D> calcObsEdgePoints(std::vector<Point2D> eps_for_OneObstacle, double lower, double upper);

    /* 计算可行驶范围 */
    DrivableRange calcDrivableRange(Vec2V obs_edge_points, double ego_pos_x, int ref_lane);
    DrivableRange calcDrivableRange(Vec2V obs_edge_points, double ego_pos_x, double ref_points);

    LocalPlanResult local_plan(std::vector<double> X_local_set, double start_point_x, double start_point_y, double start_point_dy, Vec2V edge_local_points,
        double Min_Collision_free_dis, std::vector<double> Yc_set, std::vector<int> NoObs_set, double dymax, double ddymax, double ddymin);

    LocalPlanResult local_replan(std::vector<double> X_local_set, double start_point_x, double start_point_y, double start_point_dy, Vec2V edge_local_points,
        double Min_Collision_free_dis, std::vector<double> Yc_set, std::vector<int> NoObs_set, double dymax, double ddymax, double ddymin);

    void calcPosy_Vy(std::vector<double> traj, double ego_pos_x, double & ego_pos_y, double & ego_pos_vy);

private:

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

    Lane_Para lane_para;
    std::vector<Point2D> edge_points;
    std::vector<Point2D> edge_points_for_OneObstacle;
    std::vector<Edges> edges;

    double start_point_x;
    double start_point_y;
    double start_point_dy;
    double ego_length;
    double ego_width;
    double local_map_length;
};
}


#endif