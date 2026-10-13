#ifndef STRUCT_TYPES_HPP_
#define STRUCT_TYPES_HPP_

#include <rclcpp/rclcpp.hpp>

struct CB_Trajecotry_Para{
    // 输入
    double y0;
    double y1;
    double dy0;
    double dx;
    double dymax;
    double ddymax;
    double ddymin;

    // 输出
    uint8_t state;  //0-5, 0表示直行；1表示横向距离太小，规划失败；2表示先匀速再减速；3表示先加速再减速；4表示先加速、匀速再减速；5表示横向距离太大、规划失败
    double ddy;
    double dx1;
    double dx2;
    double dx3;
    double dx4;
};

struct Lane_Para{
    double lane_begin_x;
    double lane_end_x;
    double lane_begin_y;
    double lane_end_y;
    double lane_width;
    int lane_num;
    double lane_sampling_begin_y;
    double lane_sampling_end_y;
    double lane_sampling_interval;
    // double ref_lane;
};

struct Point2D{
    double x; 
    double y;

    Point2D() = default;
    Point2D(double x_, double y_) : x(x_), y(y_) {}
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
};

struct Param
{
    double ego_length;
    double ego_width;
    double local_map_length;
    double lane_begin_x;
    double lane_end_x;
    double lane_begin_y;
    double lane_end_y;
    double lane_width;
    int lane_num;
    double lane_sampling_begin_y;
    double lane_sampling_end_y;
    double lane_sampling_interval;
};

struct Lane_Info{
    double lane_begin_x;
    double lane_begin_y;
    double lane_end_x;
    double lane_end_y;
    double lane_width;
    int lane_num;
  
    double lane_change_start_x;
    double lane_change_end_x;
};
  
struct Object_Info{
    size_t obs_id;
    double obs_global_pos_x;
    double obs_global_pos_y;
    double obs_global_heading_angle;
    double obs_length;
    double obs_width;
    double obs_velocity;
    std::vector<Point2D> edge_points;
};
  
struct Ego_Info{
    double ego_length;
    double ego_width;
    double ego_pos_x;
    double ego_pos_y;
    double ego_pos_vy;
    double ego_pos_vx;
    double steer_angle_max;
    double steer_angular_vel_max;
};
  
struct Planning_Para{
    double dest_x;
    double Min_Collision_free_distance;
    double lane_sampling_begin_y;       
    double lane_sampling_end_y;
    double lane_x_sampling_interval;
    double lane_y_sampling_interval;
    double local_map_forward_x;
    double local_map_backward_x;
    double vx_ref;
    double ax_drive;
    double ax_brake;
};

#endif