#include "optimization_path_planning.hpp"

namespace cb_planning
{
OPT_Path_Planning::OPT_Path_Planning()
{
    // decision_config = std::make_unique<cb_planning::ConfigReader>("config/planning/optimization_path_planning.yaml");
    // 读取配置文件
    opt_planning_config = std::make_unique<cb_planning::ConfigReader>();
    opt_planning_config->read_planning_config();

    RCLCPP_INFO(rclcpp::get_logger("OPT_Path_Planning"), "Optimization Path Planning Module Initialized.");
}
   
void OPT_Path_Planning::make_path_decision(const std::shared_ptr<Ego_Info> &car, 
                                            const std::vector<std::shared_ptr<ObstacleStruct>> &obses)
{
    if(obses.empty())  //  没有障碍物
    {  
        return;
    }

    // 初始化
    sl_points_.clear();
    const double lane_width = opt_planning_config->lane().lane_width;                   // 车道宽度
    const double left_bound_l = opt_planning_config->lane().lane_end_y;                 // 道路左边界
    const double right_bound_l = opt_planning_config->lane().lane_begin_y;              // 道路右边界
    const double forward_dis = opt_planning_config->planning().local_map_forward_x;     // 开始考虑障碍物的范围

    const double vx_ref = opt_planning_config->planning().vx_ref;                       // 参考车速
    const double kv = 3.6;                                                              // 变道距离的速度增益参数
    const double least_length = std::max(kv*vx_ref, 5.0);                               // 最小变道距离
    double backward_dis = std::max(least_length, opt_planning_config->planning().local_map_backward_x); // 变道后方预留距离

    SLPoint p;

    // 针对每个障碍物计算变道点位
    for(const auto &obs : obses)
    {
        const double obs_dis_s = obs->pose_x_ - car->ego_pos_x;         // 障碍物与自车的纵向距离
        if(obs_dis_s > forward_dis || obs_dis_s < -backward_dis)
        {
            continue;   // 超出考虑范围，跳过
        }
        
        double obs_vy = obs->speed_ori_ * sin(obs->pose_theta_);            // 障碍物的横向速度分量
        double obs_vx = obs->speed_ori_ * cos(obs->pose_theta_);            // 障碍物的纵向速度分量

        const double safety_margin = 0.1; // 安全边距
        const double stop_margin = 2.0;   // 停车预留距离

        if((obs->pose_y_ + obs->width_/2.0 > right_bound_l) && (obs->pose_y_ - obs->width_/2.0 < left_bound_l) &&
            fabs(obs_vy) < 0.1 && (obs_vx < vx_ref/2.0))                    // 障碍物在车道内且横向速度较小且纵向速度小于自车速度的一半
        {
            p.s_ = obs->pose_x_ + obs_vx * obs_dis_s /(vx_ref - obs_vx);    // 变道点纵向位置
            const double obs_left_bound_l = obs->pose_y_ + obs->width_/2.0;
            const double obs_right_bound_l = obs->pose_y_ - obs->width_/2.0;
            const double left_width = left_bound_l - obs_left_bound_l;
            const double right_width = obs_right_bound_l - right_bound_l;

            if(left_width > car->ego_width + safety_margin * 2.0)       // 如果左边宽度能够通过
            {
                p.l_ = (left_bound_l + obs_left_bound_l) / 2.0;
                p.type_ = static_cast<int>(SLPointType::LEFT_PASS);
                sl_points_.push_back(p);
            }else
            {
                if(right_width > car->ego_width + safety_margin * 2.0)  // 如果右边宽度能够通过
                {
                    p.l_ = (right_bound_l + obs_right_bound_l) / 2.0;
                    p.type_ = static_cast<int>(SLPointType::RIGHT_PASS);
                    sl_points_.push_back(p);
                }else                                                   // 两边都无法通过，跳过该障碍物
                {
                    p.l_ = 0.0;
                    p.s_ = obs->pose_x_ - obs->length_/2.0 - car->ego_length/2.0 - stop_margin;
                    p.type_ = static_cast<int>(SLPointType::STOP);
                    sl_points_.push_back(p);
                    RCLCPP_INFO(rclcpp::get_logger("OPT_Path_Planning"), "Obstacle ID %d cannot be avoided, need to stop.", obs->id_);
                    break;                                              // 更前方的障碍物不再考虑
                }
            }
        }
    }

    // 头尾的处理
    SLPoint p_start;
    p_start.s_ = sl_points_[0].s_ - least_length;
    p_start.l_ = right_bound_l + lane_width / 2.0;
    p_start.type_ = static_cast<int>(SLPointType::START);
    sl_points_.insert(sl_points_.begin(), p_start);        //头插

    if(sl_points_.back().type_ != static_cast<int>(SLPointType::STOP))
    {
        SLPoint p_end;
        p_end.s_ = sl_points_.back().s_ + least_length;
        p_end.l_ = right_bound_l + lane_width / 2.0;
        p_end.type_ = static_cast<int>(SLPointType::END);
        sl_points_.push_back(p_end);                            //尾插
    }
}

std::vector<Point2D> OPT_Path_Planning::create_local_path(int ref_lane, const std::shared_ptr<Ego_Info> &car, std::vector<SLPoint> &sl_points)
{
    // 初始化局部路径
    std::vector<Point2D> local_path;

    // 计算路径点的 x, y值
    double point_s = car->ego_pos_x;
    Point2D point_tmp;
    const size_t local_path_point_num = 80;
    // 计算采样间隔，局部地图长度 / 点数
    const double path_sampling_interval = opt_planning_config->planning().local_map_forward_x / static_cast<double>(local_path_point_num - 1);
    local_path.resize(local_path_point_num);
    for(size_t i=0; i<local_path_point_num; ++i)
    {
        // 规划的起点
        point_tmp.x = point_s + i * path_sampling_interval;
        if(0)
    }


    // 在x，y坐标下平滑

    
}
}