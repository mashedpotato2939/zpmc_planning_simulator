#include "cb_planning_node.hpp"
#include <chrono>
#include <cmath>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/utils.h"
#include "geometry_msgs/msg/quaternion.hpp"

namespace cb_planning
{

using namespace std::literals::chrono_literals;

CrabWalking_Planning::CrabWalking_Planning(const rclcpp::NodeOptions & options)
: Node("crabwalking_planning", options)
{
    // 初始化定位信息的订阅者
    is_odom_new = false;
    odom_sub_ = create_subscription<Odometry>(
        "input/kinematics", 10, std::bind(&CrabWalking_Planning::odom_callback, this, std::placeholders::_1));

    // 初始化轨迹输出的发布者
    traj_pub_ = this->create_publisher<Trajectory>("output/trajectory", 10);

    // 初始化可视化的发布者
    traj_rviz_pub_ = this->create_publisher<Marker>("traj_rviz", 10);
    lane_rviz_pub_ = this->create_publisher<MarkerArray>("map_rviz", 10);
    vehicle_rviz_pub_ = this->create_publisher<Marker>("vehicle_rviz",10);
    obstacle_rviz_pub_ = this->create_publisher<MarkerArray>("obstacle_rviz",10);
    dynamic_obs_rviz_pub_ = this->create_publisher<Marker>("dynamic_obstacle_rviz",10);
    dbg_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/debug/data", 10);

    sim_sync_pub_ = this->create_publisher<msgUpperTest>("/sim/sync", 10);

    srv_upper_test_ = this->create_service<srvUpperTest>("/srv_upper_test", 
        std::bind(&CrabWalking_Planning::srv_upper_test_callback, this, std::placeholders::_1, std::placeholders::_2));

    // 初始化场景使能参数
    Enable_static_obstacle_avoidance_scenario = this->declare_parameter<bool>(
        "Enable_static_obstacle_avoidance_scenario", false);
    Enable_overtaking_scenario = this->declare_parameter<bool>("Enable_overtaking_scenario", true);
    
    // 初始化配置文件读取指针
    planning_config_ = std::make_unique<ConfigReader>();

    // 初始化车道信息
    lane_info_init();
    
    // 初始化障碍物
    if(Enable_static_obstacle_avoidance_scenario)
    {
        static_obstacles_info_init();
    }else if(Enable_overtaking_scenario)
    {
        moving_obstacle_info_init();
    }

    // 初始化车辆尺寸和状态
    ego_info_init();

    // 初始化规划算法相关参数
    planning_para_init();

    // 初始化全局参考路径
    generate_reference_path();
    
    // 初始化定时器
    using namespace std::literals::chrono_literals;
    timer_ = rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&CrabWalking_Planning::onTimer, this));
}


void CrabWalking_Planning::onTimer()
{
    // auto start = std::chrono::high_resolution_clock::now();
    if(!is_odom_new){
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "waiting for localization msgs...");
        return;
    }else{
        is_odom_new = false;
    }
    
    // 读取位置和速度
    ego_info.ego_pos_x = ego_state_ptr_->pose.pose.position.x;
    ego_info.ego_pos_y = ego_state_ptr_->pose.pose.position.y;
    ego_info.ego_pos_vx = ego_state_ptr_->twist.twist.linear.x;
    ego_info.ego_pos_vy = ego_state_ptr_->twist.twist.linear.y;

    vehicle_vis_update();

    // 更新起点位置
    if(is_first_plan)
    {
        start_point_x = ego_info.ego_pos_x;
        start_point_y = ego_info.ego_pos_y;
        // start_point_dy = ego_info.ego_pos_vy/std::max(ego_info.ego_pos_vx, 0.1);
        start_point_dy = 0.0;
        vx_plan = 0.0;
        is_first_plan = false;
    }else
    {
        start_point_x = ego_info.ego_pos_x;
        double opt_x_prev = opt_traj_prev.at(1);
        path_planning->calcPosy_Vy(opt_traj_prev, start_point_x - opt_x_prev, start_point_y, start_point_dy);

        //计算起点位置的速度
        double dx_tmp = start_point_x - opt_x_prev;
        if(dx_tmp < 0.0)
            vx_plan = speed_points_prev.at(0).v;
        
        for(size_t i=0; i<speed_points_prev.size() - 2; i++)
        {
            if(dx_tmp >= speed_points_prev.at(i).x && dx_tmp < speed_points_prev.at(i+1).x)
            {
                double dx_interval = speed_points_prev.at(i+1).x - speed_points_prev.at(i).x;
                double dv_interval = speed_points_prev.at(i+1).v - speed_points_prev.at(i).v;
                vx_plan = speed_points_prev.at(i).v + dv_interval/dx_interval * (dx_tmp - speed_points_prev.at(i).x);
                break;
            }
        }
        // RCLCPP_INFO(this->get_logger(), "vx:%f", vx_plan);
    }

    // 计算局部地图内障碍物位置，并显示出来
    if(Enable_static_obstacle_avoidance_scenario)
    {
        obstacle_vis_update();
    }else if(Enable_overtaking_scenario)
    {
        moving_obstacle_vis_update();
    }

    SpeedProfile res;
    LocalPlanResult res_path;
    std::vector<double> x_local_map_set;
    std::vector<double> Ll_set;
    std::vector<double> Lr_set;

    // 横向规划
    // 初始化局部地图网格
    double lane_end_x = lane_info.lane_end_x;
    double x_grid_end = std::min(planning_para.local_map_forward_x, (lane_end_x - start_point_x));
    std::vector<double> x_global_set;
    for(double x = planning_para.lane_x_sampling_interval; x <= x_grid_end; x += planning_para.lane_x_sampling_interval)
        x_global_set.push_back(x);
    
    double x_local_set_bias = start_point_x - uint32_t(start_point_x);
    for(size_t i=0; i<x_global_set.size(); i++)
        x_local_map_set.push_back(x_global_set.at(i) - x_local_set_bias);

    // if(start_point_x > 35.0)
    // {
    //     vx_ref = 6.0;
    //     ax_drive = 1.6;
    //     ax_brake = 1.6;
    // }

    // 纵向规划
    // 首先通过纵向规划得到在x轴上的点，再用横向规划计算在y轴上的点
    res = vel_planning->longitudinal_speed_planning(vx_plan, vx_ref, x_grid_end, planning_para.dest_x-start_point_x, 
        ax_drive, ax_brake, t_interval, isbraking);
    speed_points_prev = res.pts;
    SpeedPoint tmp;
    tmp.t = 0.0;
    tmp.v = vx_plan;
    tmp.x = 0.0;
    speed_points_prev.insert(speed_points_prev.begin(), tmp);
    // 调试信息：将res.pts 最近位置的速度和位置打印出来
    // std_msgs::msg::Float32MultiArray arr;
    // arr.data = {(float)res.pts.at(plot_cnt).v, (float)res.pts.at(plot_cnt).x};
    // dbg_pub_->publish(arr);
    // plot_cnt ++;
    // if(plot_cnt >= res.pts.size())
    //     plot_cnt = 0;
    // RCLCPP_INFO(this->get_logger(), "x_ref:%f, x:%f, v:%f", res.pts.at(0).x + ego_info.ego_pos_x, ego_info.ego_pos_x, res.pts.at(0).v);

    // 更新vxmax,ddymax
    auto it = std::max_element(res.pts.begin(), res.pts.end(), 
        [](const SpeedPoint& a, const SpeedPoint& b) { return a.v < b.v; });
    double vx_max = it->v;
    vx_max = std::max(vx_max, 0.1);
    double ddy_max = ego_info.steer_angular_vel_max/180.0 * M_PI/vx_max; // 转向角速度对应的横向加速度
    double ddy_min = 0.0;
    double dymax = ego_info.steer_angle_max/180.0 * M_PI; // 最大转向角对应的横向速度
    
    // 将障碍物的边缘点转换到局部坐标系下
    Vec2V edge_local_points;
    if(Enable_static_obstacle_avoidance_scenario)
    {
        for(size_t i=0; i<objects_info.size(); i++)
        {
            for(size_t j=0; j<objects_info.at(i).edge_points.size(); j++)
            {
                Point2D ep_local;
                ep_local.x = objects_info.at(i).edge_points.at(j).x - start_point_x;
                ep_local.y = objects_info.at(i).edge_points.at(j).y;
                edge_local_points.push_back(ep_local);
            }
        }
    }else if(Enable_overtaking_scenario)
    {
        auto edge_points = moving_obs_edge_points_update(moving_object_info);
        for(size_t j=0; j<edge_points.size(); j++)
        {
            Point2D ep_local;
            ep_local.x = edge_points.at(j).x - start_point_x;
            ep_local.y = edge_points.at(j).y;
            edge_local_points.push_back(ep_local);
        }
    }

    // 根据规划起点，计算车道中的可行驶区域
    std::vector <double> Yc_set;
    std::vector <int> NoObs_set;
    
    Yc_set.reserve(x_local_map_set.size());
    Ll_set.reserve(x_local_map_set.size());
    Lr_set.reserve(x_local_map_set.size());
    NoObs_set.reserve(x_local_map_set.size());
    for(size_t i=0; i<x_local_map_set.size(); i++)
    {
        DrivableRange drivable_range;
        double ref_path_x = start_point_x + x_local_map_set[i];
        size_t ind = static_cast<size_t>((ref_path_x-lane_info.lane_begin_x)/planning_para.lane_x_sampling_interval) + 1;
        ind = std::min(ind, global_reference_path.size()-1);
        double ref_path_y = global_reference_path.at(ind).y;
        drivable_range = path_planning->calcDrivableRange(edge_local_points, x_local_map_set[i], ref_path_y);
        Yc_set.push_back(drivable_range.yc);
        Ll_set.push_back(drivable_range.Ll);
        Lr_set.push_back(drivable_range.Lr);
        NoObs_set.push_back(drivable_range.NoObs);
    }
        // 在正常状态下，根据当前位置进行局部规划；如果无法正常规划出轨迹，执行重规划，重规划成功进入重规划状态
    if(!is_replanning)
    {
        // 进行横向规划
        double L_min = planning_para.Min_Collision_free_distance;
        res_path = path_planning->local_plan(x_local_map_set, start_point_x, start_point_y, start_point_dy, edge_local_points,
            L_min, Yc_set, NoObs_set, dymax, ddy_max, ddy_min);

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
            if(isnan(opt_traj_prev.at(0)))
            {
                // 第一次就无法输出可行轨迹，输出停止轨迹，直接退出
                RCLCPP_WARN(this->get_logger(), "The first planning attempt failed, so the system exited directly");
                for(size_t i=0; i<res.pts.size(); i++)
                    res.pts.at(i).v = 0.0;
            }else
            {
                // 初始化上一步的局部地图网格
                double opt_x_prev = opt_traj_prev.at(1);
                double x_grid_end_prev = std::min(planning_para.local_map_forward_x, (lane_end_x - opt_x_prev));
                std::vector<double> x_global_set_prev;
                for(double x = planning_para.lane_x_sampling_interval; x <= x_grid_end_prev; x += planning_para.lane_x_sampling_interval)
                    x_global_set_prev.push_back(x);
                
                double x_local_set_bias_prev = opt_x_prev - uint32_t(opt_x_prev);
                std::vector<double> x_local_set_prev;
                for(size_t i=0; i<x_global_set_prev.size(); i++)
                    x_local_set_prev.push_back(x_global_set_prev.at(i) - x_local_set_bias_prev);
                
                std::vector<Point2D> edge_local_points_prev;
                for(size_t i=0; i<objects_info.size(); i++)
                {
                    for(size_t j=0; j<objects_info.at(i).edge_points.size(); j++)
                    {
                        Point2D ep_local;
                        ep_local.x = objects_info.at(i).edge_points.at(j).x - opt_x_prev;
                        ep_local.y = objects_info.at(i).edge_points.at(j).y;
                        edge_local_points_prev.push_back(ep_local);
                    }
                }

                size_t replan_num = static_cast<int>(L_min)/2 - 1;
                std::vector <bool> isSuccess_replan(replan_num, false);
                for(size_t i=0; i<replan_num; i++)
                {
                    // 由于opt_traj_prev的长度是超过20的，进行9次遍历搜索不会超过轨迹总长度
                    double start_pos_x_replan = 2 * i;
                    double start_pos_y_replan;
                    double start_pos_dy_replan;
                    path_planning->calcPosy_Vy(opt_traj_prev, start_pos_x_replan, start_pos_y_replan, start_pos_dy_replan);

                    LocalPlanResult res_path_replan;
                    res_path_replan = path_planning->local_replan(x_local_set_prev, start_pos_x_replan, start_pos_y_replan, start_pos_dy_replan, 
                        edge_local_points_prev, L_min, Yc_set_prev, NoObs_set_prev, dymax, ddy_max, ddy_min);
                    
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
                        res_path_replan.opt_traj.at(1) = opt_x_prev + start_pos_x_replan; // 将重规划的起点位置转换为全局坐标系下的位置
                        for(size_t j=0; j<res_path_replan.opt_traj.size(); j++)
                        {
                            opt_traj_inreplanning.at(j)[0] = opt_traj_prev.at(j);
                            opt_traj_inreplanning.at(j)[1] = res_path_replan.opt_traj.at(j);
                        }
                        is_replanning = true;
                        RCLCPP_INFO(this->get_logger(), "x of Initial planning: %lf, x of replanning: %lf, the number of collision free trajectories of replanning: %d", 
                            opt_x_prev, start_pos_x_replan, res_path_replan.collision_free_num);
                        break;
                    }
                }

                // 如果9次重规划都失败，则输出停止轨迹，直接退出
                if(!std::any_of(isSuccess_replan.begin(), isSuccess_replan.end(), [](int v){ return v==1;}))
                {
                    RCLCPP_WARN(this->get_logger(), "Replanning failed! ");
                    is_first_plan = true;
                    for(size_t i=0; i<res.pts.size(); i++)
                        res.pts.at(i).v = 0.0;
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

    // std::cout << "error_points" << std::endl;
    // 更新车辆轨迹的横向状态
    std::vector<double> opt_traj_y;
    std::vector<double> opt_traj_vy;
    opt_traj_y.reserve(res.pts.size());
    opt_traj_vy.reserve(res.pts.size());

    std::vector<Point2D> traj_plot_xy;
    
    if(is_replanning)
    {
        double start_pos_x_replan = opt_traj_inreplanning.at(1)[1];                                                      // 全局坐标系下的重规划起点位置
        double end_pos_x_replan = start_pos_x_replan + opt_traj_inreplanning.at(9)[1] + opt_traj_inreplanning.at(8)[1];  // 全局坐标系下的重规划终点位置
        double start_pos_x_first = opt_traj_inreplanning.at(1)[0];
        if (ego_info.ego_pos_x >= start_pos_x_replan)
        {
            
            std::vector<double> opt_traj_replan;
            for(size_t i=0; i<opt_traj_inreplanning.size(); i++){
                opt_traj_replan.push_back(opt_traj_inreplanning.at(i)[1]);
            }
            // 填充横向轨迹点
            double y;
            double vy;
            for(size_t i=0; i<res.pts.size(); i++)
            {
                path_planning->calcPosy_Vy(opt_traj_replan, res.pts.at(i).x, y, vy);
                opt_traj_y.push_back(y);
                opt_traj_vy.push_back(vy * res.pts.at(i).v);
            }

            // 填充绘制轨迹所需要的横纵坐标
            traj_plot_xy = calc_XYplot(opt_traj_replan, ego_info.ego_pos_x - start_pos_x_replan, end_pos_x_replan, 10);
            is_replanning = false;
        } else
        {
            std::vector<double> opt_traj_first;
            std::vector<double> opt_traj_replan;
            for(size_t i=0; i<opt_traj_inreplanning.size(); i++){
                opt_traj_first.push_back(opt_traj_inreplanning.at(i)[0]);
                opt_traj_replan.push_back(opt_traj_inreplanning.at(i)[1]);
            }

            double y;
            double vy;
            // 填充横向轨迹点
            for(size_t i=0; i<res.pts.size(); i++)
            {
                if(res.pts.at(i).x + start_point_x < start_pos_x_replan)
                {
                    path_planning->calcPosy_Vy(opt_traj_first, res.pts.at(i).x, y, vy);
                }else
                {
                    path_planning->calcPosy_Vy(opt_traj_replan, res.pts.at(i).x, y, vy);
                }
                opt_traj_y.push_back(y);
                opt_traj_vy.push_back(vy * res.pts.at(i).v);
            }
            
            // 填充绘制轨迹所需要的横纵坐标
            std::vector<Point2D> traj_plot_xy_first;
            std::vector<Point2D> traj_plot_xy_replan;
            // RCLCPP_INFO(this->get_logger(), "start_x:%f, end_x:%f" ,ego_info.ego_pos_x - start_pos_x_first, end_pos_x_replan - start_pos_x_replan);
            traj_plot_xy_first = calc_XYplot(opt_traj_first, ego_info.ego_pos_x - start_pos_x_first, start_pos_x_replan - start_pos_x_first, 10);
            traj_plot_xy_replan = calc_XYplot(opt_traj_replan, 0.0, end_pos_x_replan - start_pos_x_replan, 10);
            for(size_t i=0; i<traj_plot_xy_first.size(); i++){
                traj_plot_xy_first.at(i).x += start_pos_x_first;
                traj_plot_xy.push_back(traj_plot_xy_first.at(i));
            }

            for(size_t i=0; i<traj_plot_xy_replan.size(); i++){
                traj_plot_xy_replan.at(i).x += start_pos_x_replan;
                traj_plot_xy.push_back(traj_plot_xy_replan.at(i));
            }
            
        }
    }else
    {
        // std::vector<double> opt_traj;
        double y;
        double vy;
        // 填充横向轨迹点
        for(size_t i=0; i<res.pts.size(); i++)
        {
            path_planning->calcPosy_Vy(res_path.opt_traj, res.pts.at(i).x, y, vy);
            opt_traj_y.push_back(y);
            opt_traj_vy.push_back(vy * res.pts.at(i).v); // 横向速度 = 横向斜率 * 纵向速度
        }

        // 填充绘制轨迹所需要的横纵坐标
        double end_pos_x = res_path.opt_traj[8] + res_path.opt_traj[9];
        traj_plot_xy = calc_XYplot(res_path.opt_traj, 0.0, end_pos_x, 10);
    }
    
    // 输出轨迹
    Trajectory traj_planned;
    for(size_t i = 0; i < res.pts.size(); i++){
        TrajectoryPoint tp;
        tp.pose.position.x = res.pts.at(i).x + start_point_x;
        tp.pose.position.y = opt_traj_y.at(i);
        
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        tp.pose.orientation = tf2::toMsg(q);
        tp.longitudinal_velocity_mps = res.pts.at(i).v;
        tp.lateral_velocity_mps = opt_traj_vy.at(i);
        tp.heading_rate_rps = 0.0;
        traj_planned.points.emplace_back(tp);
    }
    traj_pub_->publish(traj_planned);
    
    // 可视化规划轨迹，以及可行驶区域
    traj_vis_update(traj_plot_xy, x_local_map_set, Ll_set, Lr_set);
    // auto end = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // std::cout << "Execution time: " << duration.count() << " μs" << std::endl;
    
}

void CrabWalking_Planning::lane_vis_update()
{
    MarkerArray marr;

    int id = 0;
    size_t lane_num = lane_info.lane_num;

    // 绘制车道边界线
    for(size_t i =0; i< lane_num+1; i++)
    {
        Marker m;
        m.header.frame_id = planning_config_->lane().frame_;
        m.header.stamp    = rclcpp::Clock().now();
        m.ns              = "cb_planning";
        m.id              = id++;
        m.type            = Marker::LINE_STRIP;
        m.action          = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.1;                   // 线宽
        m.color.r = 1.0; 
        m.color.g = 1.0; 
        m.color.b = 1.0; 
        m.color.a = 1.0;
        m.lifetime = rclcpp::Duration::max(); // 不消失
        m.frame_locked = true;

        //计算边界线x坐标
        double len_tmp = 0.0;
        Point p;
        while (len_tmp <= lane_info.lane_end_x- lane_info.lane_begin_x)
        {
            p.x = lane_info.lane_begin_x + len_tmp;
            p.y = lane_info.lane_begin_y + i*lane_info.lane_width;
            p.z = 0.0;
            m.points.push_back(p);
            len_tmp = len_tmp + 1.0;
        }
        marr.markers.push_back(m); 
    }

    // 绘制车道中心线
    for(size_t i =0; i< lane_num; i++)
    {
        Marker m;
        m.header.frame_id = "map";
        m.header.stamp    = rclcpp::Clock().now();
        m.ns              = "cb_planning";
        m.id              = id++;
        m.type            = Marker::LINE_LIST;
        m.action          = Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.1;                   // 线宽
        m.color.r = 0.7; 
        m.color.g = 0.7; 
        m.color.b = 0.0; 
        m.color.a = 1.0;
        m.lifetime = rclcpp::Duration::max(); // 不消失
        m.frame_locked = true;

        // 计算中心线 x 的坐标
        double len_tmp = 0.0;
        Point p;
        while (len_tmp <= lane_info.lane_end_x - lane_info.lane_begin_x)
        {
            p.x = lane_info.lane_begin_x + len_tmp;
            p.y = lane_info.lane_begin_y + i*lane_info.lane_width + lane_info.lane_width/2.0;
            p.z = 0.0;
            m.points.push_back(p);
            len_tmp = len_tmp + 1.0;
        }
        if (m.points.size() % 2 == 1)
            m.points.pop_back();
        marr.markers.push_back(m);
    }

    // 发布车道线显示话题
    lane_rviz_pub_->publish(marr);
}

void CrabWalking_Planning::vehicle_vis_update()
{
    if(!ego_state_ptr_){
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No localization data");
        return;
    }

    Marker m;
    m.header.frame_id = "map";
    m.header.stamp    = rclcpp::Clock().now();
    m.ns              = "cb_planning";
    m.id              = 0;
    m.type            = Marker::CUBE;
    m.action          = Marker::ADD;
    m.frame_locked = false;
    m.pose = ego_state_ptr_->pose.pose;
    m.pose.position.z = 1.0;
    m.scale.x = planning_config_->vehicle().ego_length;
    m.scale.y = planning_config_->vehicle().ego_width;
    m.scale.z = 1.0;

    m.color.r = 0.8; 
    m.color.g = 0.6; 
    m.color.b = 0.27; 
    m.color.a = 0.8;

    vehicle_rviz_pub_->publish(m);

    /* 2. 局部地图浅灰色区域 */
    Marker local_map;
    local_map.header.frame_id = "map";
    local_map.header.stamp    = now();
    local_map.ns              = "local_map";
    local_map.id              = 0;
    local_map.type            = Marker::CUBE;
    local_map.action          = Marker::ADD;
    local_map.frame_locked    = false;

    // 中心位置：以车当前 x 为原点，x 范围 [-10,50] → 中心 20 m
    double local_start = ego_info.ego_pos_x - planning_para.local_map_backward_x;                 // 理论起点
    double local_end   = ego_info.ego_pos_x + planning_para.local_map_forward_x;                 // 理论终点
    local_end   = std::min(local_end, lane_info.lane_end_x);        // 不允许超过车道尽头
    double size_x   = local_end - local_start;
    double center_x = (local_start + local_end) / 2.0;
    double center_y = (lane_info.lane_begin_y + lane_info.lane_end_y) / 2.0;                // 你已有的车道中心 y
    local_map.pose.position.x = center_x;
    local_map.pose.position.y = center_y;
    local_map.pose.position.z = 0.05;              // 贴地
    local_map.pose.orientation = geometry_msgs::msg::Quaternion(); // 单位

    // 尺寸
    local_map.scale.x = size_x;   // -10 ~ 50
    local_map.scale.y = lane_info.lane_width*lane_info.lane_num + 4.0;    // lane_begin_y ±2
    local_map.scale.z = 0.1;    // 很薄，看起来像平面

    // 浅灰色
    local_map.color.r = 0.7f;
    local_map.color.g = 0.7f;
    local_map.color.b = 0.7f;
    local_map.color.a = 0.5f;   // 半透明，不会遮挡障碍物

    vehicle_rviz_pub_->publish(local_map);
}

void CrabWalking_Planning::obstacle_vis_update()
{
    double lower = ego_info.ego_pos_x - planning_para.local_map_backward_x;
    double upper = ego_info.ego_pos_x + planning_para.local_map_forward_x;
    const double height       = 2.0;                        // 棱柱高度，可调
    const std::string fixed_frame = "map";

    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = fixed_frame;
    mk.header.stamp = now();
    mk.ns = "obstacles";
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.pose.orientation = geometry_msgs::msg::Quaternion();
    mk.scale            = geometry_msgs::msg::Vector3();
    mk.color.r = 0.3;
    mk.color.g = 0.3;
    mk.color.b = 0.3;
    mk.color.a = 1.0;
    mk.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;

    uint32_t id = 0;
  // 依次处理 objects_info 中每个障碍物（假定每个障碍物 edge_points 为四点矩形）
  for (const auto &obj : objects_info) {
    if (obj.edge_points.size() < 3) continue;

    // 1. 转成 Point2D
    std::vector<Point2D> bbox = obj.edge_points;

    // 2. 用现有函数裁剪，得到局部地图内的多边形
    auto clipped_raw = path_planning->calcObsEdgePoints(bbox, lower, upper);
    // if (clipped_raw.empty())  continue;

    // 3. 保证顺时针/逆时针顺序
    auto clipped     = orderPolygon(clipped_raw);
    if (clipped.size() < 3) 
    {
        clipped.clear();
        for(size_t i=0; i<obj.edge_points.size(); i++)
        {
            Point2D p;
            p.x = obj.edge_points.at(i).x + 1000.0; // 移出视野外
            p.y = obj.edge_points.at(i).y + 1000.0;
            clipped.push_back(p);  
        }
    } 
    // continue;

    // 将坐标打印出来以便调试
    // RCLCPP_INFO(get_logger(), "clipped pts:");
    // for (const auto &p : clipped_raw) {
    //     RCLCPP_INFO(get_logger(), "(%f, %f)", p.x, p.y);
    // }

    // 4. 构建上下底面顶点
    std::vector<geometry_msgs::msg::Point> top, bot;
    top.reserve(clipped.size());
    bot.reserve(clipped.size());
    for (const auto &p : clipped) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x;  pt.y = p.y;
        pt.z = 0.0;  bot.push_back(pt);
        pt.z = height; top.push_back(pt);
    }

    // 5. 用 TRIANGLE_LIST 画棱柱
    visualization_msgs::msg::Marker prism = mk;
    prism.id = id; // 简单生成唯一 id
    prism.points.clear();
    id++;

    prism.scale.x = 1.0;
    prism.scale.y = 1.0;
    prism.scale.z = 1.0;

    auto addTri = [&](const auto& a, const auto& b, const auto& c) {
        prism.points.push_back(a);
        prism.points.push_back(b);
        prism.points.push_back(c);
    };
    
    /* ★2 侧面四边形 → 2 个三角形 */
    for (size_t i = 0; i < clipped.size() - 1; ++i) {   // 用 size-1 避免重复闭合
        size_t j = i + 1;
        addTri(bot[i], bot[j], top[j]);
        addTri(bot[i], top[j], top[i]);
    }
    
    /* ★3 可选底/顶面（保证 3 倍数） */
    for (size_t i = 1; i + 1 < clipped.size(); ++i) {   // 扇形 triangulation
        addTri(bot[0], bot[i+1], bot[i]);   // 底面
        addTri(top[0], top[i], top[i+1]);   // 顶面（注意 CCW）
    }
    
    /* ★4 空/无效 marker 直接不发 */
    if (prism.points.empty() || prism.points.size() % 3 != 0) {
        RCLCPP_DEBUG(get_logger(), "skip invalid TriMesh, pts=%zu", prism.points.size());
        continue;
    }

    ma.markers.push_back(prism);
  }

    // 6. 清空旧 marker（发布 DELETEALL 的单独 marker）
    if (ma.markers.empty()) {
        visualization_msgs::msg::Marker clear;
        clear.header.frame_id = fixed_frame;
        clear.header.stamp    = now();
        clear.ns              = "obstacles";
        clear.id              = 0;
        clear.action          = visualization_msgs::msg::Marker::DELETEALL;
        ma.markers.push_back(clear);
    }

    obstacle_rviz_pub_->publish(ma);
}

void CrabWalking_Planning::moving_obstacle_vis_update()
{
    // 判断自车是否到达终点，到终点后不再更新动态障碍物位置
    if(!ego_state_ptr_){
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No localization data");
        return;
    }

    if(upper_test.enable)
        is_moving = true;

    double ego_pos_x = ego_info.ego_pos_x;
    double ego_pos_y = ego_info.ego_pos_y;
    if((std::abs(ego_pos_x - planning_para.dest_x) < 0.1) && (is_moving == true))
        is_moving = false;
        
    // 对动态障碍物的位置进行更新
    if(is_moving)
        dynamic_obstacle_pos_x += moving_object_info.obs_velocity * 0.1;
    
    double obs_pos_x = dynamic_obstacle_pos_x;
    double obs_front_x = obs_pos_x + moving_object_info.obs_length / 2.0;
    double obs_rear_x  = obs_pos_x - moving_object_info.obs_length / 2.0;
    double lm_forward_x = ego_pos_x + planning_para.local_map_forward_x;
    double lm_backward_x = ego_pos_x - planning_para.local_map_backward_x;    

    // 判断是否执行超车动作，当动态障碍物进入局部地图范围内时，执行超车动作，isovertaking = true；
    // 当动态障碍物离开局部地图范围后，且完成超车绕障回到参考线附近是，isovertaking = false；
    if(obs_rear_x <= lm_forward_x && obs_front_x >= lm_backward_x)
        is_overtaking = true;
    
    if((obs_front_x < lm_backward_x) && ((ego_pos_y - lane_info.lane_begin_y - lane_info.lane_width/2.0) < 0.0))
        is_overtaking = false;
    
    // 根据isovertaking,给出参考车速，如果正在超车，则参考车速设置为超车车速；如果未超车，则参考车速设置为正常巡航车速
    const double overtake_speed = 6.0; 
    if(is_overtaking)
        vx_ref = overtake_speed;
    else
        vx_ref = planning_para.vx_ref;

    // 然后进行可视化显示，要判断是否在局部地图范围内
    double marker_scale_x = 0.0; 
    double marker_scale_y = 0.0;
    double marker_vis_pos_x = 0.0;
    if((obs_front_x <= lm_backward_x) || (obs_rear_x >= lm_forward_x))
    {
        marker_scale_x = moving_object_info.obs_length;
        marker_scale_y = moving_object_info.obs_width;
        marker_vis_pos_x = 10000.0; // 移出视野外
    } else if((obs_rear_x >= lm_backward_x) && (obs_front_x <= lm_forward_x))
    {
        marker_scale_x = moving_object_info.obs_length;
        marker_scale_y = moving_object_info.obs_width;
        marker_vis_pos_x = obs_pos_x;
    } else if((obs_rear_x < lm_backward_x) && (obs_front_x > lm_backward_x) && (obs_front_x <= lm_forward_x))
    {
        marker_scale_x = obs_front_x - lm_backward_x;
        marker_scale_y = moving_object_info.obs_width;
        marker_vis_pos_x = lm_backward_x + marker_scale_x / 2.0;
    } else if((obs_rear_x >= lm_backward_x) && (obs_rear_x < lm_forward_x) && (obs_front_x > lm_forward_x))
    {
        marker_scale_x = lm_forward_x - obs_rear_x;
        marker_scale_y = moving_object_info.obs_width;
        marker_vis_pos_x = obs_rear_x + marker_scale_x / 2.0;
    }
    
    Marker m;
    m.header.frame_id = "map";
    m.header.stamp    = rclcpp::Clock().now();
    m.ns              = "dynamic_obstacle";
    m.id              = 0;
    m.type            = Marker::CUBE;
    m.action          = Marker::ADD;
    m.frame_locked = false;
    m.pose.position.x = marker_vis_pos_x;
    m.pose.position.y = dynamic_obstacle_pos_y;
    m.pose.position.z = 1.0;
    m.scale.x = marker_scale_x;
    m.scale.y = marker_scale_y;
    m.scale.z = 1.0;

    m.color.r = 0.3;
    m.color.g = 0.3;
    m.color.b = 0.3;
    m.color.a = 0.8;

    dynamic_obs_rviz_pub_->publish(m);
}

std::vector<Point2D> CrabWalking_Planning::orderPolygon(std::vector<Point2D> pts)
{
    /* ---- 1. 去重 ---- */
    auto hash = [](const Point2D &p){ return std::hash<double>{}(p.x) ^ (std::hash<double>{}(p.y) << 1); };
    auto eq   = [](const Point2D &a, const Point2D &b){ return std::fabs(a.x - b.x) < 1e-6 && std::fabs(a.y - b.y) < 1e-6; };
    std::unordered_set<Point2D, decltype(hash), decltype(eq)> seen(0, hash, eq);
    std::vector<Point2D> unique;
    for (auto &p : pts) if (seen.insert(p).second) unique.push_back(p);
    pts = std::move(unique);
    if (pts.size() < 3) return {};

    /* ---- 2. 删共线 ---- */
    std::vector<Point2D> clean;
    const double eps = 1e-8;
    for (size_t i = 0; i < pts.size(); ++i){
        size_t prev = (i + pts.size() - 1) % pts.size();
        size_t curr = i;
        size_t next = (i + 1) % pts.size();
        double cross = (pts[curr].x - pts[prev].x)*(pts[next].y - pts[curr].y) -
                       (pts[curr].y - pts[prev].y)*(pts[next].x - pts[curr].x);
        if (std::fabs(cross) > eps) clean.push_back(pts[curr]);
    }
    pts = std::move(clean);
    if (pts.size() < 3) return {};

    /* ---- 3. 重心 + 极角排序 ---- */
    Point2D c{0,0};
    for (auto &p : pts){ c.x += p.x; c.y += p.y; }
    c.x /= pts.size(); c.y /= pts.size();
    std::sort(pts.begin(), pts.end(),
              [&](const Point2D &a, const Point2D &b){
                  return std::atan2(a.y - c.y, a.x - c.x) < std::atan2(b.y - c.y, b.x - c.x);
              });

    /* ---- 4. 闭合 & 保证 CCW ---- */
    auto& first = pts.front();
    auto& last  = pts.back();
    if (((first.x - last.x)*(first.x - last.x)+(first.y - last.y)*(first.y - last.y)) > 1e-12)
        pts.push_back(first);
    double cross = 0;
    for (size_t i = 0; i < pts.size()-1; ++i){
        size_t j = i+1;
        cross += pts[i].x * pts[j].y - pts[j].x * pts[i].y;
    }
    if (cross < 0) std::reverse(pts.begin(), pts.end());
    return pts;
}

void CrabWalking_Planning::traj_vis_update(std::vector<Point2D> traj_plot_xy, std::vector<double> x_local_map_set, 
    std::vector<double> Ll_set, std::vector<double> Lr_set)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "trajectory";
    marker.id = 1;
    marker.type = marker.LINE_STRIP;
    marker.action = marker.ADD;
    marker.scale.x = 0.2;
    marker.color.r = 0.0f;
    marker.color.g = 0.95f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    for(size_t i=0; i<traj_plot_xy.size(); i++){
        geometry_msgs::msg::Point p;
        p.x = traj_plot_xy.at(i).x + start_point_x;
        p.y = traj_plot_xy.at(i).y;
        p.z = 0.15;
        marker.points.push_back(p);
    }

    traj_rviz_pub_->publish(marker);

    // 可行驶区域显示
    Marker Ll_marker;
    Ll_marker.header.frame_id = "map";
    Ll_marker.header.stamp = now();
    Ll_marker.ns = "Ll_set";
    Ll_marker.id = 2;
    Ll_marker.type = Marker::LINE_STRIP;
    Ll_marker.action = Marker::ADD;
    Ll_marker.scale.x = 0.15;
    Ll_marker.color.r = 0.0f;
    Ll_marker.color.g = 0.0f;
    Ll_marker.color.b = 1.0f;
    Ll_marker.color.a = 1.0f;

    Ll_marker.points.clear();
    // 左边界
    for (size_t i=0; i<x_local_map_set.size(); i++){
        geometry_msgs::msg::Point p;
        p.x = x_local_map_set.at(i) + start_point_x;
        p.y = Ll_set.at(i);
        p.z = 0.1;
        Ll_marker.points.push_back(p);
    }
    traj_rviz_pub_->publish(Ll_marker);

    // 右边界
    Marker Lr_marker;
    Lr_marker.header.frame_id = "map";
    Lr_marker.header.stamp = now();
    Lr_marker.ns = "Lr_set";
    Lr_marker.id = 3;
    Lr_marker.type = Marker::LINE_STRIP;
    Lr_marker.action = Marker::ADD;
    Lr_marker.scale.x = 0.15;
    Lr_marker.color.r = 0.0f;
    Lr_marker.color.g = 0.0f;
    Lr_marker.color.b = 1.0f;
    Lr_marker.color.a = 1.0f;
    
    for (size_t i=0; i<x_local_map_set.size(); i++){
        geometry_msgs::msg::Point p;
        p.x = x_local_map_set.at(i) + start_point_x;
        p.y = Lr_set.at(i);
        p.z = 0.1;
        Lr_marker.points.push_back(p);
    }

    traj_rviz_pub_->publish(Lr_marker);
}

std::vector<Point2D> CrabWalking_Planning::calc_XYplot(std::vector<double> opt_traj, double x_start, double x_end, double sample_point_num_pm)
{
    std::vector<Point2D> xy_plot;
    uint32_t sample_point_num = static_cast<uint32_t>((x_end - x_start) * sample_point_num_pm);
    double sample_interval = (x_end - x_start)/(double)(sample_point_num - 1);
    for(size_t i=0; i<sample_point_num; i++)
    {
        Point2D p;
        double y;
        double vy;
        double x = x_start + (double)i*sample_interval;
        path_planning->calcPosy_Vy(opt_traj, x, y, vy);
        p.x = x;
        p.y = y;
        xy_plot.push_back(p);
    }
    return xy_plot;
}

void CrabWalking_Planning::lane_info_init()
{
    planning_config_->read_lane_config();
    lane_info.lane_begin_x = planning_config_->lane().lane_begin_x;
    lane_info.lane_begin_y = planning_config_->lane().lane_begin_y;
    lane_info.lane_end_x = planning_config_->lane().lane_end_x;
    lane_info.lane_end_y = planning_config_->lane().lane_end_y;
    lane_info.lane_width = planning_config_->lane().lane_width;
    lane_info.lane_num = planning_config_->lane().lane_num;
    lane_info.lane_change_start_x = planning_config_->lane().lane_change_start_x;
    lane_info.lane_change_end_x = planning_config_->lane().lane_change_end_x;

    lane_vis_update();

    RCLCPP_INFO(this->get_logger(), "lane info initialized.");
}

void CrabWalking_Planning::static_obstacles_info_init()
{
    planning_config_->read_obstacle_config();
    for(size_t i=0; i<planning_config_->obs_s().size(); i++)
    {
        Object_Info obs_tmp;
        obs_tmp.obs_id = planning_config_->obs_s().at(i).id_;
        obs_tmp.obs_length = planning_config_->obs_s().at(i).length_;
        obs_tmp.obs_width = planning_config_->obs_s().at(i).width_;
        obs_tmp.obs_global_pos_x = planning_config_->obs_s().at(i).pose_x_;
        obs_tmp.obs_global_pos_y = planning_config_->obs_s().at(i).pose_y_;
        obs_tmp.obs_global_heading_angle = planning_config_->obs_s().at(i).pose_theta_;

        // 计算出矩形框的四个顶点的位置
        obs_tmp.edge_points.clear();
        obs_tmp.edge_points = calc_static_obs_edge_points(obs_tmp);
        objects_info.push_back(obs_tmp);
    }

    RCLCPP_INFO(this->get_logger(), "static obstacle initialized.");
}

std::vector<Point2D> CrabWalking_Planning::calc_static_obs_edge_points(const Object_Info &obs_tmp)
{
    // 计算出矩形框的四个顶点的位置
    double xo = obs_tmp.obs_global_pos_x;
    double yo = obs_tmp.obs_global_pos_y;
    double psi = obs_tmp.obs_global_heading_angle /180.0 * M_PI;
    double half_L = obs_tmp.obs_length/2.0;
    double half_W = obs_tmp.obs_width/2.0;

    std::vector<Point2D> edge_points;
    double eps_x1 = std::cos(psi)*(0-half_L) - std::sin(psi)*(0-half_W) + xo;
    double eps_y1 = std::sin(psi)*(0-half_L) + std::cos(psi)*(0-half_W) + yo;
    edge_points.push_back({eps_x1, eps_y1});

    double eps_x2 = std::cos(psi)*half_L - std::sin(psi)*(0-half_W) + xo;
    double eps_y2 = std::sin(psi)*half_L + std::cos(psi)*(0-half_W) + yo;
    edge_points.push_back({eps_x2, eps_y2});

    double eps_x3 = std::cos(psi)*half_L - std::sin(psi)*half_W + xo;
    double eps_y3 = std::sin(psi)*half_L + std::cos(psi)*half_W + yo;
    edge_points.push_back({eps_x3, eps_y3});

    double eps_x4 = std::cos(psi)*(0-half_L) - std::sin(psi)*half_W + xo;
    double eps_y4 = std::sin(psi)*(0-half_L) + std::cos(psi)*half_W + yo;
    edge_points.push_back({eps_x4, eps_y4});

    return edge_points;
}

void CrabWalking_Planning::moving_obstacle_info_init()
{
    // 初始化移动障碍物
    planning_config_->read_moving_obstacle_config();
    moving_object_info.obs_id = planning_config_->moving_obs().id_;
    moving_object_info.obs_length = planning_config_->moving_obs().length_;
    moving_object_info.obs_width = planning_config_->moving_obs().width_;
    moving_object_info.obs_global_pos_x = planning_config_->moving_obs().pose_x_;
    moving_object_info.obs_global_pos_y = planning_config_->moving_obs().pose_y_;
    moving_object_info.obs_global_heading_angle = planning_config_->moving_obs().pose_theta_;
    moving_object_info.obs_velocity = planning_config_->moving_obs().speed_ori_;

    moving_obstacle_vis_update();

    RCLCPP_INFO(this->get_logger(), "dynamic obstacle initialized.");
}

std::vector<Point2D> CrabWalking_Planning::moving_obs_edge_points_update(const Object_Info &moving_object_info)
{
    // 定义超车车速，当局部地图内发现障碍车，开始超车，纵向上vx_ref设置为超车车速
    // 根据自车车速（包括加速到vx_ref时间）和障碍物车速，计算自车和障碍车交互的时间和位置
    // 交互点选择为自车的后端和障碍车的前端重合时刻
    // 如果在交互点之前，则障碍物被扩展到交互点位置；如果已经通过交互点，则障碍物在原位置
    if(!ego_state_ptr_){
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No localization data");
        return {};
    }

    double ego_pos_x = ego_info.ego_pos_x;
    double ego_pos_y = ego_info.ego_pos_y;
    double ego_rear_x = ego_pos_x - ego_info.ego_length / 2.0;
    double obs_pos_x = dynamic_obstacle_pos_x;
    double obs_front_x = obs_pos_x + moving_object_info.obs_length / 2.0;
    double obs_rear_x  = obs_pos_x - moving_object_info.obs_length / 2.0;
    double lm_forward_x = ego_pos_x + planning_para.local_map_forward_x;
    double lm_backward_x = ego_pos_x - planning_para.local_map_backward_x;

    std::cout << "ego_pos_xy:" << ego_pos_x << ", " << ego_pos_y  << "    obs_pos_xy:" << obs_pos_x << ", 0.65" << std::endl; 

    // 障碍物不在局部地图范围内，直接返回空
    if((obs_front_x <= lm_backward_x) || (obs_rear_x >= lm_forward_x))
        return {};

    const double overtake_speed = 6.0; 
    // 障碍物速度大于等于自车参考速度，不进行超车，直接返回空
    if(moving_object_info.obs_velocity >= overtake_speed)
        return {};
    
    // 已经超过交互点，返回障碍物在局部地图内的边界点
    if(ego_rear_x >= obs_front_x)
    {   
        // 超车速度重新设置为参考速度
        // vx_ref = planning_para.vx_ref;

        if((obs_rear_x >= lm_backward_x) && (obs_front_x <= lm_forward_x))
        {
            return calc_static_obs_edge_points(moving_object_info);
        } else if((obs_rear_x < lm_backward_x) && (obs_front_x > lm_backward_x) && (obs_front_x <= lm_forward_x))
        {
            // 障碍物后端超出局部地图起点
            Object_Info obs_tmp = moving_object_info;
            obs_tmp.obs_length = obs_front_x - lm_backward_x;
            obs_tmp.obs_global_pos_x = lm_backward_x + obs_tmp.obs_length / 2.0;
            return calc_static_obs_edge_points(obs_tmp);
        }

        RCLCPP_ERROR(this->get_logger(), "moving obstacle position error in moving_obs_edge_points_update.");
        return {};
    }

    // vx_ref = overtake_speed;
    
    double dv = vx_ref - ego_info.ego_pos_vx;
    double t_acc = (dv > 0.0) ? dv / ax_drive : 0.0;   // 加速时间
    double s_acc = ego_info.ego_pos_vx * t_acc + 0.5 * ax_drive * t_acc * t_acc; // 加速段行驶距离

    // 判断加速段是否已经通过交互点
    double relative_x = (ego_rear_x + s_acc) - (obs_front_x + moving_object_info.obs_velocity * t_acc);
    double relative_v = vx_ref - moving_object_info.obs_velocity;
    double t_interact = 0.0;
    double x_interact = 0.0;

    if(relative_x < 0.0)
    {
        // 未到达交互点，计算交互点位置
        t_interact = -relative_x / relative_v + t_acc;
        x_interact = obs_front_x + moving_object_info.obs_velocity * t_interact;
    }
    else
    {
        // 加速阶段已经超越交互点，则计算交互点位置
        double dv = ego_info.ego_pos_vx - moving_object_info.obs_velocity;
        t_interact = (std::sqrt(dv*dv - 2.0*ax_drive*(ego_rear_x - obs_front_x)) - dv) / ax_drive;
        x_interact = obs_front_x + moving_object_info.obs_velocity * t_interact;
    }
    // 根据交互点位置，扩展障碍物位置
    Object_Info obs_tmp = moving_object_info;
    obs_tmp.obs_length = x_interact - obs_rear_x;
    obs_tmp.obs_global_pos_x = obs_rear_x + obs_tmp.obs_length / 2.0;
    return calc_static_obs_edge_points(obs_tmp);
}

void CrabWalking_Planning::ego_info_init()
{
    planning_config_->read_vehicle_config();
    ego_info.ego_length = planning_config_->vehicle().ego_length;
    ego_info.ego_width = planning_config_->vehicle().ego_width;
    ego_info.steer_angle_max = planning_config_->vehicle().steer_angle_max;
    ego_info.steer_angular_vel_max = planning_config_->vehicle().steer_angular_vel_max;

    RCLCPP_INFO(this->get_logger(), "ego info initialized.");
}

void CrabWalking_Planning::planning_para_init()
{
    vel_planning = std::make_unique<CB_Velocity_Planning>();
    path_planning = std::make_unique<CB_Path_Planning>();

    planning_config_->read_planning_config();
    planning_para.Min_Collision_free_distance = planning_config_->planning().Min_Collision_free_distance;
    planning_para.dest_x = planning_config_->planning().dest_x;
    planning_para.lane_sampling_begin_y = planning_config_->planning().lane_sampling_begin_y;
    planning_para.lane_sampling_end_y = planning_config_->planning().lane_sampling_end_y;
    planning_para.lane_y_sampling_interval = planning_config_->planning().lane_y_sampling_interval;
    planning_para.lane_x_sampling_interval = planning_config_->planning().lane_x_sampling_interval;
    planning_para.local_map_forward_x = planning_config_->planning().local_map_forward_x;
    planning_para.local_map_backward_x = planning_config_->planning().local_map_backward_x;
    planning_para.vx_ref = planning_config_->planning().vx_ref;
    planning_para.ax_drive = planning_config_->planning().ax_drive;
    planning_para.ax_brake = planning_config_->planning().ax_brake;

    auto path_planning_param = std::make_shared<Param>();
    path_planning_param->ego_length = ego_info.ego_length;
    path_planning_param->ego_width = ego_info.ego_width;
    path_planning_param->local_map_length = planning_para.local_map_forward_x;
    path_planning_param->lane_begin_x = lane_info.lane_begin_x;
    path_planning_param->lane_end_x = lane_info.lane_end_x;
    path_planning_param->lane_begin_y = lane_info.lane_begin_y;
    path_planning_param->lane_end_y = lane_info.lane_end_y;
    path_planning_param->lane_width = lane_info.lane_width;
    path_planning_param->lane_sampling_begin_y = planning_para.lane_sampling_begin_y;
    path_planning_param->lane_sampling_end_y = planning_para.lane_sampling_end_y;
    path_planning_param->lane_sampling_interval = planning_para.lane_y_sampling_interval;
    path_planning_param->lane_num = lane_info.lane_num;
    path_planning->setParameter(path_planning_param);

    // 纵向规划参数初始化
    vx_ref = planning_para.vx_ref;
    ax_drive = planning_para.ax_drive;
    ax_brake = planning_para.ax_brake;

    vx_plan = 0;
    isbraking = false;
    t_interval = 0.1;
    plot_cnt = 0;

    // 对重规划变量进行初始化
    opt_traj_prev.resize(11);
    opt_traj_inreplanning.resize(11);
    double inf = std::numeric_limits<double>::infinity();
    for(size_t i=0; i<opt_traj_prev.size(); i++){
        opt_traj_prev.at(i) = inf;
        opt_traj_inreplanning.at(i) = {inf, inf};
    }

    dynamic_obstacle_pos_x = moving_object_info.obs_global_pos_x;
    dynamic_obstacle_pos_y = moving_object_info.obs_global_pos_y;
    is_moving = false;

    is_first_plan = true;

    RCLCPP_INFO(this->get_logger(), "parameters initialized.");
}

void CrabWalking_Planning::generate_reference_path()
{
    // 从起点开始
    double ref_path_x = lane_info.lane_begin_x;
    double start_y = lane_info.lane_begin_y + lane_info.lane_width * 0.5;
    double end_y = lane_info.lane_begin_y + lane_info.lane_width * 1.5;
    double x_interval = (lane_info.lane_change_end_x - lane_info.lane_change_start_x)/planning_para.lane_x_sampling_interval;
    double y_interval = (end_y - start_y) / x_interval;
    while(ref_path_x <= planning_para.dest_x)
    {
        Point2D point_tmp;
        point_tmp.x = ref_path_x;
        if(ref_path_x <= lane_info.lane_change_start_x)
        {
            point_tmp.y = start_y;
        }
        else if(ref_path_x >= lane_info.lane_change_end_x)
        {
            point_tmp.y = end_y;    
        }
        else if(ref_path_x > lane_info.lane_change_start_x && ref_path_x < lane_info.lane_change_end_x)
        {
            point_tmp.y = start_y + (ref_path_x - lane_info.lane_change_start_x)/planning_para.lane_x_sampling_interval * y_interval;
        }
        if(Enable_overtaking_scenario)
            point_tmp.y = start_y; // 直线参考路径
        global_reference_path.push_back(point_tmp);

        ref_path_x += planning_para.lane_x_sampling_interval;
    }

    // 可视化参考路径
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "global_path";
    marker.id = 4;
    marker.type = marker.LINE_STRIP;
    marker.action = marker.ADD;
    marker.scale.x = 0.2;
    marker.color.r = 0.95f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    for(size_t i=0; i<global_reference_path.size(); i++){
        geometry_msgs::msg::Point p;
        p.x = global_reference_path.at(i).x;
        p.y = global_reference_path.at(i).y;
        p.z = 0.1;
        marker.points.push_back(p);
    }

    traj_rviz_pub_->publish(marker);
}

void CrabWalking_Planning::srv_upper_test_callback(const srvUpperTest::Request::SharedPtr request, srvUpperTest::Response::SharedPtr response)
{
    upper_test.index = request->index;
    upper_test.enable = request->enable;
    upper_test.is_new = request->is_new;
    upper_test.bool_cmd = request->bool_cmd;
    upper_test.int_cmd = request->int_cmd;
    upper_test.float_cmd = request->float_cmd;

    msgUpperTest ut;
    ut.header.stamp = now();
    ut.header.frame_id = "sim_sync";
    ut.index = upper_test.index;
    ut.enable = upper_test.enable;
    ut.is_new = upper_test.is_new;
    ut.bool_cmd = upper_test.bool_cmd;
    ut.int_cmd = upper_test.int_cmd;
    ut.float_cmd = upper_test.float_cmd;

    sim_sync_pub_->publish(ut);

    response->result = 0;
}


}