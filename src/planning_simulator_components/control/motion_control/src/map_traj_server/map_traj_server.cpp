#include "map_traj_server.hpp"
#include "interpolation.hpp"
#include "geometry.hpp"
#include "trajectory.hpp"
#include <functional>

namespace map_traj_server
{

MapTrajServer::MapTrajServer(const rclcpp::NodeOptions &options)
    : Node("map_traj_server", options)
{
    //声明参数
    is_map_create_success = false;
    is_traj_create_success = false;

    //创建发布者
    traj_pub_ = this->create_publisher<Trajectory>("output/trajectory", 1);
    traj_rviz_pub_ = this->create_publisher<Marker>("traj_rviz", 10);
    map_rviz_pub_ = this->create_publisher<MarkerArray>("map_rviz", 10);
    vehicle_rviz_pub_ = this->create_publisher<Marker>("vehicle_rviz",10);

    //创建订阅者
    sub_kinematics_ = create_subscription<Odometry>(
        "input/kinematics", 1, [this](const Odometry::ConstSharedPtr msg) { 
            odometry_ = msg; });

    //创建服务
    map_server_ = this->create_service<MapCreatorService>(
        "pnc_map_server",
        std::bind(&MapTrajServer::srv_map_callback, this, _1, _2));
    
    traj_server_ = this->create_service<TrajCreatorService>(
        "global_path_server",
        std::bind(&MapTrajServer::srv_traj_callback, this, _1, _2));

    //创建定时器
    using namespace std::literals::chrono_literals;
    timer_ = rclcpp::create_timer(
        this, get_clock(), 100ms, std::bind(&MapTrajServer::onTimer, this));
}

void MapTrajServer::onTimer()
{
    //判断是否已经成功请求了地图，并在rviz中显示出来
    if(!is_map_create_success)
        return;

    //判断是否已经成功请求了轨迹，并在rviz中显示出来
    if(!is_traj_create_success)
        return;

    //如果以上都完成了，就根据订阅的车辆位置信息，在rviz中显示出来
    if(!odometry_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No localization data");
        return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "map_traj_server";
    marker.id = 0;
    marker.type = marker.CUBE;
    marker.action = marker.ADD;
    marker.pose = odometry_->pose.pose;
    marker.scale.x = 3.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.6;
    marker.color.r = 0.7f;
    marker.color.g = 0.7f;
    marker.color.b = 0.7f;
    marker.color.a = 0.8f;
    vehicle_rviz_pub_->publish(marker);
}

void MapTrajServer::srv_map_callback(const MapCreatorService::Request::SharedPtr request,
                                        MapCreatorService::Response::SharedPtr response)
{
    //构成T字形地图区域，传入请求的点数应该为8
    if(request->pose.size() != 8){
        is_map_create_success = false;
        RCLCPP_ERROR(this->get_logger(), "the size of points is wrong.");
        return;
    }
    
    response->result = 1;
    
    MarkerArray marr;
    int id = 0;
    size_t lane_num = 2;

    // 绘制车道边界线
    for(size_t i =0; i< lane_num+1; i++)
    {
        Marker m;
        m.header.frame_id = "map";
        m.header.stamp    = rclcpp::Clock().now();
        m.ns              = "motion_control";
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
        while (len_tmp <= 100.0 + 10.0)
        {
            p.x = -10.0 + len_tmp;
            p.y = 0.0 + i*1.3;
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
        m.ns              = "motion_control";
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
        while (len_tmp <= 110.0)
        {
            p.x =  -10.0 + len_tmp;
            p.y = 0.0 + i*1.3 + 1.3/2.0;
            p.z = 0.0;
            m.points.push_back(p);
            len_tmp = len_tmp + 1.0;
        }
        if (m.points.size() % 2 == 1)
            m.points.pop_back();
        marr.markers.push_back(m);
    }
    map_rviz_pub_->publish(marr);
    is_map_create_success = true;
}

void MapTrajServer::srv_traj_callback(const TrajCreatorService::Request::SharedPtr request, 
                                        TrajCreatorService::Response::SharedPtr response)
{

    if(!traj_creator(request->trajectory_type, request->is_reverse)) {
        RCLCPP_WARN(this->get_logger(), "Unable to generate trajectory");
        return;
    }

    if(traj_planned.points.empty()) {
        RCLCPP_WARN(this->get_logger(), "The size of trajectory points is zero");
        return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "map_traj_server";
    marker.id = 1;
    marker.type = marker.LINE_STRIP;
    marker.action = marker.ADD;
    marker.scale.x = 0.2;
    marker.color.r = 0.0f;
    marker.color.g = 0.95f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    for (const auto& pt : traj_planned.points) {
        marker.points.emplace_back(pt.pose.position);
    }

    traj_rviz_pub_->publish(marker);
    is_traj_create_success = true;

    response->result = 1;
    response->traj = traj_planned;
}


bool MapTrajServer::traj_creator(size_t type, size_t is_reverse)
{
    switch (type)
    {
    case 0:   //直行
    {
        Point start_point;
        start_point.x = 68.5;
        start_point.y = -54.8;
        start_point.z = 0.0;

        Point end_point;
        end_point.x = 32.8;
        end_point.y = -24.1;
        end_point.z = 0.0;

        const double vel_max = 3.5;
        const double acc = 0.4;

        if(!traj_creator_straight(start_point, end_point, vel_max, acc, is_reverse))
            return false;
    }
        break;
    case 1:   //变道
    {
        Point start_point;
        start_point.x = 10.0;
        start_point.y = 0.0;
        start_point.z = 0.0;

        Point end_point;
        end_point.x = 30.0;
        end_point.y = 1.5;
        end_point.z = 0.0;

        const double vel_max = 1.0;
        const double acc = 0.2;

        if(!traj_creator_lane_change(start_point, end_point, vel_max, acc, is_reverse))
            return false;
    }
        break;
    case 2:   //直角转弯
    {
        Point start_point;
        start_point.x = 20.0;
        start_point.y = -43.0;
        start_point.z = 0.0;

        Point end_point;
        end_point.x = 5.0;
        end_point.y = -28.0;
        end_point.z = 0.0;

        const double vel_max = 2.0;
        const double acc = 1.0;
        const double radius = 5.0;

        if(!traj_creator_elbow_bend(start_point, end_point, vel_max, acc, radius, is_reverse))
            return false;
    }
        break;
    case 3:   //斜行
    {
        Point start_point;
        start_point.x = 0.0;
        start_point.y = 0.0;
        start_point.z = 0.0;

        Point end_point;
        end_point.x = 15.0;
        end_point.y = 2.0;
        end_point.z = 0.0;

        const double vel_max = 1.5;
        const double acc = 0.4;
        const double rotate_angle = M_PI*3.05/4.0;
        const double bias_x = 70.0;
        const double bias_y = -52.5;

        if(!traj_creator_crabwalking(start_point, end_point, vel_max, acc, rotate_angle, bias_x, bias_y, is_reverse))
            return false;
    }
        break;
    case 4:    
    {
        traj_creator_from_txt("/home/mash/agv_8wheel_control2/src/motion_control/src/map_traj_server/load.txt");
    }
        break;
    case 5:
    {
        Point start_point;
        start_point.x = 0.0;
        start_point.y = 0.0;
        start_point.z = 0.0;

        Point end_point;
        end_point.x = 8.0;
        end_point.y = 8.0;
        end_point.z = 0.0;

        const double vel_max = 1.0;
        const double acc = 0.5;
        const double radius = 6.5;

        const double rotate_angle = M_PI/4.0;
        const double bias_x = 58.9;
        const double bias_y = -58.1;

        if(!traj_creator_turn_optimize(start_point, end_point, vel_max, acc, radius, rotate_angle, bias_x, bias_y, is_reverse))
            return false;
    }
        break;
    default:
        break;
    }
    return true;
}

//确定起点、终点坐标、最大速度、加速度，间隔为0.1s计算一次
bool MapTrajServer::traj_creator_straight(Point & sp, Point & dp, double vel_max, double acc, size_t is_reverse)
{
    //计算航向角
    const double L_threshold = 5.0;
    const double dt = 0.1;

    double dx = dp.x - sp.x;
    double dy = dp.y - sp.y;
    double L = std::hypot(dx, dy);
    
    if(L < L_threshold) 
    {
        RCLCPP_WARN(this->get_logger(), "The distance of start point and destination point is smaller than %lf m", L_threshold);
        return false;
    }

    double dir_x = dx / L;
    double dir_y = dy / L;
    
    double yaw = std::atan2(dy, dx);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    Trajectory traj_forward;
    traj_planned.points.clear();

    // 判断是否存在匀速段
    // 计算达到 vel_max 需要的加速距离
    double sa_needed = 0.5 * vel_max * vel_max / acc;
    if(2.0 * sa_needed <= L){
        /*梯形： 加速 + 匀速 + 减速*/
        double t_acc = vel_max / acc;
        double s_cruise = L - 2.0 * sa_needed;
        double t_cruise = s_cruise / vel_max;
        double t_total = 2.0 * t_acc + t_cruise;

        /*采样*/
        for(double t=0.0; t <= t_total + 1e-3; t += dt){
            double s, v;
            if(t < t_acc) {
                s = 0.5 * acc * t * t;
                v = std::max(0.1, acc * t);
            } else if (t < t_acc + t_cruise){
                double t_c = t - t_acc;
                s = sa_needed + vel_max * t_c;
                v = vel_max;
            } else {
                double t_dec = t - t_acc - t_cruise;
                s = sa_needed + s_cruise + vel_max*t_dec - 0.5*acc*t_dec*t_dec;
                v = std::max(0.0, vel_max - acc*t_dec);
            }

            TrajectoryPoint tp;
            tp.pose.position.x = sp.x + dir_x * s;
            tp.pose.position.y = sp.y + dir_y * s;
            tp.pose.position.z = 0.0;
            tp.longitudinal_velocity_mps = v;
            tp.pose.orientation = tf2::toMsg(q);
            traj_forward.points.emplace_back(tp);
        }
    } else {
        /* 三角形：无匀速段 */
        double v_real = std::sqrt(acc * L);
        double t_acc = v_real / acc;
        double t_total = 2.0 * t_acc;

        for(double t=0.0; t <= t_total + 1e-3; t += dt){
            double s, v;
            if(t < t_acc) {
                s = 0.5 * acc * t * t;
                v = std::max(0.1, acc * t);
            } else {
                double t_dec = t - t_acc;
                s = 0.5 * acc * t_acc * t_acc + v_real * t_dec - 0.5 * acc * t_dec * t_dec;
                v = std::max(0.0, v_real - acc*t_dec);
            }

            TrajectoryPoint tp;
            tp.pose.position.x = sp.x + dir_x * s;
            tp.pose.position.y = sp.y + dir_y * s;
            tp.pose.position.z = 0.0;
            tp.longitudinal_velocity_mps = v;
            tp.pose.orientation = tf2::toMsg(q);
            traj_forward.points.emplace_back(tp);
        }
    }

    if(is_reverse)
    {
        // 反向行驶时，坐标需要进行镜像变换
        size_t num_points = traj_forward.points.size();
        for(size_t i=0; i< num_points; ++i){
            TrajectoryPoint & point = traj_forward.points.at(num_points-1-i);
            
            // 朝向也需要调整
            double yaw = tf2::getYaw(point.pose.orientation);
            yaw = std::atan2(std::sin(yaw), std::cos(yaw)); // 归一化
            yaw += M_PI; // 反向旋转180度
            yaw = std::atan2(std::sin(yaw), std::cos(yaw)); // 再次归一化

            point.pose.orientation = tf2::toMsg(tf2::Quaternion(
                tf2::Vector3(0, 0, 1), yaw));
            
            // 横向速度取反
            point.lateral_velocity_mps = -point.lateral_velocity_mps;

            std::cout << "ind" << i <<", x:" << point.pose.position.x << ", y:" << point.pose.position.y <<
                ", vx:" << point.longitudinal_velocity_mps << ", dy:" << point.lateral_velocity_mps << std::endl; 
            traj_planned.points.emplace_back(point);
        }
    }else
    {
        traj_planned = traj_forward;
    }
    
    RCLCPP_INFO(this->get_logger(), "Straight trajectory: %zu pts, L = %.2f m, v_max_achieved=%.2f m/s",
                traj_planned.points.size(), L, (2.0 * sa_needed <= L ? vel_max : std::sqrt(acc * L)));
    
    return true;
}

bool MapTrajServer::traj_creator_elbow_bend(Point & sp, Point & dp, double vel_max, double acc, double radius, size_t is_reverse)
{
    const double dt = 0.1;
    const double radius_thr = 4.0; 
    if(radius <= radius_thr) {
        RCLCPP_WARN(this->get_logger(), "The radius is smaller than %lf m", radius_thr);
        return false;
    }

    // 1. 几何检查
    double dx = dp.x - sp.x;
    double dy = dp.y - sp.y;
    if(std::abs(dx) <= radius || std::abs(dy) <= radius){
        RCLCPP_WARN(this->get_logger(), "dx or dy < radius, cannot form elbow");
        return false;
    }

    //圆弧中心始终位于内侧，即与dx,dy符号相反方向
    //20250912,zhangyue
    double cx, cy;
    if(is_reverse == 0){
        cx = dp.x + (dx > 0 ? -radius : radius);
        cy = sp.y + (dy > 0 ? radius : -radius);
    } else {
        cx = sp.x + (dx < 0 ? -radius : radius);
        cy = dp.y + (dy < 0 ? radius : -radius);        
    }


    // 2. 三段长度
    double L1 = std::abs(dx) - radius;
    double L_arc = 0.5 * M_PI * radius;
    double L2 = std::abs(dy) - radius;
    double L_total = L1 + L_arc + L2;

    // 3. 梯形、三角形判断
    double sa_needed = 0.5 * vel_max * vel_max / acc;            //单边加速距离
    bool has_cruise = (2.0 * sa_needed <= L_total);

    double v_real, t_acc, t_cruise, t_total;
    if(has_cruise){
        v_real = vel_max;
        t_acc = vel_max / acc;
        t_cruise = (L_total - 2.0 * sa_needed) / vel_max;
        t_total = 2.0 * t_acc + t_cruise;
    } else {
        v_real = std::sqrt(acc * L_total);
        t_acc = v_real / acc;
        t_cruise = 0.0;
        t_total = 2.0 * t_acc;
    }

    // 4. 采样
    traj_planned.points.clear();
    
    //方向向量，保证起点、终点方向正确
    // int dir_x = (dx>0 ? 1 : -1);
    // int dir_y = (dy>0 ? 1 : -1);

    for(double t = 0.0; t <= t_total + 1e-3; t += dt) {
        double s, v;
        if(has_cruise) {
            if(t < t_acc) {
                s = 0.5 * acc * t * t;
                v = std::max(0.1, acc * t);
            } else if(t < t_acc + t_cruise) {
                double t_c = t - t_acc;
                s = sa_needed + v_real * t_c;
                v = v_real;
            } else {
                double t_dec = t - t_acc - t_cruise;
                s = sa_needed + v_real * t_cruise + v_real * t_dec - 0.5 * acc * t_dec * t_dec;
                v = std::max(0.0, v_real - acc * t_dec);
            }
        } else {
            if(t < t_acc) {
                s = 0.5 * acc * t * t;
                v = std::max(0.1, acc * t);
            } else {
                double t_dec = t - t_acc;
                s = 0.5 * acc * t_acc * t_acc + v_real * t_dec - 0.5 * acc * t_dec * t_dec;
                v = std::max(0.0, v_real - acc * t_dec);
            }
        }

        // 位姿插值
        Point pos;

        if(is_reverse == 0) {
            if (s <= L1) {
                pos.x = sp.x - s;
                pos.y = sp.y ;
            } else if (s <= L1 + L_arc) {
                double theta = (s - L1) /radius;
                // pos.x = cx + radius * std::sin(theta);
                // pos.y = cy - radius * std::cos(theta);
                pos.x = cx - radius * std::sin(theta);
                pos.y = cy - radius * std::cos(theta);

            } else {
                double s2 = s - L1 - L_arc;
                pos.x = dp.x;
                pos.y = cy + s2;
            }
        } else {
            //反向
            if (s <= L1) {
                pos.x = sp.x;
                pos.y = sp.y - s;
            } else if (s <= L1 + L_arc) {
                double theta = (s - L1) /radius;
                pos.x = cx + radius * std::cos(theta);
                pos.y = cy - radius * std::sin(theta);
            } else {
                double s2 = s - L1 - L_arc;
                pos.x = cx - s2;
                pos.y = dp.y;
            }
        }

        autoware_planning_msgs::msg::TrajectoryPoint tp;
        tp.pose.position.x = pos.x;
        tp.pose.position.y = pos.y;
        tp.pose.position.z = 0.0;
        tp.longitudinal_velocity_mps = v;
        traj_planned.points.emplace_back(tp);
    }

    for (size_t i = 0; i < traj_planned.points.size() - 1; ++i) {
        const auto & src_point = traj_planned.points.at(i).pose.position;
        const auto & dst_point = traj_planned.points.at(i+1).pose.position;
        const double pitch = control_utils::geometry::calcElevationAngle(src_point, dst_point);
        const double yaw = control_utils::geometry::calcAzimuthAngle(src_point, dst_point);
        control_utils::geometry::setOrientation(
            control_utils::geometry::createQuaternionFromRPY(0.0, pitch, yaw), traj_planned.points.at(i).pose);
        if (i == traj_planned.points.size() - 2) {
        // Terminal orientation is same as the point before it
            control_utils::geometry::setOrientation(traj_planned.points.at(i).pose.orientation, traj_planned.points.at(i + 1).pose);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Universal elbow: %zu pts, L=%.2f m, v_achieve=%.2f m/s",
                traj_planned.points.size(), L_total, v_real);
    
    return true;
}

bool MapTrajServer::traj_creator_lane_change(Point & sp, Point & dp, double vel_target, double acc, size_t is_reverse)
{
    const double dt = 0.1;
    const double W_thr = 1.0;

    double dy = dp.y - sp.y;
    double W = std::fabs(dy);
    if(W < W_thr){
        RCLCPP_WARN(this->get_logger(), "The lateral offset is smaller than %lf m", W_thr);
        return false;
    }

    double dx = dp.x - sp.x;
    double L_total = std::fabs(dx);
    
    /* 梯形速度规划 */
    double sa_needed = 0.5 * vel_target * vel_target / acc;
    bool has_cruise = 2.0 * sa_needed <= L_total;
    if(!has_cruise) {
        RCLCPP_WARN(this->get_logger(), "Lane-change requires L >= %.2f m", 2.0 * sa_needed);
        return false;
    }

    double v_real, t_acc, t_cruise, t_total;
    if(has_cruise) {
        v_real = vel_target;
        t_acc = vel_target / acc;
        t_cruise = (L_total - 2.0 * sa_needed) / vel_target;
        t_total = 2.0 * t_acc + t_cruise;
    } else {
        v_real = std::sqrt(acc * L_total);
        t_acc = v_real / acc;
        t_cruise = 0.0;
        t_total = 2.0 * t_acc;
    }

    /* 采样 */
    traj_planned.points.clear();
    int steps = static_cast<int>(t_total / dt) + 1;
    int acc_steps = static_cast<int>(t_acc / dt) + 1;
    int cruise_end = has_cruise ? acc_steps + static_cast<int>(t_cruise / dt) : acc_steps;

    for(int i=0; i <= steps; i++){
        double t = i * dt;

        /* 纵向距离s 与速度 v*/
        double s,v;
        if(i < acc_steps) {
            /* 加速段 */
            s = 0.5 * acc * t * t;
            v = std::max(0.1, acc * t);
        } else if (i < cruise_end) {
            /* 匀速段 */
            double t_c = t - t_acc;
            s = sa_needed + v_real * t_c;
            v = v_real;
        } else {
            /* 减速段 */
            double t_dec = t - (t_total - t_acc);
            s = L_total - (0.5 * acc * (t_acc - t_dec)*(t_acc - t_dec));
            v = std::max(0.0, v_real - acc*t_dec);
        }

        double y = sp.y;
        const double t_bias = 4.0;
        /* 横向五次多项式（仅匀速段生效，其余为0）*/
        if(t < t_acc + t_bias){
            y = sp.y;
        } 
        else if (t >= t_acc + t_bias && t <= t_acc + t_cruise - t_bias) {
            double t_norm = (t - t_acc - t_bias) / (t_cruise - 2*t_bias);
            double poly = 10.0 * std::pow(t_norm, 3) - 
                          15.0 * std::pow(t_norm, 4) +
                          6.0  * std::pow(t_norm, 5);
            y = sp.y + dy * poly;
        }
        else {
            y = dp.y;
        }
 
        double dir = 1.0;
        if(is_reverse == 0) 
            dir = 1.0;
        else
            dir = -1.0;


        TrajectoryPoint tp;
        tp.pose.position.x = sp.x + dir*s;
        tp.pose.position.y = y;
        tp.pose.position.z = 0.0;
        tp.longitudinal_velocity_mps = v;
        traj_planned.points.emplace_back(tp);
    }

    for (size_t i = 0; i < traj_planned.points.size() - 1; ++i) {
        const auto & src_point = traj_planned.points.at(i).pose.position;
        const auto & dst_point = traj_planned.points.at(i+1).pose.position;
        const double pitch = control_utils::geometry::calcElevationAngle(src_point, dst_point);
        const double yaw = control_utils::geometry::calcAzimuthAngle(src_point, dst_point);
        control_utils::geometry::setOrientation(
            control_utils::geometry::createQuaternionFromRPY(0.0, pitch, yaw), traj_planned.points.at(i).pose);
        if (i == traj_planned.points.size() - 2) {
        // Terminal orientation is same as the point before it
            control_utils::geometry::setOrientation(traj_planned.points.at(i).pose.orientation, traj_planned.points.at(i + 1).pose);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Phase-limited lane change: %zu pts, L=%.2f m, W=%.2f m", 
                traj_planned.points.size(), L_total, std::fabs(dy));
    return true;
}

bool MapTrajServer::traj_creator_crabwalking(Point & sp, Point & dp, double vel_target, double acc, 
    double rotate_angle, double bias_x, double bias_y, size_t is_reverse)
{
    // 初始化临时变量
    double y0 = sp.y;
    double y1 = dp.y;
    double dy0 = 0.0;
    double dx = dp.x - sp.x;
    double dymax = 30.0/180.0*M_PI;
    double ddymax = 30.0/180.0*M_PI/1.0;
    double ddymin = 0.0;

    double L_total = std::fabs(dx);
    
    /* 速度规划 */
    double sa_needed = 0.5 * vel_target * vel_target / acc + 5.0;
    bool has_cruise = true;
    // if(!has_cruise) {
    //     RCLCPP_WARN(this->get_logger(), "Lane-change requires L >= %.2f m", 2.0 * sa_needed);
    //     return false;
    // }

    double v_real, t_acc, t_cruise, t_total;
    if(has_cruise) {
        v_real = vel_target;
        t_acc = vel_target / acc;
        t_cruise = (L_total + 2.0 * 5.0) / vel_target;
        t_total = 2.0 * t_acc + t_cruise;
    } else {
        v_real = std::sqrt(acc * L_total);
        t_acc = v_real / acc;
        t_cruise = 0.0;
        t_total = 2.0 * t_acc;
    }

    /* 横向规划 */
    double start_point_x = sa_needed;
    double end_point_x = L_total + sa_needed;
    double dx_crab = end_point_x - start_point_x;
    auto crab = crabwalking_traj_generate(y0, y1, dy0, dx_crab, dymax, ddymax, ddymin);
    int state = crab.state;

    if (state == 1 || state == 5)
    {
        RCLCPP_WARN(this->get_logger(), "Unable to generate crabwalking trajectory, state:%d", state);
        return false;
    }

    double ddy = crab.ddy;
    double dx1 = crab.dx1;
    double dx2 = crab.dx2;
    double dx3 = crab.dx3;

    const double L_min = 15.0;
    double dx4 = (dx < L_min) ? (L_min - dx) : 0.0;

    std::vector<double> traj;
    traj.push_back(static_cast<double>(state));
    traj.push_back(start_point_x);
    traj.push_back(sp.y);
    traj.push_back(dy0);
    traj.push_back(ddy);
    traj.push_back(dx1);
    traj.push_back(dx2);
    traj.push_back(dx3);
    traj.push_back(dx4);
    traj.push_back(dx_crab);
    traj.push_back(y1);

    // 将res_path.opt_traj 的11个参数打印出来
    // std::cout << "Optimal trajectory parameters: ";
    // for (const auto& param : traj) {
    //     std::cout << param << " "; 
    // }  
    // std::cout << std::endl;

    /* 采样 */
    const double dt = 0.1;
    traj_planned.points.clear();
    Trajectory traj_forward;
    int steps = static_cast<int>(t_total / dt) + 1;
    int acc_steps = static_cast<int>(t_acc / dt) + 1;
    int cruise_end = has_cruise ? acc_steps + static_cast<int>(t_cruise / dt) : acc_steps;

    for(int i=0; i <= steps; i++){
        double t = i * dt;

        /* 纵向距离s 与速度 v*/
        double s,v;
        if(i < acc_steps) {
            /* 加速段 */
            s = 0.5 * acc * t * t;
            v = std::max(0.1, acc * t);
        } else if (i < cruise_end) {
            /* 匀速段 */
            double t_c = t - t_acc;
            s = sa_needed - 5.0 + v_real * t_c;
            v = v_real;
        } else {
            /* 减速段 */
            double t_dec = t - (t_cruise + t_acc);
            s = sa_needed + L_total + 5.0 + v_real*t_dec - 0.5*acc*t_dec*t_dec;
            v = std::max(0.0, v_real - acc*t_dec);
        }

        double y, dy;
        calcPosy_Vy(traj, s-start_point_x, y, dy);
        
        TrajectoryPoint tp;
        tp.pose.position.x = s;
        tp.pose.position.y = y;
        tp.pose.position.z = 0.0;
        tp.longitudinal_velocity_mps = v;
        tp.lateral_velocity_mps = dy *  v;  //横向速度=横向斜率*纵向速度
        tp.heading_rate_rps = 0.0;

        // 明确设置初始朝向（假设前进方向为0度）
        tf2::Quaternion q_initial;
        q_initial.setRPY(0, 0, 0);  // roll, pitch, yaw都设为0
        tp.pose.orientation = tf2::toMsg(q_initial);

        TrajectoryPoint tp_rotate;
        double cos_theta = std::cos(rotate_angle);
        double sin_theta = std::sin(rotate_angle);
        
        double original_x = tp.pose.position.x;
        double original_y = tp.pose.position.y;
        
        // 旋转变换
        tp_rotate.pose.position.x = original_x * cos_theta - original_y * sin_theta + bias_x;
        tp_rotate.pose.position.y = original_x * sin_theta + original_y * cos_theta + bias_y;
        
        // 处理朝向角（需要从四元数提取yaw）
        double original_yaw = tf2::getYaw(tp.pose.orientation);
        double new_yaw = original_yaw + rotate_angle;
        
        // 归一化到 [-pi, pi]
        new_yaw = std::atan2(std::sin(new_yaw), std::cos(new_yaw));
        
        tp_rotate.pose.orientation = tf2::toMsg(tf2::Quaternion(
            tf2::Vector3(0, 0, 1), new_yaw));
        
        tp_rotate.longitudinal_velocity_mps = v;
        tp_rotate.lateral_velocity_mps = dy *  v;  //横向速度=横向斜率*纵向速度
        tp_rotate.heading_rate_rps = 0.0;

        traj_forward.points.emplace_back(tp_rotate);

        // std::cout << "ind" << i <<", x:" << tp.pose.position.x << ", y:" << tp.pose.position.y << ", dy:" << dy << std::endl; 
    }

    if(is_reverse)
    {
        // 反向行驶时，坐标需要进行镜像变换
        size_t num_points = traj_forward.points.size();
        for(size_t i=0; i< num_points; ++i){
            TrajectoryPoint & point = traj_forward.points.at(num_points-1-i);
            
            // 朝向也需要调整
            double yaw = tf2::getYaw(point.pose.orientation);
            yaw = std::atan2(std::sin(yaw), std::cos(yaw)); // 归一化
            yaw += M_PI; // 反向旋转180度
            yaw = std::atan2(std::sin(yaw), std::cos(yaw)); // 再次归一化

            point.pose.orientation = tf2::toMsg(tf2::Quaternion(
                tf2::Vector3(0, 0, 1), yaw));
            
            // 横向速度取反
            point.lateral_velocity_mps = -point.lateral_velocity_mps;

            std::cout << "ind" << i <<", x:" << point.pose.position.x << ", y:" << point.pose.position.y <<
                ", vx:" << point.longitudinal_velocity_mps << ", dy:" << point.lateral_velocity_mps << std::endl; 
            traj_planned.points.emplace_back(point);
        }
    }else
    {
        traj_planned = traj_forward;
    }
    
     RCLCPP_INFO(this->get_logger(), "Crabwalking: %zu pts, L=%.2f m, v_achieve=%.2f m/s",
                traj_planned.points.size(), L_total, v_real);
    
    return true;
}

void MapTrajServer::calcPosy_Vy(std::vector<double> traj, double ego_pos_x, double &ego_pos_y, double &ego_pos_dy)
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

CrabResult MapTrajServer::crabwalking_traj_generate(double y0, double y1, double dy0, double dx, double dymax, double ddymax, double ddymin)
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

bool MapTrajServer::traj_creator_from_txt(const std::string & filename)
{
    std::vector<double> traj_x;
    std::vector<double> traj_y;
    std::vector<double> traj_vx;
    std::vector<double> traj_vy;

    std::ifstream file(filename);
    if(!file.is_open()){
        std::cerr<<"cannot open the file" << filename << std::endl;
        return false;
    }
    std::string line;
    while(std::getline(file, line)){
        std::istringstream iss(line);
        double x,y,vx,vy;
        if(iss >> x >> y >> vx >> vy){
            traj_x.push_back(x);
            traj_y.push_back(y);
            traj_vx.push_back(vx/2.0);
            traj_vy.push_back(vy/2.0);
        }
    }
    // std::cout << "traj_x_size:" << traj_x.size() << std::endl;
    // for(size_t i=0; i<traj_x.size(); i++)
    // {
    //     std::cout << "x:" << traj_x.at(i) << ";  y:"<<traj_y.at(i)<<";  yaw:"<<traj_yaw.at(i) <<std::endl;
    // }
    file.close();

    for(size_t i=0; i<traj_x.size(); i++)
    {
        TrajectoryPoint tp;
        tp.pose.position.x = traj_x.at(i);
        tp.pose.position.y = traj_y.at(i);
        tp.pose.position.z = 0.0;
        tp.longitudinal_velocity_mps = traj_vx.at(i);
        tp.lateral_velocity_mps = traj_vy.at(i);

        traj_planned.points.push_back(tp);
        control_utils::geometry::setOrientation(
            control_utils::geometry::createQuaternionFromRPY(0.0, 0.0, 0.0), traj_planned.points.at(i).pose);
    }

    return true;
}

// ---------- 自适应 Simpson 积分 ----------
double adaptive_simpson(const std::function<double(double)>& f, double a, double b, double eps, int max_depth) {
    auto simpson = [](const std::function<double(double)>& f, double a, double b) {
        return (b - a) / 6.0 * (f(a) + 4.0 * f((a + b) / 2.0) + f(b));
    };
    double m = (a + b) / 2.0;
    double whole = simpson(f, a, b);
    double left = simpson(f, a, m);
    double right = simpson(f, m, b);
    if (std::abs(left + right - whole) < 15.0 * eps || max_depth <= 0)
        return left + right + (left + right - whole) / 15.0;
    return adaptive_simpson(f, a, m, eps / 2.0, max_depth - 1) +
           adaptive_simpson(f, m, b, eps / 2.0, max_depth - 1);
}

// 菲涅尔积分 C(x) = ∫₀ˣ cos(π/2 * t²) dt
double fresnelc(double x) {
    if (x == 0.0) return 0.0;
    auto f = [](double t) { return std::cos(M_PI / 2.0 * t * t); };
    return adaptive_simpson(f, 0.0, x, 1e-9, 20);
}

// 菲涅尔积分 S(x) = ∫₀ˣ sin(π/2 * t²) dt
double fresnels(double x) {
    if (x == 0.0) return 0.0;
    auto f = [](double t) { return std::sin(M_PI / 2.0 * t * t); };
    return adaptive_simpson(f, 0.0, x, 1e-9, 20);
}

// ---------- 回旋线单点计算 ----------
void clothoid_point(double km, double L1, double s, double& x, double& y, double& theta) {
    double A = std::sqrt(L1 / km);                 // 回旋参数
    double u = s / (A * std::sqrt(M_PI));
    double C = fresnelc(u);
    double S = fresnels(u);
    x = A * std::sqrt(M_PI) * C;
    y = A * std::sqrt(M_PI) * S;
    theta = (km / (2.0 * L1)) * s * s;
}

// 第三段回旋线局部偏移（数值积分）
void clothoid_local_offset(double km, double L1, double s, double& dx, double& dy) {
    if (s == 0.0) {
        dx = 0.0;
        dy = 0.0;
        return;
    }
    auto cos_theta = [km, L1](double u) {
        double theta_off = km * u - (km / (2.0 * L1)) * u * u;
        return std::cos(theta_off);
    };
    auto sin_theta = [km, L1](double u) {
        double theta_off = km * u - (km / (2.0 * L1)) * u * u;
        return std::sin(theta_off);
    };
    dx = adaptive_simpson(cos_theta, 0.0, s, 1e-9, 20);
    dy = adaptive_simpson(sin_theta, 0.0, s, 1e-9, 20);
}

// 计算终点 x 坐标（用于求解 L1）
double compute_endpoint_x(double km, double L1) {
    double R = 1.0 / km;
    double L2 = M_PI / (2.0 * km) - L1;
    if (L2 <= 0.0) return NAN;
    double x1, y1, theta1;
    clothoid_point(km, L1, L1, x1, y1, theta1);
    double theta_arc = km * L2;
    double Ox = x1 - R * std::sin(theta1);
    double Oy = y1 + R * std::cos(theta1);
    double x2 = Ox + R * std::sin(theta1 + theta_arc);
    double y2 = Oy - R * std::cos(theta1 + theta_arc);
    double theta2 = theta1 + theta_arc;
    double dx_local, dy_local;
    clothoid_local_offset(km, L1, L1, dx_local, dy_local);
    return x2 + dx_local * std::cos(theta2) - dy_local * std::sin(theta2);
}

// 计算终点 y 坐标（验证用）
double compute_endpoint_y(double km, double L1) {
    double R = 1.0 / km;
    double L2 = M_PI / (2.0 * km) - L1;
    if (L2 <= 0.0) return NAN;
    double x1, y1, theta1;
    clothoid_point(km, L1, L1, x1, y1, theta1);
    double theta_arc = km * L2;
    double Ox = x1 - R * std::sin(theta1);
    double Oy = y1 + R * std::cos(theta1);
    double x2 = Ox + R * std::sin(theta1 + theta_arc);
    double y2 = Oy - R * std::cos(theta1 + theta_arc);
    double theta2 = theta1 + theta_arc;
    double dx_local, dy_local;
    clothoid_local_offset(km, L1, L1, dx_local, dy_local);
    return y2 + dx_local * std::sin(theta2) + dy_local * std::cos(theta2);
}

// 求解 L1, L2
void solve_clothoid_arc(double km, double dx, double dy, double& L1, double& L2) {
    double L1_low = 1e-6;
    double L1_high = M_PI / (2.0 * km) - 1e-6;
    // 使用二分法求解
    auto eqn = [km, dx](double L1) { return compute_endpoint_x(km, L1) - dx; };
    double L1_sol = (L1_low + L1_high) / 2.0;
    for (int iter = 0; iter < 100; ++iter) {
        double fmid = eqn(L1_sol);
        if (std::abs(fmid) < 1e-9) break;
        if (eqn(L1_low) * fmid < 0)
            L1_high = L1_sol;
        else
            L1_low = L1_sol;
        L1_sol = (L1_low + L1_high) / 2.0;
    }
    L1 = L1_sol;
    L2 = M_PI / (2.0 * km) - L1;
    double Ye = compute_endpoint_y(km, L1);
    std::cout << "求解结果： L1 = " << L1 << " m, L2 = " << L2 << " m\n";
    std::cout << "终点 y 坐标计算值 = " << Ye << ", 期望值 = " << dy << ", 误差 = " << Ye - dy << "\n";
}

// 直角弯生成函数
bool MapTrajServer::traj_creator_turn_optimize(Point &sp, Point &dp, double vx, double acc, double R, double rotate_angle, 
    double bias_x, double bias_y, size_t is_reverse)
{

    // 
    int pts_num_straight = 150;
    double dx = 15.0/pts_num_straight;
    std::vector<double> x_pts1, y_pts1, yaw_pts1, kappa_pts1,beta_pts1;
    for(size_t i=0; i<pts_num_straight; i++)
    {
        x_pts1.push_back(sp.x - 15.0 + i*dx);
        y_pts1.push_back(sp.y);
        yaw_pts1.push_back(0.0);
        kappa_pts1.push_back(0.0);
        beta_pts1.push_back(0.0);
    }

    double dy = 15.0/pts_num_straight;
    std::vector<double> x_pts2, y_pts2, yaw_pts2, kappa_pts2,beta_pts2;
    for(size_t i=0; i<pts_num_straight; i++)
    {
        x_pts2.push_back(dp.x);
        y_pts2.push_back(dp.y + dy*i);
        yaw_pts2.push_back(M_PI/2.0);
        kappa_pts2.push_back(0.0);
        beta_pts2.push_back(0.0);
    }

    double kappa_max = 1.0 / R;

    // 求解 L1, L2（基于终点坐标 (16,16)）
    double end_point_x = dp.x;
    double end_point_y = dp.y;
    double L1, L2;
    solve_clothoid_arc(kappa_max, end_point_x, end_point_y, L1, L2);
    double dk_ds = kappa_max / L1;

    // 第二组参数（基于航向角变化）
    double kappa_max2 = 1.0 / 4.0;
    double ds = 2.0 * L1 + L2;
    double L1_yaw = ds - M_PI / 2.0 / kappa_max2;
    double L2_yaw = M_PI / kappa_max2 - ds;
    double dk_ds_yaw = kappa_max2 / L1_yaw;

    int pts_num = 100;
    std::vector<double> x_pts, y_pts, yaw_pts, kappa_pts, yaw_ptsc;
    sampling_points_xy(pts_num, L1, L2, dk_ds, kappa_max, x_pts, y_pts);
    sampling_points_yaw(pts_num, L1_yaw, L2_yaw, dk_ds_yaw, kappa_max2, yaw_pts, kappa_pts);
    sampling_points_yaw(pts_num, L1, L2, dk_ds, kappa_max, yaw_ptsc, kappa_pts);  // 只用 yaw_pts2

    // 计算 beta_pts = yaw_pts2 - yaw_pts
    std::vector<double> beta_pts(pts_num);
    for (int i = 0; i < pts_num; ++i) beta_pts[i] = yaw_ptsc[i] - yaw_pts[i];

    // 计算各轮转角
    // double s_sum = 2.0 * L1 + L2;
    // double ds_d = s_sum / pts_num;
    // std::vector<double> s_pts(pts_num);
    // std::vector<std::vector<double>> delta_pts(pts_num, std::vector<double>(4));
    // for (int m = 0; m < pts_num; ++m) {
    //     s_pts[m] = ds_d * (m + 1);
    //     double kappa = kappa_pts[m];
    //     delta_pts[m][0] = beta_pts[m] + std::atan(kappa * wheel_base2 / 2.0);
    //     delta_pts[m][1] = beta_pts[m] + std::atan(kappa * wheel_base1 / 2.0);
    //     delta_pts[m][2] = beta_pts[m] - std::atan(kappa * wheel_base1 / 2.0);
    //     delta_pts[m][3] = beta_pts[m] - std::atan(kappa * wheel_base2 / 2.0);
    // }

    // 生成轨迹点
    traj_planned.points.clear();
    Trajectory traj_tmp;
    int acc_point_num = static_cast<int>((0.5*vx*vx/acc)/dx);
    for(int i=0; i < pts_num_straight; i++)
    {
        TrajectoryPoint tp;
        tp.pose.position.x = x_pts1.at(i);
        tp.pose.position.y = y_pts1.at(i);
        tp.pose.position.z = 0.0;
        if(i<acc_point_num)
        {
            tp.longitudinal_velocity_mps = std::sqrt(2*acc*i*dx);
        }else
        {
            tp.longitudinal_velocity_mps = vx;
        }
        
        tp.lateral_velocity_mps = beta_pts1.at(i) * vx;  // 横向速度=航向角偏差*纵向速度
        tp.heading_rate_rps = vx * kappa_pts1.at(i);  // 航向角变化率=纵向速度*曲率
        control_utils::geometry::setOrientation(
            control_utils::geometry::createQuaternionFromRPY(0.0, 0.0, yaw_pts1.at(i)), tp.pose);
        traj_tmp.points.emplace_back(tp);
    }
    for(int i=0; i < pts_num; i++){
        TrajectoryPoint tp;
        tp.pose.position.x = sp.x + x_pts.at(i);
        tp.pose.position.y = sp.y + y_pts.at(i);
        tp.pose.position.z = 0.0;
        tp.longitudinal_velocity_mps = vx;
        tp.lateral_velocity_mps = beta_pts.at(i) * vx;  // 横向速度=航向角偏差*纵向速度
        tp.heading_rate_rps = vx * kappa_pts.at(i);  // 航向角变化率=纵向速度*曲率
        control_utils::geometry::setOrientation(
            control_utils::geometry::createQuaternionFromRPY(0.0, 0.0, yaw_pts.at(i)), tp.pose);
        traj_tmp.points.emplace_back(tp);
    }
    for(int i=0; i < pts_num_straight; i++)
    {
        TrajectoryPoint tp;
        tp.pose.position.x = x_pts2.at(i);
        tp.pose.position.y = y_pts2.at(i);
        tp.pose.position.z = 0.0;
        if(i>pts_num_straight - acc_point_num)
        {
            tp.longitudinal_velocity_mps = std::sqrt(2*acc*(pts_num_straight - i)*dx);
        }else
        {
            tp.longitudinal_velocity_mps = vx;
        }
        
        tp.lateral_velocity_mps = beta_pts2.at(i) * vx;  // 横向速度=航向角偏差*纵向速度
        tp.heading_rate_rps = vx * kappa_pts2.at(i);  // 航向角变化率=纵向速度*曲率
        control_utils::geometry::setOrientation(
            control_utils::geometry::createQuaternionFromRPY(0.0, 0.0, yaw_pts2.at(i)), tp.pose);
        traj_tmp.points.emplace_back(tp);
    }

    Trajectory traj_forward;  
    for (size_t i=0; i<2*pts_num_straight + pts_num; i++)
    {
        TrajectoryPoint tp_rotate;
        TrajectoryPoint tp = traj_tmp.points.at(i);
        double cos_theta = std::cos(rotate_angle);
        double sin_theta = std::sin(rotate_angle);
        
        double original_x = tp.pose.position.x;
        double original_y = tp.pose.position.y;
        
        // 旋转变换
        tp_rotate.pose.position.x = original_x * cos_theta - original_y * sin_theta + bias_x;
        tp_rotate.pose.position.y = original_x * sin_theta + original_y * cos_theta + bias_y;
        
        // 处理朝向角（需要从四元数提取yaw）
        double original_yaw = tf2::getYaw(tp.pose.orientation);
        double new_yaw = original_yaw + rotate_angle;
        
        // 归一化到 [-pi, pi]
        new_yaw = std::atan2(std::sin(new_yaw), std::cos(new_yaw));
        
        tp_rotate.pose.orientation = tf2::toMsg(tf2::Quaternion(
            tf2::Vector3(0, 0, 1), new_yaw));
        
        tp_rotate.longitudinal_velocity_mps = tp.longitudinal_velocity_mps;
        tp_rotate.lateral_velocity_mps = tp.lateral_velocity_mps;  //横向速度=横向斜率*纵向速度
        tp_rotate.heading_rate_rps = tp.heading_rate_rps;

        traj_forward.points.emplace_back(tp_rotate);
    
    }

    if(is_reverse)
    {
        // 反向行驶时,需要将轨迹点顺序反转，并且调整朝向和横向速度
        size_t num_points = traj_forward.points.size();
        for(size_t i=0; i< num_points; ++i){
            TrajectoryPoint & point = traj_forward.points.at(num_points-1-i);
            
            // 横向速度取反
            point.lateral_velocity_mps = -point.lateral_velocity_mps;
            std::cout << "ind" << i <<", x:" << point.pose.position.x << ", y:" << point.pose.position.y << 
                ", dy:" << point.lateral_velocity_mps << ", wr:" << point.heading_rate_rps << std::endl;
            traj_planned.points.emplace_back(point);
        }
    }else
    {
        traj_planned = traj_forward;
        for(size_t i=0; i< traj_planned.points.size(); ++i){
            TrajectoryPoint & point = traj_planned.points.at(i);
            std::cout << "ind" << i <<", x:" << point.pose.position.x << ", y:" << point.pose.position.y << 
                ", dy:" << point.lateral_velocity_mps << ", wr:" << point.heading_rate_rps << std::endl;
        }
    }
    return true;
}

// ---------- 采样点生成 ----------
void MapTrajServer::sampling_points_xy(int pts_num, double ds_1, double ds_2, double dk_ds, double kappa_max,
    std::vector<double>& x_pts, std::vector<double>& y_pts) {
    double s_sum = 2.0 * ds_1 + ds_2;
    double ds_d = s_sum / pts_num;
    x_pts.resize(pts_num);
    y_pts.resize(pts_num);
    std::vector<double> theta_pts(pts_num);

    double R = 1.0 / kappa_max;
    double x1, y1, theta1;
    clothoid_point(kappa_max, ds_1, ds_1, x1, y1, theta1);
    double Ox = x1 - R * std::sin(theta1);
    double Oy = y1 + R * std::cos(theta1);
    double theta_pts1 = theta1;
    double theta_pts2 = theta1 + M_PI / 2.0 - kappa_max * ds_1;
    double x2 = Ox + R * std::cos(theta1 - M_PI / 2.0 + (ds_2 / R));
    double y2 = Oy + R * std::sin(theta1 - M_PI / 2.0 + ds_2 / R);

    double A = std::sqrt(ds_1 / kappa_max);
    double B = std::sqrt(kappa_max * ds_1 / M_PI);
    double S_B = fresnels(B);
    double C_B = fresnelc(B);

    for (int m = 0; m < pts_num; ++m) {
    double s = ds_d * (m + 1);
    if (s <= ds_1) {
        double u = std::sqrt(dk_ds / M_PI) * s;
        x_pts[m] = (1.0 / std::sqrt(dk_ds / M_PI)) * fresnelc(u);
        y_pts[m] = (1.0 / std::sqrt(dk_ds / M_PI)) * fresnels(u);
        theta_pts[m] = 0.5 * dk_ds * s * s;
    } else if (s > ds_1 && s < ds_1 + ds_2) {
        double ds = s - ds_1;
        x_pts[m] = Ox + R * std::cos(theta1 - M_PI / 2.0 + ds / R);
        y_pts[m] = Oy + R * std::sin(theta1 - M_PI / 2.0 + ds / R);
        theta_pts[m] = theta1 + ds / R;
    } else {
        double ds = s - ds_1 - ds_2;
        double dtheta = kappa_max * ds - (dk_ds / 2.0) * ds * ds;
        theta_pts[m] = theta_pts2 + dtheta;
        double t = 1.0 - ds / ds_1;
        double u = B * t;
        x_pts[m] = x2 + A * std::sqrt(M_PI) * (S_B - fresnels(u));
        y_pts[m] = y2 + A * std::sqrt(M_PI) * (C_B - fresnelc(u));
    }
}
}

void MapTrajServer::sampling_points_yaw(int pts_num, double ds_1, double ds_2, double dk_ds, double kappa_max,
     std::vector<double>& theta_pts, std::vector<double>& kappa_pts) {
    double s_sum = 2.0 * ds_1 + ds_2;
    double ds_d = s_sum / pts_num;
    theta_pts.resize(pts_num);
    kappa_pts.resize(pts_num);
    double theta_pts1 = 0.5 * dk_ds * ds_1 * ds_1;
    double theta_pts2 = theta_pts1 + M_PI / 2.0 - kappa_max * ds_1;
    double R = 1.0 / kappa_max;
    for (int m = 0; m < pts_num; ++m) {
        double s = ds_d * (m + 1);
        if (s <= ds_1) {
            theta_pts[m] = 0.5 * dk_ds * s * s;
            kappa_pts[m] = dk_ds * s;
        } else if (s > ds_1 && s < ds_1 + ds_2) {
            double ds = s - ds_1;
            theta_pts[m] = theta_pts1 + ds / R;
            kappa_pts[m] = kappa_max;
        } else {
            double ds = s - ds_1 - ds_2;
            double dtheta = kappa_max * ds - (dk_ds / 2.0) * ds * ds;
            theta_pts[m] = theta_pts2 + dtheta;
            kappa_pts[m] = kappa_max - dk_ds * ds;
        }
    }
}

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<map_traj_server::MapTrajServer>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}