#include "vehicle_sim_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/utils.h"
#include "geometry_msgs/msg/quaternion.hpp"

namespace cb_planning{

Vehicle_Simulator::Vehicle_Simulator(const rclcpp::NodeOptions & options)
: Node("vehicle_simulator", options)
{
    is_traj_new = false;
    has_traj_ = false;
    ego_traj_ptr_ = std::make_shared<Trajectory>();
    traj_sub_ = create_subscription<Trajectory>(
        "input/trajectory", 10, std::bind(&Vehicle_Simulator::onTraj, this, std::placeholders::_1));
    
    odom_pub_ = this->create_publisher<Odometry>("output/kinematics", 10);

    // srv_upper_test_ = this->create_service<srvUpperTest>("/srv_upper_test", 
    //     std::bind(&Vehicle_Simulator::srv_upper_test_callback, this, std::placeholders::_1, std::placeholders::_2));
    upper_test_sub_ = this->create_subscription<msgUpperTest>(
        "/input/upper_test", 10, std::bind(&Vehicle_Simulator::onUpperTest, this, std::placeholders::_1));

    using namespace std::literals::chrono_literals;
    timer_ = rclcpp::create_timer(
        this, get_clock(), 33ms, std::bind(&Vehicle_Simulator::onTimer, this));

    ego_state_ptr_ = std::make_shared<Odometry>();
    ego_state_ptr_->header.frame_id = "map";
    ego_state_ptr_->child_frame_id = "base_link";

    //初始化车身的位姿
    ego_state_ptr_->pose.pose.position.x = 0.0;
    ego_state_ptr_->pose.pose.position.y = 0.65;
    ego_state_ptr_->pose.pose.position.z = 0.0;
    ego_state_ptr_->twist.twist.linear.x = 0.0;
    ego_state_ptr_->twist.twist.linear.y = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    ego_state_ptr_->pose.pose.orientation = tf2::toMsg(q);
}

void Vehicle_Simulator::onTimer()
{
    // 首先发布位置信息
    ego_state_ptr_->header.stamp = now();
    odom_pub_->publish(*ego_state_ptr_);

    if(!has_traj_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for trajectory …");
        return;
    }
    
    if(upper_test_.enable)
    {
        // 首先找到与当前位置匹配的轨迹点
        size_t closest_idx = 0;
        while(ego_state_ptr_->pose.pose.position.x > ego_traj_ptr_->points.at(closest_idx).pose.position.x && 
              closest_idx < ego_traj_ptr_->points.size()-1)
        {
            closest_idx++;
        }
        geometry_msgs::msg::Pose pose = ego_state_ptr_->pose.pose;
        // double ay = (ego_traj_ptr_->points.at(closest_idx).lateral_velocity_mps - ego_traj_ptr_->points.at(closest_idx+1).lateral_velocity_mps) / 0.1;

        ego_state_ptr_->twist.twist.linear.x = ego_traj_ptr_->points.at(closest_idx).longitudinal_velocity_mps;
        ego_state_ptr_->twist.twist.linear.y = ego_traj_ptr_->points.at(closest_idx).lateral_velocity_mps;

        /* 33 ms 固定步长 */
        constexpr double dt = 0.033;
        const double vx = ego_state_ptr_->twist.twist.linear.x;
        const double vy = ego_state_ptr_->twist.twist.linear.y;

        // std::cout << "x: " << pose.position.x << ", y: " << pose.position.y << ", vx: " << vx << ", vy:" << vy << std::endl;

        /* 欧拉积分 */
        integ_x_  = vx * dt;
        integ_y_  = vy * dt;

        // 根据轨迹中两个临近点插值直接计算绝对位姿（不积分）
        geometry_msgs::msg::Pose p0, p1;
        if(closest_idx == 0)
        {
            p0 = ego_state_ptr_->pose.pose;
            p1 = ego_traj_ptr_->points.at(closest_idx).pose; 
        }else
        {
            p0 = ego_traj_ptr_->points.at(closest_idx-1).pose;
            p1 = ego_traj_ptr_->points.at(closest_idx).pose; 
        }
        
        /* 更新xy位置：起点+积分量 */
        tf2::Quaternion q0;
        tf2::fromMsg(ego_state_ptr_->pose.pose.orientation, q0);
        double r,p,yaw0;
        tf2::Matrix3x3(q0).getRPY(r, p, yaw0);
        ego_state_ptr_->pose.pose.position.x += integ_x_ * std::cos(yaw0) - integ_y_ * std::sin(yaw0);
        ego_state_ptr_->pose.pose.position.y += integ_x_ * std::sin(yaw0) + integ_y_ * std::cos(yaw0);

        double t = (ego_state_ptr_->pose.pose.position.x - p0.position.x) / (p1.position.x - p0.position.x);
        double yaw;
        yaw = tf2::getYaw(p0.orientation) + t * (tf2::getYaw(p1.orientation) - tf2::getYaw(p0.orientation));

        // ego_state_ptr_->pose.pose.position.x += integ_x_;
        // ego_state_ptr_->pose.pose.position.y += integ_y_;
        
        tf2::Quaternion q;
        q.setRPY(r, p, yaw);
        ego_state_ptr_->pose.pose.orientation = tf2::toMsg(q);
    }
}

void Vehicle_Simulator::onTraj(const Trajectory::SharedPtr msg)
{
    if(msg->points.empty()) return;

    /* 把起点一次性记下来，积分量清零 */
    // const auto& p0 = msg->points.front();
    // ego_state_ptr_->pose.pose = p0.pose;
    ego_traj_ptr_ = msg;
    // ego_state_ptr_->twist.twist.linear.x = p0.longitudinal_velocity_mps;
    // ego_state_ptr_->twist.twist.linear.y = p0.lateral_velocity_mps;
    integ_x_ = integ_y_ = integ_yaw_ = 0.0;
    has_traj_ = true;
    is_traj_new = true;
}

void Vehicle_Simulator::onUpperTest(const msgUpperTest::SharedPtr msg)
{
    upper_test_.index = msg->index;
    upper_test_.enable = msg->enable;
    upper_test_.is_new = msg->is_new;
    upper_test_.bool_cmd = msg->bool_cmd;
    upper_test_.int_cmd = msg->int_cmd;
    upper_test_.float_cmd = msg->float_cmd;
}

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cb_planning::Vehicle_Simulator>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}