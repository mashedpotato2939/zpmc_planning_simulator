#ifndef VEHICLE_SIM__NODE_HPP_
#define VEHICLE_SIM__NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pacmod_msgs/srv/upper_test.hpp>
#include <pacmod_msgs/msg/upper_test.hpp>

using nav_msgs::msg::Odometry; 
using autoware_planning_msgs::msg::Trajectory; 
using srvUpperTest = pacmod_msgs::srv::UpperTest;
using msgUpperTest = pacmod_msgs::msg::UpperTest;

namespace cb_planning
{

struct UpperTest{
    uint32_t  index;
    bool      is_new;
    bool      enable;
    bool      bool_cmd;
    int32_t   int_cmd;
    double    float_cmd;
};

class Vehicle_Simulator : public rclcpp::Node
{
public:
    explicit Vehicle_Simulator(const rclcpp::NodeOptions & node_options);

private:
    // 定位信息发布 
    rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
    Odometry::SharedPtr ego_state_ptr_;
    double integ_x_ = 0.0;
    double integ_y_ = 0.0;
    double integ_yaw_ = 0.0;

    // 轨迹信息订阅
    rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
    void onTraj(const Trajectory::SharedPtr); 
    Trajectory::SharedPtr ego_traj_ptr_;
    bool has_traj_;
    bool is_traj_new;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    void onTimer();

    // 服务回调函数
    // rclcpp::Service<srvUpperTest>::SharedPtr srv_upper_test_;
    // UpperTest upper_test;
    // void srv_upper_test_callback(const srvUpperTest::Request::SharedPtr request, 
    //                             srvUpperTest::Response::SharedPtr response);

    // 服务回调改成一个订阅者
    rclcpp::Subscription<msgUpperTest>::SharedPtr upper_test_sub_;
    void onUpperTest(const msgUpperTest::SharedPtr);
    UpperTest upper_test_;

};

}

#endif