#ifndef INTERFACE_TEST_HPP_
#define INTERFACE_TEST_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <pacmod_msgs/msg/system_cmd_bool.hpp>
#include <pacmod_msgs/msg/system_cmd_float.hpp>
#include <pacmod_msgs/msg/system_cmd_int.hpp>
#include <pacmod_msgs/msg/steering_cmd.hpp>
#include <pacmod_msgs/msg/global_rpt.hpp>
#include <pacmod_msgs/srv/upper_test.hpp>

#include <pid.hpp>

#define SHIFT_CMD 1
#define BRAKE_CMD 2
#define MOTOR_VEL_CMD 3
#define STEER_CMD 4
#define RESET_CMD 5
#define EMERGENCY_CMD 6
#define INITIAL_ENABLE 7
#define TOQUE_CMD 8      //写一个转矩接口，直接下发转矩消息（对应修改pacmod的代码）
#define VEL_TOL_CMD 9    //下发一个转速，但是通过轴速反馈和pid，输出转矩给定
#define VEL_GENERATE 10  //生成一个速度轨迹，不断地下发给底盘（但是不可能一只阻塞在里面发，需要额外有一个线程）
#define VEL_STEER_GENERATE 11  //生成一个速度和转角的轨迹，不断地下发给底盘
#define RESPONSE_TEST 12
#define CMD_END_INDEX 13

#define MAX_RET      1.0
#define MIN_RET     -1.0
#define MAX_RET_K    1.0
#define MIN_RET_K   -1.0
#define MAX_RET_I    1.0
#define MIN_RET_I   -1.0
#define MAX_RET_D    1.0
#define MIN_RET_D   -1.0

namespace interface_test
{
using nav_msgs::msg::Odometry;

using pacmod_msgs::msg::SteeringCmd;
using pacmod_msgs::msg::SystemCmdBool;
using pacmod_msgs::msg::SystemCmdInt;
using pacmod_msgs::msg::SystemCmdFloat;
using pacmod_msgs::msg::GlobalRpt;
using srvUpperTest = pacmod_msgs::srv::UpperTest;

struct UpperTest{
  uint32_t  index;
  bool      is_new;
  bool      enable;
  bool      bool_cmd;
  int32_t   int_cmd;
  double    float_cmd;
};

class Interface_Test : public rclcpp::Node
{
public:
  explicit Interface_Test(const rclcpp::NodeOptions & options);
  ~Interface_Test() = default;

private:
  //定时器，在定时器中处理测试命令，并且录制定位信息
  rclcpp::TimerBase::SharedPtr timer_;
  
  //发布者，向pacmod发送命令
  rclcpp::Publisher<SystemCmdInt>::SharedPtr pub_shift_cmd_;
  rclcpp::Publisher<SystemCmdFloat>::SharedPtr pub_brake_cmd_;
  rclcpp::Publisher<SystemCmdFloat>::SharedPtr pub_torque_cmd_;
  rclcpp::Publisher<SystemCmdFloat>::SharedPtr pub_motor_vel_cmd_;
  rclcpp::Publisher<SteeringCmd>::SharedPtr pub_steering_cmd_;
  rclcpp::Publisher<SystemCmdBool>::SharedPtr pub_reset_cmd_;
  rclcpp::Publisher<SystemCmdBool>::SharedPtr pub_emergency_cmd_;

  //订阅者，接收上位机、定位、以及pacmod反馈rpt数据
  UpperTest upper_test;
  Odometry odometry;
  GlobalRpt global_rpt_msg;
  // SystemRptFloat::SharedPtr front_steering_rpt_msg;

  // rclcpp::Subscription<UpperTest>::SharedPtr sub_upper_test_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<GlobalRpt>::SharedPtr sub_global_rpt_;
  // rclcpp::Subscription<SystemRptFloat>::SharedPtr front_steering_rpt_msg;

  rclcpp::Service<srvUpperTest>::SharedPtr srv_upper_test_;

  //bag包的写入器
  std::unique_ptr<rosbag2_cpp::Writer> writer_;

  std::unique_ptr<rosbag2_cpp::Writer> writer_1;

  //pid控制器
  std::unique_ptr<PIDController> torque_pid_;

  bool is_record_new;   //话题数据是否更新
  bool can_record;      //rosbag可以录制数据
  int traj_tcnt;        //用来对轨迹的时间步进行记录，最大300，表示运动10s

  void pid_initial();
  void onTimer();
  void upper_msg_exec();
  std::vector<double> traj_generate(double accel, double steer_aps);

  // void upper_test_callback(const UpperTest::SharedPtr msg) { upper_test = *msg;}
  void global_rpt_callback(const GlobalRpt::SharedPtr msg) {global_rpt_msg = *msg;}
  void odometry_callback(const Odometry::SharedPtr msg){ 
    odometry = *msg; 
    is_record_new = true;
  }

  void srv_upper_test_callback(const srvUpperTest::Request::SharedPtr request,
    srvUpperTest::Response::SharedPtr response);

};

}  // namespace interface_test

#endif  