#include "interface_test.hpp"

#include <chrono>
#include <thread>

namespace interface_test
{

using namespace std::literals::chrono_literals;

Interface_Test::Interface_Test(const rclcpp::NodeOptions &options)
: Node("interface_test", options)
{
    // RCLCPP_INFO(this->get_logger(), "1");
    //声明参数
    this->declare_parameter("Kp", 1.0);
    this->declare_parameter("Ki", 0.1);
    this->declare_parameter("Kd", 0.05);
    this->declare_parameter("rosbag_name", "odom_data");

    //创建发布者
    pub_shift_cmd_ = create_publisher<SystemCmdInt>("shift_cmd", 1);
    pub_brake_cmd_ = create_publisher<SystemCmdFloat>("brake_cmd", 1);
    pub_torque_cmd_ = create_publisher<SystemCmdFloat>("torque_cmd", 1);
    pub_motor_vel_cmd_ = create_publisher<SystemCmdFloat>("motor_vel_cmd", 1);
    pub_steering_cmd_ = create_publisher<SteeringCmd>("tire_steer_cmd", 1);
    pub_reset_cmd_ = create_publisher<SystemCmdBool>("reset_cmd", 1);
    pub_emergency_cmd_ = create_publisher<SystemCmdBool>("emergency_cmd", 1);

    //创建一个writer，将话题写入bag包
    // writer_ = std::make_unique<rosbag2_cpp::Writer>();
    // auto rosbag_name = this->get_parameter("rosbag_name").as_string();
    // writer_->open(rosbag_name);
    // writer_->create_topic(
    //     {"odometry",
    //     "nav_msgs/msg/Odometry",
    //     rmw_get_serialization_format(),
    //     ""});
    
    // writer_1 = std::make_unique<rosbag2_cpp::Writer>();
    // writer_1->open("speed");
    // writer_1->create_topic(
    //     {"speed",
    //     "wheel_speed_rpt",
    //     rmw_get_serialization_format(),
    //     ""});

    //创建订阅者
    // sub_upper_test_ = this->create_subscription<UpperTest>("upper_test", 1, 
    //     std::bind(&Interface_Test::upper_test_callback, this, std::placeholders::_1));
    
    sub_odometry_ = this->create_subscription<Odometry>("odometry", 10,
        std::bind(&Interface_Test::odometry_callback, this, std::placeholders::_1));
    
    sub_global_rpt_ = this->create_subscription<GlobalRpt>("global_rpt", 10,
        std::bind(&Interface_Test::global_rpt_callback, this, std::placeholders::_1));
    
    //创建消息的服务器
    srv_upper_test_ = this->create_service<srvUpperTest>("srv_upper_test", 
        std::bind(&Interface_Test::srv_upper_test_callback, this, std::placeholders::_1, std::placeholders::_2));

    //创建定时器，在定时任务中处理上位机并录制消息
    timer_ = rclcpp::create_timer(
        this, get_clock(), 33ms, std::bind(&Interface_Test::onTimer, this));

    //创建一个PID控制器
    torque_pid_ = std::make_unique<PIDController>();
    pid_initial();

    //初始化rosbag录制相关的变量
    is_record_new = false;
    can_record = false;
    traj_tcnt = 0;

    RCLCPP_INFO(this->get_logger(), "intial successful");
}

void Interface_Test::pid_initial()
{
    double Kp = this->get_parameter("Kp").as_double();
    double Ki = this->get_parameter("Ki").as_double();
    double Kd = this->get_parameter("Kd").as_double();

    torque_pid_->setGains(Kp, Ki, Kd);
    torque_pid_->setLimits(MAX_RET, MIN_RET, MAX_RET_K, MIN_RET_K, MAX_RET_I, MIN_RET_I, MAX_RET_D, MIN_RET_D);
}

void Interface_Test::onTimer()
{
    //处理上位机发送的消息
    upper_msg_exec();

    //将话题数据录制到bag包中
    if(can_record && is_record_new)
    {
        rclcpp::Time time_stamp = this->now();
        // writer_->write(odometry, "wheel_speed_rpt", time_stamp);
        // writer_1->write(wheel_speed_rpt_msg,"wheel_speed_rpt", time_stamp);
        is_record_new = false;
    }
}

void Interface_Test::upper_msg_exec()
{
    // 构造函数中初始化upper_test.is_new = false，保证话题启动之前不会执行   
    // 新的上位机消息会将is_new更新为true，并在每次执行完此函数后，将is_new=false.
    if(upper_test.is_new == false)
        return;

    // 判断upper_test_.index索引范围，如果超限也退出
    if(upper_test.index >= CMD_END_INDEX)
        return;

    switch(upper_test.index)
    {
        case SHIFT_CMD:
        {
            SystemCmdInt shift_cmd;
            shift_cmd.enable = upper_test.enable;
            shift_cmd.command = upper_test.int_cmd;
            pub_shift_cmd_->publish(shift_cmd);

            upper_test.is_new = false;
        }
            break;
        case BRAKE_CMD:
        {
            SystemCmdFloat brake_cmd;
            brake_cmd.enable = upper_test.enable;
            brake_cmd.command = upper_test.float_cmd;
            pub_brake_cmd_->publish(brake_cmd);

            upper_test.is_new = false;
        }
            break;
        case MOTOR_VEL_CMD:
        {
            SystemCmdFloat motor_vel_cmd;
            motor_vel_cmd.enable = upper_test.enable;
            motor_vel_cmd.command = upper_test.float_cmd;
            pub_motor_vel_cmd_->publish(motor_vel_cmd);

            upper_test.is_new = false;
        }
            break;
        case STEER_CMD:
        {   
            SteeringCmd steering_cmd;
            steering_cmd.enable = upper_test.enable;
            if(upper_test.int_cmd == 1){
                steering_cmd.axis1_command = upper_test.float_cmd;
            }else if(upper_test.int_cmd == 2){
                steering_cmd.axis2_command = upper_test.float_cmd;
            }else if(upper_test.int_cmd == 3){
                steering_cmd.axis3_command = upper_test.float_cmd;
            }else if(upper_test.int_cmd == 4){
                steering_cmd.axis4_command = upper_test.float_cmd;
            }
            
            pub_steering_cmd_->publish(steering_cmd);
            // RCLCPP_INFO(this->get_logger(), "publish tire_steering_cmd:%f", steering_cmd.command);
            upper_test.is_new = false;
        }
            break;
        case RESET_CMD:
        {
            SystemCmdBool reset_cmd;
            reset_cmd.enable = upper_test.enable;
            reset_cmd.command = upper_test.bool_cmd;
            pub_reset_cmd_->publish(reset_cmd);  

            upper_test.is_new = false;
        }
            break;
        case EMERGENCY_CMD:
        {
            SystemCmdBool emergency_cmd;
            emergency_cmd.enable = upper_test.enable;
            emergency_cmd.command = upper_test.bool_cmd;
            pub_emergency_cmd_->publish(emergency_cmd);

            upper_test.is_new = false;
        }
            break;
        case INITIAL_ENABLE:
        {
            //4个话题分别初始化使能，在pacmod中Interface_Ready=true，合油泵
            SystemCmdInt shift_cmd;
            shift_cmd.enable = true;
            shift_cmd.command = 3;             //表示初始化前进
            pub_shift_cmd_->publish(shift_cmd);
            std::this_thread::sleep_for(1ms);

            SystemCmdFloat brake_cmd;
            brake_cmd.enable = true;
            brake_cmd.command = 0;             //超过0.1就会刹车，所以初始化刹车命令打开
            pub_brake_cmd_->publish(brake_cmd);
            std::this_thread::sleep_for(1ms);
            
            SystemCmdFloat motor_vel_cmd;
            motor_vel_cmd.enable = true;
            motor_vel_cmd.command = 0;         //初始化速度为0，后续手动给一个启动速度，车就可以跑起来
            pub_motor_vel_cmd_->publish(motor_vel_cmd);
            std::this_thread::sleep_for(1ms);

            SteeringCmd steering_cmd;
            steering_cmd.enable = true;
            steering_cmd.axis1_command = 0.0;          //最开始的时候保证轮子打直，但比较好的方法可能是读取转角编码器，让轮子处于初始值
            steering_cmd.axis2_command = 0.0;
            steering_cmd.axis3_command = 0.0;
            steering_cmd.axis4_command = 0.0;
            pub_steering_cmd_->publish(steering_cmd);

            upper_test.is_new = false;
        }
            break;
        case TOQUE_CMD:
        {
            //下发转矩命令，这个需要对应修改pacmod代码
            SystemCmdFloat torque_cmd;
            torque_cmd.enable = upper_test.enable;
            torque_cmd.command = upper_test.float_cmd;
            pub_torque_cmd_->publish(torque_cmd);
            RCLCPP_INFO(this->get_logger(), "publish torque_cmd:%f", torque_cmd.command);
            upper_test.is_new = false;
        }
            break;
        case VEL_TOL_CMD:
        {
            //订阅底盘上传的轴速数据，进行pid
            double vel_ref, vel_fbk, tor_out;
            vel_ref = upper_test.float_cmd;
            vel_fbk = (global_rpt_msg.motor1_vel_fbk_f64 + global_rpt_msg.motor2_vel_fbk_f64) / 2;
            
            std::vector<double> p;
            tor_out = torque_pid_->calculate(vel_ref-vel_fbk, 30/1000, true, p);

            SystemCmdFloat torque_cmd;
            torque_cmd.enable = upper_test.enable;
            torque_cmd.command = tor_out;
            pub_torque_cmd_->publish(torque_cmd);
            
            //这里并不会主动结束速度控制命令，需要在外部给定一个停止信号
        }
            break;
        case VEL_STEER_GENERATE:
        {
            double steer_max = upper_test.float_cmd;
            double vel_max = double(upper_test.int_cmd);
            //首先产生轨迹
            auto vel_steer = traj_generate(vel_max, steer_max);
            
            //将速度和转角发布出去
            SystemCmdFloat motor_vel_cmd;
            motor_vel_cmd.enable = upper_test.enable;
            motor_vel_cmd.command = vel_steer.at(0);
            pub_motor_vel_cmd_->publish(motor_vel_cmd);

            SteeringCmd steering_cmd;
            steering_cmd.enable = upper_test.enable;
            steering_cmd.axis1_command = vel_steer.at(1);
            steering_cmd.axis2_command = vel_steer.at(1);
            steering_cmd.axis3_command = vel_steer.at(1);
            steering_cmd.axis4_command = vel_steer.at(1);          
            pub_steering_cmd_->publish(steering_cmd);

            //更新traj_tcnt
            traj_tcnt++;
            // can_record = true;
            if(traj_tcnt>=400){
                traj_tcnt = 0;
                upper_test.is_new = false;
                // can_record = false;
                // RCLCPP_INFO(this->get_logger(), "record_finished");
            }
        }
            break;
        case RESPONSE_TEST:
        {
            // 测试用例：速度斜坡响应测试
            double vel_max = upper_test.float_cmd;   //接收最大速度

            // 如果斜坡信号的斜率是4m/s^2，定时器周期是33ms
            double traj_t_max = vel_max/4/0.033;           //当traj_tcnt计数超过该值的时候达到最大速度
            double traj_t_sum = 5/0.033;                   //记录5s，当traj_tcnt超过该值的时候结束录制
            double vel_cmd = traj_tcnt * 4 * 0.033;       
            
            can_record = true;
            if(traj_tcnt <= traj_t_max)
            {
                vel_cmd = traj_tcnt * 4 * 0.033;
                traj_tcnt++;
            }
            else if(traj_tcnt <= traj_t_sum && traj_tcnt > traj_t_max)
            {
                vel_cmd = vel_max;
                traj_tcnt++;
            }
            else
            {
                traj_tcnt = 0;
                upper_test.is_new = false;
                can_record = false;
                RCLCPP_INFO(this->get_logger(), "record_finished");
                return;
            }

            //将速度发布出去
            SystemCmdFloat motor_vel_cmd;
            motor_vel_cmd.enable = upper_test.enable;
            motor_vel_cmd.command = vel_cmd;
            pub_motor_vel_cmd_->publish(motor_vel_cmd);
            
        }
            break;
        default:
            break;
    }
}

std::vector<double> Interface_Test::traj_generate(double max_vel, double max_steer)
{
    //反馈的第一个值是速度，第二个值是转角
    std::vector<double> ret;
    double accel = max_vel/100;       //100点线性增加到最大速度
    double steer_rps = max_steer/50;  //50点线性增加到最大转角 

    //三个阶段：从0启动加速、匀速运动、减速到0
    if(traj_tcnt>=0 && traj_tcnt<100)
    {
        //这个阶段按照accel加速
        ret.push_back(accel*traj_tcnt);
        ret.push_back(0);
    }else if(traj_tcnt>=100 && traj_tcnt<300)
    {
        //这个阶段速度保持匀速，改变转角
        ret.push_back(max_vel);
        if(traj_tcnt>=150 && traj_tcnt<200)
        {
            ret.push_back(steer_rps*(traj_tcnt - 150));    
        }else if(traj_tcnt>=200 && traj_tcnt<250)
        {
            ret.push_back(max_steer - steer_rps*(traj_tcnt - 200));
        }else
        {
            ret.push_back(0.0);
        }
    }else if(traj_tcnt>=300 && traj_tcnt<400)
    {
        //这个阶段按照-accel减速到0
        ret.push_back(max_vel - accel*(traj_tcnt - 300));
        ret.push_back(0.0);
    }

    return ret;
}

void Interface_Test::srv_upper_test_callback(const srvUpperTest::Request::SharedPtr request, srvUpperTest::Response::SharedPtr response)
{
    upper_test.index = request->index;
    upper_test.enable = request->enable;
    upper_test.is_new = request->is_new;
    upper_test.bool_cmd = request->bool_cmd;
    upper_test.int_cmd = request->int_cmd;
    upper_test.float_cmd = request->float_cmd;

    response->result = 0;
}

}
