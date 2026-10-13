#include "motion_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

namespace
{
double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yawFromPose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double pathTangentAt(const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const size_t index)
{
  const size_t n = points.size();
  const size_t prev = index == 0 ? 0 : index - 1;
  const size_t next = index + 1 >= n ? n - 1 : index + 1;
  const auto & p0 = points.at(prev).pose.position;
  const auto & p1 = points.at(next).pose.position;
  return std::atan2(p1.y - p0.y, p1.x - p0.x);
}
}  // namespace

namespace motion_controller
{

  using namespace std::literals::chrono_literals;

MotionController::MotionController(const rclcpp::NodeOptions & options)
: Node("motion_control", options)
{
  tracker_       = std::make_shared<TrajectoryTracker>();
  tracker_param_ = std::make_shared<TrackerParam>();

  pub_cmd_ = create_publisher<Control>("output/control_cmd", 1);
  pub_motor_vel_cmd_ = create_publisher<SystemCmdFloat>("/pacmod/motor_vel_cmd", 1);
  pub_steering_cmd_ = create_publisher<SteeringCmd>("/pacmod/steering_cmd", 1);
  pub_shift_cmd_ = create_publisher<SystemCmdInt>("/pacmod/shift_cmd", 1);
  pub_brake_cmd_ = create_publisher<SystemCmdFloat>("/pacmod/brake_cmd", 1);
  dbg_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/debug/data", 10);
  traj_debug_pub_ =
    create_publisher<std_msgs::msg::Float32MultiArray>("/debug/trajectory_points", 1);
  traj_yaw_vs_s_pub_ =
    create_publisher<std_msgs::msg::Float32MultiArray>("/debug/trajectory/yaw_vs_s", 1);
  traj_kappa_vs_s_pub_ =
    create_publisher<std_msgs::msg::Float32MultiArray>("/debug/trajectory/kappa_vs_s", 1);
  traj_beta_vs_s_pub_ =
    create_publisher<std_msgs::msg::Float32MultiArray>("/debug/trajectory/beta_vs_s", 1);

  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, [this](const Odometry::ConstSharedPtr msg) {
        odometry_ = msg;
        is_odometry_new = true;});
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", 1, [this](const Trajectory::ConstSharedPtr msg) {
        trajectory_ = msg;
        is_trajectory_new = true;});

  // /planning/reverse_state is published latched by igv_path_planner and consumed
  // by the rest of the stack (vehicle_cmd_gate, planning_validator, ...).
  rclcpp::QoS reverse_qos(1);
  reverse_qos.reliable().transient_local();
  sub_reverse_state_ = create_subscription<reverse_vehicle_msg::msg::VehicleReversed>(
    "/planning/reverse_state", reverse_qos,
    [this](const reverse_vehicle_msg::msg::VehicleReversed::ConstSharedPtr msg) {
        tracker_param_->is_forward = !msg->is_reversed;});

  // 声明参数
  tracker_param_->omega = declare_parameter<double>("trk_omega", 30.0);
  tracker_param_->dt = declare_parameter<double>("t_interval", 0.02);  // 采样时间间隔
  const auto fallback_wheel_base =
    declare_parameter<double>("wheelbase", 6.0);  // legacy motion_control name
  tracker_param_->wheel_base =
    declare_parameter<double>("wheel_base", fallback_wheel_base);  // vehicle_info.param.yaml
  tracker_param_->wheel_base_ratio =
    declare_parameter<double>("wheelbase_ratio", 1.0);  // 二三轴轴距与一四轴轴距的比值
  const auto fallback_max_steering_angle = declare_parameter<double>("max_steering_angle", 0.70);
  tracker_param_->max_steering_angle = declare_parameter<double>(
    "max_steer_angle", fallback_max_steering_angle);  // vehicle_info.param.yaml
  tracker_param_->delta_limit =
    declare_parameter<double>("delta_limit", tracker_param_->max_steering_angle);
  tracker_param_->d_delta_limit =
    declare_parameter<double>("d_delta_limit", tracker_param_->max_steering_angle);
  tracker_param_->predicted_ts =
    declare_parameter<double>("predicted_time", 0.2);              // 预测时间，用于计算预瞄点位置
  tracker_param_->ky = declare_parameter<double>("trk_ky", -20.0);  // 预瞄点位置横向误差的增益
  tracker_param_->ky_reverse = declare_parameter<double>("trk_ky_reverse", -1.0);
  tracker_param_->kvy = declare_parameter<double>("trk_kvy", 0.0);                             // 预瞄点位置横向速度误差的增益
  tracker_param_->k_com = declare_parameter<double>("trk_kcom", 0.0);                          // 速度前馈增益
  tracker_param_->kyaw = declare_parameter<double>("trk_kyaw", -40.0);                          // 预瞄点位置航向误差的增益
  tracker_param_->kyaw_reverse = declare_parameter<double>("trk_kyaw_reverse", -0.5);
  tracker_param_->vx_min = declare_parameter<double>("trk_vxmin", 0.2);                        // 比例系数中的最小线速度，防止在低速时控制指令过大
  tracker_param_->is_forward = declare_parameter<bool>("is_forward", true);                    // 是否为前向行驶

  use_maptraj_server = declare_parameter<bool>("use_traj_server", false);
  publish_trajectory_debug = declare_parameter<bool>("publish_trajectory_debug", true);
  const auto trajectory_debug_interval =
    declare_parameter<double>("trajectory_debug_interval", 1.0);
  srv_upper_test_ = this->create_service<srvUpperTest>("/srv_upper_test",
    std::bind(&MotionController::srv_upper_test_callback, this, std::placeholders::_1, std::placeholders::_2));

  // 初始化 为 tracker_ 设置一次参数
  tracker_->setParameter(tracker_param_);
  RCLCPP_INFO(
    get_logger(), "motion_control vehicle geometry: wheel_base=%.3f, max_steer_angle=%.3f",
    tracker_param_->wheel_base, tracker_param_->max_steering_angle);

  // 发布底盘初始化的命令
  Vehicle_Init();

  // 创建定时器，周期为20ms，回调函数为onTimer
  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 20ms, std::bind(&MotionController::onTimer, this));

  const auto debug_period = std::chrono::duration<double>(
    std::max(trajectory_debug_interval, 0.1));
  traj_debug_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(debug_period),
    std::bind(&MotionController::publishTrajectoryDebug, this));

}

void MotionController::onTimer()
{
  if(!isDataReady()){
    return;
  }

  if(tracker_param_->is_forward)
  {
    SystemCmdInt shift_cmd;
    shift_cmd.enable = true;
    shift_cmd.command = 3;             //表示初始化前进
    pub_shift_cmd_->publish(shift_cmd);
    std::this_thread::sleep_for(1ms);
  }else
  {
    SystemCmdInt shift_cmd;
    shift_cmd.enable = true;
    shift_cmd.command = 1;             //表示初始化倒车
    pub_shift_cmd_->publish(shift_cmd);
    std::this_thread::sleep_for(1ms);
  }

  // 向tracker_设置轨迹和里程计数据，以及反向行驶命令
  if(is_trajectory_new){
    tracker_->setTrajectory(trajectory_);
    is_trajectory_new = false;
  }
  tracker_->setOdometry(odometry_);
  is_odometry_new = false;
  tracker_->setReverseCmd(tracker_param_->is_forward);

  // 运行tracker_，得到控制命令
  bool res;
  res = tracker_->run();

  // 打印debug数据
  std_msgs::msg::Float32MultiArray debug_msg;
  debug_msg.data.push_back(tracker_->betar_debug);
  debug_msg.data.push_back(tracker_->delta_debug);
  debug_msg.data.push_back(tracker_->ey_debug);
  debug_msg.data.push_back(tracker_->kappar_debug);
  debug_msg.data.push_back(tracker_->d_delta_debug);
  debug_msg.data.push_back(tracker_->eyaw_debug);
  dbg_pub_->publish(debug_msg);

  // 如果tracker_运行成功，则发布控制命令；否则发布零命令并打印错误日志
  if(res){
    if(upper_test.enable){
      SystemCmdFloat motor_vel_cmd;
      motor_vel_cmd.enable = upper_test.enable;
      motor_vel_cmd.command = tracker_->cmd.velocity_cmd;

      pub_motor_vel_cmd_->publish(motor_vel_cmd);

      SteeringCmd steering_cmd;
      steering_cmd.enable = upper_test.enable;
      steering_cmd.axis2_command = tracker_->cmd.axis2_steer_cmd;
      steering_cmd.axis3_command = tracker_->cmd.axis3_steer_cmd;
      steering_cmd.axis1_command = tracker_->cmd.axis1_steer_cmd;
      steering_cmd.axis4_command = tracker_->cmd.axis4_steer_cmd;
      pub_steering_cmd_->publish(steering_cmd);

      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "enable");
    } else {
      SystemCmdFloat motor_vel_cmd;
      motor_vel_cmd.enable = upper_test.enable;
      motor_vel_cmd.command = 0.0;
      pub_motor_vel_cmd_->publish(motor_vel_cmd);

      SteeringCmd steering_cmd;
      steering_cmd.enable = upper_test.enable;
      steering_cmd.axis2_command = 0.0;
      steering_cmd.axis3_command = 0.0;
      steering_cmd.axis1_command = 0.0;
      steering_cmd.axis4_command = 0.0;
      pub_steering_cmd_->publish(steering_cmd);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "disable");
    }

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 2000, "vx_ref:%.2f, yaw_ref:%.2f, yaw:%.2f",
      tracker_->cmd.velocity_cmd, tracker_->yawr_debug, tracker_->yaw_debug);
  } else{
    SystemCmdFloat motor_vel_cmd;
    motor_vel_cmd.enable = upper_test.enable;
    motor_vel_cmd.command = 0.0;
    pub_motor_vel_cmd_->publish(motor_vel_cmd);

    SteeringCmd steering_cmd;
    steering_cmd.enable = upper_test.enable;
    steering_cmd.axis1_command = 0.0;
    steering_cmd.axis2_command = 0.0;
    steering_cmd.axis3_command = 0.0;
    steering_cmd.axis4_command = 0.0;
    pub_steering_cmd_->publish(steering_cmd);

    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "trajectory_tracker run failed");
  }

  // High-level Control msg for vehicle_cmd_gate / simulator (every cycle for the heartbeat).
  Control control_msg;
  control_msg.stamp = control_msg.lateral.stamp = control_msg.longitudinal.stamp = get_clock()->now();
  if (res) {
    // The tracker keeps velocity_cmd >= 0 internally; restore the sign here from
    // is_forward (driven by /planning/reverse_state). vehicle_cmd_gate flips
    // steering for reverse but NOT velocity, so the sign must be correct here.
    const float v_sign = tracker_param_->is_forward ? 1.0f : -1.0f;
    control_msg.lateral.steering_tire_angle      = tracker_->cmd.axis2_steer_cmd;
    control_msg.lateral.rear_steering_tire_angle = tracker_->cmd.axis3_steer_cmd;
    control_msg.lateral.is_defined_rear_steering = true;
    control_msg.longitudinal.velocity            = v_sign * static_cast<float>(tracker_->cmd.velocity_cmd);
  } else {
    control_msg.lateral.steering_tire_angle      = 0.0f;
    control_msg.lateral.rear_steering_tire_angle = 0.0f;
    control_msg.lateral.is_defined_rear_steering = false;
    control_msg.longitudinal.velocity            = 0.0f;
  }
  pub_cmd_->publish(control_msg);
}

void MotionController::publishTrajectoryDebug()
{
  if (!publish_trajectory_debug || !trajectory_ || trajectory_->points.size() < 2) {
    return;
  }

  const auto & points = trajectory_->points;
  const size_t point_size = points.size();
  constexpr size_t field_size = 7;

  std::vector<double> s(point_size, 0.0);
  std::vector<double> yaw(point_size, 0.0);
  std::vector<double> kappa(point_size, 0.0);
  std::vector<double> beta(point_size, 0.0);

  for (size_t i = 0; i < point_size; ++i) {
    yaw.at(i) = yawFromPose(points.at(i).pose);
    if (i > 0) {
      const auto & p0 = points.at(i - 1).pose.position;
      const auto & p1 = points.at(i).pose.position;
      s.at(i) = s.at(i - 1) + std::hypot(p1.x - p0.x, p1.y - p0.y);
    }
  }

  constexpr double min_ds = 1.0e-3;
  for (size_t i = 0; i < point_size; ++i) {
    const size_t prev = i == 0 ? 0 : i - 1;
    const size_t next = i + 1 >= point_size ? point_size - 1 : i + 1;
    const double ds = std::max(s.at(next) - s.at(prev), min_ds);
    kappa.at(i) = normalizeAngle(yaw.at(next) - yaw.at(prev)) / ds;
    beta.at(i) = normalizeAngle(yaw.at(i) - pathTangentAt(points, i));
  }

  std_msgs::msg::Float32MultiArray msg;
  msg.layout.dim.resize(2);
  msg.layout.dim.at(0).label = "point";
  msg.layout.dim.at(0).size = point_size;
  msg.layout.dim.at(0).stride = point_size * field_size;
  msg.layout.dim.at(1).label = "field:s,yaw,kappa,beta,vx,heading_rate,acceleration";
  msg.layout.dim.at(1).size = field_size;
  msg.layout.dim.at(1).stride = field_size;
  msg.layout.data_offset = 0;
  msg.data.reserve(point_size * field_size);

  for (size_t i = 0; i < point_size; ++i) {
    const auto & point = points.at(i);
    const float vx = static_cast<float>(point.longitudinal_velocity_mps);
    const float heading_rate = static_cast<float>(std::fabs(vx) * kappa.at(i));
    msg.data.push_back(static_cast<float>(s.at(i)));
    msg.data.push_back(static_cast<float>(yaw.at(i)));
    msg.data.push_back(static_cast<float>(kappa.at(i)));
    msg.data.push_back(static_cast<float>(beta.at(i)));
    msg.data.push_back(vx);
    msg.data.push_back(heading_rate);
    msg.data.push_back(static_cast<float>(point.acceleration_mps2));
  }

  traj_debug_pub_->publish(msg);

  auto make_xy_msg = [point_size](const std::string & label,
                       const std::vector<double> & x,
                       const std::vector<double> & y) {
    std_msgs::msg::Float32MultiArray xy_msg;
    xy_msg.layout.dim.resize(2);
    xy_msg.layout.dim.at(0).label = "point";
    xy_msg.layout.dim.at(0).size = point_size;
    xy_msg.layout.dim.at(0).stride = point_size * 2;
    xy_msg.layout.dim.at(1).label = label;
    xy_msg.layout.dim.at(1).size = 2;
    xy_msg.layout.dim.at(1).stride = 2;
    xy_msg.layout.data_offset = 0;
    xy_msg.data.reserve(point_size * 2);

    for (size_t i = 0; i < point_size; ++i) {
      xy_msg.data.push_back(static_cast<float>(x.at(i)));
      xy_msg.data.push_back(static_cast<float>(y.at(i)));
    }

    return xy_msg;
  };

  traj_yaw_vs_s_pub_->publish(make_xy_msg("pair:s,yaw", s, yaw));
  traj_kappa_vs_s_pub_->publish(make_xy_msg("pair:s,kappa", s, kappa));
  traj_beta_vs_s_pub_->publish(make_xy_msg("pair:s,beta", s, beta));
}

bool MotionController::isDataReady()
{
  if(!trajectory_){
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for trajectory...");
    return false;
  }

  if(!odometry_ || !is_odometry_new){
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for odometry...");
    return false;
  }

  return true;
}

void MotionController::Vehicle_Init(){
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
}

//请求地图服务器的连接，并且
template <typename T>
bool MotionController::connect_server(const T &client)
{
  std::string server_name;
  if constexpr (std::is_same_v<T, rclcpp::Client<MapCreatorService>::SharedPtr>)  //C++17引入的
  {
      server_name = "map";
  } else if constexpr (std::is_same_v<T, rclcpp::Client<TrajCreatorService>::SharedPtr>)
  {
      server_name = "trajectory";
  } else
  {
      RCLCPP_ERROR(this->get_logger(), "Invalid client type");
      return false;
  }

  //等待服务器
  using namespace std::literals::chrono_literals;
  while (!client->wait_for_service(1000ms))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the %s server.", server_name.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "%s server not available, waiting again...", server_name.c_str());
  }

  return true;
}

bool MotionController::map_request()
{
  RCLCPP_INFO(this->get_logger(), "Sending map request/");

  auto request = std::make_shared<MapCreatorService::Request>();

  // 在request中填入T字型边界所需要的8个点的坐标
  const double bias_x = 0.0;
  const double bias_y = 0.0;

  geometry_msgs::msg::Pose p;
  p.orientation.w = 1.0;
  for(size_t i=0; i<8; i++){
    p.position.x = map_edge_points[i][0] + bias_x;
    p.position.y = map_edge_points[i][1] + bias_y;
    p.position.z = map_edge_points[i][2];
    request->pose.emplace_back(p);
  }

  // 获取响应
  auto result_furture = map_client_->async_send_request(request);

  // 判断响应是否成功
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_furture) == rclcpp::FutureReturnCode::SUCCESS)
  {
      RCLCPP_INFO(this->get_logger(), "global_path response success");
      return true;
  }
  else
  {
      RCLCPP_ERROR(this->get_logger(), "Global path response failed");
      return false;
  }

  return false;
}

void MotionController::srv_upper_test_callback(const srvUpperTest::Request::SharedPtr request, srvUpperTest::Response::SharedPtr response)
{
    upper_test.index = request->index;
    upper_test.enable = request->enable;
    upper_test.is_new = request->is_new;
    upper_test.bool_cmd = request->bool_cmd;
    upper_test.int_cmd = request->int_cmd;
    upper_test.float_cmd = request->float_cmd;

    response->result = 0;
}

}  // namespace motion_controller
