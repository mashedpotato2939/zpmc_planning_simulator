#include "trajectory_tracker.hpp"

namespace motion_controller
{
TrajectoryTracker::TrajectoryTracker()
{
    is_odometry_new = false;

    param.omega = 20.0;                  // 特征频率，控制估计器的响应速度
    param.dt = 0.02;                    // 采样时间间隔
    param.wheel_base = 2.3;              // 轴距
    param.wheel_base_ratio = 2.3;       // 二三轴轴距与一四轴轴距的比值
    param.max_steering_angle = 0.55;     // 最大转向角，按照一四轴最大幅度计算（33度左右）
    param.delta_limit = param.max_steering_angle;           
    param.d_delta_limit = param.max_steering_angle; 
    param.predicted_ts = 0.0;           // 预测时间，用于计算预瞄点位置
    param.ky = -0.5;                     // 预瞄点位置横向误差的增益
    param.kvy = 0.0;                    // 预瞄点位置横向速度误差的增益
    param.k_com = 0.0;                 // 速度前馈增益
    param.kyaw = -0.5;                   // 预瞄点位置航向误差的增益
    param.vx_min = 0.2;                 // 比例系数中的最小线速度，防止在低速时控制指令过大
    param.is_forward = false;
    state_x_ = {};
    state_y_ = {};
    state_yaw_ = {};
    state_ey_ = {};
    has_yaw_measurement_ = false;
    last_yaw_measurement_ = 0.0;
    unwrapped_yaw_measurement_ = 0.0;

    // 基于极点配置设计：极点位于 s = -ω (三重极点)
    // 特征方程: (s + ω)^3 = s^3 + 3ω s^2 + 3ω^2 s + ω^3 = 0
    // 控制增益: k1 = ω^3, k2 = 3ω^2, k3 = 3ω
    gains_.k1 = std::pow(param.omega, 3);
    gains_.k2 = 3.0 * std::pow(param.omega, 2);
    gains_.k3 = 3.0 * param.omega;

    param.omega = 5.0;
    gains_ey_.k1 = std::pow(param.omega, 3);
    gains_ey_.k2 = 3.0 * std::pow(param.omega, 2);
    gains_ey_.k3 = 3.0 * param.omega;
}

bool TrajectoryTracker::run(){
    
    //首先检查是否收到轨迹和定位信号
    if(!checkData()){
        return false;
    }

    // 对轨迹进行预处理
    Trajectory traj_processed = traj_preprocess(*trajectory_, param.is_forward);

    // 对纵向速度、横向速度、航向角速度进行三阶线性跟踪微分器估计
    update_differentiator(*odometry_);
    double global_vx = state_x_.x2;  // 纵向速度估计
    double global_vy = state_y_.x2;  // 横向速度估计
    double yaw = state_yaw_.x1;
    double current_vx = global_vx * std::cos(yaw) + global_vy * std::sin(yaw);  // 将全局速度转换为车体坐标系下的纵向速度
    double current_vy = -global_vx * std::sin(yaw) + global_vy * std::cos(yaw);  // 将全局速度转换为车体坐标系下的横向速度

    vx_debug = current_vx;

    Odometry odom_filter;
    if(param.is_forward)
    {
        odom_filter.pose.pose.position.x = state_x_.x1;
        odom_filter.pose.pose.position.y = state_y_.x1;
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        odom_filter.pose.pose.orientation = tf2::toMsg(q);
        odom_filter.twist.twist.linear.x = current_vx;
        odom_filter.twist.twist.linear.y = current_vy;
        odom_filter.twist.twist.angular.z = state_yaw_.x2;
    }else
    {
        odom_filter.pose.pose.position.x = state_x_.x1;
        odom_filter.pose.pose.position.y = state_y_.x1;
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw + M_PI);
        odom_filter.pose.pose.orientation = tf2::toMsg(q);
        odom_filter.twist.twist.linear.x = -current_vx;
        odom_filter.twist.twist.linear.y = -current_vy;
        odom_filter.twist.twist.angular.z = state_yaw_.x2;
    }

    // 计算轨迹最近点，读取期望纵向速度
    // output_tp_array_.resize(trajectory_->points.size());
    // std::copy(trajectory_->points.begin(), trajectory_->points.end(), output_tp_array_.begin());
    output_tp_array_.resize(traj_processed.points.size());
    std::copy(traj_processed.points.begin(), traj_processed.points.end(), output_tp_array_.begin());
    closest_idx_result = findNearestIndex(output_tp_array_, odom_filter, 2.0, M_PI_4);
    if (!closest_idx_result) {
        std::cerr << "[ERROR] cannot find closest waypoint\n";
        return false;
    }
    const size_t ind = std::min(*closest_idx_result + 2, traj_processed.points.size() - 1);
    cmd.velocity_cmd = std::fabs(output_tp_array_.at(ind).longitudinal_velocity_mps);
    yawr_debug = tf2::getYaw(output_tp_array_.at(ind).pose.orientation);
    yaw_debug = tf2::getYaw(odom_filter.pose.pose.orientation);

    // 计算预瞄点
    Odometry preview_odom = calc_preview_point(param.predicted_ts, odom_filter);
    
    // 计算误差
    double ey, e_yaw, betar, kappar;
    if (!errorCalc(output_tp_array_, preview_odom, ey, e_yaw, betar, kappar)) {
        std::cerr << "[ERROR] error calculation failed\n";
        return false;
    }
    betar_debug = betar;
    kappar_debug = kappar;

    // 计算横向误差微分项
    third_order_tracking_differentiator(ey, state_ey_, gains_);
    double ey_dot = state_ey_.x2;  // 横向误差的速度估计

    double wr = odom_filter.twist.twist.angular.z;  // 当前航向角速度
    auto cmd_tmp = calcSteerCmd(ey, ey_dot, e_yaw, betar, kappar, wr, current_vx);

    cmd.axis1_steer_cmd = cmd_tmp.axis1_steer_cmd;
    cmd.axis2_steer_cmd = cmd_tmp.axis2_steer_cmd;
    cmd.axis3_steer_cmd = cmd_tmp.axis3_steer_cmd;
    cmd.axis4_steer_cmd = cmd_tmp.axis4_steer_cmd;

    return true;
}

bool TrajectoryTracker::checkData()
{
    //是否接收到一条轨迹
    if(!trajectory_){   
        std::cerr << "[ERROR] there is no trajectory\n";
        return false;
    }

    //定位数据是否更新（定位数据更新是比控制执行更快的）
    if(odometry_ && is_odometry_new){
        is_odometry_new = false;
    } else{
        std::cerr << "[ERROR] the odometry does not update\n";
        return false;
    }

    return true;
}

Trajectory TrajectoryTracker::traj_preprocess(Trajectory traj, bool is_forward)
{
    Trajectory traj_output = traj;

    // 计算轨迹点的累积路径长度 
    std::vector<double> s_array(traj.points.size(), 0.0);
    for(size_t i=1; i< traj.points.size(); ++i){
        const auto & prev_point = traj.points.at(i-1);
        const auto & curr_point = traj.points.at(i);
        double dx = curr_point.pose.position.x - prev_point.pose.position.x;
        double dy = curr_point.pose.position.y - prev_point.pose.position.y;
        s_array.at(i) = s_array.at(i-1) + std::hypot(dx, dy);
    }

    // 将轨迹点纵向速度都取绝对值，确保在后续计算中速度符号不会引起问题
    for(size_t i=0; i< traj.points.size(); ++i){
        traj_output.points.at(i).longitudinal_velocity_mps = std::fabs(traj.points.at(i).longitudinal_velocity_mps);
    }

    // 每个点的实际航向角
    std::vector<double> yaw_array(traj.points.size(), 0.0);
    if(is_forward)
    {
        for(size_t i=0; i< traj.points.size(); ++i){
            yaw_array.at(i) = tf2::getYaw(traj.points.at(i).pose.orientation);
        }
    }else
    {
        for(size_t i=0; i< traj.points.size(); ++i){
            yaw_array.at(i) = tf2::getYaw(traj.points.at(i).pose.orientation);
            yaw_array.at(i) = normalize_angle(yaw_array.at(i) + M_PI);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw_array.at(i));
            traj_output.points.at(i).pose.orientation = tf2::toMsg(q);
        }
    }
    
    // lateral_velocity_mps is intentionally NOT re-derived here. errorCalc computes
    // the crab feedforward on-demand via crabAt() from pose.orientation vs path
    // tangent (matching the velocity_smoother contract for OBLIQUE/Q_TURN, where
    // the field arrives zeroed and the crab angle is encoded in orientation).

    // 计算每个点的曲率
    std::vector<double> kappa(traj.points.size(), 0.0);
    const double min_ds = 0.01;  // 最小路径长度差，防止除以零或过小导致数值不稳定
    // 第一个点的曲率使用前向差分计算
    kappa.at(0) = normalize_angle(yaw_array.at(1) - yaw_array.at(0)) / std::max((s_array.at(1) - s_array.at(0)), min_ds);

    // 中间点的曲率使用中心差分计算
    for(size_t i=1; i< traj.points.size()-1; ++i){
        kappa.at(i) = normalize_angle(yaw_array.at(i+1) - yaw_array.at(i-1)) / std::max((s_array.at(i+1) - s_array.at(i-1)), min_ds);
    }

    // 最后一个点的曲率使用后向差分计算
    kappa.at(traj.points.size()-1) = normalize_angle(yaw_array.at(traj.points.size()-1) - yaw_array.at(traj.points.size()-2)) / std::max((s_array.at(traj.points.size()-1) - s_array.at(traj.points.size()-2)), min_ds);

    // 计算每个点的航向角速度
    for(size_t i=0; i< traj.points.size(); ++i){
        double vx = std::fabs(traj.points.at(i).longitudinal_velocity_mps);
        traj_output.points.at(i).heading_rate_rps = vx * kappa.at(i);
    }
    
    return traj_output;
}

Odometry TrajectoryTracker::calc_preview_point(double predicted_ts, Odometry current_odom)
{
    double x = current_odom.pose.pose.position.x;
    double y = current_odom.pose.pose.position.y;
    tf2::Quaternion q_tf;
    tf2::fromMsg(current_odom.pose.pose.orientation, q_tf);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);

    double vx = current_odom.twist.twist.linear.x;
    double vy = current_odom.twist.twist.linear.y;
    double v_yaw = current_odom.twist.twist.angular.z;

    double x_preview = x + vx * predicted_ts * std::cos(yaw) - vy * predicted_ts * std::sin(yaw);
    double y_preview = y + vx * predicted_ts * std::sin(yaw) + vy * predicted_ts * std::cos(yaw);
    double yaw_preview = yaw + v_yaw * predicted_ts;

    Odometry preview_odom;
    preview_odom.pose.pose.position.x = x_preview;
    preview_odom.pose.pose.position.y = y_preview;
    preview_odom.pose.pose.position.z = 0.0;
    tf2::Quaternion q_preview;
    q_preview.setRPY(0.0, 0.0, yaw_preview);
    preview_odom.pose.pose.orientation = tf2::toMsg(q_preview);
    preview_odom.twist.twist.linear.x = vx;
    preview_odom.twist.twist.linear.y = vy;
    preview_odom.twist.twist.angular.z = v_yaw;
    
    return preview_odom;
}

TrackerCmd TrajectoryTracker::calcSteerCmd(double ey, double ey_dot, double e_yaw, double betar, double kappar, double wr, double vx)
{
    // vx_safe is the SPEED used to scale proportional gains. In reverse the
    // body-frame longitudinal velocity is negative, so use |vx| — otherwise
    // std::max(negative, vx_min) collapses to vx_min and the gain becomes
    // ky_reverse / vx_min instead of ky_reverse / |vx| (~24× over-gain at
    // cruise → saturated bang-bang steering).
    double vx_safe = std::max(std::abs(vx), param.vx_min);
    double d_delta = 0.0;
    double delta = 0.0;
    double d_deltar = atan(param.wheel_base * kappar / 2);
    if(!param.is_forward)
    {
        // betar is +betar here (not -betar like the forward branch) because
        // the per-axis sign flip at the bottom of this function inverts the
        // whole cmd for reverse. Net result in axis_*_cmd:
        //   forward cmd = +ky*ey + ... - betar
        //   reverse cmd = +ky*ey + ... - betar   (same physical mapping)
        // crabAt() doesn't know which direction it's in — it returns the same
        // value for the same body/tangent geometry — so the +/- must be flipped
        // here to compensate for the downstream axis flip. Without this, reverse
        // OBLIQUE crab steers the wheels with the wrong sign and the vehicle
        // drifts off-trajectory until findNearestIndex aborts the tracker.
        delta = -param.ky_reverse/vx_safe*ey - (param.kvy/vx_safe) * ey_dot + betar - param.k_com*(vx*wr);
        d_delta = param.kyaw_reverse / vx_safe * e_yaw + d_deltar;
    }else
    {
        delta = param.ky/vx_safe*ey + (param.kvy/vx_safe) * ey_dot - betar + param.k_com*(vx*wr);
        //delta = param.ky/vx_safe*ey + (param.kvy/vx_safe) * ey_dot;
        d_delta = param.kyaw / vx_safe * e_yaw + d_deltar;
        // d_delta = param.kyaw / vx_safe * e_yaw;
    }

    auto delta_l = static_cast<float>(std::min(std::max(delta, -param.delta_limit), param.delta_limit));
    auto d_delta_l = static_cast<float>(std::min(std::max(d_delta, -param.d_delta_limit), param.d_delta_limit));

    ey_debug = ey; 
    eyaw_debug= e_yaw;
    delta_debug = delta;
    deltal_debug = delta_l;
    d_delta_debug = d_delta;
    d_deltal_debug = d_delta_l;

    TrackerCmd cmd;
    cmd.axis2_steer_cmd = delta_l + d_delta_l/param.wheel_base_ratio;
    cmd.axis3_steer_cmd = delta_l - d_delta_l/param.wheel_base_ratio;
    cmd.axis1_steer_cmd = delta_l + d_delta_l;
    cmd.axis4_steer_cmd = delta_l - d_delta_l;
    if(!param.is_forward)
    {
        cmd.axis2_steer_cmd = -cmd.axis2_steer_cmd;
        cmd.axis3_steer_cmd = -cmd.axis3_steer_cmd;
        cmd.axis1_steer_cmd = -cmd.axis1_steer_cmd;
        cmd.axis4_steer_cmd = -cmd.axis4_steer_cmd;
    }

    cmd.axis1_steer_cmd = std::min(std::max(cmd.axis1_steer_cmd, -param.max_steering_angle), param.max_steering_angle);
    cmd.axis2_steer_cmd = std::min(std::max(cmd.axis2_steer_cmd, -param.max_steering_angle), param.max_steering_angle);
    cmd.axis3_steer_cmd = std::min(std::max(cmd.axis3_steer_cmd, -param.max_steering_angle), param.max_steering_angle);
    cmd.axis4_steer_cmd = std::min(std::max(cmd.axis4_steer_cmd, -param.max_steering_angle), param.max_steering_angle);

    return cmd;
}


std::optional<size_t> TrajectoryTracker::findNearestIndex(const std::vector<TrajectoryPoint> &points, Odometry preview_point,
    double max_dist, double max_yaw)
{
    (void)max_yaw;

    if (points.empty()) {
        throw std::invalid_argument("轨迹点序列为空");
    }

  // sq_dist is squared distance; compare against max_dist² so max_dist keeps
  // its intended meaning (a linear radius in metres). The earlier code
  // compared squared distance to the linear threshold, shrinking the effective
  // radius from max_dist to sqrt(max_dist) (e.g. 2.0 → ~1.41 m).
  const double max_sq_dist = max_dist * max_dist;
  double min_sq_dist = std::numeric_limits<double>::max();
  bool is_nearest_found = false;
  size_t min_idx = 0;

  double preview_x = preview_point.pose.pose.position.x;
  double preview_y = preview_point.pose.pose.position.y;

  for (size_t i = 0; i < points.size(); ++i) {
    const double dx = points.at(i).pose.position.x - preview_x;
    const double dy = points.at(i).pose.position.y - preview_y;
    const double sq_dist = dx * dx + dy * dy;
    if (sq_dist > max_sq_dist || sq_dist >= min_sq_dist) {
      continue;
    }
    min_sq_dist = sq_dist;
    min_idx = i;
    is_nearest_found = true;
  }

  if (is_nearest_found) {
    return min_idx;
  }
  return std::nullopt;
}

// 辅助函数：计算有符号角度差，结果在 [-pi, pi]
static double angleDiff(double angle1, double angle2) {
    double d = std::fmod(angle2 - angle1 + M_PI, 2.0 * M_PI);
    if (d < 0) d += 2.0 * M_PI;
    return d - M_PI;
}

// 辅助函数：横向误差 (小车相对于投影点，沿法线方向)
static double computeLateralError(double xo, double yo, double xt, double yt, double yaw_t) {
    double dx = xt - xo;
    double dy = yt - yo;
    double nx = -std::sin(yaw_t);
    double ny = std::cos(yaw_t);
    return dx * nx + dy * ny;
}

// Path tangent at trajectory index k. Mirrors traj_preprocess yawc_array:
// forward diff at start, central diff in middle, backward diff at end.
static double tangentAt(const std::vector<TrajectoryPoint> & pts, size_t k) {
    const size_t n = pts.size();
    size_t a, b;
    if (k == 0)            { a = 0;     b = 1;     }
    else if (k + 1 >= n)   { a = n - 2; b = n - 1; }
    else                   { a = k - 1; b = k + 1; }
    return std::atan2(
        pts.at(b).pose.position.y - pts.at(a).pose.position.y,
        pts.at(b).pose.position.x - pts.at(a).pose.position.x);
}

// Crab-angle feedforward (slip ratio) at trajectory point k. In OBLIQUE/Q_TURN
// the velocity_smoother zeros lateral_velocity_mps and re-injects the constant
// body yaw into pose.orientation, so the crab angle lives in the gap between
// the per-point body yaw and the geometric path tangent.
//
// Returns tan(body_yaw - tangent) — positive when the path tangent lies to the
// body-RIGHT of body_yaw (body_yaw CCW of tangent → body_yaw - tangent > 0).
//
// Forward: a positive crabAt means the vehicle must crab body-RIGHT, i.e. δ < 0.
// calcSteerCmd's `delta = ... - betar` line produces δ < 0 for betar > 0. ✓
//
// Reverse: pts[k].pose.orientation has already been π-flipped by traj_preprocess,
// so body_yaw here is the *motion* direction, not the body-facing direction.
// crabAt returns the same number it would for the equivalent forward maneuver,
// BUT calcSteerCmd applies a per-axis sign flip at the bottom (cmd = -delta) on
// top of the reverse branch's already-flipped feedback signs. To make the crab
// feedforward come out with the same sign in axis_*_cmd as in forward, the
// reverse branch uses `+betar` instead of `-betar`. See the comment in
// calcSteerCmd for the cancellation arithmetic.
static double crabAt(const std::vector<TrajectoryPoint> & pts, size_t k) {
    const double body_yaw = tf2::getYaw(pts.at(k).pose.orientation);
    const double tangent  = tangentAt(pts, k);
    return std::tan(angleDiff(tangent, body_yaw));
}

bool TrajectoryTracker::errorCalc(const std::vector<TrajectoryPoint> &points, Odometry preview_point, 
    double &ey, double &eyaw, double &beta_t, double &kappa)
{
    if (points.size() < 2) {
        ey = 0.0; eyaw = 0.0; beta_t = 0.0; kappa = 0.0;
        return false;
    }

    // 1. 寻找最近点
    closest_idx_result = findNearestIndex(points, preview_point, 4.0, M_PI_4);
    if (!closest_idx_result) {
        std::cerr << "[ERROR] cannot find preview closest waypoint\n";
        return false;
    }

    double xs = points.at(*closest_idx_result).pose.position.x;
    double ys = points.at(*closest_idx_result).pose.position.y;
    double yaw_s = tf2::getYaw(points.at(*closest_idx_result).pose.orientation);
    double vx = points.at(*closest_idx_result).longitudinal_velocity_mps;

    double xo = preview_point.pose.pose.position.x;
    double yo = preview_point.pose.pose.position.y;
    double yaw_o = tf2::getYaw(preview_point.pose.pose.orientation);

    // 2. 判断车辆位于最近点的前方还是后方
    double dx_to_car = xo - xs;
    double dy_to_car = yo - ys;
    double dir_x = std::cos(yaw_s);
    double dir_y = std::sin(yaw_s);
    bool isAhead = (dx_to_car * dir_x + dy_to_car * dir_y) > 0.0;

    int idxP, idxQ;
    size_t nearestIdx = *closest_idx_result;
    size_t n = points.size();
    if (isAhead) {
        // 前方：使用下一个点
        if (nearestIdx + 1 >= n) {
            // 已是最后一个点，直接返回最近点本身
            double xt = xs; 
            double yt = ys; 
            double yaw_t = yaw_s; 
            
            beta_t = crabAt(points, nearestIdx);
            kappa = points.at(nearestIdx).heading_rate_rps / std::max(0.1, vx);
            ey = computeLateralError(xo, yo, xt, yt, yaw_t);
            eyaw = angleDiff(yaw_o, yaw_t);
            return true;
        }
        idxP = nearestIdx;
        idxQ = nearestIdx + 1;
    } else {
        // 后方：使用上一个点
        if (nearestIdx == 0) {
            // 已是第一个点，直接返回最近点本身
            double xt = xs; 
            double yt = ys; 
            double yaw_t = yaw_s; 

            beta_t = crabAt(points, nearestIdx);
            kappa = 0.0;
            ey = computeLateralError(xo, yo, xt, yt, yaw_t);
            eyaw = angleDiff(yaw_o, yaw_t);
            return true;
        }
        idxP = nearestIdx - 1;
        idxQ = nearestIdx;
    }

    double Px = points.at(idxP).pose.position.x;
    double Py = points.at(idxP).pose.position.y;
    double Pyaw = tf2::getYaw(points.at(idxP).pose.orientation);
    double Pbeta = crabAt(points, idxP);
    double Pkappa = points.at(idxP).heading_rate_rps / std::max(0.1, vx);

    double Qx = points.at(idxQ).pose.position.x;
    double Qy = points.at(idxQ).pose.position.y;
    double Qyaw = tf2::getYaw(points.at(idxQ).pose.orientation);
    double Qbeta = crabAt(points, idxQ);
    double Qkappa = points.at(idxQ).heading_rate_rps / std::max(0.1, vx);

    // 3. 建立直线方程求投影点
    double dx_l2 = Qx - Px;
    double dy_l2 = Qy - Py;

    double denominator = std::cos(yaw_s) * dx_l2 + std::sin(yaw_s) * dy_l2;
    double u = 0.0;
    if (std::abs(denominator) > 1e-8) {
        double numerator = -(std::cos(yaw_s) * (Px - xo) + std::sin(yaw_s) * (Py - yo));
        u = numerator / denominator;
        u = std::max(0.0, std::min(1.0, u));
    }
    double xt = Px + u * dx_l2;
    double yt = Py + u * dy_l2;
    double yaw_t = Pyaw + u * normalize_angle(Qyaw - Pyaw);
    
    beta_t = Pbeta + u * (Qbeta - Pbeta);
    kappa = Pkappa + u * (Qkappa - Pkappa);
    ey = computeLateralError(xo, yo, xt, yt, yaw_t);
    eyaw = angleDiff(yaw_o, yaw_t);
    yawr_debug = yaw_t;
    yaw_debug = yaw_o;

    return true;
}

void TrajectoryTracker::update_differentiator(const nav_msgs::msg::Odometry &odom)
{   
    // 获取当前测量值
    double x_meas = odom.pose.pose.position.x;
    double y_meas = odom.pose.pose.position.y;
    double yaw_meas = get_yaw_from_quaternion(odom.pose.pose.orientation);

    yaw_meas = normalize_angle(yaw_meas);
    if (!has_yaw_measurement_) {
        has_yaw_measurement_ = true;
        last_yaw_measurement_ = yaw_meas;
        unwrapped_yaw_measurement_ = yaw_meas;
        state_yaw_.x1 = yaw_meas;
        state_yaw_.x2 = odom.twist.twist.angular.z;
        state_yaw_.x3 = 0.0;
        state_yaw_.u = 0.0;
    } else {
        unwrapped_yaw_measurement_ += normalize_angle(yaw_meas - last_yaw_measurement_);
        last_yaw_measurement_ = yaw_meas;
    }
    
    // 更新各个维度的微分器
    third_order_tracking_differentiator(x_meas, state_x_, gains_);
    third_order_tracking_differentiator(y_meas, state_y_, gains_);
    third_order_tracking_differentiator(unwrapped_yaw_measurement_, state_yaw_, gains_);
}

void TrajectoryTracker::third_order_tracking_differentiator(
    double measured_value,
    ThirdOrderState& state,
    const EstimatorGains& gains)
{
    // 三阶线性跟踪微分器
    // x1(k+1) = x1(k) + Ts * x2(k)
    // x2(k+1) = x2(k) + Ts * x3(k)
    // x3(k+1) = x3(k) + Ts * u(k)
    // u(k) = k1 * (v(k) - x1(k)) - k2 * x2(k) - k3 * x3(k)
    double dt = param.dt;
    double error = measured_value - state.x1;
    state.u = gains.k1 * error - gains.k2 * state.x2 - gains.k3 * state.x3;
    state.x3 = state.x3 + dt * state.u;
    state.x2 = state.x2 + dt * state.x3;
    state.x1 = state.x1 + dt * state.x2;
}

double TrajectoryTracker::get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quat)
{
    // 使用tf2的函数从四元数中提取yaw角
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 mat(tf_quat);
    
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    
    return yaw;
}

double TrajectoryTracker::normalize_angle(double angle)
{
    // 将角度归一化到[-π, π]
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

}
