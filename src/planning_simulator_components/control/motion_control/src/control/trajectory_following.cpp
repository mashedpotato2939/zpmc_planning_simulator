#include "trajectory_following.hpp"

namespace motion_controller
{
TrajectoryFollowing::TrajectoryFollowing()
{
    //初始化各个参数值
    use_wp_replace_tgt_pose = false;
    is_trajectory_new = false;
    is_odometry_new = false;

    param.wheel_base = 6.0;
    param.max_steering_angle = 20.0;
    param.resampling_ds = 0.1;
    param.ld_velocity_ratio = 3.0;
    param.ld_curvature_ratio = 60.0;
    param.ld_lateral_error_ratio = 3.6;
    param.long_ld_lateral_error_threshold = 0.5;
    param.max_lookahead_distance = 10.0;
    param.min_lookahead_distance = 1.0;
    param.curvature_calculation_distance = 2.0;
    param.min_traj_points_size = 10;
    param.max_traj_points_interval = 1.0;
}

bool TrajectoryFollowing::run()
{
    //首先检查是否收到轨迹和定位信号
    if(!checkData()){
        return false;
    }

    //如果是新的轨迹先对轨迹插值，使其为等间隔密集的轨迹点
    // if(is_trajectory_new){
        resampled_trajectory = resampleTrajectory(*trajectory_, param.resampling_ds);
        output_tp_array_.resize(resampled_trajectory.points.size());
        std::copy(resampled_trajectory.points.begin(), resampled_trajectory.points.end(), output_tp_array_.begin());
    // }

    //找到最临近点，将该点纵向速度作为车速命令
    closest_idx_result = control_utils::trajectory::findNearestIndex(output_tp_array_, odometry_->pose.pose, 3.0, M_PI_4);
    if (!closest_idx_result) {
        std::cerr << "[ERROR] cannot find closest waypoint\n";
        return false;
    }
    tf_cmd.velocity_cmd = output_tp_array_.at(*closest_idx_result).longitudinal_velocity_mps;

    //找到预瞄点
    std::pair<bool, geometry_msgs::msg::Point> preview_point_pair = FindLookaheadPoint(odometry_->pose.pose);
    if(!preview_point_pair.first){
        std::cerr << "[ERROR] cannot find preview point\n";
        return false;
    }    
    preview_point = preview_point_pair.second;

    //根据几何关系求出转向角
    tf_cmd.steer_cmd = calcSteerCmd(preview_point, odometry_->pose.pose);

    return true;
}

bool TrajectoryFollowing::checkData()
{
    /*轨迹检查*/
    //是否接收到一条轨迹
    if(!trajectory_){   
        std::cerr << "[ERROR] there is no trajectory\n";
        return false;
    }

    //轨迹点数、点之间的间隔需要符合要求
    if(is_trajectory_new){

        is_trajectory_new = false;

        if(trajectory_->points.size() >= param.min_traj_points_size)
        {
          auto n = trajectory_->points.size();
          for(size_t i=0; i<trajectory_->points.size()-1; i++){
            auto p1 = trajectory_->points.at(i);
            auto p2 = trajectory_->points.at(i+1).pose.position;
            auto dist = control_utils::geometry::calcSquaredDistance2d(p1,p2);
            if(dist > param.max_traj_points_interval){
                trajectory_ = nullptr;
                std::cerr << "[ERROR] the interval of trajectory points is too large, idx:" << i <<"size:" << n <<"\n";
                return false;
            }
            
            //如果连续两个点距离非常近则跳过，防止停车点多个点重叠在一起
            if(dist < 1E-3){
                continue;
            }
            //取消倒车模式，要求轨迹一定需要正向运动
            const bool is_driving_forward = 
                control_utils::geometry::isDrivingForward(trajectory_->points.at(i), trajectory_->points.at(i+1));
            if(!is_driving_forward || trajectory_->points.at(i).longitudinal_velocity_mps < 0){
                trajectory_ = nullptr;
                std::cerr << "[ERROR] motion must be forward\n";
                return false;
            }
          }
        } else{
            trajectory_ = nullptr;
            std::cerr << "[ERROR] the size of trajectory points is not enough\n";
            return false;
        }
    }

    /*定位检查*/
    //定位数据是否更新（定位数据更新是比控制执行更快的）
    if(odometry_ && is_odometry_new){
        is_odometry_new = false;
    } else{
        std::cerr << "[ERROR] the odometry does not update\n";
        return false;
    }

    //其他定位数据有效性检查，例如范围、多个定位数据间的连续性

    return true;
}

Trajectory TrajectoryFollowing::resampleTrajectory(
    const autoware_planning_msgs::msg::Trajectory & input_trajectory, const double resample_interval)
{
    //计算轨迹总的长度：累加各个轨迹点之间的距离
    double input_trajectory_len = 0.0;
    double input_trajectory_points_size = input_trajectory.points.size();
    for(size_t i=0; i<input_trajectory_points_size-1; i++){
        input_trajectory_len += control_utils::geometry::calcDistance2d(
            input_trajectory.points.at(i).pose.position, input_trajectory.points.at(i+1).pose.position);
    }

    //按照间隔长度将轨迹等间距分割
    std::vector<double> resampling_arclength;
    for (double s = 0.0; s < input_trajectory_len; s += resample_interval) {
        resampling_arclength.push_back(s);
    }
    
    //插入轨迹终止点——terminal points
    if (input_trajectory_len - resampling_arclength.back() <0.1) {
        resampling_arclength.back() = input_trajectory_len;
    } else{
        resampling_arclength.push_back(input_trajectory_len);
    }

    //输入轨迹信息
    std::vector<double> input_arclength;
    std::vector<geometry_msgs::msg::Pose> input_pose;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    std::vector<double> v_lon;
    std::vector<double> acceleration;
    std::vector<double> time_from_start;

    //给容器预先申请长度
    input_arclength.reserve(input_trajectory_points_size);
    input_pose.reserve(input_trajectory_points_size);
    x.reserve(input_trajectory_points_size);
    y.reserve(input_trajectory_points_size);
    z.reserve(input_trajectory_points_size);
    v_lon.reserve(input_trajectory_points_size);
    acceleration.reserve(input_trajectory_points_size);
    time_from_start.reserve(input_trajectory_points_size);

    //初始化
    input_arclength.push_back(0.0);
    input_pose.push_back(input_trajectory.points.front().pose);
    x.push_back(input_pose.front().position.x);
    y.push_back(input_pose.front().position.y);
    z.push_back(input_pose.front().position.z);
    v_lon.push_back(input_trajectory.points.front().longitudinal_velocity_mps);
    acceleration.push_back(input_trajectory.points.front().acceleration_mps2);
    time_from_start.push_back(
        rclcpp::Duration(input_trajectory.points.front().time_from_start).seconds());
    
    //为每个轨迹点赋值
    for (size_t i = 1; i < input_trajectory.points.size(); ++i) {
        const auto & prev_pt = input_trajectory.points.at(i - 1);
        const auto & curr_pt = input_trajectory.points.at(i);
        const double ds = control_utils::geometry::calcDistance2d(prev_pt.pose.position, curr_pt.pose.position);

        input_arclength.push_back(ds + input_arclength.back());
        input_pose.push_back(curr_pt.pose);
        x.push_back(curr_pt.pose.position.x);
        y.push_back(curr_pt.pose.position.y);
        z.push_back(curr_pt.pose.position.z);
        v_lon.push_back(curr_pt.longitudinal_velocity_mps);
        acceleration.push_back(curr_pt.acceleration_mps2);
        time_from_start.push_back(rclcpp::Duration(curr_pt.time_from_start).seconds());
    }

    //插入停车点——stop points，如果某个点的纵向速度小于0.1，就将该点及以后的速度都设置为0
    bool stop_point_found_in_v_lon = false;
    constexpr double epsilon = 0.09;
    for (size_t i = 0; i < v_lon.size(); ++i) {
        if (std::abs(v_lon.at(i)) < epsilon) {
            stop_point_found_in_v_lon = true;
        }
        if (stop_point_found_in_v_lon) {
            v_lon.at(i) = 0.0;
        }
    }

  // 定义插值lamda函数
  const auto lerp = [&](const auto & input) {
    return control_utils::interpolation::lerp(input_arclength, input, resampling_arclength);
  };

  std::vector<size_t> closest_segment_indices;
    closest_segment_indices = control_utils::interpolation::calc_closest_segment_indices(input_arclength, resampling_arclength);
  const auto zoh = [&](const auto & input) {
    return control_utils::interpolation::zero_order_hold(input_arclength, input, closest_segment_indices);
  };

  const auto spline_by_akima = [&](const auto & input) {
    return control_utils::interpolation::splineByAkima(input_arclength, input, resampling_arclength);
  };

  //为pose中的Point分量插值
  const auto interpolated_x = spline_by_akima(x);
  const auto interpolated_y = spline_by_akima(y);
  const auto interpolated_z = lerp(z);

  std::vector<geometry_msgs::msg::Pose> interpolated_pose;
  interpolated_pose.resize(interpolated_x.size());
  for (size_t i = 0; i < interpolated_pose.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = interpolated_x.at(i);
    point.y = interpolated_y.at(i);
    point.z = interpolated_z.at(i);
    interpolated_pose.at(i).position = point;
  }

  //为pose中的oriention分量插值
  const bool is_driving_forward = control_utils::geometry::isDrivingForward(input_trajectory.points.at(0), input_trajectory.points.at(1));
  insertOrientation(interpolated_pose, is_driving_forward);
  if (!is_driving_forward) {
    interpolated_pose.at(0).orientation = input_trajectory.points.at(0).pose.orientation;
  }

  //为twist中纵向速度和纵向加速度插值
  const auto interpolated_v_lon = zoh(v_lon);
  const auto interpolated_acceleration = zoh(acceleration);
  const auto interpolated_time_from_start = lerp(time_from_start);
  
  //将插值结果整合到Trajectroy的变量中
  Trajectory resampled_trajectory;
  resampled_trajectory.header = input_trajectory.header;
  resampled_trajectory.points.resize(interpolated_pose.size());
  for (size_t i = 0; i < resampled_trajectory.points.size(); ++i) {
    TrajectoryPoint traj_point;
    traj_point.pose = interpolated_pose.at(i);
    traj_point.longitudinal_velocity_mps = interpolated_v_lon.at(i);
    traj_point.acceleration_mps2 = interpolated_acceleration.at(i);
    traj_point.time_from_start = rclcpp::Duration::from_seconds(interpolated_time_from_start.at(i));
    resampled_trajectory.points.at(i) = traj_point;
  }

  return resampled_trajectory;

}

void TrajectoryFollowing::insertOrientation(std::vector<geometry_msgs::msg::Pose> & points, const bool is_driving_forward)
{
    if (is_driving_forward) {
    for (size_t i = 0; i < points.size() - 1; ++i) {
        const auto & src_point = points.at(i).position;
        const auto & dst_point = points.at(i + 1).position;
        const double pitch = control_utils::geometry::calcElevationAngle(src_point, dst_point);
        const double yaw = control_utils::geometry::calcAzimuthAngle(src_point, dst_point);
        control_utils::geometry::setOrientation(
            control_utils::geometry::createQuaternionFromRPY(0.0, pitch, yaw), points.at(i));
        if (i == points.size() - 2) {
        // Terminal orientation is same as the point before it
            control_utils::geometry::setOrientation(points.at(i).orientation, points.at(i + 1));
        }
    }
    } else {
    for (size_t i = points.size() - 1; i >= 1; --i) {
        const auto & src_point = points.at(i).position;
        const auto & dst_point = points.at(i + 1).position;
        const double pitch = control_utils::geometry::calcElevationAngle(src_point, dst_point);
        const double yaw = control_utils::geometry::calcAzimuthAngle(src_point, dst_point);
        control_utils::geometry::setOrientation(
            control_utils::geometry::createQuaternionFromRPY(0.0, pitch, yaw), points.at(i));
    }
    // Initial orientation is same as the point after it
        control_utils::geometry::setOrientation(points.at(1).orientation, points.at(0));
    }
}

double TrajectoryFollowing::calcCurvature(const size_t closest_idx)
{
    const size_t idx_dist = static_cast<size_t>(
        std::max(static_cast<int>((param.curvature_calculation_distance) / param.resampling_ds), 1));
    
    // 寻找轨迹中用于计算曲率的点
    size_t next_idx = output_tp_array_.size() - 1;
    size_t prev_idx = 0;

    if (static_cast<size_t>(closest_idx) >= idx_dist) {
        prev_idx = closest_idx - idx_dist;
    } else {
        return 0.0;
    }

    if (output_tp_array_.size() - 1 >= closest_idx + idx_dist) {
        next_idx = closest_idx + idx_dist;
    } else {
        return 0.0;
    }

    double current_curvature = 0.0;
    try {
        current_curvature = control_utils::geometry::calcCurvature(
          output_tp_array_.at(prev_idx).pose.position,
          output_tp_array_.at(closest_idx).pose.position,
          output_tp_array_.at(next_idx).pose.position);
    } catch (std::exception const & e) {
        std::cerr << "[WARN] [trajectory_following] " << e.what() << '\n';
        current_curvature = 0.0;
    }

    return current_curvature;
}

int32_t TrajectoryFollowing::findNextPointIdx(int32_t search_start_idx, geometry_msgs::msg::Pose pose)
{
    for (size_t i = search_start_idx; i < output_tp_array_.size(); i++) {
    // 如果运动到停车点附近，轨迹前方长度不够预瞄点的距离，把最后一点返回，让最后一段线段和前视圆相交计算出预瞄点
      if (i == (output_tp_array_.size() - 1)) {
        return i;
      }

    // if waypoint direction is forward
     
    // if waypoint is not in front of ego, skip
      auto ret = control_utils::geometry::transformToRelativeCoordinate2D(
        output_tp_array_.at(i).pose.position, pose);
      if (ret.x < 0) {
        continue;
      }

      const auto & curr_motion_point = output_tp_array_.at(i);
      const geometry_msgs::msg::Point & curr_pose_point = pose.position;
      // if there exists an effective waypoint
      const double ds = control_utils::geometry::calcSquaredDistance2d(curr_motion_point, curr_pose_point);
      if (ds > std::pow(lookahead_distance, 2)) {
        return i;
      }
    }

    // if this program reaches here , it means we lost the waypoint!
    return -1;
}

std::pair<bool, geometry_msgs::msg::Point> TrajectoryFollowing::lerpNextTarget(int32_t next_wp_idx, geometry_msgs::msg::Pose pose)
{
    constexpr double ERROR2 = 1e-5;  // 0.00001
    const geometry_msgs::msg::Point & vec_end = output_tp_array_.at(next_wp_idx).pose.position;
    const geometry_msgs::msg::Point & vec_start = output_tp_array_.at(next_wp_idx - 1).pose.position;
    const geometry_msgs::msg::Pose & curr_pose = pose;

    Eigen::Vector3d vec_a(
        (vec_end.x - vec_start.x), (vec_end.y - vec_start.y), (vec_end.z - vec_start.z));
    
    if (vec_a.norm() < ERROR2) {
        std::cerr << "[ERROR] waypoint interval is almost 0 " << '\n';
        return std::make_pair(false, geometry_msgs::msg::Point());
    }

    //求当前点到轨迹线段的横向误差
    tf2::Vector3 a_vec((vec_end.x - vec_start.x), (vec_end.y - vec_start.y), 0.0);
    tf2::Vector3 b_vec((curr_pose.position.x - vec_start.x), (curr_pose.position.y - vec_start.y), 0.0);
    const double lateral_error = (a_vec.length() > 0) ? a_vec.cross(b_vec).z() / a_vec.length() : 0.0;

    if (fabs(lateral_error) > lookahead_distance) {
        std::cerr << "[ERROR] lateral error is larger than lookahead distance, there is no intersection \n";
        return std::make_pair(false, geometry_msgs::msg::Point());
    }

    Eigen::Vector2d uva2d(vec_a.x(), vec_a.y());
    uva2d.normalize();                            //线段方向的单位向量
    Eigen::Rotation2Dd rot =
        (lateral_error > 0) ? Eigen::Rotation2Dd(-M_PI / 2.0) : Eigen::Rotation2Dd(M_PI / 2.0);
    Eigen::Vector2d uva2d_rot = rot * uva2d;      //垂直指向线段方向的单位向量

    geometry_msgs::msg::Point h;
    h.x = curr_pose.position.x + fabs(lateral_error) * uva2d_rot.x();
    h.y = curr_pose.position.y + fabs(lateral_error) * uva2d_rot.y();
    h.z = curr_pose.position.z;

    // 如果只有一个交点
    if (fabs(fabs(lateral_error) - lookahead_distance) < ERROR2) {
        return std::make_pair(true, h);
    } else {
        // 如果有两个交点，选择更靠近路径点前方的
        const double s = sqrt(pow(lookahead_distance, 2) - pow(lateral_error, 2));
        geometry_msgs::msg::Point res;
        res.x = h.x + s * uva2d.x();
        res.y = h.y + s * uva2d.y();
        res.z = curr_pose.position.z;
        return std::make_pair(true, res);
    }

}

double TrajectoryFollowing::CalcLookaheadDistance(
    const double lateral_error, const double curvature, const double velocity, const double min_ld)
{
    const double vel_ld = abs(param.ld_velocity_ratio * velocity);
    const double curvature_ld = -abs(param.ld_curvature_ratio * curvature);

    double lateral_error_ld = 0.0;
    if (abs(lateral_error) >= param.long_ld_lateral_error_threshold) {
        //如果横向误差太大，应该适当增大ld，以避免过大的航向误差
        lateral_error_ld = abs(param.ld_lateral_error_ratio * lateral_error);
    }

    const double total_ld =
        std::clamp(vel_ld + curvature_ld + lateral_error_ld, min_ld, param.max_lookahead_distance);
    
    return total_ld;
}

std::pair<bool, geometry_msgs::msg::Point> TrajectoryFollowing::FindLookaheadPoint(geometry_msgs::msg::Pose pose){
    
    //求横向误差
    const double lateral_error =
        control_utils::trajectory::calcLateralOffset(output_tp_array_, pose.position);
    
    lateral_error_d = lateral_error;
    
    //计算最近点轨迹的曲率
    const double current_curvature = calcCurvature(*closest_idx_result);

    //计算前视距离
    const double min_lookahead_distance = param.min_lookahead_distance;
    lookahead_distance = CalcLookaheadDistance(
        lateral_error, current_curvature, odometry_->twist.twist.linear.x, min_lookahead_distance);
    
    //找到距离预瞄点最近的途经点
    int32_t next_wp_idx = findNextPointIdx(*closest_idx_result, pose);
    if (next_wp_idx == -1) {
        std::cerr << "[WARN] lost next waypoint\n";
        return std::make_pair(false, geometry_msgs::msg::Point());
    }

    geometry_msgs::msg::Point loc_next_wp = output_tp_array_.at(next_wp_idx).pose.position;
    if(use_wp_replace_tgt_pose){
        return std::make_pair(true, loc_next_wp);
    }

    //通过前视圆和轨迹线段交点来确定预瞄点
    geometry_msgs::msg::Point next_tgt_pos;

    if (next_wp_idx == 0) {
        next_tgt_pos = loc_next_wp;
    } else {
        // linear interpolation
        std::pair<bool, geometry_msgs::msg::Point> lerp_pair = lerpNextTarget(next_wp_idx, pose);
        if (!lerp_pair.first) {
            std::cerr << "[WARN] lost next waypoint\n";
            return std::make_pair(false, geometry_msgs::msg::Point());
        }
        next_tgt_pos = lerp_pair.second;
    }

    return std::make_pair(true, next_tgt_pos);
}

float TrajectoryFollowing::calcSteerCmd(const geometry_msgs::msg::Point & target, const geometry_msgs::msg::Pose & current_pose)
{
    const double numerator = 2 * control_utils::geometry::transformToRelativeCoordinate2D(target, current_pose).y;
    const double denominator = control_utils::geometry::calcSquaredDistance2d(target, current_pose.position);

    const double kappa = numerator / denominator;
    const double tmp_steering = atan(param.wheel_base * kappa / 2);

    auto steering = static_cast<float>(std::min(std::max(tmp_steering, -param.max_steering_angle), param.max_steering_angle));
    return steering;
}

double TrajectoryFollowing::get_longtitude_error()
{
    //1. 车辆位姿
    const double cx = odometry_->pose.pose.position.x;
    const double cy = odometry_->pose.pose.position.y;
    const double cyaw = tf2::getYaw(odometry_->pose.pose.orientation);

    //2. 把轨迹所有点投影到车辆纵轴上
    std::vector<double> proj_x;
    for(const auto & pt : trajectory_->points) {
        double tx = pt.pose.position.x - cx;
        double ty = pt.pose.position.y - cy;

        double proj = tx * std::cos(cyaw) + ty * std::sin(cyaw);
        proj_x.emplace_back(proj);
    }

    //3. 车辆自身在纵轴上的投影
    const double ego_proj = 0.0;

    //4. 计算最小、最大投影
    double min_proj = *std::min_element(proj_x.begin(), proj_x.end());
    double max_proj = *std::max_element(proj_x.begin(), proj_x.end());

    //5. 纵向误差：车辆 0 和轨迹前后端的插值
    if(min_proj > ego_proj)       return min_proj;
    else if(max_proj < ego_proj)  return max_proj;
    else                          return 0.0;
}

}