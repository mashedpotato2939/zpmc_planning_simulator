#include "cb_velocity_planning.hpp"

namespace cb_planning
{
CB_Velocity_Planning::CB_Velocity_Planning()
{
    
}

SpeedProfile CB_Velocity_Planning::longitudinal_speed_planning(double vx0, double vx_ref, double x_grid_end, double x_end_point, 
    double ax_drive, double ax_brake, double dt, bool isbraking)
{
    // 当面临这些异常情况时，全部规划出停止路径点
    if (vx0 < 0 || vx_ref < 0){
        std::cerr << "Warning: Speed value cannot be negative\n";
        auto traj = generate_stop_trajectory(x_grid_end, 0.1);
        return {traj, true};
    }        

    if(x_end_point < 0){
        // std::cerr << "Error: the position of vehicle is out of destination\n";
        auto traj = generate_stop_trajectory(x_grid_end, 0.1);
        return {traj, true};
    }

    const bool has_end_point = x_grid_end >= x_end_point;
    if (isbraking && !has_end_point){
        std::cerr << "Warning: Cannot be in a braking state without an endpoint\n";
        auto traj = generate_stop_trajectory(x_grid_end, 0.1);
        return {traj, true};
    }
    
    /* ---------- 2. 预计算 ---------- */
    double t_accel = std::fabs(vx0 - vx_ref) / ax_drive;
    double x_accel = (vx0 + vx_ref) * t_accel / 2.0;
    double t_brake = vx_ref / ax_brake;
    double x_brake = vx_ref * t_brake / 2.0;

    double t_total = 0.0;
    int    state   = 0;
    double ax_use  = 0.0;

    /* ---------- 3. 工况判断 ---------- */
    if (!has_end_point) {
        if (x_accel > x_grid_end) {               // 情况1: 无终点，加/减速到目标速度，局部地图内没有达到目标速度
            state    = 1;
            t_total  = (std::sqrt(vx0*vx0 + 2*ax_drive*x_grid_end) - vx0) / ax_drive;
            ax_use   = sign(vx_ref - vx0) * ax_drive;
        } else {                                  // 情况2：无终点，加/减速到目标速度，之后保持匀速
            state    = 2;
            double t_const = (x_grid_end - x_accel) / vx_ref;
            t_total = t_accel + t_const;
        }
    } else {  // 有终点
        if (!isbraking) {
            if (x_brake + x_accel <= x_end_point) { // 情况3：有终点，未处在停车规划状态，停车距离+（到达目标速度的距离）<= 到终点的距离。
                                                    // 这种情况，当刹车时间和车速与总时间以及目标速度都比较接近的时候，就可以进入刹车状态了
                state    = 3;
                double t_const = (x_end_point - x_brake - x_accel) / vx_ref;
                t_total = t_accel + t_const + t_brake;
                if (std::fabs(t_total - t_brake) < 1e-2 && std::fabs(vx0 - vx_ref) < 1e-2)
                    isbraking = true;
            } else {                                // 情况4：有终点，未处在停车规划状态，停车距离+（到达目标速度的距离）> 到终点的距离，此时直接进入刹车状态
                state    = 4;
                isbraking = true;
                t_brake  = 2.0 * x_end_point / vx0;
                double ax_cons = vx0 / t_brake;
                if (ax_cons <= ax_brake) {
                    t_total = t_brake;
                    ax_use  = -ax_cons;
                } else {
                    t_total = vx0 / ax_brake;
                    ax_use  = -ax_brake;
                }
            }
        } else {                                    // 情况5：有终点，已处在停车规划状态，直接规划到终点的速度曲线
            state    = 5;
            isbraking = true;
            t_brake  = 2.0 * x_end_point / vx0;
            double ax_cons = vx0 / t_brake;
            if (ax_cons <= ax_brake) {
                t_total = t_brake;
                ax_use  = -ax_cons;
            } else {
                t_total = vx0 / ax_brake;
                ax_use  = -ax_brake;
            }
        }
    }

    /* ---------- 4. 采样时间向量 ---------- */
    int    n = static_cast<int>(std::ceil(t_total / dt));
    if(n <= 0){
        // std::cerr << "Warning: the number of trajectory points < 0\n";
        auto traj = generate_stop_trajectory(x_grid_end, 0.1);
        return {traj, true};
    }

    std::vector<double> t_k(n);
    for (int i = 0; i < n; ++i) 
        t_k[i] = (i + 1) * dt;

    if (t_k.back() < t_total) {
        t_k.push_back(t_total);
        ++n;
    }

    /* ---------- 5. 根据 state 填充 v, x ---------- */
    std::vector<SpeedPoint> profile(n);
    for (int i = 0; i < n; ++i) {
        double tk = t_k[i];
        SpeedPoint &p = profile[i];
        p.t = tk;

        switch (state) {
        case 1: {  // 纯加/减速
            double ax = sign(vx_ref - vx0) * ax_drive;
            p.v = vx0 + ax * tk;
            p.x = vx0 * tk + 0.5 * ax * tk * tk;
            break;
        }
        case 2: {  // 加/减速 + 匀速
            double ax1 = sign(vx_ref - vx0) * ax_drive;
            if (tk < t_accel) {
                p.v = vx0 + ax1 * tk;
                p.x = vx0 * tk + 0.5 * ax1 * tk * tk;
            } else {
                double v1 = vx0 + ax1 * t_accel;
                double x1 = vx0 * t_accel + 0.5 * ax1 * t_accel * t_accel;
                p.v = v1;
                p.x = x1 + v1 * (tk - t_accel);
            }
            break;
        }
        case 3: {  // 加/减速 + 匀速 + 制动到 0
            double ax1 = sign(vx_ref - vx0) * ax_drive;
            double t1 = t_accel;
            double t2 = t1 + (x_end_point - x_accel - x_brake) / vx_ref;
            if (tk < t1) {
                p.v = vx0 + ax1 * tk;
                p.x = vx0 * tk + 0.5 * ax1 * tk * tk;
            } else if (tk < t2) {
                double v1 = vx0 + ax1 * t1;
                double x1 = vx0 * t1 + 0.5 * ax1 * t1 * t1;
                p.v = v1;
                p.x = x1 + v1 * (tk - t1);
            } else {
                double v1 = vx0 + ax1 * t1;
                double x1 = vx0 * t1 + 0.5 * ax1 * t1 * t1;
                double x2 = x1 + v1 * (t2 - t1);
                double ax2 = -ax_brake;
                p.v = v1 + ax2 * (tk - t2);
                p.x = x2 + v1 * (tk - t2) + 0.5 * ax2 * (tk - t2) * (tk - t2);
            }
            break;
        }
        case 4:
        case 5: {  // 直接制动到 0
            p.v = vx0 + ax_use * tk;
            p.x = vx0 * tk + 0.5 * ax_use * tk * tk;
            break;
        }
        default:
            std::cerr << "Warning: longitudinal speed planning error occurred\n";
        }
    }

    // 如果has_end_point=true，后续以0.1m，一直填充到 x_grid_end
    double x_stop = profile.back().x;
    double dx = 0.1;
    const double inf_time = std::numeric_limits<double>::infinity();
    size_t n1 = static_cast<size_t>(std::floor((x_grid_end-x_stop) / dx + 1.0))+1;
    if (has_end_point){
        for (size_t i = 1; i < n1; ++i) {
            SpeedPoint pt;
            pt.x = std::min(x_stop+ i*dx, x_grid_end);  // 最后一点恰好卡到 x_stop
            pt.v = 0.0;
            pt.t = inf_time;
            profile.emplace_back(pt);
        }
    }

    // 如果发现第一个点的x坐标已经超过终点坐标，则直接返回停止轨迹
    double x_first = profile.front().x;
    if(x_first >= x_end_point){
        auto traj = generate_stop_trajectory(x_grid_end, 0.1);
        return {traj, true};
    }

    // 当速度已经很接近0时，强制将其置为0，避免数值误差带来的负速度
    for (auto& pt : profile) {
        if (pt.v < 5e-3) {
            pt.v = 0.0;
        }
    }                                       

    return {profile, isbraking};
}

std::vector<SpeedPoint> CB_Velocity_Planning::generate_stop_trajectory(double x_stop, double dx)
{
    std::vector<SpeedPoint> traj;
    const double inf_time = std::numeric_limits<double>::infinity();
    
    size_t n = static_cast<size_t>(std::floor(x_stop / dx + 1.0)) + 1;
    traj.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        SpeedPoint pt;
        pt.x = std::min(i * dx, x_stop);  // 最后一点恰好卡到 x_stop
        pt.v = 0.0;
        pt.t = inf_time;                  // 永远到不了的时刻
        traj.emplace_back(pt);
    }

    return traj;
}

}