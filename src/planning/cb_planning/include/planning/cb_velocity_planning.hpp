#ifndef CRABWALKING_VELOCITY_PLANNING_HPP_
#define CRABWALKING_VELOCITY_PLANNING_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace cb_planning
{

struct SpeedPoint {
    double t = 0.0;  // 时间 s
    double v = 0.0;  // 速度 m/s
    double x = 0.0;  // 位移 m
};

struct SpeedProfile {
    std::vector<SpeedPoint> pts;
    bool isbrake = false;
};

class CB_Velocity_Planning{
public:
    explicit CB_Velocity_Planning();
    ~CB_Velocity_Planning() = default;

    SpeedProfile longitudinal_speed_planning(double vx0, double vx_ref, double x_grid_end, double x_end_point, double ax_drive,
        double ax_brake, double dt, bool isbraking);
private:
    inline int sign(double x) { return (x>0) - (x<0);}
    std::vector<SpeedPoint> generate_stop_trajectory(double x_stop, double dx = 0.1);

};
}


#endif