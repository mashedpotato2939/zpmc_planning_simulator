#include "cb_path_planning.hpp"

namespace cb_planning
{
CB_Path_Planning::CB_Path_Planning()
{
    // lane_para.lane_begin_x = -10.0;
    // lane_para.lane_end_x = 120.0;
    // lane_para.lane_width = 4.0;
    // lane_para.lane_num = 2;
    // lane_para.lane_sampling_begin_y = 1.5;
    // lane_para.lane_sampling_end_y = 6.5;
    // lane_para.lane_sampling_interval = 0.25;
}

LocalPlanResult CB_Path_Planning::local_plan(std::vector<double> X_set, double start_point_x, double start_point_y, double start_point_dy, Vec2V edge_points,
                    double L_min, std::vector<double> Yc_set, std::vector<int> NoObs_set, double dymax, double ddymax, double ddymin)
{
    std::vector<double> X_local_set = X_set;
    std::vector<double> Y_set;
    for(double y = lane_para.lane_sampling_begin_y; y<=lane_para.lane_sampling_end_y; y += lane_para.lane_sampling_interval)
        Y_set.push_back(y);
    
    size_t X_len = X_local_set.size();
    size_t Y_len = Y_set.size(); 

    std::vector<std::vector<int>>    state_set(X_len, std::vector<int>(Y_len, 0));
    std::vector<std::vector<double>> ddy_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> dx1_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> dx2_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> dx3_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> dx4_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> Cost_set(X_len, std::vector<double>(Y_len, 0.0));

    using CostElem = std::array<double, 3>;  //{cost, m, n}
    std::vector<CostElem> Cost_priority(X_len*Y_len, {std::numeric_limits<double>::infinity(), 0, 0});
    int collision_free_num = 0;

    Vec2V edge_local_points = edge_points;

    /* 2. 双重循环：遍历所有 (m,n) 网格 */
    for (size_t m = 0; m < X_len; ++m){
        double dx = X_local_set[m];
        for (size_t n = 0; n < Y_len; ++n){
            double y0 = start_point_y;
            double y1 = Y_set[n];
            double dy0 = start_point_dy;

            /* 2.1 生成斜行轨迹参数 */
            auto crab = crabwalking_traj_generate(y0, y1, dy0, dx, dymax, ddymax, ddymin);
            
            int state = crab.state;
            state_set[m][n] = state;
            if (state == 1 || state == 5) continue;

            ddy_set[m][n] = crab.ddy;
            dx1_set[m][n] = crab.dx1;
            dx2_set[m][n] = crab.dx2;
            dx3_set[m][n] = crab.dx3;

            /* 2.2 计算 dx4 */
            double dx4 = (dx < L_min) ? (L_min - dx) : 0.0;
            dx4_set[m][n] = dx4;

            /* 2.3 碰撞检测 */
            auto col = CollisionCheck(state, y0, y1, dy0, crab.ddy, dx, crab.dx1, crab.dx2, crab.dx3, dx4, edge_local_points);
            if (col.isCollision) continue;

            /* 2.4 计算代价 */
            auto cost_res = calcCost(state, y0, y1, dy0, crab.ddy, dx, crab.dx1, crab.dx3, dx4,
                                     X_local_set, Yc_set, NoObs_set);
            double cost_val = cost_res.cost;
            Cost_set[m][n] = cost_val;

            /* 2.5 插入排序（升序） */
            CostElem elem{cost_val, static_cast<double>(m), static_cast<double>(n)};
            auto it = std::lower_bound(Cost_priority.begin(), Cost_priority.end(), elem,
                                       [](const CostElem& a, const CostElem& b){
                                           return a[0] < b[0];
                                       });
            if (it != Cost_priority.end()) *it = elem;

            collision_free_num++;
        }
    }

    /* 3. 输出结果 */
    LocalPlanResult res;
    res.collision_free_num = collision_free_num;
    if (collision_free_num == 0){
        res.isSuccess = 0;
        res.opt_traj.resize(11);
        std::fill(res.opt_traj.begin(), res.opt_traj.end(),
                  std::numeric_limits<double>::quiet_NaN());
    }
    else{
        res.isSuccess = 1;
        size_t opt_m = static_cast<size_t>(Cost_priority[0][1]);
        size_t opt_n = static_cast<size_t>(Cost_priority[0][2]);
        int    opt_state = state_set[opt_m][opt_n];
        double opt_ddy   = ddy_set[opt_m][opt_n];
        double opt_dx1   = dx1_set[opt_m][opt_n];
        double opt_dx2   = dx2_set[opt_m][opt_n];
        double opt_dx3   = dx3_set[opt_m][opt_n];
        double opt_dx4   = dx4_set[opt_m][opt_n];
        double opt_dx    = X_local_set[opt_m];
        double opt_y1    = Y_set[opt_n];

        res.opt_traj = { static_cast<double>(opt_state),
                         start_point_x,
                         start_point_y,
                         start_point_dy,
                         opt_ddy,
                         opt_dx1,
                         opt_dx2,
                         opt_dx3,
                         opt_dx4,
                         opt_dx,
                         opt_y1 };
    }
    return res;
}

LocalPlanResult CB_Path_Planning::local_replan(std::vector<double> X_set, double start_point_x, double start_point_y, double start_point_dy, Vec2V edge_points, 
    double L_min, std::vector<double> Yc_set, std::vector<int> NoObs_set, double dymax, double ddymax, double ddymin)
{
    // LocalPlanResult res{};

    /* 1. 构造局部地图 */
    int x_grid_end = static_cast<int>(std::min(30.0, X_set.back()));
    int idx0 = static_cast<int>(start_point_x);
    int idx1 = static_cast<int>(x_grid_end);
    int len = idx1 - idx0 + 1;

    std::vector<double> X_local_set(len), Yc_local_set(len);
    std::vector<int> NoObs_local_set(len);
    for (int i = 0; i < len; ++i) {
        int k = idx0 + i;
        X_local_set[i]     = X_set[k] - start_point_x;
        Yc_local_set[i]    = Yc_set[k];
        NoObs_local_set[i] = NoObs_set[k];
    }

     /* 2. 构造 Y 搜索空间 1.5:0.25:6.5 */
    std::vector<double> Y_set;
    for(double y = lane_para.lane_sampling_begin_y; y<=lane_para.lane_sampling_end_y; y += lane_para.lane_sampling_interval)
        Y_set.push_back(y);
    
    size_t X_len = X_local_set.size();
    size_t Y_len = Y_set.size(); 

    std::vector<std::vector<int>>    state_set(X_len, std::vector<int>(Y_len, 0));
    std::vector<std::vector<double>> ddy_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> dx1_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> dx2_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> dx3_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> dx4_set(X_len, std::vector<double>(Y_len, 0.0));
    std::vector<std::vector<double>> Cost_set(X_len, std::vector<double>(Y_len, 0.0));

    using CostElem = std::array<double, 3>;  //{cost, m, n}
    std::vector<CostElem> Cost_priority(X_len*Y_len, {std::numeric_limits<double>::infinity(), 0, 0});
    int collision_free_num = 0;

    /* 4. 障碍物坐标平移 */
    Vec2V edge_local_points = edge_points;
    for (size_t i = 0; i < edge_points.size(); ++i) {
        edge_local_points.at(i).x = edge_points.at(i).x - start_point_x;
    }

    /* 5. 遍历网格 */
    /* 2. 双重循环：遍历所有 (m,n) 网格 */
    for (size_t m = 0; m < X_len; ++m){
        double dx = X_local_set[m];
        for (size_t n = 0; n < Y_len; ++n){
            double y0 = start_point_y;
            double y1 = Y_set[n];
            double dy0 = start_point_dy;

            /* 2.1 生成斜行轨迹参数 */
            auto crab = crabwalking_traj_generate(y0, y1, dy0, dx, dymax, ddymax, ddymin);
            
            int state = crab.state;
            state_set[m][n] = state;
            if (state == 1 || state == 5) continue;

            ddy_set[m][n] = crab.ddy;
            dx1_set[m][n] = crab.dx1;
            dx2_set[m][n] = crab.dx2;
            dx3_set[m][n] = crab.dx3;

            /* 2.2 计算 dx4 */
            double dx4 = (dx < (L_min - start_point_x +1.0)) ? (L_min - start_point_x +1.0 - dx) : 0.0;
            dx4_set[m][n] = dx4;

            /* 2.3 碰撞检测 */
            auto col = CollisionCheck(state, y0, y1, dy0, crab.ddy, dx, crab.dx1, crab.dx2, crab.dx3, dx4, edge_local_points);
            if (col.isCollision) continue;

            /* 2.4 计算代价 */
            auto cost_res = calcCost(state, y0, y1, dy0, crab.ddy, dx, crab.dx1, crab.dx3, dx4,
                                     X_local_set, Yc_set, NoObs_set);
            double cost_val = cost_res.cost;
            Cost_set[m][n] = cost_val;

            /* 2.5 插入排序（升序） */
            CostElem elem{cost_val, static_cast<double>(m), static_cast<double>(n)};
            auto it = std::lower_bound(Cost_priority.begin(), Cost_priority.end(), elem,
                                       [](const CostElem& a, const CostElem& b){
                                           return a[0] < b[0];
                                       });
            if (it != Cost_priority.end()) *it = elem;

            collision_free_num++;
        }
    }

    /* 3. 输出结果 */
    LocalPlanResult res;
    res.collision_free_num = collision_free_num;
    if (collision_free_num == 0){
        res.isSuccess = 0;
        res.opt_traj.resize(11);
        std::fill(res.opt_traj.begin(), res.opt_traj.end(),
                  std::numeric_limits<double>::quiet_NaN());
    }
    else{
        res.isSuccess = 1;
        size_t opt_m = static_cast<size_t>(Cost_priority[0][1]);
        size_t opt_n = static_cast<size_t>(Cost_priority[0][2]);
        int    opt_state = state_set[opt_m][opt_n];
        double opt_ddy   = ddy_set[opt_m][opt_n];
        double opt_dx1   = dx1_set[opt_m][opt_n];
        double opt_dx2   = dx2_set[opt_m][opt_n];
        double opt_dx3   = dx3_set[opt_m][opt_n];
        double opt_dx4   = dx4_set[opt_m][opt_n];
        double opt_dx    = X_local_set[opt_m];
        double opt_y1    = Y_set[opt_n];

        res.opt_traj = { static_cast<double>(opt_state),
                         start_point_x,
                         start_point_y,
                         start_point_dy,
                         opt_ddy,
                         opt_dx1,
                         opt_dx2,
                         opt_dx3,
                         opt_dx4,
                         opt_dx,
                         opt_y1 };
    }
    return res;
}

void CB_Path_Planning::calcPosy_Vy(std::vector<double> traj, double ego_pos_x, double &ego_pos_y, double &ego_pos_dy)
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

CrabResult CB_Path_Planning::crabwalking_traj_generate(double y0, double y1, double dy0, double dx, double dymax, double ddymax, double ddymin)
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

double CB_Path_Planning::calcTrajPoint(int state, double y0, double y1, double dy0, double ddy, double dx, double dx1, double dx3, double ego_pos_x)
{
    //初始化变量
    double x = ego_pos_x;
    double y = 0;

    if(state == 0){
        y = y0;
    }else if(state == 2){
        if(x>=0 && x<dx1){
            y = y0 + dy0 * x;
        }else if(x>=dx1 && x<=dx){
            y = y0 + dy0 * x - 0.5*ddy*std::pow(x-dx1, 2);
        }
    }else if(state == 3){
        if(x>=0.0 && x<dx1){
            y = y0 + dy0*x + 0.5*ddy*x*x;
        }else if(x>=dx1 && x<=dx){
            double y_t1 = y0 + dy0*dx1 + 0.5*ddy*dx1*dx1;
            double v_t1 = dy0 + ddy*dx1;
            y = y_t1 + v_t1*(x - dx1) - 0.5*ddy*std::pow(x-dx1, 2);
        }
    }else if(state == 4){
        if(x>=0.0 && x<=dx1){
            y = y0 + dy0*x + 0.5*ddy*x*x;
        }else if(x>=dx1 && x<(dx1 + dx3)){
            double y_t1 = y0 + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
            double v_t1 = dy0 + ddy * dx1;
            y = y_t1 + v_t1*(x - dx1);
        }else if(x >= (dx1 + dx3) && x <= dx){
            double y_t1 = y0 + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
            double v_t1 = dy0 + ddy * dx1;
            double y_t2 = y_t1 + v_t1 * dx3;
            double v_t2 = v_t1;
            y = y_t2 + v_t2 * (x - (dx1 + dx3)) - 0.5 * ddy * std::pow(x - (dx1 + dx3), 2);
        }
    }

    /* 边界处理 */
    if(x < 0.0)
        y = y0;
    else if(x > dx)
        y = y1;
    
    return y;
}

void CB_Path_Planning::calcEdges(int state, double y0, double y1, double dy0, double ddy, double dx, double dx1, double dx2, double dx3, double dx4, 
                    double ego_pos_x, double& leftEdge, double& rightEdge){
    //初始化变量

    const double inf  =  std::numeric_limits<double>::infinity();
    leftEdge = inf;
    rightEdge = -inf;

    /* ---------- 提前退出区间 ---------- */
    if ((ego_pos_x > (dx+dx4+ego_length/2)) && (ego_pos_x < -ego_length/2))
        return;
    
    /* ---------- 匀速直线段 ---------- */
    if ((ego_pos_x > (dx + ego_length/2)) && (ego_pos_x <= (dx+dx4+ego_length/2)))
    {
        leftEdge = y1 + ego_width/2;
        rightEdge = y1 - ego_width/2;
        return;
    }

    /* ---------- 状态机 ---------- */
    if (state == 0)   // 直线
    {
        leftEdge  = y0 + ego_width / 2;
        rightEdge = y0 - ego_width / 2;
    }else if (state == 2)   // 简单换道（两段加速度）
    {
        if (dy0 < 0)
        {
            double x_left  = ego_pos_x - ego_length / 2;
            double y_left  = y0 + ego_width / 2;
            double x_right = ego_pos_x + ego_length / 2;
            double y_right = y0 - ego_width / 2;

            /* ---- 左边界 ---- */
            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < ego_length / 2)
                leftEdge = y_left;
            else if (ego_pos_x >= ego_length / 2 && ego_pos_x < dx1 + ego_length / 2)
                leftEdge = y_left + dy0 * x_left;
            else if (ego_pos_x >= dx1 + ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
                leftEdge = y_left + dy0 * x_left - 0.5 * ddy * std::pow(x_left - dx1, 2);
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state2 left, dy0<0)\n";
                return;
            }

            /* ---- 右边界 ---- */
            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < dx1 - ego_length / 2)
                rightEdge = y_right + dy0 * x_right;
            else if (ego_pos_x >= dx1 - ego_length / 2 && ego_pos_x < dx - ego_length / 2)
                rightEdge = y_right + dy0 * x_right - 0.5 * ddy * std::pow(x_right - dx1, 2);
            else if (ego_pos_x >= dx - ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
                rightEdge = y_right + dy0 * dx - 0.5 * ddy * dx2 * dx2;
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state2 right, dy0<0)\n";
                return;
            }
        }
        else if (dy0 > 0)
        {
            double x_left  = ego_pos_x + ego_length / 2;
            double y_left  = y0 + ego_width / 2;
            double x_right = ego_pos_x - ego_length / 2;
            double y_right = y0 - ego_width / 2;

            /* ---- 左边界 ---- */
            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < dx1 - ego_length / 2)
                leftEdge = y_left + dy0 * x_left;
            else if (ego_pos_x >= dx1 - ego_length / 2 && ego_pos_x < dx - ego_length / 2)
                leftEdge = y_left + dy0 * x_left - 0.5 * ddy * std::pow(x_left - dx1, 2);
            else if (ego_pos_x >= dx - ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
                leftEdge = y_left + dy0 * dx - 0.5 * ddy * dx2 * dx2;
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state2 left, dy0>0)\n";
                return;
            }

            /* ---- 右边界 ---- */
            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < ego_length / 2)
                rightEdge = y_right;
            else if (ego_pos_x >= ego_length / 2 && ego_pos_x < dx1 + ego_length / 2)
                rightEdge = y_right + dy0 * x_right;
            else if (ego_pos_x >= dx1 + ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
                rightEdge = y_right + dy0 * x_right - 0.5 * ddy * std::pow(x_right - dx1, 2);
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state2 right, dy0>0)\n";
                return;
            }
        }
        else
        {
            std::cerr << "warning: abnormal status (state2 dy0==0)\n";
            return;
        }
    }else if (state == 3)   // 三段加速度（先加后减）
    {
        /* 以下结构与 MATLAB 完全一致，仅做语法转换，不再逐行注释 */
        if (dy0 >= 0 && ddy > 0)
        {
            double x_left  = ego_pos_x + ego_length / 2;
            double y_left  = y0 + ego_width / 2;
            double x_right = ego_pos_x - ego_length / 2;
            double y_right = y0 - ego_width / 2;

            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < dx1 - ego_length / 2)
                leftEdge = y_left + dy0 * x_left + 0.5 * ddy * x_left * x_left;
            else if (ego_pos_x >= dx1 - ego_length / 2 && ego_pos_x < dx - ego_length / 2)
            {
                double y_t1 = y_left + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                leftEdge = y_t1 + v_t1 * (x_left - dx1) - 0.5 * ddy * std::pow(x_left - dx1, 2);
            }
            else if (ego_pos_x >= dx - ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
            {
                double y_t1 = y_left + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                leftEdge = y_t1 + v_t1 * dx2 - 0.5 * ddy * dx2 * dx2;
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state3 left, dy0≥0, ddy>0)\n";
                return;
            }

            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < ego_length / 2)
                rightEdge = y_right;
            else if (ego_pos_x >= ego_length / 2 && ego_pos_x < dx1 + ego_length / 2)
                rightEdge = y_right + dy0 * x_right + 0.5 * ddy * x_right * x_right;
            else if (ego_pos_x >= dx1 + ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
            {
                double y_t1 = y_right + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                rightEdge = y_t1 + v_t1 * (x_right - dx1) - 0.5 * ddy * std::pow(x_right - dx1, 2);
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state3 right, dy0≥0, ddy>0)\n";
                return;
            }
        }
        else if (dy0 <= 0 && ddy < 0)
        {
            double x_left  = ego_pos_x - ego_length / 2;
            double y_left  = y0 + ego_width / 2;
            double x_right = ego_pos_x + ego_length / 2;
            double y_right = y0 - ego_width / 2;

            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < ego_length / 2)
                leftEdge = y_left;
            else if (ego_pos_x >= ego_length / 2 && ego_pos_x < dx1 + ego_length / 2)
                leftEdge = y_left + dy0 * x_left + 0.5 * ddy * x_left * x_left;
            else if (ego_pos_x >= dx1 + ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
            {
                double y_t1 = y_left + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                leftEdge = y_t1 + v_t1 * (x_left - dx1) - 0.5 * ddy * std::pow(x_left - dx1, 2);
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state3 left, dy0≤0, ddy<0)\n";
                return;
            }

            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < dx1 - ego_length / 2)
                rightEdge = y_right + dy0 * x_right + 0.5 * ddy * x_right * x_right;
            else if (ego_pos_x >= dx1 - ego_length / 2 && ego_pos_x < dx - ego_length / 2)
            {
                double y_t1 = y_right + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                rightEdge = y_t1 + v_t1 * (x_right - dx1) - 0.5 * ddy * std::pow(x_right - dx1, 2);
            }
            else if (ego_pos_x >= dx - ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
            {
                double y_t1 = y_right + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                rightEdge = y_t1 + v_t1 * dx2 - 0.5 * ddy * dx2 * dx2;
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state3 right, dy0≤0, ddy<0)\n";
                return;
            }
        }
        else if (dy0 > 0 && ddy < 0)
        {
            double xlf = ego_pos_x - ego_length / 2;
            double ylf = y0 + ego_width / 2;
            double xlb = ego_pos_x + ego_length / 2;
            double ylb = y0 + ego_width / 2;
            double dx_01 = std::fabs(dy0 / ddy);

            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < dx_01 - ego_length / 2)
                leftEdge = ylb + dy0 * xlb + 0.5 * ddy * xlb * xlb;
            else if (ego_pos_x >= dx_01 - ego_length / 2 && ego_pos_x < dx_01 + ego_length / 2)
                leftEdge = ylb + dy0 * dx_01 + 0.5 * ddy * dx_01 * dx_01;
            else if (ego_pos_x >= dx_01 + ego_length / 2 && ego_pos_x < dx1 + ego_length / 2)
                leftEdge = ylf + dy0 * xlf + 0.5 * ddy * xlf * xlf;
            else if (ego_pos_x >= dx1 + ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
            {
                double y_t1 = ylf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                leftEdge = y_t1 + v_t1 * (xlf - dx1) - 0.5 * ddy * std::pow(xlf - dx1, 2);
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state3 left, dy0>0, ddy<0)\n";
                return;
            }

            /* ---- 右边界：取最小 ---- */
            double xrf = ego_pos_x - ego_length / 2;
            double yrf = y0 - ego_width / 2;
            double xrb = ego_pos_x + ego_length / 2;
            double yrb = y0 - ego_width / 2;
            rightEdge = inf;

            auto update = [&](double y) { if (y < rightEdge) rightEdge = y; };

            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < ego_length / 2)
                update(yrf);
            if (ego_pos_x >= ego_length / 2 && ego_pos_x < dx_01 + ego_length / 2)
                update(yrf + dy0 * xrf + 0.5 * ddy * xrf * xrf);
            if (ego_pos_x >= dx_01 - ego_length / 2 && ego_pos_x < dx1 - ego_length / 2)
                update(yrb + dy0 * xrb + 0.5 * ddy * xrb * xrb);
            if (ego_pos_x >= dx1 - ego_length / 2 && ego_pos_x < dx - ego_length / 2)
            {
                double y_t1 = yrb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                update(y_t1 + v_t1 * (xrb - dx1) - 0.5 * ddy * std::pow(xrb - dx1, 2));
            }
            if (ego_pos_x >= dx - ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
            {
                double y_t1 = yrb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx2;
                double v_t2 = v_t1;
                update(y_t2 + v_t2 * dx2 - 0.5 * ddy * dx2 * dx2);
            }
        }
        else if (dy0 < 0 && ddy > 0)
        {
            /* 与 dy0>0 && ddy<0 完全对称，左边界取最大，右边界正常分段 */
            double xlb = ego_pos_x + ego_length / 2;
            double ylb = y0 + ego_width / 2;
            double xlf = ego_pos_x - ego_length / 2;
            double ylf = y0 + ego_width / 2;
            double dx_01 = std::fabs(dy0 / ddy);
            leftEdge = -inf;

            auto updateL = [&](double y) { if (y > leftEdge) leftEdge = y; };

            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < ego_length / 2)
                updateL(ylf);
            if (ego_pos_x >= ego_length / 2 && ego_pos_x < dx_01 + ego_length / 2)
                updateL(ylf + dy0 * xlf + 0.5 * ddy * xlf * xlf);
            if (ego_pos_x >= dx_01 - ego_length / 2 && ego_pos_x < dx1 - ego_length / 2)
                updateL(ylb + dy0 * xlb + 0.5 * ddy * xlb * xlb);
            if (ego_pos_x >= dx1 - ego_length / 2 && ego_pos_x < dx - ego_length / 2)
            {
                double y_t1 = ylb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                updateL(y_t1 + v_t1 * (xlb - dx1) - 0.5 * ddy * std::pow(xlb - dx1, 2));
            }
            if (ego_pos_x >= dx - ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
            {
                double y_t1 = ylb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx2;
                double v_t2 = v_t1;
                updateL(y_t2 + v_t2 * dx2 - 0.5 * ddy * dx2 * dx2);
            }

            /* ---- 右边界 ---- */
            double xrf = ego_pos_x - ego_length / 2;
            double yrf = y0 - ego_width / 2;
            double xrb = ego_pos_x + ego_length / 2;
            double yrb = y0 - ego_width / 2;

            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < dx_01 - ego_length / 2)
                rightEdge = yrb + dy0 * xrb + 0.5 * ddy * xrb * xrb;
            else if (ego_pos_x >= dx_01 - ego_length / 2 && ego_pos_x < dx_01 + ego_length / 2)
                rightEdge = yrb + dy0 * dx_01 + 0.5 * ddy * dx_01 * dx_01;
            else if (ego_pos_x >= dx_01 + ego_length / 2 && ego_pos_x < dx1 + ego_length / 2)
                rightEdge = yrf + dy0 * xrf + 0.5 * ddy * xrf * xrf;
            else if (ego_pos_x >= dx1 + ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
            {
                double y_t1 = yrf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                rightEdge = y_t1 + v_t1 * (xrf - dx1) - 0.5 * ddy * std::pow(xrf - dx1, 2);
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state3 right, dy0<0, ddy>0)\n";
                return;
            }
        }
    }else if (state == 4)   // 四段加速度（加-匀-减）
    {
        /* 以下结构与 MATLAB 完全一致，仅做语法转换，不再逐行注释 */
        if (dy0 >= 0 && ddy > 0)
        {
            double xlb = ego_pos_x + ego_length / 2;
            double ylb = y0 + ego_width / 2;
            double xrf = ego_pos_x - ego_length / 2;
            double yrf = y0 - ego_width / 2;

            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < dx1 - ego_length / 2)
                leftEdge = ylb + dy0 * xlb + 0.5 * ddy * xlb * xlb;
            else if (ego_pos_x >= dx1 - ego_length / 2 && ego_pos_x < (dx1 + dx3) - ego_length / 2)
            {
                double y_t1 = ylb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                leftEdge = y_t1 + v_t1 * (xlb - dx1);
            }
            else if (ego_pos_x >= (dx1 + dx3) - ego_length / 2 && ego_pos_x < dx - ego_length / 2)
            {
                double y_t1 = ylb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                leftEdge = y_t2 + v_t2 * (xlb - (dx1 + dx3)) - 0.5 * ddy * std::pow(xlb - (dx1 + dx3), 2);
            }
            else if (ego_pos_x >= dx - ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
            {
                double y_t1 = ylb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                leftEdge = y_t2 + v_t2 * dx2 - 0.5 * ddy * dx2 * dx2;
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state4 left, dy0≥0, ddy>0)\n";
                return;
            }

            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < ego_length / 2)
                rightEdge = yrf;
            else if (ego_pos_x >= ego_length / 2 && ego_pos_x < dx1 + ego_length / 2)
                rightEdge = yrf + dy0 * xrf + 0.5 * ddy * xrf * xrf;
            else if (ego_pos_x >= dx1 + ego_length / 2 && ego_pos_x < (dx1 + dx3) + ego_length / 2)
            {
                double y_t1 = yrf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                rightEdge = y_t1 + v_t1 * (xrf - dx1);
            }
            else if (ego_pos_x >= (dx1 + dx3) + ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
            {
                double y_t1 = yrf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                rightEdge = y_t2 + v_t2 * (xrf - (dx1 + dx3)) - 0.5 * ddy * std::pow(xrf - (dx1 + dx3), 2);
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state4 right, dy0≥0, ddy>0)\n";
                return;
            }
        }
        else if (dy0 <= 0 && ddy < 0)
        {
            double xlf = ego_pos_x - ego_length / 2;
            double ylf = y0 + ego_width / 2;
            double xrb = ego_pos_x + ego_length / 2;
            double yrb = y0 - ego_width / 2;

            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < ego_length / 2)
                leftEdge = ylf;
            else if (ego_pos_x >= ego_length / 2 && ego_pos_x < dx1 + ego_length / 2)
                leftEdge = ylf + dy0 * xlf + 0.5 * ddy * xlf * xlf;
            else if (ego_pos_x >= dx1 + ego_length / 2 && ego_pos_x < (dx1 + dx3) + ego_length / 2)
            {
                double y_t1 = ylf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                leftEdge = y_t1 + v_t1 * (xlf - dx1);
            }
            else if (ego_pos_x >= (dx1 + dx3) + ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
            {
                double y_t1 = ylf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                leftEdge = y_t2 + v_t2 * (xlf - (dx1 + dx3)) - 0.5 * ddy * std::pow(xlf - (dx1 + dx3), 2);
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state4 left, dy0≤0, ddy<0)\n";
                return;
            }

            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < dx1 - ego_length / 2)
                rightEdge = yrb + dy0 * xrb + 0.5 * ddy * xrb * xrb;
            else if (ego_pos_x >= dx1 - ego_length / 2 && ego_pos_x < (dx1 + dx3) - ego_length / 2)
            {
                double y_t1 = yrb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                rightEdge = y_t1 + v_t1 * (xrb - dx1);
            }
            else if (ego_pos_x >= (dx1 + dx3) - ego_length / 2 && ego_pos_x < dx - ego_length / 2)
            {
                double y_t1 = yrb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                rightEdge = y_t2 + v_t2 * (xrb - (dx1 + dx3)) - 0.5 * ddy * std::pow(xrb - (dx1 + dx3), 2);
            }
            else if (ego_pos_x >= dx - ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
            {
                double y_t1 = yrb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                rightEdge = y_t2 + v_t2 * dx2 - 0.5 * ddy * dx2 * dx2;
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state4 right, dy0≤0, ddy<0)\n";
                return;
            }
        }
        else if (dy0 > 0 && ddy < 0)
        {
            double xlb = ego_pos_x + ego_length / 2;
            double ylb = y0 + ego_width / 2;
            double xlf = ego_pos_x - ego_length / 2;
            double ylf = y0 + ego_width / 2;
            double dx_01 = std::fabs(dy0 / ddy);

            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < dx_01 - ego_length / 2)
                leftEdge = ylb + dy0 * xlb + 0.5 * ddy * xlb * xlb;
            else if (ego_pos_x >= dx_01 - ego_length / 2 && ego_pos_x < dx_01 + ego_length / 2)
                leftEdge = ylb + dy0 * dx_01 + 0.5 * ddy * dx_01 * dx_01;
            else if (ego_pos_x >= dx_01 + ego_length / 2 && ego_pos_x < dx1 + ego_length / 2)
                leftEdge = ylf + dy0 * xlf + 0.5 * ddy * xlf * xlf;
            else if (ego_pos_x >= dx1 + ego_length / 2 && ego_pos_x < (dx1 + dx3) + ego_length / 2)
            {
                double y_t1 = ylf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                leftEdge = y_t1 + v_t1 * (xlf - dx1);
            }
            else if (ego_pos_x >= (dx1 + dx3) + ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
            {
                double y_t1 = ylf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                leftEdge = y_t2 + v_t2 * (xlf - (dx1 + dx3)) - 0.5 * ddy * std::pow(xlf - (dx1 + dx3), 2);
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state4 left, dy0>0, ddy<0)\n";
                return;
            }

            double xrf = ego_pos_x - ego_length / 2;
            double yrf = y0 - ego_width / 2;
            double xrb = ego_pos_x + ego_length / 2;
            double yrb = y0 - ego_width / 2;
            rightEdge = inf;

            auto updateR = [&](double y) { if (y < rightEdge) rightEdge = y; };

            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < ego_length / 2)
                updateR(yrf);
            if (ego_pos_x >= ego_length / 2 && ego_pos_x < dx_01 + ego_length / 2)
                updateR(yrf + dy0 * xrf + 0.5 * ddy * xrf * xrf);
            if (ego_pos_x >= dx_01 - ego_length / 2 && ego_pos_x < dx1 - ego_length / 2)
                updateR(yrb + dy0 * xrb + 0.5 * ddy * xrb * xrb);
            if (ego_pos_x >= dx1 - ego_length / 2 && ego_pos_x < (dx1 + dx3) - ego_length / 2)
            {
                double y_t1 = yrb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                updateR(y_t1 + v_t1 * (xrb - dx1));
            }
            if (ego_pos_x >= (dx1 + dx3) - ego_length / 2 && ego_pos_x < dx - ego_length / 2)
            {
                double y_t1 = yrb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                updateR(y_t2 + v_t2 * (xrb - (dx1 + dx3)) - 0.5 * ddy * std::pow(xrb - (dx1 + dx3), 2));
            }
            if (ego_pos_x >= dx - ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
            {
                double y_t1 = yrb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                updateR(y_t2 + v_t2 * dx2 - 0.5 * ddy * dx2 * dx2);
            }
        }
        else if (dy0 < 0 && ddy > 0)
        {
            double xlb = ego_pos_x + ego_length / 2;
            double ylb = y0 + ego_width / 2;
            double xlf = ego_pos_x - ego_length / 2;
            double ylf = y0 + ego_width / 2;
            double dx_01 = std::fabs(dy0 / ddy);
            leftEdge = -inf;

            auto updateL = [&](double y) { if (y > leftEdge) leftEdge = y; };

            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < ego_length / 2)
                updateL(ylf);
            if (ego_pos_x >= ego_length / 2 && ego_pos_x < dx_01 + ego_length / 2)
                updateL(ylf + dy0 * xlf + 0.5 * ddy * xlf * xlf);
            if (ego_pos_x >= dx_01 - ego_length / 2 && ego_pos_x < dx1 - ego_length / 2)
                updateL(ylb + dy0 * xlb + 0.5 * ddy * xlb * xlb);
            if (ego_pos_x >= dx1 - ego_length / 2 && ego_pos_x < (dx1 + dx3) - ego_length / 2)
            {
                double y_t1 = ylb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                updateL(y_t1 + v_t1 * (xlb - dx1));
            }
            if (ego_pos_x >= (dx1 + dx3) - ego_length / 2 && ego_pos_x < dx - ego_length / 2)
            {
                double y_t1 = ylb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                updateL(y_t2 + v_t2 * (xlb - (dx1 + dx3)) - 0.5 * ddy * std::pow(xlb - (dx1 + dx3), 2));
            }
            if (ego_pos_x >= dx - ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
            {
                double y_t1 = ylb + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                updateL(y_t2 + v_t2 * dx2 - 0.5 * ddy * dx2 * dx2);
            }

            double xrf = ego_pos_x - ego_length / 2;
            double yrf = y0 - ego_width / 2;
            double xrb = ego_pos_x + ego_length / 2;
            double yrb = y0 - ego_width / 2;

            if (ego_pos_x >= -ego_length / 2 && ego_pos_x < dx_01 - ego_length / 2)
                rightEdge = yrb + dy0 * xrb + 0.5 * ddy * xrb * xrb;
            else if (ego_pos_x >= dx_01 - ego_length / 2 && ego_pos_x < dx_01 + ego_length / 2)
                rightEdge = yrb + dy0 * dx_01 + 0.5 * ddy * dx_01 * dx_01;
            else if (ego_pos_x >= dx_01 + ego_length / 2 && ego_pos_x < dx1 + ego_length / 2)
                rightEdge = yrf + dy0 * xrf + 0.5 * ddy * xrf * xrf;
            else if (ego_pos_x >= dx1 + ego_length / 2 && ego_pos_x < (dx1 + dx3) + ego_length / 2)
            {
                double y_t1 = yrf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                rightEdge = y_t1 + v_t1 * (xrf - dx1);
            }
            else if (ego_pos_x >= (dx1 + dx3) + ego_length / 2 && ego_pos_x <= dx + ego_length / 2)
            {
                double y_t1 = yrf + dy0 * dx1 + 0.5 * ddy * dx1 * dx1;
                double v_t1 = dy0 + ddy * dx1;
                double y_t2 = y_t1 + v_t1 * dx3;
                double v_t2 = v_t1;
                rightEdge = y_t2 + v_t2 * (xrf - (dx1 + dx3)) - 0.5 * ddy * std::pow(xrf - (dx1 + dx3), 2);
            }
            else
            {
                std::cerr << "warning: obs_edge_points_x is over limit (state4 right, dy0<0, ddy>0)\n";
                return;
            }
        }
    }
}

std::vector<Point2D> CB_Path_Planning::calcObsEdgePoints(std::vector<Point2D> bbox, double lower, double upper)
{
    if(bbox.size() != 4){
        std::cerr << "warning: bbox size != 4\n";
        return {};
    }

    auto mmx = std::minmax_element(bbox.begin(), bbox.end(),
                                   [](const Point2D& a, const Point2D& b)
                                   { return a.x < b.x; });
    double min_x = mmx.first->x;
    double max_x = mmx.second->x;

    /* 快速排除 */
    if (max_x < lower || min_x > upper) return{};

    /* 刚好贴边 */
    if(max_x == lower){
        std::vector<Point2D> pts;
        for(const auto& p : bbox)
            if(p.x == max_x) pts.push_back(p);
        return pts;
    }
    if(min_x == upper){
        std::vector<Point2D> pts;
        for(const auto& p:bbox)
            if(p.x == min_x) pts.push_back(p);
        return pts;
    }

    /* 完全包含 */
    if(min_x >= lower && max_x <= upper) return bbox;

    /* 一般情况： 求四条边与x=lower / x=upper, 并补充内部顶点 */
    std::vector<Point2D> pts;
    std::vector<Point2D> tmp = bbox;
    tmp.push_back(bbox[0]);

    auto addIntersect = [&](double xcut)
    {
        for (size_t i = 0; i < 4; ++i) {
            double x1 = tmp[i].x, y1 = tmp[i].y;
            double x2 = tmp[i + 1].x, y2 = tmp[i + 1].y;

            if (y1 == y2) continue;            // 水平边
            if (x1 == x2) {                    // 垂直边
                if (xcut >= min_x && xcut <= max_x) {
                    pts.emplace_back(xcut, y1);
                    pts.emplace_back(xcut, y2);
                }
                break;
            }
            double t = (xcut - x1) / (x2 - x1);
            if (t < 0.0 || t > 1.0) continue;
            double y = y1 + t * (y2 - y1);
            pts.emplace_back(xcut, y);
        }
    };
    addIntersect(lower);
    addIntersect(upper);

    /* 补充位于 (lower,upper) 内的原始顶点 */
    for (const auto& p : bbox)
        if (p.x > lower && p.x < upper)
            pts.push_back(p);

    /* 排序 + 去重 */
    std::sort(pts.begin(), pts.end(), cmpPt);
    pts.erase(std::unique(pts.begin(), pts.end(),
                          [](const Point2D& a, const Point2D& b)
                          { return a.x == b.x && a.y == b.y; }),
              pts.end());
    return pts;
}

OneObsResult CB_Path_Planning::CollisionCheck_for_OneObstacle(int state, double y0, double y1, double dy0, double ddy,
                                double dxsum, double dx1, double dx2, double dx3, double dx4, const Vec2V& bbox)
{
    OneObsResult res;

    /* 1. 计算有效边界点 */
    res.obsEdgePts = calcObsEdgePoints(bbox, -ego_length / 2.0,  (dxsum + dx4) + ego_length / 2.0);
    std::size_t n = res.obsEdgePts.size();
    if(n == 0){
        res.isCollision = false;
        return res;
    }

    /* 2. 对每个点判断是否左右越界 */
    std::vector<bool> onLeft(n, false), onRight(n, false);
    for(std::size_t i=0; i<n; i++){
        double obs_x = res.obsEdgePts[i].x;
        double obs_y = res.obsEdgePts[i].y;

        double leftEdge, rightEgde;
        calcEdges(state, y0, y1, dy0, ddy, dxsum, dx1, dx2, dx3, dx4, obs_x, leftEdge,rightEgde);
        
        if(obs_y > leftEdge)        onLeft[i] = true;
        else if(obs_y < rightEgde)  onRight[i] = true;
        /* 介于区间内，两个标志均为false */
    }

    /* 3. 碰撞逻辑：只有“全部在左侧”或“全部在右侧”才算无碰撞 */
    bool allLeft = std::all_of(onLeft.begin(), onLeft.end(), [](bool v){ return v; });
    bool allRight = std::all_of(onRight.begin(), onRight.end(), [](bool v){ return v;});

    res.isLoE = allLeft;
    res.isRoE = allRight;
    res.isCollision = !(allLeft || allRight);
    return res;
}

BatchObsResult CB_Path_Planning::CollisionCheck(int state, double y0, double y1, double dy0, double ddy,
                                double dx, double dx1, double dx2, double dx3, double dx4, const Vec2V& edge_points)
{
    BatchObsResult res;
    size_t obs_num = edge_points.size() / 4;   // 4 个点一组
    res.isLoE.resize(obs_num, false);
    res.isRoE.resize(obs_num, false);
    res.obsEps.resize(obs_num);

    for (size_t m = 0; m < obs_num; ++m)
    {
        std::vector<Point2D> bbox(edge_points.begin() + m * 4,
                                    edge_points.begin() + (m + 1) * 4);

        auto r = CollisionCheck_for_OneObstacle(state, y0, y1, dy0, ddy, dx, dx1, dx2, dx3, dx4, bbox);
        res.isLoE[m] = r.isLoE;
        res.isRoE[m] = r.isRoE;
        res.obsEps[m] = std::move(r.obsEdgePts);

        if (r.isCollision)
        {
            res.isCollision = true;
            return res;               // 一旦碰撞立即退出，与 MATLAB 一致
        }
    }
    res.isCollision = false;
    return res;
}

DrivableRange CB_Path_Planning::calcDrivableRange(Vec2V obs_edge_points, double ego_pos_x, int ref_lane)
{
    /* 1. 检查输入障碍物点的数量 */
    Vec2V pts = obs_edge_points;
    if (pts.size() % 4 != 0){
        std::cerr << "顶点数必须是 4 的倍数\n";
        return {};
    }

    size_t obs_num = pts.size() / 4;

    /* 2. 先筛选在 [ego_pos_x-ego_length/2, ego_pos_x+ego_length/2] 范围内的障碍物 */
    Vec2V obs_eps_Vlength;                  // 存有效障碍物顶点（连续 4 个一组）
    std::vector<double> Yo_max_Vlength;
    std::vector<double> Yo_min_Vlength;
    size_t no = 0;                          // 有效障碍物计数

    double x_low = ego_pos_x - ego_length/2.0;
    double x_high = ego_pos_x + ego_length/2.0;

    for(size_t k=0; k<obs_num; k++){
        Vec2V bbox(pts.begin() + k * 4, pts.begin() + (k + 1) * 4);
        Vec2V eps = calcObsEdgePoints(bbox, x_low, x_high);
        if (!eps.empty()){
            /* 记录整个障碍物 4 顶点 */
            obs_eps_Vlength.insert(obs_eps_Vlength.end(), bbox.begin(), bbox.end());
            
            /* 记录该障碍物在纵向范围内的 Y 极值 */
            auto mmy = std::minmax_element(eps.begin(), eps.end(), [](const Point2D& a, const Point2D& b) {return a.y < b.y; });
            Yo_max_Vlength.push_back(mmy.second->y);
            Yo_min_Vlength.push_back(mmy.first ->y);
            no ++;
        }
    }

    /* 3.车身纵向范围内无障碍物 -> 走车道中心 */
    if(no == 0){
        double Ll = lane_para.lane_begin_y + ref_lane * lane_para.lane_width;
        double Lr = lane_para.lane_begin_y + (ref_lane - 1) * lane_para.lane_width;
        return {(Ll+Lr)/2.0, Ll, Lr, 0};
    }

    /* 4.再计算这些障碍物在局部地图 X 范围中的 Y 极值 */
    std::vector<double> Yo_max_localMap(no);
    std::vector<double> Yo_min_localMap(no);
    std::vector<int>    is_inRefLane(no, 0);

    double map_x_low = -ego_length/2.0;
    double map_x_high = local_map_length + ego_length/2.0;

    double Ll_lane = lane_para.lane_begin_y + std::min(ref_lane + 1, lane_para.lane_num) * lane_para.lane_width;
    double Lr_lane = lane_para.lane_begin_y + std::max(ref_lane - 2, 0) * lane_para.lane_width;

    for(size_t k=0; k<no; k++){
        Vec2V bbox(obs_eps_Vlength.begin() + k * 4, obs_eps_Vlength.begin() + (k + 1) * 4);
        Vec2V eps = calcObsEdgePoints(bbox, map_x_low, map_x_high);
        auto mmy = std::minmax_element(eps.begin(), eps.end(), [](const Point2D& a, const Point2D& b) {return a.y < b.y;});
        Yo_max_localMap[k] = mmy.second->y;
        Yo_min_localMap[k] = mmy.first->y;

        /* 判断是否侵入当前车道 */
        if(Yo_max_localMap[k] < (ref_lane - 1)*lane_para.lane_width ||
           Yo_min_localMap[k] > ref_lane * lane_para.lane_width)
            is_inRefLane[k] = 0;
        else
            is_inRefLane[k] = 1;
    }

    /* 5. 若局部地图内所有障碍物均未侵入车道 -> 走车道中心 */
    if(!std::any_of(is_inRefLane.begin(), is_inRefLane.end(), [](int v){ return v==1;}))
    {
        double Ll = lane_para.lane_begin_y + ref_lane * lane_para.lane_width;
        double Lr = lane_para.lane_begin_y + (ref_lane - 1) * lane_para.lane_width;
        return {(Ll+Lr)/2.0, Ll, Lr, 0};
    }

    /* 6. 有障碍物侵入，计算绕行边界 */
    std::vector<double> FreeSp_l(no), FreeSp_r(no);
    std::vector<double> Ll_obs(no), Lr_obs(no);

    for (size_t k = 0; k < no; ++k){
        FreeSp_l[k] = Ll_lane - Yo_max_Vlength[k];
        FreeSp_r[k] = Yo_min_Vlength[k] - Lr_lane;

        if (FreeSp_l[k] >= FreeSp_r[k]){
            Ll_obs[k] = std::numeric_limits<double>::infinity();
            Lr_obs[k] = Yo_max_Vlength[k];
        } else {
            Ll_obs[k] = Yo_min_Vlength[k];
            Lr_obs[k] = -std::numeric_limits<double>::infinity();
        }
    }

    /* 7. 合并车道边界与障碍物边界 */
    Ll_obs.push_back(Ll_lane);
    Lr_obs.push_back(Lr_lane);

    // for(size_t k=0; k<Ll_obs.size(); k++){
    //     std::cout << "Ll_obs[" << k << "] = " << Ll_obs[k] << ", Lr_obs[" << k << "] = " << Lr_obs[k] << "\n";
    // }

    double Ll = *std::min_element(Ll_obs.begin(), Ll_obs.end());
    double Lr = *std::max_element(Lr_obs.begin(), Lr_obs.end());
    double yc = (Ll + Lr) / 2.0;

    return { yc, Ll, Lr, 1 };
}

DrivableRange CB_Path_Planning::calcDrivableRange(Vec2V obs_edge_points, double ego_pos_x, double ref_points_y)
{
    double lane1_middle_y = lane_para.lane_begin_y + lane_para.lane_width * 0.5;
    double ref_lane_tmp = (ref_points_y - lane1_middle_y)/lane_para.lane_width + 1.0;
    int ref_lane = static_cast<int>(ref_lane_tmp);
    if(ref_lane_tmp == static_cast<double>(ref_lane))
    {
        return calcDrivableRange(obs_edge_points, ego_pos_x, ref_lane);
    }

    // 剩下的情况是参考路径不在车道中心线的情况（变道情况）

    /* 1. 检查输入障碍物点的数量 */
    Vec2V pts = obs_edge_points;
    if (pts.size() % 4 != 0){
        std::cerr << "顶点数必须是 4 的倍数\n";
        return {};
    }

    size_t obs_num = pts.size() / 4;

    /* 2. 先筛选在 [ego_pos_x-ego_length/2, ego_pos_x+ego_length/2] 范围内的障碍物 */
    Vec2V obs_eps_Vlength;                  // 存有效障碍物顶点（连续 4 个一组）
    std::vector<double> Yo_max_Vlength;
    std::vector<double> Yo_min_Vlength;
    size_t no = 0;                          // 有效障碍物计数

    double x_low = ego_pos_x - ego_length/2.0;
    double x_high = ego_pos_x + ego_length/2.0;

    for(size_t k=0; k<obs_num; k++){
        Vec2V bbox(pts.begin() + k * 4, pts.begin() + (k + 1) * 4);
        Vec2V eps = calcObsEdgePoints(bbox, x_low, x_high);
        if (!eps.empty()){
            /* 记录整个障碍物 4 顶点 */
            obs_eps_Vlength.insert(obs_eps_Vlength.end(), bbox.begin(), bbox.end());
            
            /* 记录该障碍物在纵向范围内的 Y 极值 */
            auto mmy = std::minmax_element(eps.begin(), eps.end(), [](const Point2D& a, const Point2D& b) {return a.y < b.y; });
            Yo_max_Vlength.push_back(mmy.second->y);
            Yo_min_Vlength.push_back(mmy.first ->y);
            no ++;
        }
    }

    /* 3.车身纵向范围内无障碍物 -> 沿着参考线走 */
    if(no == 0){
        double Ll = lane_para.lane_begin_y + (ref_lane + 1) * lane_para.lane_width;
        double Lr = lane_para.lane_begin_y + (ref_lane - 1) * lane_para.lane_width;
        return {ref_points_y, Ll, Lr, 0};
    }

    /* 4.再计算这些障碍物在局部地图 X 范围中的 Y 极值 */
    std::vector<double> Yo_max_localMap(no);
    std::vector<double> Yo_min_localMap(no);
    std::vector<int>    is_inRefLane(no, 0);

    double map_x_low = -ego_length/2.0;
    double map_x_high = local_map_length + ego_length/2.0;

    double Ll_lane = lane_para.lane_begin_y + std::min(ref_lane + 1, lane_para.lane_num) * lane_para.lane_width;
    double Lr_lane = lane_para.lane_begin_y + std::max(ref_lane - 2, 0) * lane_para.lane_width;

    for(size_t k=0; k<no; k++){
        Vec2V bbox(obs_eps_Vlength.begin() + k * 4, obs_eps_Vlength.begin() + (k + 1) * 4);
        Vec2V eps = calcObsEdgePoints(bbox, map_x_low, map_x_high);
        auto mmy = std::minmax_element(eps.begin(), eps.end(), [](const Point2D& a, const Point2D& b) {return a.y < b.y;});
        Yo_max_localMap[k] = mmy.second->y;
        Yo_min_localMap[k] = mmy.first->y;

        /* 判断是否侵入当前车道 */
        if(Yo_max_localMap[k] < (ref_lane - 1)*lane_para.lane_width ||
           Yo_min_localMap[k] > (ref_lane + 1) * lane_para.lane_width)
            is_inRefLane[k] = 0;
        else
            is_inRefLane[k] = 1;
    }

    /* 5. 若局部地图内所有障碍物均未侵入两个车道 -> 走车道中心 */
    if(!std::any_of(is_inRefLane.begin(), is_inRefLane.end(), [](int v){ return v==1;}))
    {
        double Ll = lane_para.lane_begin_y + (ref_lane + 1) * lane_para.lane_width;
        double Lr = lane_para.lane_begin_y + (ref_lane - 1) * lane_para.lane_width;
        return {ref_points_y, Ll, Lr, 0};
    }

    /* 6. 有障碍物侵入，计算绕行边界 */
    std::vector<double> FreeSp_l(no), FreeSp_r(no);
    std::vector<double> Ll_obs(no), Lr_obs(no);

    for (size_t k = 0; k < no; ++k){
        FreeSp_l[k] = Ll_lane - Yo_max_Vlength[k];
        FreeSp_r[k] = Yo_min_Vlength[k] - Lr_lane;

        if (FreeSp_l[k] >= FreeSp_r[k]){
            Ll_obs[k] = std::numeric_limits<double>::infinity();
            Lr_obs[k] = Yo_max_Vlength[k];
        } else {
            Ll_obs[k] = Yo_min_Vlength[k];
            Lr_obs[k] = -std::numeric_limits<double>::infinity();
        }
    }

    /* 7. 合并车道边界与障碍物边界 */
    Ll_obs.push_back(Ll_lane);
    Lr_obs.push_back(Lr_lane);

    double Ll = *std::min_element(Ll_obs.begin(), Ll_obs.end());
    double Lr = *std::max_element(Lr_obs.begin(), Lr_obs.end());
    double yc = (Ll + Lr) / 2.0;

    return { yc, Ll, Lr, 1 };
}

CostResult CB_Path_Planning::calcCost(int state, double y0, double y1, double dy0, double ddy, double dx, double dx1, double dx3, double dx4, 
                    std::vector<double> X_local_set, std::vector<double> Yc_set, std::vector<int> NoObs_set)
{
    //初始化变量
    // const double dis0 = 0.1;
    const double W11  = 10.0;
    const double W12  = 0.5;
    const double W2   = 0.0;
    // const double W3   = 1.0;

    double dis        = dx + dx4;
    size_t points_num = 0;
    double y_deviation_sum = 0.0;
    double y_longitude_sum = 0.0;
    size_t k_max = static_cast<size_t>(std::ceil(dis/2.0));
    std::vector<double> Disafemin_set(k_max, 0.0);

    for (size_t k = 0; k < k_max; ++k){
        double ego_pos_x = X_local_set[k];
        double ego_pos_y = calcTrajPoint(state, y0, y1, dy0, ddy, dx, dx1, dx3, ego_pos_x);
        points_num ++;

        double refY = Yc_set[k];
        int NoObs = NoObs_set[k];
        if(NoObs)
            y_deviation_sum += W12 * std::fabs(ego_pos_y - refY);
        else
            y_deviation_sum += W11 * std::fabs(ego_pos_y - refY);
    }

    double y_deviation_cost = y_deviation_sum / points_num;
    double y_longitude_cost = y_longitude_sum / points_num;
    double ay_cost = W2 * std::fabs(ddy);

    /* 总代价 */
    double cost = y_deviation_cost + 50.0 * y_longitude_cost + ay_cost;
    return {cost, Disafemin_set, y_longitude_cost};
}


}