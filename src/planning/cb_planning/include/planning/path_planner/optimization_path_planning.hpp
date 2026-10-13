#ifndef CRABWALKING_PATH_PLANNING_HPP_
#define CRABWALKING_PATH_PLANNING_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <config_reader.hpp>
#include "struct_types.hpp"

namespace cb_planning
{

enum class SLPointType
{
    LEFT_PASS,      //从左侧经过
    RIGHT_PASS,     //从右侧经过
    STOP,           //停车点
    START,          //整个过程的起点
    END             //整个过程的终点
};    

struct SLPoint
{
    double s_ = 0.0;
    double l_ = 0.0;
    int type_ = 0;
};

class OPT_Path_Planning{
public:
    explicit OPT_Path_Planning();
    ~OPT_Path_Planning() = default;

    void make_path_decision(const std::shared_ptr<Ego_Info> &car,
                            const std::vector<std::shared_ptr<ObstacleStruct>> &obstacles);
    inline std::vector<SLPoint> sl_points() const { return sl_points_; }

    std::vector<Point2D> create_local_path(int ref_lane, const std::shared_ptr<Ego_Info> &car, std::vector<SLPoint> &sl_points);

private:
    std::unique_ptr<ConfigReader> opt_planning_config;
    std::vector<SLPoint> sl_points_;   //变道点位


};
}

#endif