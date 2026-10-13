#ifndef CONFIG_READER_H_
#define CONFIG_READER_H_

#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <unordered_map>

namespace cb_planning
{
    struct ObstacleStruct
    {
        int id_ = 0;
        std::string frame_ = "";
        double length_ = 0.0;
        double width_ = 0.0;
        double pose_x_ = 0.0;
        double pose_y_ = 0.0;
        double pose_theta_ = 0.0;  // 航向角
        double speed_ori_ = 0.0;   // 初速度
    };

    struct LaneStruct
    {
        std::string frame_ = "";
        double lane_begin_x = 0.0;          // 道路起点的x坐标
        double lane_end_x = 0.0;            // 道路终点的x坐标
        double lane_begin_y = 0.0;          // 道路起点的y坐标
        double lane_end_y = 0.0;            // 道路终点的y坐标
        double lane_width = 0.0;            // 道路分段长度
        int lane_num = 0;                   // 车道数量
        double lane_change_start_x = 0.0;
        double lane_change_end_x = 0.0;
    };

    struct VehicleStruct
    {
        std::string frame_ = "";
        double ego_length = 0.0;
        double ego_width = 0.0;
        double steer_angle_max = 0.0;
        double steer_angular_vel_max = 0.0;
    };

    struct PlanningStruct
    {
        double dest_x = 0.0;
        double Min_Collision_free_distance = 0.0;
        double lane_sampling_begin_y = 0.0;       
        double lane_sampling_end_y = 0.0;
        double lane_x_sampling_interval = 0.0;
        double lane_y_sampling_interval = 0.0;
        double local_map_forward_x = 0.0;
        double local_map_backward_x = 0.0;
        double vx_ref = 0.0;
        double ax_drive = 0.0;
        double ax_brake = 0.0;
    };

    class ConfigReader
    {
        public:
            ConfigReader();

            // obstacle
            void read_obstacle_config(ObstacleStruct &vehicle, const std::string &name);
            void read_obstacle_config();
            inline ObstacleStruct obs1() const { return obs_1_; }
            inline ObstacleStruct obs2() const { return obs_2_; }
            inline ObstacleStruct obs3() const { return obs_3_; }
            inline ObstacleStruct obs4() const { return obs_4_; }
            inline ObstacleStruct obs5() const { return obs_5_; }
            inline std::unordered_map<int, ObstacleStruct> obs_s() const { return obs_s_; }

            void read_moving_obstacle_config();
            inline ObstacleStruct moving_obs() const { return moving_obs_1_; }

            // pnc_map 
            void read_lane_config();
            inline LaneStruct lane() const { return lane_; }

            // vehicle
            void read_vehicle_config();
            inline VehicleStruct vehicle() const {return vehicle_; }

            // planning
            void read_planning_config();
            inline PlanningStruct planning() const { return planning_; }

        private:
            YAML::Node planning_config;

            ObstacleStruct obs_1_;
            ObstacleStruct obs_2_;
            ObstacleStruct obs_3_;
            ObstacleStruct obs_4_;
            ObstacleStruct obs_5_;
            ObstacleStruct moving_obs_1_;
            std::unordered_map<int, ObstacleStruct> obs_s_;

            //pnc_map
            LaneStruct lane_;
            VehicleStruct vehicle_;
            PlanningStruct planning_;
    };
}  
#endif  //CONFIG_READER_H_
