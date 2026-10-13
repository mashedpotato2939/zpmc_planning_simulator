#include "config_reader.hpp"

namespace cb_planning
{
    ConfigReader::ConfigReader()  //配置文件读取
    {
        // 获取workspace/install/planning/share/planning/ 目录路径
        std::string planning_share_directory = ament_index_cpp::get_package_share_directory("cb_planning");

        // 然后获取配置文件
        planning_config = YAML::LoadFile(planning_share_directory + "/config/crabwalking_planning_config.yaml");
    }

    /************obstacle*************/
    void ConfigReader::read_obstacle_config(ObstacleStruct &obs, const std::string &name)
    {   
        obs.id_ = planning_config["obstacle"][name]["id"].as<int>();
        obs.frame_ = planning_config["obstacle"][name]["frame"].as<std::string>();
        obs.length_ = planning_config["obstacle"][name]["length"].as<double>();
        obs.width_ = planning_config["obstacle"][name]["width"].as<double>();
        obs.pose_x_ = planning_config["obstacle"][name]["global_pos_x"].as<double>();
        obs.pose_y_ = planning_config["obstacle"][name]["global_pos_y"].as<double>();
        obs.pose_theta_ = planning_config["obstacle"][name]["global_heading_angle"].as<double>();

        obs_s_.emplace(obs.id_, obs);
    }

    void ConfigReader::read_moving_obstacle_config()
    {
        moving_obs_1_.id_ = planning_config["moving_obstacle"]["obs1"]["id"].as<int>();
        moving_obs_1_.frame_ = planning_config["moving_obstacle"]["obs1"]["frame"].as<std::string>();
        moving_obs_1_.length_ = planning_config["moving_obstacle"]["obs1"]["length"].as<double>();
        moving_obs_1_.width_ = planning_config["moving_obstacle"]["obs1"]["width"].as<double>();
        moving_obs_1_.pose_x_ = planning_config["moving_obstacle"]["obs1"]["initial_pos_x"].as<double>();
        moving_obs_1_.pose_y_ = planning_config["moving_obstacle"]["obs1"]["initial_pos_y"].as<double>();
        moving_obs_1_.pose_theta_ = planning_config["moving_obstacle"]["obs1"]["initial_heading_angle"].as<double>();
        moving_obs_1_.speed_ori_ = planning_config["moving_obstacle"]["obs1"]["moving_velocity"].as<double>();
    }

    void ConfigReader::read_obstacle_config()
    {
        try
        {
            read_obstacle_config(obs_1_, "obs1");
            read_obstacle_config(obs_2_, "obs2");
            read_obstacle_config(obs_3_, "obs3");
            read_obstacle_config(obs_4_, "obs4");
            read_obstacle_config(obs_5_, "obs5");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load obstacle config: %s", e.what());
        }
    }

    /************lane*************/
    void ConfigReader::read_lane_config()
    {
        try
        {
            lane_.frame_ = planning_config["lane"]["frame"].as<std::string>();
            lane_.lane_begin_x = planning_config["lane"]["lane_begin_x"].as<double>();
            lane_.lane_end_x = planning_config["lane"]["lane_end_x"].as<double>();
            lane_.lane_begin_y = planning_config["lane"]["lane_begin_y"].as<double>();
            lane_.lane_end_y = planning_config["lane"]["lane_end_y"].as<double>();
            lane_.lane_width = planning_config["lane"]["lane_width"].as<double>();
            lane_.lane_num = planning_config["lane"]["lane_num"].as<int>();
            lane_.lane_change_start_x = planning_config["lane"]["lane_change_start_x"].as<double>();
            lane_.lane_change_end_x = planning_config["lane"]["lane_change_end_x"].as<double>();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load lane config: %s", e.what());
        }
    }

     /************vehicle*************/
    void ConfigReader::read_vehicle_config()
    {
        try
        {
            vehicle_.frame_ = planning_config["vehicle"]["frame"].as<std::string>();
            vehicle_.ego_length = planning_config["vehicle"]["ego_length"].as<double>();
            vehicle_.ego_width = planning_config["vehicle"]["ego_width"].as<double>();
            vehicle_.steer_angle_max = planning_config["vehicle"]["steer_angle_max"].as<double>();
            vehicle_.steer_angular_vel_max = planning_config["vehicle"]["steer_angular_vel_max"].as<double>();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load vehicle config: %s", e.what());
        }
    }

    void ConfigReader::read_planning_config()
    {
        try
        {
            planning_.dest_x = planning_config["planning"]["dest_x"].as<double>();
            planning_.Min_Collision_free_distance = planning_config["planning"]["Min_Collision_free_distance"].as<double>();
            planning_.lane_sampling_begin_y = planning_config["planning"]["lane_sampling_begin_y"].as<double>();
            planning_.lane_sampling_end_y = planning_config["planning"]["lane_sampling_end_y"].as<double>();
            planning_.lane_y_sampling_interval = planning_config["planning"]["lane_y_sampling_interval"].as<double>();
            planning_.lane_x_sampling_interval = planning_config["planning"]["lane_x_sampling_interval"].as<double>();
            planning_.local_map_forward_x = planning_config["planning"]["local_map_forward_x"].as<double>();
            planning_.local_map_backward_x = planning_config["planning"]["local_map_backward_x"].as<double>();
            planning_.vx_ref = planning_config["planning"]["vx_ref"].as<double>();
            planning_.ax_drive = planning_config["planning"]["ax_drive"].as<double>();
            planning_.ax_brake = planning_config["planning"]["ax_brake"].as<double>();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("config"), "Failed to load planning config: %s", e.what());
        }
    }
}
