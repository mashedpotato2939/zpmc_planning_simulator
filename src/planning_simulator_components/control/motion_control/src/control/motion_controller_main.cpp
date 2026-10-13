#include "motion_controller.hpp"

using MotionController = motion_controller::MotionController;
using autoware_control_msgs::srv::MapCreatorService;
using autoware_control_msgs::srv::TrajCreatorService;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionController>(rclcpp::NodeOptions());

    if(node->get_parameter("use_traj_server").as_bool())
    {
        auto map_client_ = node->create_client<MapCreatorService>("pnc_map_server");
        if (!map_client_->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_ERROR(node->get_logger(), "Map service not available");
            return 1;
        }

        auto request = std::make_shared<MapCreatorService::Request>();
        const double bias_x = 12.0;
        const double bias_y = -0.0;
        
        geometry_msgs::msg::Pose p;
        p.orientation.w = 1.0;
        for(size_t i=0; i<8; i++){
            p.position.x = map_edge_points[i][0] + bias_x;
            p.position.y = map_edge_points[i][1] + bias_y;
            p.position.z = map_edge_points[i][2];
            request->pose.emplace_back(p);
        }

        auto result_furture = map_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), result_furture) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "Map response success");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Map response failed");
            rclcpp::shutdown();
            return 1;
        }

        auto traj_client_ = node->create_client<TrajCreatorService>("global_path_server");
        if (!traj_client_->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_ERROR(node->get_logger(), "Traj service not available");
            return 1;
        }
        auto traj_request = std::make_shared<TrajCreatorService::Request>();
        traj_request->trajectory_type = 5;
        traj_request->is_reverse = (!node->get_parameter("is_forward").as_bool()) ? 1 : 0;
        
        auto traj_result_furture = traj_client_->async_send_request(traj_request);
        if (rclcpp::spin_until_future_complete(node->get_node_base_interface(), traj_result_furture) == rclcpp::FutureReturnCode::SUCCESS)
        {
            node->trajectory_ = std::make_shared<autoware_planning_msgs::msg::Trajectory>(traj_result_furture.get()->traj);
            node->is_trajectory_new = true;
            RCLCPP_INFO(node->get_logger(), "global_path response success");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Global path response failed");
            rclcpp::shutdown();
            return 1;
        }

    }
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}