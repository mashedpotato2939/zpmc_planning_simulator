// Copyright 2024 ZPMC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <memory>
#include <mutex>
#include <string>

namespace igv_velocity_planner
{

class TrajectorySwitchNode : public rclcpp::Node
{
public:
  explicit TrajectorySwitchNode(const rclcpp::NodeOptions & options)
  : Node("trajectory_switch", options)
  {
    // Parameters
    use_fms_ = declare_parameter<bool>("use_fms_trajectory", false);
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);
    timeout_ = declare_parameter<double>("timeout", 1.0);

    // Subscribers
    fms_traj_sub_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
      "~/input/fms_trajectory", rclcpp::QoS(10),
      std::bind(&TrajectorySwitchNode::fmsCallback, this, std::placeholders::_1));

    autoware_traj_sub_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
      "~/input/autoware_trajectory", rclcpp::QoS(10),
      std::bind(&TrajectorySwitchNode::autowareCallback, this, std::placeholders::_1));

    // Publisher
    output_pub_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
      "~/output/trajectory", rclcpp::QoS(10));

    // Service to switch mode
    switch_service_ = create_service<std_srvs::srv::SetBool>(
      "~/switch_to_fms",
      std::bind(&TrajectorySwitchNode::switchCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    // Dynamic parameter callback
    param_callback_ = add_on_set_parameters_callback(
      std::bind(&TrajectorySwitchNode::paramCallback, this, std::placeholders::_1));

    // Timer
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TrajectorySwitchNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "Trajectory Switch Node initialized");
    RCLCPP_INFO(get_logger(), "  Initial mode: %s", use_fms_ ? "FMS" : "Autoware");
  }

private:
  void fmsCallback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fms_traj_ = msg;
    fms_last_time_ = this->now();
  }

  void autowareCallback(const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    autoware_traj_ = msg;
    autoware_last_time_ = this->now();
  }

  void timerCallback()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    autoware_planning_msgs::msg::Trajectory::SharedPtr selected_traj;

    if (use_fms_) {
      // Check if FMS trajectory is available and not stale
      if (fms_traj_ && (this->now() - fms_last_time_).seconds() < timeout_) {
        selected_traj = fms_traj_;
      } else {
        // Fallback to Autoware if FMS is not available
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "FMS trajectory not available, falling back to Autoware");
        if (autoware_traj_ && (this->now() - autoware_last_time_).seconds() < timeout_) {
          selected_traj = autoware_traj_;
        }
      }
    } else {
      // Use Autoware trajectory
      if (autoware_traj_ && (this->now() - autoware_last_time_).seconds() < timeout_) {
        selected_traj = autoware_traj_;
      }
    }

    if (selected_traj) {
      auto output = *selected_traj;
      output.header.stamp = this->now();
      output_pub_->publish(output);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No valid trajectory available");
    }
  }

  void switchCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    use_fms_ = request->data;
    set_parameter(rclcpp::Parameter("use_fms_trajectory", use_fms_));

    response->success = true;
    response->message = use_fms_ ? "Switched to FMS trajectory" : "Switched to Autoware trajectory";

    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & param : params) {
      if (param.get_name() == "use_fms_trajectory") {
        use_fms_ = param.as_bool();
        RCLCPP_INFO(get_logger(), "Switched to %s trajectory via parameter",
          use_fms_ ? "FMS" : "Autoware");
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // Subscribers
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr fms_traj_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr autoware_traj_sub_;

  // Publisher
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr output_pub_;

  // Service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr switch_service_;

  // Parameter callback
  OnSetParametersCallbackHandle::SharedPtr param_callback_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  std::mutex mutex_;
  autoware_planning_msgs::msg::Trajectory::SharedPtr fms_traj_;
  autoware_planning_msgs::msg::Trajectory::SharedPtr autoware_traj_;
  rclcpp::Time fms_last_time_;
  rclcpp::Time autoware_last_time_;

  // Parameters
  bool use_fms_;
  double publish_rate_;
  double timeout_;
};

}  // namespace igv_velocity_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(igv_velocity_planner::TrajectorySwitchNode)
