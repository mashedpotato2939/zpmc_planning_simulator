// Copyright 2024 ZPMC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef IGV_PATH_PLANNER__FMS_PATH_CONVERTER_NODE_HPP_
#define IGV_PATH_PLANNER__FMS_PATH_CONVERTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/path_point.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <igv_planning_msgs/msg/fms_route_window.hpp>
#include <igv_planning_msgs/msg/fms_status.hpp>
#include <igv_planning_msgs/msg/fms_stop_point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <reverse_vehicle_msg/msg/vehicle_reversed.hpp>
#include <reverse_vehicle_msg/srv/set_vehicle_reverse_state.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include "igv_path_planner/fms_path_planning.hpp"
#include "igv_path_planner/avoidance_planner/crab_avoidance_planner.hpp"
#include "igv_path_planner/protocols/fms_protocol.hpp"
#include "igv_path_planner/map_path_generator.hpp"
#include "igv_path_planner/path_generator.hpp"

#include <memory>
#include <mutex>
#include <optional>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace igv_path_planner
{

class FMSPathConverterNode : public rclcpp::Node
{
public:
  explicit FMSPathConverterNode(const rclcpp::NodeOptions & options);
  ~FMSPathConverterNode() override = default;

private:
  void onRouteWindow(const igv_planning_msgs::msg::FMSRouteWindow::SharedPtr msg);
  void onStopPoint(const igv_planning_msgs::msg::FMSStopPoint::SharedPtr msg);
  void onObjects(const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();
  void updateSegmentPath();
  void holdSegmentPath(const std::string & reason);

  autoware_planning_msgs::msg::Path convertRouteWindowToAutowarePath(
    const igv_planning_msgs::msg::FMSRouteWindow & window);
  autoware_planning_msgs::msg::PathPoint makePathPoint(
    const igv_planning_msgs::msg::SegmentWaypoint & waypoint, bool is_reverse);
  std::vector<autoware_planning_msgs::msg::PathPoint> generateSubsegmentPath(
    const std::vector<autoware_planning_msgs::msg::PathPoint> & points,
    planning::SubsegmentType type, bool is_reverse);
  std::vector<autoware_planning_msgs::msg::PathPoint> extractPreviousFrameSubpath(
    const std::vector<autoware_planning_msgs::msg::PathPoint> & visible_points) const;
  void applyCrabAvoidance(
    autoware_planning_msgs::msg::Path & path,
    const std::vector<bool> & avoidance_allowed,
    const std::vector<std::pair<size_t, size_t>> & protected_ranges,
    bool is_reverse);

  bool publishPathFromReference();
  void publishStatus();
  void publishReverseState();

  void connectEgoToPath(autoware_planning_msgs::msg::Path & path);
  void truncatePathFromEgo(autoware_planning_msgs::msg::Path & path);
  void interpolatePath(autoware_planning_msgs::msg::Path & path);
  void applyStopPoint(autoware_planning_msgs::msg::Path & path);
  void generateDrivableBounds(autoware_planning_msgs::msg::Path & path);
  void updateReverseStateFromPath(const autoware_planning_msgs::msg::Path & path);
  void updateReverseState(bool is_reverse, bool force_publish = false);
  double computeLateralDeviationFromCachedPath() const;

  unique_identifier_msgs::msg::UUID generateUUID();
  void handleReverseStateService(
    const std::shared_ptr<reverse_vehicle_msg::srv::SetVehicleReverseState::Request> request,
    std::shared_ptr<reverse_vehicle_msg::srv::SetVehicleReverseState::Response> response);

  std::unique_ptr<FMSProtocol> protocol_;
  std::unique_ptr<PathGenerator> path_generator_;
  std::unique_ptr<cb_planning::CB_Path_Planning> crab_avoidance_planner_;
  std::unique_ptr<MapPathGenerator> map_path_generator_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr objects_sub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_pub_;
  rclcpp::Publisher<igv_planning_msgs::msg::FMSStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr maneuver_type_pub_;
  rclcpp::Publisher<reverse_vehicle_msg::msg::VehicleReversed>::SharedPtr reverse_state_pub_;
  rclcpp::Service<reverse_vehicle_msg::srv::SetVehicleReverseState>::SharedPtr reverse_state_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex mutex_;
  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_;
  igv_planning_msgs::msg::FMSRouteWindow::SharedPtr last_window_;
  autoware_planning_msgs::msg::Path::SharedPtr last_reference_path_;
  autoware_planning_msgs::msg::Path::SharedPtr last_output_path_;
  autoware_planning_msgs::msg::Path::SharedPtr last_generated_path_;
  std::optional<igv_planning_msgs::msg::FMSStopPoint> active_stop_;
  planning::RouteSegmentSelector route_segment_selector_;

  rclcpp::Time last_fms_message_time_;
  std::optional<rclcpp::Time> last_odometry_time_;
  std::optional<rclcpp::Time> last_objects_time_;
  bool fms_connected_{false};
  bool current_reverse_state_{false};
  bool last_reverse_state_{false};
  bool has_original_goal_{false};
  bool crab_avoidance_enabled_{false};
  cb_planning::Planning_Para crab_planning_params_{};
  double crab_max_heading_curvature_{0.05};
  double crab_max_heading_deviation_{0.25};
  bool last_plan_used_fresh_objects_{false};
  double crab_avoidance_object_timeout_{1.0};
  uint64_t objects_revision_{0};
  uint64_t last_planned_objects_revision_{0};
  std::string last_error_;
  std::string last_published_route_id_;
  std::string last_published_base_route_id_;
  std::string vehicle_id_;
  geometry_msgs::msg::Point original_goal_position_;

  std::string frame_id_;
  double default_velocity_{5.0};
  double min_velocity_{0.5};
  double max_velocity_{15.0};
  double path_interval_{1.0};
  double forward_path_length_{100.0};
  double backward_path_length_{5.0};
  double stale_threshold_{5.0};
  double ego_nearest_dist_threshold_{3.0};
  double ego_nearest_yaw_threshold_{1.57};
  double replan_lateral_deviation_threshold_{1.0};
  double publish_rate_{10.0};

  bool generate_drivable_area_{true};
  double left_bound_offset_{1.75};
  double right_bound_offset_{1.75};
  double map_snap_distance_threshold_{5.0};
};

}  // namespace igv_path_planner

#endif  // IGV_PATH_PLANNER__FMS_PATH_CONVERTER_NODE_HPP_
