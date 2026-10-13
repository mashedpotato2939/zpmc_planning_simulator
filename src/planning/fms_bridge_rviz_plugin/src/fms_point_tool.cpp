// Copyright 2024 ZPMC. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "fms_point_tool.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <string>

namespace fms_bridge_rviz_plugin
{

FmsPointTool::FmsPointTool()
{
  shortcut_key_ = 'p';
}

FmsPointTool::~FmsPointTool() = default;

void FmsPointTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("FMS Pick Point");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/SetGoal.png"));

  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  clock_ = node->get_clock();
  publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/rviz/fms_bridge/point_pose", rclcpp::QoS(1));
}

void FmsPointTool::onPoseSet(double x, double y, double theta)
{
  const std::string fixed_frame = context_->getFixedFrame().toStdString();
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = fixed_frame;
  msg.header.stamp = clock_->now();
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = 0.0;
  msg.pose.orientation = orientationAroundZAxis(theta);
  publisher_->publish(msg);
}

}  // namespace fms_bridge_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fms_bridge_rviz_plugin::FmsPointTool, rviz_common::Tool)
