// Copyright 2024 ZPMC. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef FMS_POINT_TOOL_HPP_
#define FMS_POINT_TOOL_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>

namespace fms_bridge_rviz_plugin
{

class FmsPointTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  FmsPointTool();
  ~FmsPointTool() override;
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace fms_bridge_rviz_plugin

#endif  // FMS_POINT_TOOL_HPP_
