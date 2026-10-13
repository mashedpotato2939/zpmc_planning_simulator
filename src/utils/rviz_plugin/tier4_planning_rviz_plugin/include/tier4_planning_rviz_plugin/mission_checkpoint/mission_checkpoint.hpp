// Copyright 2020 Tier IV, Inc.
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
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TIER4_PLANNING_RVIZ_PLUGIN__MISSION_ROUTE_POINTS_TOOL_HPP_
#define TIER4_PLANNING_RVIZ_PLUGIN__MISSION_ROUTE_POINTS_TOOL_HPP_

#ifndef Q_MOC_RUN  // https://bugreports.qt-project.org/browse/QTBUG-22829

#include <memory>
#include <vector>

// ROS2 消息与服务类型
#include <geometry_msgs/msg/pose.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>

// RViz 基类和辅助组件
#include <rviz_default_plugins/tools/pose/pose_tool.hpp> 
#include <rviz_common/properties/bool_property.hpp> 
#include <rviz_common/display_context.hpp> 
#include <rviz_rendering/objects/arrow.hpp> 

#include <tf2/LinearMath/Quaternion.h>


#endif

namespace rviz_plugins
{

// 声明自定义工具类
class MissionCheckpointTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  MissionCheckpointTool();
  ~MissionCheckpointTool() override;

  void onInitialize() override;


protected:
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void onSetRouteClicked();

private:
  rviz_common::properties::BoolProperty* set_route_property_;

  rclcpp::Node::SharedPtr ros_node_;  
  rclcpp::Client<autoware_adapi_v1_msgs::srv::SetRoutePoints>::SharedPtr route_client_;

  std::vector<geometry_msgs::msg::Pose> route_points_;

  std::vector<rviz_rendering::Arrow*> arrows_;
  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* frame_node_;


};
}


#endif  // TIER4_PLANNING_RVIZ_PLUGIN__MISSION_ROUTE_POINTS_TOOL_HPP_