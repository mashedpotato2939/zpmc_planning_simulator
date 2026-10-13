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

#include "tier4_planning_rviz_plugin/mission_checkpoint/mission_checkpoint.hpp"

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/property.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreColourValue.h>

#include <pluginlib/class_list_macros.hpp>

namespace rviz_plugins
{

MissionCheckpointTool::MissionCheckpointTool()
: rviz_default_plugins::tools::PoseTool()
{
  set_route_property_ = new rviz_common::properties::BoolProperty(
      "Set Route Points", false,
      "发送当前收集的路径点到路径规划服务", 
      getPropertyContainer(),  
      SLOT(onSetRouteClicked()), this);
}

MissionCheckpointTool::~MissionCheckpointTool()
{
  for (auto arrow : arrows_) {
    delete arrow;
  }
  arrows_.clear();
}

void MissionCheckpointTool::onInitialize()
{
  PoseTool::onInitialize(); 

  ros_node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

  route_client_ = ros_node_->create_client<autoware_adapi_v1_msgs::srv::SetRoutePoints>(
      "/api/routing/set_route_points");
  
  scene_manager_ = context_->getSceneManager();
  frame_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();


}

void MissionCheckpointTool::onPoseSet(double x, double y, double theta)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0; 
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  pose.orientation = tf2::toMsg(q);
  route_points_.push_back(pose);

  rviz_rendering::Arrow* arrow = new rviz_rendering::Arrow(
      scene_manager_, frame_node_, 
      2.0f, 0.2f,   
      0.4f, 0.4f);  
  arrow->setPosition(Ogre::Vector3(x, y, 0.0f));

  tf2::Quaternion orientation_quat;
  tf2::Quaternion yaw_quat;
  yaw_quat.setRPY(0.0, 0.0, theta);         
  tf2::Quaternion tilt_quat;
  tilt_quat.setRPY(0.0, -M_PI_2, 0.0);      
  orientation_quat = yaw_quat * tilt_quat;  
  Ogre::Quaternion ogre_quat(
      orientation_quat.getW(),
      orientation_quat.getX(),
      orientation_quat.getY(),
      orientation_quat.getZ());
  arrow->setOrientation(ogre_quat);

  arrow->setColor(Ogre::ColourValue(1.0f, 0.9f, 0.2f, 0.9f));

  arrows_.push_back(arrow);
}

void MissionCheckpointTool::onSetRouteClicked()
{
  if (!set_route_property_->getBool()) {
    return;
  }

  // RCLCPP_INFO(ros_node_->get_logger(), "........................ %zu 个路径点请求：", route_points_.size());
  // for (size_t i = 0; i < route_points_.size(); ++i) {
  //   const auto &p = route_points_[i];
  //   RCLCPP_INFO(
  //     ros_node_->get_logger(),
  //     "    [%2zu] x = %.3f, y = %.3f, z = %.3f", 
  //     i,
  //     p.position.x,
  //     p.position.y,
  //     p.position.z
  //   );
  // }

if (route_points_.empty()) return;

  int iLastPoint = static_cast<int>(route_points_.size()) - 1;

  auto request = std::make_shared<autoware_adapi_v1_msgs::srv::SetRoutePoints::Request>();

  request->header.stamp = ros_node_->now();
  request->header.frame_id = "map";

  request->option.allow_goal_modification = true;

  for (int i = 0; i < iLastPoint; ++i) {
    request->waypoints.push_back(route_points_[i]);
  }

  request->goal = route_points_[iLastPoint];

  // RCLCPP_INFO(
  //   ros_node_->get_logger(),
  //   "........................ [SetRoute] header.frame_id='%s', stamp=(%u.%u)",
  //   request->header.frame_id.c_str(),
  //   request->header.stamp.sec,
  //   request->header.stamp.nanosec
  // );

  // for (size_t i = 0; i < request->waypoints.size(); ++i) {
  //   const auto &p = request->waypoints[i];
  //   tf2::Quaternion quat;
  //   tf2::fromMsg(p.orientation, quat);
  //   double roll, pitch, yaw;
  //   tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  //   RCLCPP_INFO(
  //     ros_node_->get_logger(),
  //     "........................ [SetRoute] waypoint[%2zu] = (x=%.3f, y=%.3f, z=%.3f, yaw=%.3f)",
  //     i,
  //     p.position.x,
  //     p.position.y,
  //     p.position.z,
  //     yaw
  //   );
  // }

  // const auto &g = request->goal;
  // RCLCPP_INFO(
  //   ros_node_->get_logger(),
  //   "........................ [SetRoute] goal = (x=%.3f, y=%.3f, z=%.3f)",
  //   g.position.x,
  //   g.position.y,
  //   g.position.z
  // );

  route_client_->async_send_request(
    request,
    [iLastPoint, this](
      rclcpp::Client<autoware_adapi_v1_msgs::srv::SetRoutePoints>::SharedFuture future
    ) {
      auto response = future.get();
      const auto& status = response->status;
      if (status.success) {
        RCLCPP_INFO(
          ros_node_->get_logger(),
          "SetRoutePoints Success"
        );
      } else {
        RCLCPP_ERROR(
          ros_node_->get_logger(),
          "SetRoutePoints Failed: %s",
          status.message.c_str()
        );
      }
    }
  );


  set_route_property_->setBool(false);
  route_points_.clear();
  for (auto arrow : arrows_) {
    delete arrow;
  }
  arrows_.clear();



}



}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::MissionCheckpointTool, rviz_common::Tool)
