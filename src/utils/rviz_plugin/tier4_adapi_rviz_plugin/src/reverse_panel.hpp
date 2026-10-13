//  Copyright 2024 ZPMC
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef REVERSE_PANEL_HPP_
#define REVERSE_PANEL_HPP_

#ifndef Q_MOC_RUN
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <reverse_vehicle_msg/srv/set_vehicle_reverse_state.hpp>
#include <reverse_vehicle_msg/msg/vehicle_reversed.hpp>
#endif

namespace tier4_adapi_rviz_plugins
{

class ReversePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ReversePanel(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:
  void onToggleReverse();

protected:
  void onReverseState(const reverse_vehicle_msg::msg::VehicleReversed::ConstSharedPtr msg);
  void updateButtonState(bool is_reversed);

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Client<reverse_vehicle_msg::srv::SetVehicleReverseState>::SharedPtr reverse_client_;
  rclcpp::Subscription<reverse_vehicle_msg::msg::VehicleReversed>::SharedPtr reverse_state_sub_;

  QPushButton * toggle_button_;
  QLabel * status_label_;

  bool current_reverse_state_{false};
};

}  // namespace tier4_adapi_rviz_plugins

#endif  // REVERSE_PANEL_HPP_
