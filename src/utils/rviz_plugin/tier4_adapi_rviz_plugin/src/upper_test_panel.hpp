//  Copyright 2026 ZPMC
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

#ifndef UPPER_TEST_PANEL_HPP_
#define UPPER_TEST_PANEL_HPP_

#include <pacmod_msgs/srv/upper_test.hpp>
#include <rclcpp/rclcpp.hpp>
#include <reverse_vehicle_msg/msg/vehicle_reversed.hpp>
#include <reverse_vehicle_msg/srv/set_vehicle_reverse_state.hpp>
#include <rviz_common/panel.hpp>

#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>

namespace tier4_adapi_rviz_plugins
{

class UpperTestPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit UpperTestPanel(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:
  void onEnable4ws();
  void onDisable4ws();
  void onEnableJog();
  void onDisableJog();
  void onSetForward();
  void onSetReverse();

private:
  using UpperTest = pacmod_msgs::srv::UpperTest;
  using VehicleReversed = reverse_vehicle_msg::msg::VehicleReversed;
  using SetVehicleReverseState = reverse_vehicle_msg::srv::SetVehicleReverseState;

  void sendRequest(uint32_t index, bool enable, double float_cmd, const QString & command_name);
  void sendReverseStateRequest(bool is_reversed);
  void onReverseState(const VehicleReversed::ConstSharedPtr msg);
  void updateReverseStateDisplay(bool is_reversed);
  void setStatus(const QString & text, const QString & color);
  void setReverseStatus(const QString & text, const QString & color);

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Client<UpperTest>::SharedPtr upper_test_client_;
  rclcpp::Client<SetVehicleReverseState>::SharedPtr reverse_state_client_;
  rclcpp::Subscription<VehicleReversed>::SharedPtr reverse_state_sub_;

  QLabel * status_label_{nullptr};
  QLabel * reverse_state_label_{nullptr};
  QDoubleSpinBox * jog_distance_input_{nullptr};
  QPushButton * enable_4ws_button_{nullptr};
  QPushButton * disable_4ws_button_{nullptr};
  QPushButton * enable_jog_button_{nullptr};
  QPushButton * disable_jog_button_{nullptr};
  QPushButton * set_forward_button_{nullptr};
  QPushButton * set_reverse_button_{nullptr};
  bool current_reverse_state_{false};
};

}  // namespace tier4_adapi_rviz_plugins

#endif  // UPPER_TEST_PANEL_HPP_
