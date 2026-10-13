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

#include "reverse_panel.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <memory>

namespace tier4_adapi_rviz_plugins
{

ReversePanel::ReversePanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // Create toggle button
  toggle_button_ = new QPushButton("Enable Reverse");
  toggle_button_->setMinimumHeight(50);
  toggle_button_->setStyleSheet(
    "QPushButton { background-color: #4CAF50; color: white; font-size: 16px; font-weight: bold; "
    "border-radius: 8px; } "
    "QPushButton:hover { background-color: #45a049; }");

  // Create status label
  status_label_ = new QLabel("Reverse Mode: DISABLED");
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setStyleSheet("font-size: 14px; font-weight: bold; color: #333;");

  // Connect button click
  connect(toggle_button_, SIGNAL(clicked()), this, SLOT(onToggleReverse()));

  // Create layout
  auto * layout = new QVBoxLayout;
  layout->addWidget(status_label_);
  layout->addWidget(toggle_button_);
  layout->addStretch();
  setLayout(layout);
}

void ReversePanel::onInitialize()
{
  raw_node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  // Create service client
  reverse_client_ = raw_node_->create_client<reverse_vehicle_msg::srv::SetVehicleReverseState>(
    "/set_reverse_vehicle_state_service");

  // Subscribe to reverse state
  // Use transient_local QoS to receive the current state from reverse_planner
  reverse_state_sub_ = raw_node_->create_subscription<reverse_vehicle_msg::msg::VehicleReversed>(
    "/planning/reverse_state",
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&ReversePanel::onReverseState, this, std::placeholders::_1));

  RCLCPP_INFO(raw_node_->get_logger(), "ReversePanel initialized");
}

void ReversePanel::onToggleReverse()
{
  bool new_state = !current_reverse_state_;

  if (!reverse_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(raw_node_->get_logger(), "Reverse service not available");
    status_label_->setText("Service not available!");
    status_label_->setStyleSheet("font-size: 14px; font-weight: bold; color: #f44336;");
    return;
  }

  auto request = std::make_shared<reverse_vehicle_msg::srv::SetVehicleReverseState::Request>();
  request->is_reversed = new_state;

  auto future = reverse_client_->async_send_request(
    request,
    [this, new_state](
      rclcpp::Client<reverse_vehicle_msg::srv::SetVehicleReverseState>::SharedFuture response) {
      try {
        auto result = response.get();
        if (result->success) {
          current_reverse_state_ = new_state;
          updateButtonState(new_state);
          RCLCPP_INFO(
            raw_node_->get_logger(), "Reverse state set to: %s", new_state ? "ENABLED" : "DISABLED");
        } else {
          RCLCPP_WARN(raw_node_->get_logger(), "Failed to set reverse state");
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(raw_node_->get_logger(), "Service call failed: %s", e.what());
      }
    });
}

void ReversePanel::onReverseState(
  const reverse_vehicle_msg::msg::VehicleReversed::ConstSharedPtr msg)
{
  current_reverse_state_ = msg->is_reversed;
  updateButtonState(msg->is_reversed);
}

void ReversePanel::updateButtonState(bool is_reversed)
{
  if (is_reversed) {
    toggle_button_->setText("Disable Reverse");
    toggle_button_->setStyleSheet(
      "QPushButton { background-color: #f44336; color: white; font-size: 16px; font-weight: bold; "
      "border-radius: 8px; } "
      "QPushButton:hover { background-color: #da190b; }");
    status_label_->setText("Reverse Mode: ENABLED");
    status_label_->setStyleSheet("font-size: 14px; font-weight: bold; color: #f44336;");
  } else {
    toggle_button_->setText("Enable Reverse");
    toggle_button_->setStyleSheet(
      "QPushButton { background-color: #4CAF50; color: white; font-size: 16px; font-weight: bold; "
      "border-radius: 8px; } "
      "QPushButton:hover { background-color: #45a049; }");
    status_label_->setText("Reverse Mode: DISABLED");
    status_label_->setStyleSheet("font-size: 14px; font-weight: bold; color: #4CAF50;");
  }
}

}  // namespace tier4_adapi_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tier4_adapi_rviz_plugins::ReversePanel, rviz_common::Panel)
