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

#include "upper_test_panel.hpp"

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <memory>

namespace tier4_adapi_rviz_plugins
{

namespace
{
constexpr uint32_t NORMAL_4WS = 0;
constexpr uint32_t JOG_MOVE = 1;
}  // namespace

UpperTestPanel::UpperTestPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  status_label_ = new QLabel("Upper test service: idle");
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setStyleSheet("font-size: 14px; font-weight: bold; color: #333;");

  reverse_state_label_ = new QLabel("Reverse State: UNKNOWN");
  reverse_state_label_->setAlignment(Qt::AlignCenter);
  reverse_state_label_->setStyleSheet("font-size: 14px; font-weight: bold; color: #333;");

  enable_4ws_button_ = new QPushButton("Enable 4WS");
  disable_4ws_button_ = new QPushButton("Disable 4WS");
  enable_jog_button_ = new QPushButton("Enable Jog");
  disable_jog_button_ = new QPushButton("Disable Jog");
  set_forward_button_ = new QPushButton("Set Forward");
  set_reverse_button_ = new QPushButton("Set Reverse");

  jog_distance_input_ = new QDoubleSpinBox;
  jog_distance_input_->setDecimals(3);
  jog_distance_input_->setRange(0.0, 1000.0);
  jog_distance_input_->setSingleStep(0.1);
  jog_distance_input_->setSuffix(" m");

  auto * four_ws_layout = new QHBoxLayout;
  four_ws_layout->addWidget(enable_4ws_button_);
  four_ws_layout->addWidget(disable_4ws_button_);

  auto * four_ws_group = new QGroupBox("4WS");
  four_ws_group->setLayout(four_ws_layout);

  auto * jog_layout = new QGridLayout;
  jog_layout->addWidget(new QLabel("Distance"), 0, 0);
  jog_layout->addWidget(jog_distance_input_, 0, 1, 1, 2);
  jog_layout->addWidget(enable_jog_button_, 1, 1);
  jog_layout->addWidget(disable_jog_button_, 1, 2);

  auto * jog_group = new QGroupBox("Jog Move");
  jog_group->setLayout(jog_layout);

  auto * reverse_layout = new QVBoxLayout;
  auto * reverse_button_layout = new QHBoxLayout;
  reverse_button_layout->addWidget(set_forward_button_);
  reverse_button_layout->addWidget(set_reverse_button_);
  reverse_layout->addWidget(reverse_state_label_);
  reverse_layout->addLayout(reverse_button_layout);

  auto * reverse_group = new QGroupBox("Planning Reverse State");
  reverse_group->setLayout(reverse_layout);

  connect(enable_4ws_button_, SIGNAL(clicked()), this, SLOT(onEnable4ws()));
  connect(disable_4ws_button_, SIGNAL(clicked()), this, SLOT(onDisable4ws()));
  connect(enable_jog_button_, SIGNAL(clicked()), this, SLOT(onEnableJog()));
  connect(disable_jog_button_, SIGNAL(clicked()), this, SLOT(onDisableJog()));
  connect(set_forward_button_, SIGNAL(clicked()), this, SLOT(onSetForward()));
  connect(set_reverse_button_, SIGNAL(clicked()), this, SLOT(onSetReverse()));

  auto * layout = new QVBoxLayout;
  layout->addWidget(status_label_);
  layout->addWidget(four_ws_group);
  layout->addWidget(jog_group);
  layout->addWidget(reverse_group);
  layout->addStretch();
  setLayout(layout);
}

void UpperTestPanel::onInitialize()
{
  raw_node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  upper_test_client_ = raw_node_->create_client<UpperTest>("/srv_upper_test");
  reverse_state_client_ =
    raw_node_->create_client<SetVehicleReverseState>("/set_reverse_vehicle_state_service");

  reverse_state_sub_ = raw_node_->create_subscription<VehicleReversed>(
    "/planning/reverse_state", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&UpperTestPanel::onReverseState, this, std::placeholders::_1));

  RCLCPP_INFO(raw_node_->get_logger(), "UpperTestPanel initialized");
}

void UpperTestPanel::onEnable4ws()
{
  sendRequest(NORMAL_4WS, true, 0.0, "4WS enabled");
}

void UpperTestPanel::onDisable4ws()
{
  sendRequest(NORMAL_4WS, false, 0.0, "4WS disabled");
}

void UpperTestPanel::onEnableJog()
{
  sendRequest(JOG_MOVE, true, jog_distance_input_->value(), "Jog enabled");
}

void UpperTestPanel::onDisableJog()
{
  sendRequest(JOG_MOVE, false, jog_distance_input_->value(), "Jog disabled");
}

void UpperTestPanel::onSetForward()
{
  sendReverseStateRequest(false);
}

void UpperTestPanel::onSetReverse()
{
  sendReverseStateRequest(true);
}

void UpperTestPanel::sendRequest(
  uint32_t index, bool enable, double float_cmd, const QString & command_name)
{
  if (!upper_test_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(raw_node_->get_logger(), "Upper test service not available");
    setStatus("Service unavailable", "#f44336");
    return;
  }

  auto request = std::make_shared<UpperTest::Request>();
  request->index = index;
  request->is_new = false;
  request->enable = enable;
  request->bool_cmd = false;
  request->int_cmd = 0;
  request->float_cmd = float_cmd;

  setStatus(QString("%1: pending").arg(command_name), "#333");

  upper_test_client_->async_send_request(
    request,
    [this, command_name, float_cmd](rclcpp::Client<UpperTest>::SharedFuture response) {
      try {
        const auto result = response.get();
        const auto result_text = QString("%1, result: %2").arg(command_name).arg(result->result);
        const auto text = command_name.startsWith("Jog enabled")
          ? QString("%1 (%2 m), result: %3")
              .arg(command_name)
              .arg(float_cmd, 0, 'f', 3)
              .arg(result->result)
          : result_text;
        setStatus(text, "#4CAF50");
        RCLCPP_INFO(
          raw_node_->get_logger(), "Upper test command completed: %s, result=%d",
          command_name.toStdString().c_str(), result->result);
      } catch (const std::exception & e) {
        setStatus("Service call failed", "#f44336");
        RCLCPP_ERROR(raw_node_->get_logger(), "Upper test service call failed: %s", e.what());
      }
    });
}

void UpperTestPanel::sendReverseStateRequest(bool is_reversed)
{
  if (!reverse_state_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(raw_node_->get_logger(), "Reverse state service not available");
    setReverseStatus("Reverse service unavailable", "#f44336");
    return;
  }

  auto request = std::make_shared<SetVehicleReverseState::Request>();
  request->is_reversed = is_reversed;

  setReverseStatus(
    QString("Reverse State: setting %1").arg(is_reversed ? "REVERSE" : "FORWARD"), "#333");

  reverse_state_client_->async_send_request(
    request,
    [this, is_reversed](rclcpp::Client<SetVehicleReverseState>::SharedFuture response) {
      try {
        const auto result = response.get();
        if (result->success) {
          current_reverse_state_ = is_reversed;
          updateReverseStateDisplay(is_reversed);
          RCLCPP_INFO(
            raw_node_->get_logger(), "Reverse state set to: %s",
            is_reversed ? "REVERSE" : "FORWARD");
        } else {
          setReverseStatus("Reverse state request failed", "#f44336");
          RCLCPP_WARN(raw_node_->get_logger(), "Failed to set reverse state");
        }
      } catch (const std::exception & e) {
        setReverseStatus("Reverse state service call failed", "#f44336");
        RCLCPP_ERROR(raw_node_->get_logger(), "Reverse state service call failed: %s", e.what());
      }
    });
}

void UpperTestPanel::onReverseState(const VehicleReversed::ConstSharedPtr msg)
{
  current_reverse_state_ = msg->is_reversed;
  updateReverseStateDisplay(msg->is_reversed);
}

void UpperTestPanel::updateReverseStateDisplay(bool is_reversed)
{
  if (is_reversed) {
    set_reverse_button_->setEnabled(false);
    set_forward_button_->setEnabled(true);
    setReverseStatus("Reverse State: REVERSE", "#f44336");
  } else {
    set_forward_button_->setEnabled(false);
    set_reverse_button_->setEnabled(true);
    setReverseStatus("Reverse State: FORWARD", "#4CAF50");
  }
}

void UpperTestPanel::setStatus(const QString & text, const QString & color)
{
  status_label_->setText(text);
  status_label_->setStyleSheet(
    QString("font-size: 14px; font-weight: bold; color: %1;").arg(color));
}

void UpperTestPanel::setReverseStatus(const QString & text, const QString & color)
{
  reverse_state_label_->setText(text);
  reverse_state_label_->setStyleSheet(
    QString("font-size: 14px; font-weight: bold; color: %1;").arg(color));
}

}  // namespace tier4_adapi_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tier4_adapi_rviz_plugins::UpperTestPanel, rviz_common::Panel)
