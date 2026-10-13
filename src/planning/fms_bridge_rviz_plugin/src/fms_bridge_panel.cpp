// Copyright 2024 ZPMC. All rights reserved.
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

#include "fms_bridge_panel.hpp"

#include <QEvent>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMainWindow>
#include <QScrollArea>
#include <QString>
#include <QTimer>
#include <QToolBar>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <string>
#include <vector>

#undef signals

namespace fms_bridge_rviz_plugin
{

FmsBridgePanel::FmsBridgePanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * content_widget = new QWidget;
  auto * main_layout = new QVBoxLayout;

  // === Publish-via toggle ===
  // Lets the operator choose between the legacy direct-to-converter path
  // (publishes FMSRouteWindow / FMSStopPoint) and the v1.2 path through
  // vehicle_agent (publishes IrMission / IrCommand on the injection topics).
  auto * via_layout = new QHBoxLayout;
  via_layout->addWidget(new QLabel("Publish via:"));
  publish_via_combo_ = new QComboBox();
  publish_via_combo_->addItem("fms_path_converter (legacy direct)", QString("converter"));
  publish_via_combo_->addItem("vehicle_agent (via ros_injector)", QString("vehicle_agent"));
  publish_via_combo_->setToolTip(
    "converter: publish directly to /fms/route_window — bypasses VA, fast feedback for debugging the converter.\n"
    "vehicle_agent: publish IrMission to /vehicle_agent/inject_mission — exercises the full VA→bridge→converter path.");
  via_layout->addWidget(publish_via_combo_, 1);
  main_layout->addLayout(via_layout);

  // === Route section ===
  auto * route_group = new QGroupBox("Route");
  auto * route_layout = new QVBoxLayout;

  auto * route_ids_layout1 = new QHBoxLayout;
  route_ids_layout1->addWidget(new QLabel("Vehicle ID:"));
  vehicle_id_input_ = new QLineEdit("v001");
  route_ids_layout1->addWidget(vehicle_id_input_);
  route_ids_layout1->addWidget(new QLabel("Route ID:"));
  route_id_input_ = new QLineEdit("test_001");
  route_ids_layout1->addWidget(route_id_input_);
  route_layout->addLayout(route_ids_layout1);

  auto * route_ids_layout2 = new QHBoxLayout;
  route_ids_layout2->addWidget(new QLabel("Job ID:"));
  job_id_input_ = new QLineEdit("job_01");
  route_ids_layout2->addWidget(job_id_input_);
  route_ids_layout2->addWidget(new QLabel("Task ID:"));
  task_id_input_ = new QLineEdit("task_01");
  route_ids_layout2->addWidget(task_id_input_);
  route_layout->addLayout(route_ids_layout2);

  auto * orient_layout = new QHBoxLayout;
  orient_layout->addWidget(new QLabel("Orientation:"));
  orientation_combo_ = new QComboBox();
  orientation_combo_->addItem("Forward", 0);
  orientation_combo_->addItem("Reverse", 1);
  orient_layout->addWidget(orientation_combo_);
  orient_layout->addStretch();
  route_layout->addLayout(orient_layout);

  // Waypoint table. Column 6 holds the type enum (carry-over from earlier layout).
  // Column 7 holds the per-row driving direction: "Fwd" or "Rev".
  waypoint_table_ = new QTableWidget(0, 8);
  waypoint_table_->setHorizontalHeaderLabels(
    {"X", "Y", "Z", "Heading", "Type", "Speed", "", "Dir"});
  waypoint_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  waypoint_table_->verticalHeader()->hide();
  waypoint_table_->setMinimumHeight(120);
  route_layout->addWidget(waypoint_table_);

  // Add point inputs
  auto * add_pt_layout1 = new QHBoxLayout;
  add_pt_layout1->addWidget(new QLabel("X:"));
  x_input_ = new QDoubleSpinBox();
  x_input_->setRange(-99999.0, 99999.0);
  x_input_->setDecimals(2);
  add_pt_layout1->addWidget(x_input_);
  add_pt_layout1->addWidget(new QLabel("Y:"));
  y_input_ = new QDoubleSpinBox();
  y_input_->setRange(-99999.0, 99999.0);
  y_input_->setDecimals(2);
  add_pt_layout1->addWidget(y_input_);
  add_pt_layout1->addWidget(new QLabel("Z:"));
  z_input_ = new QDoubleSpinBox();
  z_input_->setRange(-99999.0, 99999.0);
  z_input_->setDecimals(2);
  z_input_->setValue(0.0);
  add_pt_layout1->addWidget(z_input_);
  add_pt_layout1->addWidget(new QLabel("Heading:"));
  heading_input_ = new QDoubleSpinBox();
  heading_input_->setRange(-6.2832, 6.2832);
  heading_input_->setDecimals(4);
  heading_input_->setSingleStep(0.1);
  add_pt_layout1->addWidget(heading_input_);
  pick_from_map_btn_ = new QPushButton("Pick From Map");
  pick_from_map_btn_->setToolTip(
    "Activate the 'FMS Pick Point' tool and click on the map to fill X, Y, and Heading.\n"
    "Works like RViz's '2D Goal Pose' button.");
  pick_from_map_btn_->setStyleSheet(
    "QPushButton:hover { background-color: #009688; color: white; }");
  add_pt_layout1->addWidget(pick_from_map_btn_);
  route_layout->addLayout(add_pt_layout1);

  auto * add_pt_layout2 = new QHBoxLayout;
  add_pt_layout2->addWidget(new QLabel("Type:"));
  type_combo_ = new QComboBox();
  type_combo_->addItem("NORMAL", SegmentWaypoint::POINT_NORMAL);
  type_combo_->addItem("DESTINATION", SegmentWaypoint::POINT_DESTINATION);
  type_combo_->addItem("LC_START", SegmentWaypoint::POINT_LC_START);
  type_combo_->addItem("LC_END", SegmentWaypoint::POINT_LC_END);
  type_combo_->addItem("POI", SegmentWaypoint::POINT_POI);
  type_combo_->addItem("LOCAL_PLAN_START", SegmentWaypoint::POINT_LOCAL_PLAN_START);
  type_combo_->addItem("LOCAL_PLAN_END", SegmentWaypoint::POINT_LOCAL_PLAN_END);
  type_combo_->addItem("DRAWING", SegmentWaypoint::POINT_DRAWING);
  add_pt_layout2->addWidget(type_combo_);
  add_pt_layout2->addWidget(new QLabel("Speed:"));
  speed_input_ = new QDoubleSpinBox();
  speed_input_->setRange(0.0, 50.0);
  speed_input_->setDecimals(1);
  speed_input_->setSingleStep(0.5);
  speed_input_->setValue(5.0);
  speed_input_->setSuffix(" m/s");
  add_pt_layout2->addWidget(speed_input_);
  route_layout->addLayout(add_pt_layout2);

  // Buttons: Add / Remove / Clear
  auto * pt_btn_layout = new QHBoxLayout;
  auto * add_btn = new QPushButton("Add Point");
  auto * remove_btn = new QPushButton("Remove Last");
  auto * clear_btn = new QPushButton("Clear All");
  pt_btn_layout->addWidget(add_btn);
  pt_btn_layout->addWidget(remove_btn);
  pt_btn_layout->addWidget(clear_btn);
  route_layout->addLayout(pt_btn_layout);

  // Buttons: Save / Load
  auto * file_btn_layout = new QHBoxLayout;
  auto * save_btn = new QPushButton("Save Points...");
  save_btn->setStyleSheet(
    "QPushButton:hover { background-color: #FF9800; color: white; }");
  auto * load_btn = new QPushButton("Load Points...");
  load_btn->setStyleSheet(
    "QPushButton:hover { background-color: #9C27B0; color: white; }");
  file_btn_layout->addWidget(save_btn);
  file_btn_layout->addWidget(load_btn);
  route_layout->addLayout(file_btn_layout);

  // Send Route
  auto * send_route_btn = new QPushButton(">>> Send Route <<<");
  send_route_btn->setStyleSheet(
    "QPushButton { font-weight: bold; padding: 6px; }"
    "QPushButton:hover { background-color: #4CAF50; color: white; }");
  route_layout->addWidget(send_route_btn);

  route_group->setLayout(route_layout);
  main_layout->addWidget(route_group);

  // === Sliding Window section ===
  auto * window_group = new QGroupBox("Sliding Window");
  auto * window_layout = new QVBoxLayout;

  auto * window_row1 = new QHBoxLayout;
  window_enable_check_ = new QCheckBox("Enable sliding window");
  window_enable_check_->setChecked(true);
  window_enable_check_->setToolTip(
    "When enabled, Send Route stores the full route and publishes\n"
    "a sliding window around the ego position at 1 Hz.");
  window_row1->addWidget(window_enable_check_);
  window_row1->addStretch();
  window_row1->addWidget(new QLabel("Ahead:"));
  window_size_input_ = new QDoubleSpinBox();
  window_size_input_->setRange(10.0, 2000.0);
  window_size_input_->setDecimals(0);
  window_size_input_->setSingleStep(50.0);
  window_size_input_->setValue(100.0);
  window_size_input_->setSuffix(" m");
  window_size_input_->setToolTip("Window size ahead of ego (meters)");
  window_row1->addWidget(window_size_input_);
  window_row1->addWidget(new QLabel("Behind:"));
  window_lookback_input_ = new QDoubleSpinBox();
  window_lookback_input_->setRange(0.0, 100.0);
  window_lookback_input_->setDecimals(0);
  window_lookback_input_->setSingleStep(5.0);
  window_lookback_input_->setValue(10.0);
  window_lookback_input_->setSuffix(" m");
  window_lookback_input_->setToolTip("Window size behind ego (meters)");
  window_row1->addWidget(window_lookback_input_);
  window_layout->addLayout(window_row1);

  auto * window_row2 = new QHBoxLayout;
  stop_window_btn_ = new QPushButton("Stop Window");
  stop_window_btn_->setStyleSheet(
    "QPushButton:hover { background-color: #F44336; color: white; }");
  stop_window_btn_->setEnabled(false);
  window_row2->addWidget(stop_window_btn_);
  window_status_label_ = new QLabel("Window: inactive");
  window_status_label_->setStyleSheet("color: gray;");
  window_row2->addWidget(window_status_label_);
  window_row2->addStretch();
  window_layout->addLayout(window_row2);

  window_group->setLayout(window_layout);
  main_layout->addWidget(window_group);

  // === Stop / Resume section ===
  auto * stop_group = new QGroupBox("Stop / Resume");
  auto * stop_layout = new QHBoxLayout;
  stop_layout->addWidget(new QLabel("Stop X:"));
  stop_x_input_ = new QDoubleSpinBox();
  stop_x_input_->setRange(-99999.0, 99999.0);
  stop_x_input_->setDecimals(2);
  stop_layout->addWidget(stop_x_input_);
  stop_layout->addWidget(new QLabel("Stop Y:"));
  stop_y_input_ = new QDoubleSpinBox();
  stop_y_input_->setRange(-99999.0, 99999.0);
  stop_y_input_->setDecimals(2);
  stop_layout->addWidget(stop_y_input_);
  auto * stop_btn = new QPushButton("Send Stop");
  stop_btn->setStyleSheet("QPushButton:hover { background-color: #FF5722; color: white; }");
  stop_layout->addWidget(stop_btn);
  auto * resume_btn = new QPushButton("Send Resume");
  resume_btn->setStyleSheet("QPushButton:hover { background-color: #2196F3; color: white; }");
  stop_layout->addWidget(resume_btn);
  stop_group->setLayout(stop_layout);
  main_layout->addWidget(stop_group);

  // === Status section ===
  auto * status_group = new QGroupBox("Status (live from /fms_bridge/adapter_status)");
  auto * status_layout = new QVBoxLayout;
  status_label_ = new QLabel("Waiting for status...");
  status_label_->setWordWrap(true);
  status_layout->addWidget(status_label_);
  status_group->setLayout(status_layout);
  main_layout->addWidget(status_group);

  content_widget->setLayout(main_layout);

  auto * scroll_area = new QScrollArea;
  scroll_area->setWidget(content_widget);
  scroll_area->setWidgetResizable(true);
  scroll_area->setFrameShape(QFrame::NoFrame);

  auto * outer_layout = new QVBoxLayout;
  outer_layout->setContentsMargins(0, 0, 0, 0);
  outer_layout->addWidget(scroll_area);
  setLayout(outer_layout);

  // Connect signals
  connect(add_btn, SIGNAL(clicked()), this, SLOT(onAddPoint()));
  connect(remove_btn, SIGNAL(clicked()), this, SLOT(onRemoveLastPoint()));
  connect(clear_btn, SIGNAL(clicked()), this, SLOT(onClearPoints()));
  connect(send_route_btn, SIGNAL(clicked()), this, SLOT(onSendRoute()));
  connect(stop_btn, SIGNAL(clicked()), this, SLOT(onSendStop()));
  connect(resume_btn, SIGNAL(clicked()), this, SLOT(onSendResume()));
  connect(save_btn, SIGNAL(clicked()), this, SLOT(onSavePoints()));
  connect(load_btn, SIGNAL(clicked()), this, SLOT(onLoadPoints()));
  connect(stop_window_btn_, SIGNAL(clicked()), this, SLOT(onStopWindow()));
  connect(pick_from_map_btn_, SIGNAL(clicked()), this, SLOT(onPickFromMap()));

  // Sliding window timer (1 Hz, started/stopped on demand)
  window_timer_ = new QTimer(this);
  window_timer_->setInterval(1000);
  connect(window_timer_, &QTimer::timeout, this, &FmsBridgePanel::onWindowTimer);
}

void FmsBridgePanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  pub_route_ = raw_node_->create_publisher<FMSRouteWindow>(
    "/fms/route_window", rclcpp::QoS(10));
  pub_stop_ = raw_node_->create_publisher<FMSStopPoint>(
    "/fms/stop_point", rclcpp::QoS(10));
  pub_ir_mission_ = raw_node_->create_publisher<IrMission>(
    "/vehicle_agent/inject_mission", rclcpp::QoS(10));
  pub_ir_command_ = raw_node_->create_publisher<IrCommand>(
    "/vehicle_agent/inject_command", rclcpp::QoS(10));

  pub_markers_ = raw_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/fms/waypoint_markers", rclcpp::QoS(1).transient_local());

  sub_status_ = raw_node_->create_subscription<AdapterStatus>(
    "/fms_bridge/adapter_status", rclcpp::QoS(10),
    std::bind(&FmsBridgePanel::onStatus, this, std::placeholders::_1));

  sub_odom_ = raw_node_->create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", rclcpp::SensorDataQoS(),
    std::bind(&FmsBridgePanel::onOdometry, this, std::placeholders::_1));

  sub_picked_pose_ = raw_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/rviz/fms_bridge/point_pose", rclcpp::QoS(1),
    std::bind(&FmsBridgePanel::onPickedPose, this, std::placeholders::_1));

  // Republish markers periodically (also covers table cell edits)
  marker_timer_ = new QTimer(this);
  connect(marker_timer_, &QTimer::timeout, this, [this]() { publishWaypointMarkers(); });
  marker_timer_->start(1000);

  // Refresh markers whenever the user edits a cell directly
  connect(waypoint_table_, &QTableWidget::itemChanged, this,
    [this](QTableWidgetItem *) { publishWaypointMarkers(); });

  // Publish an initial (empty DELETEALL) frame so RViz sees the topic
  publishWaypointMarkers();

  // Prevent toolbar auto-collapse: find QMainWindow and install event filter on toolbars
  QTimer::singleShot(0, this, [this]() {
    QWidget * top = window();
    if (!top) return;
    auto * main_win = qobject_cast<QMainWindow *>(top);
    if (!main_win) return;
    for (auto * tb : main_win->findChildren<QToolBar *>()) {
      tb->installEventFilter(this);
      connect(tb->toggleViewAction(), &QAction::triggered, this, [this]() {
        toolbar_manual_toggle_ = true;
      });
    }
  });
}

bool FmsBridgePanel::eventFilter(QObject * obj, QEvent * event)
{
  if (event->type() == QEvent::Hide) {
    auto * toolbar = qobject_cast<QToolBar *>(obj);
    if (toolbar) {
      if (toolbar_manual_toggle_) {
        // User toggled via Panels menu — allow hide/show
        toolbar_manual_toggle_ = false;
      } else {
        // Auto-collapse (mouse leave) — prevent it by re-showing
        QTimer::singleShot(0, toolbar, &QWidget::show);
      }
    }
  }
  return rviz_common::Panel::eventFilter(obj, event);
}

void FmsBridgePanel::onAddPoint()
{
  const int row = waypoint_table_->rowCount();
  waypoint_table_->insertRow(row);

  waypoint_table_->setItem(row, 0, new QTableWidgetItem(QString::number(x_input_->value(), 'f', 2)));
  waypoint_table_->setItem(row, 1, new QTableWidgetItem(QString::number(y_input_->value(), 'f', 2)));
  waypoint_table_->setItem(row, 2, new QTableWidgetItem(QString::number(z_input_->value(), 'f', 2)));
  waypoint_table_->setItem(
    row, 3, new QTableWidgetItem(QString::number(heading_input_->value(), 'f', 4)));
  waypoint_table_->setItem(row, 4, new QTableWidgetItem(type_combo_->currentText()));
  waypoint_table_->setItem(
    row, 5, new QTableWidgetItem(QString::number(speed_input_->value(), 'f', 1)));
  // Store the type enum value in column 6 (hidden data)
  auto * type_item = new QTableWidgetItem(QString::number(type_combo_->currentData().toInt()));
  waypoint_table_->setItem(row, 6, type_item);
  // Per-row direction defaults to the top orientation combo's current selection.
  const bool default_reverse =
    orientation_combo_->currentData().toInt() == FMSPathSegment::ORIENT_REVERSE;
  waypoint_table_->setItem(
    row, 7, new QTableWidgetItem(default_reverse ? "Rev" : "Fwd"));

  publishWaypointMarkers();
}

void FmsBridgePanel::onRemoveLastPoint()
{
  const int row = waypoint_table_->rowCount();
  if (row > 0) {
    waypoint_table_->removeRow(row - 1);
  }
  publishWaypointMarkers();
}

void FmsBridgePanel::onClearPoints()
{
  waypoint_table_->setRowCount(0);
  publishWaypointMarkers();
}

void FmsBridgePanel::onSendRoute()
{
  if (waypoint_table_->rowCount() == 0) {
    QMessageBox::warning(this, "Send Route", "No waypoints to send.");
    return;
  }

  if (isViaVehicleAgent()) {
    // Route through vehicle_agent. We publish IrMission once and stop — the
    // downstream va_route_bridge latches the mission and slides an ego-centric
    // window over it at ~1 Hz, so downstream liveness is decoupled from this
    // panel's send cadence. Same-id re-sends are treated as data-plane
    // refreshes (no FSM preemption) so it is safe to click Send Route again
    // after editing waypoints.
    auto msg = buildIrMission();
    pub_ir_mission_->publish(msg);
    RCLCPP_INFO(
      raw_node_->get_logger(),
      "Published IrMission via vehicle_agent: id=%s, seq=%lu, %zu waypoints",
      msg.id.c_str(), static_cast<unsigned long>(msg.seq), msg.waypoints.size());
    publishWaypointMarkers();
    return;
  }

  if (window_enable_check_->isChecked()) {
    // Sliding window mode: store full route, start timer
    master_waypoints_.clear();
    master_directions_.clear();
    master_orientation_ = static_cast<uint8_t>(orientation_combo_->currentData().toInt());

    for (int row = 0; row < waypoint_table_->rowCount(); ++row) {
      SegmentWaypoint pt;
      pt.x = waypoint_table_->item(row, 0)->text().toDouble();
      pt.y = waypoint_table_->item(row, 1)->text().toDouble();
      pt.z = waypoint_table_->item(row, 2)->text().toDouble();
      pt.heading = waypoint_table_->item(row, 3)->text().toDouble();
      pt.point_type = static_cast<uint8_t>(waypoint_table_->item(row, 6)->text().toInt());
      pt.speed_limit = waypoint_table_->item(row, 5)->text().toDouble();
      master_waypoints_.push_back(pt);

      auto * dir_item = waypoint_table_->item(row, 7);
      const uint8_t row_orient = (dir_item &&
        dir_item->text().compare("Rev", Qt::CaseInsensitive) == 0)
        ? FMSPathSegment::ORIENT_REVERSE
        : (dir_item ? FMSPathSegment::ORIENT_FORWARD : master_orientation_);
      master_directions_.push_back(row_orient);
    }

    window_active_ = true;
    stop_window_btn_->setEnabled(true);
    window_timer_->start();

    // Publish the first window immediately
    publishWindow();

    RCLCPP_INFO(
      raw_node_->get_logger(),
      "Sliding window started: %zu master waypoints, window=%.0fm ahead / %.0fm behind",
      master_waypoints_.size(), window_size_input_->value(), window_lookback_input_->value());
  } else {
    // One-shot mode: publish all points at once (original behavior)
    auto msg = buildFullRoute();
    pub_route_->publish(msg);

    RCLCPP_INFO(
      raw_node_->get_logger(), "Published FMSRouteWindow (one-shot): route_id=%s, %zu points",
      msg.route_id.c_str(), msg.segments.empty() ? 0 : msg.segments[0].points.size());
  }

  publishWaypointMarkers();
}

void FmsBridgePanel::onSendStop()
{
  if (isViaVehicleAgent()) {
    IrCommand c;
    c.mission_id = route_id_input_->text().toStdString();
    c.kind = IrCommand::KIND_PAUSE;
    c.issued_at_ms = static_cast<uint64_t>(raw_node_->now().nanoseconds() / 1'000'000);
    c.source_adapter = "fms_bridge_rviz_plugin";
    pub_ir_command_->publish(c);
    RCLCPP_INFO(
      raw_node_->get_logger(), "Published IrCommand PAUSE via vehicle_agent (mission=%s)",
      c.mission_id.c_str());
    return;
  }

  FMSStopPoint msg;
  msg.header.stamp = raw_node_->now();
  msg.vehicle_id = vehicle_id_input_->text().toStdString();
  msg.x = stop_x_input_->value();
  msg.y = stop_y_input_->value();
  msg.clear = false;
  pub_stop_->publish(msg);

  RCLCPP_INFO(
    raw_node_->get_logger(), "Published FMSStopPoint: vehicle_id=%s, x=%.2f, y=%.2f",
    msg.vehicle_id.c_str(), msg.x, msg.y);
}

void FmsBridgePanel::onSendResume()
{
  if (isViaVehicleAgent()) {
    IrCommand c;
    c.mission_id = route_id_input_->text().toStdString();
    c.kind = IrCommand::KIND_RESUME;
    c.issued_at_ms = static_cast<uint64_t>(raw_node_->now().nanoseconds() / 1'000'000);
    c.source_adapter = "fms_bridge_rviz_plugin";
    pub_ir_command_->publish(c);
    RCLCPP_INFO(
      raw_node_->get_logger(), "Published IrCommand RESUME via vehicle_agent (mission=%s)",
      c.mission_id.c_str());
    return;
  }

  FMSStopPoint msg;
  msg.header.stamp = raw_node_->now();
  msg.vehicle_id = vehicle_id_input_->text().toStdString();
  msg.x = 0.0;
  msg.y = 0.0;
  msg.clear = true;
  pub_stop_->publish(msg);

  RCLCPP_INFO(
    raw_node_->get_logger(), "Published FMSStopPoint (clear): vehicle_id=%s",
    msg.vehicle_id.c_str());
}

void FmsBridgePanel::onSavePoints()
{
  if (waypoint_table_->rowCount() == 0) {
    QMessageBox::warning(this, "Save Points", "No waypoints to save.");
    return;
  }

  QString path = QFileDialog::getSaveFileName(
    this, "Save Waypoints", "", "JSON Files (*.json);;All Files (*)");
  if (path.isEmpty()) {
    return;
  }

  QJsonObject root;
  root["vehicle_id"] = vehicle_id_input_->text();
  root["route_id"] = route_id_input_->text();
  root["job_id"] = job_id_input_->text();
  root["task_id"] = task_id_input_->text();
  root["orientation"] = orientation_combo_->currentData().toInt();

  QJsonArray points;
  for (int row = 0; row < waypoint_table_->rowCount(); ++row) {
    QJsonObject pt;
    pt["x"] = waypoint_table_->item(row, 0)->text().toDouble();
    pt["y"] = waypoint_table_->item(row, 1)->text().toDouble();
    pt["z"] = waypoint_table_->item(row, 2)->text().toDouble();
    pt["heading"] = waypoint_table_->item(row, 3)->text().toDouble();
    pt["type_text"] = waypoint_table_->item(row, 4)->text();
    pt["speed"] = waypoint_table_->item(row, 5)->text().toDouble();
    pt["type_enum"] = waypoint_table_->item(row, 6)->text().toInt();
    {
      auto * dir_item = waypoint_table_->item(row, 7);
      const bool is_reverse = dir_item &&
        dir_item->text().compare("Rev", Qt::CaseInsensitive) == 0;
      pt["is_reverse"] = is_reverse;
    }
    points.append(pt);
  }
  root["points"] = points;

  QFile file(path);
  if (!file.open(QIODevice::WriteOnly)) {
    QMessageBox::critical(this, "Save Points", "Failed to open file for writing.");
    return;
  }
  file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
  file.close();

  RCLCPP_INFO(
    raw_node_->get_logger(), "Saved %d waypoints to %s",
    waypoint_table_->rowCount(), path.toStdString().c_str());
}

void FmsBridgePanel::onLoadPoints()
{
  QString path = QFileDialog::getOpenFileName(
    this, "Load Waypoints", "", "JSON Files (*.json);;All Files (*)");
  if (path.isEmpty()) {
    return;
  }

  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    QMessageBox::critical(this, "Load Points", "Failed to open file for reading.");
    return;
  }

  QJsonParseError parse_err;
  QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parse_err);
  file.close();

  if (parse_err.error != QJsonParseError::NoError) {
    QMessageBox::critical(
      this, "Load Points",
      QString("JSON parse error: %1").arg(parse_err.errorString()));
    return;
  }

  QJsonObject root = doc.object();

  if (root.contains("vehicle_id")) vehicle_id_input_->setText(root["vehicle_id"].toString());
  if (root.contains("route_id")) route_id_input_->setText(root["route_id"].toString());
  if (root.contains("job_id")) job_id_input_->setText(root["job_id"].toString());
  if (root.contains("task_id")) task_id_input_->setText(root["task_id"].toString());
  if (root.contains("orientation")) {
    int idx = orientation_combo_->findData(root["orientation"].toInt());
    if (idx >= 0) orientation_combo_->setCurrentIndex(idx);
  }

  QJsonArray points = root["points"].toArray();
  waypoint_table_->setRowCount(0);

  for (int i = 0; i < points.size(); ++i) {
    QJsonObject pt = points[i].toObject();
    const int row = waypoint_table_->rowCount();
    waypoint_table_->insertRow(row);

    waypoint_table_->setItem(
      row, 0, new QTableWidgetItem(QString::number(pt["x"].toDouble(), 'f', 2)));
    waypoint_table_->setItem(
      row, 1, new QTableWidgetItem(QString::number(pt["y"].toDouble(), 'f', 2)));
    waypoint_table_->setItem(
      row, 2, new QTableWidgetItem(QString::number(pt["z"].toDouble(), 'f', 2)));
    waypoint_table_->setItem(
      row, 3, new QTableWidgetItem(QString::number(pt["heading"].toDouble(), 'f', 4)));
    waypoint_table_->setItem(
      row, 4, new QTableWidgetItem(pt["type_text"].toString()));
    waypoint_table_->setItem(
      row, 5, new QTableWidgetItem(QString::number(pt["speed"].toDouble(), 'f', 1)));
    waypoint_table_->setItem(
      row, 6, new QTableWidgetItem(QString::number(pt["type_enum"].toInt())));
    // 兼容旧格式：若无 is_reverse 字段，则回退到顶层 orientation
    bool is_reverse = false;
    if (pt.contains("is_reverse")) {
      is_reverse = pt["is_reverse"].toBool();
    } else if (root.contains("orientation")) {
      is_reverse = (root["orientation"].toInt() == FMSPathSegment::ORIENT_REVERSE);
    }
    waypoint_table_->setItem(
      row, 7, new QTableWidgetItem(is_reverse ? "Rev" : "Fwd"));
  }

  RCLCPP_INFO(
    raw_node_->get_logger(), "Loaded %d waypoints from %s",
    waypoint_table_->rowCount(), path.toStdString().c_str());

  publishWaypointMarkers();
}

FMSRouteWindow FmsBridgePanel::buildFullRoute() const
{
  FMSRouteWindow msg;
  msg.header.stamp = raw_node_->now();
  msg.header.frame_id = "map";
  msg.vehicle_id = vehicle_id_input_->text().toStdString();
  msg.route_id = route_id_input_->text().toStdString();
  msg.job_id = job_id_input_->text().toStdString();
  msg.job_task_id = task_id_input_->text().toStdString();
  msg.status = FMSRouteWindow::STATUS_ACTIVE;

  // 默认方向（顶部下拉）作为没有 Dir 列时的回退。
  const uint8_t default_orient =
    static_cast<uint8_t>(orientation_combo_->currentData().toInt());

  // 按相邻 Dir 列分段：当方向翻转时新开 FMSPathSegment。边界点同时出现在两个段
  // 中，便于下游 fms_path_converter 在分段边界处构造朝向反转。
  FMSPathSegment segment;
  segment.orientation = default_orient;
  bool segment_orient_set = false;

  auto rowDirection = [this, default_orient](int row) -> uint8_t {
    auto * dir_item = waypoint_table_->item(row, 7);
    if (!dir_item) return default_orient;
    return dir_item->text().compare("Rev", Qt::CaseInsensitive) == 0
      ? FMSPathSegment::ORIENT_REVERSE
      : FMSPathSegment::ORIENT_FORWARD;
  };

  for (int row = 0; row < waypoint_table_->rowCount(); ++row) {
    SegmentWaypoint pt;
    pt.x = waypoint_table_->item(row, 0)->text().toDouble();
    pt.y = waypoint_table_->item(row, 1)->text().toDouble();
    pt.z = waypoint_table_->item(row, 2)->text().toDouble();
    pt.heading = waypoint_table_->item(row, 3)->text().toDouble();
    pt.point_type = static_cast<uint8_t>(waypoint_table_->item(row, 6)->text().toInt());
    pt.speed_limit = waypoint_table_->item(row, 5)->text().toDouble();

    const uint8_t row_orient = rowDirection(row);

    if (!segment_orient_set) {
      segment.orientation = row_orient;
      segment_orient_set = true;
    } else if (row_orient != segment.orientation) {
      // 方向翻转：把当前点也作为旧段的最后一个点（边界冗余），收尾后开新段。
      segment.points.push_back(pt);
      msg.segments.push_back(segment);
      segment = FMSPathSegment{};
      segment.orientation = row_orient;
    }

    segment.points.push_back(pt);
  }

  if (!segment.points.empty()) {
    msg.segments.push_back(segment);
  }
  return msg;
}

bool FmsBridgePanel::isViaVehicleAgent() const
{
  return publish_via_combo_ &&
         publish_via_combo_->currentData().toString() == QStringLiteral("vehicle_agent");
}

IrMission FmsBridgePanel::buildIrMission() const
{
  IrMission m;
  m.id = route_id_input_->text().toStdString();
  m.kind = IrMission::KIND_ROUTE;
  m.seq = ++const_cast<FmsBridgePanel *>(this)->mission_seq_;
  m.issued_at_ms = static_cast<uint64_t>(raw_node_->now().nanoseconds() / 1'000'000);
  m.deadline_ms = 0;
  m.source_adapter = "fms_bridge_rviz_plugin";

  // Encode each direction run as "segN:fwd" / "segN:rev". At a direction
  // change, emit the boundary point once for the old segment and once for the
  // new segment, matching buildFullRoute()'s shared-boundary representation.
  const uint8_t default_orient =
    static_cast<uint8_t>(orientation_combo_->currentData().toInt());
  auto rowDirection = [this, default_orient](int row) -> uint8_t {
    auto * dir_item = waypoint_table_->item(row, 7);
    if (!dir_item) return default_orient;
    return dir_item->text().compare("Rev", Qt::CaseInsensitive) == 0
      ? FMSPathSegment::ORIENT_REVERSE
      : FMSPathSegment::ORIENT_FORWARD;
  };

  int segment_index = 0;
  uint8_t current_orient = default_orient;
  bool first = true;
  for (int row = 0; row < waypoint_table_->rowCount(); ++row) {
    const uint8_t row_orient = rowDirection(row);

    IrWaypoint w;
    w.x = waypoint_table_->item(row, 0)->text().toDouble();
    w.y = waypoint_table_->item(row, 1)->text().toDouble();
    w.z = waypoint_table_->item(row, 2)->text().toDouble();
    w.yaw = waypoint_table_->item(row, 3)->text().toDouble();
    w.target_speed = waypoint_table_->item(row, 5)->text().toDouble();
    w.point_type = static_cast<uint8_t>(waypoint_table_->item(row, 6)->text().toInt());

    if (first) {
      current_orient = row_orient;
      first = false;
    } else if (row_orient != current_orient) {
      auto old_segment_boundary = w;
      old_segment_boundary.yaw = m.waypoints.back().yaw;
      old_segment_boundary.segment_id =
        std::string("seg") + std::to_string(segment_index) +
        (current_orient == FMSPathSegment::ORIENT_REVERSE ? ":rev" : ":fwd");
      m.waypoints.push_back(old_segment_boundary);

      ++segment_index;
      current_orient = row_orient;
    }

    w.segment_id = std::string("seg") + std::to_string(segment_index) +
                   (current_orient == FMSPathSegment::ORIENT_REVERSE ? ":rev" : ":fwd");
    m.waypoints.push_back(w);
  }
  return m;
}

void FmsBridgePanel::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  ego_x_ = msg->pose.pose.position.x;
  ego_y_ = msg->pose.pose.position.y;
  ego_received_ = true;
}

void FmsBridgePanel::onWindowTimer()
{
  if (!window_active_) {
    return;
  }
  publishWindow();
}

void FmsBridgePanel::publishWindow()
{
  if (master_waypoints_.empty()) {
    return;
  }

  const double window_ahead = window_size_input_->value();
  const double window_behind = window_lookback_input_->value();

  // Find the nearest waypoint to ego
  size_t nearest_idx = 0;
  if (ego_received_) {
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < master_waypoints_.size(); ++i) {
      const double dx = master_waypoints_[i].x - ego_x_;
      const double dy = master_waypoints_[i].y - ego_y_;
      const double d = dx * dx + dy * dy;
      if (d < min_dist) {
        min_dist = d;
        nearest_idx = i;
      }
    }
  }

  // Compute cumulative arc-length distances from the nearest point
  // Walk backward to find window start
  size_t win_start = nearest_idx;
  {
    double acc = 0.0;
    for (size_t i = nearest_idx; i > 0; --i) {
      const double dx = master_waypoints_[i].x - master_waypoints_[i - 1].x;
      const double dy = master_waypoints_[i].y - master_waypoints_[i - 1].y;
      acc += std::sqrt(dx * dx + dy * dy);
      if (acc > window_behind) {
        break;
      }
      win_start = i - 1;
    }
  }

  // Walk forward to find window end
  size_t win_end = nearest_idx;
  {
    double acc = 0.0;
    for (size_t i = nearest_idx; i + 1 < master_waypoints_.size(); ++i) {
      const double dx = master_waypoints_[i + 1].x - master_waypoints_[i].x;
      const double dy = master_waypoints_[i + 1].y - master_waypoints_[i].y;
      acc += std::sqrt(dx * dx + dy * dy);
      win_end = i + 1;
      if (acc > window_ahead) {
        break;
      }
    }
  }

  // Preserve LC pair integrity: if window cuts between LC_START and LC_END, extend
  // Check if we start inside an LC zone (there's an LC_START before win_start with no LC_END)
  bool inside_lc = false;
  for (size_t i = 0; i < win_start; ++i) {
    if (master_waypoints_[i].point_type == SegmentWaypoint::POINT_LC_START) {
      inside_lc = true;
    } else if (master_waypoints_[i].point_type == SegmentWaypoint::POINT_LC_END) {
      inside_lc = false;
    }
  }
  if (inside_lc) {
    // Extend win_start backward to include the LC_START
    for (size_t i = win_start; i > 0; --i) {
      if (master_waypoints_[i - 1].point_type == SegmentWaypoint::POINT_LC_START) {
        win_start = i - 1;
        break;
      }
    }
  }

  // Check if win_end cuts an LC zone (LC_START without matching LC_END in window)
  inside_lc = false;
  for (size_t i = win_start; i <= win_end; ++i) {
    if (master_waypoints_[i].point_type == SegmentWaypoint::POINT_LC_START) {
      inside_lc = true;
    } else if (master_waypoints_[i].point_type == SegmentWaypoint::POINT_LC_END) {
      inside_lc = false;
    }
  }
  if (inside_lc) {
    // Extend win_end forward to include the LC_END
    for (size_t i = win_end + 1; i < master_waypoints_.size(); ++i) {
      win_end = i;
      if (master_waypoints_[i].point_type == SegmentWaypoint::POINT_LC_END) {
        break;
      }
    }
  }

  // Ensure at least one preceding waypoint so the path generator has a segment to work with.
  // Without this, when nearest_idx is the DESTINATION, the window collapses to a single
  // zero-velocity point and the vehicle stops early.
  if (win_start == win_end && win_start > 0) {
    win_start = win_start - 1;
  }

  // Build windowed message
  FMSRouteWindow msg;
  msg.header.stamp = raw_node_->now();
  msg.header.frame_id = "map";
  msg.vehicle_id = vehicle_id_input_->text().toStdString();
  msg.route_id = route_id_input_->text().toStdString();
  msg.job_id = job_id_input_->text().toStdString();
  msg.job_task_id = task_id_input_->text().toStdString();
  msg.status = FMSRouteWindow::STATUS_ACTIVE;

  // 按方向切分窗口内的航点为多个 FMSPathSegment。当 master_directions_ 为空（旧路径）时
  // 整窗使用 master_orientation_。
  FMSPathSegment segment;
  segment.orientation =
    (master_directions_.empty() ? master_orientation_ : master_directions_[win_start]);
  for (size_t i = win_start; i <= win_end; ++i) {
    const uint8_t row_orient =
      master_directions_.empty() ? master_orientation_ : master_directions_[i];
    if (row_orient != segment.orientation) {
      segment.points.push_back(master_waypoints_[i]);    // 边界点冗余进旧段
      msg.segments.push_back(segment);
      segment = FMSPathSegment{};
      segment.orientation = row_orient;
    }
    segment.points.push_back(master_waypoints_[i]);
  }
  if (!segment.points.empty()) {
    msg.segments.push_back(segment);
  }
  pub_route_->publish(msg);

  // Update UI status label
  QString status_text;
  if (ego_received_) {
    status_text = QString("Window: pts [%1-%2] of %3  |  nearest=%4  |  ego=(%5, %6)")
      .arg(win_start)
      .arg(win_end)
      .arg(master_waypoints_.size())
      .arg(nearest_idx)
      .arg(ego_x_, 0, 'f', 1)
      .arg(ego_y_, 0, 'f', 1);
  } else {
    status_text = QString("Window: pts [%1-%2] of %3  |  no odom yet")
      .arg(win_start)
      .arg(win_end)
      .arg(master_waypoints_.size());
  }
  window_status_label_->setText(status_text);
  window_status_label_->setStyleSheet("color: green; font-weight: bold;");
}

void FmsBridgePanel::onStopWindow()
{
  window_active_ = false;
  window_timer_->stop();
  master_waypoints_.clear();
  master_directions_.clear();
  stop_window_btn_->setEnabled(false);
  window_status_label_->setText("Window: inactive");
  window_status_label_->setStyleSheet("color: gray;");

  // Publish STATUS_CANCELLED so the bridge knows to stop
  FMSRouteWindow msg;
  msg.header.stamp = raw_node_->now();
  msg.header.frame_id = "map";
  msg.vehicle_id = vehicle_id_input_->text().toStdString();
  msg.route_id = route_id_input_->text().toStdString();
  msg.status = FMSRouteWindow::STATUS_CANCELLED;
  pub_route_->publish(msg);

  RCLCPP_INFO(raw_node_->get_logger(), "Sliding window stopped.");
}

void FmsBridgePanel::onPickFromMap()
{
  // Find the QAction for the "FMS Pick Point" tool in the RViz toolbars and trigger it.
  // This mirrors clicking the tool's button in the toolbar.
  QWidget * top = window();
  auto * main_win = top ? qobject_cast<QMainWindow *>(top) : nullptr;
  if (!main_win) {
    QMessageBox::warning(
      this, "Pick From Map",
      "Could not find main window. Activate the 'FMS Pick Point' tool manually.");
    return;
  }

  for (auto * tb : main_win->findChildren<QToolBar *>()) {
    for (auto * action : tb->actions()) {
      if (action->text().contains("FMS Pick Point", Qt::CaseInsensitive)) {
        action->trigger();
        return;
      }
    }
  }

  QMessageBox::information(
    this, "Pick From Map",
    "The 'FMS Pick Point' tool is not in the toolbar.\n"
    "Add it via the '+' button in RViz's tool toolbar, then use this button or press 'p'.");
}

void FmsBridgePanel::onPickedPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  const auto & q = msg->pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  // SegmentWaypoint.heading is travel direction. The picked arrow represents the vehicle's
  // physical facing direction (most intuitive for the user). For reverse rows, travel
  // direction = facing direction + π.
  const bool default_reverse =
    orientation_combo_->currentData().toInt() == FMSPathSegment::ORIENT_REVERSE;
  if (default_reverse) {
    yaw += M_PI;
    // Normalize to [-π, π]
    while (yaw > M_PI)  yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
  }

  const double x = msg->pose.position.x;
  const double y = msg->pose.position.y;

  // Thread-safe UI update
  QMetaObject::invokeMethod(this, [this, x, y, yaw]() {
    x_input_->setValue(x);
    y_input_->setValue(y);
    heading_input_->setValue(yaw);
  }, Qt::QueuedConnection);
}

void FmsBridgePanel::onStatus(const AdapterStatus::ConstSharedPtr msg)
{
  // State enum to string
  std::string state_str;
  switch (msg->state) {
    case AdapterStatus::STATE_IDLE:
      state_str = "IDLE";
      break;
    case AdapterStatus::STATE_ACTIVE:
      state_str = "ACTIVE";
      break;
    case AdapterStatus::STATE_SUSPENDED:
      state_str = "SUSPENDED";
      break;
    case AdapterStatus::STATE_ERROR:
      state_str = "ERROR";
      break;
    default:
      state_str = "UNKNOWN(" + std::to_string(msg->state) + ")";
      break;
  }

  QString text = QString(
    "State: %1\n"
    "Route: %2   Job: %3\n"
    "Stop: %4   Progress: %5")
    .arg(QString::fromStdString(state_str))
    .arg(QString::fromStdString(msg->current_route_id))
    .arg(QString::fromStdString(msg->current_job_id))
    .arg(msg->stop_active ? "active" : "inactive")
    .arg(msg->route_progress, 0, 'f', 3);

  if (!msg->last_error.empty()) {
    text += QString("\nError: %1").arg(QString::fromStdString(msg->last_error));
  }

  // Thread-safe UI update via QMetaObject::invokeMethod
  QMetaObject::invokeMethod(
    status_label_, "setText", Qt::QueuedConnection, Q_ARG(QString, text));
}

void FmsBridgePanel::publishWaypointMarkers()
{
  if (!pub_markers_) {
    return;
  }

  using visualization_msgs::msg::Marker;
  using visualization_msgs::msg::MarkerArray;

  MarkerArray array;
  const auto stamp = raw_node_->now();
  const std::string ns = "fms_waypoints";
  const std::string frame = "map";

  // First: DELETEALL to wipe stale markers (handles row removal cleanly)
  {
    Marker clear;
    clear.header.frame_id = frame;
    clear.header.stamp = stamp;
    clear.ns = ns;
    clear.action = Marker::DELETEALL;
    array.markers.push_back(clear);
  }

  const int rows = waypoint_table_->rowCount();

  auto color_for_type = [](int t, float & r, float & g, float & b) {
    switch (t) {
      case SegmentWaypoint::POINT_NORMAL:           r = 1.0f; g = 1.0f; b = 1.0f; break;
      case SegmentWaypoint::POINT_DESTINATION:      r = 1.0f; g = 0.1f; b = 0.1f; break;
      case SegmentWaypoint::POINT_LC_START:         r = 0.0f; g = 0.9f; b = 0.9f; break;
      case SegmentWaypoint::POINT_LC_END:           r = 0.2f; g = 0.4f; b = 1.0f; break;
      case SegmentWaypoint::POINT_POI:              r = 1.0f; g = 0.9f; b = 0.1f; break;
      case SegmentWaypoint::POINT_LOCAL_PLAN_START: r = 0.2f; g = 1.0f; b = 0.2f; break;
      case SegmentWaypoint::POINT_LOCAL_PLAN_END:   r = 0.0f; g = 0.5f; b = 0.0f; break;
      default:                                      r = 1.0f; g = 0.0f; b = 1.0f; break;  // DRAWING / unknown
    }
  };

  // Line strip connecting all points
  Marker line;
  line.header.frame_id = frame;
  line.header.stamp = stamp;
  line.ns = ns;
  line.id = 9000;
  line.type = Marker::LINE_STRIP;
  line.action = Marker::ADD;
  line.scale.x = 0.1;
  line.color.r = 0.2f;
  line.color.g = 1.0f;
  line.color.b = 0.2f;
  line.color.a = 0.9f;
  line.pose.orientation.w = 1.0;

  for (int row = 0; row < rows; ++row) {
    if (!waypoint_table_->item(row, 0) || !waypoint_table_->item(row, 1) ||
        !waypoint_table_->item(row, 6))
    {
      continue;
    }
    const double x = waypoint_table_->item(row, 0)->text().toDouble();
    const double y = waypoint_table_->item(row, 1)->text().toDouble();
    const double z = waypoint_table_->item(row, 2)
      ? waypoint_table_->item(row, 2)->text().toDouble() : 0.0;
    const double heading = waypoint_table_->item(row, 3)
      ? waypoint_table_->item(row, 3)->text().toDouble() : 0.0;
    const QString type_text = waypoint_table_->item(row, 4)
      ? waypoint_table_->item(row, 4)->text() : QString("?");
    const double speed = waypoint_table_->item(row, 5)
      ? waypoint_table_->item(row, 5)->text().toDouble() : 0.0;
    const int type_enum = waypoint_table_->item(row, 6)->text().toInt();
    auto * dir_item = waypoint_table_->item(row, 7);
    const bool is_reverse = dir_item &&
      dir_item->text().compare("Rev", Qt::CaseInsensitive) == 0;

    float cr, cg, cb;
    color_for_type(type_enum, cr, cg, cb);
    if (is_reverse) {
      // 倒车点：强调橙色，并淡化 type 颜色，便于一眼区分
      cr = 1.0f; cg = 0.5f; cb = 0.0f;
    }

    // Sphere at the waypoint
    Marker sphere;
    sphere.header.frame_id = frame;
    sphere.header.stamp = stamp;
    sphere.ns = ns;
    sphere.id = 100 + row;
    sphere.type = Marker::SPHERE;
    sphere.action = Marker::ADD;
    sphere.pose.position.x = x;
    sphere.pose.position.y = y;
    sphere.pose.position.z = z;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = 0.6;
    sphere.scale.y = 0.6;
    sphere.scale.z = 0.6;
    sphere.color.r = cr;
    sphere.color.g = cg;
    sphere.color.b = cb;
    sphere.color.a = 1.0f;
    array.markers.push_back(sphere);

    // Arrow for heading (yaw -> quaternion)
    Marker arrow;
    arrow.header.frame_id = frame;
    arrow.header.stamp = stamp;
    arrow.ns = ns;
    arrow.id = 1000 + row;
    arrow.type = Marker::ARROW;
    arrow.action = Marker::ADD;
    arrow.pose.position.x = x;
    arrow.pose.position.y = y;
    arrow.pose.position.z = z;
    // 箭头始终指向车头朝向：正向 = heading，倒车 = heading + π。
    const double display_yaw = is_reverse ? (heading + M_PI) : heading;
    const double half = display_yaw * 0.5;
    arrow.pose.orientation.x = 0.0;
    arrow.pose.orientation.y = 0.0;
    arrow.pose.orientation.z = std::sin(half);
    arrow.pose.orientation.w = std::cos(half);
    arrow.scale.x = 1.5;    // length
    arrow.scale.y = 0.15;   // shaft diameter
    arrow.scale.z = 0.25;   // head diameter
    arrow.color.r = cr;
    arrow.color.g = cg;
    arrow.color.b = cb;
    arrow.color.a = 0.9f;
    array.markers.push_back(arrow);

    // Text label above the point
    Marker text;
    text.header.frame_id = frame;
    text.header.stamp = stamp;
    text.ns = ns;
    text.id = 2000 + row;
    text.type = Marker::TEXT_VIEW_FACING;
    text.action = Marker::ADD;
    text.pose.position.x = x;
    text.pose.position.y = y;
    text.pose.position.z = z + 0.8;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.5;
    text.color.r = 1.0f;
    text.color.g = 1.0f;
    text.color.b = 1.0f;
    text.color.a = 1.0f;
    text.text = QString("#%1 %2%3 v=%4")
      .arg(row)
      .arg(is_reverse ? "[R] " : "")
      .arg(type_text)
      .arg(speed, 0, 'f', 1).toStdString();
    array.markers.push_back(text);

    // Line strip vertex
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    line.points.push_back(p);
  }

  if (line.points.size() >= 2) {
    array.markers.push_back(line);
  }

  pub_markers_->publish(array);
}

}  // namespace fms_bridge_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fms_bridge_rviz_plugin::FmsBridgePanel, rviz_common::Panel)
