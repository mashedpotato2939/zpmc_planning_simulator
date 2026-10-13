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

#ifndef FMS_BRIDGE_PANEL_HPP_
#define FMS_BRIDGE_PANEL_HPP_

#ifndef Q_MOC_RUN
#include <qt5/QtWidgets/QCheckBox>
#include <qt5/QtWidgets/QComboBox>
#include <qt5/QtWidgets/QDoubleSpinBox>
#include <qt5/QtWidgets/QFileDialog>
#include <qt5/QtWidgets/QLabel>
#include <qt5/QtWidgets/QLineEdit>
#include <qt5/QtWidgets/QMessageBox>
#include <qt5/QtWidgets/QPushButton>
#include <qt5/QtWidgets/QTableWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <igv_planning_msgs/msg/adapter_status.hpp>
#include <igv_planning_msgs/msg/fms_route_window.hpp>
#include <igv_planning_msgs/msg/fms_path_segment.hpp>
#include <igv_planning_msgs/msg/segment_waypoint.hpp>
#include <igv_planning_msgs/msg/fms_stop_point.hpp>
#include <igv_planning_msgs/msg/ir_command.hpp>
#include <igv_planning_msgs/msg/ir_mission.hpp>
#include <igv_planning_msgs/msg/ir_waypoint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#endif

namespace fms_bridge_rviz_plugin
{

using igv_planning_msgs::msg::AdapterStatus;
using igv_planning_msgs::msg::FMSRouteWindow;
using igv_planning_msgs::msg::FMSPathSegment;
using igv_planning_msgs::msg::SegmentWaypoint;
using igv_planning_msgs::msg::FMSStopPoint;
using igv_planning_msgs::msg::IrMission;
using igv_planning_msgs::msg::IrCommand;
using igv_planning_msgs::msg::IrWaypoint;

class FmsBridgePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit FmsBridgePanel(QWidget * parent = nullptr);
  void onInitialize() override;
  bool eventFilter(QObject * obj, QEvent * event) override;

public Q_SLOTS:
  void onAddPoint();
  void onRemoveLastPoint();
  void onClearPoints();
  void onSendRoute();
  void onSendStop();
  void onSendResume();
  void onSavePoints();
  void onLoadPoints();
  void onStopWindow();
  void onPickFromMap();

private:
  void onStatus(const AdapterStatus::ConstSharedPtr msg);
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onPickedPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void onWindowTimer();

  // Build an FMSRouteWindow from all waypoints in the table
  FMSRouteWindow buildFullRoute() const;

  // Build an IrMission from the same table — used by the "Publish via
  // vehicle_agent" toggle. Segment orientation is encoded into each
  // waypoint's segment_id ("seg0:fwd" / "seg0:rev") so the va_route_bridge
  // can recover it on the way back out.
  IrMission buildIrMission() const;

  // Compute and publish a windowed subset of master_route_ around ego
  void publishWindow();

  // Publish a MarkerArray reflecting the current waypoints in the table
  void publishWaypointMarkers();

  // Returns true when the panel should publish IR-shaped messages to
  // vehicle_agent instead of the legacy FMSRouteWindow / FMSStopPoint
  // shapes.
  bool isViaVehicleAgent() const;

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Publisher<FMSRouteWindow>::SharedPtr pub_route_;
  rclcpp::Publisher<FMSStopPoint>::SharedPtr pub_stop_;
  rclcpp::Publisher<IrMission>::SharedPtr pub_ir_mission_;
  rclcpp::Publisher<IrCommand>::SharedPtr pub_ir_command_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Subscription<AdapterStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_picked_pose_;

  // Header / global controls
  QComboBox * publish_via_combo_{nullptr};   // fms_path_converter | vehicle_agent
  uint64_t mission_seq_{0};                  // monotonic counter for IrMission.seq

  // Route section
  QLineEdit * vehicle_id_input_;
  QLineEdit * route_id_input_;
  QLineEdit * job_id_input_;
  QLineEdit * task_id_input_;
  QComboBox * orientation_combo_;
  QTableWidget * waypoint_table_;

  // Add point section
  QDoubleSpinBox * x_input_;
  QDoubleSpinBox * y_input_;
  QDoubleSpinBox * z_input_;
  QDoubleSpinBox * heading_input_;
  QPushButton * pick_from_map_btn_;
  QComboBox * type_combo_;
  QDoubleSpinBox * speed_input_;

  // Stop section
  QDoubleSpinBox * stop_x_input_;
  QDoubleSpinBox * stop_y_input_;

  // Sliding window section
  QCheckBox * window_enable_check_;
  QDoubleSpinBox * window_size_input_;
  QDoubleSpinBox * window_lookback_input_;
  QLabel * window_status_label_;
  QPushButton * stop_window_btn_;

  // Status section
  QLabel * status_label_;

  // Sliding window state
  QTimer * window_timer_{nullptr};
  QTimer * marker_timer_{nullptr};
  std::vector<SegmentWaypoint> master_waypoints_;   // full route stored on Send
  std::vector<uint8_t> master_directions_;          // per-waypoint orientation parallel to master_waypoints_
  uint8_t master_orientation_{0};
  bool window_active_{false};
  double ego_x_{0.0};
  double ego_y_{0.0};
  bool ego_received_{false};

  // Toolbar anti-auto-collapse
  bool toolbar_manual_toggle_ = false;
};

}  // namespace fms_bridge_rviz_plugin

#endif  // FMS_BRIDGE_PANEL_HPP_
