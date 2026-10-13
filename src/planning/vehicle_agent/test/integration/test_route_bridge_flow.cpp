// Copyright 2026 ZPMC
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

#include "vehicle_agent/ros_bridge/route_bridge.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;
using vehicle_agent::RouteBridgeNode;
using IrMission = igv_planning_msgs::msg::IrMission;
using IrCommand = igv_planning_msgs::msg::IrCommand;
using IrWaypoint = igv_planning_msgs::msg::IrWaypoint;
using FMSRouteWindow = igv_planning_msgs::msg::FMSRouteWindow;
using FMSStopPoint = igv_planning_msgs::msg::FMSStopPoint;

class RouteBridgeFlow : public ::testing::Test
{
protected:
  // The executor is heap-allocated so we can defer construction until after
  // rclcpp::init has produced a valid global context — gtest fixture members
  // are constructed BEFORE SetUp() runs, and an executor built against a null
  // context throws from its ctor.
  void SetUp() override
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    rclcpp::NodeOptions opts;
    // Unique topic names per test instance so parallel ctest runs don't cross-talk.
    static int counter = 0;
    const auto suffix = std::to_string(++counter);
    mission_topic_ = "/vehicle_agent/mission_" + suffix;
    command_topic_ = "/vehicle_agent/command_" + suffix;
    route_topic_ = "/fms/route_window_" + suffix;
    stop_topic_ = "/fms/stop_point_" + suffix;
    odom_topic_ = "/localization/kinematic_state_" + suffix;

    opts.parameter_overrides({
      {"vehicle_id", std::string("IGV2001")},
      {"mission_topic", mission_topic_},
      {"command_topic", command_topic_},
      {"odometry_topic", odom_topic_},
      {"route_window_topic", route_topic_},
      {"stop_point_topic", stop_topic_},
      // Fast republish + tiny window so timer-driven assertions converge
      // within a few hundred ms rather than seconds.
      {"window_rate_hz", 20.0},
      {"window_ahead_m", 5.0},
      {"window_lookback_m", 1.0},
    });
    bridge_ = std::make_shared<RouteBridgeNode>(opts);
    helper_ = std::make_shared<rclcpp::Node>("route_bridge_test_helper" + suffix);

    pub_mission_ = helper_->create_publisher<IrMission>(mission_topic_, 10);
    pub_command_ = helper_->create_publisher<IrCommand>(command_topic_, 10);
    pub_odom_ = helper_->create_publisher<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS());

    sub_route_ = helper_->create_subscription<FMSRouteWindow>(
      route_topic_, 10,
      [this](FMSRouteWindow::ConstSharedPtr msg) {
        last_route_ = *msg;
        ++route_count_;
      });
    sub_stop_ = helper_->create_subscription<FMSStopPoint>(
      stop_topic_, 10,
      [this](FMSStopPoint::ConstSharedPtr msg) { last_stop_ = *msg; });

    exec_->add_node(bridge_);
    exec_->add_node(helper_);
  }

  void TearDown() override
  {
    exec_->remove_node(helper_);
    exec_->remove_node(bridge_);
    exec_.reset();
  }

  // Spin until pred() returns true or timeout. Returns whether pred was met.
  template <typename Pred>
  bool spinUntil(Pred pred, int timeout_ms = 1000)
  {
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
      exec_->spin_some();
      if (pred()) return true;
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::shared_ptr<RouteBridgeNode> bridge_;
  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::Publisher<IrMission>::SharedPtr pub_mission_;
  rclcpp::Publisher<IrCommand>::SharedPtr pub_command_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Subscription<FMSRouteWindow>::SharedPtr sub_route_;
  rclcpp::Subscription<FMSStopPoint>::SharedPtr sub_stop_;
  std::optional<FMSRouteWindow> last_route_;
  std::optional<FMSStopPoint> last_stop_;
  int route_count_{0};
  std::string mission_topic_, command_topic_, route_topic_, stop_topic_, odom_topic_;
};

TEST_F(RouteBridgeFlow, MissionInRouteOut)
{
  IrMission m;
  m.id = "R-flow-1";
  m.kind = IrMission::KIND_ROUTE;
  IrWaypoint w;
  w.x = 1.0; w.y = 2.0; w.yaw = 0.0; w.target_speed = 1.0;
  w.segment_id = "seg0:fwd";
  m.waypoints.push_back(w);
  pub_mission_->publish(m);

  ASSERT_TRUE(spinUntil([&]() { return last_route_.has_value(); }))
    << "bridge never emitted FMSRouteWindow";
  EXPECT_EQ(last_route_->route_id, "R-flow-1");
  ASSERT_EQ(last_route_->segments.size(), 1u);
  EXPECT_DOUBLE_EQ(last_route_->segments[0].points[0].x, 1.0);
}

TEST_F(RouteBridgeFlow, PauseUsesLatchedEgo)
{
  // Send an odometry sample first, then a PAUSE command.
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 5.5;
  odom.pose.pose.position.y = -1.5;
  pub_odom_->publish(odom);

  // Spin briefly so the bridge actually consumes the odom.
  spinUntil([]() { return false; }, 100);

  IrCommand c;
  c.mission_id = "R-pause";
  c.kind = IrCommand::KIND_PAUSE;
  pub_command_->publish(c);

  ASSERT_TRUE(spinUntil([&]() { return last_stop_.has_value(); }))
    << "bridge never emitted FMSStopPoint";
  EXPECT_DOUBLE_EQ(last_stop_->x, 5.5);
  EXPECT_DOUBLE_EQ(last_stop_->y, -1.5);
  EXPECT_FALSE(last_stop_->clear);
}

TEST_F(RouteBridgeFlow, EstopDoesNotEmitStopPoint)
{
  IrCommand c;
  c.mission_id = "R-estop";
  c.kind = IrCommand::KIND_ESTOP;
  pub_command_->publish(c);
  // ~250 ms is enough — if it were going to publish, it would have by now.
  spinUntil([&]() { return last_stop_.has_value(); }, 250);
  EXPECT_FALSE(last_stop_.has_value())
    << "ESTOP must not be routed through the converter";
}

TEST_F(RouteBridgeFlow, OneMissionGivesRepeatedWindows)
{
  // The single-push -> stale-and-halt failure mode this whole change exists
  // to fix: one IrMission in should produce a steady stream of route windows
  // out, at the configured window_rate_hz.
  IrMission m;
  m.id = "R-repeat";
  m.kind = IrMission::KIND_ROUTE;
  for (int i = 0; i < 5; ++i) {
    IrWaypoint w;
    w.x = double(i); w.y = 0.0; w.segment_id = "seg0:fwd";
    m.waypoints.push_back(w);
  }
  pub_mission_->publish(m);
  ASSERT_TRUE(spinUntil([&]() { return last_route_.has_value(); }))
    << "no window after initial publish";

  // window_rate_hz=20 → 4 windows in 300 ms with margin.
  const int initial_count = route_count_;
  spinUntil([&]() { return route_count_ >= initial_count + 4; }, 500);
  EXPECT_GE(route_count_, initial_count + 4)
    << "timer did not republish windows at the expected cadence";
  EXPECT_EQ(last_route_->route_id, "R-repeat");
}

TEST_F(RouteBridgeFlow, SameIdRefreshReplacesGeometry)
{
  // Send mission #1, wait for at least one window, then re-send with same id
  // and a different (extended) polyline. The downstream stream must reflect
  // the new geometry without any intervening STATUS_CANCELLED.
  IrMission m1;
  m1.id = "R-refresh";
  m1.kind = IrMission::KIND_ROUTE;
  for (int i = 0; i < 3; ++i) {
    IrWaypoint w; w.x = double(i); w.y = 0.0; w.segment_id = "seg0:fwd";
    m1.waypoints.push_back(w);
  }
  pub_mission_->publish(m1);
  ASSERT_TRUE(spinUntil([&]() { return last_route_.has_value(); }));

  IrMission m2 = m1;
  m2.seq = 1;
  for (int i = 3; i < 10; ++i) {
    IrWaypoint w; w.x = double(i); w.y = 0.0; w.segment_id = "seg0:fwd";
    m2.waypoints.push_back(w);
  }
  pub_mission_->publish(m2);

  ASSERT_TRUE(spinUntil([&]() {
    return last_route_.has_value() && !last_route_->segments.empty() &&
           last_route_->segments[0].points.size() >= 5u;
  }, 500)) << "extended waypoints never landed downstream";

  EXPECT_EQ(last_route_->status, FMSRouteWindow::STATUS_ACTIVE)
    << "refresh must not flip the stream to CANCELLED";
  // No odometry was published, so re-partition is inactive and the bare base id
  // is preserved across a same-id refresh.
  EXPECT_EQ(last_route_->route_id, "R-refresh");
}

TEST_F(RouteBridgeFlow, RouteIdRepartitionsByDistance)
{
  // The horizon-roll fix: as the ego advances, route_bridge bumps the published
  // route_id every repartition_step_m (default 50 m), which drives the converter
  // to rebuild and roll its forward horizon. Long straight route so the ego can
  // cross several partitions.
  IrMission m;
  m.id = "R-part";
  m.kind = IrMission::KIND_ROUTE;
  for (int i = 0; i <= 200; ++i) {
    IrWaypoint w; w.x = double(i); w.y = 0.0; w.segment_id = "seg0:fwd";
    m.waypoints.push_back(w);
  }
  pub_mission_->publish(m);

  auto publishEgo = [&](double x) {
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = 0.0;
    pub_odom_->publish(odom);
  };

  // First localized window anchors partition 0.
  publishEgo(0.0);
  ASSERT_TRUE(spinUntil([&]() {
    return last_route_.has_value() && last_route_->route_id == "R-part#p0";
  })) << "expected partition 0 once ego is localized, got "
      << (last_route_ ? last_route_->route_id : std::string("<none>"));

  // ~60 m forward (> 50 m step) → partition 1.
  publishEgo(60.0);
  ASSERT_TRUE(spinUntil([&]() {
    return last_route_.has_value() && last_route_->route_id == "R-part#p1";
  })) << "expected partition 1 after 60 m of travel";

  // Another ~60 m → partition 2.
  publishEgo(120.0);
  ASSERT_TRUE(spinUntil([&]() {
    return last_route_.has_value() && last_route_->route_id == "R-part#p2";
  })) << "expected partition 2 after 120 m of travel";

  // A sub-step move (< 50 m past the anchor) must NOT bump the partition.
  publishEgo(140.0);
  spinUntil([]() { return false; }, 150);
  EXPECT_EQ(last_route_->route_id, "R-part#p2")
    << "sub-step motion must not re-partition";
}
