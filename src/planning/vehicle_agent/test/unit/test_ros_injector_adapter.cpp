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

#include "vehicle_agent/adapters/ros_injector_adapter.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;
using vehicle_agent::RosInjectorAdapter;
using vehicle_agent::AdapterCallbacks;
using IrMission = igv_planning_msgs::msg::IrMission;
using IrCommand = igv_planning_msgs::msg::IrCommand;
using IrWaypoint = igv_planning_msgs::msg::IrWaypoint;

class RosInjectorAdapterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
    exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    static int counter = 0;
    const auto suffix = std::to_string(++counter);
    mission_topic_ = "/inject_mission_" + suffix;
    command_topic_ = "/inject_command_" + suffix;

    host_ = std::make_shared<rclcpp::Node>("rosinj_host" + suffix);
    helper_ = std::make_shared<rclcpp::Node>("rosinj_helper" + suffix);

    pub_mission_ = helper_->create_publisher<IrMission>(mission_topic_, 10);
    pub_command_ = helper_->create_publisher<IrCommand>(command_topic_, 10);

    exec_->add_node(host_);
    exec_->add_node(helper_);
  }

  void TearDown() override
  {
    exec_->remove_node(host_);
    exec_->remove_node(helper_);
    exec_.reset();
  }

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
  std::shared_ptr<rclcpp::Node> host_;
  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::Publisher<IrMission>::SharedPtr pub_mission_;
  rclcpp::Publisher<IrCommand>::SharedPtr pub_command_;
  std::string mission_topic_, command_topic_;
};

TEST_F(RosInjectorAdapterTest, MissionInTriggersOnMission)
{
  RosInjectorAdapter::Config cfg;
  cfg.mission_topic = mission_topic_;
  cfg.command_topic = command_topic_;
  RosInjectorAdapter adapter(host_.get(), cfg);

  std::optional<vehicle_agent::ir::Mission> got;
  bool conn_up = false;
  AdapterCallbacks cb;
  cb.on_mission = [&](vehicle_agent::ir::Mission m) { got = std::move(m); };
  cb.on_connection_state_changed =
    [&](std::string, bool up) { if (up) conn_up = true; };
  adapter.start(cb);
  EXPECT_TRUE(conn_up) << "start() should fire connection-state(true)";
  EXPECT_TRUE(adapter.connected());

  IrMission m;
  m.id = "INJ-1";
  m.kind = IrMission::KIND_ROUTE;
  m.seq = 7;
  IrWaypoint w;
  w.x = 11.0; w.y = 22.0; w.yaw = 0.5; w.target_speed = 1.25;
  w.segment_id = "seg0:fwd";
  m.waypoints.push_back(w);
  pub_mission_->publish(m);

  ASSERT_TRUE(spinUntil([&]() { return got.has_value(); }));
  EXPECT_EQ(got->id, "INJ-1");
  EXPECT_EQ(got->seq, 7u);
  EXPECT_EQ(got->source_adapter, "ros_injector")
    << "adapter must stamp source_adapter unconditionally";
  ASSERT_EQ(got->waypoints.size(), 1u);
  EXPECT_DOUBLE_EQ(got->waypoints[0].x, 11.0);
  EXPECT_DOUBLE_EQ(got->waypoints[0].target_speed, 1.25);
  EXPECT_EQ(got->waypoints[0].segment_id, "seg0:fwd");

  const auto cnt = adapter.counters();
  EXPECT_EQ(cnt.missions_in, 1u);

  adapter.stop();
  EXPECT_FALSE(adapter.connected());
}

TEST_F(RosInjectorAdapterTest, CommandInTriggersOnCommand)
{
  RosInjectorAdapter::Config cfg;
  cfg.mission_topic = mission_topic_;
  cfg.command_topic = command_topic_;
  RosInjectorAdapter adapter(host_.get(), cfg);

  std::optional<vehicle_agent::ir::Command> got;
  AdapterCallbacks cb;
  cb.on_command = [&](vehicle_agent::ir::Command c) { got = std::move(c); };
  adapter.start(cb);

  IrCommand c;
  c.mission_id = "INJ-mid";
  c.kind = IrCommand::KIND_PAUSE;
  c.issued_at_ms = 1234567890ULL;
  pub_command_->publish(c);

  ASSERT_TRUE(spinUntil([&]() { return got.has_value(); }));
  EXPECT_EQ(got->mission_id, "INJ-mid");
  EXPECT_EQ(got->kind, vehicle_agent::ir::CommandKind::PAUSE);
  EXPECT_EQ(got->source_adapter, "ros_injector");
  EXPECT_EQ(got->issued_at_ms, 1234567890ULL);

  adapter.stop();
}

TEST_F(RosInjectorAdapterTest, StopDropsSubscriptionsAndFlipsConnState)
{
  RosInjectorAdapter::Config cfg;
  cfg.mission_topic = mission_topic_;
  cfg.command_topic = command_topic_;
  RosInjectorAdapter adapter(host_.get(), cfg);

  std::vector<std::pair<std::string, bool>> conn_events;
  std::size_t mission_count = 0;
  AdapterCallbacks cb;
  cb.on_mission = [&](vehicle_agent::ir::Mission) { ++mission_count; };
  cb.on_connection_state_changed =
    [&](std::string n, bool up) { conn_events.emplace_back(std::move(n), up); };
  adapter.start(cb);
  ASSERT_FALSE(conn_events.empty());
  EXPECT_TRUE(conn_events.front().second);

  adapter.stop();
  EXPECT_FALSE(adapter.connected());
  ASSERT_GE(conn_events.size(), 2u);
  EXPECT_FALSE(conn_events.back().second)
    << "stop() should fire connection-state(false)";

  // Post-stop publishes must not be observed.
  IrMission m;
  m.id = "after-stop";
  pub_mission_->publish(m);
  spinUntil([]() { return false; }, 200);
  EXPECT_EQ(mission_count, 0u);
}
