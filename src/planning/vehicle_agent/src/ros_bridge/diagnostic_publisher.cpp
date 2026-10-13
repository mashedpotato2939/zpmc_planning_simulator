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

#include "vehicle_agent/ros_bridge/diagnostic_publisher.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <chrono>
#include <sstream>

namespace vehicle_agent
{

namespace
{
using DS = diagnostic_msgs::msg::DiagnosticStatus;

diagnostic_msgs::msg::KeyValue kv(const std::string & k, const std::string & v)
{
  diagnostic_msgs::msg::KeyValue out;
  out.key = k;
  out.value = v;
  return out;
}

const char * fsm_state_name(ir::MissionState s)
{
  return ir::to_string(s);
}

uint64_t now_ms()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}
}  // namespace

DiagnosticPublisher::DiagnosticPublisher(rclcpp::Node & node,
                                         Config cfg,
                                         QueueDepthFn queue_depth,
                                         FsmStateFn fsm_state)
: node_(node),
  cfg_(std::move(cfg)),
  queue_depth_(std::move(queue_depth)),
  fsm_state_(std::move(fsm_state))
{
  pub_ = node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(10));

  const auto period_ms = static_cast<int>(1000.0 / std::max(cfg_.rate_hz, 0.1));
  timer_ = node_.create_wall_timer(
    std::chrono::milliseconds(period_ms),
    [this]() { tick(); });
}

void DiagnosticPublisher::registerAdapter(const std::string & name)
{
  std::lock_guard<std::mutex> lock(mtx_);
  adapters_.emplace(name, AdapterState{});
}

void DiagnosticPublisher::setAdapterConnected(const std::string & name, bool connected)
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto & s = adapters_[name];
  if (connected && !s.connected) {
    s.connected_since_ms = now_ms();
  }
  s.connected = connected;
}

void DiagnosticPublisher::setAdapterLastError(const std::string & name,
                                              const std::string & error)
{
  std::lock_guard<std::mutex> lock(mtx_);
  adapters_[name].last_error = error;
}

void DiagnosticPublisher::tick()
{
  auto msg = buildArray();
  pub_->publish(std::move(msg));
}

diagnostic_msgs::msg::DiagnosticArray DiagnosticPublisher::buildArray()
{
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = node_.now();

  size_t depth = queue_depth_ ? queue_depth_() : 0u;
  ir::MissionState fsm = fsm_state_ ? fsm_state_() : ir::MissionState::IDLE;

  std::lock_guard<std::mutex> lock(mtx_);

  // Per-adapter rows.
  size_t connected_count = 0;
  for (const auto & [name, s] : adapters_) {
    DS d;
    d.name = "vehicle_agent: adapter/" + name;
    d.hardware_id = cfg_.hardware_id;
    if (s.connected) {
      d.level = DS::OK;
      d.message = "connected";
      ++connected_count;
    } else {
      d.level = DS::WARN;
      d.message = "disconnected";
    }
    d.values.push_back(kv("connected", s.connected ? "true" : "false"));
    if (!s.last_error.empty()) {
      d.values.push_back(kv("last_error", s.last_error));
    }
    if (s.connected_since_ms != 0) {
      d.values.push_back(kv("connected_since_ms",
                            std::to_string(s.connected_since_ms)));
    }
    arr.status.push_back(std::move(d));
  }

  // Top-level vehicle_agent row.
  DS top;
  top.name = "vehicle_agent: state";
  top.hardware_id = cfg_.hardware_id;
  if (fsm == ir::MissionState::FAILED) {
    top.level = DS::ERROR;
    top.message = "FSM in FAILED";
  } else if (!adapters_.empty() && connected_count == 0) {
    top.level = DS::WARN;
    top.message = "no adapters connected";
  } else if (depth > cfg_.queue_warn_threshold) {
    top.level = DS::WARN;
    std::ostringstream m;
    m << "offline queue depth=" << depth << " (>" << cfg_.queue_warn_threshold << ")";
    top.message = m.str();
  } else {
    top.level = DS::OK;
    top.message = "ok";
  }
  top.values.push_back(kv("fsm_state", fsm_state_name(fsm)));
  top.values.push_back(kv("queue_depth", std::to_string(depth)));
  top.values.push_back(kv("adapter_count", std::to_string(adapters_.size())));
  top.values.push_back(kv("adapters_connected", std::to_string(connected_count)));
  arr.status.push_back(std::move(top));

  return arr;
}

}  // namespace vehicle_agent
