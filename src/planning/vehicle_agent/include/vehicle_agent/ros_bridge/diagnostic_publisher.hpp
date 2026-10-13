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

#ifndef VEHICLE_AGENT__ROS_BRIDGE__DIAGNOSTIC_PUBLISHER_HPP_
#define VEHICLE_AGENT__ROS_BRIDGE__DIAGNOSTIC_PUBLISHER_HPP_

#include "vehicle_agent/ir/mission_status.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <map>
#include <mutex>
#include <string>

namespace vehicle_agent
{

// Publishes /diagnostics at 1Hz with one DiagnosticStatus per adapter plus
// a top-level vehicle_agent status. Status levels:
//   OK    — at least one adapter connected, FSM not in FAILED
//   WARN  — all adapters disconnected, OR queue depth > queue_warn_threshold
//   ERROR — FSM is FAILED
class DiagnosticPublisher
{
public:
  using QueueDepthFn = std::function<size_t()>;
  using FsmStateFn   = std::function<ir::MissionState()>;

  struct Config
  {
    double rate_hz{1.0};
    size_t queue_warn_threshold{32};
    std::string hardware_id{"vehicle_agent"};
  };

  DiagnosticPublisher(rclcpp::Node & node,
                      Config cfg,
                      QueueDepthFn queue_depth,
                      FsmStateFn fsm_state);

  // Track an adapter. Initial state is disconnected with no last_error.
  void registerAdapter(const std::string & name);

  // Adapter connection state flipped (called from on_connection_state_changed).
  void setAdapterConnected(const std::string & name, bool connected);

  // Adapter reported an error (string is freeform; goes into diagnostic
  // key/value pairs).
  void setAdapterLastError(const std::string & name, const std::string & error);

private:
  struct AdapterState
  {
    bool connected{false};
    std::string last_error;
    uint64_t connected_since_ms{0};
  };

  void tick();
  diagnostic_msgs::msg::DiagnosticArray buildArray();

  rclcpp::Node & node_;
  Config cfg_;
  QueueDepthFn queue_depth_;
  FsmStateFn fsm_state_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex mtx_;
  std::map<std::string, AdapterState> adapters_;
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__ROS_BRIDGE__DIAGNOSTIC_PUBLISHER_HPP_
