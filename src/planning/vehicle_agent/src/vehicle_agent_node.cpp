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

#include "vehicle_agent/adapters/adapter_multiplexer.hpp"
#include "vehicle_agent/adapters/ahtfms_ns_adapter.hpp"
#include "vehicle_agent/adapters/internal_zpmc_adapter.hpp"
#include "vehicle_agent/adapters/ros_injector_adapter.hpp"
#include "vehicle_agent/adapters/scenario_loopback_adapter.hpp"
#include "vehicle_agent/app/estop_channel.hpp"
#include "vehicle_agent/app/event_bus.hpp"
#include "vehicle_agent/app/mission_state_machine.hpp"
#include "vehicle_agent/app/structured_log.hpp"
#include "vehicle_agent/ros_bridge/diagnostic_publisher.hpp"
#include "vehicle_agent/ros_bridge/ros_publisher.hpp"
#include "vehicle_agent/ros_bridge/ros_subscriber.hpp"
#include "vehicle_agent/storage/offline_queue.hpp"

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <atomic>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace vehicle_agent
{

namespace
{
std::string expand_path(const std::string & in)
{
  if (in.empty() || in[0] != '~') return in;
  const char * home = std::getenv("HOME");
  if (!home) return in;
  return std::string(home) + in.substr(1);
}
}  // namespace

class VehicleAgentNode : public rclcpp::Node
{
public:
  explicit VehicleAgentNode(const rclcpp::NodeOptions & opts)
  : rclcpp::Node("vehicle_agent", opts),
    bus_(declare_parameter<int>("event_bus_capacity", 1024))
  {
    const auto ns_prefix = declare_parameter<std::string>("publish_ns", "~");
    const auto queue_db_path = expand_path(declare_parameter<std::string>(
      "queue_db_path", "~/.ros/vehicle_agent/queue.db"));
    const auto vehicle_id = declare_parameter<std::string>("vehicle_id", "");
    const auto odometry_topic = declare_parameter<std::string>(
      "odometry_topic", "/localization/kinematic_state");
    const auto telemetry_rate_hz = declare_parameter<double>("telemetry_rate_hz", 10.0);
    const auto diag_rate_hz = declare_parameter<double>("diagnostic_rate_hz", 1.0);
    const auto queue_warn_threshold = static_cast<size_t>(
      declare_parameter<int>("queue_warn_threshold", 32));

    publisher_ = std::make_unique<RosPublisher>(*this, ns_prefix);
    queue_ = std::make_unique<OfflineQueue>(queue_db_path);
    RCLCPP_INFO(get_logger(), "offline_queue: %s (pending=%zu)",
                queue_->path().c_str(), queue_->pendingCount());

    MissionStateMachine::Sinks sinks;
    sinks.on_status = [this](const ir::MissionStatus & s) {
      publisher_->publishMissionStatus(s);
      mux_.publishMissionStatus(s);
      if (subscriber_) {
        const bool e_stop = (s.state == ir::MissionState::ABORTING ||
                             s.state == ir::MissionState::FAILED) &&
                             estop_->active();
        subscriber_->updateMissionContext(
          s.mission_id, static_cast<uint8_t>(s.state), e_stop);
      }
      onMissionStatus(s);
      VA_LOG_INFO(get_logger(), s.mission_id, "[fsm] state=%s detail=\"%s\"",
                  ir::to_string(s.state), s.detail.c_str());
    };
    sinks.on_ack = [this](const ir::Ack & a) {
      mux_.publishAck(a);
    };
    fsm_ = std::make_unique<MissionStateMachine>(std::move(sinks));
    estop_ = std::make_unique<EstopChannel>(*fsm_);

    DiagnosticPublisher::Config diag_cfg;
    diag_cfg.rate_hz = diag_rate_hz;
    diag_cfg.queue_warn_threshold = queue_warn_threshold;
    diag_ = std::make_unique<DiagnosticPublisher>(
      *this, diag_cfg,
      [this]() { return queue_ ? queue_->pendingCount() : 0u; },
      [this]() { return fsm_ ? fsm_->state() : ir::MissionState::IDLE; });

    // Telemetry subscriber feeds adapter uplink.
    RosSubscriber::Config sub_cfg;
    sub_cfg.vehicle_id = vehicle_id;
    sub_cfg.odometry_topic = odometry_topic;
    sub_cfg.max_rate_hz = telemetry_rate_hz;
    RosSubscriber::Sinks sub_sinks;
    sub_sinks.on_telemetry = [this](const ir::Telemetry & t) {
      mux_.publishTelemetry(t);
    };
    subscriber_ = std::make_unique<RosSubscriber>(*this, sub_cfg, std::move(sub_sinks));

    // Adapter wiring (Sprint 1: scenario_loopback only).
    const auto adapters = declare_parameter<std::vector<std::string>>(
      "adapters", std::vector<std::string>{"scenario_loopback"});
    const auto scenario_path = declare_parameter<std::string>(
      "scenario_path", "");

    // AHTFMS-NS adapter parameters. Declared up front so launches can override
    // even when the adapter isn't selected (avoids spurious "param not
    // declared" warnings from rclcpp).
    AhtfmsNsAdapter::Config ahtfms_cfg;
    ahtfms_cfg.vehicle_id = declare_parameter<std::string>(
      "ahtfms_ns.vehicle_id", vehicle_id.empty() ? std::string{"IGV2001"} : vehicle_id);
    ahtfms_cfg.fms_sys = declare_parameter<std::string>("ahtfms_ns.fms_sys", "AHTFMS");
    ahtfms_cfg.broker_host = declare_parameter<std::string>(
      "ahtfms_ns.broker_host", "127.0.0.1");
    ahtfms_cfg.broker_port = static_cast<uint16_t>(
      declare_parameter<int>("ahtfms_ns.broker_port", 8801));
    ahtfms_cfg.max_payload_bytes = static_cast<uint32_t>(
      declare_parameter<int>("ahtfms_ns.max_payload_bytes", 1 << 20));
    ahtfms_cfg.reconnect_min_ms = static_cast<uint32_t>(
      declare_parameter<int>("ahtfms_ns.reconnect_min_ms", 500));
    ahtfms_cfg.reconnect_max_ms = static_cast<uint32_t>(
      declare_parameter<int>("ahtfms_ns.reconnect_max_ms", 10000));

    // Internal ZPMC adapter parameters. Same up-front-declaration pattern as
    // above: declared even when the adapter isn't selected so launches can
    // override without rclcpp warnings.
    InternalZpmcAdapter::Config zpmc_cfg;
    zpmc_cfg.vehicle_id = declare_parameter<std::string>(
      "internal_zpmc.vehicle_id", vehicle_id.empty() ? std::string{"IGV2001"} : vehicle_id);
    zpmc_cfg.broker_host = declare_parameter<std::string>(
      "internal_zpmc.broker_host", "127.0.0.1");
    zpmc_cfg.broker_port = static_cast<uint16_t>(
      declare_parameter<int>("internal_zpmc.broker_port", 8883));
    zpmc_cfg.hmac_key = declare_parameter<std::string>("internal_zpmc.hmac_key", "");
    zpmc_cfg.skew_tolerance_ms = static_cast<uint64_t>(
      declare_parameter<int>("internal_zpmc.skew_tolerance_ms", 30000));
    zpmc_cfg.nonce_capacity = static_cast<size_t>(
      declare_parameter<int>("internal_zpmc.nonce_capacity", 4096));
    zpmc_cfg.nonce_ttl_ms = static_cast<uint64_t>(
      declare_parameter<int>("internal_zpmc.nonce_ttl_ms", 60000));
    zpmc_cfg.mtls.ca_path = declare_parameter<std::string>(
      "internal_zpmc.mtls.ca_path", "");
    zpmc_cfg.mtls.cert_path = declare_parameter<std::string>(
      "internal_zpmc.mtls.cert_path", "");
    zpmc_cfg.mtls.key_path = declare_parameter<std::string>(
      "internal_zpmc.mtls.key_path", "");

    // ros_injector adapter parameters — declared up front for the same
    // reason as the others above.
    RosInjectorAdapter::Config injector_cfg;
    injector_cfg.mission_topic = declare_parameter<std::string>(
      "ros_injector.mission_topic", "/vehicle_agent/inject_mission");
    injector_cfg.command_topic = declare_parameter<std::string>(
      "ros_injector.command_topic", "/vehicle_agent/inject_command");

    for (const auto & name : adapters) {
      if (name == "scenario_loopback") {
        if (scenario_path.empty()) {
          RCLCPP_ERROR(get_logger(), "scenario_path must be set for scenario_loopback");
          continue;
        }
        mux_.addAdapter(std::make_unique<ScenarioLoopbackAdapter>(scenario_path));
        diag_->registerAdapter("scenario_loopback");
        RCLCPP_INFO(get_logger(), "added adapter: scenario_loopback (%s)", scenario_path.c_str());
      } else if (name == "ahtfms_ns") {
        mux_.addAdapter(std::make_unique<AhtfmsNsAdapter>(ahtfms_cfg));
        diag_->registerAdapter("ahtfms_ns");
        RCLCPP_INFO(get_logger(),
          "added adapter: ahtfms_ns (CM=%s:%u, vehicle=%s)",
          ahtfms_cfg.broker_host.c_str(), ahtfms_cfg.broker_port,
          ahtfms_cfg.vehicle_id.c_str());
      } else if (name == "ros_injector") {
        mux_.addAdapter(std::make_unique<RosInjectorAdapter>(this, injector_cfg));
        diag_->registerAdapter("ros_injector");
        RCLCPP_INFO(get_logger(),
          "added adapter: ros_injector (mission=%s, command=%s)",
          injector_cfg.mission_topic.c_str(), injector_cfg.command_topic.c_str());
      } else if (name == "internal_zpmc") {
        // The production MQTT bus (paho-cpp) is not yet wired; constructing
        // the adapter is safe, but start() will throw until paho lands. Keep
        // the registration here so the launch graph is real and the gap is
        // obvious in logs the moment someone enables it.
        try {
          mux_.addAdapter(std::make_unique<InternalZpmcAdapter>(zpmc_cfg));
          diag_->registerAdapter("internal_zpmc");
          RCLCPP_INFO(get_logger(),
            "added adapter: internal_zpmc (broker=%s:%u, vehicle=%s)",
            zpmc_cfg.broker_host.c_str(), zpmc_cfg.broker_port,
            zpmc_cfg.vehicle_id.c_str());
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(),
            "internal_zpmc adapter unavailable: %s", e.what());
        }
      } else {
        RCLCPP_WARN(get_logger(), "adapter '%s' not yet implemented", name.c_str());
      }
    }

    AdapterCallbacks cb;
    cb.on_mission = [this](ir::Mission m) {
      const int64_t row = queue_->enqueueMission(m);
      {
        std::lock_guard<std::mutex> lk(map_mtx_);
        mission_rows_[m.id] = row;
      }
      publisher_->publishMission(m);
      ir::Event ev{ir::EventKind::MISSION_RECEIVED, std::move(m), row};
      bus_.push(std::move(ev));
    };
    cb.on_command = [this](ir::Command c) {
      // E-stop is the fast path: bypass the queue and the bus.
      if (c.kind == ir::CommandKind::ESTOP) {
        publisher_->publishCommand(c);
        estop_->trigger(c);
        return;
      }
      const int64_t row = queue_->enqueueCommand(c);
      publisher_->publishCommand(c);
      ir::Event ev{ir::EventKind::COMMAND_RECEIVED, std::move(c), row};
      bus_.push(std::move(ev));
    };
    cb.on_execution_complete = [this](std::string mission_id) {
      bus_.push(ir::Event{ir::EventKind::EXECUTION_COMPLETE,
                          ir::ExecutionComplete{std::move(mission_id)}, 0});
    };
    cb.on_connection_state_changed = [this](std::string name, bool connected) {
      if (diag_) diag_->setAdapterConnected(name, connected);
      RCLCPP_INFO(get_logger(), "adapter '%s' connected=%s",
        name.c_str(), connected ? "true" : "false");
    };

    // FSM thread (sole consumer of the bus).
    fsm_thread_ = std::thread([this]() {
      while (!stop_) {
        auto ev = bus_.popBlocking();
        if (!ev) break;
        const int64_t row = ev->queue_row_id;
        const auto kind = ev->kind;
        fsm_->handle(*ev);
        if (row != 0 && kind == ir::EventKind::COMMAND_RECEIVED) {
          // Commands are transient; ack as soon as the FSM has handled.
          try {
            queue_->ack(row);
          } catch (const std::exception & e) {
            RCLCPP_WARN(get_logger(), "queue ack(command,%ld) failed: %s", row, e.what());
          }
        }
      }
    });

    mux_.startAll(std::move(cb));

    // Replay any pending downlink left over from a prior crash. Done AFTER
    // adapters start, so the replay rejoins the same event stream order.
    replayPending();

    RCLCPP_INFO(get_logger(), "vehicle_agent started with %zu adapter(s); primary=%s",
      mux_.size(), mux_.primaryName().c_str());
  }

  ~VehicleAgentNode() override
  {
    mux_.stopAll();
    stop_ = true;
    bus_.close();
    if (fsm_thread_.joinable()) fsm_thread_.join();
  }

private:
  // Replay unacked rows in receive order. Decode payload back into IR and
  // push onto the bus as if a fresh adapter delivered it.
  void replayPending()
  {
    auto items = queue_->peekPending(4096);
    if (items.empty()) return;
    RCLCPP_INFO(get_logger(), "replaying %zu pending offline rows", items.size());
    for (const auto & it : items) {
      try {
        auto j = nlohmann::json::parse(it.payload_json);
        if (it.kind == "mission") {
          auto m = j.get<ir::Mission>();
          {
            std::lock_guard<std::mutex> lk(map_mtx_);
            mission_rows_[m.id] = it.id;
          }
          bus_.push(ir::Event{ir::EventKind::MISSION_RECEIVED, std::move(m), it.id});
        } else if (it.kind == "command") {
          auto c = j.get<ir::Command>();
          // Re-issue ESTOP fast-path immediately rather than via the bus.
          if (c.kind == ir::CommandKind::ESTOP) {
            estop_->trigger(c);
            try { queue_->ack(it.id); } catch (...) {}
            continue;
          }
          bus_.push(ir::Event{ir::EventKind::COMMAND_RECEIVED, std::move(c), it.id});
        } else {
          RCLCPP_WARN(get_logger(), "replay: unknown kind '%s' for row %ld",
                      it.kind.c_str(), it.id);
          try { queue_->ack(it.id); } catch (...) {}
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(),
          "replay: decode failed for row %ld kind=%s: %s — acking to drop",
          it.id, it.kind.c_str(), e.what());
        try { queue_->ack(it.id); } catch (...) {}
      }
    }
  }

  void onMissionStatus(const ir::MissionStatus & s)
  {
    // Ack mission queue row once the FSM reaches a terminal state.
    if (s.state != ir::MissionState::DONE && s.state != ir::MissionState::FAILED) {
      return;
    }
    int64_t row = 0;
    {
      std::lock_guard<std::mutex> lk(map_mtx_);
      auto it = mission_rows_.find(s.mission_id);
      if (it == mission_rows_.end()) return;
      row = it->second;
      mission_rows_.erase(it);
    }
    try {
      queue_->ack(row);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "queue ack(mission=%s,row=%ld) failed: %s",
                  s.mission_id.c_str(), row, e.what());
    }
  }

  EventBus bus_;
  AdapterMultiplexer mux_;
  std::unique_ptr<MissionStateMachine> fsm_;
  std::unique_ptr<EstopChannel> estop_;
  std::unique_ptr<RosPublisher> publisher_;
  std::unique_ptr<RosSubscriber> subscriber_;
  std::unique_ptr<OfflineQueue> queue_;
  std::unique_ptr<DiagnosticPublisher> diag_;
  std::thread fsm_thread_;
  std::atomic<bool> stop_{false};

  std::mutex map_mtx_;
  std::unordered_map<std::string, int64_t> mission_rows_;
};

}  // namespace vehicle_agent

RCLCPP_COMPONENTS_REGISTER_NODE(vehicle_agent::VehicleAgentNode)
