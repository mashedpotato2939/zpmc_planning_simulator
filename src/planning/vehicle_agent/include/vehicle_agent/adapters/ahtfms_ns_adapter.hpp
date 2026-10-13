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

#ifndef VEHICLE_AGENT__ADAPTERS__AHTFMS_NS_ADAPTER_HPP_
#define VEHICLE_AGENT__ADAPTERS__AHTFMS_NS_ADAPTER_HPP_

#include "vehicle_agent/adapters/ahtfms_ns/transport.hpp"
#include "vehicle_agent/adapters/i_cloud_protocol_adapter.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace vehicle_agent
{

// AHTFMS-NS cloud protocol adapter.
//
// Speaks the ICD v1.1.1 framing (FMS1 + CM Header + Msg Header + PB payload)
// over an injectable transport. Production uses TcpTransport; tests inject
// FakeTransport so the adapter can be exercised end-to-end without a broker.
//
// Sprint 4 scope:
//   - Inbound: RouteNotify, JobNotify, StopPointNotify, RemoteCommand
//     → ir::Mission / ir::Command via AdapterCallbacks
//   - Outbound: telemetry → VehicleBasicStatus; ack → RemoteCommandResponse
//   - On connect: emit one VehicleKeyEventStatus SYNC_REQUEST so FMS re-pushes
//     job/route context (ICD §8.8).
//
// Not yet wired (later sprints): RestrictedArea/Vessel/QC/YC/Container/Config
// notifications are decoded but discarded; trajectory uplink is stubbed.
class AhtfmsNsAdapter : public ICloudProtocolAdapter
{
public:
  struct Config
  {
    std::string vehicle_id;          // CM Header sender_sys (also self target_sys for inbound gate)
    std::string fms_sys{"AHTFMS"};   // CM Header target_sys for outbound
    std::string broker_host;         // TCP host
    uint16_t broker_port{0};
    uint32_t max_payload_bytes{1u << 20};   // hard cap per frame (1 MiB)
    uint32_t reconnect_min_ms{500};
    uint32_t reconnect_max_ms{10000};
  };

  // Production constructor: builds a TcpTransport from cfg.
  explicit AhtfmsNsAdapter(Config cfg);

  // Test constructor: caller-provided transport (typically FakeTransport).
  // Ownership transfers to the adapter so it can manage lifecycle uniformly.
  AhtfmsNsAdapter(Config cfg, std::unique_ptr<ahtfms_ns::ITransport> transport);

  ~AhtfmsNsAdapter() override;

  std::string name() const override { return "ahtfms_ns"; }
  AdapterCapabilities capabilities() const override;

  void start(AdapterCallbacks callbacks) override;
  void stop() override;
  bool connected() const override;

  // Uplink.
  void publishTelemetry(const ir::Telemetry & t) override;
  void publishMissionStatus(const ir::MissionStatus & s) override;
  void publishAck(const ir::Ack & ack) override;

  // Test helpers — accessible without RTTI to keep gtest setup simple.
  ahtfms_ns::ITransport & transport() { return *transport_; }

  // Diagnostic counters. Bumped on every frame seen on the wire.
  struct Counters
  {
    uint64_t frames_in{0};
    uint64_t frames_out{0};
    uint64_t decode_errors{0};
    uint64_t target_mismatches{0};
    uint64_t unsupported_msg_type{0};
  };
  Counters counters() const;

private:
  void onBytes(const std::vector<uint8_t> & bytes);
  void onConnectionState(bool up);
  bool sendFrame(uint16_t msg_type_id, const std::vector<uint8_t> & payload);

  Config cfg_;
  std::unique_ptr<ahtfms_ns::ITransport> transport_;
  AdapterCallbacks callbacks_;
  std::atomic<bool> started_{false};

  // Inbound stream-reassembly buffer. recv() returns arbitrary chunks; we keep
  // a tail until we can pull a whole frame out.
  std::mutex rx_mtx_;
  std::vector<uint8_t> rx_tail_;

  mutable std::mutex counters_mtx_;
  Counters counters_{};
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__ADAPTERS__AHTFMS_NS_ADAPTER_HPP_
