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

#ifndef VEHICLE_AGENT__ADAPTERS__INTERNAL_ZPMC_ADAPTER_HPP_
#define VEHICLE_AGENT__ADAPTERS__INTERNAL_ZPMC_ADAPTER_HPP_

#include "vehicle_agent/adapters/i_cloud_protocol_adapter.hpp"
#include "vehicle_agent/adapters/internal_zpmc/bus.hpp"
#include "vehicle_agent/security/clock_skew_window.hpp"
#include "vehicle_agent/security/hmac_signer.hpp"
#include "vehicle_agent/security/mtls_context.hpp"
#include "vehicle_agent/security/nonce_lru.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace vehicle_agent
{

// Adapter for ZPMC's internal MQTT 5 + HMAC cloud protocol. Every outbound
// message is wrapped:
//   { "payload": <ir_json>, "ts": <epoch_ms>, "nonce": <hex>, "sig": <hex> }
// Inbound messages are accepted iff
//   (a) HMAC verifies,
//   (b) ts is within ±skew_tolerance_ms of the local clock,
//   (c) the nonce has not been seen recently (replay protection).
//
// Topics are flat exact-match strings derived from vehicle_id. See
// docs/internal_zpmc-topics.md (planned) for the full table.
class InternalZpmcAdapter : public ICloudProtocolAdapter
{
public:
  struct Config
  {
    std::string vehicle_id{"IGV2001"};
    std::string broker_host{"127.0.0.1"};
    uint16_t broker_port{8883};

    // HMAC-SHA256 shared secret. Empty disables signing on outbound and
    // verification on inbound — useful for the test fake but should be
    // refused in production by config validation.
    std::string hmac_key;

    // Anti-replay window. Messages older/newer than this from local time are
    // rejected even with a valid MAC.
    uint64_t skew_tolerance_ms{30000};   // ±30 s

    // Nonce ring. Sized for ~30 s of traffic at 100 Hz per channel.
    std::size_t nonce_capacity{4096};
    uint64_t nonce_ttl_ms{60000};

    // mTLS material. Only validated; the actual paho ssl_options wiring lives
    // in the production bus once paho-cpp is in the build.
    MtlsConfig mtls;

    // Topic plan. Defaults match docs/vehicle_agent/Vehicle_Agent_Design_v1.2.
    std::string topic_mission{"zpmc/v1/mission/"};       // append vehicle_id
    std::string topic_command{"zpmc/v1/command/"};
    std::string topic_telemetry{"zpmc/v1/telemetry/"};
    std::string topic_status{"zpmc/v1/status/"};
    std::string topic_ack{"zpmc/v1/ack/"};
  };

  struct Counters
  {
    uint64_t messages_in{0};
    uint64_t messages_out{0};
    uint64_t bad_signature{0};
    uint64_t skew_rejections{0};
    uint64_t replay_rejections{0};
    uint64_t decode_errors{0};
  };

  // Production constructor: builds the real (paho-backed) MQTT bus internally
  // once the vendored paho is restored. Until then this constructor returns
  // an adapter that fails to start; the test constructor below is the only
  // working entrypoint in Sprint 5.
  explicit InternalZpmcAdapter(Config cfg);

  // Test constructor: caller supplies the bus.
  InternalZpmcAdapter(Config cfg, std::unique_ptr<internal_zpmc::IMqttBus> bus);

  ~InternalZpmcAdapter() override;

  std::string name() const override { return "internal_zpmc"; }
  AdapterCapabilities capabilities() const override;

  void start(AdapterCallbacks callbacks) override;
  void stop() override;
  bool connected() const override;

  void publishTelemetry(const ir::Telemetry & t) override;
  void publishMissionStatus(const ir::MissionStatus & s) override;
  void publishAck(const ir::Ack & a) override;

  Counters counters() const;

private:
  std::string topicFor(const std::string & prefix) const;
  void onBusMessage(const std::string & topic, const std::vector<uint8_t> & payload);
  void onBusState(bool up);

  // Wrap, sign, send. Returns false if the bus rejected the publish (offline)
  // — the multiplexer/queue is expected to retry.
  bool sendSigned(const std::string & topic, const nlohmann::json & body);

  // Verify+open the security envelope. Returns the inner payload JSON when
  // accepted; std::nullopt otherwise (with reason recorded in counters).
  std::optional<nlohmann::json> openEnvelope(const std::vector<uint8_t> & raw);

  Config cfg_;
  std::unique_ptr<internal_zpmc::IMqttBus> bus_;
  AdapterCallbacks cb_;

  std::unique_ptr<HmacSigner> signer_;   // nullptr when key empty (permissive)
  NonceLRU nonce_;
  ClockSkewWindow skew_;
  MtlsContext mtls_;

  mutable std::mutex mtx_;
  Counters counters_{};
  std::atomic<bool> running_{false};
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__ADAPTERS__INTERNAL_ZPMC_ADAPTER_HPP_
