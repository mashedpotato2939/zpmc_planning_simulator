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

#include "vehicle_agent/adapters/internal_zpmc_adapter.hpp"

#ifdef VEHICLE_AGENT_HAS_PAHO_MQTT
#include "vehicle_agent/adapters/internal_zpmc/paho_bus.hpp"
#endif

#include <nlohmann/json.hpp>

#include <chrono>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>

namespace vehicle_agent
{

namespace
{
uint64_t nowMs()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

std::string randomHex(std::size_t bytes)
{
  // Sufficient entropy for replay-protection nonces; not used for crypto.
  static thread_local std::mt19937_64 rng{std::random_device{}()};
  std::uniform_int_distribution<uint32_t> dist(0, 0xFFFFFFFFu);
  std::ostringstream os;
  os << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < bytes; i += 4) {
    os << std::setw(8) << dist(rng);
  }
  return os.str().substr(0, bytes * 2);
}
}  // namespace

namespace
{
std::unique_ptr<internal_zpmc::IMqttBus> buildPahoBus(
  const InternalZpmcAdapter::Config & cfg)
{
#ifdef VEHICLE_AGENT_HAS_PAHO_MQTT
  internal_zpmc::PahoMqttBus::Config pcfg;
  pcfg.broker_host = cfg.broker_host;
  pcfg.broker_port = cfg.broker_port;
  // The default :8883 port implies TLS; :1883 is plaintext. Callers can
  // still use ssl:// against any port by leaving mtls populated.
  pcfg.use_tls = (cfg.broker_port == 8883) || cfg.mtls.cert_path.size() > 0 ||
                 cfg.mtls.ca_path.size() > 0;
  pcfg.client_id = std::string("va-internal-zpmc-") + cfg.vehicle_id;
  pcfg.mtls = cfg.mtls;
  return std::make_unique<internal_zpmc::PahoMqttBus>(std::move(pcfg));
#else
  (void)cfg;
  return nullptr;
#endif
}
}  // namespace

InternalZpmcAdapter::InternalZpmcAdapter(Config cfg)
: InternalZpmcAdapter(cfg, buildPahoBus(cfg))
{
}

InternalZpmcAdapter::InternalZpmcAdapter(Config cfg,
                                         std::unique_ptr<internal_zpmc::IMqttBus> bus)
: cfg_(std::move(cfg)),
  bus_(std::move(bus)),
  signer_(cfg_.hmac_key.empty() ? nullptr
                                : std::make_unique<HmacSigner>(cfg_.hmac_key)),
  nonce_(cfg_.nonce_capacity, cfg_.nonce_ttl_ms),
  skew_(cfg_.skew_tolerance_ms),
  mtls_(cfg_.mtls)
{
}

InternalZpmcAdapter::~InternalZpmcAdapter()
{
  stop();
}

AdapterCapabilities InternalZpmcAdapter::capabilities() const
{
  AdapterCapabilities c;
  c.supports_mission = true;
  c.supports_command = true;
  c.supports_telemetry_uplink = true;
  c.supports_ack = true;
  c.supports_estop = true;     // ESTOP arrives on the command channel
  c.priority = 300;            // > ahtfms_ns (200) > scenario_loopback (100)
  return c;
}

void InternalZpmcAdapter::start(AdapterCallbacks callbacks)
{
  if (running_.exchange(true)) return;
  cb_ = std::move(callbacks);

  if (!bus_) {
    // Production path: paho-cpp wiring lands once the vendored library is
    // restored in src/others/. Surface the gap loudly rather than silently
    // running as a no-op.
    throw std::runtime_error(
      "InternalZpmcAdapter: production MQTT bus not yet available; supply a "
      "test bus or restore paho-cpp");
  }

  bus_->setOnMessage([this](const std::string & t, const std::vector<uint8_t> & p) {
    onBusMessage(t, p);
  });
  bus_->setOnConnectionState([this](bool up) { onBusState(up); });
  bus_->start();

  // Topics we listen on are vehicle-scoped.
  bus_->subscribe(topicFor(cfg_.topic_mission));
  bus_->subscribe(topicFor(cfg_.topic_command));
}

void InternalZpmcAdapter::stop()
{
  if (!running_.exchange(false)) return;
  if (bus_) {
    bus_->unsubscribe(topicFor(cfg_.topic_mission));
    bus_->unsubscribe(topicFor(cfg_.topic_command));
    bus_->stop();
  }
}

bool InternalZpmcAdapter::connected() const
{
  return bus_ && bus_->connected();
}

std::string InternalZpmcAdapter::topicFor(const std::string & prefix) const
{
  return prefix + cfg_.vehicle_id;
}

bool InternalZpmcAdapter::sendSigned(const std::string & topic,
                                     const nlohmann::json & body)
{
  if (!bus_) return false;
  const auto payload = body.dump();
  nlohmann::json env;
  env["payload"] = body;
  env["ts"] = nowMs();
  env["nonce"] = randomHex(8);
  if (signer_) {
    env["sig"] = signer_->sign(payload, env["ts"].get<uint64_t>(),
                               env["nonce"].get<std::string>());
  }
  const auto s = env.dump();
  std::vector<uint8_t> bytes(s.begin(), s.end());
  const bool ok = bus_->publish(topic, bytes);
  if (ok) {
    std::lock_guard<std::mutex> lk(mtx_);
    ++counters_.messages_out;
  }
  return ok;
}

std::optional<nlohmann::json>
InternalZpmcAdapter::openEnvelope(const std::vector<uint8_t> & raw)
{
  nlohmann::json env;
  try {
    env = nlohmann::json::parse(raw.begin(), raw.end());
  } catch (const std::exception &) {
    std::lock_guard<std::mutex> lk(mtx_);
    ++counters_.decode_errors;
    return std::nullopt;
  }

  if (!env.contains("payload") || !env.contains("ts") || !env.contains("nonce")) {
    std::lock_guard<std::mutex> lk(mtx_);
    ++counters_.decode_errors;
    return std::nullopt;
  }

  const auto ts = env["ts"].get<uint64_t>();
  const auto nonce = env["nonce"].get<std::string>();
  const auto payload_str = env["payload"].dump();

  if (signer_) {
    if (!env.contains("sig")) {
      std::lock_guard<std::mutex> lk(mtx_);
      ++counters_.bad_signature;
      return std::nullopt;
    }
    if (!signer_->verify(payload_str, ts, nonce, env["sig"].get<std::string>())) {
      std::lock_guard<std::mutex> lk(mtx_);
      ++counters_.bad_signature;
      return std::nullopt;
    }
  }

  if (!skew_.accepts(ts, nowMs())) {
    std::lock_guard<std::mutex> lk(mtx_);
    ++counters_.skew_rejections;
    return std::nullopt;
  }

  if (nonce_.seenAndRecord(nonce, ts)) {
    std::lock_guard<std::mutex> lk(mtx_);
    ++counters_.replay_rejections;
    return std::nullopt;
  }

  return env["payload"];
}

void InternalZpmcAdapter::onBusMessage(const std::string & topic,
                                      const std::vector<uint8_t> & payload)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    ++counters_.messages_in;
  }
  auto inner = openEnvelope(payload);
  if (!inner) return;

  try {
    if (topic == topicFor(cfg_.topic_mission)) {
      auto m = inner->get<ir::Mission>();
      m.source_adapter = name();
      if (cb_.on_mission) cb_.on_mission(std::move(m));
    } else if (topic == topicFor(cfg_.topic_command)) {
      auto c = inner->get<ir::Command>();
      c.source_adapter = name();
      if (cb_.on_command) cb_.on_command(std::move(c));
    }
    // Other topics (status, telemetry, ack) are uplink-only; ignore inbound.
  } catch (const std::exception &) {
    std::lock_guard<std::mutex> lk(mtx_);
    ++counters_.decode_errors;
  }
}

void InternalZpmcAdapter::onBusState(bool up)
{
  if (cb_.on_connection_state_changed) {
    cb_.on_connection_state_changed(name(), up);
  }
}

void InternalZpmcAdapter::publishTelemetry(const ir::Telemetry & t)
{
  nlohmann::json j = t;
  sendSigned(topicFor(cfg_.topic_telemetry), j);
}

void InternalZpmcAdapter::publishMissionStatus(const ir::MissionStatus & s)
{
  nlohmann::json j = s;
  sendSigned(topicFor(cfg_.topic_status), j);
}

void InternalZpmcAdapter::publishAck(const ir::Ack & a)
{
  nlohmann::json j = a;
  sendSigned(topicFor(cfg_.topic_ack), j);
}

InternalZpmcAdapter::Counters InternalZpmcAdapter::counters() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return counters_;
}

}  // namespace vehicle_agent
