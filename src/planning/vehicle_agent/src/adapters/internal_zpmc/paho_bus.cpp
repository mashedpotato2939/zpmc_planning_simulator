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

#include "vehicle_agent/adapters/internal_zpmc/paho_bus.hpp"

#include <mqtt/async_client.h>
#include <mqtt/callback.h>
#include <mqtt/connect_options.h>
#include <mqtt/ssl_options.h>

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <utility>

namespace vehicle_agent::internal_zpmc
{

// Paho dispatches every connection / delivery callback on its own IO thread.
// We forward to the owning PahoMqttBus, which re-enters the bus-level
// callbacks the adapter installed. The bus owns the lifetime of this object;
// paho holds a non-owning reference.
class PahoMqttBus::CallbackImpl : public mqtt::callback
{
public:
  explicit CallbackImpl(PahoMqttBus * owner) : owner_(owner) {}

  void connected(const std::string & cause) override
  {
    if (owner_) owner_->onConnected(cause);
  }

  void connection_lost(const std::string & cause) override
  {
    if (owner_) owner_->onConnectionLost(cause);
  }

  void message_arrived(mqtt::const_message_ptr msg) override
  {
    if (!owner_ || !msg) return;
    const auto & topic = msg->get_topic();
    const auto & payload = msg->get_payload_ref();
    const auto * raw = reinterpret_cast<const uint8_t *>(payload.data());
    std::vector<uint8_t> bytes(raw, raw + payload.size());
    owner_->onMessage(topic, bytes);
  }

  // delivery_complete left at base no-op — paho's async client gives us
  // delivery tokens already, and the adapter does not currently consume
  // delivery confirmations.

private:
  PahoMqttBus * owner_;
};

PahoMqttBus::PahoMqttBus(Config cfg) : cfg_(std::move(cfg))
{
  if (cfg_.client_id.empty()) {
    throw std::invalid_argument("PahoMqttBus: client_id must not be empty");
  }
  // Build the async_client up front so callers can register callbacks via
  // setOnMessage / setOnConnectionState before start(). The client is
  // disconnected until start() runs.
  mqtt::create_options copts(MQTTVERSION_5);
  if (cfg_.persistence_dir.empty()) {
    client_ = std::make_unique<mqtt::async_client>(
      composeUri(), cfg_.client_id, copts, nullptr);
  } else {
    client_ = std::make_unique<mqtt::async_client>(
      composeUri(), cfg_.client_id, copts, cfg_.persistence_dir);
  }

  callback_ = std::make_unique<CallbackImpl>(this);
  client_->set_callback(*callback_);
}

PahoMqttBus::~PahoMqttBus()
{
  stop();
  // Drop the callback ref before tearing down the callback object so paho
  // does not dispatch into a dangling pointer if a final IO event arrives
  // during teardown.
  if (client_) {
    try {
      // Replacing the callback with a no-op stand-in by swapping in a
      // default-constructed mqtt::callback would require an extra heap
      // alloc; instead null out our forwarding owner so any stray dispatch
      // is dropped.
      callback_.reset();
    } catch (...) {
      // Destructor must not propagate.
    }
  }
}

std::string PahoMqttBus::composeUri() const
{
  const std::string scheme = cfg_.use_tls ? "ssl://" : "tcp://";
  return scheme + cfg_.broker_host + ":" + std::to_string(cfg_.broker_port);
}

void PahoMqttBus::start()
{
  if (running_.exchange(true)) return;

  mqtt::connect_options_builder builder = mqtt::connect_options_builder{}
    .mqtt_version(MQTTVERSION_5)
    .clean_start(cfg_.clean_start)
    .keep_alive_interval(std::chrono::seconds(cfg_.keep_alive_sec))
    .connect_timeout(std::chrono::seconds(cfg_.connect_timeout_sec))
    .automatic_reconnect(
      std::chrono::seconds(cfg_.min_retry_sec),
      std::chrono::seconds(cfg_.max_retry_sec));
  if (!cfg_.username.empty()) builder.user_name(cfg_.username);
  if (!cfg_.password.empty()) builder.password(cfg_.password);

  if (cfg_.use_tls) {
    mqtt::ssl_options_builder ssl_b;
    if (!cfg_.mtls.ca_path.empty()) ssl_b.trust_store(cfg_.mtls.ca_path);
    if (!cfg_.mtls.cert_path.empty()) ssl_b.key_store(cfg_.mtls.cert_path);
    if (!cfg_.mtls.key_path.empty()) ssl_b.private_key(cfg_.mtls.key_path);
    if (!cfg_.mtls.key_password.empty()) {
      ssl_b.private_keypassword(cfg_.mtls.key_password);
    }
    ssl_b.enable_server_cert_auth(cfg_.mtls.verify_server);
    ssl_b.verify(cfg_.mtls.verify_server);
    if (cfg_.mtls.enable_alpn) {
      ssl_b.alpn_protos({"mqtt"});
    }
    builder.ssl(ssl_b.finalize());
  }

  try {
    client_->connect(builder.finalize());
    // Connection completion arrives asynchronously via onConnected(); the
    // adapter learns about it through on_state_(true). We deliberately do
    // NOT block here — start() must remain non-blocking so the agent can
    // come up even when the broker is unreachable.
  } catch (const mqtt::exception & ex) {
    // A throw here means the connect could not even be dispatched (bad
    // URI, invalid options, etc.). Surface as a disconnected state so the
    // adapter records it; the runtime_error wraps for the caller's log.
    onConnectionLost(ex.what());
    running_.store(false);
    throw std::runtime_error(std::string("PahoMqttBus: connect failed: ") + ex.what());
  }
}

void PahoMqttBus::stop()
{
  if (!running_.exchange(false)) return;
  if (!client_) return;
  try {
    if (client_->is_connected()) {
      // Best-effort unsub of every active subscription so the broker
      // releases server-side state.
      std::vector<std::string> subs;
      {
        std::lock_guard<std::mutex> lk(mtx_);
        subs.swap(active_subs_);
      }
      for (const auto & t : subs) {
        try { client_->unsubscribe(t)->wait_for(std::chrono::seconds(2)); }
        catch (...) { /* ignored — we're shutting down */ }
      }
      try { client_->disconnect()->wait_for(std::chrono::seconds(2)); }
      catch (...) { /* ignored */ }
    }
  } catch (...) {
    // stop() must not throw — paho can still raise if the broker is
    // already gone.
  }
  connected_.store(false);
}

bool PahoMqttBus::publish(const std::string & topic,
                          const std::vector<uint8_t> & payload,
                          uint8_t qos)
{
  if (!client_) return false;
  if (!connected_.load()) return false;
  try {
    auto msg = mqtt::make_message(topic, payload.data(), payload.size(),
                                  static_cast<int>(qos), /*retained=*/false);
    client_->publish(msg);
    return true;
  } catch (const mqtt::exception &) {
    return false;
  }
}

void PahoMqttBus::subscribe(const std::string & topic)
{
  if (!client_) return;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (std::find(active_subs_.begin(), active_subs_.end(), topic) ==
        active_subs_.end()) {
      active_subs_.push_back(topic);
    }
  }
  if (!connected_.load()) {
    // Defer the actual SUBSCRIBE until the broker is reachable. paho's
    // async_client does NOT replay subscriptions across a connect, so
    // onConnected() walks active_subs_ and re-issues them.
    return;
  }
  try {
    client_->subscribe(topic, /*qos=*/1);
  } catch (const mqtt::exception &) {
    // Best-effort. A failed SUBSCRIBE leaves the entry in active_subs_;
    // the next onConnected() will retry.
  }
}

void PahoMqttBus::unsubscribe(const std::string & topic)
{
  if (!client_) return;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    active_subs_.erase(
      std::remove(active_subs_.begin(), active_subs_.end(), topic),
      active_subs_.end());
  }
  if (!connected_.load()) return;
  try {
    client_->unsubscribe(topic);
  } catch (const mqtt::exception &) {
    // Ignored — server-side state will be cleaned up on disconnect.
  }
}

bool PahoMqttBus::connected() const
{
  return connected_.load();
}

void PahoMqttBus::setOnMessage(OnMessage cb)
{
  std::lock_guard<std::mutex> lk(mtx_);
  on_message_ = std::move(cb);
}

void PahoMqttBus::setOnConnectionState(OnConnectionState cb)
{
  std::lock_guard<std::mutex> lk(mtx_);
  on_state_ = std::move(cb);
}

void PahoMqttBus::onConnected(const std::string & /*cause*/)
{
  connected_.store(true);

  // Re-issue any subscriptions that were requested before the broker came
  // up (or that survived a reconnect — paho's automatic_reconnect does NOT
  // replay subscriptions, only the TCP/MQTT session).
  std::vector<std::string> subs;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    subs = active_subs_;
  }
  for (const auto & t : subs) {
    try { client_->subscribe(t, /*qos=*/1); }
    catch (const mqtt::exception &) { /* retry on next reconnect */ }
  }

  OnConnectionState cb;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cb = on_state_;
  }
  if (cb) cb(true);
}

void PahoMqttBus::onConnectionLost(const std::string & /*cause*/)
{
  connected_.store(false);
  OnConnectionState cb;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cb = on_state_;
  }
  if (cb) cb(false);
}

void PahoMqttBus::onMessage(const std::string & topic,
                            const std::vector<uint8_t> & payload)
{
  OnMessage cb;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cb = on_message_;
  }
  if (cb) cb(topic, payload);
}

}  // namespace vehicle_agent::internal_zpmc
