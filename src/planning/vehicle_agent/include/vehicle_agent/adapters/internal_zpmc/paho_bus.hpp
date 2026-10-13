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

#ifndef VEHICLE_AGENT__ADAPTERS__INTERNAL_ZPMC__PAHO_BUS_HPP_
#define VEHICLE_AGENT__ADAPTERS__INTERNAL_ZPMC__PAHO_BUS_HPP_

#include "vehicle_agent/adapters/internal_zpmc/bus.hpp"
#include "vehicle_agent/security/mtls_context.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Forward-declare paho-cpp types so this header doesn't drag the full
// async_client.h through every translation unit that includes the bus.
namespace mqtt
{
class async_client;
}

namespace vehicle_agent::internal_zpmc
{

// Real MQTT 5 bus backed by Eclipse Paho's async_client. Mirrors the
// IMqttBus surface used by FakeMqttBus so InternalZpmcAdapter can be
// switched between the two with no logic changes.
//
// Threading: paho dispatches `message_arrived` and the connection-state
// hooks on its internal IO thread. We re-enter the caller-supplied
// OnMessage / OnConnectionState callbacks from that thread; the adapter
// guards its own state with a mutex.
class PahoMqttBus : public IMqttBus
{
public:
  struct Config
  {
    // tcp://host:1883 or ssl://host:8883. The host:port pair from the
    // adapter config is composed here in the ctor — callers do not need
    // to build the URI themselves.
    std::string broker_host{"127.0.0.1"};
    uint16_t broker_port{8883};
    bool use_tls{true};

    std::string client_id;          // required; broker rejects duplicates
    std::string username;            // optional
    std::string password;            // optional

    // ConnectOption tuning. Defaults match the ZPMC topic plan's QoS
    // expectations: clean start TRUE (we don't replay queued downlink on
    // reconnect — the cloud retransmits), 30s keep-alive, automatic
    // reconnect with capped backoff.
    bool clean_start{true};
    uint32_t keep_alive_sec{30};
    uint32_t connect_timeout_sec{20};
    uint32_t min_retry_sec{1};
    uint32_t max_retry_sec{30};

    // mTLS material. Only honored when use_tls is true. If ca_path is
    // empty paho falls back to the system trust store.
    MtlsConfig mtls;

    // Persistence directory. Empty disables persistence (in-memory only) —
    // the InternalZpmcAdapter's offline queue is the real durability seam,
    // so disabling paho's persistence avoids a second write path.
    std::string persistence_dir;
  };

  explicit PahoMqttBus(Config cfg);
  ~PahoMqttBus() override;

  // Idempotent. Kicks off an async connect; the OnConnectionState callback
  // fires when the broker accepts. publish() before connection is queued by
  // paho up to the in-flight window, but we report `connected()==false`
  // until the connect actually completes so the adapter routes to the
  // offline queue.
  void start() override;
  void stop() override;

  bool publish(const std::string & topic,
               const std::vector<uint8_t> & payload,
               uint8_t qos = 1) override;

  void subscribe(const std::string & topic) override;
  void unsubscribe(const std::string & topic) override;

  bool connected() const override;

  void setOnMessage(OnMessage cb) override;
  void setOnConnectionState(OnConnectionState cb) override;

private:
  class CallbackImpl;
  friend class CallbackImpl;

  // Compose ssl://host:port or tcp://host:port from cfg_.
  std::string composeUri() const;

  // Invoked by CallbackImpl on the paho dispatch thread.
  void onConnected(const std::string & cause);
  void onConnectionLost(const std::string & cause);
  void onMessage(const std::string & topic,
                 const std::vector<uint8_t> & payload);

  Config cfg_;
  std::unique_ptr<mqtt::async_client> client_;
  std::unique_ptr<CallbackImpl> callback_;

  mutable std::mutex mtx_;
  OnMessage on_message_;
  OnConnectionState on_state_;
  std::vector<std::string> active_subs_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
};

}  // namespace vehicle_agent::internal_zpmc
#endif  // VEHICLE_AGENT__ADAPTERS__INTERNAL_ZPMC__PAHO_BUS_HPP_
