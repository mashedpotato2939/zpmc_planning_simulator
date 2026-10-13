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

#ifndef VEHICLE_AGENT__ADAPTERS__INTERNAL_ZPMC__BUS_HPP_
#define VEHICLE_AGENT__ADAPTERS__INTERNAL_ZPMC__BUS_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

// Publish/subscribe seam between InternalZpmcAdapter and the wire. The real
// transport is MQTT 5 over TLS (paho-cpp); tests use FakeMqttBus, an
// in-memory broker that routes between paired client handles.
//
// This abstraction deliberately mirrors only the subset of MQTT that the
// adapter actually uses — topic-exact pub/sub, QoS 1, no retained, no
// wildcards. Adding wildcards later is straightforward but pulls in topic-
// filter parsing that is not needed for the ZPMC topic plan.

namespace vehicle_agent::internal_zpmc
{

using OnMessage = std::function<void(const std::string & topic,
                                     const std::vector<uint8_t> & payload)>;
using OnConnectionState = std::function<void(bool /*connected*/)>;

class IMqttBus
{
public:
  virtual ~IMqttBus() = default;

  virtual void start() = 0;
  virtual void stop() = 0;

  // Returns false when the bus is disconnected — the adapter is expected to
  // route the message into the offline queue in that case.
  virtual bool publish(const std::string & topic,
                       const std::vector<uint8_t> & payload,
                       uint8_t qos = 1) = 0;

  virtual void subscribe(const std::string & topic) = 0;
  virtual void unsubscribe(const std::string & topic) = 0;

  virtual bool connected() const = 0;

  virtual void setOnMessage(OnMessage cb) = 0;
  virtual void setOnConnectionState(OnConnectionState cb) = 0;
};

// In-process broker that several FakeMqttBus handles can attach to. The
// broker is the test rig's stand-in for an MQTT server: it owns the topic
// routing table and synchronously delivers each publish to every subscriber
// of an exact-match topic. Wildcards are intentionally unsupported here —
// the ZPMC topic plan is exact-match.
class FakeMqttBroker
{
public:
  struct Subscriber
  {
    std::string client_id;
    OnMessage cb;
  };

  void subscribe(const std::string & topic,
                 const std::string & client_id,
                 OnMessage cb);
  void unsubscribe(const std::string & topic, const std::string & client_id);

  // Returns count of subscribers the message was delivered to. Useful in
  // tests to assert "exactly N consumers saw this".
  std::size_t publish(const std::string & topic,
                      const std::vector<uint8_t> & payload);

  std::size_t subscriberCount(const std::string & topic) const;

private:
  mutable std::mutex mtx_;
  std::unordered_map<std::string, std::vector<Subscriber>> table_;
};

class FakeMqttBus : public IMqttBus
{
public:
  // The broker is borrowed; the caller keeps it alive for the lifetime of
  // every attached FakeMqttBus.
  FakeMqttBus(FakeMqttBroker & broker, std::string client_id);
  ~FakeMqttBus() override;

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

  // Test helpers — flip the connection without restarting.
  void connect();
  void disconnect();

  // Counters useful for assertions.
  std::size_t publishesAttempted() const;
  std::size_t publishesAccepted() const;

private:
  void deliver(const std::string & topic,
               const std::vector<uint8_t> & payload);

  FakeMqttBroker & broker_;
  std::string client_id_;

  mutable std::mutex mtx_;
  std::vector<std::string> subscriptions_;
  OnMessage on_message_;
  OnConnectionState on_state_;
  bool connected_{false};
  std::size_t pub_attempted_{0};
  std::size_t pub_accepted_{0};
};

}  // namespace vehicle_agent::internal_zpmc
#endif  // VEHICLE_AGENT__ADAPTERS__INTERNAL_ZPMC__BUS_HPP_
