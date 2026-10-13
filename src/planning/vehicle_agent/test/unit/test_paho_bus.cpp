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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

// These tests exercise the offline path of PahoMqttBus — the parts that do
// not require a reachable broker. A separate, broker-gated set of cases at
// the bottom only runs when the MQTT_TEST_BROKER environment variable is
// set (e.g. MQTT_TEST_BROKER=tcp://127.0.0.1:1883). The broker-gated set
// is the closest thing to an integration test we can run inside colcon
// without a Mosquitto sidecar.

using vehicle_agent::internal_zpmc::PahoMqttBus;

namespace
{

PahoMqttBus::Config defaultLocalCfg()
{
  PahoMqttBus::Config cfg;
  cfg.broker_host = "127.0.0.1";
  cfg.broker_port = 1883;
  cfg.use_tls = false;
  cfg.client_id = "va-test-paho-bus";
  cfg.keep_alive_sec = 5;
  cfg.connect_timeout_sec = 1;
  cfg.min_retry_sec = 1;
  cfg.max_retry_sec = 1;
  return cfg;
}

}  // namespace

TEST(PahoMqttBusTest, ConstructionRejectsEmptyClientId)
{
  auto cfg = defaultLocalCfg();
  cfg.client_id.clear();
  EXPECT_THROW(PahoMqttBus bus(cfg), std::invalid_argument);
}

TEST(PahoMqttBusTest, PublishBeforeStartReturnsFalse)
{
  PahoMqttBus bus(defaultLocalCfg());
  EXPECT_FALSE(bus.connected());
  EXPECT_FALSE(bus.publish("zpmc/v1/telemetry/IGV2001", {0x01, 0x02, 0x03}));
}

TEST(PahoMqttBusTest, SubscribeBeforeStartIsBufferedNoThrow)
{
  // The contract: subscribe() pre-start (or pre-connect) is buffered into
  // active_subs_ and re-issued by onConnected. We can't verify the re-issue
  // here without a broker, but we can verify the no-throw + queued-state
  // behavior: a subsequent unsubscribe() of the same topic must also be a
  // no-throw no-op.
  PahoMqttBus bus(defaultLocalCfg());
  EXPECT_NO_THROW(bus.subscribe("zpmc/v1/mission/IGV2001"));
  EXPECT_NO_THROW(bus.unsubscribe("zpmc/v1/mission/IGV2001"));
  EXPECT_FALSE(bus.connected());
}

TEST(PahoMqttBusTest, StartAgainstUnreachableBrokerStaysDisconnected)
{
  // Point at a port that should never have anything listening on it. With
  // automatic_reconnect on the bus stays in "trying to reconnect" — it must
  // not report connected(), and it must not throw on start() (we want the
  // vehicle agent to come up even when the cloud is unreachable).
  auto cfg = defaultLocalCfg();
  cfg.broker_port = 1;   // privileged + nothing listening
  cfg.connect_timeout_sec = 1;

  PahoMqttBus bus(cfg);
  std::atomic<bool> any_state_event{false};
  std::atomic<bool> saw_connected_true{false};
  bus.setOnConnectionState([&](bool up) {
    any_state_event.store(true);
    if (up) saw_connected_true.store(true);
  });

  // start() should not throw even if the underlying connect can't reach.
  EXPECT_NO_THROW(bus.start());

  // Give paho a moment to try — connection_timeout_sec=1 above means we
  // wait a touch longer than that to be sure.
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  EXPECT_FALSE(bus.connected())
    << "bus must not report connected against an unreachable broker";
  EXPECT_FALSE(saw_connected_true.load())
    << "on_connection_state(true) must not fire when broker is unreachable";

  // stop() must be a clean no-throw even when never connected.
  EXPECT_NO_THROW(bus.stop());
}

TEST(PahoMqttBusTest, StartIsIdempotent)
{
  auto cfg = defaultLocalCfg();
  cfg.broker_port = 1;

  PahoMqttBus bus(cfg);
  EXPECT_NO_THROW(bus.start());
  // Second start() must be a silent no-op — the running_ atomic guards it.
  EXPECT_NO_THROW(bus.start());
  EXPECT_NO_THROW(bus.stop());
  // Second stop() likewise.
  EXPECT_NO_THROW(bus.stop());
}

// ---------------------------------------------------------------------------
// Broker-gated round trip. Skipped unless MQTT_TEST_BROKER is set.
//
// Example invocation:
//   docker run --rm -p 1883:1883 eclipse-mosquitto
//   MQTT_TEST_BROKER=tcp://127.0.0.1:1883 colcon test
//       --packages-select vehicle_agent --ctest-args -R PahoMqttBusBroker
// ---------------------------------------------------------------------------

TEST(PahoMqttBusBrokerTest, RoundTripsAPublish)
{
  const char * broker = std::getenv("MQTT_TEST_BROKER");
  if (!broker) {
    GTEST_SKIP() << "MQTT_TEST_BROKER not set — skipping live-broker test";
  }

  // Parse "tcp://host:port" or "ssl://host:port" — minimal scheme support.
  std::string uri = broker;
  bool tls = uri.rfind("ssl://", 0) == 0;
  const auto scheme_end = uri.find("://");
  std::string hostport = (scheme_end == std::string::npos)
                           ? uri : uri.substr(scheme_end + 3);
  const auto colon = hostport.rfind(':');
  ASSERT_NE(colon, std::string::npos) << "broker URI must end in :port";
  const std::string host = hostport.substr(0, colon);
  const uint16_t port = static_cast<uint16_t>(std::stoi(hostport.substr(colon + 1)));

  PahoMqttBus::Config cfg;
  cfg.broker_host = host;
  cfg.broker_port = port;
  cfg.use_tls = tls;
  cfg.client_id = "va-test-paho-bus-roundtrip";
  cfg.connect_timeout_sec = 5;

  PahoMqttBus bus(cfg);

  std::atomic<bool> connected{false};
  std::vector<std::vector<uint8_t>> delivered;
  std::mutex deliver_mtx;
  bus.setOnConnectionState([&](bool up) { connected.store(up); });
  bus.setOnMessage([&](const std::string &, const std::vector<uint8_t> & p) {
    std::lock_guard<std::mutex> lk(deliver_mtx);
    delivered.push_back(p);
  });

  bus.start();

  // Wait up to 5s for connect.
  for (int i = 0; i < 50 && !connected.load(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_TRUE(connected.load()) << "broker connect timed out";

  const std::string topic = "va/test/paho_bus/round_trip";
  bus.subscribe(topic);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  const std::vector<uint8_t> payload{0xAA, 0xBB, 0xCC};
  ASSERT_TRUE(bus.publish(topic, payload));

  // Wait up to 2s for the round trip.
  for (int i = 0; i < 20; ++i) {
    {
      std::lock_guard<std::mutex> lk(deliver_mtx);
      if (!delivered.empty()) break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::lock_guard<std::mutex> lk(deliver_mtx);
  ASSERT_EQ(delivered.size(), 1u);
  EXPECT_EQ(delivered[0], payload);

  bus.stop();
}
