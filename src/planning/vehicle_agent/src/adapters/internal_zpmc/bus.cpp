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

#include "vehicle_agent/adapters/internal_zpmc/bus.hpp"

#include <algorithm>
#include <utility>

namespace vehicle_agent::internal_zpmc
{

void FakeMqttBroker::subscribe(const std::string & topic,
                               const std::string & client_id,
                               OnMessage cb)
{
  std::lock_guard<std::mutex> lk(mtx_);
  auto & v = table_[topic];
  // Replace any prior subscription from the same client (idempotent).
  v.erase(std::remove_if(v.begin(), v.end(),
                         [&](const Subscriber & s) { return s.client_id == client_id; }),
          v.end());
  v.push_back({client_id, std::move(cb)});
}

void FakeMqttBroker::unsubscribe(const std::string & topic,
                                 const std::string & client_id)
{
  std::lock_guard<std::mutex> lk(mtx_);
  auto it = table_.find(topic);
  if (it == table_.end()) return;
  auto & v = it->second;
  v.erase(std::remove_if(v.begin(), v.end(),
                         [&](const Subscriber & s) { return s.client_id == client_id; }),
          v.end());
  if (v.empty()) table_.erase(it);
}

std::size_t FakeMqttBroker::publish(const std::string & topic,
                                    const std::vector<uint8_t> & payload)
{
  // Snapshot subscribers under the lock, then deliver outside it — the
  // delivery callbacks may themselves publish or subscribe, which would
  // re-enter and deadlock if we held the broker mutex.
  std::vector<Subscriber> subs;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = table_.find(topic);
    if (it == table_.end()) return 0;
    subs = it->second;
  }
  for (const auto & s : subs) {
    if (s.cb) s.cb(topic, payload);
  }
  return subs.size();
}

std::size_t FakeMqttBroker::subscriberCount(const std::string & topic) const
{
  std::lock_guard<std::mutex> lk(mtx_);
  auto it = table_.find(topic);
  return it == table_.end() ? 0u : it->second.size();
}

FakeMqttBus::FakeMqttBus(FakeMqttBroker & broker, std::string client_id)
: broker_(broker), client_id_(std::move(client_id))
{
}

FakeMqttBus::~FakeMqttBus()
{
  stop();
}

void FakeMqttBus::start()
{
  connect();
}

void FakeMqttBus::stop()
{
  std::vector<std::string> subs;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    subs = subscriptions_;
    subscriptions_.clear();
  }
  for (const auto & t : subs) broker_.unsubscribe(t, client_id_);
  disconnect();
}

void FakeMqttBus::connect()
{
  OnConnectionState cb;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (connected_) return;
    connected_ = true;
    cb = on_state_;
  }
  if (cb) cb(true);
}

void FakeMqttBus::disconnect()
{
  OnConnectionState cb;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!connected_) return;
    connected_ = false;
    cb = on_state_;
  }
  if (cb) cb(false);
}

bool FakeMqttBus::publish(const std::string & topic,
                          const std::vector<uint8_t> & payload,
                          uint8_t /*qos*/)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    ++pub_attempted_;
    if (!connected_) return false;
    ++pub_accepted_;
  }
  broker_.publish(topic, payload);
  return true;
}

void FakeMqttBus::subscribe(const std::string & topic)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (std::find(subscriptions_.begin(), subscriptions_.end(), topic) ==
        subscriptions_.end()) {
      subscriptions_.push_back(topic);
    }
  }
  broker_.subscribe(topic, client_id_,
                    [this](const std::string & t, const std::vector<uint8_t> & p) {
                      deliver(t, p);
                    });
}

void FakeMqttBus::unsubscribe(const std::string & topic)
{
  {
    std::lock_guard<std::mutex> lk(mtx_);
    subscriptions_.erase(std::remove(subscriptions_.begin(), subscriptions_.end(),
                                     topic),
                         subscriptions_.end());
  }
  broker_.unsubscribe(topic, client_id_);
}

void FakeMqttBus::deliver(const std::string & topic,
                          const std::vector<uint8_t> & payload)
{
  OnMessage cb;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    // Even though the broker only routes to current subscribers, drop the
    // delivery if we've gone offline since the route was set up.
    if (!connected_) return;
    cb = on_message_;
  }
  if (cb) cb(topic, payload);
}

bool FakeMqttBus::connected() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return connected_;
}

void FakeMqttBus::setOnMessage(OnMessage cb)
{
  std::lock_guard<std::mutex> lk(mtx_);
  on_message_ = std::move(cb);
}

void FakeMqttBus::setOnConnectionState(OnConnectionState cb)
{
  std::lock_guard<std::mutex> lk(mtx_);
  on_state_ = std::move(cb);
}

std::size_t FakeMqttBus::publishesAttempted() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return pub_attempted_;
}

std::size_t FakeMqttBus::publishesAccepted() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return pub_accepted_;
}

}  // namespace vehicle_agent::internal_zpmc
