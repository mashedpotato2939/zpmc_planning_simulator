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

#ifndef VEHICLE_AGENT__ADAPTERS__AHTFMS_NS__TRANSPORT_HPP_
#define VEHICLE_AGENT__ADAPTERS__AHTFMS_NS__TRANSPORT_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Byte-level transport between NS and CM. The ICD says this is a single TCP
// connection initiated by NS (§4.1). We hide it behind an interface so the
// codec/adapter unit tests can drive a deterministic in-memory pipe instead
// of opening real sockets.

namespace vehicle_agent::ahtfms_ns
{

class ITransport
{
public:
  virtual ~ITransport() = default;

  // Open the connection (or arrange for it to be opened in the background).
  // Implementations should be tolerant of being called more than once.
  virtual void start() = 0;
  // Tear down. Must be safe to call from any thread, including from inside an
  // on_bytes callback.
  virtual void stop() = 0;

  // Push raw bytes to the CM. Returns false if the transport is currently
  // disconnected; the adapter is expected to fall back to OfflineQueue.
  virtual bool send(const std::vector<uint8_t> & bytes) = 0;

  // Snapshot of connection state.
  virtual bool connected() const = 0;

  // Inbound bytes from the CM. The transport implementation is responsible
  // for delivering exact byte stream chunks; the adapter does the framing-
  // level reassembly via ahtfms_ns::decodeFrame.
  virtual void setOnBytes(std::function<void(const std::vector<uint8_t> &)> cb) = 0;

  // Connection-state notification. Called on any flip.
  virtual void setOnConnectionState(std::function<void(bool)> cb) = 0;
};

// In-process fake transport for tests. Two paired instances act as the NS end
// and the CM end of a single TCP connection.
class FakeTransport : public ITransport
{
public:
  FakeTransport();
  ~FakeTransport() override;

  // Wire `a` and `b` so bytes written into one are delivered to the other.
  // The pair starts in disconnected state; call connect()/disconnect() to
  // simulate link transitions.
  static void pair(FakeTransport & a, FakeTransport & b);

  void start() override;
  void stop() override;
  bool send(const std::vector<uint8_t> & bytes) override;
  bool connected() const override;
  void setOnBytes(std::function<void(const std::vector<uint8_t> &)> cb) override;
  void setOnConnectionState(std::function<void(bool)> cb) override;

  // Test helpers.
  void connect();      // flips both ends to connected
  void disconnect();   // flips both ends to disconnected
  size_t bytesSent() const;
  void clearBytesSent();

private:
  void deliver(const std::vector<uint8_t> & bytes);
  void notifyState(bool up);

  mutable std::mutex mtx_;
  FakeTransport * peer_{nullptr};
  std::function<void(const std::vector<uint8_t> &)> on_bytes_;
  std::function<void(bool)> on_state_;
  bool connected_{false};
  size_t bytes_sent_{0};
};

// Production TCP transport. Spawns one I/O thread that owns the socket and
// performs the (re)connect/read loop. send() is callable from any thread.
class TcpTransport : public ITransport
{
public:
  struct Config
  {
    std::string host;
    uint16_t port{0};
    // Reconnect backoff (ms) starts at `reconnect_min_ms` and doubles up to
    // `reconnect_max_ms` on consecutive failures.
    uint32_t reconnect_min_ms{500};
    uint32_t reconnect_max_ms{10000};
    // Read buffer size per syscall. Frames larger than this are reassembled
    // across reads.
    uint32_t read_chunk_bytes{4096};
  };

  explicit TcpTransport(Config cfg);
  ~TcpTransport() override;

  void start() override;
  void stop() override;
  bool send(const std::vector<uint8_t> & bytes) override;
  bool connected() const override;
  void setOnBytes(std::function<void(const std::vector<uint8_t> &)> cb) override;
  void setOnConnectionState(std::function<void(bool)> cb) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vehicle_agent::ahtfms_ns
#endif  // VEHICLE_AGENT__ADAPTERS__AHTFMS_NS__TRANSPORT_HPP_
