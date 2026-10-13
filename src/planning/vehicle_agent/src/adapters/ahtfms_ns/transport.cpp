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

#include "vehicle_agent/adapters/ahtfms_ns/transport.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace vehicle_agent::ahtfms_ns
{

// ---------- FakeTransport ----------

FakeTransport::FakeTransport() = default;
FakeTransport::~FakeTransport() { stop(); }

void FakeTransport::pair(FakeTransport & a, FakeTransport & b)
{
  // Set peers without holding both locks (callers haven't started threads).
  a.peer_ = &b;
  b.peer_ = &a;
}

void FakeTransport::start()
{
  // No-op: connect() drives the state transition in tests.
}

void FakeTransport::stop()
{
  std::function<void(bool)> cb;
  bool was_connected = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    was_connected = connected_;
    connected_ = false;
    cb = on_state_;
  }
  if (was_connected && cb) cb(false);
}

bool FakeTransport::send(const std::vector<uint8_t> & bytes)
{
  FakeTransport * peer;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!connected_) return false;
    peer = peer_;
    bytes_sent_ += bytes.size();
  }
  if (peer) peer->deliver(bytes);
  return true;
}

bool FakeTransport::connected() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return connected_;
}

void FakeTransport::setOnBytes(std::function<void(const std::vector<uint8_t> &)> cb)
{
  std::lock_guard<std::mutex> lk(mtx_);
  on_bytes_ = std::move(cb);
}

void FakeTransport::setOnConnectionState(std::function<void(bool)> cb)
{
  std::lock_guard<std::mutex> lk(mtx_);
  on_state_ = std::move(cb);
}

void FakeTransport::connect()
{
  notifyState(true);
  if (peer_) peer_->notifyState(true);
}

void FakeTransport::disconnect()
{
  notifyState(false);
  if (peer_) peer_->notifyState(false);
}

void FakeTransport::deliver(const std::vector<uint8_t> & bytes)
{
  std::function<void(const std::vector<uint8_t> &)> cb;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cb = on_bytes_;
  }
  if (cb) cb(bytes);
}

void FakeTransport::notifyState(bool up)
{
  std::function<void(bool)> cb;
  bool changed = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (connected_ != up) {
      connected_ = up;
      changed = true;
      cb = on_state_;
    }
  }
  if (changed && cb) cb(up);
}

size_t FakeTransport::bytesSent() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return bytes_sent_;
}

void FakeTransport::clearBytesSent()
{
  std::lock_guard<std::mutex> lk(mtx_);
  bytes_sent_ = 0;
}

// ---------- TcpTransport ----------

struct TcpTransport::Impl
{
  Config cfg;
  std::atomic<bool> running{false};
  std::atomic<int> fd{-1};               // -1 = no socket
  std::atomic<bool> connected{false};

  std::thread io_thread;

  std::mutex mtx;
  std::function<void(const std::vector<uint8_t> &)> on_bytes;
  std::function<void(bool)> on_state;
  std::condition_variable wake;          // poked by stop() to break sleeps

  bool openSocket()
  {
    // Resolve host (synchronous; the I/O thread already runs off the main loop).
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo * res = nullptr;
    const std::string port_s = std::to_string(cfg.port);
    if (getaddrinfo(cfg.host.c_str(), port_s.c_str(), &hints, &res) != 0 || !res) {
      return false;
    }
    int s = -1;
    for (addrinfo * p = res; p != nullptr; p = p->ai_next) {
      s = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);
      if (s < 0) continue;
      // TCP_NODELAY: ICD payloads are small and latency-sensitive (suspend,
      // estop). Don't let Nagle stall command responses.
      int yes = 1;
      ::setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
      if (::connect(s, p->ai_addr, p->ai_addrlen) == 0) break;
      ::close(s);
      s = -1;
    }
    freeaddrinfo(res);
    if (s < 0) return false;
    fd.store(s);
    return true;
  }

  void closeSocket()
  {
    const int s = fd.exchange(-1);
    if (s >= 0) ::close(s);
  }

  void notifyState(bool up)
  {
    if (connected.exchange(up) == up) return;   // no change
    std::function<void(bool)> cb;
    {
      std::lock_guard<std::mutex> lk(mtx);
      cb = on_state;
    }
    if (cb) cb(up);
  }

  void readLoop()
  {
    std::vector<uint8_t> buf(cfg.read_chunk_bytes);
    while (running.load() && connected.load()) {
      const int s = fd.load();
      if (s < 0) break;
      const ssize_t n = ::recv(s, buf.data(), buf.size(), 0);
      if (n > 0) {
        std::function<void(const std::vector<uint8_t> &)> cb;
        {
          std::lock_guard<std::mutex> lk(mtx);
          cb = on_bytes;
        }
        if (cb) cb(std::vector<uint8_t>(buf.begin(), buf.begin() + n));
        continue;
      }
      // 0 = peer closed; <0 = error → both drop the connection.
      break;
    }
  }

  void run()
  {
    uint32_t backoff = cfg.reconnect_min_ms;
    while (running.load()) {
      if (openSocket()) {
        notifyState(true);
        backoff = cfg.reconnect_min_ms;   // reset on successful connect
        readLoop();
        notifyState(false);
        closeSocket();
      }
      if (!running.load()) break;
      // Sleep up to `backoff` ms but break out promptly on stop().
      std::unique_lock<std::mutex> lk(mtx);
      wake.wait_for(lk, std::chrono::milliseconds(backoff),
                    [this]() { return !running.load(); });
      backoff = std::min(backoff * 2, cfg.reconnect_max_ms);
    }
    notifyState(false);
    closeSocket();
  }
};

TcpTransport::TcpTransport(Config cfg) : impl_(std::make_unique<Impl>())
{
  impl_->cfg = std::move(cfg);
}

TcpTransport::~TcpTransport() { stop(); }

void TcpTransport::start()
{
  if (impl_->running.exchange(true)) return;
  impl_->io_thread = std::thread([this]() { impl_->run(); });
}

void TcpTransport::stop()
{
  if (!impl_->running.exchange(false)) return;
  {
    std::lock_guard<std::mutex> lk(impl_->mtx);
    impl_->wake.notify_all();
  }
  // Closing the fd kicks blocking recv() out of its syscall.
  const int s = impl_->fd.load();
  if (s >= 0) ::shutdown(s, SHUT_RDWR);
  if (impl_->io_thread.joinable()) impl_->io_thread.join();
  impl_->closeSocket();
}

bool TcpTransport::send(const std::vector<uint8_t> & bytes)
{
  if (!impl_->connected.load()) return false;
  const int s = impl_->fd.load();
  if (s < 0) return false;
  const uint8_t * p = bytes.data();
  size_t remaining = bytes.size();
  while (remaining > 0) {
    const ssize_t n = ::send(s, p, remaining, MSG_NOSIGNAL);
    if (n <= 0) return false;
    p += n;
    remaining -= static_cast<size_t>(n);
  }
  return true;
}

bool TcpTransport::connected() const { return impl_->connected.load(); }

void TcpTransport::setOnBytes(std::function<void(const std::vector<uint8_t> &)> cb)
{
  std::lock_guard<std::mutex> lk(impl_->mtx);
  impl_->on_bytes = std::move(cb);
}

void TcpTransport::setOnConnectionState(std::function<void(bool)> cb)
{
  std::lock_guard<std::mutex> lk(impl_->mtx);
  impl_->on_state = std::move(cb);
}

}  // namespace vehicle_agent::ahtfms_ns
