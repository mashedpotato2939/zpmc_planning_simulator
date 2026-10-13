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

#include "vehicle_agent/adapters/ahtfms_ns_adapter.hpp"

#include "vehicle_agent/adapters/ahtfms_ns/codec.hpp"
#include "vehicle_agent/adapters/ahtfms_ns/framing.hpp"

#include <chrono>
#include <utility>

namespace vehicle_agent
{

namespace
{
uint64_t nowMs()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
}
}  // namespace

AhtfmsNsAdapter::AhtfmsNsAdapter(Config cfg) : cfg_(std::move(cfg))
{
  ahtfms_ns::TcpTransport::Config tc;
  tc.host = cfg_.broker_host;
  tc.port = cfg_.broker_port;
  tc.reconnect_min_ms = cfg_.reconnect_min_ms;
  tc.reconnect_max_ms = cfg_.reconnect_max_ms;
  transport_ = std::make_unique<ahtfms_ns::TcpTransport>(std::move(tc));
}

AhtfmsNsAdapter::AhtfmsNsAdapter(Config cfg,
                                 std::unique_ptr<ahtfms_ns::ITransport> transport)
: cfg_(std::move(cfg)), transport_(std::move(transport)) {}

AhtfmsNsAdapter::~AhtfmsNsAdapter() { stop(); }

AdapterCapabilities AhtfmsNsAdapter::capabilities() const
{
  AdapterCapabilities c;
  c.supports_mission = true;
  c.supports_command = true;
  c.supports_telemetry_uplink = true;
  c.supports_ack = true;
  c.supports_estop = true;          // ICD has EMERGENCY_BRAKE stopping_type
  c.priority = 200;                 // prefer over loopback (100)
  return c;
}

void AhtfmsNsAdapter::start(AdapterCallbacks callbacks)
{
  if (started_.exchange(true)) return;
  callbacks_ = std::move(callbacks);

  transport_->setOnBytes([this](const std::vector<uint8_t> & b) { onBytes(b); });
  transport_->setOnConnectionState([this](bool up) { onConnectionState(up); });
  transport_->start();
}

void AhtfmsNsAdapter::stop()
{
  if (!started_.exchange(false)) return;
  transport_->stop();
}

bool AhtfmsNsAdapter::connected() const { return transport_ && transport_->connected(); }

AhtfmsNsAdapter::Counters AhtfmsNsAdapter::counters() const
{
  std::lock_guard<std::mutex> lk(counters_mtx_);
  return counters_;
}

void AhtfmsNsAdapter::onBytes(const std::vector<uint8_t> & bytes)
{
  // Append to the reassembly tail, then drain whole frames.
  std::vector<uint8_t> drain;
  {
    std::lock_guard<std::mutex> lk(rx_mtx_);
    rx_tail_.insert(rx_tail_.end(), bytes.begin(), bytes.end());
    drain.swap(rx_tail_);
  }
  std::size_t pos = 0;
  while (pos < drain.size()) {
    ahtfms_ns::Frame f;
    const auto err = ahtfms_ns::decodeFrame(drain, pos, f, cfg_.max_payload_bytes);
    if (err == ahtfms_ns::FrameError::TRUNCATED_HEADER ||
        err == ahtfms_ns::FrameError::TRUNCATED_PAYLOAD) {
      // Not enough bytes — put the unconsumed tail back and wait for more.
      std::vector<uint8_t> rest(drain.begin() + pos, drain.end());
      std::lock_guard<std::mutex> lk(rx_mtx_);
      // Prepend any chunks that may have arrived between swap and now.
      rest.insert(rest.end(), rx_tail_.begin(), rx_tail_.end());
      rx_tail_ = std::move(rest);
      return;
    }
    if (err == ahtfms_ns::FrameError::BAD_MAGIC ||
        err == ahtfms_ns::FrameError::OVERSIZED_PAYLOAD) {
      // Unrecoverable: skip one byte and rescan. Bumps decode_errors so the
      // diagnostic publisher can surface stream corruption.
      {
        std::lock_guard<std::mutex> lk(counters_mtx_);
        counters_.decode_errors++;
      }
      pos += 1;
      continue;
    }

    {
      std::lock_guard<std::mutex> lk(counters_mtx_);
      counters_.frames_in++;
    }

    ahtfms_ns::DecodeContext ctx;
    ctx.vehicle_sys = cfg_.vehicle_id;
    auto decoded = ahtfms_ns::decode(f, ctx);
    if (!decoded) {
      std::lock_guard<std::mutex> lk(counters_mtx_);
      counters_.unsupported_msg_type++;
      continue;
    }
    // Mission / Command dispatch into the application layer.
    if (auto * m = std::get_if<ir::Mission>(&*decoded)) {
      if (callbacks_.on_mission) callbacks_.on_mission(*m);
    } else if (auto * c = std::get_if<ir::Command>(&*decoded)) {
      if (callbacks_.on_command) callbacks_.on_command(*c);
    } else {
      // DecodedNothing: unsupported sub-topic OR target_sys mismatch. Count
      // target mismatches separately so diagnostics can distinguish them.
      if (!ctx.vehicle_sys.empty() && f.cm.target_sys != ctx.vehicle_sys) {
        std::lock_guard<std::mutex> lk(counters_mtx_);
        counters_.target_mismatches++;
      }
    }
  }
}

void AhtfmsNsAdapter::onConnectionState(bool up)
{
  if (callbacks_.on_connection_state_changed) {
    callbacks_.on_connection_state_changed(name(), up);
  }
  if (up) {
    // ICD §8.8: NS issues a SYNC_REQUEST on (re)start so FMS re-pushes Job /
    // Route etc. Without this an FMS that pushes "on change" will leave NS in
    // the dark after a reconnect.
    auto payload = ahtfms_ns::encodeVehicleKeyEventSyncRequest(nowMs());
    sendFrame(static_cast<uint16_t>(ahtfms_ns::MsgType::VEHICLE_KEY_EVENT_STATUS),
              payload);
  }
}

bool AhtfmsNsAdapter::sendFrame(uint16_t msg_type_id,
                                const std::vector<uint8_t> & payload)
{
  ahtfms_ns::Frame f;
  f.cm.sender_sys = cfg_.vehicle_id;
  f.cm.target_sys = cfg_.fms_sys;
  f.cm.event_dt = ahtfms_ns::formatEventDt(nowMs());
  f.msg.msg_type_id = msg_type_id;
  f.msg.msg_len_q = static_cast<uint32_t>(payload.size());
  f.payload = payload;
  const auto bytes = ahtfms_ns::encodeFrame(f);
  const bool ok = transport_->send(bytes);
  if (ok) {
    std::lock_guard<std::mutex> lk(counters_mtx_);
    counters_.frames_out++;
  }
  return ok;
}

void AhtfmsNsAdapter::publishTelemetry(const ir::Telemetry & t)
{
  sendFrame(static_cast<uint16_t>(ahtfms_ns::MsgType::VEHICLE_BASIC_STATUS),
            ahtfms_ns::encodeVehicleBasicStatus(t));
}

void AhtfmsNsAdapter::publishMissionStatus(const ir::MissionStatus &)
{
  // The ICD has no direct equivalent for ir::MissionStatus — FMS infers it
  // from VehicleBasicStatus + JobEventStatus. Leave as a no-op for Sprint 4.
}

void AhtfmsNsAdapter::publishAck(const ir::Ack & ack)
{
  sendFrame(static_cast<uint16_t>(ahtfms_ns::MsgType::REMOTE_COMMAND_RESPONSE),
            ahtfms_ns::encodeRemoteCommandResponse(ack));
}

}  // namespace vehicle_agent
