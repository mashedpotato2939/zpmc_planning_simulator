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

#ifndef VEHICLE_AGENT__ADAPTERS__AHTFMS_NS__FRAMING_HPP_
#define VEHICLE_AGENT__ADAPTERS__AHTFMS_NS__FRAMING_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// Byte-exact AHTFMS-NS ICD v1.1.1 framing (§4.3 CM Header, §4.4 Msg Header).
//
// Wire format (one frame):
//
//   CM Header (45 bytes, all CHAR[]):
//     magic_id[4]    "FMS1" (0x46 0x53 0x4D 0x31)
//     sender_sys[9]  null-padded if shorter
//     target_sys[9]  null-padded if shorter
//     event_dt[23]   "DD-MM-YYYYTHH:MM:SS.zzz"
//
//   Msg Header (6 bytes, little-endian per ICD):
//     msg_type_id    UINT16
//     msg_len_q      UINT32  (length of PB payload that follows)
//
//   PB payload (msg_len_q bytes, opaque)
//
// We treat the PB payload as opaque bytes at this layer. Decoding into
// application messages lives in `codec.hpp`. The ICD allows batching multiple
// (Msg Header + PB) pairs after one CM Header, but no batch length field is
// specified in v1.1.1 — Sprint 4 emits one body per CM Header (matching what
// the spec shows in `CM Header | Msg Header | PB Data`).

namespace vehicle_agent::ahtfms_ns
{

inline constexpr std::size_t kMagicSize = 4;
inline constexpr std::size_t kSenderSize = 9;
inline constexpr std::size_t kTargetSize = 9;
inline constexpr std::size_t kEventDtSize = 23;
inline constexpr std::size_t kCmHeaderSize =
  kMagicSize + kSenderSize + kTargetSize + kEventDtSize;   // 45
inline constexpr std::size_t kMsgHeaderSize = 6;           // u16 + u32
inline constexpr std::size_t kFrameHeaderSize = kCmHeaderSize + kMsgHeaderSize;  // 51
inline constexpr char kMagic[5] = "FMS1";   // includes trailing NUL for safety

// Message type IDs per ICD §5, §6, §7.
enum class MsgType : uint16_t
{
  // 1xxx — Commands (FMS → NS)
  REMOTE_COMMAND = 1001,

  // 2xxx — Command Responses (NS → FMS)
  REMOTE_COMMAND_RESPONSE = 2001,

  // 3xxx — Notify (FMS → NS)
  JOB_NOTIFY = 3001,
  ROUTE_NOTIFY = 3002,
  STOP_POINT_NOTIFY = 3003,
  RESTRICTED_AREA_NOTIFY = 3004,
  VESSEL_INFO_NOTIFY = 3005,
  QUAY_CRANE_INFO_NOTIFY = 3006,
  YARD_CRANE_INFO_NOTIFY = 3007,
  CONTAINER_INFO_NOTIFY = 3008,
  CONFIG_NOTIFY = 3010,

  // 4xxx — Status (NS → FMS)
  VEHICLE_KEY_EVENT_STATUS = 4001,
  VEHICLE_BASIC_STATUS = 4002,
  VEHICLE_TRAJECTORY_STATUS = 4003,
};

struct CmHeader
{
  std::string sender_sys;   // ≤ 9 chars, null-padded on wire
  std::string target_sys;   // ≤ 9 chars, null-padded on wire
  std::string event_dt;     // "DD-MM-YYYYTHH:MM:SS.zzz", padded to 23 chars
};

struct MsgHeader
{
  uint16_t msg_type_id{0};
  uint32_t msg_len_q{0};
};

struct Frame
{
  CmHeader cm;
  MsgHeader msg;
  std::vector<uint8_t> payload;   // length == msg.msg_len_q
};

// Reasons a frame might be rejected. Surfaced for diagnostics + tests.
enum class FrameError : uint8_t
{
  OK = 0,
  TRUNCATED_HEADER,     // fewer than kFrameHeaderSize bytes available
  BAD_MAGIC,            // first 4 bytes != "FMS1"
  TRUNCATED_PAYLOAD,    // header says msg_len_q but payload short
  OVERSIZED_PAYLOAD,    // msg_len_q exceeds configured max
};

// Encode a frame into bytes. CHAR[] fields are null-padded if shorter than the
// fixed slot; truncated if longer (over-length is a programming error and is
// caught with an exception). Multi-byte ints are little-endian.
std::vector<uint8_t> encodeFrame(const Frame & f);

// Decode one frame from `bytes` starting at offset `pos`. On success, advances
// `pos` past the consumed bytes and returns FrameError::OK with the frame
// filled in. On failure, `pos` is left unchanged and `out` is unspecified.
//
// `max_payload` is the hard cap accepted before declaring OVERSIZED_PAYLOAD.
// A value of 0 means "no cap"; production passes a sane bound (e.g. 1 MiB).
FrameError decodeFrame(const std::vector<uint8_t> & bytes, std::size_t & pos,
                       Frame & out, uint32_t max_payload = 1u << 20);

// Format a Unix epoch millisecond timestamp as the ICD event_dt string
// (UTC, "DD-MM-YYYYTHH:MM:SS.zzz", 23 chars). Used by the adapter when
// composing outgoing frames.
std::string formatEventDt(uint64_t epoch_ms);

// Inverse of formatEventDt. Returns nullopt if the input is malformed.
// Lenient: trailing nulls (from CHAR[] padding) are tolerated.
std::optional<uint64_t> parseEventDt(const std::string & s);

}  // namespace vehicle_agent::ahtfms_ns
#endif  // VEHICLE_AGENT__ADAPTERS__AHTFMS_NS__FRAMING_HPP_
