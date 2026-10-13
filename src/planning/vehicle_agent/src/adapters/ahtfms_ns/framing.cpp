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

#include "vehicle_agent/adapters/ahtfms_ns/framing.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <stdexcept>

namespace vehicle_agent::ahtfms_ns
{

namespace
{
void writeChars(std::vector<uint8_t> & buf, const std::string & s, std::size_t slot)
{
  if (s.size() > slot) {
    throw std::invalid_argument(
      "ahtfms_ns: field '" + s + "' exceeds slot size " + std::to_string(slot));
  }
  buf.insert(buf.end(), s.begin(), s.end());
  buf.insert(buf.end(), slot - s.size(), uint8_t{0});
}

std::string readChars(const uint8_t * p, std::size_t slot)
{
  // CHAR[] is null-terminated when shorter than its slot — strip trailing NULs
  // to recover the logical string.
  std::size_t n = slot;
  while (n > 0 && p[n - 1] == 0) --n;
  return std::string(reinterpret_cast<const char *>(p), n);
}

void writeU16Le(std::vector<uint8_t> & buf, uint16_t v)
{
  buf.push_back(static_cast<uint8_t>(v & 0xFF));
  buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

void writeU32Le(std::vector<uint8_t> & buf, uint32_t v)
{
  buf.push_back(static_cast<uint8_t>(v & 0xFF));
  buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
  buf.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
  buf.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
}

uint16_t readU16Le(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t readU32Le(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}
}  // namespace

std::vector<uint8_t> encodeFrame(const Frame & f)
{
  if (f.payload.size() != f.msg.msg_len_q) {
    throw std::invalid_argument("ahtfms_ns: payload size != msg_len_q");
  }
  std::vector<uint8_t> out;
  out.reserve(kFrameHeaderSize + f.payload.size());

  // magic_id[4] — "FMS1"
  out.insert(out.end(), kMagic, kMagic + kMagicSize);
  // sender_sys[9], target_sys[9], event_dt[23]
  writeChars(out, f.cm.sender_sys, kSenderSize);
  writeChars(out, f.cm.target_sys, kTargetSize);
  writeChars(out, f.cm.event_dt, kEventDtSize);
  // Msg Header: u16 type, u32 len, little-endian
  writeU16Le(out, f.msg.msg_type_id);
  writeU32Le(out, f.msg.msg_len_q);
  // Payload bytes (opaque to this layer)
  out.insert(out.end(), f.payload.begin(), f.payload.end());
  return out;
}

FrameError decodeFrame(const std::vector<uint8_t> & bytes, std::size_t & pos,
                       Frame & out, uint32_t max_payload)
{
  if (pos + kFrameHeaderSize > bytes.size()) {
    return FrameError::TRUNCATED_HEADER;
  }
  const uint8_t * p = bytes.data() + pos;
  // magic
  if (std::memcmp(p, kMagic, kMagicSize) != 0) {
    return FrameError::BAD_MAGIC;
  }
  CmHeader cm;
  cm.sender_sys = readChars(p + kMagicSize, kSenderSize);
  cm.target_sys = readChars(p + kMagicSize + kSenderSize, kTargetSize);
  cm.event_dt = readChars(p + kMagicSize + kSenderSize + kTargetSize, kEventDtSize);

  MsgHeader msg;
  const uint8_t * mh = p + kCmHeaderSize;
  msg.msg_type_id = readU16Le(mh);
  msg.msg_len_q = readU32Le(mh + 2);

  if (max_payload > 0 && msg.msg_len_q > max_payload) {
    return FrameError::OVERSIZED_PAYLOAD;
  }
  if (pos + kFrameHeaderSize + msg.msg_len_q > bytes.size()) {
    return FrameError::TRUNCATED_PAYLOAD;
  }
  std::vector<uint8_t> payload(
    bytes.begin() + pos + kFrameHeaderSize,
    bytes.begin() + pos + kFrameHeaderSize + msg.msg_len_q);

  out.cm = std::move(cm);
  out.msg = msg;
  out.payload = std::move(payload);
  pos += kFrameHeaderSize + msg.msg_len_q;
  return FrameError::OK;
}

std::string formatEventDt(uint64_t epoch_ms)
{
  const std::time_t secs = static_cast<std::time_t>(epoch_ms / 1000);
  const uint32_t msec = static_cast<uint32_t>(epoch_ms % 1000);
  std::tm tm{};
  gmtime_r(&secs, &tm);
  char buf[80];
  // ICD: "DD-MM-YYYYTHH:MM:SS.zzz" — UTC chosen for consistency.
  std::snprintf(buf, sizeof(buf), "%02d-%02d-%04dT%02d:%02d:%02d.%03u",
                tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 1900,
                tm.tm_hour, tm.tm_min, tm.tm_sec, msec);
  return std::string(buf);
}

std::optional<uint64_t> parseEventDt(const std::string & s_in)
{
  // Trim any trailing NULs that may have come from the on-wire CHAR[23] slot.
  std::string s = s_in;
  while (!s.empty() && s.back() == '\0') s.pop_back();
  if (s.size() != kEventDtSize) return std::nullopt;
  int dd, mm, yy, h, mi, se, ms;
  if (std::sscanf(s.c_str(), "%2d-%2d-%4dT%2d:%2d:%2d.%3d",
                  &dd, &mm, &yy, &h, &mi, &se, &ms) != 7) {
    return std::nullopt;
  }
  std::tm tm{};
  tm.tm_mday = dd;
  tm.tm_mon = mm - 1;
  tm.tm_year = yy - 1900;
  tm.tm_hour = h;
  tm.tm_min = mi;
  tm.tm_sec = se;
  // timegm is GNU/Linux. Available on the Galactic build target (libc6).
  const std::time_t t = timegm(&tm);
  if (t == static_cast<std::time_t>(-1)) return std::nullopt;
  return static_cast<uint64_t>(t) * 1000ULL + static_cast<uint64_t>(ms);
}

}  // namespace vehicle_agent::ahtfms_ns
