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

#include "vehicle_agent/security/hmac_signer.hpp"

#include <openssl/hmac.h>

#include <array>
#include <sstream>
#include <stdexcept>

namespace vehicle_agent
{

namespace
{
std::string to_hex(const unsigned char * buf, size_t len)
{
  static const char digits[] = "0123456789abcdef";
  std::string out;
  out.resize(len * 2);
  for (size_t i = 0; i < len; ++i) {
    out[2 * i] = digits[(buf[i] >> 4) & 0xF];
    out[2 * i + 1] = digits[buf[i] & 0xF];
  }
  return out;
}

std::string canonical(const std::string & payload, uint64_t ts_ms, const std::string & nonce)
{
  std::ostringstream os;
  os << payload << '\n' << ts_ms << '\n' << nonce;
  return os.str();
}

// Constant-time equality on equal-length strings. Returns 0 if equal.
int const_time_neq(const std::string & a, const std::string & b)
{
  if (a.size() != b.size()) return 1;
  unsigned char diff = 0;
  for (size_t i = 0; i < a.size(); ++i) {
    diff |= static_cast<unsigned char>(a[i]) ^ static_cast<unsigned char>(b[i]);
  }
  return diff;
}
}  // namespace

HmacSigner::HmacSigner(std::string key) : key_(std::move(key))
{
  if (key_.empty()) {
    throw std::invalid_argument("HmacSigner: key must be non-empty");
  }
}

std::string HmacSigner::sign(const std::string & payload,
                             uint64_t timestamp_ms,
                             const std::string & nonce) const
{
  const auto msg = canonical(payload, timestamp_ms, nonce);
  std::array<unsigned char, EVP_MAX_MD_SIZE> mac{};
  unsigned int mac_len = 0;
  const auto * res = HMAC(EVP_sha256(),
                          key_.data(), static_cast<int>(key_.size()),
                          reinterpret_cast<const unsigned char *>(msg.data()),
                          msg.size(),
                          mac.data(), &mac_len);
  if (!res || mac_len == 0) {
    throw std::runtime_error("HmacSigner: HMAC failed");
  }
  return to_hex(mac.data(), mac_len);
}

bool HmacSigner::verify(const std::string & payload,
                        uint64_t timestamp_ms,
                        const std::string & nonce,
                        const std::string & signature_hex) const
{
  const auto expected = sign(payload, timestamp_ms, nonce);
  return const_time_neq(expected, signature_hex) == 0;
}

}  // namespace vehicle_agent
