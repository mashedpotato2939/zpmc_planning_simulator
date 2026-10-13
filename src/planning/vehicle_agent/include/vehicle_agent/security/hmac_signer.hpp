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

#ifndef VEHICLE_AGENT__SECURITY__HMAC_SIGNER_HPP_
#define VEHICLE_AGENT__SECURITY__HMAC_SIGNER_HPP_

#include <cstdint>
#include <string>

namespace vehicle_agent
{

// HMAC-SHA256 signer / verifier for outbound and inbound IR JSON. The
// canonical input is "<payload>\n<timestamp_ms>\n<nonce>" — newline
// delimiters bind the three fields together so a tampered timestamp or
// nonce produces a different MAC.
//
// Signatures are returned as lowercase hex (64 chars). Constant-time compare
// on verify so an attacker cannot probe the MAC byte-by-byte.
class HmacSigner
{
public:
  // key must be non-empty; empty key throws std::invalid_argument.
  explicit HmacSigner(std::string key);

  // Returns the lowercase-hex SHA-256 HMAC of the canonical input.
  std::string sign(const std::string & payload,
                   uint64_t timestamp_ms,
                   const std::string & nonce) const;

  // True iff `signature_hex` matches sign(payload, timestamp_ms, nonce).
  bool verify(const std::string & payload,
              uint64_t timestamp_ms,
              const std::string & nonce,
              const std::string & signature_hex) const;

private:
  std::string key_;
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__SECURITY__HMAC_SIGNER_HPP_
