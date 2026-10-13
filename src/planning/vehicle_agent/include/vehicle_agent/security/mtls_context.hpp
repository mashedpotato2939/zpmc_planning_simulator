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

#ifndef VEHICLE_AGENT__SECURITY__MTLS_CONTEXT_HPP_
#define VEHICLE_AGENT__SECURITY__MTLS_CONTEXT_HPP_

#include <string>

namespace vehicle_agent
{

// Holds mTLS material paths for an MQTT adapter. Sprint 3 only owns the
// configuration shape and file-existence validation; Sprint 5 maps these
// into paho::mqtt::ssl_options when the InternalZpmcAdapter goes online.
struct MtlsConfig
{
  std::string ca_path;          // CA bundle to verify broker
  std::string cert_path;        // client certificate (PEM)
  std::string key_path;         // client private key (PEM)
  std::string key_password;     // optional, empty if key is unencrypted
  bool verify_server{true};
  bool enable_alpn{false};
};

class MtlsContext
{
public:
  explicit MtlsContext(MtlsConfig cfg);

  // True iff all configured paths exist and are readable.
  bool ready() const { return ready_; }

  // Empty when ready() is true, otherwise the reason validation failed.
  const std::string & lastError() const { return last_error_; }

  const MtlsConfig & config() const { return cfg_; }

private:
  void validate();

  MtlsConfig cfg_;
  bool ready_{false};
  std::string last_error_;
};

}  // namespace vehicle_agent
#endif  // VEHICLE_AGENT__SECURITY__MTLS_CONTEXT_HPP_
