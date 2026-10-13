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

#include "vehicle_agent/security/mtls_context.hpp"

#include <filesystem>
#include <sstream>

namespace vehicle_agent
{

MtlsContext::MtlsContext(MtlsConfig cfg) : cfg_(std::move(cfg))
{
  validate();
}

void MtlsContext::validate()
{
  std::ostringstream errs;
  auto check = [&errs](const std::string & path, const char * label) -> bool {
    if (path.empty()) {
      errs << label << " path is empty; ";
      return false;
    }
    std::error_code ec;
    if (!std::filesystem::exists(path, ec) || ec) {
      errs << label << " not found: " << path << "; ";
      return false;
    }
    return true;
  };
  const bool ca_ok = check(cfg_.ca_path, "ca");
  const bool cert_ok = check(cfg_.cert_path, "cert");
  const bool key_ok = check(cfg_.key_path, "key");
  ready_ = ca_ok && cert_ok && key_ok;
  if (!ready_) last_error_ = errs.str();
}

}  // namespace vehicle_agent
