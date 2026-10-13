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

#ifndef VEHICLE_AGENT__APP__STRUCTURED_LOG_HPP_
#define VEHICLE_AGENT__APP__STRUCTURED_LOG_HPP_

#include <rclcpp/rclcpp.hpp>

// Structured-logging helpers. Every line carries a mission_id (or "-" if
// none yet) so grepping a single mission's lifecycle is one command. The
// stream form lets us prepend a tag without forcing every call site to use
// printf-style formatting.
//
// Usage:
//   VA_LOG_INFO(logger, mission_id, "received seq=%lu", seq);
//   VA_LOG_WARN(logger, "-", "adapter %s reconnect failed", name.c_str());

#define VA_LOG_INFO(logger, mission_id, fmt, ...) \
  RCLCPP_INFO((logger), "[mid=%s] " fmt, \
              ((mission_id).empty() ? "-" : (mission_id).c_str()), ##__VA_ARGS__)

#define VA_LOG_WARN(logger, mission_id, fmt, ...) \
  RCLCPP_WARN((logger), "[mid=%s] " fmt, \
              ((mission_id).empty() ? "-" : (mission_id).c_str()), ##__VA_ARGS__)

#define VA_LOG_ERROR(logger, mission_id, fmt, ...) \
  RCLCPP_ERROR((logger), "[mid=%s] " fmt, \
               ((mission_id).empty() ? "-" : (mission_id).c_str()), ##__VA_ARGS__)

#define VA_LOG_DEBUG(logger, mission_id, fmt, ...) \
  RCLCPP_DEBUG((logger), "[mid=%s] " fmt, \
               ((mission_id).empty() ? "-" : (mission_id).c_str()), ##__VA_ARGS__)

#endif  // VEHICLE_AGENT__APP__STRUCTURED_LOG_HPP_
