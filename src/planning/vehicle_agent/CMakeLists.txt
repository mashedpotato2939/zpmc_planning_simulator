cmake_minimum_required(VERSION 3.14)
project(vehicle_agent)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(igv_planning_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(eclipse-paho-mqtt-c QUIET)
find_package(PahoMqttCpp QUIET)

set(VEHICLE_AGENT_HAS_PAHO_MQTT FALSE)
if(eclipse-paho-mqtt-c_FOUND AND PahoMqttCpp_FOUND)
  set(VEHICLE_AGENT_PAHO_LINK_TARGETS)
  if(TARGET PahoMqttCpp::paho-mqttpp3-shared)
    list(APPEND VEHICLE_AGENT_PAHO_LINK_TARGETS PahoMqttCpp::paho-mqttpp3-shared)
  elseif(TARGET PahoMqttCpp::paho-mqttpp3)
    list(APPEND VEHICLE_AGENT_PAHO_LINK_TARGETS PahoMqttCpp::paho-mqttpp3)
  endif()

  if(TARGET eclipse-paho-mqtt-c::paho-mqtt3as)
    list(APPEND VEHICLE_AGENT_PAHO_LINK_TARGETS eclipse-paho-mqtt-c::paho-mqtt3as)
  endif()

  if(VEHICLE_AGENT_PAHO_LINK_TARGETS)
    set(VEHICLE_AGENT_HAS_PAHO_MQTT TRUE)
  else()
    message(WARNING "Paho MQTT packages were found, but no known CMake targets were exported. Building vehicle_agent without internal_zpmc MQTT support.")
  endif()
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core library: IR types are header-only; this holds adapters, app, ros_bridge.
set(VEHICLE_AGENT_CORE_SOURCES
  src/app/event_bus.cpp
  src/app/mission_state_machine.cpp
  src/app/estop_channel.cpp
  src/adapters/adapter_multiplexer.cpp
  src/adapters/scenario_loopback_adapter.cpp
  src/adapters/ahtfms_ns/framing.cpp
  src/adapters/ahtfms_ns/codec.cpp
  src/adapters/ahtfms_ns/transport.cpp
  src/adapters/ahtfms_ns_adapter.cpp
  src/adapters/internal_zpmc/bus.cpp
  src/adapters/internal_zpmc_adapter.cpp
  src/adapters/ros_injector_adapter.cpp
  src/security/hmac_signer.cpp
  src/security/nonce_lru.cpp
  src/security/clock_skew_window.cpp
  src/security/mtls_context.cpp
  src/storage/offline_queue.cpp
  src/ros_bridge/ros_publisher.cpp
  src/ros_bridge/ros_subscriber.cpp
  src/ros_bridge/diagnostic_publisher.cpp
  src/ros_bridge/route_bridge.cpp
)

if(VEHICLE_AGENT_HAS_PAHO_MQTT)
  list(APPEND VEHICLE_AGENT_CORE_SOURCES
    src/adapters/internal_zpmc/paho_bus.cpp
  )
endif()

ament_auto_add_library(vehicle_agent_core SHARED
  ${VEHICLE_AGENT_CORE_SOURCES}
)
target_include_directories(vehicle_agent_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(vehicle_agent_core
  nlohmann_json::nlohmann_json
  yaml-cpp
  SQLite::SQLite3
  OpenSSL::Crypto
)
if(VEHICLE_AGENT_HAS_PAHO_MQTT)
  target_link_libraries(vehicle_agent_core ${VEHICLE_AGENT_PAHO_LINK_TARGETS})
  target_compile_definitions(vehicle_agent_core PRIVATE VEHICLE_AGENT_HAS_PAHO_MQTT)
endif()
ament_target_dependencies(vehicle_agent_core igv_planning_msgs diagnostic_msgs)

# Composable node
ament_auto_add_library(vehicle_agent_node SHARED
  src/vehicle_agent_node.cpp
)
target_link_libraries(vehicle_agent_node vehicle_agent_core)
rclcpp_components_register_node(vehicle_agent_node
  PLUGIN "vehicle_agent::VehicleAgentNode"
  EXECUTABLE vehicle_agent
)

# Standalone executable for the IR → FMS topic bridge. Registered as a
# composable node so it can also be loaded into a shared rclcpp_components
# container when node count matters.
rclcpp_components_register_node(vehicle_agent_core
  PLUGIN "vehicle_agent::RouteBridgeNode"
  EXECUTABLE va_route_bridge
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_event_bus test/unit/test_event_bus.cpp)
  target_link_libraries(test_event_bus vehicle_agent_core)

  ament_add_gtest(test_mission_state_machine test/unit/test_mission_state_machine.cpp)
  target_link_libraries(test_mission_state_machine vehicle_agent_core)

  ament_add_gtest(test_scenario_loopback test/unit/test_scenario_loopback.cpp)
  target_link_libraries(test_scenario_loopback vehicle_agent_core)

  ament_add_gtest(test_offline_queue test/unit/test_offline_queue.cpp)
  target_link_libraries(test_offline_queue vehicle_agent_core)

  ament_add_gtest(test_estop_channel test/unit/test_estop_channel.cpp)
  target_link_libraries(test_estop_channel vehicle_agent_core)

  ament_add_gtest(test_loopback_replay test/integration/test_loopback_replay.cpp)
  target_link_libraries(test_loopback_replay vehicle_agent_core)

  ament_add_gtest(test_hmac_signer test/unit/test_hmac_signer.cpp)
  target_link_libraries(test_hmac_signer vehicle_agent_core)

  ament_add_gtest(test_nonce_lru test/unit/test_nonce_lru.cpp)
  target_link_libraries(test_nonce_lru vehicle_agent_core)

  ament_add_gtest(test_clock_skew_window test/unit/test_clock_skew_window.cpp)
  target_link_libraries(test_clock_skew_window vehicle_agent_core)

  ament_add_gtest(test_ahtfms_ns_codec test/unit/test_ahtfms_ns_codec.cpp)
  target_link_libraries(test_ahtfms_ns_codec vehicle_agent_core)

  ament_add_gtest(test_ahtfms_ns_flow test/integration/test_ahtfms_ns_flow.cpp)
  target_link_libraries(test_ahtfms_ns_flow vehicle_agent_core)

  ament_add_gtest(test_multiplexer_dual test/integration/test_multiplexer_dual.cpp)
  target_link_libraries(test_multiplexer_dual vehicle_agent_core)

  ament_add_gtest(test_route_bridge_mapping test/unit/test_route_bridge_mapping.cpp)
  target_link_libraries(test_route_bridge_mapping vehicle_agent_core)

  ament_add_gtest(test_route_bridge_window test/unit/test_route_bridge_window.cpp)
  target_link_libraries(test_route_bridge_window vehicle_agent_core)

  ament_add_gtest(test_route_bridge_flow test/integration/test_route_bridge_flow.cpp)
  target_link_libraries(test_route_bridge_flow vehicle_agent_core)

  ament_add_gtest(test_ros_injector_adapter test/unit/test_ros_injector_adapter.cpp)
  target_link_libraries(test_ros_injector_adapter vehicle_agent_core)

  ament_add_gtest(test_paho_bus test/unit/test_paho_bus.cpp)
  target_link_libraries(test_paho_bus vehicle_agent_core)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
    scenarios
)
