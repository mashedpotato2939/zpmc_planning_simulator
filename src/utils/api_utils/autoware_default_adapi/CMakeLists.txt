cmake_minimum_required(VERSION 3.14)
project(autoware_default_adapi)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp_components REQUIRED)
find_package(launch_testing_ament_cmake QUIET)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Explicitly find missing packages
find_package(tier4_localization_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_system_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(tier4_api_msgs REQUIRED)
find_package(tier4_control_msgs REQUIRED)
find_package(tier4_system_msgs REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/diagnostics.cpp
  src/fail_safe.cpp
  src/heartbeat.cpp
  src/interface.cpp
  src/localization.cpp
  src/motion.cpp
  src/operation_mode.cpp
  src/perception.cpp
  src/planning.cpp
  src/routing.cpp
  src/vehicle.cpp
  src/vehicle_info.cpp
  src/vehicle_door.cpp
  src/utils/localization_conversion.cpp
  src/utils/route_conversion.cpp
  src/compatibility/autoware_state.cpp
)

# Add include directories and link libraries
target_include_directories(${PROJECT_NAME} PUBLIC
  $<TARGET_PROPERTY:tier4_localization_msgs::tier4_localization_msgs__rosidl_generator_cpp,INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:tier4_vehicle_msgs::tier4_vehicle_msgs__rosidl_generator_cpp,INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:geographic_msgs::geographic_msgs__rosidl_generator_cpp,INTERFACE_INCLUDE_DIRECTORIES>
)

target_link_libraries(${PROJECT_NAME}
  tier4_localization_msgs::tier4_localization_msgs__rosidl_generator_cpp
  tier4_vehicle_msgs::tier4_vehicle_msgs__rosidl_generator_cpp
  geographic_msgs::geographic_msgs__rosidl_generator_cpp
  autoware_adapi_v1_msgs::autoware_adapi_v1_msgs__rosidl_generator_cpp
  autoware_planning_msgs::autoware_planning_msgs__rosidl_generator_cpp
  autoware_system_msgs::autoware_system_msgs__rosidl_generator_cpp
  autoware_vehicle_msgs::autoware_vehicle_msgs__rosidl_generator_cpp
  tier4_api_msgs::tier4_api_msgs__rosidl_generator_cpp
  tier4_control_msgs::tier4_control_msgs__rosidl_generator_cpp
  tier4_system_msgs::tier4_system_msgs__rosidl_generator_cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "autoware::default_adapi::AutowareStateNode"
  "autoware::default_adapi::DiagnosticsNode"
  "autoware::default_adapi::FailSafeNode"
  "autoware::default_adapi::HeartbeatNode"
  "autoware::default_adapi::InterfaceNode"
  "autoware::default_adapi::LocalizationNode"
  "autoware::default_adapi::MotionNode"
  "autoware::default_adapi::OperationModeNode"
  "autoware::default_adapi::PerceptionNode"
  "autoware::default_adapi::PlanningNode"
  "autoware::default_adapi::RoutingNode"
  "autoware::default_adapi::VehicleNode"
  "autoware::default_adapi::VehicleInfoNode"
  "autoware::default_adapi::VehicleDoorNode"
)

if(BUILD_TESTING AND launch_testing_ament_cmake_FOUND)
  add_launch_test(test/main.test.py)
endif()

install(
  PROGRAMS script/web_server.py script/guide.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE config launch test)
