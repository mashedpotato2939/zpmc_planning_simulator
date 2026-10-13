cmake_minimum_required(VERSION 3.5)
project(autoware_control_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(msg_files
  "msg/Control.msg"
  "msg/ControlHorizon.msg"
  "msg/Lateral.msg"
  "msg/Longitudinal.msg")

set(srv_files
  "srv/MapCreatorService.srv"
  "srv/TrajCreatorService.srv")

set(msg_dependencies
  builtin_interfaces
  geometry_msgs
  autoware_planning_msgs)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES ${msg_dependencies}
  ADD_LINTER_TESTS)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
