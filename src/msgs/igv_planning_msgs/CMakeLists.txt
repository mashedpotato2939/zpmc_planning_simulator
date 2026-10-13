cmake_minimum_required(VERSION 3.14)
project(igv_planning_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Existing messages (backward compatible)
  "msg/FMSWaypoint.msg"
  "msg/FMSPath.msg"
  "msg/FMSTrajectory.msg"
  "msg/FMSStatus.msg"
  # Protocol messages (ICD-aligned)
  "msg/SegmentWaypoint.msg"
  "msg/FMSPathSegment.msg"
  "msg/FMSRouteWindow.msg"
  "msg/FMSStopPoint.msg"
  "msg/VehicleStatus.msg"
  "msg/TrajectoryReport.msg"
  "msg/TrajectoryPoint.msg"
  "msg/JobEvent.msg"
  "msg/DetourRequest.msg"
  "msg/DetourArea.msg"
  "msg/AdapterStatus.msg"
  # IR-aligned messages (published by vehicle_agent)
  "msg/IrWaypoint.msg"
  "msg/IrMission.msg"
  "msg/IrMissionStatus.msg"
  "msg/IrCommand.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
