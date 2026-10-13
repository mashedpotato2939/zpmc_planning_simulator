cmake_minimum_required(VERSION 3.14)
project(vehicle_door_simulator)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(dummy_doors
  src/dummy_doors.cpp
)

ament_auto_package(INSTALL_TO_SHARE launch)
