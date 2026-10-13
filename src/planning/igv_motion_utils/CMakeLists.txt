cmake_minimum_required(VERSION 3.14)
project(igv_motion_utils)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(igv_motion_utils SHARED
  src/trajectory_resample.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(trajectory_resample_test
    test/trajectory_resample_test.cpp)
  target_link_libraries(trajectory_resample_test
    igv_motion_utils)
  ament_target_dependencies(trajectory_resample_test
    autoware_planning_msgs geometry_msgs)
endif()

ament_auto_package()
