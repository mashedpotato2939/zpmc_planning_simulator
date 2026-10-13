cmake_minimum_required(VERSION 3.14)
project(autoware_interpolation)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_auto_add_library(autoware_interpolation SHARED
  src/linear_interpolation.cpp
  src/spline_interpolation.cpp
  src/spline_interpolation_points_2d.cpp
  src/spherical_linear_interpolation.cpp
)

ament_target_dependencies(autoware_interpolation
  autoware_universe_utils
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  file(GLOB_RECURSE test_files test/**/*.cpp)

  ament_add_ros_isolated_gtest(test_interpolation ${test_files})

  target_link_libraries(test_interpolation
    autoware_interpolation
  )
endif()

ament_auto_package()
