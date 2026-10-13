cmake_minimum_required(VERSION 3.14)
project(autoware_utils)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_auto_add_library(autoware_utils SHARED
  src/autoware_utils.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  file(GLOB_RECURSE test_files test/**/*.cpp)
  ament_add_ros_isolated_gtest(test_autoware_utils ${test_files})
  target_link_libraries(test_autoware_utils
    autoware_utils
  )
endif()

ament_auto_package()
