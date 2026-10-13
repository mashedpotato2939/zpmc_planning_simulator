cmake_minimum_required(VERSION 3.14)
project(autoware_geography_utils)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# GeographicLib
find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES NAMES Geographic)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/height.cpp
  src/projection.cpp
  src/lanelet2_projector.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${GeographicLib_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  file(GLOB_RECURSE test_files test/*.cpp)

  ament_add_ros_isolated_gtest(test_${PROJECT_NAME} ${test_files})

  target_link_libraries(test_${PROJECT_NAME}
  ${PROJECT_NAME}
  )
endif()

ament_auto_package()
