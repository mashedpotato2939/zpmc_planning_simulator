cmake_minimum_required(VERSION 3.14)
project(autoware_kalman_filter)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/kalman_filter.cpp
  src/time_delay_kalman_filter.cpp
  include/autoware/kalman_filter/kalman_filter.hpp
  include/autoware/kalman_filter/time_delay_kalman_filter.hpp
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB_RECURSE test_files test/*.cpp)
  ament_add_gtest(test_${PROJECT_NAME} ${test_files})

  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_auto_package()
