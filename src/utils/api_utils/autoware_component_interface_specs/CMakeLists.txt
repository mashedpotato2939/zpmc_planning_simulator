cmake_minimum_required(VERSION 3.14)
project(autoware_component_interface_specs)

# Suppress CMake Python policy warning
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# This is a header-only library - install headers
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
  FILES_MATCHING PATTERN "*.hpp"
)

# Export the library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(gtest_${PROJECT_NAME}
    test/gtest_main.cpp
    test/test_planning.cpp
    test/test_control.cpp
    test/test_localization.cpp
    test/test_system.cpp
    test/test_map.cpp
    test/test_perception.cpp
    test/test_vehicle.cpp
  )
  target_include_directories(gtest_${PROJECT_NAME} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(gtest_${PROJECT_NAME} ${PROJECT_NAME})
  ament_target_dependencies(gtest_${PROJECT_NAME}
    autoware_adapi_v1_msgs
    autoware_perception_msgs
    autoware_planning_msgs
    autoware_vehicle_msgs
    nav_msgs
    rclcpp
    tier4_control_msgs
    tier4_localization_msgs
    tier4_map_msgs
    tier4_planning_msgs
    tier4_system_msgs
    tier4_vehicle_msgs
  )
endif()

ament_auto_package()
