cmake_minimum_required(VERSION 3.16)
project(autoware_planning_validator)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp_components REQUIRED)
find_package(reverse_vehicle_msg REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_auto_add_library(autoware_planning_validator_helpers SHARED
  src/utils.cpp
  src/debug_marker.cpp
)

# planning validator
ament_auto_add_library(autoware_planning_validator_component SHARED
  include/autoware/planning_validator/planning_validator.hpp
  src/planning_validator.cpp
)
target_link_libraries(autoware_planning_validator_component autoware_planning_validator_helpers)
rclcpp_components_register_node(autoware_planning_validator_component
  PLUGIN "autoware::planning_validator::PlanningValidator"
  EXECUTABLE planning_validator_node
)

# invalid trajectory publisher (for debug)
ament_auto_add_library(invalid_trajectory_publisher_node SHARED
  src/invalid_trajectory_publisher/invalid_trajectory_publisher.cpp
)
rclcpp_components_register_node(invalid_trajectory_publisher_node
  PLUGIN "autoware::planning_validator::InvalidTrajectoryPublisherNode"
  EXECUTABLE invalid_trajectory_publisher
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/PlanningValidatorStatus.msg"
  DEPENDENCIES builtin_interfaces
)

# to use a message defined in the same package
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
    rosidl_target_interfaces(autoware_planning_validator_component
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    rosidl_get_typesupport_target(
            cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(autoware_planning_validator_component "${cpp_typesupport_target}")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  ament_add_ros_isolated_gtest(test_autoware_planning_validator
    test/src/test_main.cpp
    test/src/test_planning_validator_functions.cpp
    test/src/test_planning_validator_helper.cpp
    test/src/test_planning_validator_pubsub.cpp
    test/src/test_planning_validator_node_interface.cpp

  )
  ament_target_dependencies(test_autoware_planning_validator
    rclcpp
    autoware_planning_msgs
    "reverse_vehicle_msg"
  )
  target_link_libraries(test_autoware_planning_validator
  autoware_planning_validator_component
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
