cmake_minimum_required(VERSION 3.14)
project(autoware_freespace_planning_algorithms)

# Set C++20 standard for better compatibility with GCC 9
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add ROS_DISTRO_GALACTIC macro for conditional compilation
add_definitions(-DROS_DISTRO_GALACTIC)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(python_cmake_module REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autoware_universe_utils REQUIRED)
find_package(autoware_vehicle_info_utils REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_auto_add_library(reeds_shepp SHARED
  src/reeds_shepp.cpp
)

ament_auto_add_library(rrtstar_core SHARED
  src/rrtstar_core.cpp
)

ament_target_dependencies(rrtstar_core
  tf2
)
target_link_libraries(rrtstar_core
  reeds_shepp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/abstract_algorithm.cpp
  src/astar_search.cpp
  src/rrtstar.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  tf2
  tf2_geometry_msgs
  autoware_universe_utils
  autoware_vehicle_info_utils
)
target_link_libraries(${PROJECT_NAME}
  reeds_shepp
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  ament_add_ros_isolated_gtest(${PROJECT_NAME}-test
    test/src/test_freespace_planning_algorithms.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}-test
    rosbag2_cpp
    rosbag2_storage
  )
  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
  )

  ament_add_gtest(rrtstar_core_informed-test
    test/src/test_rrtstar_core_informed.cpp
  )
  target_link_libraries(rrtstar_core_informed-test
    ${PROJECT_NAME}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)

install(PROGRAMS
  test/debug_plot.py
  DESTINATION lib/${PROJECT_NAME}
)

# Force use of system Python 3.8
set(Python3_FIND_STRATEGY LOCATION)
set(Python3_FIND_REGISTRY LAST)
set(Python3_FIND_PREFER_EXECUTABLE TRUE)
set(Python3_EXECUTABLE /usr/bin/python3.8)
set(Python3_INCLUDE_DIR /usr/include/python3.8)
set(Python3_LIBRARY /usr/lib/aarch64-linux-gnu/libpython3.8.so)

# find_package(Python3 3.8 REQUIRED COMPONENTS Interpreter Development)
# find_package(pybind11_vendor REQUIRED)
# find_package(pybind11 REQUIRED)

# pybind11_add_module(${PROJECT_NAME}_pybind SHARED
#   scripts/bind/astar_search_pybind.cpp
# )

# include_directories(${PROJECT_NAME}_pybind PRIVATE
#   include/
# )
# target_link_libraries(${PROJECT_NAME}_pybind PRIVATE
#   ${PROJECT_NAME}
# )

# install(TARGETS ${PROJECT_NAME}_pybind
#   DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
# )
