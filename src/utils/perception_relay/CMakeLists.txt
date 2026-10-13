cmake_minimum_required(VERSION 3.14)
project(perception_relay)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Perception relay node executable
ament_auto_add_executable(perception_relay_node
  src/perception_relay_node.cpp
  src/perception_relay_main.cpp
)

# Galactic perception test publisher executable (for testing perception relay)
ament_auto_add_executable(galactic_perception_test_publisher
  src/galactic_perception_test_publisher.cpp
  src/galactic_perception_test_publisher_main.cpp
)

# Install launch and config directories
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
