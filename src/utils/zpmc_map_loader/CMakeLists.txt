cmake_minimum_required(VERSION 3.14)
project(zpmc_map_loader)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(zpmc_map_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hdmap REQUIRED)

# zpmc_map_loader node
add_library(zpmc_map_loader_node SHARED
  src/zpmc_map_loader_node.cpp
)
target_include_directories(zpmc_map_loader_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(zpmc_map_loader_node
  rclcpp
  rclcpp_components
  zpmc_map_msgs
  visualization_msgs
)
target_link_libraries(zpmc_map_loader_node
  hdmap::hdmap_lib
)
rclcpp_components_register_node(zpmc_map_loader_node
  PLUGIN "zpmc_map_loader::ZpmcMapLoaderNode"
  EXECUTABLE zpmc_map_loader
)

# Install
install(TARGETS zpmc_map_loader_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(zpmc_map_loader_node)
ament_export_dependencies(rclcpp zpmc_map_msgs hdmap)

ament_package()
