cmake_minimum_required(VERSION 3.14)
project(autoware_external_cmd_converter)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(autoware_external_cmd_converter SHARED
  src/node.cpp
)

rclcpp_components_register_node(autoware_external_cmd_converter
  PLUGIN "autoware::external_cmd_converter::ExternalCmdConverterNode"
  EXECUTABLE external_cmd_converter_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}
    tests/test_external_cmd_converter.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    autoware_external_cmd_converter
  )
endif()


ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
