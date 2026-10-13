cmake_minimum_required(VERSION 3.14)
project(fms_bridge_rviz_plugin)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/fms_bridge_panel.cpp
  src/fms_point_tool.cpp
)

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})

target_compile_options(${PROJECT_NAME} PUBLIC -Wno-error=deprecated-copy -Wno-error=pedantic)

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

ament_auto_package(INSTALL_TO_SHARE plugins)
