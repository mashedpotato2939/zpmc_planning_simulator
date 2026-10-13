cmake_minimum_required(VERSION 3.14)
project(fms_sensor_kit_launch)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_auto_package()
