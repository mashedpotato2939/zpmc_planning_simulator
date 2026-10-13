cmake_minimum_required(VERSION 3.14)
project(fms_sensor_kit_description)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)

ament_auto_package()
