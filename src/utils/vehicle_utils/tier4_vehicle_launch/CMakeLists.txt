cmake_minimum_required(VERSION 3.14)
project(tier4_vehicle_launch)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(PROGRAMS
    scripts/fix_xacro.sh
    DESTINATION share/${PROJECT_NAME}/scripts
)

ament_auto_package(INSTALL_TO_SHARE
    launch
    urdf
)
