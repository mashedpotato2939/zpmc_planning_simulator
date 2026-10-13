cmake_minimum_required(VERSION 3.14)
project(zpmc_map_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ZpmcMapInfo.msg"
  DEPENDENCIES std_msgs
)

ament_package()
