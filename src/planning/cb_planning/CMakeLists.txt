cmake_minimum_required(VERSION 3.16)
project(cb_planning)

# 设置C和C++标准
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# CMAKE_COMPILER_IS_GNUXX 在新版cmake中已经弃用，改用CMAKE_CXX_COMPILER_ID
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置预编译指令，生成compile_commands.json
# 然后让c_cpp_properties.json读取这个文件，防止预编译找不到cmake中的宏而报错
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PACKAGE_NAME ${PROJECT_NAME})

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pacmod_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# 20250912,zhangyue
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# 包含头文件目录
include_directories(
  include/common
  include/planning
  include/planning/path_planner
  include/simulator
  /usr/local/include/eigen-3.4.0
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${autoware_planning_msgs_INCLUDE_DIRS}
  ${pacmod_msgs_INCLUDE_DIRS}
  /usr/include/eigen3
)

# 添加子目录
add_subdirectory(src/common)
add_subdirectory(src/planning)
add_subdirectory(src/simulator)

# 安装辅助目录
install(DIRECTORY 
  config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# 安装 include 目录下的一级子目录
install(
  DIRECTORY
    include/common
    include/planning
    include/simulator
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
