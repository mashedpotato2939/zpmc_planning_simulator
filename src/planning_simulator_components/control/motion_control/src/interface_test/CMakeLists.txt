cmake_minimum_required(VERSION 3.16)
project(interface_test)

# simple trajectory follower
add_library(${PROJECT_NAME}_lib SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/interface_test.cpp
)

add_executable(${PROJECT_NAME}_exe
  ${CMAKE_CURRENT_SOURCE_DIR}/interface_test_main.cpp
)

target_link_libraries(${PROJECT_NAME}_exe
  ${PROJECT_NAME}_lib
  control_utils_lib
)

# 安装动态库
install(TARGETS ${PROJECT_NAME}_lib
  DESTINATION lib
)

# Install executable
install(TARGETS ${PROJECT_NAME}_exe
  DESTINATION lib/${PACKAGE_NAME}
)
