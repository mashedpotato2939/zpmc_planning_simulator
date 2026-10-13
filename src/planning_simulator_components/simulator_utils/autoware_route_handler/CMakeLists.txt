cmake_minimum_required(VERSION 3.14)
project(autoware_route_handler)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/route_handler.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_autoware_route_handler
    test/test_route_handler.cpp)

  target_link_libraries(test_autoware_route_handler
    ${PROJECT_NAME}
  )

endif()

ament_auto_package(INSTALL_TO_SHARE
  test_route
)
