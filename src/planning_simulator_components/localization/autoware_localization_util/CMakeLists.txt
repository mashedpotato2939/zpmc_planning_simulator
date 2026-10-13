cmake_minimum_required(VERSION 3.14)
project(autoware_localization_util)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/util_func.cpp
  src/diagnostics_module.cpp
  src/smart_pose_buffer.cpp
  src/tree_structured_parzen_estimator.cpp
  src/covariance_ellipse.cpp
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_smart_pose_buffer
    test/test_smart_pose_buffer.cpp
    src/smart_pose_buffer.cpp
  )
  target_link_libraries(test_smart_pose_buffer ${PROJECT_NAME})

  ament_add_gtest(test_tpe
    test/test_tpe.cpp
    src/tree_structured_parzen_estimator.cpp
  )
  target_link_libraries(test_tpe ${PROJECT_NAME})
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)
