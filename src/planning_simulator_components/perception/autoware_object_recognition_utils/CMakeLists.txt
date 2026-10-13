cmake_minimum_required(VERSION 3.14)
project(autoware_object_recognition_utils)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/predicted_path_utils.cpp
  src/conversion.cpp
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)

  file(GLOB_RECURSE test_files test/**/*.cpp)

  ament_add_gtest(test_object_recognition_utils ${test_files})

  target_link_libraries(test_object_recognition_utils
    ${PROJECT_NAME}
  )
endif()

ament_auto_package()
