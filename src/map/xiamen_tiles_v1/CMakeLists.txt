cmake_minimum_required(VERSION 3.14)
project(xiamen_tiles_v1)

find_package(ament_cmake REQUIRED)

install(
  FILES
    README.md
    map_header.bin
    tile_3221225472.bin
    tile_3221225473.bin
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
