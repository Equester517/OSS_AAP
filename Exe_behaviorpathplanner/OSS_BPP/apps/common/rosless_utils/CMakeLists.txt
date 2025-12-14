cmake_minimum_required(VERSION 3.14)
project(rosless_utils CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(rosless_utils
  src/bpp_path_utils.cpp
  src/drivable_area.cpp
  src/resample.cpp
)

target_include_directories(rosless_utils
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OSS_BPP_ROOT_DIR}/include
)

target_link_libraries(rosless_utils
  PUBLIC
    common_types
    lanelet2_core
    route_handler
)
