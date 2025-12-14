cmake_minimum_required(VERSION 3.14)
project(motion_utils CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(motion_utils
  src/trajectory.cpp
)

target_include_directories(motion_utils
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OSS_BPP_ROOT_DIR}/include
)

target_link_libraries(motion_utils
  PUBLIC
    common_types
    lanelet2_core
)
