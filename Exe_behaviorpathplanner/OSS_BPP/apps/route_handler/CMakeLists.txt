cmake_minimum_required(VERSION 3.10)
project(route_handler CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(route_handler
    src/route_handler.cpp
)

target_include_directories(route_handler
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Rosless-Lanelet2 에서 제공하는 타겟 사용
target_link_libraries(route_handler
    PUBLIC
        common_types
        lanelet2_core
        lanelet2_io
        lanelet2_routing
        lanelet2_traffic_rules
        lanelet2_projection
)
