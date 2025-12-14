cmake_minimum_required(VERSION 3.14)
project(OSS_Planner CXX)

if(NOT DEFINED OSS_BPP_ROOT_DIR)
    set(OSS_BPP_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------------------------------------
# 0. 공통 타입(common_types) 인터페이스 라이브러리
# ------------------------------------------------------------
add_library(common_types INTERFACE)
target_include_directories(common_types INTERFACE
    ${OSS_BPP_ROOT_DIR}/include
)

# ------------------------------------------------------------
# 1. 외부 라이브러리: Rosless-Lanelet2
#    lanelet2_core, lanelet2_io, lanelet2_routing, ...
# ------------------------------------------------------------
add_subdirectory(libs/Rosless-Lanelet2)

# ------------------------------------------------------------
# 2. 공통 유틸 (apps/common)
# ------------------------------------------------------------
add_subdirectory(apps/common)

# ------------------------------------------------------------
# 3. route_handler 라이브러리
# ------------------------------------------------------------
add_subdirectory(apps/route_handler)

# ------------------------------------------------------------
# 4. behavior_path_planner 앱
# ------------------------------------------------------------
add_subdirectory(apps/behavior_path_planner)
