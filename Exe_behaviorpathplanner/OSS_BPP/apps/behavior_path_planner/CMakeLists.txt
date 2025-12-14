cmake_minimum_required(VERSION 3.10)
project(behavior_path_planner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# libs/Rosless-Lanelet2, libs/route_handler 는
# 최상위 CMakeLists.txt 에서 add_subdirectory() 이미 호출됨

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Eigen3 REQUIRED)

# ------------------------------------------------------------
# 2. behavior_path_planner_common 라이브러리
# ------------------------------------------------------------
file(GLOB_RECURSE BPP_COMMON_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/autoware_behavior_path_planner_common/src/*.cpp
)

add_library(autoware_behavior_path_planner_common STATIC
  ${BPP_COMMON_SOURCES}
)

target_include_directories(autoware_behavior_path_planner_common
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/autoware_behavior_path_planner_common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/autoware_behavior_path_planner/include
)

target_link_libraries(autoware_behavior_path_planner_common
  PUBLIC
    common_types
    rosless_utils
    motion_utils
    Boost::system
    Boost::filesystem
    Eigen3::Eigen
    route_handler        # 우리가 만든 lib
    lanelet2_core        # Rosless-Lanelet2 타겟
    lanelet2_routing
    lanelet2_traffic_rules
    lanelet2_io
    lanelet2_projection
)

# ------------------------------------------------------------
# 3. 실행 파일: autoware_behavior_path_planner
# ------------------------------------------------------------
# ------------------------------------------------------------
# 3. 실행 파일: autoware_behavior_path_planner
# ------------------------------------------------------------
add_executable(autoware_behavior_path_planner
  ${CMAKE_CURRENT_SOURCE_DIR}/autoware_behavior_path_planner/src/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/autoware_behavior_path_planner/src/planner_manager.cpp
  # 필요시 원본 비교용
  # ${CMAKE_CURRENT_SOURCE_DIR}/autoware_behavior_path_planner/src/planner_manager_ori.cpp
)

target_include_directories(autoware_behavior_path_planner
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/autoware_behavior_path_planner/include
    ${CMAKE_CURRENT_SOURCE_DIR}/autoware_behavior_path_planner_common/include
)

target_link_libraries(autoware_behavior_path_planner
  PRIVATE
    autoware_behavior_path_planner_common
    common_types
    route_handler
    rosless_utils
    motion_utils
    lanelet2_core
    lanelet2_io
    lanelet2_projection
)
