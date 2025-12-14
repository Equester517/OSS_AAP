cmake_minimum_required(VERSION 3.14)
project(path_optimizer)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Find dependencies
find_package(Eigen3 REQUIRED)

# OSQP 찾기
set(OSQP_LIBRARY "/usr/local/lib/libosqp.so")
set(OSQP_INCLUDE_DIR "/usr/local/include")

# ============================================================
# OPTIMIZATION METHOD SELECTION
# ============================================================
# Option 1: USE_SIMPLE_OPTIMIZATION - Simple analytical method (no dependencies)
# Option 2: USE_OSQP - OSQP-based QP solver (requires OSQP library)
#
# To switch between methods:
#   - For simple: set(USE_SIMPLE TRUE)
#   - For OSQP:   set(USE_SIMPLE FALSE)
# ============================================================
set(USE_SIMPLE TRUE)  # Set to FALSE to use OSQP

if(USE_SIMPLE)
  message(STATUS "===================================")
  message(STATUS "Using SIMPLE analytical optimization")
  message(STATUS "===================================")
  add_definitions(-DUSE_SIMPLE_OPTIMIZATION)
  set(OSQP_FOUND FALSE)
elseif(EXISTS "${OSQP_LIBRARY}" AND EXISTS "${OSQP_INCLUDE_DIR}/osqp/osqp.h")
  message(STATUS "===================================")
  message(STATUS "Using OSQP-based optimization")
  message(STATUS "OSQP library: ${OSQP_LIBRARY}")
  message(STATUS "===================================")
  set(OSQP_FOUND TRUE)
  add_definitions(-DUSE_OSQP)
else()
  message(WARNING "OSQP not found and SIMPLE not selected. Using fallback.")
  set(OSQP_FOUND FALSE)
endif()

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Also allow using headers from the parent Exe_pathoptimizer include directory
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

if(OSQP_FOUND)
  add_definitions(-DUSE_OSQP)
  include_directories(${OSQP_INCLUDE_DIR})
  message(STATUS "Building with OSQP support")
else()
  message(STATUS "Building without OSQP support (simplified solver)")
endif()

# Source files
set(SOURCE_FILES
  src/path_optimizer.cpp
  src/mpt_optimizer.cpp
  src/replan_checker.cpp
)

# Include runner implementation so its symbol is available when linked by the exe
list(APPEND SOURCE_FILES src/path_optimizer_runner.cpp)

# OSQP interface 추가 (OSQP 사용 시)
if(OSQP_FOUND)
  list(APPEND SOURCE_FILES src/osqp_interface.cpp)
endif()

# Create library
add_library(${PROJECT_NAME}_lib ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Expose the exe include dir so files like path_optimizer/path_optimizer_runner.hpp
# located in Exe_pathoptimizer/include are discoverable when building as a subproject.
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)
target_link_libraries(${PROJECT_NAME}_lib
  Eigen3::Eigen
)

if(OSQP_FOUND)
  if(TARGET osqp::osqpstatic)
    target_link_libraries(${PROJECT_NAME}_lib osqp::osqpstatic)
  else()
    target_link_libraries(${PROJECT_NAME}_lib ${OSQP_LIBRARY})
  endif()
endif()

# Create executable
add_executable(${PROJECT_NAME}
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
  Eigen3::Eigen
)

if(OSQP_FOUND)
  if(TARGET osqp::osqpstatic)
    target_link_libraries(${PROJECT_NAME} osqp::osqpstatic)
  else()
    target_link_libraries(${PROJECT_NAME} ${OSQP_LIBRARY})
  endif()
endif()

# Print configuration summary
message(STATUS "===========================================")
message(STATUS "Path Optimizer Configuration:")
message(STATUS "  - C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  - Eigen3 found: ${Eigen3_FOUND}")
message(STATUS "  - OSQP found: ${OSQP_FOUND}")
if(OSQP_FOUND)
  message(STATUS "  - OSQP library: ${OSQP_LIBRARY}")
endif()
message(STATUS "  - Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "===========================================")

# Install
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

# Print configuration
message(STATUS "Path Optimizer Configuration:")
message(STATUS "  - C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  - Eigen3 found: ${EIGEN3_FOUND}")
message(STATUS "  - Build type: ${CMAKE_BUILD_TYPE}")
