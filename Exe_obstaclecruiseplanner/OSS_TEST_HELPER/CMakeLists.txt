cmake_minimum_required(VERSION 3.20)
project(OSS_TEST_HELPER VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Create library for OSS_TEST_HELPER types and utilities
add_library(oss_test_helper_lib STATIC
  src/jsonl_reader.cpp
  src/jsonl_writer.cpp
)

target_include_directories(
  oss_test_helper_lib PUBLIC
  include
  include/vendor
)

# Executable for testing purposes
add_executable(OSS_TEST_HELPER
  src/main.cpp
  src/trigger.cpp
  src/processor.cpp
)

target_link_libraries(OSS_TEST_HELPER PRIVATE oss_test_helper_lib)
