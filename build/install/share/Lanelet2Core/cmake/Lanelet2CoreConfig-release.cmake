#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lanelet2_core" for configuration "Release"
set_property(TARGET lanelet2_core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lanelet2_core PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblanelet2_core.so.1.0.0"
  IMPORTED_SONAME_RELEASE "liblanelet2_core.so.1.0.0"
  )

list(APPEND _IMPORT_CHECK_TARGETS lanelet2_core )
list(APPEND _IMPORT_CHECK_FILES_FOR_lanelet2_core "${_IMPORT_PREFIX}/lib/liblanelet2_core.so.1.0.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
