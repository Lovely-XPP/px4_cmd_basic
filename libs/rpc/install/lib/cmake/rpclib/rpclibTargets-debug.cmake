#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rpclib::rpc" for configuration "Debug"
set_property(TARGET rpclib::rpc APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(rpclib::rpc PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/rpc.lib"
  )

list(APPEND _cmake_import_check_targets rpclib::rpc )
list(APPEND _cmake_import_check_files_for_rpclib::rpc "${_IMPORT_PREFIX}/lib/rpc.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
