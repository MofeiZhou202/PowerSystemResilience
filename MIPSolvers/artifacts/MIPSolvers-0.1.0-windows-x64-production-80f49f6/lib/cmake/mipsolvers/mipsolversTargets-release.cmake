#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mipsolvers::mipsolvers" for configuration "Release"
set_property(TARGET mipsolvers::mipsolvers APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(mipsolvers::mipsolvers PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/mipsolvers.lib"
  )

list(APPEND _cmake_import_check_targets mipsolvers::mipsolvers )
list(APPEND _cmake_import_check_files_for_mipsolvers::mipsolvers "${_IMPORT_PREFIX}/lib/mipsolvers.lib" )

# Import target "mipsolvers::cholmod_vendored" for configuration "Release"
set_property(TARGET mipsolvers::cholmod_vendored APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(mipsolvers::cholmod_vendored PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/cholmod_vendored.lib"
  )

list(APPEND _cmake_import_check_targets mipsolvers::cholmod_vendored )
list(APPEND _cmake_import_check_files_for_mipsolvers::cholmod_vendored "${_IMPORT_PREFIX}/lib/cholmod_vendored.lib" )

# Import target "mipsolvers::umfpack_vendored" for configuration "Release"
set_property(TARGET mipsolvers::umfpack_vendored APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(mipsolvers::umfpack_vendored PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/umfpack_vendored.lib"
  )

list(APPEND _cmake_import_check_targets mipsolvers::umfpack_vendored )
list(APPEND _cmake_import_check_files_for_mipsolvers::umfpack_vendored "${_IMPORT_PREFIX}/lib/umfpack_vendored.lib" )

# Import target "mipsolvers::klu_vendored" for configuration "Release"
set_property(TARGET mipsolvers::klu_vendored APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(mipsolvers::klu_vendored PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/klu_vendored.lib"
  )

list(APPEND _cmake_import_check_targets mipsolvers::klu_vendored )
list(APPEND _cmake_import_check_files_for_mipsolvers::klu_vendored "${_IMPORT_PREFIX}/lib/klu_vendored.lib" )

# Import target "mipsolvers::fmt" for configuration "Release"
set_property(TARGET mipsolvers::fmt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(mipsolvers::fmt PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/fmt.lib"
  )

list(APPEND _cmake_import_check_targets mipsolvers::fmt )
list(APPEND _cmake_import_check_files_for_mipsolvers::fmt "${_IMPORT_PREFIX}/lib/fmt.lib" )

# Import target "mipsolvers::highs" for configuration "Release"
set_property(TARGET mipsolvers::highs APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(mipsolvers::highs PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C;CXX;RC"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/highs.lib"
  )

list(APPEND _cmake_import_check_targets mipsolvers::highs )
list(APPEND _cmake_import_check_files_for_mipsolvers::highs "${_IMPORT_PREFIX}/lib/highs.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
