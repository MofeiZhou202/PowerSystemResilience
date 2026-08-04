cmake_minimum_required(VERSION 3.20)

project(mipsolvers_prebuilt_consumer_smoke LANGUAGES CXX)

if(NOT DEFINED MIPSOLVERS_PREBUILT_CONFIG OR
   NOT EXISTS "${MIPSOLVERS_PREBUILT_CONFIG}")
  message(FATAL_ERROR
    "MIPSOLVERS_PREBUILT_CONFIG must name an installed package config")
endif()
if(NOT DEFINED MIPSOLVERS_EXPECTED_PREFIX)
  message(FATAL_ERROR "MIPSOLVERS_EXPECTED_PREFIX is required")
endif()

add_subdirectory(loader)

foreach(_required_target IN ITEMS
    MIPSolvers::MKL ipopt_local Ipopt::ipopt
    highs highs::highs libscip SCIP::libscip
    Catch2 Catch2::Catch2 Catch2WithMain Catch2::Catch2WithMain
    Eigen3::Eigen nlohmann_json::nlohmann_json)
  if(NOT TARGET ${_required_target})
    message(FATAL_ERROR
      "Prebuilt package did not restore target ${_required_target}")
  endif()
endforeach()
foreach(_suitesparse_target IN ITEMS umfpack_vendored klu_vendored)
  if(TARGET ${_suitesparse_target} AND
     NOT TARGET mipsolvers::${_suitesparse_target})
    message(FATAL_ERROR
      "Prebuilt package did not restore alias mipsolvers::${_suitesparse_target}")
  endif()
endforeach()

get_target_property(_mkl_includes MIPSolvers::MKL INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(_mkl_libraries MIPSolvers::MKL INTERFACE_LINK_LIBRARIES)
if(NOT _mkl_includes OR NOT _mkl_libraries)
  message(FATAL_ERROR "MIPSolvers::MKL has an incomplete link interface")
endif()
list(LENGTH _mkl_libraries _mkl_library_count)
if(NOT _mkl_library_count EQUAL 3)
  message(FATAL_ERROR
    "Expected three sequential static oneMKL libraries, got: ${_mkl_libraries}")
endif()

foreach(_payload IN LISTS _mkl_includes _mkl_libraries)
  if(NOT IS_ABSOLUTE "${_payload}" OR NOT EXISTS "${_payload}")
    message(FATAL_ERROR "Missing prebuilt oneMKL payload: ${_payload}")
  endif()
  file(RELATIVE_PATH _payload_relative "${MIPSOLVERS_EXPECTED_PREFIX}" "${_payload}")
  if(_payload_relative MATCHES "^\\.\\.(/|$)")
    message(FATAL_ERROR
      "oneMKL payload escapes the prebuilt prefix: ${_payload}")
  endif()
endforeach()

get_target_property(_ipopt_link_libraries ipopt_local INTERFACE_LINK_LIBRARIES)
string(FIND "${_ipopt_link_libraries}" "MIPSolvers::MKL" _mkl_target_index)
if(_mkl_target_index EQUAL -1)
  message(FATAL_ERROR
    "ipopt_local does not link through the relocatable MIPSolvers::MKL target")
endif()

add_library(mipsolvers_prebuilt_smoke_consumer INTERFACE)
target_link_libraries(mipsolvers_prebuilt_smoke_consumer INTERFACE Ipopt::ipopt)
