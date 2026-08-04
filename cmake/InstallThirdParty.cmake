# cmake/InstallThirdParty.cmake
# Install/export/manifest logic for MIPSOLVERS_THIRD_PARTY_ONLY mode.
# Included from the top-level CMakeLists.txt right after Dependencies.cmake;
# the caller return()s immediately afterwards, so this file owns the whole
# install surface of the third-party-only build:
#
#   libraries  — every vendored target (SuiteSparse, MUMPS, HiGHS, SCIP,
#                Ipopt, fmt, Catch2, reflapack when built) in the
#                mipsolversThirdPartyTargets export set (no namespace: the
#                imported names must equal the in-tree target names)
#   headers    — fmt, Catch2 (amalgamated + catch2/ forwarding headers),
#                Eigen, nlohmann_json, SuiteSparse, MUMPS, HiGHS (+HConfig.h),
#                SCIP (+config.h), Ipopt (+config.h/config_ipopt.h)
#   package    — mipsolversThirdPartyConfig.cmake (CMakePackageConfigHelpers)
#   manifest   — manifest.cmake: toolchain + configuration record validated by
#                cmake/UsePrebuiltThirdParty.cmake on the consumer side

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(MIPSOLVERS_TP_CMAKEDIR "${CMAKE_INSTALL_LIBDIR}/cmake/mipsolvers-third-party")

# ── Make the vendored targets exportable / self-contained ───────────────────
# Catch2's in-tree include dir is a bare source-tree path, which install(EXPORT)
# rejects; re-wrap it as BUILD_INTERFACE + INSTALL_INTERFACE.
foreach(_tp_catch Catch2 Catch2WithMain)
  if(TARGET ${_tp_catch})
    set_target_properties(${_tp_catch} PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/catch2>;$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
  endif()
endforeach()

# INSTALL_INTERFACE include dirs so the exported targets resolve their public
# headers against the install prefix (BUILD_INTERFACE entries are dropped by
# install(EXPORT)).  SuiteSparse stays variable-driven (see the
# MIPSOLVERS_TP_*_INCLUDE_DIRS values in mipsolversThirdPartyConfig.cmake.in),
# matching the in-tree layout where its include dirs are PRIVATE.
if(TARGET highs)
  set_property(TARGET highs APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/highs>")
endif()
if(TARGET libscip)
  # scip/CMakeLists.txt already contributes INSTALL_INTERFACE:include (the
  # SCIP header root); lpiexact/ and symmetry/ are separate in-tree include
  # dirs (cmake/BuildSCIP.cmake) and are mirrored here.
  set_property(TARGET libscip APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/lpiexact>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/symmetry>")
endif()
if(TARGET ipopt_local)
  set_property(TARGET ipopt_local APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ipopt/Algorithm>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ipopt/Algorithm/Inexact>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ipopt/Algorithm/LinearSolvers>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ipopt/Common>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ipopt/Interfaces>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ipopt/LinAlg>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ipopt/LinAlg/TMatrices>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ipopt/contrib/CGPenalty>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ipopt/config>")
endif()
if(TARGET dmumps)
  set_property(TARGET dmumps APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/mumps>")
endif()

# ── Install the third-party libraries ─────────────────────────────────────────
# dmumps/MUMPS/reflapack_vendored/mipsolvers_fortran_main_stub are installed
# by cmake/BuildMUMPS.cmake / cmake/BuildRefLAPACK.cmake directly into
# ${MIPSOLVERS_THIRD_PARTY_EXPORT_SET}; the rest is handled here.
set(_tp_lib_targets "")
foreach(_tp_target IN ITEMS
    cholmod_vendored umfpack_vendored klu_vendored
    fmt highs libscip ipopt_local Catch2 Catch2WithMain
    dmumps MUMPS reflapack_vendored mipsolvers_fortran_main_stub)
  if(TARGET ${_tp_target})
    list(APPEND _tp_lib_targets ${_tp_target})
  endif()
endforeach()

# Umbrella default-build target: fmt/HiGHS/SCIP come from EXCLUDE_FROM_ALL
# add_subdirectory calls, so in this mode (no mipsolvers library links them)
# the default "all" target would skip them and cmake --install would fail on
# the missing archives.
add_custom_target(mipsolvers_third_party ALL DEPENDS ${_tp_lib_targets})

foreach(_tp_target IN ITEMS
    cholmod_vendored umfpack_vendored klu_vendored
    fmt highs libscip ipopt_local Catch2 Catch2WithMain)
  if(TARGET ${_tp_target})
    install(TARGETS ${_tp_target}
      EXPORT ${MIPSOLVERS_THIRD_PARTY_EXPORT_SET}
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
  endif()
endforeach()

# ── Headers: fmt / Catch2 / Eigen / nlohmann_json ─────────────────────────────
if(MIPSOLVERS_FMT_VENDORED)
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/third_party/fmt/include/fmt"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()

if(TARGET Catch2)
  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/third_party/catch2/catch_amalgamated.hpp"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  # catch2/catch_*.hpp forwarding headers (they include ../catch_amalgamated.hpp)
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/third_party/catch2/catch2"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endif()

# Header-only deps: resolve the include dir from the imported target so a
# (non-default) system Eigen3/nlohmann_json source is bundled correctly too.
if(TARGET Eigen3::Eigen)
  get_target_property(_tp_eigen_incs Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
  foreach(_tp_inc IN LISTS _tp_eigen_incs)
    if(EXISTS "${_tp_inc}/Eigen/Core")
      install(DIRECTORY "${_tp_inc}/Eigen"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/mipsolvers-deps/eigen3")
      if(EXISTS "${_tp_inc}/unsupported")
        install(DIRECTORY "${_tp_inc}/unsupported"
          DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/mipsolvers-deps/eigen3")
      endif()
      break()
    endif()
  endforeach()
  unset(_tp_eigen_incs)
endif()
if(TARGET nlohmann_json::nlohmann_json)
  get_target_property(_tp_json_incs nlohmann_json::nlohmann_json INTERFACE_INCLUDE_DIRECTORIES)
  foreach(_tp_inc IN LISTS _tp_json_incs)
    if(EXISTS "${_tp_inc}/nlohmann/json.hpp")
      install(DIRECTORY "${_tp_inc}/nlohmann"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/mipsolvers-deps/nlohmann_json")
      break()
    endif()
  endforeach()
  unset(_tp_json_incs)
endif()

# ── Headers: SuiteSparse (component dirs mirrored flat, as in-tree) ──────────
if(MIPSOLVERS_HAVE_CHOLMOD)
  set(_tp_ss "${CMAKE_CURRENT_SOURCE_DIR}/suitesparse")
  install(DIRECTORY "${_tp_ss}/SuiteSparse_config/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/suitesparse/SuiteSparse_config"
    FILES_MATCHING PATTERN "*.h")
  foreach(_tp_ss_comp AMD CAMD COLAMD CCOLAMD CHOLMOD UMFPACK KLU BTF)
    install(DIRECTORY "${_tp_ss}/${_tp_ss_comp}/Include/"
      DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/suitesparse/${_tp_ss_comp}"
      FILES_MATCHING PATTERN "*.h")
  endforeach()
  unset(_tp_ss)
endif()

# ── Headers: MUMPS (include/ + libseq/ flattened into one dir, as in-tree) ───
if(TARGET dmumps)
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/mumps/include/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/mumps" FILES_MATCHING PATTERN "*.h")
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/mumps/libseq/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/mumps" FILES_MATCHING PATTERN "*.h")
endif()

# ── Headers: HiGHS (whole tree + generated HConfig.h) ────────────────────────
if(TARGET highs)
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/highs/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/highs"
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/_deps/embedded_highs/HConfig.h"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/highs")
endif()

# ── Headers: SCIP (tree contents at the include root + generated config.h) ───
if(TARGET libscip)
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/scip/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
  # Both files are generated by the embedded SCIP configure. Public SCIP
  # headers include scip_export.h, so omitting it makes the installed target
  # unusable even though libscip itself was installed successfully.
  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/_deps/embedded_scip/scip/config.h"
    "${CMAKE_CURRENT_BINARY_DIR}/_deps/embedded_scip/scip/scip_export.h"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/scip")
endif()

# ── Headers: Ipopt (per-dir layout, as in-tree) + generated config ───────────
if(TARGET ipopt_local)
  foreach(_tp_ipopt_dir IN ITEMS
      Algorithm Algorithm/Inexact Algorithm/LinearSolvers Common Interfaces
      LinAlg LinAlg/TMatrices contrib/CGPenalty)
    install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/ipopt/${_tp_ipopt_dir}/"
      DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/ipopt/${_tp_ipopt_dir}"
      FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
  endforeach()
  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/_deps/embedded_ipopt/config.h"
    "${CMAKE_CURRENT_BINARY_DIR}/_deps/embedded_ipopt/config_ipopt.h"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/ipopt/config")
endif()

# ── Hermetic oneMKL static payload ───────────────────────────────────────────
# ipopt_local is a static archive, so its MKL symbols are resolved only when a
# consumer links the final executable. Copy the static libraries and headers
# into the package and recreate MIPSolvers::MKL from package-relative paths.
set(_tp_mkl_library_names "")
set(_tp_mkl_version "")
if(MIPSOLVERS_HAVE_MKL_PARDISO)
  set(_tp_mkl_include_dir "")
  foreach(_tp_mkl_inc IN LISTS MIPSOLVERS_MKL_INCLUDE_DIRS)
    if(EXISTS "${_tp_mkl_inc}/mkl_pardiso.h")
      set(_tp_mkl_include_dir "${_tp_mkl_inc}")
      break()
    endif()
  endforeach()
  if(NOT _tp_mkl_include_dir)
    message(FATAL_ERROR
      "MKL Pardiso is enabled, but no include directory contains mkl_pardiso.h")
  endif()

  install(DIRECTORY "${_tp_mkl_include_dir}/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/mipsolvers-deps/oneapi-mkl"
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

  foreach(_tp_mkl_lib IN LISTS MIPSOLVERS_MKL_LIBRARIES)
    if(IS_ABSOLUTE "${_tp_mkl_lib}" AND EXISTS "${_tp_mkl_lib}")
      get_filename_component(_tp_mkl_name "${_tp_mkl_lib}" NAME)
      install(FILES "${_tp_mkl_lib}"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/mipsolvers-deps/oneapi-mkl")
      list(APPEND _tp_mkl_library_names "${_tp_mkl_name}")
    endif()
  endforeach()
  if(NOT _tp_mkl_library_names)
    message(FATAL_ERROR
      "MKL Pardiso is enabled, but no concrete MKL library files can be packaged")
  endif()

  if(MIPSOLVERS_MKL_ROOT)
    if(NOT EXISTS "${MIPSOLVERS_MKL_ROOT}/manifest.cmake" OR
       NOT EXISTS "${MIPSOLVERS_MKL_ROOT}/SHA256SUMS" OR
       NOT IS_DIRECTORY "${MIPSOLVERS_MKL_ROOT}/licensing")
      message(FATAL_ERROR
        "The local oneMKL bundle is missing manifest.cmake, SHA256SUMS, or "
        "licensing/. Recreate it with third_party/stage_onemkl.ps1.")
    endif()
    include("${MIPSOLVERS_MKL_ROOT}/manifest.cmake")
    if(NOT MIPSOLVERS_LOCAL_MKL_MANIFEST_VERSION STREQUAL "1" OR
       NOT MIPSOLVERS_LOCAL_MKL_LINKAGE STREQUAL "static" OR
       NOT MIPSOLVERS_LOCAL_MKL_THREADING STREQUAL "sequential")
      message(FATAL_ERROR
        "Unsupported oneMKL bundle manifest at ${MIPSOLVERS_MKL_ROOT}")
    endif()
    if(WIN32 AND
       (NOT MIPSOLVERS_LOCAL_MKL_ARCHITECTURE STREQUAL "x64" OR
        NOT CMAKE_SIZEOF_VOID_P EQUAL 8))
      message(FATAL_ERROR
        "The staged oneMKL bundle is x64-only and cannot be packaged for the "
        "current Windows target")
    endif()
    set(_tp_mkl_version "${MIPSOLVERS_LOCAL_MKL_VERSION}")
    install(DIRECTORY "${MIPSOLVERS_MKL_ROOT}/licensing/"
      DESTINATION "${CMAKE_INSTALL_DATADIR}/mipsolvers-third-party/licenses/oneapi-mkl")
    install(FILES
      "${MIPSOLVERS_MKL_ROOT}/manifest.cmake"
      "${MIPSOLVERS_MKL_ROOT}/SHA256SUMS"
      DESTINATION "${CMAKE_INSTALL_DATADIR}/mipsolvers-third-party/oneapi-mkl")
  endif()
endif()

# ── Export set + package config ───────────────────────────────────────────────
install(EXPORT ${MIPSOLVERS_THIRD_PARTY_EXPORT_SET}
  FILE mipsolversThirdPartyTargets.cmake
  DESTINATION ${MIPSOLVERS_TP_CMAKEDIR})

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/mipsolversThirdPartyConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/mipsolversThirdPartyConfig.cmake
  INSTALL_DESTINATION ${MIPSOLVERS_TP_CMAKEDIR}
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR)

# ── manifest.cmake — toolchain + configuration record ────────────────────────
set(_tp_enabled_deps "")
foreach(_tp_dep IN ITEMS
    "highs:highs" "scip:libscip" "ipopt:ipopt_local" "mumps:dmumps"
    "mkl:MIPSolvers::MKL"
    "cholmod:cholmod_vendored" "umfpack:umfpack_vendored" "klu:klu_vendored"
    "fmt:fmt" "catch2:Catch2" "reflapack:reflapack_vendored"
    "eigen:Eigen3::Eigen" "nlohmann_json:nlohmann_json::nlohmann_json")
  string(FIND "${_tp_dep}" ":" _tp_sep)
  string(SUBSTRING "${_tp_dep}" 0 ${_tp_sep} _tp_dep_name)
  math(EXPR _tp_sep_next "${_tp_sep} + 1")
  string(SUBSTRING "${_tp_dep}" ${_tp_sep_next} -1 _tp_dep_target)
  if(TARGET ${_tp_dep_target})
    list(APPEND _tp_enabled_deps "${_tp_dep_name}")
  endif()
endforeach()

# PUBLIC compile definitions the consumer build re-applies (recorded for
# inspection; UsePrebuiltThirdParty.cmake re-derives them from the recorded
# HAVE state, so CMakeLists.txt needs no changes).
set(_tp_public_defs "")
if(NOT MIPSOLVERS_EIGEN_VECTORIZE)
  list(APPEND _tp_public_defs EIGEN_DONT_VECTORIZE=1 EIGEN_UNALIGNED_VECTORIZE=0)
endif()
if(MIPSOLVERS_EIGEN_MAX_ALIGN_BYTES)
  list(APPEND _tp_public_defs EIGEN_MAX_ALIGN_BYTES=${MIPSOLVERS_EIGEN_MAX_ALIGN_BYTES})
endif()
if(WIN32)
  list(APPEND _tp_public_defs NOMINMAX WIN32_LEAN_AND_MEAN)
endif()
if(MIPSOLVERS_HAVE_HIGHS_LIB)
  list(APPEND _tp_public_defs
    MIPSOLVERS_HAVE_HIGHS_LIB=1 HACDCPF_HAVE_HIGHS_LIB=1
    MIPSOLVERS_NATIVE_BC_STRICT_HIGHS=1 MIPSOLVERS_NATIVE_BC_DYNAMIC_CUTS=1)
endif()
if(MIPSOLVERS_HAVE_SCIP_LIB)
  list(APPEND _tp_public_defs MIPSOLVERS_HAVE_SCIP_LIB=1 HACDCPF_HAVE_SCIP_LIB=1)
endif()
if(MIPSOLVERS_HAVE_SUITESPARSE)
  list(APPEND _tp_public_defs
    MIPSOLVERS_HAVE_SUITESPARSE=1 HACDCPF_HAVE_SUITESPARSE=1
    MIPSOLVERS_HAVE_UMFPACK=1 HACDCPF_HAVE_UMFPACK=1
    MIPSOLVERS_HAVE_KLU=1 HACDCPF_HAVE_KLU=1)
endif()
if(MIPSOLVERS_HAVE_CHOLMOD)
  list(APPEND _tp_public_defs MIPSOLVERS_HAVE_CHOLMOD=1 HACDCPF_HAVE_CHOLMOD=1)
endif()
if(MIPSOLVERS_HAVE_IPOPT)
  list(APPEND _tp_public_defs MIPSOLVERS_HAVE_IPOPT=1 HACDCPF_HAVE_IPOPT=1)
endif()
if(MIPSOLVERS_HAVE_MKL_PARDISO)
  list(APPEND _tp_public_defs
    MIPSOLVERS_HAVE_MKL_PARDISO=1 HACDCPF_HAVE_MKL_PARDISO=1)
endif()
if((NOT DEFINED MIPSOLVERS_USE_MUMPS OR MIPSOLVERS_USE_MUMPS) AND TARGET MUMPS::MUMPS)
  list(APPEND _tp_public_defs MIPSOLVERS_HAVE_MUMPS=1 HACDCPF_HAVE_MUMPS=1)
endif()
if(APPLE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang")
  list(APPEND _tp_public_defs MIPSOLVERS_HAVE_ACCELERATE=1 HACDCPF_HAVE_ACCELERATE=1)
endif()

set(_tp_manifest "${CMAKE_CURRENT_BINARY_DIR}/manifest.cmake")

# CMake leaves the runtime selection empty when MSVC's default /MD[/d] policy
# is in effect. Record that effective default explicitly so an apparently
# empty field cannot hide a /MT versus /MD ABI mismatch.
set(_tp_msvc_runtime "${CMAKE_MSVC_RUNTIME_LIBRARY}")
if(MSVC AND NOT _tp_msvc_runtime)
  set(_tp_msvc_runtime "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

set(_tp_build_config "${MIPSOLVERS_THIRD_PARTY_BUILD_CONFIG}")
if(NOT _tp_build_config)
  set(_tp_build_config "${CMAKE_BUILD_TYPE}")
endif()
if(NOT _tp_build_config AND CMAKE_CONFIGURATION_TYPES)
  set(_tp_build_config "Release")
endif()
get_property(_tp_is_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
set(_tp_configuration_types "")
if(_tp_is_multi_config)
  # cmake --install installs one --config per invocation. Advertise only that
  # configuration, even when the generator's default list contains several.
  set(_tp_configuration_types "${_tp_build_config}")
endif()

file(WRITE "${_tp_manifest}"
  "# manifest.cmake — recorded by a MIPSOLVERS_THIRD_PARTY_ONLY build.\n"
  "# Validated by cmake/UsePrebuiltThirdParty.cmake before the prebuilt\n"
  "# package is consumed; keep field names stable.\n"
  "\n"
  "set(MIPSOLVERS_TP_MANIFEST_VERSION \"3\")\n"
  "set(MIPSOLVERS_TP_CXX_COMPILER_ID \"${CMAKE_CXX_COMPILER_ID}\")\n"
  "set(MIPSOLVERS_TP_CXX_COMPILER_VERSION \"${CMAKE_CXX_COMPILER_VERSION}\")\n"
  "set(MIPSOLVERS_TP_C_COMPILER_ID \"${CMAKE_C_COMPILER_ID}\")\n"
  "set(MIPSOLVERS_TP_C_COMPILER_VERSION \"${CMAKE_C_COMPILER_VERSION}\")\n"
  "set(MIPSOLVERS_TP_SYSTEM_NAME \"${CMAKE_SYSTEM_NAME}\")\n"
  "set(MIPSOLVERS_TP_SYSTEM_VERSION \"${CMAKE_SYSTEM_VERSION}\")\n"
  "set(MIPSOLVERS_TP_SYSTEM_PROCESSOR \"${CMAKE_SYSTEM_PROCESSOR}\")\n"
  "set(MIPSOLVERS_TP_SIZEOF_VOID_P \"${CMAKE_SIZEOF_VOID_P}\")\n"
  "set(MIPSOLVERS_TP_GENERATOR \"${CMAKE_GENERATOR}\")\n"
  "set(MIPSOLVERS_TP_GENERATOR_PLATFORM \"${CMAKE_GENERATOR_PLATFORM}\")\n"
  "set(MIPSOLVERS_TP_GENERATOR_TOOLSET \"${CMAKE_GENERATOR_TOOLSET}\")\n"
  "set(MIPSOLVERS_TP_VS_PLATFORM_NAME \"${CMAKE_VS_PLATFORM_NAME}\")\n"
  "set(MIPSOLVERS_TP_CXX_COMPILER_ARCHITECTURE_ID \"${CMAKE_CXX_COMPILER_ARCHITECTURE_ID}\")\n"
  "set(MIPSOLVERS_TP_MSVC \"${MSVC}\")\n"
  "set(MIPSOLVERS_TP_MSVC_VERSION \"${MSVC_VERSION}\")\n"
  "set(MIPSOLVERS_TP_MSVC_RUNTIME_LIBRARY \"${_tp_msvc_runtime}\")\n"
  "set(MIPSOLVERS_TP_OSX_SYSROOT \"${CMAKE_OSX_SYSROOT}\")\n"
  "set(MIPSOLVERS_TP_OSX_DEPLOYMENT_TARGET \"${CMAKE_OSX_DEPLOYMENT_TARGET}\")\n"
  "set(MIPSOLVERS_TP_BUILD_TYPE \"${_tp_build_config}\")\n"
  "set(MIPSOLVERS_TP_MULTI_CONFIG \"${_tp_is_multi_config}\")\n"
  "set(MIPSOLVERS_TP_CONFIGURATION_TYPES \"${_tp_configuration_types}\")\n"
  "set(MIPSOLVERS_TP_EIGEN_VECTORIZE \"${MIPSOLVERS_EIGEN_VECTORIZE}\")\n"
  "set(MIPSOLVERS_TP_EIGEN_MAX_ALIGN_BYTES \"${MIPSOLVERS_EIGEN_MAX_ALIGN_BYTES}\")\n"
  "set(MIPSOLVERS_TP_HAVE_HIGHS_LIB \"${MIPSOLVERS_HAVE_HIGHS_LIB}\")\n"
  "set(MIPSOLVERS_TP_HAVE_SCIP_LIB \"${MIPSOLVERS_HAVE_SCIP_LIB}\")\n"
  "set(MIPSOLVERS_TP_HAVE_IPOPT \"${MIPSOLVERS_HAVE_IPOPT}\")\n"
  "set(MIPSOLVERS_TP_IPOPT_LINEAR_SOLVER \"${MIPSOLVERS_IPOPT_LINEAR_SOLVER}\")\n"
  "set(MIPSOLVERS_TP_HAVE_MKL_PARDISO \"${MIPSOLVERS_HAVE_MKL_PARDISO}\")\n"
  "set(MIPSOLVERS_TP_MKL_LIBRARY_NAMES \"${_tp_mkl_library_names}\")\n"
  "set(MIPSOLVERS_TP_MKL_VERSION \"${_tp_mkl_version}\")\n"
  "set(MIPSOLVERS_TP_HAVE_CHOLMOD \"${MIPSOLVERS_HAVE_CHOLMOD}\")\n"
  "set(MIPSOLVERS_TP_HAVE_SUITESPARSE \"${MIPSOLVERS_HAVE_SUITESPARSE}\")\n"
  "set(MIPSOLVERS_TP_HAVE_UMFPACK \"${MIPSOLVERS_HAVE_UMFPACK}\")\n"
  "set(MIPSOLVERS_TP_HAVE_KLU \"${MIPSOLVERS_HAVE_KLU}\")\n"
  "set(MIPSOLVERS_TP_ENABLED_DEPS \"${_tp_enabled_deps}\")\n"
  "set(MIPSOLVERS_TP_BLAS_SOURCE \"${_MIPSOLVERS_BLAS_SOURCE}\")\n"
  "set(MIPSOLVERS_TP_BLAS_LIBRARIES \"${MIPSOLVERS_BLAS_LIBRARIES}\")\n"
  "set(MIPSOLVERS_TP_PUBLIC_COMPILE_DEFINITIONS \"${_tp_public_defs}\")\n")
if(CMAKE_Fortran_COMPILER)
  file(APPEND "${_tp_manifest}"
    "set(MIPSOLVERS_TP_Fortran_COMPILER \"${CMAKE_Fortran_COMPILER}\")\n"
    "set(MIPSOLVERS_TP_Fortran_COMPILER_VERSION \"${CMAKE_Fortran_COMPILER_VERSION}\")\n")
endif()
install(FILES
  "${_tp_manifest}"
  "${CMAKE_CURRENT_BINARY_DIR}/mipsolversThirdPartyConfig.cmake"
  DESTINATION ${MIPSOLVERS_TP_CMAKEDIR})

message(STATUS
  "mipsolvers: THIRD_PARTY_ONLY install → ${CMAKE_INSTALL_PREFIX} "
  "(deps: ${_tp_enabled_deps})")

unset(_tp_enabled_deps)
unset(_tp_public_defs)
unset(_tp_manifest)
unset(_tp_msvc_runtime)
unset(_tp_build_config)
unset(_tp_is_multi_config)
unset(_tp_configuration_types)
unset(_tp_mkl_include_dir)
unset(_tp_mkl_library_names)
unset(_tp_mkl_version)
