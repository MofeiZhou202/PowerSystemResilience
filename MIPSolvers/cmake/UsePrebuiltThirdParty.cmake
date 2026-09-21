# cmake/UsePrebuiltThirdParty.cmake
# Consumer side of the prebuilt third-party mode.  Included from the top of
# cmake/Dependencies.cmake when MIPSOLVERS_USE_PREBUILT_THIRD_PARTY is AUTO/ON
# (and this is not itself a MIPSOLVERS_THIRD_PARTY_ONLY build).
#
# Locates third_party/install (produced by third_party/build_third_party.sh),
# validates its manifest.cmake against the current toolchain, then includes
# mipsolversThirdPartyConfig.cmake and sets the same result variables the
# in-tree vendored path would have set, so the rest of the build is unchanged.
#
# On success sets MIPSOLVERS_THIRD_PARTY_PREBUILT=ON; Dependencies.cmake then
# skips every Build*.cmake module and all vendored find/add logic.  On any
# problem: FATAL_ERROR when the mode is ON, loud WARNING + in-tree fallback
# when the mode is AUTO.

set(_MIPSOLVERS_TP_BUILD_SCRIPT "third_party/build_third_party.sh")
if(WIN32)
  set(_MIPSOLVERS_TP_BUILD_SCRIPT "third_party\\build_third_party.ps1")
endif()

get_filename_component(_MIPSOLVERS_TP_PREFIX
  "${MIPSOLVERS_PREBUILT_THIRD_PARTY_PREFIX}" ABSOLUTE
  BASE_DIR "${CMAKE_CURRENT_BINARY_DIR}")
set(_MIPSOLVERS_TP_DIR "")
foreach(_MIPSOLVERS_TP_CANDIDATE IN ITEMS
    "${_MIPSOLVERS_TP_PREFIX}"
    "${_MIPSOLVERS_TP_PREFIX}/lib/cmake/mipsolvers-third-party"
    "${_MIPSOLVERS_TP_PREFIX}/lib64/cmake/mipsolvers-third-party"
    "${_MIPSOLVERS_TP_PREFIX}/share/mipsolvers-third-party")
  if(EXISTS "${_MIPSOLVERS_TP_CANDIDATE}/mipsolversThirdPartyConfig.cmake")
    set(_MIPSOLVERS_TP_DIR "${_MIPSOLVERS_TP_CANDIDATE}")
    break()
  endif()
endforeach()
unset(_MIPSOLVERS_TP_CANDIDATE)
set(_MIPSOLVERS_TP_CONFIG "${_MIPSOLVERS_TP_DIR}/mipsolversThirdPartyConfig.cmake")
set(_MIPSOLVERS_TP_MANIFEST "${_MIPSOLVERS_TP_DIR}/manifest.cmake")

if(NOT EXISTS "${_MIPSOLVERS_TP_CONFIG}" OR NOT EXISTS "${_MIPSOLVERS_TP_MANIFEST}")
  if(MIPSOLVERS_USE_PREBUILT_THIRD_PARTY STREQUAL "ON")
    message(FATAL_ERROR
      "MIPSOLVERS_USE_PREBUILT_THIRD_PARTY=ON but no prebuilt third-party "
      "package was found under\n  ${_MIPSOLVERS_TP_PREFIX}\n"
      "Build it first with:  ${_MIPSOLVERS_TP_BUILD_SCRIPT}\n"
      "or set -DMIPSOLVERS_PREBUILT_THIRD_PARTY_PREFIX=<prefix>,\n"
      "or configure with -DMIPSOLVERS_USE_PREBUILT_THIRD_PARTY=AUTO/OFF.")
  endif()
  message(STATUS
    "mipsolvers: no prebuilt third-party package found under "
    "${_MIPSOLVERS_TP_PREFIX} "
    "— resolving third-party dependencies in-tree (vendored build)")
  return()
endif()

# ── Load and validate the manifest ────────────────────────────────────────────
include("${_MIPSOLVERS_TP_MANIFEST}")

set(_MIPSOLVERS_TP_MISMATCH "")
if(NOT "${MIPSOLVERS_TP_MANIFEST_VERSION}" STREQUAL "3")
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "manifest format: prebuilt '${MIPSOLVERS_TP_MANIFEST_VERSION}' vs required '3'")
endif()

# Format-3 Windows packages produced before the threading field existed used
# the sealed sequential profile by construction.
if(WIN32 AND MIPSOLVERS_TP_HAVE_MKL_PARDISO AND
   NOT MIPSOLVERS_TP_MKL_THREADING)
  set(MIPSOLVERS_TP_MKL_THREADING "SEQUENTIAL")
endif()

# The normal in-tree defaults require embedded HiGHS, SCIP, and Ipopt. Mirror
# those defaults here because prebuilt resolution runs before their option()
# declarations in Dependencies.cmake. A parent project can explicitly disable
# any of them before adding MIPSolvers.
set(_MIPSOLVERS_TP_WANT_HIGHS ON)
set(_MIPSOLVERS_TP_WANT_SCIP ON)
set(_MIPSOLVERS_TP_WANT_IPOPT ON)
if(DEFINED MIPSOLVERS_BUILD_EMBEDDED_HIGHS AND
   NOT MIPSOLVERS_BUILD_EMBEDDED_HIGHS)
  set(_MIPSOLVERS_TP_WANT_HIGHS OFF)
endif()
if(DEFINED MIPSOLVERS_BUILD_EMBEDDED_SCIP AND
   NOT MIPSOLVERS_BUILD_EMBEDDED_SCIP)
  set(_MIPSOLVERS_TP_WANT_SCIP OFF)
endif()
if(DEFINED MIPSOLVERS_BUILD_LOCAL_IPOPT AND
   NOT MIPSOLVERS_BUILD_LOCAL_IPOPT)
  set(_MIPSOLVERS_TP_WANT_IPOPT OFF)
endif()
if(_MIPSOLVERS_TP_WANT_HIGHS AND NOT MIPSOLVERS_TP_HAVE_HIGHS_LIB)
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "embedded HiGHS was requested but the prebuilt package does not contain it")
endif()
if(_MIPSOLVERS_TP_WANT_SCIP AND NOT MIPSOLVERS_TP_HAVE_SCIP_LIB)
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "embedded SCIP was requested but the prebuilt package does not contain it")
endif()
if(_MIPSOLVERS_TP_WANT_IPOPT AND NOT MIPSOLVERS_TP_HAVE_IPOPT)
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "embedded Ipopt was requested but the prebuilt package does not contain it")
endif()
if(MIPSOLVERS_TP_HAVE_IPOPT)
  if(DEFINED MIPSOLVERS_IPOPT_LINEAR_SOLVER AND
     NOT MIPSOLVERS_IPOPT_LINEAR_SOLVER STREQUAL "")
    string(TOLOWER "${MIPSOLVERS_IPOPT_LINEAR_SOLVER}"
      _MIPSOLVERS_TP_REQUESTED_IPOPT_LINEAR_SOLVER)
    string(TOLOWER "${MIPSOLVERS_TP_IPOPT_LINEAR_SOLVER}"
      _MIPSOLVERS_TP_PREBUILT_IPOPT_LINEAR_SOLVER)
    if(NOT _MIPSOLVERS_TP_REQUESTED_IPOPT_LINEAR_SOLVER MATCHES
       "^(mumps|pardisomkl)$")
      list(APPEND _MIPSOLVERS_TP_MISMATCH
        "invalid requested Ipopt linear solver '${MIPSOLVERS_IPOPT_LINEAR_SOLVER}'")
    elseif(NOT _MIPSOLVERS_TP_REQUESTED_IPOPT_LINEAR_SOLVER STREQUAL
           _MIPSOLVERS_TP_PREBUILT_IPOPT_LINEAR_SOLVER)
      list(APPEND _MIPSOLVERS_TP_MISMATCH
        "Ipopt linear solver: prebuilt '${MIPSOLVERS_TP_IPOPT_LINEAR_SOLVER}' vs requested '${MIPSOLVERS_IPOPT_LINEAR_SOLVER}'")
    endif()
  elseif(WIN32 AND NOT DEFINED MIPSOLVERS_IPOPT_LINEAR_SOLVER AND
         NOT MIPSOLVERS_TP_IPOPT_LINEAR_SOLVER STREQUAL "pardisomkl")
    list(APPEND _MIPSOLVERS_TP_MISMATCH
      "Ipopt linear solver: prebuilt '${MIPSOLVERS_TP_IPOPT_LINEAR_SOLVER}' vs Windows default 'pardisomkl'")
  endif()
endif()
if(WIN32 AND MIPSOLVERS_TP_HAVE_IPOPT AND
   MIPSOLVERS_TP_IPOPT_LINEAR_SOLVER STREQUAL "pardisomkl" AND
   NOT MIPSOLVERS_TP_HAVE_MKL_PARDISO)
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "Windows Ipopt package uses pardisomkl but contains no oneMKL payload")
endif()
if(WIN32 AND MIPSOLVERS_TP_HAVE_MKL_PARDISO AND
   DEFINED MIPSOLVERS_MKL_THREADING AND
   NOT MIPSOLVERS_MKL_THREADING STREQUAL "")
  string(TOUPPER "${MIPSOLVERS_MKL_THREADING}"
    _MIPSOLVERS_TP_REQUESTED_MKL_THREADING)
  if(NOT _MIPSOLVERS_TP_REQUESTED_MKL_THREADING STREQUAL
         MIPSOLVERS_TP_MKL_THREADING)
    list(APPEND _MIPSOLVERS_TP_MISMATCH
      "oneMKL threading: prebuilt '${MIPSOLVERS_TP_MKL_THREADING}' vs requested '${_MIPSOLVERS_TP_REQUESTED_MKL_THREADING}'")
  endif()
elseif(WIN32 AND MIPSOLVERS_TP_HAVE_MKL_PARDISO AND
       NOT MIPSOLVERS_TP_MKL_THREADING STREQUAL "SEQUENTIAL")
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "oneMKL threading: prebuilt '${MIPSOLVERS_TP_MKL_THREADING}' vs default 'SEQUENTIAL'")
endif()
if(NOT CMAKE_CXX_COMPILER_ID STREQUAL MIPSOLVERS_TP_CXX_COMPILER_ID)
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "CXX compiler id: prebuilt '${MIPSOLVERS_TP_CXX_COMPILER_ID}' vs current '${CMAKE_CXX_COMPILER_ID}'")
endif()
if(NOT CMAKE_CXX_COMPILER_VERSION STREQUAL MIPSOLVERS_TP_CXX_COMPILER_VERSION)
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "CXX compiler version: prebuilt '${MIPSOLVERS_TP_CXX_COMPILER_VERSION}' vs current '${CMAKE_CXX_COMPILER_VERSION}'")
endif()
if(NOT CMAKE_C_COMPILER_ID STREQUAL MIPSOLVERS_TP_C_COMPILER_ID)
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "C compiler id: prebuilt '${MIPSOLVERS_TP_C_COMPILER_ID}' vs current '${CMAKE_C_COMPILER_ID}'")
endif()
if(NOT CMAKE_C_COMPILER_VERSION STREQUAL MIPSOLVERS_TP_C_COMPILER_VERSION)
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "C compiler version: prebuilt '${MIPSOLVERS_TP_C_COMPILER_VERSION}' vs current '${CMAKE_C_COMPILER_VERSION}'")
endif()
if(NOT "${CMAKE_SYSTEM_NAME}" STREQUAL "${MIPSOLVERS_TP_SYSTEM_NAME}")
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "target system: prebuilt '${MIPSOLVERS_TP_SYSTEM_NAME}' vs current '${CMAKE_SYSTEM_NAME}'")
endif()
if(WIN32 AND NOT "${CMAKE_SYSTEM_VERSION}" STREQUAL "${MIPSOLVERS_TP_SYSTEM_VERSION}")
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "Windows target version: prebuilt '${MIPSOLVERS_TP_SYSTEM_VERSION}' vs current '${CMAKE_SYSTEM_VERSION}'")
endif()
if(NOT "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "${MIPSOLVERS_TP_SYSTEM_PROCESSOR}")
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "target architecture: prebuilt '${MIPSOLVERS_TP_SYSTEM_PROCESSOR}' vs current '${CMAKE_SYSTEM_PROCESSOR}'")
endif()
if(NOT "${CMAKE_SIZEOF_VOID_P}" STREQUAL "${MIPSOLVERS_TP_SIZEOF_VOID_P}")
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "pointer width: prebuilt '${MIPSOLVERS_TP_SIZEOF_VOID_P}' vs current '${CMAKE_SIZEOF_VOID_P}'")
endif()
if(NOT "${CMAKE_GENERATOR_PLATFORM}" STREQUAL "${MIPSOLVERS_TP_GENERATOR_PLATFORM}")
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "generator platform: prebuilt '${MIPSOLVERS_TP_GENERATOR_PLATFORM}' vs current '${CMAKE_GENERATOR_PLATFORM}'")
endif()
if(NOT "${CMAKE_GENERATOR_TOOLSET}" STREQUAL "${MIPSOLVERS_TP_GENERATOR_TOOLSET}")
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "generator toolset: prebuilt '${MIPSOLVERS_TP_GENERATOR_TOOLSET}' vs current '${CMAKE_GENERATOR_TOOLSET}'")
endif()
if(WIN32 AND NOT "${CMAKE_GENERATOR}" STREQUAL "${MIPSOLVERS_TP_GENERATOR}")
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "generator: prebuilt '${MIPSOLVERS_TP_GENERATOR}' vs current '${CMAKE_GENERATOR}'")
endif()
get_property(_MIPSOLVERS_CURRENT_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT "${_MIPSOLVERS_CURRENT_MULTI_CONFIG}" STREQUAL "${MIPSOLVERS_TP_MULTI_CONFIG}")
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "generator configuration model: prebuilt multi-config='${MIPSOLVERS_TP_MULTI_CONFIG}' vs current '${_MIPSOLVERS_CURRENT_MULTI_CONFIG}'")
elseif(_MIPSOLVERS_CURRENT_MULTI_CONFIG)
  list(FIND CMAKE_CONFIGURATION_TYPES "${MIPSOLVERS_TP_BUILD_TYPE}"
    _MIPSOLVERS_TP_CONFIG_INDEX)
  if(_MIPSOLVERS_TP_CONFIG_INDEX EQUAL -1)
    list(APPEND _MIPSOLVERS_TP_MISMATCH
      "build configuration: prebuilt '${MIPSOLVERS_TP_BUILD_TYPE}' is not available in '${CMAKE_CONFIGURATION_TYPES}'")
  endif()
endif()
if(WIN32 AND NOT "${CMAKE_CXX_COMPILER_ARCHITECTURE_ID}" STREQUAL "${MIPSOLVERS_TP_CXX_COMPILER_ARCHITECTURE_ID}")
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "compiler architecture: prebuilt '${MIPSOLVERS_TP_CXX_COMPILER_ARCHITECTURE_ID}' vs current '${CMAKE_CXX_COMPILER_ARCHITECTURE_ID}'")
endif()
if(MSVC)
  set(_MIPSOLVERS_CURRENT_MSVC_RUNTIME "${CMAKE_MSVC_RUNTIME_LIBRARY}")
  if(NOT _MIPSOLVERS_CURRENT_MSVC_RUNTIME)
    set(_MIPSOLVERS_CURRENT_MSVC_RUNTIME "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
  endif()
  if(NOT "${MSVC_VERSION}" STREQUAL "${MIPSOLVERS_TP_MSVC_VERSION}")
    list(APPEND _MIPSOLVERS_TP_MISMATCH
      "MSVC toolset version: prebuilt '${MIPSOLVERS_TP_MSVC_VERSION}' vs current '${MSVC_VERSION}'")
  endif()
  if(NOT "${_MIPSOLVERS_CURRENT_MSVC_RUNTIME}" STREQUAL "${MIPSOLVERS_TP_MSVC_RUNTIME_LIBRARY}")
    list(APPEND _MIPSOLVERS_TP_MISMATCH
      "MSVC runtime: prebuilt '${MIPSOLVERS_TP_MSVC_RUNTIME_LIBRARY}' vs current '${_MIPSOLVERS_CURRENT_MSVC_RUNTIME}'")
  endif()
endif()
if(NOT _MIPSOLVERS_CURRENT_MULTI_CONFIG AND CMAKE_BUILD_TYPE AND
   NOT "${CMAKE_BUILD_TYPE}" STREQUAL "${MIPSOLVERS_TP_BUILD_TYPE}")
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "build configuration: prebuilt '${MIPSOLVERS_TP_BUILD_TYPE}' vs current '${CMAKE_BUILD_TYPE}'")
endif()
if(APPLE AND NOT "${CMAKE_OSX_SYSROOT}" STREQUAL "${MIPSOLVERS_TP_OSX_SYSROOT}")
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "macOS SDK (CMAKE_OSX_SYSROOT): prebuilt '${MIPSOLVERS_TP_OSX_SYSROOT}' vs current '${CMAKE_OSX_SYSROOT}'")
endif()
if(NOT "${MIPSOLVERS_EIGEN_MAX_ALIGN_BYTES}" STREQUAL "${MIPSOLVERS_TP_EIGEN_MAX_ALIGN_BYTES}")
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "EIGEN_MAX_ALIGN_BYTES: prebuilt '${MIPSOLVERS_TP_EIGEN_MAX_ALIGN_BYTES}' vs current '${MIPSOLVERS_EIGEN_MAX_ALIGN_BYTES}'")
endif()
if(NOT "${MIPSOLVERS_EIGEN_VECTORIZE}" STREQUAL "${MIPSOLVERS_TP_EIGEN_VECTORIZE}")
  list(APPEND _MIPSOLVERS_TP_MISMATCH
    "Eigen vectorization: prebuilt '${MIPSOLVERS_TP_EIGEN_VECTORIZE}' vs current '${MIPSOLVERS_EIGEN_VECTORIZE}'")
endif()

if(_MIPSOLVERS_TP_MISMATCH)
  string(REPLACE ";" "\n  " _MIPSOLVERS_TP_MISMATCH_TEXT "${_MIPSOLVERS_TP_MISMATCH}")
  if(MIPSOLVERS_USE_PREBUILT_THIRD_PARTY STREQUAL "ON")
    message(FATAL_ERROR
      "MIPSOLVERS_USE_PREBUILT_THIRD_PARTY=ON but the prebuilt third-party "
      "package does not match the current toolchain:\n"
      "  ${_MIPSOLVERS_TP_MISMATCH_TEXT}\n"
      "Rebuild it with ${_MIPSOLVERS_TP_BUILD_SCRIPT}, or configure with "
      "-DMIPSOLVERS_USE_PREBUILT_THIRD_PARTY=OFF to build in-tree.")
  endif()
  message(WARNING
    "mipsolvers: ignoring the prebuilt third-party package at "
    "${_MIPSOLVERS_TP_PREFIX} — toolchain/ABI mismatch:\n"
    "  ${_MIPSOLVERS_TP_MISMATCH_TEXT}\n"
    "Falling back to the in-tree vendored build "
    "(rebuild the package with ${_MIPSOLVERS_TP_BUILD_SCRIPT} to re-enable).")
  return()
endif()

# A third-party-only install contains one concrete configuration. Restrict a
# multi-config consumer to it so `--config Debug` cannot silently link Release
# libraries (notably /MD versus /MDd on MSVC).
if(_MIPSOLVERS_CURRENT_MULTI_CONFIG)
  set(CMAKE_CONFIGURATION_TYPES "${MIPSOLVERS_TP_BUILD_TYPE}" CACHE STRING
    "Configurations compatible with the prebuilt MIPSolvers dependencies" FORCE)
  message(STATUS
    "mipsolvers: multi-config generator restricted to prebuilt configuration "
    "${MIPSOLVERS_TP_BUILD_TYPE}")
endif()

# ── Load the prebuilt targets ─────────────────────────────────────────────────
include("${_MIPSOLVERS_TP_CONFIG}")

# ── Re-apply the recorded result variables ────────────────────────────────────
# These mirror exactly what the in-tree path (Dependencies.cmake + the
# Build*.cmake modules) sets; CMakeLists.txt consumes them unchanged and
# re-derives the matching MIPSOLVERS_HAVE_*/HACDCPF_HAVE_* PUBLIC compile
# definitions from them.
set(MIPSOLVERS_BLAS_LIBRARIES "${MIPSOLVERS_TP_BLAS_LIBRARIES}")

set(MIPSOLVERS_HIGHS_LIB_SOURCE "none")
set(MIPSOLVERS_HAVE_HIGHS_LIB "${MIPSOLVERS_TP_HAVE_HIGHS_LIB}")
if(MIPSOLVERS_HAVE_HIGHS_LIB)
  set(MIPSOLVERS_HIGHS_LIB_SOURCE "prebuilt")
endif()

set(MIPSOLVERS_SCIP_LIB_SOURCE "none")
set(MIPSOLVERS_HAVE_SCIP_LIB "${MIPSOLVERS_TP_HAVE_SCIP_LIB}")
if(MIPSOLVERS_HAVE_SCIP_LIB)
  set(MIPSOLVERS_SCIP_LIB_SOURCE "prebuilt")
endif()

set(MIPSOLVERS_HAVE_IPOPT "${MIPSOLVERS_TP_HAVE_IPOPT}")
if(MIPSOLVERS_HAVE_IPOPT)
  set(MIPSOLVERS_IPOPT_INCLUDE_DIRS ${MIPSOLVERS_TP_IPOPT_INCLUDE_DIRS})
  set(MIPSOLVERS_IPOPT_LIBRARIES ipopt_local)
endif()
set(MIPSOLVERS_HAVE_MKL_PARDISO "${MIPSOLVERS_TP_HAVE_MKL_PARDISO}")
set(MIPSOLVERS_MKL_INCLUDE_DIRS "${MIPSOLVERS_TP_MKL_INCLUDE_DIRS}")
set(MIPSOLVERS_MKL_LIBRARIES "${MIPSOLVERS_TP_MKL_LIBRARIES}")
if(WIN32)
  set(MIPSOLVERS_MKL_THREADING "${MIPSOLVERS_TP_MKL_THREADING}" CACHE STRING
    "Windows oneMKL threading layer for PARDISO (SEQUENTIAL or INTEL)" FORCE)
endif()
set(MIPSOLVERS_MKL_RUNTIME_DLLS "${MIPSOLVERS_TP_MKL_RUNTIME_DLLS}"
  CACHE INTERNAL
  "Runtime DLLs required by the selected oneMKL threading layer" FORCE)

set(MIPSOLVERS_HAVE_CHOLMOD "${MIPSOLVERS_TP_HAVE_CHOLMOD}")
set(MIPSOLVERS_CHOLMOD_INCLUDE_DIRS "")
set(MIPSOLVERS_HAVE_SUITESPARSE "${MIPSOLVERS_TP_HAVE_SUITESPARSE}")
set(MIPSOLVERS_SUITESPARSE_LIBRARIES "")
set(MIPSOLVERS_SUITESPARSE_INCLUDE_DIRS "")
set(MIPSOLVERS_SUITESPARSE_VENDORED OFF)
set(MIPSOLVERS_HAVE_UMFPACK "${MIPSOLVERS_TP_HAVE_UMFPACK}")
set(MIPSOLVERS_HAVE_KLU "${MIPSOLVERS_TP_HAVE_KLU}")
if(MIPSOLVERS_HAVE_CHOLMOD)
  set(MIPSOLVERS_CHOLMOD_INCLUDE_DIRS ${MIPSOLVERS_TP_CHOLMOD_INCLUDE_DIRS})
  set(MIPSOLVERS_SUITESPARSE_VENDORED ON)
  set(MIPSOLVERS_SUITESPARSE_INCLUDE_DIRS ${MIPSOLVERS_TP_SUITESPARSE_INCLUDE_DIRS})
endif()

set(MIPSOLVERS_FMT_VENDORED OFF)
if("fmt" IN_LIST MIPSOLVERS_TP_ENABLED_DEPS)
  set(MIPSOLVERS_FMT_VENDORED ON)
endif()
set(Eigen3_FOUND TRUE)
set(Catch2_FOUND TRUE)

# System numeric backends are not part of the prebuilt package. Gurobi and the
# bundled PaPILO headers are resolved after this file returns.
foreach(_MIPSOLVERS_TP_OPT_OPT SUPERLU)
  if(MIPSOLVERS_USE_${_MIPSOLVERS_TP_OPT_OPT})
    message(WARNING
      "mipsolvers: MIPSOLVERS_USE_${_MIPSOLVERS_TP_OPT_OPT}=ON is ignored in "
      "prebuilt third-party mode; configure with "
      "-DMIPSOLVERS_USE_PREBUILT_THIRD_PARTY=OFF to enable it.")
  endif()
endforeach()
unset(_MIPSOLVERS_TP_OPT_OPT)
set(MIPSOLVERS_HAVE_SUPERLU OFF)
set(MIPSOLVERS_SUPERLU_INCLUDE_DIR "")
set(MIPSOLVERS_SUPERLU_LIBRARIES "")
set(MIPSOLVERS_CONSUMER_NEEDS_BREW_MUMPS OFF)

set(MIPSOLVERS_THIRD_PARTY_PREBUILT ON)

message(STATUS
  "mipsolvers: using prebuilt third-party package from ${_MIPSOLVERS_TP_PREFIX} "
  "(deps: ${MIPSOLVERS_TP_ENABLED_DEPS})")
message(STATUS
  "mipsolvers: prebuilt manifest: ${MIPSOLVERS_TP_CXX_COMPILER_ID} "
  "${MIPSOLVERS_TP_CXX_COMPILER_VERSION}, ${MIPSOLVERS_TP_BUILD_TYPE}, "
  "BLAS/LAPACK = ${MIPSOLVERS_TP_BLAS_SOURCE}")
message(STATUS
  "mipsolvers: skipped in-tree modules: BuildRefLAPACK.cmake, BuildHiGHS.cmake, "
  "BuildSCIP.cmake, BuildIpopt.cmake, BuildMUMPS.cmake, BuildCHOLMOD.cmake and "
  "the vendored fmt/Catch2/Eigen/nlohmann_json setup")

unset(_MIPSOLVERS_TP_DIR)
unset(_MIPSOLVERS_TP_PREFIX)
unset(_MIPSOLVERS_TP_CONFIG)
unset(_MIPSOLVERS_TP_MANIFEST)
unset(_MIPSOLVERS_TP_MISMATCH)
unset(_MIPSOLVERS_TP_CONFIG_INDEX)
unset(_MIPSOLVERS_CURRENT_MULTI_CONFIG)
unset(_MIPSOLVERS_CURRENT_MSVC_RUNTIME)
unset(_MIPSOLVERS_CURRENT_MULTI_CONFIG)
unset(_MIPSOLVERS_TP_BUILD_SCRIPT)
unset(_MIPSOLVERS_TP_WANT_HIGHS)
unset(_MIPSOLVERS_TP_WANT_SCIP)
unset(_MIPSOLVERS_TP_WANT_IPOPT)
unset(_MIPSOLVERS_TP_REQUESTED_IPOPT_LINEAR_SOLVER)
unset(_MIPSOLVERS_TP_PREBUILT_IPOPT_LINEAR_SOLVER)
