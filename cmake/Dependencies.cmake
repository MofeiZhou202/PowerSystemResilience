# cmake/Dependencies.cmake
# All dependencies resolved locally — no network downloads.
# HiGHS, SCIP, and Ipopt are compiled from the local source directories.

foreach(_MIPSOLVERS_FORBIDDEN_SOLVER_CACHE
    MIPSOLVERS_HIGHS_EXECUTABLE
    MIPSOLVERS_IPOPT_EXECUTABLE
    MIPSOLVERS_SCIP_EXECUTABLE
    MIPSOLVERS_IPOPT_INCLUDE_DIR
    MIPSOLVERS_IPOPT_LIBRARY)
  unset(${_MIPSOLVERS_FORBIDDEN_SOLVER_CACHE} CACHE)
  unset(${_MIPSOLVERS_FORBIDDEN_SOLVER_CACHE})
endforeach()
unset(_MIPSOLVERS_FORBIDDEN_SOLVER_CACHE)

# Optional hermetic oneMKL bundle. A non-empty value is authoritative: the
# resolver must use that bundle and must not fall back to a machine install.
set(MIPSOLVERS_MKL_ROOT "" CACHE PATH
  "Root of a local static oneMKL bundle (include/, lib/, licensing/)")
set(MIPSOLVERS_MKL_RUNTIME_DLLS "" CACHE INTERNAL
  "Runtime DLLs required by the selected oneMKL threading layer" FORCE)
set(MIPSOLVERS_MKL_THREADING "SEQUENTIAL" CACHE STRING
  "oneMKL threading layer for PARDISO (SEQUENTIAL or INTEL)")
set_property(CACHE MIPSOLVERS_MKL_THREADING PROPERTY STRINGS SEQUENTIAL INTEL)
string(TOUPPER "${MIPSOLVERS_MKL_THREADING}" MIPSOLVERS_MKL_THREADING)
if(NOT MIPSOLVERS_MKL_THREADING MATCHES "^(SEQUENTIAL|INTEL)$")
  message(FATAL_ERROR
    "MIPSOLVERS_MKL_THREADING must be SEQUENTIAL or INTEL "
    "(got '${MIPSOLVERS_MKL_THREADING}').")
endif()
if(MIPSOLVERS_MKL_THREAD_LIB)
  get_filename_component(_MIPSOLVERS_MKL_THREAD_BASENAME
    "${MIPSOLVERS_MKL_THREAD_LIB}" NAME_WE)
  if((MIPSOLVERS_MKL_THREADING STREQUAL "INTEL" AND
      NOT _MIPSOLVERS_MKL_THREAD_BASENAME STREQUAL "mkl_intel_thread") OR
     (MIPSOLVERS_MKL_THREADING STREQUAL "SEQUENTIAL" AND
      NOT _MIPSOLVERS_MKL_THREAD_BASENAME STREQUAL "mkl_sequential"))
    unset(MIPSOLVERS_MKL_THREAD_LIB CACHE)
    unset(MIPSOLVERS_MKL_THREAD_LIB)
  endif()
  unset(_MIPSOLVERS_MKL_THREAD_BASENAME)
endif()
if(NOT MIPSOLVERS_MKL_ROOT AND DEFINED ENV{MIPSOLVERS_MKL_ROOT})
  set(MIPSOLVERS_MKL_ROOT "$ENV{MIPSOLVERS_MKL_ROOT}" CACHE PATH
    "Root of a local static oneMKL bundle (include/, lib/, licensing/)" FORCE)
endif()
if(NOT MIPSOLVERS_MKL_ROOT AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/oneapi-mkl/manifest.cmake")
  set(MIPSOLVERS_MKL_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/oneapi-mkl" CACHE PATH
    "Root of a local static oneMKL bundle (include/, lib/, licensing/)" FORCE)
endif()
if(MIPSOLVERS_MKL_ROOT)
  # A selected bundle is authoritative even when this build directory was
  # previously configured against a system oneAPI installation.
  foreach(_MIPSOLVERS_MKL_CACHE IN ITEMS
      MIPSOLVERS_MKL_INCLUDE_DIR MIPSOLVERS_MKL_LP64_LIB
      MIPSOLVERS_MKL_THREAD_LIB MIPSOLVERS_MKL_CORE_LIB)
    unset(${_MIPSOLVERS_MKL_CACHE} CACHE)
    unset(${_MIPSOLVERS_MKL_CACHE})
  endforeach()
  unset(_MIPSOLVERS_MKL_CACHE)
  set(MIPSOLVERS_HAVE_MKL_PARDISO OFF)
  set(MIPSOLVERS_MKL_INCLUDE_DIRS "")
  set(MIPSOLVERS_MKL_LIBRARIES "")
endif()

# ── Prebuilt third-party package (AUTO/ON/OFF) ───────────────────────────────
# When third_party/install (built by third_party/build_third_party.sh with
# -DMIPSOLVERS_THIRD_PARTY_ONLY=ON) is present and toolchain-compatible, load
# it and skip every Build*.cmake module and all vendored find/add logic below.
# MIPSOLVERS_THIRD_PARTY_PREBUILT is read by the top-level CMakeLists.txt to
# skip the install rules that would re-export the (imported) vendored targets.
set(MIPSOLVERS_USE_PREBUILT_THIRD_PARTY "AUTO" CACHE STRING
  "Use a prebuilt third-party package (AUTO/ON/OFF)")
set_property(CACHE MIPSOLVERS_USE_PREBUILT_THIRD_PARTY
  PROPERTY STRINGS AUTO ON OFF)
set(MIPSOLVERS_PREBUILT_THIRD_PARTY_PREFIX
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/install" CACHE PATH
  "Prefix containing the prebuilt mipsolvers third-party package")
set(MIPSOLVERS_THIRD_PARTY_PREBUILT OFF)
if(NOT MIPSOLVERS_THIRD_PARTY_ONLY AND
   NOT MIPSOLVERS_USE_PREBUILT_THIRD_PARTY STREQUAL "OFF")
  if(NOT MIPSOLVERS_USE_PREBUILT_THIRD_PARTY MATCHES "^(AUTO|ON)$")
    message(FATAL_ERROR
      "MIPSOLVERS_USE_PREBUILT_THIRD_PARTY must be AUTO, ON or OFF "
      "(got '${MIPSOLVERS_USE_PREBUILT_THIRD_PARTY}').")
  endif()
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/UsePrebuiltThirdParty.cmake")
endif()
if(MIPSOLVERS_THIRD_PARTY_PREBUILT)
  include(${CMAKE_CURRENT_LIST_DIR}/DetectOptionalAdapters.cmake)
  return()
endif()

# ── BLAS/LAPACK resolution (unified) ─────────────────────────────────────────
# Single variable — MIPSOLVERS_BLAS_LIBRARIES — consumed by BuildCHOLMOD.cmake,
# BuildIpopt.cmake, and BuildMUMPS.cmake instead of their own ad-hoc probes.
# Resolution order:
#   macOS : Accelerate framework → vendored reference LAPACK
#   Linux : system BLAS/LAPACK  → vendored reference LAPACK (third_party/lapack)
#   Windows: system BLAS/LAPACK when available; MKL/Pardiso is the default
#            Ipopt path, while the vendored fallback is explicit opt-in
# MIPSOLVERS_USE_VENDORED_BLAS (default ON) merely *allows* the vendored
# fallback; MIPSOLVERS_FORCE_VENDORED_BLAS (hidden, for testing) skips the
# Accelerate/system probes entirely.  Runs before the Ipopt/MUMPS/CHOLMOD
# includes below so the variable is set when those modules consume it.
set(MIPSOLVERS_BLAS_LIBRARIES "")
set(_MIPSOLVERS_BLAS_SOURCE "none")
option(MIPSOLVERS_USE_VENDORED_BLAS
  "Allow the vendored reference BLAS/LAPACK (third_party/lapack) as a fallback" ON)
option(MIPSOLVERS_FORCE_VENDORED_BLAS
  "Force the vendored reference BLAS/LAPACK (skip Accelerate/system probes)" OFF)

if(NOT MIPSOLVERS_FORCE_VENDORED_BLAS)
  if(APPLE)
    # Accelerate is always present on macOS and is the preferred BLAS/LAPACK.
    # Single string: separate list items would make CMake render the bare name
    # "Accelerate" as "-lAccelerate".
    set(MIPSOLVERS_BLAS_LIBRARIES "-framework Accelerate")
    set(_MIPSOLVERS_BLAS_SOURCE "Accelerate framework")
  else()
    find_package(BLAS QUIET)
    find_package(LAPACK QUIET)
    if(BLAS_FOUND AND LAPACK_FOUND)
      set(MIPSOLVERS_BLAS_LIBRARIES ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
      set(_MIPSOLVERS_BLAS_SOURCE "system BLAS/LAPACK")
    endif()
  endif()
endif()

if(WIN32 AND NOT MIPSOLVERS_BLAS_LIBRARIES AND
   NOT MIPSOLVERS_FORCE_VENDORED_BLAS)
  message(STATUS
    "mipsolvers: no standalone Windows BLAS/LAPACK found; the default embedded "
    "Ipopt build uses Intel MKL/Pardiso. For the MUMPS backend, provide a "
    "system BLAS/LAPACK or set MIPSOLVERS_FORCE_VENDORED_BLAS=ON with Intel "
    "oneAPI ifx/ifort or MinGW gfortran.")
endif()

if(NOT MIPSOLVERS_BLAS_LIBRARIES AND
   MIPSOLVERS_USE_VENDORED_BLAS AND
   (NOT WIN32 OR MIPSOLVERS_FORCE_VENDORED_BLAS) AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/lapack/BLAS/SRC/dgemm.f")
  # Windows keeps the pre-existing tolerance for "no BLAS anywhere" (CHOLMOD
  # degrades to NSUPERNODAL, Ipopt uses MKL Pardiso) rather than hard
  # requiring a Fortran compiler for the vendored fallback; the hidden
  # FORCE flag overrides that tolerance for testing.
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BuildRefLAPACK.cmake")
  set(MIPSOLVERS_BLAS_LIBRARIES reflapack_vendored)
  set(_MIPSOLVERS_BLAS_SOURCE "vendored reference BLAS/LAPACK (third_party/lapack)")
endif()

if(NOT MIPSOLVERS_BLAS_LIBRARIES AND NOT WIN32)
  # Non-Windows embedded Ipopt hard-requires LAPACK (IPOPT_HAS_LAPACK=1), so
  # having no BLAS/LAPACK at all is a configure-time error.  On Windows the
  # MKL Pardiso path supplies BLAS/LAPACK for Ipopt and CHOLMOD degrades
  # gracefully (NSUPERNODAL), so an empty value is tolerated there.
  if(MIPSOLVERS_USE_VENDORED_BLAS)
    message(FATAL_ERROR
      "No BLAS/LAPACK available: no system BLAS/LAPACK found and the vendored "
      "fallback is missing (expected third_party/lapack/BLAS/SRC/dgemm.f). "
      "Restore the vendored third_party/lapack tree or install a system "
      "BLAS/LAPACK.")
  else()
    message(FATAL_ERROR
      "No system BLAS/LAPACK found and MIPSOLVERS_USE_VENDORED_BLAS=OFF. "
      "Install a system BLAS/LAPACK or re-enable the vendored fallback.")
  endif()
endif()
message(STATUS "mipsolvers: BLAS/LAPACK = ${_MIPSOLVERS_BLAS_SOURCE}")

# ── HiGHS in-process library ─────────────────────────────────────────────────
set(MIPSOLVERS_HAVE_HIGHS_LIB OFF)
set(MIPSOLVERS_HIGHS_LIB_SOURCE "none")
option(MIPSOLVERS_BUILD_EMBEDDED_HIGHS
  "Build embedded HiGHS source (highs/) as an in-process library" ON)

# 1. Use local src-only copy (highs/ contains HiGHS src/ files)
if(NOT MIPSOLVERS_HAVE_HIGHS_LIB AND
   MIPSOLVERS_BUILD_EMBEDDED_HIGHS AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/highs/CMakeLists.txt" AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/highs/HConfig.h.in" AND
   NOT TARGET highs)
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BuildHiGHS.cmake")
  if(TARGET highs)
    set(MIPSOLVERS_HAVE_HIGHS_LIB ON)
    set(MIPSOLVERS_HIGHS_LIB_SOURCE "local-source")
  endif()
endif()

# 2. Fallback: full HiGHS project tree with its own CMakeLists.txt
if(NOT MIPSOLVERS_HAVE_HIGHS_LIB AND
   MIPSOLVERS_BUILD_EMBEDDED_HIGHS AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/HiGHS/CMakeLists.txt" AND
   NOT TARGET highs)

  set(_MIPSOLVERS_HIGHS_VARS
    FAST_BUILD BUILD_CXX BUILD_CXX_EXE BUILD_CXX_EXAMPLE BUILD_EXAMPLES
    BUILD_TESTING PYTHON_BUILD_SETUP FORTRAN CSHARP HIPO BUILD_OPENBLAS
    CUPDLP_GPU CUPDLP_FIND_CUDA HIGHS_GPU_LIB BUILD_SHARED_LIBS ZLIB
    CMAKE_CXX_STANDARD CMAKE_CXX_STANDARD_REQUIRED CMAKE_CXX_EXTENSIONS)

  foreach(_v IN LISTS _MIPSOLVERS_HIGHS_VARS)
    if(DEFINED ${_v})
      set(_MIPSOLVERS_SAVE_${_v}_DEF TRUE)
      set(_MIPSOLVERS_SAVE_${_v}_VAL "${${_v}}")
    else()
      set(_MIPSOLVERS_SAVE_${_v}_DEF FALSE)
    endif()
  endforeach()

  set(FAST_BUILD ON)
  set(BUILD_CXX ON)
  set(BUILD_CXX_EXE OFF)
  set(BUILD_CXX_EXAMPLE OFF)
  set(BUILD_EXAMPLES OFF)
  set(BUILD_TESTING OFF)
  set(PYTHON_BUILD_SETUP OFF)
  set(FORTRAN OFF)
  set(CSHARP OFF)
  set(HIPO OFF)
  set(BUILD_OPENBLAS OFF)
  set(CUPDLP_GPU OFF)
  set(CUPDLP_FIND_CUDA OFF)
  set(HIGHS_GPU_LIB OFF)
  set(BUILD_SHARED_LIBS OFF)
  set(ZLIB OFF)

  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/HiGHS"
                   "${CMAKE_CURRENT_BINARY_DIR}/_deps/embedded_highs"
                   EXCLUDE_FROM_ALL)

  foreach(_v IN LISTS _MIPSOLVERS_HIGHS_VARS)
    if(_MIPSOLVERS_SAVE_${_v}_DEF)
      set(${_v} "${_MIPSOLVERS_SAVE_${_v}_VAL}")
    else()
      unset(${_v})
    endif()
    unset(_MIPSOLVERS_SAVE_${_v}_DEF)
    unset(_MIPSOLVERS_SAVE_${_v}_VAL)
  endforeach()
  unset(_MIPSOLVERS_HIGHS_VARS)

  if(TARGET highs AND NOT TARGET highs::highs)
    add_library(highs::highs ALIAS highs)
  endif()
  if(TARGET highs::highs)
    set(MIPSOLVERS_HAVE_HIGHS_LIB ON)
    set(MIPSOLVERS_HIGHS_LIB_SOURCE "full-embedded-source")
    message(STATUS "mipsolvers: building full embedded HiGHS from HiGHS/")
  endif()
endif()

if(MIPSOLVERS_BUILD_EMBEDDED_HIGHS AND NOT MIPSOLVERS_HAVE_HIGHS_LIB)
  message(FATAL_ERROR
    "MIPSOLVERS_BUILD_EMBEDDED_HIGHS=ON, but no embedded HiGHS target was "
    "created from highs/ or HiGHS/. This project must use the customized "
    "in-repository HiGHS source; system HiGHS is not used.")
endif()

# ── SCIP in-process library ───────────────────────────────────────────────────
set(MIPSOLVERS_HAVE_SCIP_LIB OFF)
set(MIPSOLVERS_SCIP_LIB_SOURCE "none")
option(MIPSOLVERS_BUILD_EMBEDDED_SCIP
  "Build embedded SCIP source (scip/) as an in-process library" ON)

if(MIPSOLVERS_BUILD_EMBEDDED_SCIP AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scip/CMakeLists.txt" AND
   NOT TARGET libscip)
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BuildSCIP.cmake")
  if(TARGET libscip)
    if(NOT TARGET SCIP::libscip)
      add_library(SCIP::libscip ALIAS libscip)
    endif()
    set(MIPSOLVERS_HAVE_SCIP_LIB ON)
    set(MIPSOLVERS_SCIP_LIB_SOURCE "local-source")
  endif()
endif()

if(MIPSOLVERS_BUILD_EMBEDDED_SCIP AND NOT MIPSOLVERS_HAVE_SCIP_LIB)
  message(FATAL_ERROR
    "MIPSOLVERS_BUILD_EMBEDDED_SCIP=ON, but no embedded SCIP target was "
    "created from scip/. This project must use the customized in-repository "
    "SCIP source; system SCIP is not used.")
endif()

# ── Ipopt NLP solver ──────────────────────────────────────────────────────────
set(MIPSOLVERS_HAVE_IPOPT OFF)
# Default for the consumer config; BuildMUMPS.cmake flips this ON only when a
# Homebrew MUMPS is actually used (Apple opt-out path).
set(MIPSOLVERS_CONSUMER_NEEDS_BREW_MUMPS OFF)
set(_MIPSOLVERS_BUILD_LOCAL_IPOPT_DEFAULT ON)
option(MIPSOLVERS_BUILD_LOCAL_IPOPT
  "Build embedded Ipopt source (ipopt/) from this repository"
  ${_MIPSOLVERS_BUILD_LOCAL_IPOPT_DEFAULT})
unset(_MIPSOLVERS_BUILD_LOCAL_IPOPT_DEFAULT)

if(MIPSOLVERS_BUILD_LOCAL_IPOPT AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/ipopt/Interfaces/IpIpoptApplication.hpp" AND
   NOT TARGET ipopt_local)
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BuildIpopt.cmake")
  if(TARGET ipopt_local)
    set(MIPSOLVERS_HAVE_IPOPT ON)
    set(MIPSOLVERS_IPOPT_INCLUDE_DIRS
      "${CMAKE_CURRENT_SOURCE_DIR}/ipopt/Interfaces"
      "${CMAKE_CURRENT_SOURCE_DIR}/ipopt/Common"
      "${CMAKE_CURRENT_SOURCE_DIR}/ipopt/LinAlg"
      "${CMAKE_CURRENT_SOURCE_DIR}/ipopt/Algorithm"
      "${CMAKE_CURRENT_BINARY_DIR}/_deps/embedded_ipopt")
    set(MIPSOLVERS_IPOPT_LIBRARIES ipopt_local)
  endif()
endif()

if(MIPSOLVERS_BUILD_LOCAL_IPOPT AND NOT MIPSOLVERS_HAVE_IPOPT)
  message(FATAL_ERROR
    "MIPSOLVERS_BUILD_LOCAL_IPOPT=ON, but embedded Ipopt was not built from "
    "ipopt/. This project must use the customized in-repository Ipopt source; "
    "system Ipopt is not used.")
elseif(NOT MIPSOLVERS_BUILD_LOCAL_IPOPT)
  message(STATUS
    "mipsolvers: embedded Ipopt disabled; TNLP bridge will be unavailable")
endif()

# ── Optional solver adapters ─────────────────────────────────────────────────
include(${CMAKE_CURRENT_LIST_DIR}/DetectOptionalAdapters.cmake)

# ── SuiteSparse (optional) ───────────────────────────────────────────────────
option(MIPSOLVERS_USE_SUITESPARSE "Enable SuiteSparse backends when available" ON)
set(MIPSOLVERS_HAVE_SUITESPARSE OFF)
set(MIPSOLVERS_SUITESPARSE_LIBRARIES "")
# ON when SuiteSparse support comes from the exported in-tree vendored targets
# (umfpack_vendored/klu_vendored) — the consumer config then skips system
# SuiteSparse rediscovery.
set(MIPSOLVERS_SUITESPARSE_VENDORED OFF)

# Build vendored CHOLMOD/UMFPACK/KLU only when the SuiteSparse family is
# enabled. This keeps the Eigen-only fallback free of SuiteSparse headers,
# targets, and feature macros.
if(MIPSOLVERS_USE_SUITESPARSE)
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BuildCHOLMOD.cmake")
else()
  set(MIPSOLVERS_HAVE_CHOLMOD OFF)
  set(MIPSOLVERS_CHOLMOD_INCLUDE_DIRS "")
  message(STATUS "mipsolvers: SuiteSparse disabled")
endif()

# Vendored SuiteSparse (CHOLMOD/UMFPACK/KLU from in-tree sources via
# cmake/BuildCHOLMOD.cmake) is the authoritative offline path — when it is
# active we do not look for a system/Homebrew SuiteSparse at all.
if(MIPSOLVERS_USE_SUITESPARSE AND MIPSOLVERS_HAVE_CHOLMOD)
  set(MIPSOLVERS_HAVE_SUITESPARSE ON)
  set(MIPSOLVERS_SUITESPARSE_VENDORED ON)
  message(STATUS "mipsolvers: SuiteSparse via vendored in-tree sources (offline)")
elseif(MIPSOLVERS_USE_SUITESPARSE)
  set(MIPSOLVERS_HAVE_UMFPACK OFF)
  set(MIPSOLVERS_HAVE_KLU OFF)
  set(_SS_HINTS
    $ENV{SUITESPARSE_ROOT}
    /opt/homebrew/opt/suite-sparse
    /opt/homebrew
    /usr/local/opt/suite-sparse
    /usr/local
    /usr
    "C:/vcpkg/installed/x64-windows"
    "C:/SuiteSparse")
  find_path(MIPSOLVERS_SUITESPARSE_INCLUDE_DIR NAMES umfpack.h klu.h
    HINTS ${_SS_HINTS} PATH_SUFFIXES include include/suitesparse)
  find_library(MIPSOLVERS_UMFPACK_LIBRARY NAMES umfpack HINTS ${_SS_HINTS} PATH_SUFFIXES lib)
  find_library(MIPSOLVERS_KLU_LIBRARY NAMES klu HINTS ${_SS_HINTS} PATH_SUFFIXES lib)
  if(MIPSOLVERS_SUITESPARSE_INCLUDE_DIR)
    set(MIPSOLVERS_HAVE_SUITESPARSE ON)
    list(APPEND MIPSOLVERS_SUITESPARSE_INCLUDE_DIRS ${MIPSOLVERS_SUITESPARSE_INCLUDE_DIR})
    if(MIPSOLVERS_UMFPACK_LIBRARY)
      set(MIPSOLVERS_HAVE_UMFPACK ON)
      list(APPEND MIPSOLVERS_SUITESPARSE_LIBRARIES ${MIPSOLVERS_UMFPACK_LIBRARY})
    endif()
    if(MIPSOLVERS_KLU_LIBRARY)
      set(MIPSOLVERS_HAVE_KLU ON)
      list(APPEND MIPSOLVERS_SUITESPARSE_LIBRARIES ${MIPSOLVERS_KLU_LIBRARY})
    endif()
    find_library(_SS_AMD NAMES amd HINTS ${_SS_HINTS} PATH_SUFFIXES lib)
    find_library(_SS_COLAMD NAMES colamd HINTS ${_SS_HINTS} PATH_SUFFIXES lib)
    find_library(_SS_SUITESPARSECONFIG NAMES suitesparseconfig HINTS ${_SS_HINTS} PATH_SUFFIXES lib)
    foreach(_SS_LIB _SS_AMD _SS_COLAMD _SS_SUITESPARSECONFIG)
      if(${_SS_LIB})
        list(APPEND MIPSOLVERS_SUITESPARSE_LIBRARIES "${${_SS_LIB}}")
      endif()
    endforeach()
    message(STATUS "mipsolvers: SuiteSparse detected")
  else()
    message(STATUS "mipsolvers: SuiteSparse not found; sparse LU via built-in only")
  endif()
endif()

# ── SuperLU (optional) ────────────────────────────────────────────────────────
# Eigen/SuperLUSupport includes headers as <slu_ddefs.h> (no subdirectory prefix),
# so MIPSOLVERS_SUPERLU_INCLUDE_DIR must be the folder that directly contains them.
set(MIPSOLVERS_HAVE_SUPERLU OFF)
set(MIPSOLVERS_SUPERLU_INCLUDE_DIR "")
set(MIPSOLVERS_SUPERLU_LIBRARIES "")  # list: superlu + transitive BLAS
option(MIPSOLVERS_USE_SUPERLU "Enable SuperLU backend when available" ON)
if(MIPSOLVERS_USE_SUPERLU)
  set(_SLU_HINTS
    $ENV{SUPERLU_ROOT}
    /opt/homebrew
    /usr/local
    /usr
    "C:/vcpkg/installed/x64-windows"
    "C:/vcpkg/installed/arm64-windows"
    "C:/SuperLU")
  find_path(MIPSOLVERS_SUPERLU_INCLUDE_DIR NAMES slu_ddefs.h
    HINTS ${_SLU_HINTS}
    PATH_SUFFIXES include/superlu include superlu)
  find_library(_MIPSOLVERS_SUPERLU_LIB
    NAMES superlu superlu_5.3 superlu_5.2 superlu_5.1 superlu_5.0 superlu_4.3
    HINTS ${_SLU_HINTS} PATH_SUFFIXES lib lib64)
  if(MIPSOLVERS_SUPERLU_INCLUDE_DIR AND _MIPSOLVERS_SUPERLU_LIB)
    set(MIPSOLVERS_HAVE_SUPERLU ON)
    list(APPEND MIPSOLVERS_SUPERLU_LIBRARIES ${_MIPSOLVERS_SUPERLU_LIB})
    # SuperLU requires BLAS; in shared-lib builds BLAS is embedded, but for
    # static builds (e.g. vcpkg x64-windows-static) it must be linked explicitly.
    find_package(BLAS QUIET)
    if(BLAS_FOUND)
      list(APPEND MIPSOLVERS_SUPERLU_LIBRARIES ${BLAS_LIBRARIES})
    endif()
    message(STATUS "mipsolvers: SuperLU detected at ${MIPSOLVERS_SUPERLU_INCLUDE_DIR}")
  else()
    message(STATUS "mipsolvers: SuperLU not found")
  endif()
endif()

# ── Intel MKL PARDISO (optional) ──────────────────────────────────────────────
# Intel MKL PARDISO is only available on Linux and Windows (not macOS).
if(NOT DEFINED MIPSOLVERS_HAVE_MKL_PARDISO)
  set(MIPSOLVERS_HAVE_MKL_PARDISO OFF)
endif()
if(NOT DEFINED MIPSOLVERS_MKL_INCLUDE_DIRS)
  set(MIPSOLVERS_MKL_INCLUDE_DIRS "")
endif()
if(NOT DEFINED MIPSOLVERS_MKL_LIBRARIES)
  set(MIPSOLVERS_MKL_LIBRARIES "")
endif()
option(MIPSOLVERS_USE_MKL "Enable Intel MKL PARDISO backend when available" ON)
if(MIPSOLVERS_USE_MKL AND NOT APPLE AND NOT MIPSOLVERS_HAVE_MKL_PARDISO)
  if(MIPSOLVERS_MKL_ROOT)
    # The explicit bundle is authoritative. Do not mix its headers or archives
    # with a machine-level oneAPI installation when the bundle is incomplete.
    set(_MKL_HINTS "${MIPSOLVERS_MKL_ROOT}")
    set(_MIPSOLVERS_MKL_FIND_MODE NO_DEFAULT_PATH)
  else()
    # Build the hints list carefully: $ENV{ONEAPI_ROOT} may be unset, which
    # would expand to the bogus path "/mkl/latest" inside a quoted string.
    set(_MKL_HINTS $ENV{MKLROOT})
    if(DEFINED ENV{ONEAPI_ROOT})
      list(APPEND _MKL_HINTS "$ENV{ONEAPI_ROOT}/mkl/latest")
    endif()
    list(APPEND _MKL_HINTS
      "${CMAKE_CURRENT_SOURCE_DIR}/third_party/oneapi-mkl"
      "/opt/intel/oneapi/mkl/latest"
      "/opt/intel/mkl"
      "C:/Program Files (x86)/Intel/oneAPI/mkl/latest"
      "C:/Program Files/Intel/oneAPI/mkl/latest")
    set(_MIPSOLVERS_MKL_FIND_MODE "")
  endif()
  find_path(MIPSOLVERS_MKL_INCLUDE_DIR NAMES mkl_pardiso.h
    HINTS ${_MKL_HINTS} ${_MIPSOLVERS_MKL_FIND_MODE}
    PATH_SUFFIXES include)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    find_library(MIPSOLVERS_MKL_LP64_LIB   NAMES mkl_intel_lp64
      HINTS ${_MKL_HINTS} ${_MIPSOLVERS_MKL_FIND_MODE}
      PATH_SUFFIXES lib lib/intel64)
    if(MIPSOLVERS_MKL_THREADING STREQUAL "INTEL")
      find_library(MIPSOLVERS_MKL_THREAD_LIB NAMES mkl_intel_thread
        HINTS ${_MKL_HINTS} ${_MIPSOLVERS_MKL_FIND_MODE}
        PATH_SUFFIXES lib lib/intel64)
    else()
      find_library(MIPSOLVERS_MKL_THREAD_LIB NAMES mkl_sequential
        HINTS ${_MKL_HINTS} ${_MIPSOLVERS_MKL_FIND_MODE}
        PATH_SUFFIXES lib lib/intel64)
    endif()
    find_library(MIPSOLVERS_MKL_CORE_LIB   NAMES mkl_core
      HINTS ${_MKL_HINTS} ${_MIPSOLVERS_MKL_FIND_MODE}
      PATH_SUFFIXES lib lib/intel64)
  else()
    find_library(MIPSOLVERS_MKL_LP64_LIB   NAMES mkl_intel_c
      HINTS ${_MKL_HINTS} ${_MIPSOLVERS_MKL_FIND_MODE}
      PATH_SUFFIXES lib lib/ia32)
    if(MIPSOLVERS_MKL_THREADING STREQUAL "INTEL")
      find_library(MIPSOLVERS_MKL_THREAD_LIB NAMES mkl_intel_thread
        HINTS ${_MKL_HINTS} ${_MIPSOLVERS_MKL_FIND_MODE}
        PATH_SUFFIXES lib lib/ia32)
    else()
      find_library(MIPSOLVERS_MKL_THREAD_LIB NAMES mkl_sequential
        HINTS ${_MKL_HINTS} ${_MIPSOLVERS_MKL_FIND_MODE}
        PATH_SUFFIXES lib lib/ia32)
    endif()
    find_library(MIPSOLVERS_MKL_CORE_LIB   NAMES mkl_core
      HINTS ${_MKL_HINTS} ${_MIPSOLVERS_MKL_FIND_MODE}
      PATH_SUFFIXES lib lib/ia32)
  endif()

  # vcpkg toolchain settings can restrict find_* to rooted prefixes and miss
  # oneAPI's system install path on Windows. Fall back to direct path checks.
  if(WIN32 AND NOT MIPSOLVERS_MKL_ROOT)
    if(NOT MIPSOLVERS_MKL_INCLUDE_DIR)
      set(_MKL_DEFAULT_INCLUDE "C:/Program Files (x86)/Intel/oneAPI/mkl/latest/include")
      if(EXISTS "${_MKL_DEFAULT_INCLUDE}/mkl_pardiso.h")
        set(MIPSOLVERS_MKL_INCLUDE_DIR "${_MKL_DEFAULT_INCLUDE}")
      endif()
      unset(_MKL_DEFAULT_INCLUDE)
    endif()

    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
      set(_MKL_DEFAULT_LIBDIR "C:/Program Files (x86)/Intel/oneAPI/mkl/latest/lib")
      if(NOT MIPSOLVERS_MKL_LP64_LIB AND EXISTS "${_MKL_DEFAULT_LIBDIR}/mkl_intel_lp64.lib")
        set(MIPSOLVERS_MKL_LP64_LIB "${_MKL_DEFAULT_LIBDIR}/mkl_intel_lp64.lib")
      endif()
      if(NOT MIPSOLVERS_MKL_THREAD_LIB)
        if(MIPSOLVERS_MKL_THREADING STREQUAL "INTEL" AND
           EXISTS "${_MKL_DEFAULT_LIBDIR}/mkl_intel_thread.lib")
          set(MIPSOLVERS_MKL_THREAD_LIB "${_MKL_DEFAULT_LIBDIR}/mkl_intel_thread.lib")
        elseif(MIPSOLVERS_MKL_THREADING STREQUAL "SEQUENTIAL" AND
               EXISTS "${_MKL_DEFAULT_LIBDIR}/mkl_sequential.lib")
          set(MIPSOLVERS_MKL_THREAD_LIB "${_MKL_DEFAULT_LIBDIR}/mkl_sequential.lib")
        endif()
      endif()
      if(NOT MIPSOLVERS_MKL_CORE_LIB AND EXISTS "${_MKL_DEFAULT_LIBDIR}/mkl_core.lib")
        set(MIPSOLVERS_MKL_CORE_LIB "${_MKL_DEFAULT_LIBDIR}/mkl_core.lib")
      endif()
      if(NOT _MKL_IOMP5MD AND EXISTS "C:/Program Files (x86)/Intel/oneAPI/compiler/latest/lib/libiomp5md.lib")
        set(_MKL_IOMP5MD "C:/Program Files (x86)/Intel/oneAPI/compiler/latest/lib/libiomp5md.lib")
      endif()
      unset(_MKL_DEFAULT_LIBDIR)
    endif()
  endif()

  if(MIPSOLVERS_MKL_ROOT AND
     (NOT MIPSOLVERS_MKL_INCLUDE_DIR OR NOT MIPSOLVERS_MKL_LP64_LIB OR
      NOT MIPSOLVERS_MKL_THREAD_LIB OR NOT MIPSOLVERS_MKL_CORE_LIB))
    message(FATAL_ERROR
      "MIPSOLVERS_MKL_ROOT is set to '${MIPSOLVERS_MKL_ROOT}', but it is not "
      "a complete static oneMKL bundle. Expected include/mkl_pardiso.h and "
      "the LP64, ${MIPSOLVERS_MKL_THREADING} threading, and core libraries "
      "under lib/. Run "
      "third_party/stage_onemkl.ps1 on Windows.")
  endif()

  if(MIPSOLVERS_MKL_INCLUDE_DIR AND MIPSOLVERS_MKL_LP64_LIB
      AND MIPSOLVERS_MKL_THREAD_LIB AND MIPSOLVERS_MKL_CORE_LIB)
    set(MIPSOLVERS_HAVE_MKL_PARDISO ON)
    list(APPEND MIPSOLVERS_MKL_INCLUDE_DIRS ${MIPSOLVERS_MKL_INCLUDE_DIR})
    list(APPEND MIPSOLVERS_MKL_LIBRARIES
      ${MIPSOLVERS_MKL_LP64_LIB} ${MIPSOLVERS_MKL_THREAD_LIB} ${MIPSOLVERS_MKL_CORE_LIB})
    if(WIN32 AND MIPSOLVERS_MKL_THREADING STREQUAL "INTEL")
      # mkl_intel_thread requires the Intel OpenMP runtime on Windows.
      find_library(_MKL_IOMP5MD NAMES libiomp5md
        HINTS ${_MKL_HINTS} "$ENV{INTEL_COMPILER_ROOT}"
        PATH_SUFFIXES lib lib/intel64 redist/intel64/compiler)
      if(NOT _MKL_IOMP5MD)
        message(FATAL_ERROR
          "MIPSOLVERS_MKL_THREADING=INTEL requires libiomp5md on Windows.")
      endif()
      list(APPEND MIPSOLVERS_MKL_LIBRARIES ${_MKL_IOMP5MD})
      find_file(_MKL_IOMP5MD_DLL NAMES libiomp5md.dll
        HINTS "$ENV{INTEL_COMPILER_ROOT}"
              "C:/Program Files (x86)/Intel/oneAPI/compiler/latest"
        PATH_SUFFIXES bin redist/intel64/compiler)
      if(NOT _MKL_IOMP5MD_DLL)
        message(FATAL_ERROR
          "MIPSOLVERS_MKL_THREADING=INTEL requires libiomp5md.dll on Windows.")
      endif()
      set(MIPSOLVERS_MKL_RUNTIME_DLLS "${_MKL_IOMP5MD_DLL}" CACHE INTERNAL
        "Runtime DLLs required by the selected oneMKL threading layer" FORCE)
    elseif(UNIX AND MIPSOLVERS_MKL_THREADING STREQUAL "INTEL")
      find_library(_MKL_IOMP5 NAMES iomp5
        HINTS ${_MKL_HINTS} "$ENV{INTEL_COMPILER_ROOT}"
        PATH_SUFFIXES lib lib/intel64)
      if(_MKL_IOMP5)
        list(APPEND MIPSOLVERS_MKL_LIBRARIES ${_MKL_IOMP5})
      endif()
      list(APPEND MIPSOLVERS_MKL_LIBRARIES -lpthread -lm -ldl)
    endif()
    message(STATUS "mipsolvers: Intel MKL detected at ${MIPSOLVERS_MKL_INCLUDE_DIR}")
  else()
    message(STATUS "mipsolvers: Intel MKL not found")
  endif()
elseif(MIPSOLVERS_HAVE_MKL_PARDISO)
  message(STATUS "mipsolvers: Intel MKL detected at ${MIPSOLVERS_MKL_INCLUDE_DIRS}")
endif()
unset(_MIPSOLVERS_MKL_FIND_MODE)

# ── Eigen3 (header-only) — vendored in third_party/eigen, then system ─────────
# The vendored copy is authoritative (hermetic build); system/vcpkg installs are
# only a fallback for checkouts that lack third_party/eigen.  No network access.
if(NOT TARGET Eigen3::Eigen AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen/Eigen/Core" AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen/unsupported/Eigen/MatrixFunctions")
  # IMPORTED GLOBAL (not ALIAS of a local target) so install(EXPORT) passes the
  # name through to consumers, who recreate Eigen3::Eigen via
  # mipsolversConfig.cmake (system package or bundled headers).
  add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
  target_include_directories(Eigen3::Eigen INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen")
  set(Eigen3_FOUND TRUE)
  message(STATUS "mipsolvers: Eigen3 = vendored (third_party/eigen, 3.4.1)")
endif()

if(NOT Eigen3_FOUND)
  set(_EIGEN_VCPKG_TRIPLET_HINTS)
  if(WIN32)
    if(DEFINED VCPKG_TARGET_TRIPLET)
      list(APPEND _EIGEN_VCPKG_TRIPLET_HINTS "${VCPKG_TARGET_TRIPLET}")
    endif()
    list(APPEND _EIGEN_VCPKG_TRIPLET_HINTS "x64-windows")
  endif()

  set(_EIGEN_VCPKG_CONFIG_HINTS)
  set(_EIGEN_VCPKG_INCLUDE_HINTS)
  set(_EIGEN_VCPKG_INCLUDE_DIR "")
  if(DEFINED ENV{VCPKG_ROOT})
    foreach(_triplet IN LISTS _EIGEN_VCPKG_TRIPLET_HINTS)
      list(APPEND _EIGEN_VCPKG_CONFIG_HINTS "$ENV{VCPKG_ROOT}/installed/${_triplet}/share/eigen3")
      list(APPEND _EIGEN_VCPKG_INCLUDE_HINTS "$ENV{VCPKG_ROOT}/installed/${_triplet}/include/eigen3")
      if(NOT _EIGEN_VCPKG_INCLUDE_DIR AND EXISTS "$ENV{VCPKG_ROOT}/installed/${_triplet}/include/eigen3/Eigen/Core")
        set(_EIGEN_VCPKG_INCLUDE_DIR "$ENV{VCPKG_ROOT}/installed/${_triplet}/include/eigen3")
      endif()
    endforeach()
  endif()

  if(_EIGEN_VCPKG_INCLUDE_DIR)
    if(NOT TARGET Eigen3::Eigen)
      add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
      target_include_directories(Eigen3::Eigen INTERFACE "${_EIGEN_VCPKG_INCLUDE_DIR}")
    endif()
    set(Eigen3_FOUND TRUE)
    message(STATUS "mipsolvers: Eigen3 found via vcpkg at ${_EIGEN_VCPKG_INCLUDE_DIR}")
  endif()

  if(NOT Eigen3_FOUND)
    find_package(Eigen3 3.3 CONFIG QUIET PATHS ${_EIGEN_VCPKG_CONFIG_HINTS} NO_DEFAULT_PATH)
  endif()
  if(NOT Eigen3_FOUND)
    find_package(Eigen3 3.3 CONFIG QUIET)
  endif()

  if(NOT Eigen3_FOUND)
    find_path(MIPSOLVERS_EIGEN3_INCLUDE_DIR
      NAMES Eigen/Core
      HINTS
        $ENV{EIGEN3_ROOT}
        $ENV{EIGEN_ROOT}
        ${_EIGEN_VCPKG_INCLUDE_HINTS}
        /usr/include/eigen3
        /usr/local/include/eigen3
        /opt/homebrew/include/eigen3
        "C:/vcpkg/installed/x64-windows/include/eigen3"
        "C:/Program Files/eigen3/include/eigen3"
      PATH_SUFFIXES
        include
        include/eigen3
        eigen3)

    if(MIPSOLVERS_EIGEN3_INCLUDE_DIR)
      if(NOT TARGET Eigen3::Eigen)
        add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
        target_include_directories(Eigen3::Eigen INTERFACE
          "${MIPSOLVERS_EIGEN3_INCLUDE_DIR}")
      endif()
      set(Eigen3_FOUND TRUE)
      message(STATUS "mipsolvers: Eigen3 found via include path at ${MIPSOLVERS_EIGEN3_INCLUDE_DIR}")
    endif()
  endif()
endif()

if(NOT Eigen3_FOUND)
  message(FATAL_ERROR
    "Eigen3 not found. The vendored copy third_party/eigen is missing or "
    "incomplete (expected Eigen/Core and unsupported/Eigen/MatrixFunctions) "
    "— restore it from the "
    "repository. No network fallback exists by design (hermetic build); a "
    "system Eigen3 (brew install eigen / apt install libeigen3-dev) is also "
    "accepted as a fallback.")
endif()

# ── fmt (formatting) — vendored in third_party/fmt by default ────────────────
# Vendored fmt is compiled in-tree and exported with mipsolversTargets, so
# consumers need no system fmt.  A system fmt is used only on explicit opt-in.
option(MIPSOLVERS_USE_SYSTEM_FMT
  "Use a system-installed fmt instead of the vendored third_party/fmt" OFF)
set(MIPSOLVERS_FMT_VENDORED OFF)
if(TARGET fmt::fmt)
  message(STATUS "mipsolvers: fmt = existing parent target")
elseif(MIPSOLVERS_USE_SYSTEM_FMT)
  find_package(fmt CONFIG REQUIRED)
  message(STATUS "mipsolvers: fmt = system (MIPSOLVERS_USE_SYSTEM_FMT=ON)")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/fmt/CMakeLists.txt")
  set(FMT_MASTER_PROJECT OFF CACHE BOOL "" FORCE)
  set(FMT_INSTALL OFF CACHE BOOL "" FORCE)  # exported via mipsolversTargets instead
  set(FMT_TEST OFF CACHE BOOL "" FORCE)
  set(FMT_DOC OFF CACHE BOOL "" FORCE)
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/third_party/fmt"
                   "${CMAKE_CURRENT_BINARY_DIR}/_deps/fmt_vendored"
                   EXCLUDE_FROM_ALL)
  if(NOT TARGET fmt::fmt)
    message(FATAL_ERROR
      "vendored fmt at third_party/fmt did not create the fmt::fmt target")
  endif()
  # fmt links into a static library that may later land in shared objects.
  # INSTALL_INTERFACE include dir: fmt headers are installed alongside the
  # export (public mipsolvers headers include <fmt/format.h>).
  set_target_properties(fmt PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    PUBLIC_HEADER ""
    INTERFACE_INCLUDE_DIRECTORIES
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/fmt/include>;$<INSTALL_INTERFACE:include>")
  set(MIPSOLVERS_FMT_VENDORED ON)
  message(STATUS "mipsolvers: fmt = vendored (third_party/fmt, 12.1.0)")
else()
  message(FATAL_ERROR
    "vendored fmt missing (expected third_party/fmt/CMakeLists.txt) — restore "
    "it from the repository, or configure with -DMIPSOLVERS_USE_SYSTEM_FMT=ON "
    "to use a system-installed fmt.")
endif()

# ── nlohmann/json (header-only) — vendored single header, then system ────────
if(NOT TARGET nlohmann_json::nlohmann_json AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann_json/include/nlohmann/json.hpp")
  # IMPORTED GLOBAL so install(EXPORT) passes the name through to consumers,
  # who recreate it via mipsolversConfig.cmake (system package or bundled header).
  add_library(nlohmann_json::nlohmann_json INTERFACE IMPORTED GLOBAL)
  target_include_directories(nlohmann_json::nlohmann_json INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann_json/include")
  message(STATUS "mipsolvers: nlohmann_json = vendored (third_party/nlohmann_json, 3.11.3)")
endif()
if(NOT TARGET nlohmann_json::nlohmann_json)
  find_package(nlohmann_json 3.11 CONFIG QUIET)
endif()
if(NOT TARGET nlohmann_json::nlohmann_json)
  message(FATAL_ERROR
    "nlohmann_json not found. The vendored copy third_party/nlohmann_json is "
    "missing or incomplete (expected "
    "third_party/nlohmann_json/include/nlohmann/json.hpp) — restore it from "
    "the repository. A system nlohmann_json (brew install nlohmann-json / "
    "apt install nlohmann-json3-dev) is also accepted as a fallback.")
endif()

# ── Catch2 (test framework) — vendored amalgamated v3.7.1 in third_party/catch2 ──
if(MIPSOLVERS_BUILD_TESTS)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/catch2/catch_amalgamated.cpp")
    # Official amalgamated single-TU distribution.  catch_amalgamated.cpp
    # provides a default main() unless CATCH_AMALGAMATED_CUSTOM_MAIN is
    # defined, so one compilation serves each variant.  The forwarding headers
    # under third_party/catch2/catch2/ keep <catch2/...> includes working.
    find_package(Threads REQUIRED)

    add_library(Catch2 STATIC
      "${CMAKE_CURRENT_SOURCE_DIR}/third_party/catch2/catch_amalgamated.cpp")
    target_include_directories(Catch2 PUBLIC
      "${CMAKE_CURRENT_SOURCE_DIR}/third_party/catch2")
    target_compile_definitions(Catch2 PRIVATE CATCH_AMALGAMATED_CUSTOM_MAIN)
    target_link_libraries(Catch2 PUBLIC Threads::Threads)
    set_target_properties(Catch2 PROPERTIES POSITION_INDEPENDENT_CODE ON)
    add_library(Catch2::Catch2 ALIAS Catch2)

    add_library(Catch2WithMain STATIC
      "${CMAKE_CURRENT_SOURCE_DIR}/third_party/catch2/catch_amalgamated.cpp")
    target_include_directories(Catch2WithMain PUBLIC
      "${CMAKE_CURRENT_SOURCE_DIR}/third_party/catch2")
    target_link_libraries(Catch2WithMain PUBLIC Threads::Threads)
    set_target_properties(Catch2WithMain PROPERTIES POSITION_INDEPENDENT_CODE ON)
    add_library(Catch2::Catch2WithMain ALIAS Catch2WithMain)

    set(Catch2_FOUND TRUE)
    message(STATUS "mipsolvers: Catch2 = vendored (third_party/catch2, 3.7.1 amalgamated)")
  else()
    find_package(Catch2 3 CONFIG QUIET)
    if(NOT Catch2_FOUND)
      set(_CATCH2_LOCAL_HINTS
        "${CMAKE_CURRENT_SOURCE_DIR}/../HybridACDCPowerSystemsPlanning/build_rel/_deps/catch2-src"
        "${CMAKE_CURRENT_SOURCE_DIR}/../HybridACDCPowerSystemsPlanning/build/_deps/catch2-src"
      )
      set(_CATCH2_FOUND_LOCAL FALSE)
      foreach(_CATCH2_DIR IN LISTS _CATCH2_LOCAL_HINTS)
        if(EXISTS "${_CATCH2_DIR}/CMakeLists.txt")
          set(CATCH_INSTALL_DOCS OFF CACHE BOOL "" FORCE)
          set(CATCH_INSTALL_EXTRAS OFF CACHE BOOL "" FORCE)
          set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
          add_subdirectory("${_CATCH2_DIR}"
                           "${CMAKE_CURRENT_BINARY_DIR}/_deps/catch2-build"
                           EXCLUDE_FROM_ALL)
          set(_CATCH2_FOUND_LOCAL TRUE)
          message(STATUS "mipsolvers: Catch2 found locally at ${_CATCH2_DIR}")
          break()
        endif()
      endforeach()
      if(NOT _CATCH2_FOUND_LOCAL)
        message(FATAL_ERROR
          "Catch2 not found. The vendored copy third_party/catch2 is missing or "
          "incomplete (expected third_party/catch2/catch_amalgamated.cpp) — "
          "restore it from the repository. A system Catch2 v3 (brew install "
          "catch2 / apt install catch2) is also accepted as a fallback.")
      endif()
    endif()
  endif()
endif()

# ── Status messages ───────────────────────────────────────────────────────────
message(STATUS "mipsolvers: HiGHS lib         = ${MIPSOLVERS_HAVE_HIGHS_LIB} (${MIPSOLVERS_HIGHS_LIB_SOURCE})")
message(STATUS "mipsolvers: SCIP lib          = ${MIPSOLVERS_HAVE_SCIP_LIB} (${MIPSOLVERS_SCIP_LIB_SOURCE})")
message(STATUS "mipsolvers: Gurobi            = ${MIPSOLVERS_HAVE_GUROBI}")
message(STATUS "mipsolvers: Ipopt lib         = ${MIPSOLVERS_HAVE_IPOPT}")
message(STATUS "mipsolvers: PaPILO            = ${MIPSOLVERS_HAVE_PAPILO}")
message(STATUS "mipsolvers: SuiteSparse       = ${MIPSOLVERS_HAVE_SUITESPARSE}")
