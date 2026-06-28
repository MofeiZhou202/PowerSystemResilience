# cmake/BuildIpopt.cmake
# Compiles the Ipopt NLP solver from the local ipopt/ source directory.
# This replaces the system-wide or homebrew Ipopt detection for an embedded build.
#
# After inclusion, the following CMake target will exist:
#   ipopt_local  — the Ipopt static library (with MUMPS linear solver built from source)
#
# MUMPS is compiled from source via cmake/BuildMUMPS.cmake (uses
# scivision/mumps-superbuild + FetchContent to download MUMPS 5.9).
# This makes the build fully standalone: no homebrew ipopt required.

if(NOT APPLE AND NOT WIN32 AND NOT UNIX)
  message(FATAL_ERROR
    "BuildIpopt.cmake currently supports macOS, Linux, and Windows. "
    "No system Ipopt fallback is allowed for this project.")
endif()

# ── Build MUMPS from source (sequential, no MPI) ──────────────────────────────
# This must be done BEFORE the Ipopt targets are created, so that the dmumps
# target is available for linking.
# Requires: a Fortran compiler (brew install gcc provides gfortran on macOS).
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BuildMUMPS.cmake")

set(_IPOPT_SRC "${CMAKE_CURRENT_SOURCE_DIR}/ipopt")
set(_IPOPT_BIN "${CMAKE_CURRENT_BINARY_DIR}/_deps/embedded_ipopt")
file(MAKE_DIRECTORY "${_IPOPT_BIN}")

# ── 1. Generate config headers ────────────────────────────────────────────────
# Ipopt's config.h.in uses autoconf-style `#undef` macros.  We write a
# purpose-built config.h that enables MUMPS (via homebrew dylibs) and BLAS/LAPACK
# (via macOS Accelerate framework) and disables everything else.
set(_IPOPT_CONFIG_H "${_IPOPT_BIN}/config.h")
file(WRITE "${_IPOPT_CONFIG_H}" [=[
/* config.h — generated for embedded Ipopt build.
 * Pull in upstream fallback defaults first so compiler/platform-specific
 * macros (e.g. IPOPTLIB_EXPORT, F77_FUNC, HAVE_CSTDDEF) are defined. */
#include "config_default.h"

#define PACKAGE_VERSION "3.14.20"
#define PACKAGE "Ipopt"
#define PACKAGE_NAME "Ipopt"
#define PACKAGE_TARNAME "Ipopt"
#define PACKAGE_URL "https://github.com/coin-or/Ipopt"
#define PACKAGE_BUGREPORT "http://projects.coin-or.org/Ipopt"
#define IPOPT_VERSION "3.14.20"
#define IPOPT_VERSION_MAJOR 3
#define IPOPT_VERSION_MINOR 14
#define IPOPT_VERSION_RELEASE 20

/* Enable MUMPS linear solver (built from source, sequential mode) */
#define IPOPT_HAS_MUMPS 1

/* Enable LAPACK (Accelerate on macOS, MKL on Windows, BLAS/LAPACK on Linux) */
#define IPOPT_HAS_LAPACK 1

/* BLAS/LAPACK function mappers rely on F77_FUNC/F77_FUNC_ from config_default.h */
#define IPOPT_LAPACK_FUNC(name,NAME) F77_FUNC(name,NAME)
#define IPOPT_LAPACK_FUNC_(name,NAME) F77_FUNC_(name,NAME)

/* Random number generator — standard rand() is always available */
#define IPOPT_HAS_RAND 1
#ifndef _WIN32
#define IPOPT_HAS_DRAND48 1
#endif

/* Available standard C++ features */
#define HAVE_CMATH 1
#define HAVE_CFLOAT 1
#define HAVE_STD_ISNAN 1
#define HAVE_STD_ISINF 1

/* Threading: no OpenMP */
/* #undef HAVE_OPENMP */

/* No HSL (MA27/MA57/etc.) built in; can be loaded dynamically at runtime */
/* #undef IPOPT_HAS_HSL */
/* #undef IPOPT_HAS_PARDISO */
/* #undef IPOPT_HAS_PARDISO_MKL */
/* #undef IPOPT_HAS_SPRAL */
/* #undef IPOPT_HAS_WSMP */
]=])

# config_ipopt.h (used when HAVE_CONFIG_H is NOT defined)
set(_IPOPT_CONFIG_IPOPT_H "${_IPOPT_BIN}/config_ipopt.h")
file(WRITE "${_IPOPT_CONFIG_IPOPT_H}" [=[
/* config_ipopt.h — alias for the build-tree config.h */
#include "config.h"
]=])

# ── 2. Collect Ipopt source files ─────────────────────────────────────────────
# Core algorithm
file(GLOB _IPOPT_ALGO_SRC
  "${_IPOPT_SRC}/Algorithm/*.cpp")
file(GLOB _IPOPT_ALGO_INEXACT_SRC
  "${_IPOPT_SRC}/Algorithm/Inexact/*.cpp")
file(GLOB _IPOPT_LINSOLVER_SRC
  "${_IPOPT_SRC}/Algorithm/LinearSolvers/*.cpp"
  "${_IPOPT_SRC}/Algorithm/LinearSolvers/*.c")
file(GLOB _IPOPT_COMMON_SRC
  "${_IPOPT_SRC}/Common/*.cpp")
file(GLOB _IPOPT_INTERFACES_SRC
  "${_IPOPT_SRC}/Interfaces/*.cpp")
file(GLOB _IPOPT_LINALG_SRC
  "${_IPOPT_SRC}/LinAlg/*.cpp"
  "${_IPOPT_SRC}/LinAlg/TMatrices/*.cpp")
file(GLOB _IPOPT_CONTRIB_SRC
  "${_IPOPT_SRC}/contrib/CGPenalty/*.cpp")

# Exclude files that need Fortran interfaces or Java/AMPL that we don't need
list(FILTER _IPOPT_INTERFACES_SRC EXCLUDE REGEX
  "IpStdFInterface\\.cpp|IpStdJInterface\\.cpp")

# Exclude Inexact Pardiso interface (needs Pardiso headers) and WSMP-iterative
list(FILTER _IPOPT_ALGO_INEXACT_SRC EXCLUDE REGEX
  "IpIterativePardisoSolverInterface\\.cpp")
# Exclude solvers that need unavailable headers (SPRAL, MKL Pardiso, WSMP)
list(FILTER _IPOPT_LINSOLVER_SRC EXCLUDE REGEX
  "IpIterativeWsmpSolverInterface\\.cpp|IpWsmpSolverInterface\\.cpp|IpSpralSolverInterface\\.cpp|IpPardisoMKLSolverInterface\\.cpp")

set(_IPOPT_ALL_SRC
  ${_IPOPT_ALGO_SRC}
  ${_IPOPT_ALGO_INEXACT_SRC}
  ${_IPOPT_LINSOLVER_SRC}
  ${_IPOPT_COMMON_SRC}
  ${_IPOPT_INTERFACES_SRC}
  ${_IPOPT_LINALG_SRC}
  ${_IPOPT_CONTRIB_SRC})

# ── 3. Create the library target ──────────────────────────────────────────────
add_library(ipopt_local STATIC ${_IPOPT_ALL_SRC})

# Mark it as Ipopt for downstream targets
add_library(Ipopt::ipopt ALIAS ipopt_local)

# ── 4. Include directories ────────────────────────────────────────────────────
target_include_directories(ipopt_local
  PUBLIC
    $<BUILD_INTERFACE:${_IPOPT_SRC}/Algorithm>
    $<BUILD_INTERFACE:${_IPOPT_SRC}/Algorithm/Inexact>
    $<BUILD_INTERFACE:${_IPOPT_SRC}/Algorithm/LinearSolvers>
    $<BUILD_INTERFACE:${_IPOPT_SRC}/Common>
    $<BUILD_INTERFACE:${_IPOPT_SRC}/Interfaces>
    $<BUILD_INTERFACE:${_IPOPT_SRC}/LinAlg>
    $<BUILD_INTERFACE:${_IPOPT_SRC}/LinAlg/TMatrices>
    $<BUILD_INTERFACE:${_IPOPT_SRC}/contrib/CGPenalty>
    $<BUILD_INTERFACE:${_IPOPT_BIN}>)  # for config.h

# ── 5. Compile definitions ────────────────────────────────────────────────────
target_compile_definitions(ipopt_local
  PRIVATE
    HAVE_CONFIG_H=1          # use our generated config.h
    IPOPTLIB_BUILD=1         # build-time flag
    IPOPT_HAS_MUMPS=1)

# C++ standard
target_compile_features(ipopt_local PUBLIC cxx_std_14)

# Suppress common warnings in third-party code
target_compile_options(ipopt_local PRIVATE
  $<$<CXX_COMPILER_ID:AppleClang,Clang,GNU>:
    -Wno-unused-variable
    -Wno-unused-parameter
    -Wno-unused-function
    -Wno-deprecated-declarations
    -Wno-sign-compare>)

# ── 6. Link libraries ─────────────────────────────────────────────────────────
# BLAS/LAPACK backend:
# - macOS: Accelerate
# - Windows: Intel MKL (required for embedded Ipopt build)
target_link_libraries(ipopt_local PRIVATE
  "$<$<PLATFORM_ID:Darwin>:-framework Accelerate>")

if(WIN32)
  # Dependencies.cmake currently evaluates MKL detection after including this
  # file. Probe oneAPI MKL directly here so embedded Ipopt can configure.
  if(NOT MIPSOLVERS_HAVE_MKL_PARDISO)
    set(_MIPSOLVERS_IPOPT_MKL_ROOTS
      "$ENV{MKLROOT}"
      "C:/Program Files (x86)/Intel/oneAPI/mkl/latest"
      "C:/Program Files/Intel/oneAPI/mkl/latest")
    foreach(_mkl_root IN LISTS _MIPSOLVERS_IPOPT_MKL_ROOTS)
      if(NOT _mkl_root)
        continue()
      endif()
      set(_mkl_inc "${_mkl_root}/include")
      set(_mkl_lib "${_mkl_root}/lib")
      if(EXISTS "${_mkl_inc}/mkl_pardiso.h"
         AND EXISTS "${_mkl_lib}/mkl_intel_lp64.lib"
         AND EXISTS "${_mkl_lib}/mkl_sequential.lib"
         AND EXISTS "${_mkl_lib}/mkl_core.lib")
        set(MIPSOLVERS_HAVE_MKL_PARDISO ON)
        set(MIPSOLVERS_MKL_INCLUDE_DIRS "${_mkl_inc}")
        set(MIPSOLVERS_MKL_LIBRARIES
          "${_mkl_lib}/mkl_intel_lp64.lib"
          "${_mkl_lib}/mkl_sequential.lib"
          "${_mkl_lib}/mkl_core.lib")
        if(EXISTS "C:/Program Files (x86)/Intel/oneAPI/compiler/latest/lib/libiomp5md.lib")
          list(APPEND MIPSOLVERS_MKL_LIBRARIES
            "C:/Program Files (x86)/Intel/oneAPI/compiler/latest/lib/libiomp5md.lib")
        endif()
        break()
      endif()
    endforeach()
    unset(_MIPSOLVERS_IPOPT_MKL_ROOTS)
  endif()

  if(NOT MIPSOLVERS_HAVE_MKL_PARDISO)
    message(FATAL_ERROR
      "Embedded Ipopt on Windows requires MKL for BLAS/LAPACK. "
      "Set MIPSOLVERS_USE_MKL=ON and provide a valid oneAPI MKL installation.")
  endif()
  target_include_directories(ipopt_local PRIVATE ${MIPSOLVERS_MKL_INCLUDE_DIRS})
  target_link_libraries(ipopt_local PRIVATE ${MIPSOLVERS_MKL_LIBRARIES})
elseif(UNIX AND NOT APPLE)
  find_package(BLAS REQUIRED)
  find_package(LAPACK REQUIRED)
  target_link_libraries(ipopt_local PRIVATE ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
endif()

# MUMPS sequential linear solver — built from source by cmake/BuildMUMPS.cmake.
# MUMPS::MUMPS is an interface alias that transitively pulls in dmumps,
# mumps_common, pord, and the mpiseq sequential stub.
target_link_libraries(ipopt_local PRIVATE MUMPS::MUMPS)

# dl (for dynamic loading of HSL solvers at runtime)
target_link_libraries(ipopt_local PRIVATE ${CMAKE_DL_LIBS})

message(STATUS "mipsolvers: building embedded Ipopt from ${_IPOPT_SRC}")
