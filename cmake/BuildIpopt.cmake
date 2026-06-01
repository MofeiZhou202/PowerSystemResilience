# cmake/BuildIpopt.cmake
# Compiles the Ipopt NLP solver from the local ipopt/ source directory.
# This replaces the system-wide or homebrew Ipopt detection for an embedded build.
#
# After inclusion, the following CMake target will exist:
#   ipopt_local  — the Ipopt static library (with MUMPS linear solver via homebrew dylibs)

if(NOT APPLE)
  message(FATAL_ERROR
    "BuildIpopt.cmake is currently macOS/Homebrew-specific. "
    "No system Ipopt fallback is allowed for this project. Disable "
    "MIPSOLVERS_BUILD_LOCAL_IPOPT only if the TNLP bridge is intentionally "
    "unavailable, or add the required in-repository BLAS/LAPACK/MUMPS build "
    "for this platform.")
endif()

set(_IPOPT_SRC "${CMAKE_CURRENT_SOURCE_DIR}/ipopt")
set(_IPOPT_BIN "${CMAKE_CURRENT_BINARY_DIR}/_deps/embedded_ipopt")
file(MAKE_DIRECTORY "${_IPOPT_BIN}")

# ── 1. Generate config headers ────────────────────────────────────────────────
# Ipopt's config.h.in uses autoconf-style `#undef` macros.  We write a
# purpose-built config.h that enables MUMPS (via homebrew dylibs) and BLAS/LAPACK
# (via macOS Accelerate framework) and disables everything else.
set(_IPOPT_CONFIG_H "${_IPOPT_BIN}/config.h")
file(WRITE "${_IPOPT_CONFIG_H}" [=[
/* config.h — generated for embedded Ipopt build on macOS.
 * MUMPS backend provided by libdmumps.dylib from homebrew ipopt cellar.
 * BLAS/LAPACK backend provided by macOS Accelerate framework. */
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

/* Enable MUMPS linear solver (linked against homebrew libdmumps.dylib) */
#define IPOPT_HAS_MUMPS 1

/* Enable LAPACK (via Accelerate framework on macOS) */
#define IPOPT_HAS_LAPACK 1

/* BLAS/LAPACK functions — use Accelerate's Fortran name-mangling convention */
#define F77_FUNC(name,NAME) name ## _
#define F77_FUNC_(name,NAME) name ## _
#define IPOPT_LAPACK_FUNC(name,NAME) F77_FUNC(name,NAME)
#define IPOPT_LAPACK_FUNC_(name,NAME) F77_FUNC_(name,NAME)

/* Random number generator — standard rand() is always available */
#define IPOPT_HAS_RAND 1
#define IPOPT_HAS_DRAND48 1

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
# BLAS/LAPACK via macOS Accelerate framework
# 使用 generator expression：非 Apple 平台上此条目求值为空，避免写入导出文件。
target_link_libraries(ipopt_local PRIVATE
  "$<$<PLATFORM_ID:Darwin>:-framework Accelerate>")

# MUMPS sequential linear solver from homebrew ipopt cellar
# BUILD_INTERFACE：macOS Homebrew 绝对路径不写入安装树的 INTERFACE_LINK_LIBRARIES。
set(_IPOPT_MUMPS_LIB_DIR "/opt/homebrew/opt/ipopt/lib")
if(EXISTS "${_IPOPT_MUMPS_LIB_DIR}/libdmumps.dylib")
  foreach(_mumps_lib
      "${_IPOPT_MUMPS_LIB_DIR}/libdmumps.dylib"
      "${_IPOPT_MUMPS_LIB_DIR}/libmumps_common.dylib"
      "${_IPOPT_MUMPS_LIB_DIR}/libmpiseq.dylib"
      "${_IPOPT_MUMPS_LIB_DIR}/libpord.dylib")
    target_link_libraries(ipopt_local PRIVATE "$<BUILD_INTERFACE:${_mumps_lib}>")
  endforeach()
  # Need gcc runtime for Fortran-compiled MUMPS
  find_library(_GFORTRAN_LIB gfortran
    HINTS /opt/homebrew/opt/gcc/lib/gcc/current /opt/homebrew/lib
    NO_DEFAULT_PATH)
  if(_GFORTRAN_LIB)
    target_link_libraries(ipopt_local PRIVATE "$<BUILD_INTERFACE:${_GFORTRAN_LIB}>")
  endif()
else()
  message(WARNING "BuildIpopt: homebrew MUMPS dylibs not found at ${_IPOPT_MUMPS_LIB_DIR}")
endif()

# OpenBLAS used by MUMPS
find_library(_OPENBLAS_LIB openblas HINTS /opt/homebrew/opt/openblas/lib NO_DEFAULT_PATH)
if(_OPENBLAS_LIB)
  target_link_libraries(ipopt_local PRIVATE "$<BUILD_INTERFACE:${_OPENBLAS_LIB}>")
endif()

# dl (for dynamic loading of HSL solvers at runtime)
target_link_libraries(ipopt_local PRIVATE ${CMAKE_DL_LIBS})

message(STATUS "mipsolvers: building embedded Ipopt from ${_IPOPT_SRC}")
