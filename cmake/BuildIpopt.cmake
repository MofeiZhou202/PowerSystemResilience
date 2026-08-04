# cmake/BuildIpopt.cmake
# Compiles the Ipopt NLP solver from the local ipopt/ source directory.
# This replaces the system-wide or homebrew Ipopt detection for an embedded build.
#
# After inclusion, the following CMake target will exist:
#   ipopt_local  - the Ipopt static library.
#
# Linear solver backend:
#   - Windows: MKL Pardiso by default (no Fortran compiler required)
#   - macOS/Linux: MUMPS by default (requires Fortran unless system MUMPS is used)

if(NOT APPLE AND NOT WIN32 AND NOT UNIX)
  message(FATAL_ERROR
    "BuildIpopt.cmake currently supports macOS, Linux, and Windows. "
    "No system Ipopt fallback is allowed for this project.")
endif()

if(WIN32)
  set(_MIPSOLVERS_IPOPT_LINEAR_SOLVER_DEFAULT "pardisomkl")
else()
  set(_MIPSOLVERS_IPOPT_LINEAR_SOLVER_DEFAULT "mumps")
endif()
set(MIPSOLVERS_IPOPT_LINEAR_SOLVER "${_MIPSOLVERS_IPOPT_LINEAR_SOLVER_DEFAULT}"
  CACHE STRING "Embedded Ipopt linear solver backend (mumps or pardisomkl)")
set_property(CACHE MIPSOLVERS_IPOPT_LINEAR_SOLVER PROPERTY STRINGS mumps pardisomkl)
unset(_MIPSOLVERS_IPOPT_LINEAR_SOLVER_DEFAULT)

string(TOLOWER "${MIPSOLVERS_IPOPT_LINEAR_SOLVER}" _MIPSOLVERS_IPOPT_LINEAR_SOLVER)
if(NOT _MIPSOLVERS_IPOPT_LINEAR_SOLVER MATCHES "^(mumps|pardisomkl)$")
  message(FATAL_ERROR
    "MIPSOLVERS_IPOPT_LINEAR_SOLVER must be either 'mumps' or 'pardisomkl' "
    "(got '${MIPSOLVERS_IPOPT_LINEAR_SOLVER}').")
endif()

if(_MIPSOLVERS_IPOPT_LINEAR_SOLVER STREQUAL "pardisomkl" AND NOT WIN32)
  message(FATAL_ERROR
    "MIPSOLVERS_IPOPT_LINEAR_SOLVER=pardisomkl is currently wired for the "
    "Windows embedded Ipopt build only.")
endif()

set(_IPOPT_LINEAR_SOLVER_CONFIG [=[
/* Enable MUMPS linear solver */
#define IPOPT_HAS_MUMPS 1
/* #undef IPOPT_HAS_PARDISO_MKL */
]=])
set(_IPOPT_LINEAR_SOLVER_COMPILE_DEFS IPOPT_HAS_MUMPS=1)

if(_MIPSOLVERS_IPOPT_LINEAR_SOLVER STREQUAL "mumps")
  # Build MUMPS before creating Ipopt targets, so MUMPS::MUMPS is available.
  # This path enables Fortran; Windows uses MKL Pardiso by default to avoid it.
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BuildMUMPS.cmake")
else()
  if(NOT MIPSOLVERS_HAVE_MKL_PARDISO)
    if(MIPSOLVERS_MKL_ROOT)
      set(_MIPSOLVERS_IPOPT_MKL_ROOTS "${MIPSOLVERS_MKL_ROOT}")
    else()
      set(_MIPSOLVERS_IPOPT_MKL_ROOTS
        "${CMAKE_CURRENT_SOURCE_DIR}/third_party/oneapi-mkl"
        "$ENV{MKLROOT}")
      if(DEFINED ENV{ONEAPI_ROOT})
        list(APPEND _MIPSOLVERS_IPOPT_MKL_ROOTS "$ENV{ONEAPI_ROOT}/mkl/latest")
      endif()
      list(APPEND _MIPSOLVERS_IPOPT_MKL_ROOTS
        "C:/Program Files (x86)/Intel/oneAPI/mkl/latest"
        "C:/Program Files/Intel/oneAPI/mkl/latest")
    endif()

    foreach(_mkl_root IN LISTS _MIPSOLVERS_IPOPT_MKL_ROOTS)
      if(NOT _mkl_root)
        continue()
      endif()
      set(_mkl_inc "${_mkl_root}/include")
      set(_mkl_lib "${_mkl_root}/lib")
      if(NOT EXISTS "${_mkl_lib}/mkl_intel_lp64.lib" AND
         EXISTS "${_mkl_root}/lib/intel64/mkl_intel_lp64.lib")
        set(_mkl_lib "${_mkl_root}/lib/intel64")
      endif()
      if(EXISTS "${_mkl_inc}/mkl_pardiso.h"
         AND EXISTS "${_mkl_lib}/mkl_intel_lp64.lib"
         AND EXISTS "${_mkl_lib}/mkl_sequential.lib"
         AND EXISTS "${_mkl_lib}/mkl_core.lib")
        set(MIPSOLVERS_HAVE_MKL_PARDISO ON)
        set(MIPSOLVERS_MKL_INCLUDE_DIR "${_mkl_inc}")
        set(MIPSOLVERS_MKL_LP64_LIB "${_mkl_lib}/mkl_intel_lp64.lib")
        set(MIPSOLVERS_MKL_THREAD_LIB "${_mkl_lib}/mkl_sequential.lib")
        set(MIPSOLVERS_MKL_CORE_LIB "${_mkl_lib}/mkl_core.lib")
        set(MIPSOLVERS_MKL_INCLUDE_DIRS "${MIPSOLVERS_MKL_INCLUDE_DIR}")
        set(MIPSOLVERS_MKL_LIBRARIES
          "${MIPSOLVERS_MKL_LP64_LIB}"
          "${MIPSOLVERS_MKL_THREAD_LIB}"
          "${MIPSOLVERS_MKL_CORE_LIB}")
        break()
      endif()
    endforeach()
    unset(_MIPSOLVERS_IPOPT_MKL_ROOTS)
  endif()

  if(MIPSOLVERS_MKL_ROOT)
    foreach(_mkl_required IN ITEMS
        "${MIPSOLVERS_MKL_ROOT}/include/mkl_pardiso.h"
        "${MIPSOLVERS_MKL_ROOT}/lib/mkl_intel_lp64.lib"
        "${MIPSOLVERS_MKL_ROOT}/lib/mkl_sequential.lib"
        "${MIPSOLVERS_MKL_ROOT}/lib/mkl_core.lib"
        "${MIPSOLVERS_MKL_ROOT}/licensing")
      if(NOT EXISTS "${_mkl_required}")
        message(FATAL_ERROR
          "Incomplete hermetic oneMKL bundle at '${MIPSOLVERS_MKL_ROOT}': "
          "missing '${_mkl_required}'. Run third_party/stage_onemkl.ps1.")
      endif()
    endforeach()
    unset(_mkl_required)
  endif()

  if(NOT MIPSOLVERS_HAVE_MKL_PARDISO)
    message(FATAL_ERROR
      "Embedded Ipopt on Windows uses MKL Pardiso by default and requires a "
      "valid oneAPI MKL installation. Stage a local bundle with "
      "third_party/stage_onemkl.ps1, or configure with "
      "-DMIPSOLVERS_IPOPT_LINEAR_SOLVER=mumps and provide a Fortran compiler.")
  endif()

  set(_IPOPT_LINEAR_SOLVER_CONFIG [=[
/* #undef IPOPT_HAS_MUMPS */
/* Enable Intel MKL Pardiso linear solver */
#define IPOPT_HAS_PARDISO_MKL 1
]=])
  set(_IPOPT_LINEAR_SOLVER_COMPILE_DEFS IPOPT_HAS_PARDISO_MKL=1)
endif()

set(_IPOPT_SRC "${CMAKE_CURRENT_SOURCE_DIR}/ipopt")
set(_IPOPT_BIN "${CMAKE_CURRENT_BINARY_DIR}/_deps/embedded_ipopt")
file(MAKE_DIRECTORY "${_IPOPT_BIN}")

# ── 1. Generate config headers ────────────────────────────────────────────────
# Ipopt's config.h.in uses autoconf-style `#undef` macros.  We write a
# purpose-built config.h that enables the selected linear solver and BLAS/LAPACK.
set(_IPOPT_CONFIG_H "${_IPOPT_BIN}/config.h")
set(_IPOPT_CONFIG_CONTENT [=[
/* config.h - generated for embedded Ipopt build.
 * This is intentionally self-contained: Ipopt's config_default.h delegates to
 * an MSVC-only fallback header and fails on Clang/GCC when HAVE_CONFIG_H is set. */

#ifndef IPOPTLIB_EXPORT
# if defined(_WIN32) && defined(DLL_EXPORT)
#  define IPOPTLIB_EXPORT __declspec(dllexport)
# elif defined(__GNUC__) && __GNUC__ >= 4
#  define IPOPTLIB_EXPORT __attribute__((__visibility__("default")))
# else
#  define IPOPTLIB_EXPORT
# endif
#endif

#ifndef SIPOPTLIB_EXPORT
# if defined(_WIN32) && defined(DLL_EXPORT)
#  define SIPOPTLIB_EXPORT __declspec(dllexport)
# elif defined(__GNUC__) && __GNUC__ >= 4
#  define SIPOPTLIB_EXPORT __attribute__((__visibility__("default")))
# else
#  define SIPOPTLIB_EXPORT
# endif
#endif

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

#ifdef _MSC_VER
#define F77_FUNC(name,NAME) NAME
#define F77_FUNC_(name,NAME) NAME
#define IPOPT_C_FINITE _finite
#define IPOPT_HAS_FOPEN_S 1
#define IPOPT_HAS_GETENV_S 1
#define HAVE_WINDOWS_H 1
#else
#define F77_FUNC(name,NAME) name ## _
#define F77_FUNC_(name,NAME) name ## _
#define IPOPT_C_FINITE std::isfinite
#define HAVE_DLFCN_H 1
#define IPOPT_HAS_VA_COPY 1
#endif

/* Enable LAPACK (Accelerate on macOS, MKL on Windows, BLAS/LAPACK on Linux) */
#define IPOPT_HAS_LAPACK 1

]=])
string(APPEND _IPOPT_CONFIG_CONTENT "${_IPOPT_LINEAR_SOLVER_CONFIG}")
string(APPEND _IPOPT_CONFIG_CONTENT [=[

#define IPOPT_BLAS_FUNC(name,NAME) F77_FUNC(name,NAME)
#define IPOPT_LAPACK_FUNC(name,NAME) F77_FUNC(name,NAME)
#define IPOPT_LAPACK_FUNC_(name,NAME) F77_FUNC_(name,NAME)
#define IPOPT_PARDISO_FUNC(name,NAME) F77_FUNC(name,NAME)

/* Random number generator - standard rand() is always available */
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
/* #undef IPOPT_HAS_SPRAL */
/* #undef IPOPT_HAS_WSMP */
]=])
file(WRITE "${_IPOPT_CONFIG_H}" "${_IPOPT_CONFIG_CONTENT}")
unset(_IPOPT_CONFIG_CONTENT)

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
# Exclude solvers that need unavailable headers or are not selected.
set(_IPOPT_LINSOLVER_EXCLUDE_REGEX
  "IpIterativeWsmpSolverInterface\\.cpp|IpWsmpSolverInterface\\.cpp|IpSpralSolverInterface\\.cpp")
if(NOT _MIPSOLVERS_IPOPT_LINEAR_SOLVER STREQUAL "pardisomkl")
  string(APPEND _IPOPT_LINSOLVER_EXCLUDE_REGEX "|IpPardisoMKLSolverInterface\\.cpp")
endif()
if(NOT _MIPSOLVERS_IPOPT_LINEAR_SOLVER STREQUAL "mumps")
  string(APPEND _IPOPT_LINSOLVER_EXCLUDE_REGEX "|IpMumpsSolverInterface\\.cpp")
endif()
list(FILTER _IPOPT_LINSOLVER_SRC EXCLUDE REGEX "${_IPOPT_LINSOLVER_EXCLUDE_REGEX}")
unset(_IPOPT_LINSOLVER_EXCLUDE_REGEX)

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
    ${_IPOPT_LINEAR_SOLVER_COMPILE_DEFS})

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
# - Windows: Intel MKL (required for embedded Ipopt build)
# - macOS/Linux: MIPSOLVERS_BLAS_LIBRARIES, resolved once in
#   cmake/Dependencies.cmake (Accelerate on macOS, system BLAS/LAPACK on Linux,
#   vendored reference LAPACK as offline fallback)
if(WIN32)
  # Keep the exported static Ipopt target relocatable. The prebuilt package
  # config recreates this imported interface against its installed MKL copy.
  if(NOT TARGET MIPSolvers::MKL)
    add_library(MIPSolvers::MKL INTERFACE IMPORTED GLOBAL)
    set_target_properties(MIPSolvers::MKL PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${MIPSOLVERS_MKL_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${MIPSOLVERS_MKL_LIBRARIES}")
  endif()
  target_link_libraries(ipopt_local PRIVATE MIPSolvers::MKL)
elseif(MIPSOLVERS_BLAS_LIBRARIES)
  target_link_libraries(ipopt_local PRIVATE ${MIPSOLVERS_BLAS_LIBRARIES})
else()
  message(FATAL_ERROR
    "Embedded Ipopt requires BLAS/LAPACK (IPOPT_HAS_LAPACK=1) but "
    "MIPSOLVERS_BLAS_LIBRARIES is empty — see cmake/Dependencies.cmake.")
endif()

if(_MIPSOLVERS_IPOPT_LINEAR_SOLVER STREQUAL "mumps")
  # MUMPS sequential linear solver, built or resolved by cmake/BuildMUMPS.cmake.
  target_link_libraries(ipopt_local PRIVATE MUMPS::MUMPS)
endif()

# dl (for dynamic loading of HSL solvers at runtime)
target_link_libraries(ipopt_local PRIVATE ${CMAKE_DL_LIBS})

message(STATUS
  "mipsolvers: building embedded Ipopt from ${_IPOPT_SRC} "
  "(linear solver: ${_MIPSOLVERS_IPOPT_LINEAR_SOLVER})")
