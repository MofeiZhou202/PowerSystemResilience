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

# ── Gurobi (optional, detect only) ───────────────────────────────────────────
set(MIPSOLVERS_HAVE_GUROBI OFF)
set(MIPSOLVERS_GUROBI_INCLUDE_DIRS "")
set(MIPSOLVERS_GUROBI_LIBRARIES "")
option(MIPSOLVERS_USE_GUROBI
  "Enable Gurobi detection and native C API adapter when available" OFF)
if(MIPSOLVERS_USE_GUROBI)
  find_path(MIPSOLVERS_GUROBI_INCLUDE_DIR NAMES gurobi_c.h
    HINTS /Library/gurobi1300/macos_universal2 /Library/gurobi1200/macos_universal2
          /opt/gurobi/macos_universal2
          "C:/gurobi1300/win64" "C:/gurobi1200/win64" "C:/gurobi1100/win64"
          $ENV{GUROBI_HOME}
    PATH_SUFFIXES include)
  foreach(_grb_ver 130 120 110 100 95)
    if(NOT MIPSOLVERS_GUROBI_LIBRARY)
      find_library(MIPSOLVERS_GUROBI_LIBRARY NAMES gurobi${_grb_ver}
        HINTS /Library/gurobi1300/macos_universal2 /Library/gurobi1200/macos_universal2
              /opt/gurobi/macos_universal2
              "C:/gurobi1300/win64" "C:/gurobi1200/win64" "C:/gurobi1100/win64"
              $ENV{GUROBI_HOME}
        PATH_SUFFIXES lib)
    endif()
  endforeach()
  if(MIPSOLVERS_GUROBI_INCLUDE_DIR AND MIPSOLVERS_GUROBI_LIBRARY)
    set(MIPSOLVERS_HAVE_GUROBI ON)
    set(MIPSOLVERS_GUROBI_INCLUDE_DIRS ${MIPSOLVERS_GUROBI_INCLUDE_DIR})
    set(MIPSOLVERS_GUROBI_LIBRARIES ${MIPSOLVERS_GUROBI_LIBRARY})
    message(STATUS "mipsolvers: Gurobi detected: ${MIPSOLVERS_GUROBI_LIBRARY}")
  else()
    message(STATUS "mipsolvers: Gurobi enabled but not detected; Gurobi adapter will be unavailable")
  endif()
else()
  message(STATUS "mipsolvers: Gurobi disabled (MIPSOLVERS_USE_GUROBI=OFF)")
endif()

# ── PaPILO (optional) ────────────────────────────────────────────────────────
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
set(MIPSOLVERS_HAVE_PAPILO OFF)
option(MIPSOLVERS_USE_PAPILO "Enable PaPILO presolve when available" ON)
if(MIPSOLVERS_USE_PAPILO)
  find_package(papilo CONFIG QUIET
    HINTS
      $ENV{PAPILO_ROOT}
      /opt/homebrew
      /opt/homebrew/lib/cmake/papilo
      "C:/vcpkg/installed/x64-windows")
  if(papilo_FOUND)
    set(MIPSOLVERS_HAVE_PAPILO ON)
    message(STATUS "mipsolvers: PaPILO detected")
  else()
    message(STATUS "mipsolvers: PaPILO not found; using native MILP presolve only")
  endif()
else()
  message(STATUS "mipsolvers: PaPILO disabled (MIPSOLVERS_USE_PAPILO=OFF)")
endif()

# ── Vendored CHOLMOD (offline, in-tree SuiteSparse sources) ─────────────────
# Sets MIPSOLVERS_HAVE_CHOLMOD and creates the cholmod_vendored target.
# Must run before the system SuiteSparse block below, which skips the system
# cholmod library when the vendored build is active (no duplicate symbols).
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/BuildCHOLMOD.cmake")

# ── SuiteSparse (optional) ───────────────────────────────────────────────────
set(MIPSOLVERS_HAVE_SUITESPARSE OFF)
set(MIPSOLVERS_SUITESPARSE_LIBRARIES "")
option(MIPSOLVERS_USE_SUITESPARSE "Enable SuiteSparse backends when available" ON)

# Vendored SuiteSparse (CHOLMOD/UMFPACK/KLU from in-tree sources via
# cmake/BuildCHOLMOD.cmake) is the authoritative offline path — when it is
# active we do not look for a system/Homebrew SuiteSparse at all.
if(MIPSOLVERS_USE_SUITESPARSE AND MIPSOLVERS_HAVE_CHOLMOD)
  set(MIPSOLVERS_HAVE_SUITESPARSE ON)
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
  # Build the hints list carefully: $ENV{ONEAPI_ROOT} may be unset, which
  # would expand to the bogus path "/mkl/latest" inside a quoted string.
  set(_MKL_HINTS $ENV{MKLROOT})
  if(DEFINED ENV{ONEAPI_ROOT})
    list(APPEND _MKL_HINTS "$ENV{ONEAPI_ROOT}/mkl/latest")
  endif()
  list(APPEND _MKL_HINTS
    "/opt/intel/oneapi/mkl/latest"
    "/opt/intel/mkl"
    "C:/Program Files (x86)/Intel/oneAPI/mkl/latest"
    "C:/Program Files/Intel/oneAPI/mkl/latest")
  find_path(MIPSOLVERS_MKL_INCLUDE_DIR NAMES mkl_pardiso.h
    HINTS ${_MKL_HINTS} PATH_SUFFIXES include)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    find_library(MIPSOLVERS_MKL_LP64_LIB   NAMES mkl_intel_lp64
      HINTS ${_MKL_HINTS} PATH_SUFFIXES lib lib/intel64)
    if(WIN32)
      find_library(MIPSOLVERS_MKL_THREAD_LIB NAMES mkl_sequential mkl_intel_thread
        HINTS ${_MKL_HINTS} PATH_SUFFIXES lib lib/intel64)
    else()
      find_library(MIPSOLVERS_MKL_THREAD_LIB NAMES mkl_intel_thread mkl_sequential
        HINTS ${_MKL_HINTS} PATH_SUFFIXES lib lib/intel64)
    endif()
    find_library(MIPSOLVERS_MKL_CORE_LIB   NAMES mkl_core
      HINTS ${_MKL_HINTS} PATH_SUFFIXES lib lib/intel64)
  else()
    find_library(MIPSOLVERS_MKL_LP64_LIB   NAMES mkl_intel_c
      HINTS ${_MKL_HINTS} PATH_SUFFIXES lib lib/ia32)
    if(WIN32)
      find_library(MIPSOLVERS_MKL_THREAD_LIB NAMES mkl_sequential mkl_intel_thread
        HINTS ${_MKL_HINTS} PATH_SUFFIXES lib lib/ia32)
    else()
      find_library(MIPSOLVERS_MKL_THREAD_LIB NAMES mkl_intel_thread mkl_sequential
        HINTS ${_MKL_HINTS} PATH_SUFFIXES lib lib/ia32)
    endif()
    find_library(MIPSOLVERS_MKL_CORE_LIB   NAMES mkl_core
      HINTS ${_MKL_HINTS} PATH_SUFFIXES lib lib/ia32)
  endif()

  # vcpkg toolchain settings can restrict find_* to rooted prefixes and miss
  # oneAPI's system install path on Windows. Fall back to direct path checks.
  if(WIN32)
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
      if(NOT MIPSOLVERS_MKL_THREAD_LIB AND EXISTS "${_MKL_DEFAULT_LIBDIR}/mkl_sequential.lib")
        set(MIPSOLVERS_MKL_THREAD_LIB "${_MKL_DEFAULT_LIBDIR}/mkl_sequential.lib")
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

  if(MIPSOLVERS_MKL_INCLUDE_DIR AND MIPSOLVERS_MKL_LP64_LIB
      AND MIPSOLVERS_MKL_THREAD_LIB AND MIPSOLVERS_MKL_CORE_LIB)
    set(MIPSOLVERS_HAVE_MKL_PARDISO ON)
    list(APPEND MIPSOLVERS_MKL_INCLUDE_DIRS ${MIPSOLVERS_MKL_INCLUDE_DIR})
    list(APPEND MIPSOLVERS_MKL_LIBRARIES
      ${MIPSOLVERS_MKL_LP64_LIB} ${MIPSOLVERS_MKL_THREAD_LIB} ${MIPSOLVERS_MKL_CORE_LIB})
    if(WIN32)
      # mkl_intel_thread requires the Intel OpenMP runtime on Windows.
      find_library(_MKL_IOMP5MD NAMES libiomp5md
        HINTS ${_MKL_HINTS} "$ENV{INTEL_COMPILER_ROOT}"
        PATH_SUFFIXES lib lib/intel64 redist/intel64/compiler)
      if(_MKL_IOMP5MD)
        list(APPEND MIPSOLVERS_MKL_LIBRARIES ${_MKL_IOMP5MD})
      endif()
    elseif(UNIX)
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

# ── Eigen3 (header-only) — prefer vcpkg, then system include path, then local sibling ──
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

if(NOT Eigen3_FOUND)
  # Try sibling project's fetched copy (offline-friendly)
  set(_EIGEN_LOCAL_HINTS
    "${CMAKE_CURRENT_SOURCE_DIR}/../HybridACDCPowerSystemsPlanning/build_rel/_deps/eigen-src"
    "${CMAKE_CURRENT_SOURCE_DIR}/../HybridACDCPowerSystemsPlanning/build/_deps/eigen-src"
    "${CMAKE_CURRENT_SOURCE_DIR}/../HybridACDCPowerSystemsPlanning/build_engine_rel/_deps/eigen-src"
  )
  set(_EIGEN_FOUND_LOCAL FALSE)
  foreach(_EIGEN_DIR IN LISTS _EIGEN_LOCAL_HINTS)
    if(EXISTS "${_EIGEN_DIR}/Eigen/Core")
      if(NOT TARGET Eigen3::Eigen)
        add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
        target_include_directories(Eigen3::Eigen INTERFACE "${_EIGEN_DIR}")
      endif()
      set(_EIGEN_FOUND_LOCAL TRUE)
      set(Eigen3_FOUND TRUE)
      message(STATUS "mipsolvers: Eigen3 found locally at ${_EIGEN_DIR}")
      break()
    endif()
  endforeach()
  if(NOT _EIGEN_FOUND_LOCAL)
    # Last resort: fetch Eigen3 from GitLab (works on CI without any pre-installed packages)
    message(STATUS "mipsolvers: Eigen3 not found locally; downloading via FetchContent...")
    include(FetchContent)
    FetchContent_Declare(
      eigen
      GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
      GIT_TAG        3.4.0
      GIT_SHALLOW    TRUE)
    # Disable Eigen's own install/test targets to keep the build clean
    set(EIGEN_BUILD_DOC     OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTING       OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(eigen)
    set(Eigen3_FOUND TRUE)
    message(STATUS "mipsolvers: Eigen3 fetched via FetchContent")
  endif()
endif()

# ── fmt (formatting) ─────────────────────────────────────────────────────────
find_package(fmt CONFIG QUIET)
if(NOT fmt_FOUND)
  message(FATAL_ERROR
    "fmt library not found. Install via: brew install fmt  (macOS) or "
    "apt install libfmt-dev  (Ubuntu).")
endif()

# ── nlohmann/json (header-only) ──────────────────────────────────────────────
find_package(nlohmann_json 3.11 CONFIG QUIET)
if(NOT nlohmann_json_FOUND)
  # Try sibling project's fetched copy
  set(_JSON_LOCAL_HINTS
    "${CMAKE_CURRENT_SOURCE_DIR}/../HybridACDCPowerSystemsPlanning/build_rel/_deps/nlohmann_json-src"
    "${CMAKE_CURRENT_SOURCE_DIR}/../HybridACDCPowerSystemsPlanning/build/_deps/nlohmann_json-src"
  )
  set(_JSON_FOUND_LOCAL FALSE)
  foreach(_JSON_DIR IN LISTS _JSON_LOCAL_HINTS)
    if(EXISTS "${_JSON_DIR}/include/nlohmann/json.hpp")
      add_library(nlohmann_json::nlohmann_json INTERFACE IMPORTED GLOBAL)
      target_include_directories(nlohmann_json::nlohmann_json INTERFACE "${_JSON_DIR}/include")
      set(_JSON_FOUND_LOCAL TRUE)
      message(STATUS "mipsolvers: nlohmann_json found locally at ${_JSON_DIR}")
      break()
    endif()
  endforeach()
  if(NOT _JSON_FOUND_LOCAL)
    message(FATAL_ERROR
      "nlohmann_json not found. Install via: brew install nlohmann-json  (macOS) or "
      "apt install nlohmann-json3-dev  (Ubuntu).")
  endif()
endif()

# ── Catch2 (test framework) — use local sibling or installed ─────────────────
if(MIPSOLVERS_BUILD_TESTS)
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
        "Catch2 not found. Install via: brew install catch2  (macOS) or "
        "apt install catch2  (Ubuntu), or ensure "
        "HybridACDCPowerSystemsPlanning build dirs contain catch2-src/.")
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
