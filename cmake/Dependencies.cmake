# cmake/Dependencies.cmake
# All dependencies resolved locally — no network downloads.
# HiGHS, SCIP, and Ipopt are compiled from the local source directories.

# ── Optional external solver executables ─────────────────────────────────────
find_program(MIPSOLVERS_HIGHS_EXECUTABLE
  NAMES highs
  HINTS /opt/homebrew/bin /usr/local/bin /usr/bin)

find_program(MIPSOLVERS_IPOPT_EXECUTABLE
  NAMES ipopt
  HINTS /opt/homebrew/bin /usr/local/bin /usr/bin)

find_program(MIPSOLVERS_SCIP_EXECUTABLE
  NAMES scip
  HINTS /opt/homebrew/bin /usr/local/bin /usr/bin)

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

# ── Ipopt NLP solver ──────────────────────────────────────────────────────────
set(MIPSOLVERS_HAVE_IPOPT OFF)
option(MIPSOLVERS_BUILD_LOCAL_IPOPT
  "Build embedded Ipopt source (ipopt/) instead of using system Ipopt" ON)

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

if(NOT MIPSOLVERS_HAVE_IPOPT)
  # Fall back to system/homebrew Ipopt
  find_path(MIPSOLVERS_IPOPT_INCLUDE_DIR NAMES IpIpoptApplication.hpp
    HINTS /opt/homebrew /usr/local /usr
    PATH_SUFFIXES include/coin-or coin-or include)
  find_library(MIPSOLVERS_IPOPT_LIBRARY NAMES ipopt
    HINTS /opt/homebrew /usr/local /usr PATH_SUFFIXES lib)
  if(MIPSOLVERS_IPOPT_INCLUDE_DIR AND MIPSOLVERS_IPOPT_LIBRARY)
    set(MIPSOLVERS_HAVE_IPOPT ON)
    set(MIPSOLVERS_IPOPT_INCLUDE_DIRS ${MIPSOLVERS_IPOPT_INCLUDE_DIR})
    set(MIPSOLVERS_IPOPT_LIBRARIES ${MIPSOLVERS_IPOPT_LIBRARY})
    message(STATUS "mipsolvers: Ipopt detected (system): ${MIPSOLVERS_IPOPT_LIBRARY}")
  else()
    message(STATUS "mipsolvers: Ipopt not detected; building adapter without TNLP bridge")
  endif()
endif()

# ── Gurobi (optional, detect only) ───────────────────────────────────────────
set(MIPSOLVERS_HAVE_GUROBI OFF)
set(MIPSOLVERS_GUROBI_INCLUDE_DIRS "")
set(MIPSOLVERS_GUROBI_LIBRARIES "")
find_path(MIPSOLVERS_GUROBI_INCLUDE_DIR NAMES gurobi_c.h
  HINTS /Library/gurobi1300/macos_universal2 /Library/gurobi1200/macos_universal2
        /opt/gurobi/macos_universal2 ENV GUROBI_HOME
  PATH_SUFFIXES include)
foreach(_grb_ver 130 120 110 100)
  if(NOT MIPSOLVERS_GUROBI_LIBRARY)
    find_library(MIPSOLVERS_GUROBI_LIBRARY NAMES gurobi${_grb_ver}
      HINTS /Library/gurobi1300/macos_universal2 /Library/gurobi1200/macos_universal2
            /opt/gurobi/macos_universal2 ENV GUROBI_HOME
      PATH_SUFFIXES lib)
  endif()
endforeach()
if(MIPSOLVERS_GUROBI_INCLUDE_DIR AND MIPSOLVERS_GUROBI_LIBRARY)
  set(MIPSOLVERS_HAVE_GUROBI ON)
  set(MIPSOLVERS_GUROBI_INCLUDE_DIRS ${MIPSOLVERS_GUROBI_INCLUDE_DIR})
  set(MIPSOLVERS_GUROBI_LIBRARIES ${MIPSOLVERS_GUROBI_LIBRARY})
  message(STATUS "mipsolvers: Gurobi detected: ${MIPSOLVERS_GUROBI_LIBRARY}")
else()
  message(STATUS "mipsolvers: Gurobi not detected; Gurobi adapter will be unavailable")
endif()

# ── PaPILO (optional) ────────────────────────────────────────────────────────
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
set(MIPSOLVERS_HAVE_PAPILO OFF)
find_package(papilo CONFIG QUIET HINTS /opt/homebrew /opt/homebrew/lib/cmake/papilo)
if(papilo_FOUND)
  set(MIPSOLVERS_HAVE_PAPILO ON)
  message(STATUS "mipsolvers: PaPILO detected")
else()
  message(STATUS "mipsolvers: PaPILO not found; using native MILP presolve only")
endif()

# ── SuiteSparse (optional) ───────────────────────────────────────────────────
set(MIPSOLVERS_HAVE_SUITESPARSE OFF)
set(MIPSOLVERS_HAVE_UMFPACK OFF)
set(MIPSOLVERS_HAVE_KLU OFF)
set(MIPSOLVERS_SUITESPARSE_INCLUDE_DIRS "")
set(MIPSOLVERS_SUITESPARSE_LIBRARIES "")
option(MIPSOLVERS_USE_SUITESPARSE "Enable SuiteSparse backends when available" ON)
if(MIPSOLVERS_USE_SUITESPARSE)
  set(_SS_HINTS /opt/homebrew/opt/suite-sparse /opt/homebrew /usr/local/opt/suite-sparse /usr/local /usr)
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
    find_library(_SS_CHOLMOD NAMES cholmod HINTS ${_SS_HINTS} PATH_SUFFIXES lib)
    find_library(_SS_SUITESPARSECONFIG NAMES suitesparseconfig HINTS ${_SS_HINTS} PATH_SUFFIXES lib)
    foreach(_SS_LIB _SS_AMD _SS_COLAMD _SS_CHOLMOD _SS_SUITESPARSECONFIG)
      if(${_SS_LIB})
        list(APPEND MIPSOLVERS_SUITESPARSE_LIBRARIES "${${_SS_LIB}}")
      endif()
    endforeach()
    message(STATUS "mipsolvers: SuiteSparse detected")
  else()
    message(STATUS "mipsolvers: SuiteSparse not found; sparse LU via built-in only")
  endif()
endif()

# ── Eigen3 (header-only) — use installed package, system include path, or local sibling ──
find_package(Eigen3 3.3 CONFIG QUIET)

if(NOT Eigen3_FOUND)
  find_path(MIPSOLVERS_EIGEN3_INCLUDE_DIR
    NAMES Eigen/Core
    HINTS
      $ENV{EIGEN3_ROOT}
      $ENV{EIGEN_ROOT}
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
    message(FATAL_ERROR
      "Eigen3 not found. Install Eigen or set EIGEN3_ROOT/EIGEN_ROOT to its include prefix.")
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
message(STATUS "mipsolvers: HiGHS exe        = ${MIPSOLVERS_HIGHS_EXECUTABLE}")
message(STATUS "mipsolvers: Ipopt exe         = ${MIPSOLVERS_IPOPT_EXECUTABLE}")
message(STATUS "mipsolvers: SCIP exe          = ${MIPSOLVERS_SCIP_EXECUTABLE}")
message(STATUS "mipsolvers: HiGHS lib         = ${MIPSOLVERS_HAVE_HIGHS_LIB} (${MIPSOLVERS_HIGHS_LIB_SOURCE})")
message(STATUS "mipsolvers: SCIP lib          = ${MIPSOLVERS_HAVE_SCIP_LIB} (${MIPSOLVERS_SCIP_LIB_SOURCE})")
message(STATUS "mipsolvers: Gurobi            = ${MIPSOLVERS_HAVE_GUROBI}")
message(STATUS "mipsolvers: Ipopt lib         = ${MIPSOLVERS_HAVE_IPOPT}")
message(STATUS "mipsolvers: PaPILO            = ${MIPSOLVERS_HAVE_PAPILO}")
message(STATUS "mipsolvers: SuiteSparse       = ${MIPSOLVERS_HAVE_SUITESPARSE}")
