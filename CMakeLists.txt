cmake_minimum_required(VERSION 3.20)

project(HybridACDCDistributionSystemsSimulation VERSION 0.1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Build type defaults ───────────────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Build type (Debug, Release, RelWithDebInfo)" FORCE)
endif()

include(CheckIPOSupported)
check_ipo_supported(RESULT _ipo_ok OUTPUT _ipo_out)
if(_ipo_ok)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON)
endif()

# ── Options ───────────────────────────────────────────────────────────────────
option(HACDCDSS_BUILD_TESTS   "Build unit/integration tests"               ON)
option(HACDCDSS_ENABLE_WERROR "Treat warnings as errors for core library"  OFF)
option(HACDCDSS_ENABLE_NATIVE_ARCH
       "Enable -march=native for host-specific builds"                     OFF)

# ── Dependencies (pulls in MIPSolvers + transitive: Eigen3, fmt, nlohmann_json,
#                 HiGHS, optional Ipopt/SCIP) ─────────────────────────────────
include(cmake/Dependencies.cmake)

# ── Core library ──────────────────────────────────────────────────────────────
#
# hacdcdss is one standalone static library that includes four modules:
#
#   model               – AC/DC network data structures, scenario & time-series
#   power_models        – admittance matrix (Y-bus / G_dc) construction
#   power_flow          – Newton-Raphson AC/DC power-flow solver
#   optimal_power_flow  – DC-OPF (LP) and AC-OPF (NLP) formulations
#
# The only external dependency is mipsolvers::mipsolvers, which transitively
# provides Eigen3, fmt, nlohmann_json, HiGHS, and optional Ipopt/SCIP.
# No Homebrew or system-installed library is required.

add_library(hacdcdss STATIC
  # ── model ────────────────────────────────────────────────────────────────
  src/model/network_model.cpp
  src/model/scenario_data.cpp
  # ── power_models ─────────────────────────────────────────────────────────
  src/power_models/admittance.cpp
  # ── power_flow ───────────────────────────────────────────────────────────
  src/power_flow/pf_solver.cpp
  # ── optimal_power_flow ───────────────────────────────────────────────────
  src/optimal_power_flow/opf_model.cpp
)

add_library(hacdcdss::hacdcdss ALIAS hacdcdss)

# ── Include paths ─────────────────────────────────────────────────────────────
target_include_directories(hacdcdss
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ── Link against MIPSolvers (brings Eigen3, fmt, nlohmann_json, HiGHS, Ipopt) ─
target_link_libraries(hacdcdss PUBLIC mipsolvers::mipsolvers)

# ── Compiler flags ────────────────────────────────────────────────────────────
if(HACDCDSS_ENABLE_WERROR)
  target_compile_options(hacdcdss PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Werror>
    $<$<CXX_COMPILER_ID:MSVC>:/WX>)
endif()

if(HACDCDSS_ENABLE_NATIVE_ARCH)
  target_compile_options(hacdcdss PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-march=native>)
endif()

# ── Tests ─────────────────────────────────────────────────────────────────────
if(HACDCDSS_BUILD_TESTS)
  include(CTest)
  enable_testing()
  add_subdirectory(tests)
endif()
