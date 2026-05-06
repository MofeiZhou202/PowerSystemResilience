cmake_minimum_required(VERSION 3.20)

project(HybridACDCPowerFlowCpp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Build type defaults & optimization ─────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type (Debug, Release, RelWithDebInfo)" FORCE)
endif()

# Enable link-time optimization for Release builds.
include(CheckIPOSupported)
check_ipo_supported(RESULT _ipo_ok OUTPUT _ipo_out)
if(_ipo_ok)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON)
endif()

option(HACDCPF_BUILD_TESTS "Build scaffold tests" ON)
option(HACDCPF_USE_SUITESPARSE "Enable SuiteSparse backends when available" ON)
option(HACDCPF_ENABLE_WERROR "Treat warnings as errors for core library" ON)
option(HACDCPF_ENABLE_FACTOR_BACKEND_B "Enable Backend B (HiGHS-like) skeleton paths" OFF)
option(HACDCPF_BUILD_HFACTOR "Build vendored HiGHS HFactor static library (U.7.118)" ON)
option(HACDCPF_ENABLE_NATIVE_ARCH "Enable -march=native for host-specific benchmark builds" OFF)
option(HACDCPF_EIGEN_VECTORIZE "Enable Eigen vectorization intrinsics" ON)
set(HACDCPF_EIGEN_MAX_ALIGN_BYTES "32" CACHE STRING "Eigen max alignment bytes (16/32/64; empty to use Eigen default)")

include(cmake/Dependencies.cmake)
include(cmake/OpenDSSCompare.cmake)

add_library(hacdcpf STATIC
  src/aml/set.cpp
  src/aml/model.cpp
  src/power_models/dc_opf_builder.cpp
  src/power_models/acopf_builder.cpp
  src/power_models/acdcopf_builder.cpp
  src/power_models/scuc_builder.cpp
  src/power_models/lindistflow_builder.cpp
  src/api/hacdcpf.cpp
  src/engine/api/solver.cpp
  src/engine/strategy/dispatcher.cpp
  src/engine/strategy/warmstart.cpp
  src/engine/strategy/highs_presolve_side_state.cpp
  src/engine/strategy/presolve_manager.cpp
  src/engine/strategy/postsolve_manager.cpp
  src/planning/microgrid_planning_input.cpp
  src/planning/microgrid_planning_builder.cpp
  src/planning/microgrid_planning_ai.cpp
  src/planning/microgrid_planning_solver.cpp
  src/planning/microgrid_planning_reporting.cpp
  src/planning/hybrid_distribution_planning_validation.cpp
  src/decomposition/stochastic_decomp.cpp
  src/decomposition/centralized_native.cpp
  src/decomposition/aop_native.cpp
  src/decomposition/ccg_native.cpp
  src/decomposition/bdd_native.cpp
  src/decomposition/nested_benders_native.cpp
  src/engine/solver/solver_adapter.cpp
  src/engine/kernel/linear_algebra/linear_solver.cpp
  src/engine/kernel/linear_algebra/sparse_lu_factor.cpp
  src/engine/kernel/linear_algebra/hfactor_backend.cpp
  src/engine/kernel/lp_kernel/dual_simplex.cpp
  src/engine/kernel/lp_kernel/dual_simplex_api.cpp
  src/engine/kernel/kkt/kkt_system.cpp
  src/engine/solver/native/milp/bc/legacy/branch_and_cut.cpp
  src/engine/solver/native/milp/bc/legacy/bc_parallel.cpp
  src/engine/solver/native/milp/bc/legacy/bc_utils.cpp
  src/engine/solver/native/milp/bc/milp_presolve.cpp
  src/engine/strategy/papilo_presolve.cpp
  src/engine/solver/native/milp/bc/legacy/bc_branching.cpp
  src/engine/solver/native/milp/bc/legacy/bc_cuts.cpp
  src/engine/solver/native/milp/bc/legacy/bc_cglp.cpp
  src/engine/solver/native/milp/bc/legacy/bc_cglp_model.cpp
  src/engine/solver/native/milp/bc/legacy/bc_clique_table.cpp
  src/engine/solver/native/milp/bc/legacy/bc_relaxation.cpp
  src/engine/solver/native/milp/bc/parallel/shared_state.cpp
  src/engine/solver/native/milp/bc/parallel/parallel_search.cpp
  src/engine/solver/native/milp/bc/api.cpp
  src/engine/solver/native/milp/bc/search/node_evaluator.cpp
  src/engine/solver/native/milp/bc/root/root_solve.cpp
  src/engine/solver/native/milp/bc/search/sequential_search.cpp
  src/engine/solver/adapter_registry.cpp
  src/engine/solver/native/native_adapters.cpp
  src/engine/solver/external/adapters.cpp
  src/engine/kernel/ipm/lcqp_solver.cpp
  src/engine/solver/native/lp/pdlp_solver.cpp
  src/engine/kernel/ipm/ipm_lp_solver.cpp
  src/engine/kernel/ipm/ipm_solver.cpp
  src/engine/kernel/ipm/ipm_filter.cpp
  src/engine/kernel/ipm/ipm_scaling.cpp
  src/engine/kernel/ipm/ipm_restoration.cpp
  src/engine/util/problem_validation.cpp
  src/engine/solver/native/nle/newton_solver.cpp
  src/engine/solver/native/nle/nle_solver.cpp
  src/power_flow/solver_data.cpp
  src/power_flow/workspace.cpp
  src/power_flow/admittance_builder.cpp
  src/power_flow/converter_model.cpp
  src/power_flow/residual_evaluator.cpp
  src/power_flow/jacobian_builder.cpp
  src/power_flow/jacobian_pattern.cpp
  src/power_flow/ac_kernel.cpp
  src/power_flow/dc_solver.cpp
  src/power_flow/pf_injection_assembly.cpp
  src/power_flow/island_detector.cpp
  src/power_flow/adaptive_solver.cpp
  src/power_flow/distributed_slack_solver.cpp
  src/power_flow/branch_flow.cpp
  src/power_flow/fdpf_solver.cpp
  src/power_flow/ac_linearized_pf.cpp
  src/power_flow/nonlinear_scaling.cpp
  src/power_flow/robust_nonlinear_options.cpp
  src/power_flow/nonmonotone_linesearch.cpp
  src/power_flow/lm_trust_region.cpp
  src/power_flow/homotopy_continuation.cpp
  src/power_flow/helm_solver.cpp
  src/power_flow/newton_krylov.cpp
  src/power_flow/voltage_stability.cpp
  src/model/network_utils.cpp
  src/optimal_power_flow/ac_opf.cpp
  src/optimal_power_flow/dc_opf.cpp
  src/optimal_power_flow/parity_formulation.cpp
  src/optimal_power_flow/parity_ipm.cpp
  src/optimal_power_flow/reactive_power_opt.cpp
  src/io/case_builders.cpp
  src/io/matpower_parser.cpp
  src/io/json_io.cpp
  src/analysis/distribution_power_flow.cpp
  src/analysis/three_phase_nr.cpp
  src/analysis/short_circuit.cpp
  src/analysis/topology_analysis.cpp
  src/analysis/carbon_analysis.cpp
  src/analysis/time_series_pf.cpp
  src/analysis/time_series_distribution.cpp
  src/analysis/annual_production_sim.cpp
  src/analysis/lifecycle_simulation.cpp
  src/analysis/market_simulation.cpp
  src/analysis/reliability_assessment.cpp
  src/analysis/resilience_assessment.cpp
  src/analysis/resilience_restoration_mip.cpp
  src/visualization/html_visualizer.cpp
  src/power_flow/ac.cpp
  src/power_flow/dc.cpp
  src/power_flow/hybrid.cpp
)

# Excel I/O (compiled only when OpenXLSX is available)
if(HACDCPF_HAVE_OPENXLSX)
  target_sources(hacdcpf PRIVATE src/io/excel_io.cpp)
endif()

# OpenDSS bridge (compiled only when dss_capi is available)
if(HACDCPF_HAVE_OPENDSS)
  target_sources(hacdcpf PRIVATE src/io/opendss_bridge.cpp)
endif()

add_library(hacdcpf::hacdcpf ALIAS hacdcpf)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmark/solver_benchmark_runner.cpp")
  add_executable(solver_benchmark_runner benchmark/solver_benchmark_runner.cpp)
  target_link_libraries(solver_benchmark_runner PRIVATE hacdcpf)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmark/milp_benchmark_runner.cpp")
  add_executable(milp_benchmark_runner benchmark/milp_benchmark_runner.cpp)
  target_link_libraries(milp_benchmark_runner PRIVATE hacdcpf)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmark/presolve_structure_diag.cpp")
  add_executable(presolve_structure_diag benchmark/presolve_structure_diag.cpp)
  target_link_libraries(presolve_structure_diag PRIVATE hacdcpf)
endif()

add_executable(pdlp_scaling_runner tests/pdlp_scaling_runner.cpp)
target_link_libraries(pdlp_scaling_runner PRIVATE hacdcpf)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmark/pdlp_pmm_benchmark.cpp")
  add_executable(pdlp_pmm_benchmark benchmark/pdlp_pmm_benchmark.cpp)
  target_link_libraries(pdlp_pmm_benchmark PRIVATE hacdcpf)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmark/ipm_lp_benchmark.cpp")
  add_executable(ipm_lp_benchmark benchmark/ipm_lp_benchmark.cpp)
  target_link_libraries(ipm_lp_benchmark PRIVATE hacdcpf)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmark/nlp_ipm_benchmark.cpp")
  add_executable(nlp_ipm_benchmark benchmark/nlp_ipm_benchmark.cpp)
  target_link_libraries(nlp_ipm_benchmark PRIVATE hacdcpf)
endif()

add_executable(lp_simplex_benchmark benchmark/lp_simplex_benchmark.cpp)
target_link_libraries(lp_simplex_benchmark PRIVATE hacdcpf)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmark/helm_powerflow_benchmark.cpp")
  add_executable(helm_powerflow_benchmark benchmark/helm_powerflow_benchmark.cpp)
  target_link_libraries(helm_powerflow_benchmark PRIVATE hacdcpf)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmark/helm_merit_demo.cpp")
  add_executable(helm_merit_demo benchmark/helm_merit_demo.cpp)
  target_link_libraries(helm_merit_demo PRIVATE hacdcpf)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmark/cpf_runner.cpp")
  add_executable(cpf_runner benchmark/cpf_runner.cpp)
  target_link_libraries(cpf_runner PRIVATE hacdcpf)
endif()

target_include_directories(hacdcpf
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/decomposition
)

target_link_libraries(hacdcpf PUBLIC Eigen3::Eigen nlohmann_json::nlohmann_json spdlog::spdlog fmt::fmt)
if(HACDCPF_HAVE_OPENXLSX)
  target_link_libraries(hacdcpf PUBLIC OpenXLSX::OpenXLSX)
endif()
if(HACDCPF_HAVE_OPENDSS)
  target_link_libraries(hacdcpf PUBLIC dss_capi::dss_capi)
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_HAVE_OPENDSS=1)
endif()
if(HACDCPF_HAVE_OPENDSS_COMPARE)
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_HAVE_OPENDSS_COMPARE=1)

  add_executable(opendss_snapshot_adapter tools/opendss_snapshot_adapter.cpp)
  target_link_libraries(opendss_snapshot_adapter PRIVATE hacdcpf)
  target_compile_definitions(opendss_snapshot_adapter PRIVATE HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
  target_include_directories(opendss_snapshot_adapter PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/tools")
  hacdcpf_configure_opendss_runtime(opendss_snapshot_adapter)

  add_executable(opendss_pf_compare tools/opendss_pf_compare.cpp)
  target_link_libraries(opendss_pf_compare PRIVATE hacdcpf)
  target_compile_definitions(opendss_pf_compare PRIVATE HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
  target_include_directories(opendss_pf_compare PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/tools")
  hacdcpf_configure_opendss_runtime(opendss_pf_compare)
endif()
target_compile_definitions(hacdcpf PRIVATE HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
if(HACDCPF_ENABLE_FACTOR_BACKEND_B)
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_ENABLE_FACTOR_BACKEND_B=1)
else()
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_ENABLE_FACTOR_BACKEND_B=0)
endif()
# U.7.118 Phase 1: build vendored HiGHS HFactor as a standalone static lib.
# Phase 2: link it into hacdcpf so HFactorBackend can call into it.
if(HACDCPF_BUILD_HFACTOR)
  add_subdirectory(src/engine/kernel/linear_algebra/highs_factor)
  target_link_libraries(hacdcpf PRIVATE hacdcpf_hfactor)
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_HAVE_HFACTOR=1)
endif()
# U.7.118 Phase B: link the bundled HiGHS library so the engine can call
# HiGHS in-process for LP relaxation solves.
if(HACDCPF_HAVE_HIGHS_LIB)
  target_link_libraries(hacdcpf PRIVATE highs::highs)
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_HAVE_HIGHS_LIB=1)
  target_compile_definitions(hacdcpf PRIVATE HACDCPF_HIGHS_LIB_SOURCE="${HACDCPF_HIGHS_LIB_SOURCE}")
endif()
if(HACDCPF_HIGHS_EXECUTABLE)
  target_compile_definitions(hacdcpf PRIVATE HACDCPF_HIGHS_EXECUTABLE="${HACDCPF_HIGHS_EXECUTABLE}")
endif()
if(HACDCPF_IPOPT_EXECUTABLE)
  target_compile_definitions(hacdcpf PRIVATE HACDCPF_IPOPT_EXECUTABLE="${HACDCPF_IPOPT_EXECUTABLE}")
endif()
if(HACDCPF_SCIP_EXECUTABLE)
  target_compile_definitions(hacdcpf PRIVATE HACDCPF_SCIP_EXECUTABLE="${HACDCPF_SCIP_EXECUTABLE}")
endif()
if(HACDCPF_HAVE_SCIP_LIB)
  target_link_libraries(hacdcpf PRIVATE SCIP::libscip)
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_HAVE_SCIP_LIB=1)
  target_compile_definitions(hacdcpf PRIVATE HACDCPF_SCIP_LIB_SOURCE="${HACDCPF_SCIP_LIB_SOURCE}")
endif()

# Linux deployment stability knobs for Eigen.
if(NOT HACDCPF_EIGEN_VECTORIZE)
  target_compile_definitions(hacdcpf PUBLIC EIGEN_DONT_VECTORIZE=1 EIGEN_UNALIGNED_VECTORIZE=0)
endif()
if(HACDCPF_EIGEN_MAX_ALIGN_BYTES)
  target_compile_definitions(hacdcpf PUBLIC EIGEN_MAX_ALIGN_BYTES=${HACDCPF_EIGEN_MAX_ALIGN_BYTES})
endif()

if(MSVC)
  target_compile_options(hacdcpf PRIVATE /W4)
  if(HACDCPF_ENABLE_WERROR)
    target_compile_options(hacdcpf PRIVATE /WX)
  endif()
  # MSVC Release optimizations: full optimization + intrinsics + fast-math.
  target_compile_options(hacdcpf PRIVATE
    $<$<CONFIG:Release>:/O2 /Oi /fp:fast /DNDEBUG>
    $<$<CONFIG:RelWithDebInfo>:/O2 /Oi /Zi /DNDEBUG>)
else()
  target_compile_options(hacdcpf PRIVATE -Wall -Wextra -Wpedantic)
  if(HACDCPF_ENABLE_WERROR)
    target_compile_options(hacdcpf PRIVATE -Werror)
  endif()
  # GCC/Clang Release optimizations.
  target_compile_options(hacdcpf PRIVATE
    $<$<CONFIG:Release>:-O3 -DNDEBUG>
    $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>)
  if(HACDCPF_ENABLE_NATIVE_ARCH)
    target_compile_options(hacdcpf PRIVATE
      $<$<CONFIG:Release>:-march=native>
      $<$<CONFIG:RelWithDebInfo>:-march=native>)
  endif()
endif()

if(HACDCPF_HAVE_SUITESPARSE)
  target_include_directories(hacdcpf PUBLIC ${HACDCPF_SUITESPARSE_INCLUDE_DIRS})
  target_link_libraries(hacdcpf PUBLIC ${HACDCPF_SUITESPARSE_LIBRARIES})
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_HAVE_SUITESPARSE=1)
  if(HACDCPF_HAVE_UMFPACK)
    target_compile_definitions(hacdcpf PUBLIC HACDCPF_HAVE_UMFPACK=1)
  endif()
  if(HACDCPF_HAVE_KLU)
    target_compile_definitions(hacdcpf PUBLIC HACDCPF_HAVE_KLU=1)
  endif()
  if(APPLE)
    target_link_options(hacdcpf PUBLIC
      "-Wl,-rpath,/opt/homebrew/opt/suite-sparse/lib"
      "-Wl,-rpath,/usr/local/opt/suite-sparse/lib"
      "-Wl,-rpath,/opt/homebrew/opt/libomp/lib"
      "-Wl,-rpath,/usr/local/opt/libomp/lib")
  endif()
endif()

if(HACDCPF_HAVE_IPOPT)
  target_include_directories(hacdcpf PUBLIC ${HACDCPF_IPOPT_INCLUDE_DIRS})
  target_link_libraries(hacdcpf PUBLIC ${HACDCPF_IPOPT_LIBRARIES})
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_HAVE_IPOPT=1)
endif()

if(HACDCPF_HAVE_GUROBI)
  target_include_directories(hacdcpf PUBLIC ${HACDCPF_GUROBI_INCLUDE_DIRS})
  target_link_libraries(hacdcpf PUBLIC ${HACDCPF_GUROBI_LIBRARIES})
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_HAVE_GUROBI=1)
endif()

if(HACDCPF_HAVE_PAPILO)
  target_link_libraries(hacdcpf PUBLIC papilo)
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_HAVE_PAPILO=1)
endif()

# Apple Accelerate framework: optimized BLAS/LAPACK + Sparse direct solvers
if(APPLE)
  target_link_libraries(hacdcpf PUBLIC "-framework Accelerate")
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_HAVE_ACCELERATE=1)
endif()

# ── Python pybind11 module (Beta milestone) ─────────────────────────────────
option(HACDCPF_BUILD_PYTHON "Build hacdcpf_aml Python extension module" ON)
# Optionally override which Python interpreter is used for the extension module.
# Pass -DHACDCPF_PYTHON_EXECUTABLE=/path/to/python3.x on the cmake command line.
if(NOT HACDCPF_PYTHON_EXECUTABLE)
  find_program(HACDCPF_PYTHON_EXECUTABLE NAMES python3 python DOC "Python used for hacdcpf_aml")
endif()
if(HACDCPF_BUILD_PYTHON)
  # Forward the chosen interpreter to CMake's FindPython3 and to pybind11 so
  # that both pick up the same installation (include dirs, library, ABI tag).
  if(HACDCPF_PYTHON_EXECUTABLE)
    set(Python3_EXECUTABLE "${HACDCPF_PYTHON_EXECUTABLE}" CACHE FILEPATH
        "Python 3 interpreter used for hacdcpf_aml" FORCE)
  endif()

  # Locate pybind11 via the chosen interpreter's site-packages; fall back to
  # anything on CMAKE_PREFIX_PATH / system.
  if(HACDCPF_PYTHON_EXECUTABLE)
    execute_process(
      COMMAND "${HACDCPF_PYTHON_EXECUTABLE}" -c
              "import pybind11; print(pybind11.get_cmake_dir())"
      OUTPUT_VARIABLE _pybind11_cmake_dir
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
    )
  else()
    execute_process(
      COMMAND python3 -c "import pybind11; print(pybind11.get_cmake_dir())"
      OUTPUT_VARIABLE _pybind11_cmake_dir
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
    )
  endif()
  if(_pybind11_cmake_dir)
    list(APPEND CMAKE_PREFIX_PATH "${_pybind11_cmake_dir}")
  endif()

  # CMP0148: FindPythonInterp/FindPythonLibs were removed in CMake 3.27+.
  # Setting PYBIND11_FINDPYTHON=ON makes pybind11 use the modern FindPython instead.
  set(PYBIND11_FINDPYTHON ON)

  # When a specific interpreter is requested, run FindPython3 with EXACT version
  # before pybind11 does so. This prevents CMake's internal _Python_EXECUTABLE
  # tracking variable from being overridden by a system-wide Python on subsequent
  # reconfigures (e.g. Homebrew Python 3.14 shadowing pyenv Python 3.8).
  if(HACDCPF_PYTHON_EXECUTABLE)
    execute_process(
      COMMAND "${HACDCPF_PYTHON_EXECUTABLE}" -c
              "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}')"
      OUTPUT_VARIABLE _hacdcpf_python_version
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
    )
    if(_hacdcpf_python_version)
      # Pin Python3 to the exact version so CMake does not wander to another
      # installation on the PATH.
      set(Python3_ROOT_DIR "${HACDCPF_PYTHON_EXECUTABLE}" CACHE PATH "" FORCE)
      find_package(Python3 ${_hacdcpf_python_version} EXACT
                   COMPONENTS Interpreter Development REQUIRED)
      message(STATUS "hacdcpf_aml: using Python ${Python3_VERSION} "
                     "(${Python3_EXECUTABLE})")
    endif()
  endif()

  find_package(pybind11 QUIET)
  if(pybind11_FOUND)
    pybind11_add_module(hacdcpf_aml
      src/python/aml_bindings.cpp
      src/python/solver_bindings.cpp
      src/python/engine_bindings.cpp
    )
    target_link_libraries(hacdcpf_aml PRIVATE hacdcpf)
    target_include_directories(hacdcpf_aml PRIVATE
      ${CMAKE_SOURCE_DIR}/include
    )
    message(STATUS "hacdcpf_aml Python module: enabled (pybind11 ${pybind11_VERSION})")
  else()
    message(STATUS "pybind11 not found; hacdcpf_aml Python module will not be built")
  endif()

  # ── scikit-build-core install rule ───────────────────────────────────────
  # When built via `pip install` / `python -m build`, scikit-build-core sets
  # the SKBUILD cache variable.  We install the extension module into the wheel
  # root so it is importable as `import hacdcpf_aml` after installation.
  if(DEFINED SKBUILD AND pybind11_FOUND)
    install(TARGETS hacdcpf_aml DESTINATION .)
    # Also install the data directory so scripts can call load_matpower("data/...")
    # relative to the package root when HACDCPF_DATA_DIR is unset.
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/data/
            DESTINATION hacdcpf_data
            FILES_MATCHING
              PATTERN "*.m"
              PATTERN "*.json")
  endif()
endif()

find_program(HACDCPF_LATEXMK_EXECUTABLE latexmk)
if(HACDCPF_LATEXMK_EXECUTABLE)
  add_custom_target(technical_notebook_pdf
    COMMAND ${HACDCPF_LATEXMK_EXECUTABLE}
            -pdf
            -interaction=nonstopmode
            -halt-on-error
            -file-line-error
            -outdir=${CMAKE_CURRENT_SOURCE_DIR}/docs/technical_notebook
            ${CMAKE_CURRENT_SOURCE_DIR}/docs/technical_notebook/main.tex
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/docs/technical_notebook
    COMMENT "Building technical notebook PDF with latexmk"
    VERBATIM
  )
else()
  message(STATUS "latexmk not found; target technical_notebook_pdf will be unavailable")
endif()

if(HACDCPF_BUILD_TESTS)
  enable_testing()
  include(Catch2::Catch2WithMain OPTIONAL)
  include(CTest)
  if(TARGET Catch2::Catch2WithMain)
    # Nothing — Catch2 already provides catch_discover_tests via its CMake integration
  endif()

  # Helper: add a Catch2 test executable with an optional CTest tier label.
  # Usage: hacdcpf_add_test(<name> [TIER unit|integration|slow])
  macro(hacdcpf_add_test _name)
    add_executable(${_name} tests/${_name}.cpp)
    target_link_libraries(${_name} PRIVATE hacdcpf Catch2::Catch2WithMain)
    add_test(NAME ${_name} COMMAND ${_name})
    # Parse optional TIER argument
    set(_tier "")
    set(_args ${ARGN})
    list(LENGTH _args _nargs)
    if(_nargs GREATER 1)
      list(GET _args 0 _kw)
      if(_kw STREQUAL "TIER")
        list(GET _args 1 _tier)
      endif()
    endif()
    if(_tier)
      set_tests_properties(${_name} PROPERTIES LABELS "${_tier}")
    endif()
  endmacro()

  # ── Tier 1: unit tests (fast, <5 s) ──────────────────────────────────────────
  hacdcpf_add_test(test_aml_basic               TIER unit)
  hacdcpf_add_test(test_pm_dc_opf               TIER unit)
  hacdcpf_add_test(test_pm_scuc                 TIER unit)
  hacdcpf_add_test(test_beta_qp                 TIER unit)
  hacdcpf_add_test(test_pm_scuc_full            TIER unit)
  hacdcpf_add_test(test_pm_lindistflow          TIER unit)
  hacdcpf_add_test(test_aml_nlp                 TIER unit)
  hacdcpf_add_test(test_pm_ac_opf               TIER integration)
  hacdcpf_add_test(test_pm_ac_opf_ipopt         TIER integration)
  hacdcpf_add_test(test_pm_acdcopf_ipopt        TIER integration)
  hacdcpf_add_test(test_core_solver             TIER unit)
  hacdcpf_add_test(test_core_modules             TIER unit)
  hacdcpf_add_test(test_case_builders            TIER unit)
  hacdcpf_add_test(test_solver_handle            TIER unit)
  hacdcpf_add_test(test_matpower_parser_case59   TIER unit)
  hacdcpf_add_test(test_modeling_toolkit         TIER unit)
  hacdcpf_add_test(test_json_roundtrip_new_components TIER unit)
  hacdcpf_add_test(test_serialization_coverage   TIER unit)
  hacdcpf_add_test(test_paper_study_contracts TIER unit)
  hacdcpf_add_test(test_problem_types            TIER unit)
  hacdcpf_add_test(test_native_adapters          TIER unit)
  hacdcpf_add_test(test_dual_simplex             TIER unit)
  hacdcpf_add_test(test_sparse_dual_simplex      TIER unit)
  hacdcpf_add_test(test_sparse_lu_factor         TIER unit)
  hacdcpf_add_test(test_pdlp_solver              TIER unit)
  hacdcpf_add_test(test_ipm_lp_solver            TIER unit)
  hacdcpf_add_test(test_nlp_ipm_solver           TIER unit)
  hacdcpf_add_test(test_kkt_inertia              TIER unit)
  hacdcpf_add_test(test_ipm_filter               TIER unit)
  hacdcpf_add_test(test_ipm_restoration          TIER unit)
  hacdcpf_add_test(test_minlp_native_ipm         TIER unit)
  hacdcpf_add_test(test_cut_resolv_failure       TIER unit)
  hacdcpf_add_test(test_cglp_phase2              TIER unit)
  hacdcpf_add_test(test_external_adapters        TIER unit)
  hacdcpf_add_test(test_bc_phase_components      TIER unit)

  # ── Tier 2: integration tests (5–60 s) ──────────────────────────────────────
  hacdcpf_add_test(test_comprehensive_case       TIER integration)
  hacdcpf_add_test(test_enhanced_solver          TIER integration)
  hacdcpf_add_test(test_parity_julia             TIER integration)
  hacdcpf_add_test(test_basin_regression         TIER integration)
  hacdcpf_add_test(test_ac_opf_solver            TIER integration)
  hacdcpf_add_test(test_ac_opf_jacobian_fd       TIER integration)
  hacdcpf_add_test(test_parity_formulation_fd    TIER integration)
  hacdcpf_add_test(test_parity_hessian_fd        TIER integration)
  hacdcpf_add_test(test_parity_ipm               TIER integration)
  hacdcpf_add_test(test_projection_equivalence   TIER integration)
  hacdcpf_add_test(test_bus_merging              TIER integration)
  hacdcpf_add_test(test_lifecycle_smoke           TIER integration)
  hacdcpf_add_test(test_planning_smoke            TIER integration)
  hacdcpf_add_test(test_hybrid_distribution_planning_validation_smoke TIER integration)

  # ── Tier 3: slow / benchmark (>60 s, hidden by default) ─────────────────────
  hacdcpf_add_test(test_full_case_smoke          TIER slow)

  add_executable(benchmark_all_cases tests/benchmark_all_cases.cpp)
  target_link_libraries(benchmark_all_cases PRIVATE hacdcpf)

  add_executable(benchmark_ac_opf_cases tests/benchmark_ac_opf_cases.cpp)
  target_link_libraries(benchmark_ac_opf_cases PRIVATE hacdcpf)

  add_executable(benchmark_solver_smoke tests/benchmark_solver_smoke.cpp)
  target_link_libraries(benchmark_solver_smoke PRIVATE hacdcpf)

  add_executable(benchmark_planning_nested_bc tests/benchmark_planning_nested_bc.cpp)
  target_link_libraries(benchmark_planning_nested_bc PRIVATE hacdcpf)

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_planning_validation_cli.cpp")
    add_executable(run_planning_validation_cli tests/run_planning_validation_cli.cpp)
    target_link_libraries(run_planning_validation_cli PRIVATE hacdcpf)
  else()
    message(STATUS "Skipping run_planning_validation_cli: tests/run_planning_validation_cli.cpp not present")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_hybrid_distribution_validation_demo.cpp")
    add_executable(run_hybrid_distribution_validation_demo tests/run_hybrid_distribution_validation_demo.cpp)
    target_link_libraries(run_hybrid_distribution_validation_demo PRIVATE hacdcpf)
  else()
    message(STATUS "Skipping run_hybrid_distribution_validation_demo: tests/run_hybrid_distribution_validation_demo.cpp not present")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/benchmark_unexpected_fault_resilience_solvers.cpp")
    add_executable(benchmark_unexpected_fault_resilience_solvers
      tests/benchmark_unexpected_fault_resilience_solvers.cpp)
    target_link_libraries(benchmark_unexpected_fault_resilience_solvers PRIVATE hacdcpf)
  else()
    message(STATUS "Skipping benchmark_unexpected_fault_resilience_solvers: tests/benchmark_unexpected_fault_resilience_solvers.cpp not present")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_planning_experiment_suite.cpp")
    add_executable(run_planning_experiment_suite tests/run_planning_experiment_suite.cpp)
    target_link_libraries(run_planning_experiment_suite PRIVATE hacdcpf)
  else()
    message(STATUS "Skipping run_planning_experiment_suite: tests/run_planning_experiment_suite.cpp not present")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/analyze_case3_period_diffs.cpp")
    add_executable(analyze_case3_period_diffs tests/analyze_case3_period_diffs.cpp)
    target_link_libraries(analyze_case3_period_diffs PRIVATE hacdcpf)
  else()
    message(STATUS "Skipping analyze_case3_period_diffs: tests/analyze_case3_period_diffs.cpp not present")
  endif()

  hacdcpf_add_test(gen_excel_template)
  hacdcpf_add_test(test_excel_comprehensive_case TIER integration)

  add_executable(benchmark_excel_io tests/benchmark_excel_io.cpp)
  target_link_libraries(benchmark_excel_io PRIVATE hacdcpf)

  hacdcpf_add_test(test_all_components_pf        TIER integration)
  hacdcpf_add_test(test_all_components_opf       TIER integration)
  hacdcpf_add_test(test_branch_and_cut           TIER unit)
  hacdcpf_add_test(test_unit_commitment          TIER integration)
  hacdcpf_add_test(test_uc_performance           TIER integration)
  hacdcpf_add_test(test_time_series_pf           TIER integration)
  hacdcpf_add_test(test_annual_cost_contract    TIER integration)
  hacdcpf_add_test(test_time_series_distribution TIER integration)
  hacdcpf_add_test(test_time_series_distribution_mg_cluster TIER slow)
  hacdcpf_add_test(test_distribution_pf_crossval TIER integration)
  hacdcpf_add_test(test_short_circuit_crossval   TIER integration)
  hacdcpf_add_test(test_short_circuit_detailed   TIER integration)
  hacdcpf_add_test(test_opf_io                   TIER integration)
  hacdcpf_add_test(test_market_simulation         TIER integration)
  hacdcpf_add_test(test_market_julia_regression   TIER integration)
  hacdcpf_add_test(test_opf_pf_crossval          TIER integration)
  hacdcpf_add_test(test_acopf_dcopf_crossval     TIER integration)
  hacdcpf_add_test(test_topology_crossval        TIER slow)
  hacdcpf_add_test(test_powerflow_performance    TIER slow)
  hacdcpf_add_test(test_unified_pf_upgrade      TIER integration)
  hacdcpf_add_test(test_newton_krylov           TIER integration)
  hacdcpf_add_test(test_scaling_regression      TIER integration)
  hacdcpf_add_test(test_gap_closures            TIER integration)
  hacdcpf_add_test(test_hybrid_pf_control_combinations TIER integration)

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_annual_production_sim.cpp")
    hacdcpf_add_test(test_annual_production_sim TIER slow)
  else()
    message(STATUS "Skipping test_annual_production_sim: source not present")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_carbon_analysis.cpp")
    hacdcpf_add_test(test_carbon_analysis       TIER integration)
  else()
    message(STATUS "Skipping test_carbon_analysis: tests/test_carbon_analysis.cpp not present")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_carbon_extended.cpp")
    hacdcpf_add_test(test_carbon_extended       TIER integration)
  else()
    message(STATUS "Skipping test_carbon_extended: tests/test_carbon_extended.cpp not present")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_carbon_correctness.cpp")
    hacdcpf_add_test(test_carbon_correctness    TIER unit)
  else()
    message(STATUS "Skipping test_carbon_correctness: tests/test_carbon_correctness.cpp not present")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_reliability_assessment.cpp")
    hacdcpf_add_test(test_reliability_assessment TIER integration)
  else()
    message(STATUS "Skipping test_reliability_assessment: tests/test_reliability_assessment.cpp not present")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_resilience_assessment.cpp")
    hacdcpf_add_test(test_resilience_assessment TIER integration)
  else()
    message(STATUS "Skipping test_resilience_assessment: tests/test_resilience_assessment.cpp not present")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_visualization.cpp")
    hacdcpf_add_test(test_visualization         TIER integration)
  else()
    message(STATUS "Skipping test_visualization: tests/test_visualization.cpp not present")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_carbon_analysis_cli.cpp")
    add_executable(run_carbon_analysis_cli tests/run_carbon_analysis_cli.cpp)
    target_link_libraries(run_carbon_analysis_cli PRIVATE hacdcpf)
  else()
    message(STATUS "Skipping run_carbon_analysis_cli: tests/run_carbon_analysis_cli.cpp not present")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/export_case_builder_json.cpp")
    add_executable(export_case_builder_json tests/export_case_builder_json.cpp)
    target_link_libraries(export_case_builder_json PRIVATE hacdcpf)
  else()
    message(STATUS "Skipping export_case_builder_json: tests/export_case_builder_json.cpp not present")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_visualization_cli.cpp")
    add_executable(run_visualization_cli tests/run_visualization_cli.cpp)
    target_link_libraries(run_visualization_cli PRIVATE hacdcpf)
  else()
    message(STATUS "Skipping run_visualization_cli: tests/run_visualization_cli.cpp not present")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_market_res_sweep.cpp")
    add_executable(run_market_res_sweep tests/run_market_res_sweep.cpp)
    target_link_libraries(run_market_res_sweep PRIVATE hacdcpf)
  else()
    message(STATUS "Skipping run_market_res_sweep: tests/run_market_res_sweep.cpp not present")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_gui_server.cpp")
    add_executable(run_gui_server tests/run_gui_server.cpp)
    if(TARGET httplib::httplib)
      target_link_libraries(run_gui_server PRIVATE hacdcpf httplib::httplib)
    elseif(TARGET httplib)
      target_link_libraries(run_gui_server PRIVATE hacdcpf httplib)
    else()
      target_link_libraries(run_gui_server PRIVATE hacdcpf)
      if(DEFINED httplib_SOURCE_DIR)
        target_include_directories(run_gui_server PRIVATE ${httplib_SOURCE_DIR})
      endif()
    endif()
  else()
    message(STATUS "Skipping run_gui_server: tests/run_gui_server.cpp not present")
  endif()

  if(HACDCPF_HAVE_GUROBI AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_three_stage_stochastic.cpp")
    add_executable(run_three_stage_stochastic tests/run_three_stage_stochastic.cpp)
    target_include_directories(run_three_stage_stochastic PRIVATE ${HACDCPF_GUROBI_INCLUDE_DIRS})
    # Link both C++ and C Gurobi libraries
    find_library(HACDCPF_GUROBI_CXX_LIBRARY
      NAMES gurobi_c++
      PATHS ${HACDCPF_GUROBI_INCLUDE_DIRS}/../lib
      NO_DEFAULT_PATH)
    if(HACDCPF_GUROBI_CXX_LIBRARY)
      target_link_libraries(run_three_stage_stochastic PRIVATE
        ${HACDCPF_GUROBI_CXX_LIBRARY} ${HACDCPF_GUROBI_LIBRARIES})
    else()
      target_link_libraries(run_three_stage_stochastic PRIVATE ${HACDCPF_GUROBI_LIBRARIES})
    endif()
  else()
    message(STATUS "Skipping run_three_stage_stochastic: Gurobi not available or source not present")
  endif()

  # Deterministic dispatch baseline
  if(HACDCPF_HAVE_GUROBI AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_deterministic_dispatch.cpp")
    add_executable(run_deterministic_dispatch tests/run_deterministic_dispatch.cpp)
    target_include_directories(run_deterministic_dispatch PRIVATE ${HACDCPF_GUROBI_INCLUDE_DIRS})
    if(HACDCPF_GUROBI_CXX_LIBRARY)
      target_link_libraries(run_deterministic_dispatch PRIVATE
        ${HACDCPF_GUROBI_CXX_LIBRARY} ${HACDCPF_GUROBI_LIBRARIES})
    else()
      target_link_libraries(run_deterministic_dispatch PRIVATE ${HACDCPF_GUROBI_LIBRARIES})
    endif()
  endif()

  # Benders decomposition solver
  if(HACDCPF_HAVE_GUROBI AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_benders_three_stage.cpp")
    add_executable(run_benders_three_stage tests/run_benders_three_stage.cpp)
    target_include_directories(run_benders_three_stage PRIVATE ${HACDCPF_GUROBI_INCLUDE_DIRS})
    if(HACDCPF_GUROBI_CXX_LIBRARY)
      target_link_libraries(run_benders_three_stage PRIVATE
        ${HACDCPF_GUROBI_CXX_LIBRARY} ${HACDCPF_GUROBI_LIBRARIES})
    else()
      target_link_libraries(run_benders_three_stage PRIVATE ${HACDCPF_GUROBI_LIBRARIES})
    endif()
  endif()

  # Comprehensive case comparison
  if(HACDCPF_HAVE_GUROBI AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_comprehensive_case.cpp")
    add_executable(run_comprehensive_case tests/run_comprehensive_case.cpp)
    target_include_directories(run_comprehensive_case PRIVATE ${HACDCPF_GUROBI_INCLUDE_DIRS})
    if(HACDCPF_GUROBI_CXX_LIBRARY)
      target_link_libraries(run_comprehensive_case PRIVATE
        ${HACDCPF_GUROBI_CXX_LIBRARY} ${HACDCPF_GUROBI_LIBRARIES})
    else()
      target_link_libraries(run_comprehensive_case PRIVATE ${HACDCPF_GUROBI_LIBRARIES})
    endif()
  endif()

  # C0-C4 ablation protocol runner
  if(HACDCPF_HAVE_GUROBI AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_ablation_c0_c4.cpp")
    add_executable(run_ablation_c0_c4 tests/run_ablation_c0_c4.cpp)
    target_include_directories(run_ablation_c0_c4 PRIVATE ${HACDCPF_GUROBI_INCLUDE_DIRS})
    if(HACDCPF_GUROBI_CXX_LIBRARY)
      target_link_libraries(run_ablation_c0_c4 PRIVATE
        ${HACDCPF_GUROBI_CXX_LIBRARY} ${HACDCPF_GUROBI_LIBRARIES})
    else()
      target_link_libraries(run_ablation_c0_c4 PRIVATE ${HACDCPF_GUROBI_LIBRARIES})
    endif()
  endif()

  # Stochastic Decomposition Library
  # Always compile native-backend files (supports runtime solver selection:
  # Native B&C, HiGHS, Gurobi). Gurobi/HiGHS are used via the adapters
  # already linked into hacdcpf.
  file(GLOB DECOMP_SRCS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/decomposition/*_native.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/decomposition/stochastic_decomp.cpp")

    if(DECOMP_SRCS)
      add_library(hacdcpf_decomp STATIC ${DECOMP_SRCS})
      target_include_directories(hacdcpf_decomp
        PUBLIC  include
        PRIVATE src/decomposition)

      target_link_libraries(hacdcpf_decomp PUBLIC hacdcpf Eigen3::Eigen)
      message(STATUS "hacdcpf_decomp: ${DECOMP_SRCS}")

      # Decomposition test suite
      if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_stochastic_decomp.cpp")
        add_executable(test_stochastic_decomp tests/test_stochastic_decomp.cpp)
        target_link_libraries(test_stochastic_decomp PRIVATE
          hacdcpf_decomp Catch2::Catch2WithMain)
        target_include_directories(test_stochastic_decomp PRIVATE
          include ${HACDCPF_GUROBI_INCLUDE_DIRS})
        add_test(NAME test_stochastic_decomp COMMAND test_stochastic_decomp)
        set_tests_properties(test_stochastic_decomp PROPERTIES LABELS "integration")
      endif()

      # Decomposition scaling / performance tests
      if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_decomp_scaling.cpp")
        add_executable(test_decomp_scaling tests/test_decomp_scaling.cpp)
        target_link_libraries(test_decomp_scaling PRIVATE
          hacdcpf_decomp Catch2::Catch2WithMain)
        target_include_directories(test_decomp_scaling PRIVATE
          include ${HACDCPF_GUROBI_INCLUDE_DIRS})
        add_test(NAME test_decomp_scaling COMMAND test_decomp_scaling)
        set_tests_properties(test_decomp_scaling PROPERTIES LABELS "performance")
      endif()

      # Three-stage nested decomposition tests
      if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_three_stage_nested_decomp.cpp")
        add_executable(test_three_stage_nested_decomp
          tests/test_three_stage_nested_decomp.cpp)
        target_link_libraries(test_three_stage_nested_decomp PRIVATE
          hacdcpf_decomp Catch2::Catch2WithMain)
        target_include_directories(test_three_stage_nested_decomp PRIVATE
          include ${HACDCPF_GUROBI_INCLUDE_DIRS})
        add_test(NAME test_three_stage_nested_decomp COMMAND test_three_stage_nested_decomp)
        set_tests_properties(test_three_stage_nested_decomp PROPERTIES LABELS "integration")
      endif()

      # Native solver tests
      if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_decomp_native.cpp")
        add_executable(test_decomp_native tests/test_decomp_native.cpp)
        target_link_libraries(test_decomp_native PRIVATE
          hacdcpf_decomp Catch2::Catch2WithMain)
        target_include_directories(test_decomp_native PRIVATE
          include src/decomposition)
        add_test(NAME test_decomp_native COMMAND test_decomp_native)
        set_tests_properties(test_decomp_native PROPERTIES LABELS "integration")
      endif()

      if(TARGET run_ablation_c0_c4)
        target_link_libraries(run_ablation_c0_c4 PRIVATE hacdcpf_decomp Eigen3::Eigen)
      endif()
    endif()

  # ── Three-phase NR tests ─────────────────────────────────────────────────────
  hacdcpf_add_test(test_three_phase_nr_phase_indexing      TIER integration)
  hacdcpf_add_test(test_three_phase_nr_load_models         TIER integration)
  hacdcpf_add_test(test_three_phase_nr_transformer_source  TIER integration)
  hacdcpf_add_test(test_three_phase_nr_regulator           TIER integration)

  # ── Phase-domain tests (exist-guarded, depend on data assets) ───────────────
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_phase_domain_multi_case_compact.cpp")
    add_executable(test_phase_domain_multi_case_compact
      tests/test_phase_domain_multi_case_compact.cpp)
    target_link_libraries(test_phase_domain_multi_case_compact PRIVATE hacdcpf Catch2::Catch2WithMain)
    add_test(NAME test_phase_domain_multi_case_compact COMMAND test_phase_domain_multi_case_compact)
    set_tests_properties(test_phase_domain_multi_case_compact PROPERTIES LABELS "integration")
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_phase_domain_multi_case_compact_julia.cpp")
    add_executable(test_phase_domain_multi_case_compact_julia
      tests/test_phase_domain_multi_case_compact_julia.cpp)
    target_link_libraries(test_phase_domain_multi_case_compact_julia PRIVATE hacdcpf Catch2::Catch2WithMain)
    add_test(NAME test_phase_domain_multi_case_compact_julia COMMAND test_phase_domain_multi_case_compact_julia)
    set_tests_properties(test_phase_domain_multi_case_compact_julia PROPERTIES LABELS "integration")
  endif()

  if(HACDCPF_HAVE_OPENDSS)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_phase_domain_ieee123.cpp")
      add_executable(test_phase_domain_ieee123
        tests/test_phase_domain_ieee123.cpp)
      target_link_libraries(test_phase_domain_ieee123 PRIVATE hacdcpf Catch2::Catch2WithMain)
      add_test(NAME test_phase_domain_ieee123 COMMAND test_phase_domain_ieee123)
      set_tests_properties(test_phase_domain_ieee123 PROPERTIES LABELS "integration")
    endif()

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_phase_domain_ieee123_generic_solver.cpp")
      add_executable(test_phase_domain_ieee123_generic_solver
        tests/test_phase_domain_ieee123_generic_solver.cpp)
      target_link_libraries(test_phase_domain_ieee123_generic_solver PRIVATE hacdcpf Catch2::Catch2WithMain)
      add_test(NAME test_phase_domain_ieee123_generic_solver COMMAND test_phase_domain_ieee123_generic_solver)
      set_tests_properties(test_phase_domain_ieee123_generic_solver PROPERTIES LABELS "integration")
    endif()
  endif()

  # ── OpenDSS PF compare (requires dss_capi + compare tools) ──────────────────
  if(HACDCPF_HAVE_OPENDSS_COMPARE AND
     EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_opendss_pf_compare.cpp")
    hacdcpf_add_test(test_opendss_pf_compare TIER integration)
    target_compile_definitions(test_opendss_pf_compare PRIVATE HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
    target_include_directories(test_opendss_pf_compare PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/tools")
    hacdcpf_configure_opendss_runtime(test_opendss_pf_compare)
  endif()

endif()
