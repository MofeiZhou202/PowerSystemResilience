cmake_minimum_required(VERSION 3.20)

project(HybridACDCDistributionSystemsSimulation VERSION 0.1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Embedded dependencies (SCIP / HiGHS) declare cmake_minimum_required below the
# floor that CMake >= 4.0 accepts. Pin a compatibility policy version so they
# configure without each developer having to pass it on the command line
# (where PowerShell also tends to mangle the "=3.5" argument).
if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()

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
option(HACDCPF_BUILD_TESTS    "Build unit/integration tests"               ON)
option(HACDCPF_ENABLE_WERROR  "Treat warnings as errors for core library"  OFF)
option(HACDCPF_ENABLE_NATIVE_ARCH
       "Enable -march=native for host-specific builds"                     OFF)
# IO optional features: OpenDSS and Excel require external libs not bundled here
option(HACDCPF_ENABLE_OPENDSS "Build OpenDSS bridge (requires dss_capi)"   OFF)
option(HACDCPF_ENABLE_ETAP    "Build ETAP Excel import/export (requires OpenXLSX)" OFF)

# ── Dependencies (pulls in MIPSolvers + transitive: Eigen3, fmt, nlohmann_json,
#                 HiGHS, optional Ipopt/SCIP) ─────────────────────────────────
# Embedded SCIP uses POSIX sigaction(), which the Windows/UCRT CRT does not
# provide. Define SCIP_NO_SIGACTION so SCIP falls back to signal() there.
# Applied before the dependency add_subdirectory so it propagates to SCIP.
if(WIN32)
  add_compile_definitions(SCIP_NO_SIGACTION)
endif()
include(cmake/Dependencies.cmake)

# ── Core library ──────────────────────────────────────────────────────────────
#
# hacdcpf is a standalone static library that includes:
#
#   model               – AC/DC network data structures, system topology
#   power_models        – ACOPF/DCOPF/LinDistFlow/SCUC algebraic model builders
#   power_flow          – Newton-Raphson, FDPF, HELM, homotopy, adaptive, etc.
#   optimal_power_flow  – AC-OPF, DC-OPF, parity, reactive power optimisation
#   io                  – JSON, MATPOWER, case builders (OpenDSS/Excel optional)
#   api                 – Public high-level entry points
#
# The only required external dependency is mipsolvers::mipsolvers, which
# transitively provides Eigen3, fmt, nlohmann_json, HiGHS, and Ipopt/SCIP.
# No Homebrew or system-installed library is required.

set(HACDCPF_SOURCES
  # ── model ────────────────────────────────────────────────────────────────
  src/model/network_utils.cpp
  src/model/unit_conversion.cpp
  # ── power_models ─────────────────────────────────────────────────────────
  src/power_models/acopf_builder.cpp
  src/power_models/acdcopf_builder.cpp
  src/power_models/dc_opf_builder.cpp
  src/power_models/lindistflow_builder.cpp
  src/power_models/scuc_builder.cpp
  # ── power_flow ───────────────────────────────────────────────────────────
  src/power_flow/ac.cpp
  src/power_flow/ac_kernel.cpp
  src/power_flow/ac_linearized_pf.cpp
  src/power_flow/adaptive_solver.cpp
  src/power_flow/admittance_builder.cpp
  src/power_flow/branch_flow.cpp
  src/power_flow/converter_model.cpp
  src/power_flow/dc.cpp
  src/power_flow/dc_solver.cpp
  src/power_flow/distributed_slack_solver.cpp
  src/power_flow/fdpf_solver.cpp
  src/power_flow/helm_solver.cpp
  src/power_flow/homotopy_continuation.cpp
  src/power_flow/hybrid.cpp
  src/power_flow/island_detector.cpp
  src/power_flow/jacobian_builder.cpp
  src/power_flow/jacobian_pattern.cpp
  src/power_flow/lm_trust_region.cpp
  src/power_flow/newton_krylov.cpp
  src/power_flow/nonlinear_scaling.cpp
  src/power_flow/nonmonotone_linesearch.cpp
  src/power_flow/pf_injection_assembly.cpp
  src/power_flow/residual_evaluator.cpp
  src/power_flow/robust_nonlinear_options.cpp
  src/power_flow/solver_data.cpp
  src/power_flow/voltage_stability.cpp
  src/power_flow/workspace.cpp
  src/power_flow/newton_solver.cpp
  src/power_flow/jacobian_check.cpp
  src/power_flow/jacobian_check.cpp
  # ── optimal_power_flow ───────────────────────────────────────────────────
  src/optimal_power_flow/ac_opf.cpp
  src/optimal_power_flow/dc_opf.cpp
  src/optimal_power_flow/parity_formulation.cpp
  src/optimal_power_flow/parity_ipm.cpp
  src/optimal_power_flow/reactive_power_opt.cpp
  # ── io (always-available: JSON / MATPOWER / case builders) ───────────────
  src/io/json_io.cpp
  src/io/matpower_parser.cpp
  src/io/case_builders.cpp
  # ── validation ────────────────────────────────────────────────────────────
  src/validation/validate_system.cpp
  # ── api ──────────────────────────────────────────────────────────────────
  src/api/hacdcpf.cpp
  # ── EV power-traffic coupling ───────────────────────────────────────────
  src/ev_power_traffic/ev_power_traffic_simulation.cpp
  src/ev_power_traffic/assignment_lp.cpp
  src/ev_power_traffic/joint_social_welfare.cpp
  src/ev_power_traffic/ctm_propagation.cpp
  src/ev_power_traffic/ltm_propagation.cpp
  src/ev_power_traffic/ctm_joint_welfare.cpp
  src/ev_power_traffic/joint_optimizer.cpp
  src/ev_power_traffic/joint_optimizer_full_nlp.cpp
  src/ev_power_traffic/ctm_so_lp.cpp
  src/ev_power_traffic/ltm_so_lp.cpp
  src/ev_power_traffic/ctm_due_vi.cpp
  src/ev_power_traffic/infra_design_milp.cpp
  src/ev_power_traffic/ltm_mpc.cpp  # ── network reconfiguration (topology analysis + ONR) ──────────────────
  src/network_reconfiguration/topology_analysis.cpp
  src/network_reconfiguration/topology_reconfiguration.cpp
  # ── graph analysis & reduction ────────────────────────────────────────
  src/graph/power_system_graph.cpp
  src/graph/topology_analysis.cpp
  src/graph/switch_contraction.cpp
  src/graph/reduction_plan.cpp
  src/graph/series_reduction.cpp
  src/graph/kron_reduction.cpp
  src/graph/result_recovery.cpp
  # ── analysis ────────────────────────────────────────────────────────
  src/analysis/short_circuit.cpp
  src/analysis/dc_short_circuit.cpp
  src/reliability/three_stage_reliability.cpp
  # ── analysis stubs ───────────────────────────────────────────────────
  src/carbon_analysis/carbon_analysis.cpp
  src/carbon_analysis/annual_carbon_analysis.cpp
  src/power_flow/three_phase_nr.cpp
  src/reliability/reliability_assessment.cpp
  src/reliability/reliability_data.cpp
  src/resilience/resilience_assessment.cpp
  src/resilience/resilience_restoration_mip.cpp
  src/time_series/time_series_pf.cpp
  src/time_series/annual_production_sim.cpp
  src/time_series/lifecycle_simulation.cpp
  # ── power_models stubs (AML model builders) ──────────────────────────
  src/power_models/acopf_builder.cpp
  src/power_models/acdcopf_builder.cpp
  src/power_models/dc_opf_builder.cpp
  src/power_models/lindistflow_builder.cpp
  src/power_models/scuc_builder.cpp)

if(HACDCPF_ENABLE_ETAP)
  list(APPEND HACDCPF_SOURCES src/io/etap_io.cpp)
endif()
if(HACDCPF_ENABLE_OPENDSS)
  list(APPEND HACDCPF_SOURCES src/io/opendss_bridge.cpp)
endif()

add_library(hacdcpf STATIC ${HACDCPF_SOURCES})
add_library(hacdcpf::hacdcpf ALIAS hacdcpf)

# ── Include paths ─────────────────────────────────────────────────────────────
target_include_directories(hacdcpf
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    # Compat shim directory (e.g. spdlog -> fmt bridge)
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/compat>
)

# ── Compile definitions ───────────────────────────────────────────────────────
target_compile_definitions(hacdcpf PRIVATE
  HACDCPF_PROJECT_ROOT="${CMAKE_SOURCE_DIR}"
)
if(HACDCPF_ENABLE_ETAP)
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_ENABLE_ETAP)
endif()
if(NOT HACDCPF_ENABLE_OPENDSS)
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_NO_OPENDSS)
endif()

# ── Link against MIPSolvers (brings Eigen3, fmt, nlohmann_json, HiGHS, Ipopt) ─
target_link_libraries(hacdcpf PUBLIC mipsolvers::mipsolvers)

# ── ETAP Excel I/O backend (OpenXLSX) ────────────────────────────────────
if(HACDCPF_ENABLE_ETAP AND TARGET OpenXLSX::OpenXLSX)
  target_link_libraries(hacdcpf PUBLIC OpenXLSX::OpenXLSX)
endif()

# ── Compiler flags ────────────────────────────────────────────────────────────
if(HACDCPF_ENABLE_WERROR)
  target_compile_options(hacdcpf PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Werror>
    $<$<CXX_COMPILER_ID:MSVC>:/WX>)
endif()

if(HACDCPF_ENABLE_NATIVE_ARCH)
  target_compile_options(hacdcpf PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-march=native>)
endif()

# ── ETAP <-> JSON command-line converter (requires ETAP/OpenXLSX) ─────────────
if(HACDCPF_ENABLE_ETAP AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/etap_convert.cpp")
  add_executable(etap_convert tools/etap_convert.cpp)
  target_link_libraries(etap_convert PRIVATE hacdcpf::hacdcpf)
endif()

# ── Tests ─────────────────────────────────────────────────────────────────────
if(HACDCPF_BUILD_TESTS)
  include(CTest)
  enable_testing()
  add_subdirectory(tests)
endif()
