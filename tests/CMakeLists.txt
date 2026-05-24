cmake_minimum_required(VERSION 3.20)

function(hacdcpf_add_test target)
  add_executable(${target} ${ARGN})
  target_link_libraries(${target} PRIVATE hacdcpf::hacdcpf Catch2::Catch2WithMain)
  target_compile_definitions(${target} PRIVATE
    HACDCPF_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/data"
    HACDCPF_PROJECT_ROOT="${CMAKE_SOURCE_DIR}"
  )
  catch_discover_tests(${target})
endfunction()

hacdcpf_add_test(test_hacdcpf             test_hacdcpf.cpp)
hacdcpf_add_test(test_all_components_pf   test_all_components_pf.cpp)
hacdcpf_add_test(test_unified_pf_upgrade  test_unified_pf_upgrade.cpp)
hacdcpf_add_test(test_acopf_dcopf_crossval test_acopf_dcopf_crossval.cpp)
hacdcpf_add_test(test_enhanced_solver     test_enhanced_solver.cpp)
hacdcpf_add_test(test_scaling_regression  test_scaling_regression.cpp)
hacdcpf_add_test(test_hacdcdss            test_hacdcdss.cpp)

# ── New comprehensive test suites ─────────────────────────────────────────────

# I/O: JSON round-trip, MATPOWER parsing, safe try_ variants, OPF/PF result ser.
hacdcpf_add_test(test_io_json             test_io_json.cpp)

# Rich components: Transformer2W/3W, PVSystem, Storage, VSCConverter, etc.
hacdcpf_add_test(test_rich_components     test_rich_components.cpp)

# MATPOWER cases: Tier1/Tier2/Tier3 convergence regression suite.
hacdcpf_add_test(test_matpower_cases      test_matpower_cases.cpp)

# Validation: ValidationReport, ValidationLevel, deliberate-fault injection.
hacdcpf_add_test(test_validation          test_validation.cpp)

# Solver capabilities: SolverCapabilities struct and get_solver_capabilities().
hacdcpf_add_test(test_solver_capabilities test_solver_capabilities.cpp)

# Advanced power flow: multi-island detection/solving, distributed slack buses,
# PV/PQ conversion control – patterns adapted from luosipeng branch of
# HybridACDCPowerSystemsPlanning.
hacdcpf_add_test(test_advanced_pf         test_advanced_pf.cpp)

# Three-phase power flow: data model, projection-based solve, meshed/multi-source
# topologies, transformer, distribution PF stub, and JSON round-trip.
hacdcpf_add_test(test_three_phase_pf      test_three_phase_pf.cpp)

# Multi-port ER / multi-mode VPP: robustness tests for EnergyRouter ports with
# mixed control modes (VF + PQ + Droop) and VirtualPowerPlants operating in
# generation, demand-response, and reactive-support modes simultaneously.
hacdcpf_add_test(test_multiport_vpp_er    test_multiport_vpp_er.cpp)

# Performance test: MATPOWER cases — built with release-level optimisation
# regardless of overall build type, so timing numbers are meaningful.
add_executable(test_powerflow_performance test_powerflow_performance.cpp)
target_link_libraries(test_powerflow_performance PRIVATE hacdcpf::hacdcpf Catch2::Catch2WithMain)
target_compile_definitions(test_powerflow_performance PRIVATE
  HACDCPF_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/data"
  HACDCPF_PROJECT_ROOT="${CMAKE_SOURCE_DIR}"
)
target_compile_options(test_powerflow_performance PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-O3 -DNDEBUG>
  $<$<CXX_COMPILER_ID:MSVC>:/O2 /DNDEBUG>
)
catch_discover_tests(test_powerflow_performance
  TEST_SUFFIX " [perf]"
  PROPERTIES LABELS "performance;slow"
)

# Topology cross-validation: connectivity checks + ONR via B&C vs exhaustive
# enumeration (6-bus), 33bw ONR with Newton PF verification, DER integration.
hacdcpf_add_test(test_topology_crossval test_topology_crossval.cpp)

# Comprehensive cross-module integration: PF ↔ OPF ↔ ONR ↔ Carbon analysis.
# Catches high-level contradictions across all four solver modules:
#   A+B: IEEE 14-bus — PF/OPF consistency + carbon balance (tests A1–A6, B1–B4)
#   C+D: IEEE 33-bus BW — ONR loss improvement + carbon balance (tests C1–C4, D1–D3)
hacdcpf_add_test(test_crossmodule_integration test_crossmodule_integration.cpp)
