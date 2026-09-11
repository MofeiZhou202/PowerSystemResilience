cmake_minimum_required(VERSION 3.20)

# Keep Python test bytecode out of the source tree on every platform.
if(WIN32)
  file(TO_CMAKE_PATH "$ENV{TEMP}/hacdcpf_pycache" HACDCPF_PYTHON_CACHE_DIR)
else()
  set(HACDCPF_PYTHON_CACHE_DIR "/tmp/hacdcpf_pycache")
endif()

function(hacdcpf_add_test target)
  add_executable(${target} ${ARGN})
  target_link_libraries(${target} PRIVATE hacdcpf::hacdcpf Catch2::Catch2WithMain)
  target_compile_definitions(${target} PRIVATE
    HACDCPF_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/data"
    HACDCPF_MATPOWER_DATA_DIR="${CMAKE_SOURCE_DIR}/external_data/matpower"
    HACDCPF_PROJECT_ROOT="${CMAKE_SOURCE_DIR}"
  )
  hacdcpf_copy_opendss_runtime(${target})
  catch_discover_tests(${target})
endfunction()

function(hacdcpf_add_plain_test target)
  add_executable(${target} ${ARGN})
  target_link_libraries(${target} PRIVATE hacdcpf::hacdcpf)
  target_compile_definitions(${target} PRIVATE
    HACDCPF_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/data"
    HACDCPF_MATPOWER_DATA_DIR="${CMAKE_SOURCE_DIR}/external_data/matpower"
    HACDCPF_PROJECT_ROOT="${CMAKE_SOURCE_DIR}"
  )
  hacdcpf_copy_opendss_runtime(${target})
  add_test(NAME ${target} COMMAND ${target})
endfunction()

hacdcpf_add_test(test_hacdcpf             test_hacdcpf.cpp)
hacdcpf_add_test(test_all_components_pf   test_all_components_pf.cpp)
hacdcpf_add_test(test_multiscale_comprehensive test_multiscale_comprehensive.cpp)
hacdcpf_add_test(test_unified_pf_upgrade  test_unified_pf_upgrade.cpp)
hacdcpf_add_test(test_acopf_dcopf_crossval test_acopf_dcopf_crossval.cpp)
hacdcpf_add_test(test_opf_solver_backends test_opf_solver_backends.cpp)
hacdcpf_add_test(test_thread_pool test_thread_pool.cpp)
hacdcpf_add_test(test_review_regressions test_review_regressions.cpp)
hacdcpf_add_test(test_reactive_power_opt test_reactive_power_opt.cpp)
hacdcpf_add_test(test_enhanced_solver     test_enhanced_solver.cpp)
hacdcpf_add_test(test_uc_external_grid     test_uc_external_grid.cpp)
hacdcpf_add_test(test_uc_demand_response   test_uc_demand_response.cpp)
hacdcpf_add_test(test_uc_dispatchable_pv   test_uc_dispatchable_pv.cpp)
hacdcpf_add_test(test_uc_microgrid         test_uc_microgrid.cpp)
hacdcpf_add_test(test_uc_dc_branch_flows   test_uc_dc_branch_flows.cpp)
hacdcpf_add_test(test_uc_storage_degradation test_uc_storage_degradation.cpp)
hacdcpf_add_test(test_uc_storage_efficiency test_uc_storage_efficiency.cpp)
hacdcpf_add_test(test_uc_vpp               test_uc_vpp.cpp)
hacdcpf_add_test(test_uc_energy_router     test_uc_energy_router.cpp)
hacdcpf_add_test(test_uc_mobile_storage    test_uc_mobile_storage.cpp)
# MIPSolvers full-dev also owns a test_market_simulation target. Keep this
# project's executable target distinct so both dependency and application
# integration suites can be configured in the same CTest tree.
hacdcpf_add_test(hacdcpf_test_market_simulation test_market_simulation.cpp)
hacdcpf_add_test(test_southern_market test_southern_market.cpp)
if(UNIX)
  add_executable(run_southern_market_benchmark run_southern_market_benchmark.cpp)
  target_link_libraries(run_southern_market_benchmark PRIVATE hacdcpf)
  target_compile_definitions(run_southern_market_benchmark PRIVATE HACDCPF_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/data")
  hacdcpf_copy_opendss_runtime(run_southern_market_benchmark)
  option(HACDCPF_ENABLE_MARKET_SCALE_TESTS "Run licensed Gurobi 2000-bus market benchmarks" OFF)
  if(HACDCPF_ENABLE_MARKET_SCALE_TESTS)
    add_test(NAME southern_market_2000_benchmark COMMAND run_southern_market_benchmark
      2000 "${CMAKE_BINARY_DIR}/market-performance-2000.json" 600 compact)
    set_tests_properties(southern_market_2000_benchmark PROPERTIES
      LABELS "market;performance;gurobi" TIMEOUT 2400 RUN_SERIAL TRUE)
  endif()
endif()
hacdcpf_add_test(test_market_forecast test_market_forecast.cpp)
hacdcpf_add_test(test_integrated_energy_campus test_integrated_energy_campus.cpp)
hacdcpf_add_test(test_scaling_regression  test_scaling_regression.cpp)
hacdcpf_add_test(test_hacdcdss            test_hacdcdss.cpp)

# ── New comprehensive test suites ─────────────────────────────────────────────

# I/O: JSON round-trip, MATPOWER parsing, safe try_ variants, OPF/PF result ser.
hacdcpf_add_test(test_io_json             test_io_json.cpp)

# I/O: MATPOWER unit-discipline contract (§6.3).
hacdcpf_add_test(test_io_matpower         test_io_matpower.cpp)

# I/O: PSD-BPA / DSP card-file (.dat) importer (structure + DSP-solution parity).
hacdcpf_add_test(test_bpa_io              test_bpa_io.cpp)
hacdcpf_add_test(test_bpa_dsp_compare     test_bpa_dsp_compare.cpp)

# I/O: round-trip / conformance contract (§7).
hacdcpf_add_test(test_io_roundtrip        test_io_roundtrip.cpp)

# I/O: bounded CIM / CGMES 3.0 (EQ+SSH) import/export (§5).
hacdcpf_add_test(test_io_cim              test_io_cim.cpp)
hacdcpf_add_test(test_io_svg_distribution test_io_svg_distribution.cpp)

# I/O: text-format conversion for OpenDSS/GridLAB-D without requiring external
# engines, used by component-level cross-check and benchmark harnesses.
hacdcpf_add_test(test_external_grid_io    test_external_grid_io.cpp)

# I/O: explicit rich-component coverage contract for internal JSON, canonical
# native models, GridLAB-D text snapshots, and OpenDSS text snapshots.
hacdcpf_add_test(test_component_io_mapping test_component_io_mapping.cpp)
hacdcpf_add_test(test_model_semantics_contract test_model_semantics_contract.cpp)

# Dynamics: single-source parameter catalog stays aligned with the builder.
hacdcpf_add_test(test_dynamic_model_catalog test_dynamic_model_catalog.cpp)

# I/O: ETAP Excel round-circle (excel in / excel out). Requires OpenXLSX, so it
# is only registered when ETAP support is compiled into the core library.
if(HACDCPF_ENABLE_ETAP)
  hacdcpf_add_test(test_io_etap           test_io_etap.cpp)
endif()

if(TARGET transient_validation_matrix)
  add_test(NAME transient_native_disturbance_matrix
    COMMAND "$<TARGET_FILE:transient_validation_matrix>"
            "${CMAKE_CURRENT_BINARY_DIR}/transient_native_disturbance_matrix.json")
  set_tests_properties(transient_native_disturbance_matrix PROPERTIES
    LABELS "dynamics;transient;validation" TIMEOUT 120)
endif()

if(TARGET terminal_state_boundary_study)
  add_test(NAME terminal_state_boundary_study_smoke
    COMMAND "$<TARGET_FILE:terminal_state_boundary_study>" --smoke
            --output "${CMAKE_CURRENT_BINARY_DIR}/terminal_state_boundary_smoke.json")
  set_tests_properties(terminal_state_boundary_study_smoke PROPERTIES
    LABELS "dynamics;reliability;research" TIMEOUT 120)
endif()

if(TARGET case33bw_acdc_propagation_study)
  add_test(NAME case33bw_acdc_propagation_study_smoke
    COMMAND "$<TARGET_FILE:case33bw_acdc_propagation_study>" --smoke
            --output "${CMAKE_CURRENT_BINARY_DIR}/case33bw_acdc_propagation_smoke.json"
            --csv "${CMAKE_CURRENT_BINARY_DIR}/case33bw_acdc_propagation_smoke.csv")
  set_tests_properties(case33bw_acdc_propagation_study_smoke PROPERTIES
    LABELS "dynamics;reliability;research;acdc" TIMEOUT 120)
  find_program(HACDCPF_JULIA_EXECUTABLE julia)
  if(HACDCPF_JULIA_EXECUTABLE)
    add_test(NAME case33bw_acdc_dc_fault_julia_oracle
      COMMAND "${HACDCPF_JULIA_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tools/validate_case33bw_acdc_dc_fault_oracle.jl"
              "$<TARGET_FILE:case33bw_acdc_propagation_study>")
    set_tests_properties(case33bw_acdc_dc_fault_julia_oracle PROPERTIES
      LABELS "dynamics;reliability;research;acdc;cross-validation" TIMEOUT 120)
  endif()
endif()

if(TARGET nk_acdc_reliability_study)
  add_test(NAME nk_acdc_reliability_study_smoke
    COMMAND "$<TARGET_FILE:nk_acdc_reliability_study>" --smoke
            --audit-only
            --output "${CMAKE_CURRENT_BINARY_DIR}/nk_acdc_reliability_smoke.json")
  set_tests_properties(nk_acdc_reliability_study_smoke PROPERTIES
    LABELS "dynamics;reliability;research;acdc;nk" TIMEOUT 180)
  find_program(HACDCPF_NK_PYTHON_EXECUTABLE python3)
  if(HACDCPF_NK_PYTHON_EXECUTABLE)
    add_test(NAME nk_acdc_reliability_oracle
      COMMAND "${HACDCPF_NK_PYTHON_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tools/validate_nk_acdc_reliability.py"
              "$<TARGET_FILE:nk_acdc_reliability_study>")
    set_tests_properties(nk_acdc_reliability_oracle PROPERTIES
      LABELS "dynamics;reliability;research;acdc;nk;cross-validation"
      TIMEOUT 180)
  endif()
endif()

# Unit conversion: actual (ohm/km, kV) → per-unit branch impedance bridge.
hacdcpf_add_test(test_unit_conversion     test_unit_conversion.cpp)

# Typical parameters: opt-in sparse hybrid AC/DC parameter completion.
hacdcpf_add_test(test_typical_parameters  test_typical_parameters.cpp)

# Internal LVNT-inspired urban primary/secondary performance benchmark.
hacdcpf_add_test(test_urban_lvn_primary_secondary
                 test_urban_lvn_primary_secondary.cpp)

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

# Dedicated regression coverage for the non-default power-flow algorithms and
# the unified solver/workspace contracts.
hacdcpf_add_test(test_helm_solver         test_helm_solver.cpp)
hacdcpf_add_test(test_homotopy_continuation test_homotopy_continuation.cpp)
hacdcpf_add_test(test_voltage_stability   test_voltage_stability.cpp)
hacdcpf_add_test(test_newton_krylov       test_newton_krylov.cpp)
hacdcpf_add_test(test_power_flow_math_audit test_power_flow_math_audit.cpp)
hacdcpf_add_test(test_component_models_math_audit test_component_models_math_audit.cpp)

# DC island voltage-reference planning: a PQ VSC converter feeding a
# reference-less DC island is auto-promoted to regulate Vdc (regulate + warn).
hacdcpf_add_test(test_dc_island_reference test_dc_island_reference.cpp)

# SPPT executable theory: metamorphic relations (MR1-MR7), MR3 certificate
# corpus, admissibility guard (Alg. 2 + scripted agent-edit metrics), and DAE
# index-1 well-posedness (MR8).  Operationalizes docs/latex/sppt_theory.tex.
hacdcpf_add_test(test_sppt_metamorphic     test_sppt_metamorphic.cpp)
hacdcpf_add_test(test_sppt_certificate     test_sppt_certificate.cpp)
hacdcpf_add_test(test_sppt_guard           test_sppt_guard.cpp)
hacdcpf_add_test(test_sppt_dae_wellposedness test_sppt_dae_wellposedness.cpp)
hacdcpf_add_test(test_sppt_ablation        test_sppt_ablation.cpp)
hacdcpf_add_test(test_sppt_benchmark       test_sppt_benchmark.cpp)

find_package(Python3 COMPONENTS Interpreter QUIET)
# Documentation source-anchor drift guard: every file:symbol anchor in the
# manuals / design docs must still resolve against the source tree.  Pure
# Python, no build artifacts required, so it runs on every ctest sweep and in
# CI.  The script resolves the repository root from its own location.
if(Python3_EXECUTABLE AND
   EXISTS "${CMAKE_SOURCE_DIR}/tools/doc_anchor_check.py")
  add_test(NAME doc_anchor_check
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_SOURCE_DIR}/tools/doc_anchor_check.py")
  set_tests_properties(doc_anchor_check PROPERTIES
    LABELS "docs;anchors;drift"
    TIMEOUT 120)
endif()

if(TARGET integrated_energy_xref AND Python3_EXECUTABLE AND
   EXISTS "${CMAKE_SOURCE_DIR}/tools/integrated_energy_validation/run_cross_validation.py")
  add_test(NAME integrated_energy_cross_validation
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_SOURCE_DIR}/tools/integrated_energy_validation/run_cross_validation.py"
            --executable "$<TARGET_FILE:integrated_energy_xref>"
            --out "${CMAKE_CURRENT_BINARY_DIR}/integrated_energy_cross_validation.json"
            --markdown "${CMAKE_CURRENT_BINARY_DIR}/integrated_energy_cross_validation.md")
  set_tests_properties(integrated_energy_cross_validation PROPERTIES
    LABELS "integrated_energy;cross_validation;python_oracle"
    TIMEOUT 120)
endif()

if(TARGET model_projection_xref AND Python3_EXECUTABLE AND
   EXISTS "${CMAKE_SOURCE_DIR}/tools/model_validation/run_projection_oracle.py")
  add_test(NAME model_projection_cross_validation
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_SOURCE_DIR}/tools/model_validation/run_projection_oracle.py"
            --executable "$<TARGET_FILE:model_projection_xref>"
            --out "${CMAKE_CURRENT_BINARY_DIR}/model_projection_cross_validation.json"
            --markdown "${CMAKE_CURRENT_BINARY_DIR}/model_projection_cross_validation.md")
  set_tests_properties(model_projection_cross_validation PROPERTIES
    LABELS "model;projection;cross_validation;python_oracle"
    TIMEOUT 120)
endif()

if(TARGET power_models_xref AND Python3_EXECUTABLE AND
   EXISTS "${CMAKE_SOURCE_DIR}/tools/power_models_validation/run_cross_validation.py")
  add_test(NAME power_models_cross_validation
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_SOURCE_DIR}/tools/power_models_validation/run_cross_validation.py"
            --executable "$<TARGET_FILE:power_models_xref>"
            --out "${CMAKE_CURRENT_BINARY_DIR}/power_models_cross_validation.json"
            --markdown "${CMAKE_CURRENT_BINARY_DIR}/power_models_cross_validation.md")
  set_tests_properties(power_models_cross_validation PROPERTIES
    LABELS "power_models;cross_validation;python_oracle"
    TIMEOUT 120)
endif()

# Carbon-flow analysis independent equation oracle: validate_carbon_xref emits
# the production nodal-intensity solution for deterministic AC cases, and the
# numpy-free Python oracle re-derives and re-solves the Kang carbon-emission-flow
# linear system, checking analytic closed forms, nodal conservation and the
# loss-allocation rule.  See carbon_analysis/chapters/numerical_cross_validation.tex.
if(Python3_EXECUTABLE AND
   EXISTS "${CMAKE_SOURCE_DIR}/tools/carbon_analysis_validation/validate_carbon_xref.cpp" AND
   EXISTS "${CMAKE_SOURCE_DIR}/tools/carbon_analysis_validation/run_cross_validation.py")
  add_executable(carbon_analysis_xref
    "${CMAKE_SOURCE_DIR}/tools/carbon_analysis_validation/validate_carbon_xref.cpp")
  target_link_libraries(carbon_analysis_xref PRIVATE hacdcpf::hacdcpf)
  add_test(NAME carbon_analysis_cross_validation
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_SOURCE_DIR}/tools/carbon_analysis_validation/run_cross_validation.py"
            --executable "$<TARGET_FILE:carbon_analysis_xref>"
            --out "${CMAKE_CURRENT_BINARY_DIR}/carbon_analysis_cross_validation.json"
            --markdown "${CMAKE_CURRENT_BINARY_DIR}/carbon_analysis_cross_validation.md")
  set_tests_properties(carbon_analysis_cross_validation PROPERTIES
    LABELS "carbon_analysis;cross_validation;python_oracle"
    TIMEOUT 120)
endif()

# Reliability parameter-resolver independent equation oracle: validate_reliability_xref
# emits the resolved canonical failure-mode parameters for every conversion
# branch, and the numpy-free Python oracle re-derives the Billinton & Allan
# alternating-renewal closed forms (lambda/repair/unavailability/MTTF/frequency).
# See reliability/chapters/numerical_cross_validation.tex.
if(Python3_EXECUTABLE AND
   EXISTS "${CMAKE_SOURCE_DIR}/tools/reliability_validation/validate_reliability_xref.cpp" AND
   EXISTS "${CMAKE_SOURCE_DIR}/tools/reliability_validation/run_cross_validation.py")
  add_executable(reliability_resolver_xref
    "${CMAKE_SOURCE_DIR}/tools/reliability_validation/validate_reliability_xref.cpp")
  target_link_libraries(reliability_resolver_xref PRIVATE hacdcpf::hacdcpf)
  add_test(NAME reliability_resolver_cross_validation
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_SOURCE_DIR}/tools/reliability_validation/run_cross_validation.py"
            --executable "$<TARGET_FILE:reliability_resolver_xref>"
            --out "${CMAKE_CURRENT_BINARY_DIR}/reliability_resolver_cross_validation.json"
            --markdown "${CMAKE_CURRENT_BINARY_DIR}/reliability_resolver_cross_validation.md")
  set_tests_properties(reliability_resolver_cross_validation PROPERTIES
    LABELS "reliability;cross_validation;python_oracle"
    TIMEOUT 120)
endif()

# Market SCED/LMP + settlement independent equation oracle: validate_market_xref
# emits the production day-ahead-market clearing for deterministic single-bus
# copper-plate cases, and the numpy-free Python oracle re-derives the merit-order
# LMP and the revenue-adequacy settlement identities.
# See market/chapters/numerical_cross_validation.tex.
if(Python3_EXECUTABLE AND
   EXISTS "${CMAKE_SOURCE_DIR}/tools/market_validation/validate_market_xref.cpp" AND
   EXISTS "${CMAKE_SOURCE_DIR}/tools/market_validation/run_cross_validation.py")
  add_executable(market_sced_xref
    "${CMAKE_SOURCE_DIR}/tools/market_validation/validate_market_xref.cpp")
  target_link_libraries(market_sced_xref PRIVATE hacdcpf::hacdcpf)
  add_test(NAME market_sced_cross_validation
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_SOURCE_DIR}/tools/market_validation/run_cross_validation.py"
            --executable "$<TARGET_FILE:market_sced_xref>"
            --out "${CMAKE_CURRENT_BINARY_DIR}/market_sced_cross_validation.json"
            --markdown "${CMAKE_CURRENT_BINARY_DIR}/market_sced_cross_validation.md")
  set_tests_properties(market_sced_cross_validation PROPERTIES
    LABELS "market;cross_validation;python_oracle"
    TIMEOUT 180)
endif()

# Hosting-capacity independent equation oracle: validate_hosting_xref emits the
# production DL/T 2041-2025 equipment-level assessment for deterministic supply
# areas, and the numpy-free Python oracle re-derives the S_d closed form and the
# accessible-capacity algebra.  See analysis/chapters/numerical_cross_validation.tex.
if(Python3_EXECUTABLE AND
   EXISTS "${CMAKE_SOURCE_DIR}/tools/analysis_validation/validate_hosting_xref.cpp" AND
   EXISTS "${CMAKE_SOURCE_DIR}/tools/analysis_validation/run_cross_validation.py")
  add_executable(hosting_capacity_xref
    "${CMAKE_SOURCE_DIR}/tools/analysis_validation/validate_hosting_xref.cpp")
  target_link_libraries(hosting_capacity_xref PRIVATE hacdcpf::hacdcpf)
  add_test(NAME hosting_capacity_cross_validation
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_SOURCE_DIR}/tools/analysis_validation/run_cross_validation.py"
            --executable "$<TARGET_FILE:hosting_capacity_xref>"
            --out "${CMAKE_CURRENT_BINARY_DIR}/hosting_capacity_cross_validation.json"
            --markdown "${CMAKE_CURRENT_BINARY_DIR}/hosting_capacity_cross_validation.md")
  set_tests_properties(hosting_capacity_cross_validation PROPERTIES
    LABELS "analysis;hosting_capacity;cross_validation;python_oracle"
    TIMEOUT 120)
endif()
hacdcpf_add_test(test_sppt_fault_campaign  test_sppt_fault_campaign.cpp)
hacdcpf_add_test(test_sppt_agent           test_sppt_agent.cpp)

# Multi-converter control-mode feasibility checks based on the coordination
# theory document.
hacdcpf_add_test(test_converter_coordination test_converter_coordination.cpp)
hacdcpf_add_test(test_vsc_limit_ncp test_vsc_limit_ncp.cpp)

# Three-phase power flow: data model, projection-based solve, meshed/multi-source
# topologies, transformer, distribution PF stub, and JSON round-trip.
hacdcpf_add_test(test_three_phase_pf      test_three_phase_pf.cpp)

# Multi-port ER / multi-mode VPP: robustness tests for EnergyRouter ports with
# mixed control modes (VF + PQ + Droop) and VirtualPowerPlants operating in
# generation, demand-response, and reactive-support modes simultaneously.
hacdcpf_add_test(test_multiport_vpp_er    test_multiport_vpp_er.cpp)

# Performance test: MATPOWER cases. MSVC Debug keeps its runtime checks;
# collect meaningful timing numbers from an optimized configuration.
add_executable(test_powerflow_performance test_powerflow_performance.cpp)
target_link_libraries(test_powerflow_performance PRIVATE hacdcpf::hacdcpf Catch2::Catch2WithMain)
target_compile_definitions(test_powerflow_performance PRIVATE
  HACDCPF_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/data"
  HACDCPF_MATPOWER_DATA_DIR="${CMAKE_SOURCE_DIR}/external_data/matpower"
  HACDCPF_PROJECT_ROOT="${CMAKE_SOURCE_DIR}"
)
target_compile_options(test_powerflow_performance PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-O3 -DNDEBUG>
  $<$<AND:$<CXX_COMPILER_ID:MSVC>,$<NOT:$<CONFIG:Debug>>>:/O2 /DNDEBUG>
)
hacdcpf_copy_opendss_runtime(test_powerflow_performance)

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
hacdcpf_add_test(test_carbonflow_dynamic_storage test_carbonflow_dynamic_storage.cpp)
hacdcpf_add_test(test_carbonflow_tracing test_carbonflow_tracing.cpp)
hacdcpf_add_test(test_carbonflow_case_validation test_carbonflow_case_validation.cpp)
hacdcpf_add_test(test_nighttime_opf test_nighttime_opf.cpp)
hacdcpf_add_test(test_uc_solver_threads test_uc_solver_threads.cpp)

# Short-circuit analysis: Z-bus Thévenin cross-validation + IEC 60909 checks.
#   1. 2-bus analytical: Z_Thev = Z_line
#   2. 3-bus radial: Z_Thev(k) = sum of series impedances along path to source
#   3. IEEE 14-bus: positivity, passivity, single-bus API consistency
#   4. Single-bus API round-trip (compute_fault_at_bus matches full batch solve)
#   5. Fault-type ratios: SLG ≈ 3PH for simplified Z0=Z1 model
#   6. IEC c-factor: Sk ∝ c²  (ratio ≈ 1.21 for c=1.0 vs c=1.1)
#   7. IEEE 33-bus BW: monotonic Sk, end-of-feeder attenuation, API round-trip
hacdcpf_add_test(test_short_circuit_crossval test_short_circuit_crossval.cpp)
hacdcpf_add_test(test_dc_short_circuit test_dc_short_circuit.cpp)
# IEC TR 60909-4:2021 §6.2 reference case (33/6 kV + asynchronous motors):
# I"k/ip/Ib/Ik at the 6 kV busbar checked against the published values.
hacdcpf_add_test(test_short_circuit_iec60909_4 test_short_circuit_iec60909_4.cpp)

# Independent OpenDSS fault-study and GridLAB-D balanced-shunt matrix. The
# script owns fixed numerical gates (OpenDSS Ik'' <= 1e-6, ip <= 2%; GridLAB-D
# Ik'' <= 1e-6) and requires all 35 GridLAB-D-representable cases to execute.
set(_HACDCPF_SHORT_CIRCUIT_VALIDATION_PY
    "${CMAKE_SOURCE_DIR}/.venv/bin/python")
if(EXISTS "${_HACDCPF_SHORT_CIRCUIT_VALIDATION_PY}" AND
   EXISTS "${CMAKE_SOURCE_DIR}/tools/short_circuit_validation/run_cross_engine_matrix.py" AND
   TARGET short_circuit_validation_matrix)
  add_test(NAME short_circuit_cross_engine_matrix
    COMMAND "${_HACDCPF_SHORT_CIRCUIT_VALIDATION_PY}"
            "${CMAKE_SOURCE_DIR}/tools/short_circuit_validation/run_cross_engine_matrix.py"
            --native "$<TARGET_FILE:short_circuit_validation_matrix>"
            --require-gridlabd
            --min-gridlabd-cases 35
            --output-dir "${CMAKE_CURRENT_BINARY_DIR}/short_circuit_cross_engine_matrix")
  set_tests_properties(short_circuit_cross_engine_matrix PROPERTIES
    LABELS "short_circuit;validation;opendss;gridlabd"
    TIMEOUT 120
    ENVIRONMENT "PYTHONPYCACHEPREFIX=${HACDCPF_PYTHON_CACHE_DIR}")
endif()
if(EXISTS "${_HACDCPF_SHORT_CIRCUIT_VALIDATION_PY}" AND
   EXISTS "${CMAKE_SOURCE_DIR}/tools/short_circuit_validation/run_ieee_feeder_faultstudy.py" AND
   TARGET short_circuit_thevenin_batch)
  add_test(NAME short_circuit_ieee_feeder_thevenin
    COMMAND "${_HACDCPF_SHORT_CIRCUIT_VALIDATION_PY}"
            "${CMAKE_SOURCE_DIR}/tools/short_circuit_validation/run_ieee_feeder_faultstudy.py"
            --native "$<TARGET_FILE:short_circuit_thevenin_batch>"
            --output-dir "${CMAKE_CURRENT_BINARY_DIR}/short_circuit_ieee_feeder_thevenin")
  set_tests_properties(short_circuit_ieee_feeder_thevenin PROPERTIES
    LABELS "short_circuit;validation;opendss;ieee_feeder"
    TIMEOUT 120
    ENVIRONMENT "PYTHONPYCACHEPREFIX=${HACDCPF_PYTHON_CACHE_DIR}")
endif()

# Harmonic power flow (hybrid AC/DC): frequency-domain harmonic penetration with
# NIC AC/DC coupling, validated against closed-form analytical voltages.
#   1. AC 2-bus analytical V(h) + THD
#   2. impedance frequency scaling (|V(7)|/|V(5)| = 7/5)
#   3. linearity / superposition
#   4. radial feeder monotone harmonic-voltage accumulation
#   5. DC 2-bus ripple analytical
#   6. NIC hybrid AC/DC coupling through one operating point + P-scaling
#   7. multi-harmonic THD (RSS) formula
#   8. default spectra, branch-flow output, summary()
hacdcpf_add_test(test_harmonics_power_flow test_harmonics_power_flow.cpp)

# Three-phase Newton-Raphson: phase-node indexer, balanced/unbalanced radial,
# meshed topology, multi-source PV, HybridPowerSystem overload, VUF, losses,
# zero-load no-load, and 2-bus analytical cross-validation.
hacdcpf_add_test(test_three_phase_nr test_three_phase_nr.cpp)
hacdcpf_add_test(test_three_phase_nr_phase_indexing test_three_phase_nr_phase_indexing.cpp)
hacdcpf_add_test(test_three_phase_nr_load_models test_three_phase_nr_load_models.cpp)
hacdcpf_add_test(test_three_phase_nr_transformer_source test_three_phase_nr_transformer_source.cpp)
hacdcpf_add_test(test_three_phase_nr_regulator test_three_phase_nr_regulator.cpp)
hacdcpf_add_test(test_three_phase_hybrid_pf test_three_phase_hybrid_pf.cpp)

# Transient dynamics: rich/canonical model builder, device-stamped phasor
# network solve, events, and hybrid AC/DC converter coupling.
hacdcpf_add_test(test_transient_dynamics test_transient_dynamics.cpp)

# GridLAB-D bridge: canonical AC snapshot export and optional external
# component/feeder comparison. The test target is always built; external runs
# skip cleanly unless gridlabd is available through HACDCPF_GRIDLABD_BIN,
# GRIDLABD_BIN, PATH, or a sibling gridlab-d build.
hacdcpf_add_test(test_gridlabd_compare test_gridlabd_compare.cpp)
hacdcpf_add_test(test_gridlabd_failure_matrix test_gridlabd_failure_matrix.cpp)

set(_HACDCPF_PHASE_DOMAIN_DATA_DIR
  "${CMAKE_SOURCE_DIR}/tests/data/phase_domain_julia")
if(HACDCPF_HAVE_OPENDSS AND EXISTS "${_HACDCPF_PHASE_DOMAIN_DATA_DIR}")
  hacdcpf_add_plain_test(test_phase_domain_multi_case_compact
    test_phase_domain_multi_case_compact.cpp)
  hacdcpf_add_plain_test(test_phase_domain_multi_case_compact_julia
    test_phase_domain_multi_case_compact_julia.cpp)

		  if(EXISTS "${_HACDCPF_PHASE_DOMAIN_DATA_DIR}/IEEE123_Fixed/IEEE123Master_Fixed.dss")
		    hacdcpf_add_plain_test(test_phase_domain_ieee123 test_phase_domain_ieee123.cpp)
		    hacdcpf_add_plain_test(test_phase_domain_ieee123_generic_solver
		      test_phase_domain_ieee123_generic_solver.cpp)
		  endif()
		endif()
if(HACDCPF_HAVE_OPENDSS AND EXISTS "${CMAKE_SOURCE_DIR}/external_data/opendss_ieee_pes/opendss_reference/13_node/official_full/IEEE13Nodeckt.dss")
  hacdcpf_add_plain_test(test_phase_domain_ieee13 test_phase_domain_ieee13.cpp)
endif()

if(HACDCPF_HAVE_OPENDSS_COMPARE AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test_opendss_pf_compare.cpp")
  hacdcpf_add_test(test_opendss_pf_compare test_opendss_pf_compare.cpp)
  target_include_directories(test_opendss_pf_compare PRIVATE "${CMAKE_SOURCE_DIR}/tools")
endif()

# Distribution resilience assessment: empty/invalid rejection, no-fault zero-shed,
# single-fault shedding, fault repair restoration, tie-switch reconfiguration,
# MESS dispatch, demo data, summary(), progress callback, custom profiles,
# resilience index bounds, fault sequence, legacy field sync, step count, MIP stub.
hacdcpf_add_test(test_resilience_assessment test_resilience_assessment.cpp)
hacdcpf_add_test(test_hosting_capacity test_hosting_capacity.cpp)
hacdcpf_add_test(test_multidimensional_weak_link test_multidimensional_weak_link.cpp)
hacdcpf_add_test(test_counterfactual_planning test_counterfactual_planning.cpp)
hacdcpf_add_test(test_scenario_generation test_scenario_generation.cpp)
hacdcpf_add_test(test_typhoon_traffic_impact test_typhoon_traffic_impact.cpp)
hacdcpf_add_test(test_scenario_bundle_schema test_scenario_bundle_schema.cpp)

if(EXISTS "${CMAKE_SOURCE_DIR}/tools/scenario_generation_review_case.cpp")
  add_executable(scenario_generation_review_case
    "${CMAKE_SOURCE_DIR}/tools/scenario_generation_review_case.cpp")
  target_link_libraries(scenario_generation_review_case PRIVATE hacdcpf::hacdcpf)
  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_Interpreter_FOUND AND
     EXISTS "${CMAKE_SOURCE_DIR}/tests/scenario_generation_cross_validation.py")
    add_test(NAME scenario_generation_cross_validation
      COMMAND "${Python3_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/scenario_generation_cross_validation.py"
              --cpp-bin "$<TARGET_FILE:scenario_generation_review_case>"
              --output-dir
              "${CMAKE_CURRENT_BINARY_DIR}/scenario_generation_validation")
    set_tests_properties(scenario_generation_cross_validation PROPERTIES
      LABELS "scenario_generation;cross_validation;python_oracle"
      TIMEOUT 120 ENVIRONMENT "PYTHONPYCACHEPREFIX=${HACDCPF_PYTHON_CACHE_DIR}")
  endif()
endif()

# ── Graph analysis & reduction ──────────────────────────────────────────────

# Graph topology analysis, switch contraction, series/pendant reduction,
# and result recovery.
hacdcpf_add_test(test_graph test_graph.cpp)

# Kron (Schur complement) reduction: Y-bus partitioning, voltage recovery,
# fill-in guard, extended injection variant.
hacdcpf_add_test(test_graph_kron test_graph_kron.cpp)
hacdcpf_add_test(test_three_phase_hybrid_opf test_three_phase_hybrid_opf.cpp)

# Graph aggregation round-trip: solve PF/OPF before vs after network-level
# aggregation (switch/breaker contraction, series/pendant reduction) and assert
# the physical solution is preserved.  Covers islands, CB/switch types, meshed
# and hybrid AC/DC topologies, and real distribution feeders.
hacdcpf_add_test(test_graph_roundtrip test_graph_roundtrip.cpp)
hacdcpf_add_test(test_result_attribution test_result_attribution.cpp)

# Distribution CB/switch/VSC pipeline: rich model → graph analysis →
# canonical projection → PF → DC OPF → short circuit → ONR → JSON export.
# Traces information propagation through every analysis layer on a 9-bus AC
# + 3-bus DC hybrid distribution network with circuit breakers, tie switches,
# a VSC converter, a DC microgrid, and DER generation.
hacdcpf_add_test(test_distribution_pipeline test_distribution_pipeline.cpp)

# EV power-traffic dynamic simulation: road blockage, unmet travel demand,
# station load synthesis, V2G reverse power, and power-capacity validation.
hacdcpf_add_test(test_ev_power_traffic test_ev_power_traffic.cpp)

# EV power-traffic numerical results report: per-step traffic flow, charging
# load, V2G discharge, queue state, and power balance for four scenarios.
hacdcpf_add_test(test_ev_power_traffic_report test_ev_power_traffic_report.cpp)

# Large-scale EV fleet routing benchmark:
#   IEEE-118 bus grid, 100 fast-charging stations, 10x10 traffic grid,
#   10 000 fleet vehicles, CapacityAwareGreedy vs SystemOptimalLP.
hacdcpf_add_test(test_ev_power_traffic_largescale test_ev_power_traffic_largescale.cpp)

# Joint power-traffic social welfare maximisation:
#   2-bus grid (cheap + peaker generator), 2 routes, 5 EV vehicles.
#   LMP-coupled iterative decomposition: traffic LP + DC-OPF per step.
hacdcpf_add_test(test_ev_power_traffic_joint_opt test_ev_power_traffic_joint_opt.cpp)

# CTM + MSA-DUE: Daganzo cell transmission model with dynamic user equilibrium.
#   4-node 2-route network, symmetric and asymmetric cases, logit assignment,
#   multi-departure-step, and auto CFL time-step selection.
hacdcpf_add_test(test_ev_power_traffic_ctm_due test_ev_power_traffic_ctm_due.cpp)

# Fidelity-bound numerical validation (TSN<->CTM oracle): checks Prop. 2
# exactness (uncongested -> Delta_tau=0) and Lemma 2 congestion prediction
# (loaded -> Delta_tau>0, monotone).  See docs/.../BOUND_SKETCH.md.
hacdcpf_add_test(test_evpt_fidelity_bound test_evpt_fidelity_bound.cpp)

# Free-flow TSN approximation-error study: cumulative-count FIFO corridor CTM,
# exhaustive coupled small case, partial-loading branch-and-bound, and
# demand/capacity sensitivity sweep.
hacdcpf_add_test(test_evpt_approximation_error test_evpt_approximation_error.cpp)
target_include_directories(test_evpt_approximation_error PRIVATE
  "${_HACDCDSS_MIPSOLVERS_DIR}/highs"
  "${CMAKE_BINARY_DIR}/_deps/mipsolvers_build/_deps/embedded_highs")

# Transportation Networks for Research (TNTP) import: network metadata,
# directed links, node coordinates, OD table, and explicit unit conversion.
hacdcpf_add_test(test_tntp_parser test_tntp_parser.cpp)
if(HACDCPF_HAVE_OPENDSS)
  hacdcpf_add_test(test_sioux_falls_ieee123_case test_sioux_falls_ieee123_case.cpp)
endif()

# Formulation C: CTM-DUE jointly coupled with DC-OPF LMP price feedback.
#   2-bus grid, 4-node 2-route traffic, 4 EV vehicles, WTP=50 $/veh.
#   Iterative price coordination: CTM-DUE inner step + DC-OPF per time step.
#   Tests: fixed-price baseline, OPF-coupled convergence, welfare accounting,
#   price-driven route choice (generalized cost), and analysis printout.
hacdcpf_add_test(test_ev_power_traffic_ctm_joint test_ev_power_traffic_ctm_joint.cpp)

# EV power-traffic GUI integration layer: built-in demo scenarios
# (io/evpt_demo_cases) + JSON serialisation (io/evpt_json) exercised the same
# way the /api/session/run_ev_traffic endpoint uses them (modes B, C, D).
hacdcpf_add_test(test_evpt_gui_io test_evpt_gui_io.cpp)

# Formulation D: single LP/QP/MILP/MCP joint optimizer.
#   2-bus grid, 4-node 2-route traffic, 4 EV vehicles, WTP=50 $/veh.
#   Route assignment + station capacity + DC-OPF power balance assembled into
#   one LP/MILP; solver-certified optimal solution (no iterative coordination).
#   Tests: SocialWelfareMax LP, UserBenefitMax LP, OPF coupling, welfare
#   accounting vs Formulation C, and MILP integer-flow mode.
hacdcpf_add_test(test_ev_power_traffic_joint_opt_d test_ev_power_traffic_joint_opt_d.cpp)

# Direct public-entry tests for the Section 20 solver-backed formulations:
# SO-CTM LP, SO-LTM LP, CTM-DUE-VI, infrastructure MILP, LTM-MPC, and
# automatic candidate-route generation wiring.
hacdcpf_add_test(test_ev_power_traffic_formulations_e_f_g_h test_ev_power_traffic_formulations_e_f_g_h.cpp)

# Phase I: multi-class CTM-DUE with heterogeneous ICV and EV vehicles.
# Covers: single-class EV baseline, mixed ICV+EV capacity sharing (PCE),
# spillback propagation, road failure and rerouting, differential class costs.
hacdcpf_add_test(test_ev_power_traffic_phase1 test_ev_power_traffic_phase1.cpp)

# Device terminal flows (switch/CB) + reconfig device back-projection.
hacdcpf_add_test(test_device_flows test_device_flows.cpp)

# ONR options: split-domain radiality + G4/G5 constraint toggles.
hacdcpf_add_test(test_reconfig_options test_reconfig_options.cpp)

# Three-stage MILP fault-recovery reliability bridge (Julia/Gurobi required at
# runtime for TC-2/TC-3/TC-4; set HACDCPF_RUN_JULIA_RELIABILITY=1 to enable).
# TC-1 (symbol linkage + project tree check) runs unconditionally.
# TC-5 (missing-file error handling) runs unconditionally.
hacdcpf_add_test(test_three_stage_reliability test_three_stage_reliability.cpp)

# Unified reliability parameter resolver + hybrid AC/DC customer metrics.
#   Verifies resolve_reliability_params() conversion rules, the DC-customer
#   SAIFI/SAIDI fix (compute_distribution_indices), the data-quality summary,
#   and the opt-in data policy through run_distribution_fmea.
hacdcpf_add_test(test_reliability_resolver test_reliability_resolver.cpp)
hacdcpf_add_test(test_intelligent_cyber_physical_reliability
                 test_intelligent_cyber_physical_reliability.cpp)

# LTM traffic framework: triangular FD sending/receiving, single-link LTM,
# mesoscopic multi-link corridor, EV penetration sweep (station loading),
# individual EV agent trajectory reconstruction (FIFO), and V2G potential.
hacdcpf_add_test(test_ltm_traffic test_ltm_traffic.cpp)

# LTM stress and cross-validation suite (Groups I–XX):
#   Invariants (monotonicity, conservation, jam-density cap), propagation-delay
#   accuracy, spillback/backward-wave, node-model conservation, station
#   population balance, SOC energy balance, FIFO preservation, V2G bounds,
#   large-scale scenarios (200-link chain, 32×32 grid, 500-link corridor),
#   10× overcapacity saturation, mid-run contingency capacity loss, EV reserve
#   edge cases, LTM vs quasi-static discrepancy report, numerical stability
#   at extreme dt, 500 EV agents, tau off-by-one sweep, receiving-function
#   boundary checks, and 100 000+ vehicle accumulation.
#   Tests tagged [slow] target run times of 1–10 s each.
hacdcpf_add_test(test_ltm_stress test_ltm_stress.cpp)

# CTM vs LTM numerical cross-validation (Groups XXI–XXV):
#   Pulse conservation, sustained-capacity comparison, CFL-violating regime,
#   4-link corridor, and free-flow travel-time accuracy.
hacdcpf_add_test(test_ctm_ltm_comparison test_ctm_ltm_comparison.cpp)

add_executable(test_edition_profile
  test_edition_profile.cpp
  "${CMAKE_SOURCE_DIR}/src/server/edition_profile.cpp")
target_include_directories(test_edition_profile PRIVATE
  "${CMAKE_SOURCE_DIR}/src/server")
target_link_libraries(test_edition_profile PRIVATE
  Catch2::Catch2WithMain nlohmann_json::nlohmann_json)
if(HACDCPF_TRIAL_EDITION)
  target_compile_definitions(test_edition_profile PRIVATE HACDCPF_TRIAL_EDITION=1)
endif()
catch_discover_tests(test_edition_profile
  PROPERTIES LABELS "trial;unit")

# ── HySim Web GUI server (single-line-diagram editor + REST backend) ──────────
# Migrated from HybridACDCPowerSystemsPlanning. Header-only httplib is vendored
# under third_party/; nlohmann_json arrives transitively via hacdcpf -> mipsolvers.
# Not a Catch2 test — a standalone executable serving web/ at /xjtu/ (port 8088).
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/run_gui_server.cpp")
  add_executable(run_gui_server
    run_gui_server.cpp
    "${CMAKE_SOURCE_DIR}/src/server/edition_profile.cpp"
    "${CMAKE_SOURCE_DIR}/src/server/runtime_api_v1.cpp")
  target_link_libraries(run_gui_server PRIVATE hacdcpf::hacdcpf)
  target_include_directories(run_gui_server PRIVATE
    "${CMAKE_SOURCE_DIR}/third_party"
    "${CMAKE_SOURCE_DIR}/src/server")
  target_compile_definitions(run_gui_server PRIVATE
    HACDCPF_PROJECT_ROOT="${CMAKE_SOURCE_DIR}")
  if(HACDCPF_TRIAL_EDITION)
    target_compile_definitions(run_gui_server PRIVATE HACDCPF_TRIAL_EDITION=1)
  endif()
  # httplib needs Winsock2 on Windows (freeaddrinfo / WSACleanup / socket).
  if(WIN32)
    target_link_libraries(run_gui_server PRIVATE ws2_32)
  endif()
  add_custom_command(TARGET run_gui_server POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:run_gui_server>"
            "${CMAKE_BINARY_DIR}/$<TARGET_FILE_NAME:run_gui_server>"
    COMMENT "Copying run_gui_server to ${CMAKE_BINARY_DIR}/$<TARGET_FILE_NAME:run_gui_server>")
  if(HACDCPF_HAVE_OPENDSS AND HACDCPF_DSS_CAPI_RUNTIME_DLLS)
    add_custom_command(TARGET run_gui_server POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              ${HACDCPF_DSS_CAPI_RUNTIME_DLLS}
              "${CMAKE_BINARY_DIR}"
      COMMENT "Copying DSS C-API runtime DLLs for top-level run_gui_server"
      VERBATIM)
  endif()

  # End-to-end GUI/HTTP smoke test: starts the server and drives the REST API
  # (load case -> export ETAP .xlsx -> re-import -> import ETAP XML -> PF -> SC).
  # Requires ETAP I/O (binary export/import) and a Python 3 interpreter.
  if(HACDCPF_ENABLE_ETAP)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
      add_test(NAME gui_api_e2e
        COMMAND "${Python3_EXECUTABLE}"
                "${CMAKE_SOURCE_DIR}/tools/gui_api_e2e.py"
                --server "$<TARGET_FILE:run_gui_server>"
                --data-dir "${CMAKE_SOURCE_DIR}/data")
      set_tests_properties(gui_api_e2e PROPERTIES LABELS "gui;e2e" TIMEOUT 120)
    endif()
  endif()

  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_Interpreter_FOUND)
    add_test(NAME session_integrity_e2e
      COMMAND "${Python3_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tools/session_integrity_e2e.py"
              --server "$<TARGET_FILE:run_gui_server>")
    set_tests_properties(session_integrity_e2e PROPERTIES
      LABELS "gui;e2e;concurrency" TIMEOUT 180 SKIP_RETURN_CODE 77)
    add_test(NAME market_bid_empirical_unit
      COMMAND "${Python3_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/test_market_bid_empirical.py")
    set_tests_properties(market_bid_empirical_unit PROPERTIES
      LABELS "market;empirical" TIMEOUT 30)
    add_test(NAME market_independent_audit_unit
      COMMAND "${Python3_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/test_market_independent_audit.py")
    set_tests_properties(market_independent_audit_unit PROPERTIES
      LABELS "market;empirical" TIMEOUT 30)
    add_test(NAME runtime_api_v1_e2e
      COMMAND "${Python3_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tools/runtime_api_v1_e2e.py"
              --server "$<TARGET_FILE:run_gui_server>"
              --data-dir "${CMAKE_SOURCE_DIR}/data")
    set_tests_properties(runtime_api_v1_e2e PROPERTIES
      LABELS "api;v1;e2e" TIMEOUT 120)
    add_test(NAME gui_topology_window_e2e
      COMMAND "${Python3_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tools/gui_topology_window_e2e.py"
              --server "$<TARGET_FILE:run_gui_server>"
              --data-dir "${CMAKE_SOURCE_DIR}/data")
    set_tests_properties(gui_topology_window_e2e PROPERTIES
      LABELS "gui;api;e2e" TIMEOUT 120)
    add_test(NAME gui_result_window_e2e
      COMMAND "${Python3_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tools/gui_result_window_e2e.py"
              --server "$<TARGET_FILE:run_gui_server>"
              --data-dir "${CMAKE_SOURCE_DIR}/data")
    set_tests_properties(gui_result_window_e2e PROPERTIES
      LABELS "gui;api;e2e" TIMEOUT 120)
    if(HACDCPF_TRIAL_EDITION)
      add_test(NAME trial_edition_api_e2e
        COMMAND "${Python3_EXECUTABLE}"
                "${CMAKE_SOURCE_DIR}/tools/trial_edition_e2e.py"
                --server "$<TARGET_FILE:run_gui_server>"
                --data-dir "${CMAKE_SOURCE_DIR}/data")
      set_tests_properties(trial_edition_api_e2e PROPERTIES
        LABELS "trial;api;e2e" TIMEOUT 180)
    endif()
  endif()

  # Browser end-to-end smoke test for the large-system / scalability GUI features
  # (WebGL full-network mode, virtualized editable topology tables, global
  # element search, neighborhood sub-diagram, minimap, and the time-series
  # 时序潮流/时序生产模拟 split). Standalone Node + Playwright driver; opt-in via
  # the "browser" label since it needs `npm i -D playwright && npx playwright
  # install chromium`. Skipped automatically when Node.js is absent.
  find_program(HACDCPF_NODE_EXECUTABLE NAMES node)
  if(HACDCPF_NODE_EXECUTABLE)
    add_test(NAME reliability_control_state_test
      COMMAND "${HACDCPF_NODE_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/e2e/reliability_control_state_test.mjs")
    set_tests_properties(reliability_control_state_test PROPERTIES
      LABELS "gui;unit;reliability")

    add_test(NAME multidimensional_weak_link_gui_test
      COMMAND "${HACDCPF_NODE_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/e2e/multidimensional_weak_link_gui_test.mjs")
    set_tests_properties(multidimensional_weak_link_gui_test PROPERTIES
      LABELS "gui;unit;planning;weak_link")

    add_test(NAME top_toolbar_semantics_test
      COMMAND "${HACDCPF_NODE_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/e2e/top_toolbar_semantics_test.mjs")
    set_tests_properties(top_toolbar_semantics_test PROPERTIES
      LABELS "gui;unit;layout;semantics")

    add_test(NAME pf_ncp_schur_gui_e2e
      COMMAND "${HACDCPF_NODE_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/e2e/pf_ncp_schur_gui_e2e.mjs"
              --server "$<TARGET_FILE:run_gui_server>")
    set_tests_properties(pf_ncp_schur_gui_e2e PROPERTIES
      LABELS "gui;e2e;browser;power_flow;ncp;schur" TIMEOUT 120)

    add_test(NAME workspace_layout_e2e
      COMMAND "${HACDCPF_NODE_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/e2e/workspace_layout_e2e.mjs"
              --server "$<TARGET_FILE:run_gui_server>"
              --data-dir "${CMAKE_SOURCE_DIR}/data")
    set_tests_properties(workspace_layout_e2e PROPERTIES
      LABELS "gui;e2e;browser;layout" TIMEOUT 120)

    add_test(NAME annual_ui_reconcile_e2e
      COMMAND "${HACDCPF_NODE_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/e2e/annual_ui_reconcile_e2e.mjs"
              --server "$<TARGET_FILE:run_gui_server>")
    set_tests_properties(annual_ui_reconcile_e2e PROPERTIES LABELS "gui;e2e;browser;time_series" TIMEOUT 180)

    add_test(NAME gui_scale_features_e2e
      COMMAND "${HACDCPF_NODE_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/e2e/gui_scale_features_e2e.mjs"
              --server "$<TARGET_FILE:run_gui_server>"
              --data-dir "${CMAKE_SOURCE_DIR}/data")
    set_tests_properties(gui_scale_features_e2e PROPERTIES LABELS "gui;e2e;browser" TIMEOUT 300)

    add_test(NAME topology_transformer_link_e2e
      COMMAND "${HACDCPF_NODE_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/e2e/topology_transformer_link_e2e.mjs"
              --server "$<TARGET_FILE:run_gui_server>")
    set_tests_properties(topology_transformer_link_e2e PROPERTIES
      LABELS "gui;e2e;browser;topology;transformer" TIMEOUT 120)

    add_test(NAME gui_lcc_dat_roundtrip_e2e
      COMMAND "${HACDCPF_NODE_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/e2e/gui_lcc_dat_roundtrip_e2e.mjs"
              --server "$<TARGET_FILE:run_gui_server>"
              --data-dir "${CMAKE_SOURCE_DIR}/data")
    set_tests_properties(gui_lcc_dat_roundtrip_e2e PROPERTIES
      LABELS "gui;e2e;browser;bpa;lcc" TIMEOUT 120)

    add_test(NAME network_overview_e2e
      COMMAND "${HACDCPF_NODE_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/e2e/network_overview_e2e.mjs"
              --server "$<TARGET_FILE:run_gui_server>"
              --data-dir "${CMAKE_SOURCE_DIR}/data")
    set_tests_properties(network_overview_e2e PROPERTIES
      LABELS "gui;e2e;browser;webgl;scale" TIMEOUT 120)

    add_test(NAME local_bus_diagram_test
      COMMAND "${HACDCPF_NODE_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/e2e/local_bus_diagram_test.mjs")
    set_tests_properties(local_bus_diagram_test PROPERTIES LABELS "gui;unit;layout;scale" TIMEOUT 30)
    add_test(NAME local_bus_diagram_e2e
      COMMAND "${HACDCPF_NODE_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/e2e/local_bus_diagram_e2e.mjs"
              --server "$<TARGET_FILE:run_gui_server>" --data-dir "${CMAKE_SOURCE_DIR}/data"
              --output-dir "${CMAKE_BINARY_DIR}/local-bus-diagram-e2e")
    set_tests_properties(local_bus_diagram_e2e PROPERTIES LABELS "gui;e2e;browser;layout;scale" TIMEOUT 180)

    add_test(NAME parameter_contract_e2e
      COMMAND "${HACDCPF_NODE_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/e2e/parameter_contract_e2e.mjs"
              --server "$<TARGET_FILE:run_gui_server>")
    set_tests_properties(parameter_contract_e2e PROPERTIES
      LABELS "gui;e2e;browser;parameter_contract" TIMEOUT 120)

    add_test(NAME reliability_configuration_e2e
      COMMAND "${HACDCPF_NODE_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/e2e/reliability_configuration_e2e.mjs"
              --server "$<TARGET_FILE:run_gui_server>")
    set_tests_properties(reliability_configuration_e2e PROPERTIES
      LABELS "gui;e2e;browser;reliability;parameter_contract" TIMEOUT 180)

    add_test(NAME reliability_default_policy_e2e
      COMMAND "${HACDCPF_NODE_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/e2e/reliability_default_policy_e2e.mjs"
              --server "$<TARGET_FILE:run_gui_server>")
    set_tests_properties(reliability_default_policy_e2e PROPERTIES
      LABELS "gui;e2e;browser;reliability;three_stage" TIMEOUT 180)

    add_test(NAME reliability_workflow_e2e
      COMMAND "${HACDCPF_NODE_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/e2e/reliability_workflow_e2e.mjs"
              --server "$<TARGET_FILE:run_gui_server>")
    set_tests_properties(reliability_workflow_e2e PROPERTIES
      LABELS "gui;e2e;browser;reliability;workflow" TIMEOUT 180)

    add_test(NAME reliability_dimension_validation_e2e
      COMMAND "${HACDCPF_NODE_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/e2e/reliability_dimension_validation_e2e.mjs"
              --server "$<TARGET_FILE:run_gui_server>")
    set_tests_properties(reliability_dimension_validation_e2e PROPERTIES
      LABELS "gui;e2e;reliability;physical;information;intelligent" TIMEOUT 180)

    if(HACDCPF_TRIAL_EDITION)
      add_test(NAME trial_gui_e2e
        COMMAND "${HACDCPF_NODE_EXECUTABLE}"
                "${CMAKE_SOURCE_DIR}/tests/e2e/trial_gui_e2e.mjs"
                --server "$<TARGET_FILE:run_gui_server>")
      set_tests_properties(trial_gui_e2e PROPERTIES
        LABELS "trial;gui;e2e;browser" TIMEOUT 180)
    else()
      add_test(NAME market_gui_e2e
        COMMAND "${HACDCPF_NODE_EXECUTABLE}"
                "${CMAKE_SOURCE_DIR}/tests/e2e/market_gui_e2e.mjs"
                --server "$<TARGET_FILE:run_gui_server>")
      set_tests_properties(market_gui_e2e PROPERTIES
        LABELS "gui;e2e;browser;market" TIMEOUT 180)
      add_test(NAME market_workflow_e2e
        COMMAND "${HACDCPF_NODE_EXECUTABLE}"
                "${CMAKE_SOURCE_DIR}/tests/e2e/market_workflow_e2e.mjs"
                --server "$<TARGET_FILE:run_gui_server>")
      set_tests_properties(market_workflow_e2e PROPERTIES
        LABELS "gui;e2e;browser;market" TIMEOUT 240)
      add_test(NAME market_activity_e2e
        COMMAND "${HACDCPF_NODE_EXECUTABLE}"
                "${CMAKE_SOURCE_DIR}/tests/e2e/market_activity_e2e.mjs"
                --server "$<TARGET_FILE:run_gui_server>")
      set_tests_properties(market_activity_e2e PROPERTIES
        LABELS "gui;e2e;browser;market" TIMEOUT 180)
      add_test(NAME southern_market_e2e
        COMMAND "${HACDCPF_NODE_EXECUTABLE}"
                "${CMAKE_SOURCE_DIR}/tests/e2e/southern_market_e2e.mjs"
                --server "$<TARGET_FILE:run_gui_server>")
      set_tests_properties(southern_market_e2e PROPERTIES
        LABELS "gui;e2e;browser;market" TIMEOUT 180)
      add_test(NAME yunnan_ancillary_e2e
        COMMAND "${HACDCPF_NODE_EXECUTABLE}"
                "${CMAKE_SOURCE_DIR}/tests/e2e/yunnan_ancillary_e2e.mjs"
                --server "$<TARGET_FILE:run_gui_server>")
      set_tests_properties(yunnan_ancillary_e2e PROPERTIES
        LABELS "gui;e2e;browser;market" TIMEOUT 240)
      add_test(NAME southern_realtime_e2e
        COMMAND "${HACDCPF_NODE_EXECUTABLE}"
                "${CMAKE_SOURCE_DIR}/tests/e2e/southern_realtime_e2e.mjs"
                --server "$<TARGET_FILE:run_gui_server>")
      set_tests_properties(southern_realtime_e2e PROPERTIES
        LABELS "gui;e2e;browser;market" TIMEOUT 300)
      add_test(NAME market_operation_e2e
        COMMAND "${HACDCPF_NODE_EXECUTABLE}"
                "${CMAKE_SOURCE_DIR}/tests/e2e/market_operation_e2e.mjs"
                --server "$<TARGET_FILE:run_gui_server>")
      set_tests_properties(market_operation_e2e PROPERTIES
        LABELS "gui;e2e;browser;market" TIMEOUT 300)
      add_test(NAME market_forecast_e2e
        COMMAND "${HACDCPF_NODE_EXECUTABLE}"
                "${CMAKE_SOURCE_DIR}/tests/e2e/market_forecast_e2e.mjs"
                --server "$<TARGET_FILE:run_gui_server>")
      set_tests_properties(market_forecast_e2e PROPERTIES
        LABELS "gui;e2e;browser;market" TIMEOUT 300)
      add_test(NAME market_ieee118_e2e
        COMMAND "${HACDCPF_NODE_EXECUTABLE}"
                "${CMAKE_SOURCE_DIR}/tests/e2e/market_ieee118_e2e.mjs"
                --server "$<TARGET_FILE:run_gui_server>")
      set_tests_properties(market_ieee118_e2e PROPERTIES
        LABELS "gui;e2e;browser;market;system" TIMEOUT 1800 RUN_SERIAL TRUE)
      add_test(NAME market_ieee118_row_presolve_e2e
        COMMAND "${HACDCPF_NODE_EXECUTABLE}"
                "${CMAKE_SOURCE_DIR}/tests/e2e/market_ieee118_e2e.mjs"
                --server "$<TARGET_FILE:run_gui_server>" --row-presolve)
      set_tests_properties(market_ieee118_row_presolve_e2e PROPERTIES
        LABELS "gui;e2e;browser;market;system" TIMEOUT 1800 RUN_SERIAL TRUE)
      add_test(NAME market_ieee118_native_e2e
        COMMAND "${HACDCPF_NODE_EXECUTABLE}"
                "${CMAKE_SOURCE_DIR}/tests/e2e/market_ieee118_e2e.mjs"
                --server "$<TARGET_FILE:run_gui_server>" --solver native)
      set_tests_properties(market_ieee118_native_e2e PROPERTIES
        LABELS "gui;e2e;browser;market;system;native" TIMEOUT 1800 RUN_SERIAL TRUE)
      add_test(NAME market_ieee118_mixed_e2e
        COMMAND "${HACDCPF_NODE_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/e2e/market_ieee118_e2e.mjs"
                --server "$<TARGET_FILE:run_gui_server>" --mixed)
      set_tests_properties(market_ieee118_mixed_e2e PROPERTIES
        LABELS "gui;e2e;browser;market;system" TIMEOUT 1800 RUN_SERIAL TRUE)
      add_test(NAME market_canvas_layout_e2e
        COMMAND "${HACDCPF_NODE_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/e2e/market_canvas_layout_e2e.mjs"
                --server "$<TARGET_FILE:run_gui_server>")
      set_tests_properties(market_canvas_layout_e2e PROPERTIES
        LABELS "gui;e2e;browser;market" TIMEOUT 180 RUN_SERIAL TRUE)
      add_test(NAME market_ieee118_resources_e2e
        COMMAND "${HACDCPF_NODE_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/e2e/market_ieee118_resources_e2e.mjs"
                --server "$<TARGET_FILE:run_gui_server>")
      set_tests_properties(market_ieee118_resources_e2e PROPERTIES
        LABELS "gui;e2e;market;system" TIMEOUT 3600 RUN_SERIAL TRUE)
      add_test(NAME market_study_e2e
        COMMAND "${HACDCPF_NODE_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/e2e/market_study_e2e.mjs"
                --server "$<TARGET_FILE:run_gui_server>")
      set_tests_properties(market_study_e2e PROPERTIES
        LABELS "gui;e2e;browser;market;system" TIMEOUT 1800 RUN_SERIAL TRUE)
      add_test(NAME market_ieee118_generic_e2e
        COMMAND "${HACDCPF_NODE_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/e2e/market_gui_e2e.mjs"
                --server "$<TARGET_FILE:run_gui_server>" --builtin market_ieee118)
      set_tests_properties(market_ieee118_generic_e2e PROPERTIES
        LABELS "gui;e2e;browser;market;system" TIMEOUT 1800 RUN_SERIAL TRUE)
    endif()

    add_test(NAME rpo_gui_e2e
      COMMAND "${HACDCPF_NODE_EXECUTABLE}"
              "${CMAKE_SOURCE_DIR}/tests/e2e/rpo_gui_e2e.mjs"
              --server "$<TARGET_FILE:run_gui_server>")
    set_tests_properties(rpo_gui_e2e PROPERTIES
      LABELS "gui;e2e;browser;rpo;cross_validation" TIMEOUT 180)
  endif()
endif()

# Time-series cross-validation: the C++ driver emits HySim and mandatory local
# GridLAB-D observations; the Python oracle independently rebuilds the frozen
# four-period SCUC with SciPy/HiGHS, proves it by commitment enumeration, and
# solves six matching network snapshots with OpenDSS. See
# docs/modules/time_series/chapters/numerical_validation.tex.
if(EXISTS "${CMAKE_SOURCE_DIR}/tools/time_series_validation/validate_time_series_xref.cpp")
  add_executable(validate_time_series_xref
    "${CMAKE_SOURCE_DIR}/tools/time_series_validation/validate_time_series_xref.cpp")
  target_link_libraries(validate_time_series_xref PRIVATE hacdcpf::hacdcpf)
  set(_HACDCPF_TIME_SERIES_VALIDATION_PY
      "${CMAKE_SOURCE_DIR}/.venv/bin/python")
  if(EXISTS "${_HACDCPF_TIME_SERIES_VALIDATION_PY}" AND
     EXISTS "${CMAKE_SOURCE_DIR}/tools/time_series_validation/run_cross_engine_matrix.py")
    add_test(NAME time_series_cross_engine_matrix
      COMMAND "${_HACDCPF_TIME_SERIES_VALIDATION_PY}"
              "${CMAKE_SOURCE_DIR}/tools/time_series_validation/run_cross_engine_matrix.py"
              --cpp-bin "$<TARGET_FILE:validate_time_series_xref>"
              --out "${CMAKE_CURRENT_BINARY_DIR}/time_series_cross_engine_matrix.json"
              --markdown "${CMAKE_CURRENT_BINARY_DIR}/time_series_cross_engine_matrix.md")
    set_tests_properties(time_series_cross_engine_matrix PROPERTIES
      LABELS "time_series;validation;scipy;highs;opendss;gridlabd"
      TIMEOUT 300 ENVIRONMENT "PYTHONPYCACHEPREFIX=${HACDCPF_PYTHON_CACHE_DIR}")
  endif()
endif()

# Harmonic power-flow external cross-validation driver.  Emits the C++ solver's
# result for tools/harmonics_validation/case.json as JSON, which the independent
# numpy reference / OpenDSS cross-check (tools/harmonics_validation/compare.py)
# compares against.  nlohmann_json arrives transitively via hacdcpf.
if(EXISTS "${CMAKE_SOURCE_DIR}/tools/harmonics_validation/validate_harmonics_xref.cpp")
  add_executable(validate_harmonics_xref
    "${CMAKE_SOURCE_DIR}/tools/harmonics_validation/validate_harmonics_xref.cpp")
  target_link_libraries(validate_harmonics_xref PRIVATE hacdcpf::hacdcpf)

if(EXISTS "${CMAKE_SOURCE_DIR}/tools/harmonics_validation/validate_harmonics_ieee13.cpp")
  add_executable(validate_harmonics_ieee13
    "${CMAKE_SOURCE_DIR}/tools/harmonics_validation/validate_harmonics_ieee13.cpp")
  target_link_libraries(validate_harmonics_ieee13 PRIVATE hacdcpf::hacdcpf)
endif()

  set(_HACDCPF_HARMONICS_VALIDATION_PY
      "${CMAKE_SOURCE_DIR}/tools/harmonics_validation/.venv/bin/python")
  set(_HACDCPF_IEEE13_DSS
      "${CMAKE_SOURCE_DIR}/external_data/opendss_ieee_pes/opendss_reference/13_node/official_full/IEEE13Nodeckt.dss")
  if(TARGET validate_harmonics_ieee13 AND
     HACDCPF_HAVE_OPENDSS AND
     EXISTS "${_HACDCPF_HARMONICS_VALIDATION_PY}" AND
     EXISTS "${CMAKE_SOURCE_DIR}/tools/harmonics_validation/compare_ieee13_opendss.py" AND
     EXISTS "${_HACDCPF_IEEE13_DSS}")
    add_test(NAME harmonics_ieee13_opendss
      COMMAND "${_HACDCPF_HARMONICS_VALIDATION_PY}"
              "${CMAKE_SOURCE_DIR}/tools/harmonics_validation/compare_ieee13_opendss.py"
              --cpp-bin "$<TARGET_FILE:validate_harmonics_ieee13>"
              --dss "${_HACDCPF_IEEE13_DSS}"
              --out "${CMAKE_CURRENT_BINARY_DIR}/harmonics_ieee13_opendss.json"
              --tolerance 2e-3)
    set_tests_properties(harmonics_ieee13_opendss PROPERTIES
      LABELS "harmonics;validation;opendss;ieee13" TIMEOUT 180
      ENVIRONMENT "PYTHONPYCACHEPREFIX=${HACDCPF_PYTHON_CACHE_DIR}")
  endif()
  if(EXISTS "${_HACDCPF_HARMONICS_VALIDATION_PY}" AND
     EXISTS "${CMAKE_SOURCE_DIR}/tools/harmonics_validation/compare_device_opendss.py")
    add_custom_target(validate_harmonics_device_opendss
      COMMAND "${_HACDCPF_HARMONICS_VALIDATION_PY}"
              "${CMAKE_SOURCE_DIR}/tools/harmonics_validation/compare_device_opendss.py"
              --cpp-bin "$<TARGET_FILE:validate_harmonics_xref>"
      DEPENDS validate_harmonics_xref
      WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
      COMMENT "Running harmonic AC/DC device validation against OpenDSS")
  endif()
  if(EXISTS "${_HACDCPF_HARMONICS_VALIDATION_PY}" AND
     EXISTS "${CMAKE_SOURCE_DIR}/tools/harmonics_validation/run_cross_engine_matrix.py")
    add_custom_target(validate_harmonics_cross_engine_matrix
      COMMAND "${_HACDCPF_HARMONICS_VALIDATION_PY}"
              "${CMAKE_SOURCE_DIR}/tools/harmonics_validation/run_cross_engine_matrix.py"
              --cpp-bin "$<TARGET_FILE:validate_harmonics_xref>"
              --require-gridlabd --min-gridlabd-slices 20
      DEPENDS validate_harmonics_xref
      WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
      COMMENT "Running AC/DC harmonic cross-engine benchmark matrix")
    add_test(NAME harmonics_cross_engine_matrix
      COMMAND "${_HACDCPF_HARMONICS_VALIDATION_PY}"
              "${CMAKE_SOURCE_DIR}/tools/harmonics_validation/run_cross_engine_matrix.py"
              --cpp-bin "$<TARGET_FILE:validate_harmonics_xref>"
              --out "${CMAKE_CURRENT_BINARY_DIR}/harmonics_cross_engine_matrix.json"
              --require-gridlabd --min-gridlabd-slices 20)
    set_tests_properties(harmonics_cross_engine_matrix PROPERTIES
      LABELS "harmonics;validation;opendss;gridlabd" TIMEOUT 300
      ENVIRONMENT "PYTHONPYCACHEPREFIX=${HACDCPF_PYTHON_CACHE_DIR}")
  endif()
endif()
