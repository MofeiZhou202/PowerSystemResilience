cmake_minimum_required(VERSION 3.20)

project(HySim-XJTU-HRPES VERSION 0.1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/utf-8 /bigobj /FS)
endif()

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

if(MINGW)
  set(_HACDCPF_ENABLE_IPO_DEFAULT OFF)
else()
  set(_HACDCPF_ENABLE_IPO_DEFAULT ON)
endif()
option(HACDCPF_ENABLE_IPO
       "Enable interprocedural optimization when supported"
       ${_HACDCPF_ENABLE_IPO_DEFAULT})
unset(_HACDCPF_ENABLE_IPO_DEFAULT)
if(HACDCPF_ENABLE_IPO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT _ipo_ok OUTPUT _ipo_out)
  if(_ipo_ok)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON)
  endif()
elseif(MINGW)
  foreach(_hacdcpf_ipo_lang C CXX)
    set(CMAKE_${_hacdcpf_ipo_lang}_COMPILE_OPTIONS_IPO "")
    set(CMAKE_${_hacdcpf_ipo_lang}_LINK_OPTIONS_IPO "")
  endforeach()
  unset(_hacdcpf_ipo_lang)
endif()

# ── Options ───────────────────────────────────────────────────────────────────
option(HACDCPF_BUILD_TESTS    "Build unit/integration tests"               ON)
option(HACDCPF_ENABLE_WERROR  "Treat warnings as errors for core library"  OFF)
option(HACDCPF_ENABLE_NATIVE_ARCH
       "Enable -march=native for host-specific builds"                     OFF)
set(HACDCPF_DEPENDENCY_PROFILE "portable" CACHE STRING
    "Dependency profile: portable, full, or minimal.")
set_property(CACHE HACDCPF_DEPENDENCY_PROFILE PROPERTY STRINGS
  portable full minimal)
if(APPLE)
  set(_HACDCPF_ENABLE_IPOPT_DEFAULT ON)
else()
  set(_HACDCPF_ENABLE_IPOPT_DEFAULT OFF)
endif()
option(HACDCPF_ENABLE_IPOPT
       "Build MIPSolvers embedded Ipopt when supported by that checkout"
       ${_HACDCPF_ENABLE_IPOPT_DEFAULT})
unset(_HACDCPF_ENABLE_IPOPT_DEFAULT)
option(HACDCPF_USE_SUITESPARSE
       "Use SuiteSparse-backed sparse linear solvers when available"        ON)
option(HACDCPF_USE_GUROBI
       "Enable Gurobi detection through MIPSolvers when available"          ON)
option(HACDCPF_USE_PAPILO
       "Enable bundled local PaPILO presolve through MIPSolvers"            ON)
option(HACDCPF_USE_HFACTOR
       "Build MIPSolvers vendored HiGHS HFactor backend"                   ON)
option(HACDCPF_BUILD_MIPSOLVERS_BUNDLED_ARCHIVE
       "Build MIPSolvers single-file bundled static archive"               OFF)
set(HACDCPF_SUITESPARSE_ROOT "" CACHE PATH
    "SuiteSparse installation prefix. Mirrors SUITESPARSE_ROOT when empty.")
if(NOT HACDCPF_SUITESPARSE_ROOT AND DEFINED ENV{SUITESPARSE_ROOT})
  set(HACDCPF_SUITESPARSE_ROOT "$ENV{SUITESPARSE_ROOT}" CACHE PATH
      "SuiteSparse installation prefix. Mirrors SUITESPARSE_ROOT when empty."
      FORCE)
endif()
if(HACDCPF_SUITESPARSE_ROOT)
  set(SUITESPARSE_ROOT "${HACDCPF_SUITESPARSE_ROOT}" CACHE PATH
      "SuiteSparse installation prefix used by this project and MIPSolvers."
      FORCE)
  set(ENV{SUITESPARSE_ROOT} "${HACDCPF_SUITESPARSE_ROOT}")
  list(PREPEND CMAKE_PREFIX_PATH "${HACDCPF_SUITESPARSE_ROOT}")
endif()
# Optional I/O features: OpenXLSX is vendored; OpenDSS still needs DSS C-API.
option(HACDCPF_ENABLE_OPENDSS "Build OpenDSS bridge (requires dss_capi)"   OFF)
option(HACDCPF_ENABLE_OPENDSS_COMPARE
       "Build OpenDSS comparison tools and tests (requires dss_capi)"       OFF)
option(HACDCPF_ENABLE_ETAP    "Build Excel import/export (ETAP and scenario workbooks; requires OpenXLSX)" ON)
set(_hacdcpf_default_dss_capi_root
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/dss_capi/dss_capi")
if(APPLE AND EXISTS
    "${CMAKE_CURRENT_SOURCE_DIR}/tools/harmonics_validation/.venv/lib/python3.9/site-packages/dss_python_backend/include/dss_capi_ctx.h")
  set(_hacdcpf_default_dss_capi_root
      "${CMAKE_CURRENT_SOURCE_DIR}/tools/harmonics_validation/.venv/lib/python3.9/site-packages/dss_python_backend")
endif()
set(HACDCPF_DSS_CAPI_ROOT
    "${_hacdcpf_default_dss_capi_root}"
    CACHE PATH "DSS C-API root used when HACDCPF_ENABLE_OPENDSS=ON")
set(HACDCPF_DSS_CAPI_PLATFORM "win_x64" CACHE STRING
    "Optional DSS C-API platform directory under lib/ for vendored bundles")

# ── Dependencies (pulls in MIPSolvers + transitive: Eigen3, fmt, nlohmann_json,
#                 HiGHS, optional Ipopt/SCIP) ─────────────────────────────────
# Embedded SCIP uses POSIX sigaction() and strtok_r(), which the Windows/UCRT
# CRT does not provide. Use SCIP's own portability switches before adding the
# dependency subdirectory so they propagate into the embedded SCIP target.
# Applied before the dependency add_subdirectory so it propagates to SCIP.
if(WIN32)
  add_compile_definitions(SCIP_NO_SIGACTION)
  add_compile_definitions(SCIP_NO_STRTOK_R)
endif()

if(HACDCPF_ENABLE_IPOPT AND NOT APPLE)
  message(FATAL_ERROR
    "HACDCPF_ENABLE_IPOPT=ON was requested, but the current MIPSolvers "
    "embedded Ipopt CMake path is macOS-only. Keep it OFF on Windows/Linux "
    "or extend MIPSolvers/cmake/BuildIpopt.cmake for that platform.")
endif()

if(HACDCPF_DEPENDENCY_PROFILE STREQUAL "minimal")
  set(_hacdcpf_build_mipsolvers_tests OFF)
  set(_hacdcpf_build_mipsolvers_scuc OFF)
  set(_hacdcpf_build_mipsolvers_scuc_case_builder OFF)
  set(_hacdcpf_build_embedded_scip OFF)
  set(_hacdcpf_build_hfactor OFF)
  set(_hacdcpf_use_papilo OFF)
elseif(HACDCPF_DEPENDENCY_PROFILE STREQUAL "full")
  set(_hacdcpf_build_mipsolvers_tests ${HACDCPF_BUILD_TESTS})
  set(_hacdcpf_build_mipsolvers_scuc ON)
  set(_hacdcpf_build_mipsolvers_scuc_case_builder ON)
  set(_hacdcpf_build_embedded_scip ON)
  set(_hacdcpf_build_hfactor ${HACDCPF_USE_HFACTOR})
  set(_hacdcpf_use_papilo ${HACDCPF_USE_PAPILO})
else()
  set(_hacdcpf_build_mipsolvers_tests OFF)
  set(_hacdcpf_build_mipsolvers_scuc OFF)
  set(_hacdcpf_build_mipsolvers_scuc_case_builder OFF)
  set(_hacdcpf_build_embedded_scip ON)
  set(_hacdcpf_build_hfactor ${HACDCPF_USE_HFACTOR})
  set(_hacdcpf_use_papilo ${HACDCPF_USE_PAPILO})
endif()

set(MIPSOLVERS_BUILD_TESTS ${_hacdcpf_build_mipsolvers_tests} CACHE BOOL
    "Build MIPSolvers tests when used as a dependency" FORCE)
set(MIPSOLVERS_BUILD_SCUC ${_hacdcpf_build_mipsolvers_scuc} CACHE BOOL
    "Build MIPSolvers SCUC CLI when used as a dependency" FORCE)
set(MIPSOLVERS_BUILD_SCUC_CASE_BUILDER
    ${_hacdcpf_build_mipsolvers_scuc_case_builder} CACHE BOOL
    "Build MIPSolvers SCUC case builder when used as a dependency" FORCE)
set(MIPSOLVERS_BUILD_PYTHON OFF CACHE BOOL
    "Build MIPSolvers Python bindings when used as a dependency" FORCE)
set(MIPSOLVERS_BUILD_HFACTOR ${_hacdcpf_build_hfactor} CACHE BOOL
    "Build MIPSolvers HFactor when used as a dependency" FORCE)
set(MIPSOLVERS_BUILD_BUNDLED_ARCHIVE
    ${HACDCPF_BUILD_MIPSOLVERS_BUNDLED_ARCHIVE} CACHE BOOL
    "Build MIPSolvers bundled archive when used as a dependency" FORCE)
set(MIPSOLVERS_BUILD_EMBEDDED_HIGHS ON CACHE BOOL
    "Build embedded MIPSolvers HiGHS source" FORCE)
set(MIPSOLVERS_BUILD_EMBEDDED_SCIP ${_hacdcpf_build_embedded_scip} CACHE BOOL
    "Build embedded MIPSolvers SCIP source" FORCE)
set(MIPSOLVERS_ENABLE_NATIVE_ARCH ${HACDCPF_ENABLE_NATIVE_ARCH} CACHE BOOL
    "Propagate host-specific native-arch builds to MIPSolvers" FORCE)
set(MIPSOLVERS_USE_SUITESPARSE ${HACDCPF_USE_SUITESPARSE} CACHE BOOL
    "Use SuiteSparse in MIPSolvers when available" FORCE)
set(MIPSOLVERS_USE_GUROBI ${HACDCPF_USE_GUROBI} CACHE BOOL
    "Enable Gurobi detection in MIPSolvers when available" FORCE)
set(MIPSOLVERS_USE_PAPILO ${_hacdcpf_use_papilo} CACHE BOOL
    "Enable PaPILO presolve in MIPSolvers when available" FORCE)
# MUMPS must be built from the vendored in-tree sources — a cached
# MIPSOLVERS_FORCE_BUILD_MUMPS=OFF from an earlier configure silently links
# the Homebrew MUMPS 5.6.2 dylib while compiling against 5.7.3 headers
# (ABI mismatch → garbage KKT solves, OPF fails at iter ~3).  The deployment
# environment has no network and no Homebrew, so the from-source build is
# the only supported configuration.
set(MIPSOLVERS_FORCE_BUILD_MUMPS ON CACHE BOOL
    "Build MUMPS from vendored in-tree sources (offline; no Homebrew)" FORCE)

set(MIPSOLVERS_BUILD_LOCAL_IPOPT ${HACDCPF_ENABLE_IPOPT} CACHE BOOL
    "Build embedded Ipopt from MIPSolvers/ipopt when supported" FORCE)

if(WIN32 AND NOT DEFINED CMAKE_MSVC_RUNTIME_LIBRARY)
  set(CMAKE_MSVC_RUNTIME_LIBRARY
      "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL" CACHE STRING
      "Use the dynamic MSVC runtime so vcpkg x64-windows DLL packages link cleanly")
endif()
include(cmake/Dependencies.cmake)
if(MINGW AND NOT HACDCPF_ENABLE_IPO)
  foreach(_hacdcpf_no_ipo_target
          mipsolvers libhighs libscip scip mipsolvers_hfactor)
    if(TARGET ${_hacdcpf_no_ipo_target})
      set_property(TARGET ${_hacdcpf_no_ipo_target} PROPERTY
        INTERPROCEDURAL_OPTIMIZATION OFF)
      set_property(TARGET ${_hacdcpf_no_ipo_target} PROPERTY
        INTERPROCEDURAL_OPTIMIZATION_RELEASE OFF)
      set_property(TARGET ${_hacdcpf_no_ipo_target} PROPERTY
        INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO OFF)
    endif()
  endforeach()
  unset(_hacdcpf_no_ipo_target)
endif()

set(HACDCPF_HAVE_OPENDSS OFF)
set(HACDCPF_DSS_CAPI_RUNTIME_DLLS)
if(HACDCPF_ENABLE_OPENDSS OR HACDCPF_ENABLE_OPENDSS_COMPARE)
  set(_hacdcpf_dss_capi_include "${HACDCPF_DSS_CAPI_ROOT}/include")
  set(_hacdcpf_dss_capi_lib_dir
      "${HACDCPF_DSS_CAPI_ROOT}/lib/${HACDCPF_DSS_CAPI_PLATFORM}")

  if(NOT EXISTS "${_hacdcpf_dss_capi_include}/altdss/capi/dss_ctx.h" AND
     NOT EXISTS "${_hacdcpf_dss_capi_include}/dss_capi_ctx.h")
    message(FATAL_ERROR
      "OpenDSS is enabled, but the vendored DSS C-API header was not found. "
      "Expected ${HACDCPF_DSS_CAPI_ROOT}/include/altdss/capi/dss_ctx.h "
      "or ${HACDCPF_DSS_CAPI_ROOT}/include/dss_capi_ctx.h.")
  endif()

  find_library(_hacdcpf_dss_capi_implib
    NAMES altdss_capi dss_capi
    PATHS
      "${_hacdcpf_dss_capi_lib_dir}"
      "${HACDCPF_DSS_CAPI_ROOT}/lib"
      "${HACDCPF_DSS_CAPI_ROOT}"
    NO_DEFAULT_PATH)
  find_file(_hacdcpf_dss_capi_runtime
    NAMES
      altdss_capi.dll
      dss_capi.dll
      libaltdss_capi.dylib
      libdss_capi.dylib
      libaltdss_capi.so
      libdss_capi.so
    PATHS
      "${_hacdcpf_dss_capi_lib_dir}"
      "${HACDCPF_DSS_CAPI_ROOT}/bin"
      "${HACDCPF_DSS_CAPI_ROOT}/lib"
      "${HACDCPF_DSS_CAPI_ROOT}"
    NO_DEFAULT_PATH)

  if(NOT _hacdcpf_dss_capi_implib OR NOT _hacdcpf_dss_capi_runtime)
    message(FATAL_ERROR
      "OpenDSS is enabled, but the DSS C-API library/runtime was not found "
      "under ${HACDCPF_DSS_CAPI_ROOT}. Expected altdss_capi/dss_capi "
      "(.lib/.dll on Windows, .dylib on macOS, or .so on Linux).")
  endif()

  add_library(dss_capi::dss_capi SHARED IMPORTED GLOBAL)
  set_target_properties(dss_capi::dss_capi PROPERTIES
    IMPORTED_LOCATION "${_hacdcpf_dss_capi_runtime}"
    INTERFACE_INCLUDE_DIRECTORIES "${_hacdcpf_dss_capi_include}")
  if(WIN32)
    set_target_properties(dss_capi::dss_capi PROPERTIES
      IMPORTED_IMPLIB "${_hacdcpf_dss_capi_implib}")
  endif()

  get_filename_component(_hacdcpf_dss_capi_runtime_dir
    "${_hacdcpf_dss_capi_runtime}" DIRECTORY)
  file(GLOB _hacdcpf_dss_capi_all_dlls
    "${_hacdcpf_dss_capi_runtime_dir}/*.dll"
    "${_hacdcpf_dss_capi_runtime_dir}/*.dylib"
    "${_hacdcpf_dss_capi_runtime_dir}/*.so")
  foreach(_hacdcpf_dss_capi_dll IN LISTS _hacdcpf_dss_capi_all_dlls)
    get_filename_component(_hacdcpf_dss_capi_dll_name
      "${_hacdcpf_dss_capi_dll}" NAME)
    if(NOT _hacdcpf_dss_capi_dll_name MATCHES "d\\.dll$")
      list(APPEND HACDCPF_DSS_CAPI_RUNTIME_DLLS
        "${_hacdcpf_dss_capi_dll}")
    endif()
  endforeach()
  list(REMOVE_DUPLICATES HACDCPF_DSS_CAPI_RUNTIME_DLLS)

  set(HACDCPF_HAVE_OPENDSS ON)
  message(STATUS "Using vendored DSS C-API: ${HACDCPF_DSS_CAPI_ROOT}")
endif()

function(hacdcpf_copy_opendss_runtime target)
  if(HACDCPF_HAVE_OPENDSS AND HACDCPF_DSS_CAPI_RUNTIME_DLLS)
    add_custom_command(TARGET ${target} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              ${HACDCPF_DSS_CAPI_RUNTIME_DLLS}
              $<TARGET_FILE_DIR:${target}>
      COMMENT "Copying DSS C-API runtime DLLs for ${target}"
      VERBATIM)
  endif()
endfunction()

set(HACDCPF_HAVE_OPENDSS_COMPARE OFF)
if(HACDCPF_ENABLE_OPENDSS_COMPARE AND HACDCPF_HAVE_OPENDSS)
  set(HACDCPF_HAVE_OPENDSS_COMPARE ON)
endif()

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
  src/model/result_attribution.cpp
  src/model/standard_parameter_library.cpp
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
  src/power_flow/converter_coordination.cpp
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
  src/power_flow/lcc_model.cpp
  src/power_flow/lm_trust_region.cpp
  src/power_flow/newton_krylov.cpp
  src/power_flow/nonlinear_scaling.cpp
  src/power_flow/nonmonotone_linesearch.cpp
  src/power_flow/pf_injection_assembly.cpp
  src/power_flow/residual_evaluator.cpp
  src/power_flow/robust_nonlinear_options.cpp
  src/power_flow/solver_data.cpp
  src/power_flow/solver_interface.cpp
  src/power_flow/voltage_stability.cpp
  src/power_flow/workspace.cpp
  src/power_flow/newton_solver.cpp
  src/power_flow/jacobian_check.cpp
  src/power_flow/jacobian_check.cpp
  src/power_flow/distribution_power_flow.cpp
  # ── dynamics / transient simulation ──────────────────────────────────────
  src/dynamics/DynamicSystem.cpp
  src/dynamics/DynamicModelBuilder.cpp
  src/dynamics/DynamicModelCatalog.cpp
  src/dynamics/DynamicSolver.cpp
  src/dynamics/DynamicFrequency.cpp
  src/dynamics/DynamicDaeDiagnostics.cpp
  src/dynamics/SmallSignal.cpp
  src/dynamics/DynamicResults.cpp
  src/dynamics/devices/BasicDynamicDevices.cpp
  src/dynamics/devices/IEEE1547Protection.cpp
  src/dynamics/integration/ExplicitEuler.cpp
  src/dynamics/integration/RK4.cpp
  src/dynamics/integration/BackwardEuler.cpp
  src/dynamics/integration/Trapezoidal.cpp
  src/dynamics/integration/Rosenbrock.cpp
  src/dynamics/solvers/SparseLinearSolver.cpp
  src/dynamics/solvers/NewtonSolver.cpp
  src/dynamics/solvers/AlgebraicNetworkSolver.cpp
  src/dynamics/solvers/DaeSolver.cpp
  # ── optimal_power_flow ───────────────────────────────────────────────────
  src/optimal_power_flow/ac_opf.cpp
  src/optimal_power_flow/dc_opf.cpp
  src/optimal_power_flow/parity_formulation.cpp
  src/optimal_power_flow/parity_ipm.cpp
  src/optimal_power_flow/three_phase_hybrid_opf.cpp
  src/optimal_power_flow/three_phase_hybrid_adapter.cpp
  src/optimal_power_flow/three_phase_hybrid_relaxation.cpp
  src/optimal_power_flow/reactive_power_opt.cpp
  # ── io (always-available: JSON / MATPOWER / case builders) ───────────────
  src/io/import_report.cpp
  src/io/json_io.cpp
  src/io/roundtrip.cpp
  src/io/cim_io.cpp
  src/io/cim_dist_io.cpp
  src/io/svg_distribution_io.cpp
  src/io/matpower_parser.cpp
  src/io/tntp_parser.cpp
  src/io/sioux_falls_ieee123_case.cpp
  src/io/bpa_io.cpp
  src/io/case_builders.cpp
  src/io/scenario_bundle_io.cpp
  src/io/component_io_mapping.cpp
  src/io/powersimulationsdynamics_io.cpp
  src/io/external_grid_io.cpp
  src/io/gridlabd_bridge.cpp
  src/io/gridlabd_validation.cpp
  src/io/evpt_json.cpp
  src/io/evpt_demo_cases.cpp
  # ── validation ────────────────────────────────────────────────────────────
  src/validation/validate_system.cpp
  # ── api ──────────────────────────────────────────────────────────────────
  src/api/hacdcpf.cpp
  # ── EV power-traffic coupling ───────────────────────────────────────────
  src/ev_power_traffic/ev_power_traffic_simulation.cpp
  src/ev_power_traffic/assignment_lp.cpp
  src/ev_power_traffic/joint_social_welfare.cpp
  src/ev_power_traffic/ctm_propagation.cpp
  src/ev_power_traffic/route_generation.cpp
  src/ev_power_traffic/ltm_propagation.cpp
  src/ev_power_traffic/ctm_joint_welfare.cpp
  src/ev_power_traffic/joint_optimizer.cpp
  src/ev_power_traffic/joint_optimizer_full_nlp.cpp
  src/ev_power_traffic/ctm_so_lp.cpp
  src/ev_power_traffic/ltm_so_lp.cpp
  src/ev_power_traffic/ctm_due_vi.cpp
  src/ev_power_traffic/infra_design_milp.cpp
  src/ev_power_traffic/ltm_mpc.cpp
  # ── network reconfiguration (topology analysis + ONR) ──────────────────
  src/network_reconfiguration/topology_analysis.cpp
  src/network_reconfiguration/topology_reconfiguration.cpp
  # ── graph analysis & reduction ────────────────────────────────────────
  src/graph/power_system_graph.cpp
  src/graph/topology_analysis.cpp
  src/graph/switch_contraction.cpp
  src/graph/reduction_plan.cpp
  src/graph/series_reduction.cpp
  src/graph/kron_reduction.cpp
  src/graph/sparse_kron_reduction.cpp
  src/graph/result_recovery.cpp
  # ── short circuit analysis ───────────────────────────────────────────────
  src/short_circuit/short_circuit.cpp
  src/short_circuit/dc_short_circuit.cpp
  # ── harmonic power flow (hybrid AC/DC) ───────────────────────────────────
  src/harmonics_power_flow/harmonics_power_flow.cpp
  # ── scenario generation ───────────────────────────────────────────────────────
  src/scenario_generation/scenario_generation.cpp
  # ── hosting-capacity assessment (DL/T 2041-2025) ─────────────────────────
  src/analysis/hosting_capacity.cpp
  src/analysis/counterfactual_planning.cpp
  src/analysis/multidimensional_weak_link.cpp

  # ── analysis ────────────────────────────────────────────────────────
  src/scenario_generation/typhoon_resilience.cpp
  src/scenario_generation/typhoon_traffic_impact.cpp
  src/reliability/three_stage_reliability.cpp
  # ── analysis stubs ───────────────────────────────────────────────────
  src/carbon_analysis/carbon_analysis.cpp
  src/carbon_analysis/annual_carbon_analysis.cpp
  src/power_flow/three_phase_nr.cpp
  src/power_flow/three_phase_hybrid.cpp
  src/reliability/reliability_assessment.cpp
  src/reliability/reliability_data.cpp
  src/reliability/failure_mode.cpp
  src/resilience/resilience_assessment.cpp
  src/resilience/resilience_stage_milp.cpp
  src/resilience/resilience_restoration_mip.cpp
  src/resilience/certified_restoration.cpp
  src/time_series/time_series_pf.cpp
  src/time_series/annual_production_sim.cpp
  src/time_series/lifecycle_simulation.cpp
  src/integrated_energy/integrated_energy_optimizer.cpp
  # ── day-ahead electricity market ────────────────────────────────────────
  src/market/participant_behavior.cpp
  src/market/market_simulation.cpp
  # ── power_models stubs (AML model builders) ──────────────────────────
  src/power_models/acopf_builder.cpp
  src/power_models/acdcopf_builder.cpp
  src/power_models/dc_opf_builder.cpp
  src/power_models/lindistflow_builder.cpp
  src/power_models/scuc_builder.cpp
  # ── SPPT executable-theory layer (metamorphic relations, MR3 certificate,
  #    admissibility guard); operationalizes docs/latex/sppt_theory.tex ──────
  src/sppt/metamorphic.cpp
  src/sppt/certificate.cpp
  src/sppt/independent_residual.cpp
  src/sppt/intelligent_simulation.cpp
  src/sppt/guard.cpp
  src/sppt/ablation.cpp
  src/sppt/benchmark.cpp
  src/sppt/agent.cpp)

if(HACDCPF_ENABLE_ETAP)
  list(APPEND HACDCPF_SOURCES src/io/etap_io.cpp)
endif()
if(HACDCPF_HAVE_OPENDSS)
  list(APPEND HACDCPF_SOURCES src/io/opendss_bridge.cpp)
endif()

add_library(hacdcpf STATIC ${HACDCPF_SOURCES})
add_library(hacdcpf::hacdcpf ALIAS hacdcpf)

if(NOT WIN32)
  find_package(Iconv REQUIRED)
  target_link_libraries(hacdcpf PRIVATE Iconv::Iconv)
endif()

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
if(HACDCPF_HAVE_OPENDSS)
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_HAVE_OPENDSS=1)
else()
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_NO_OPENDSS)
endif()

# ── Link against MIPSolvers (brings Eigen3, fmt, nlohmann_json, HiGHS, Ipopt) ─
target_link_libraries(hacdcpf PUBLIC mipsolvers::mipsolvers)

# ── SuiteSparse for the OPF sparse-KKT solver (UMFPACK/KLU) ───────────────────
# Prefer the exact vendored SuiteSparse targets already built by MIPSolvers.
# System discovery remains as a compatibility fallback for older MIPSolvers
# source trees that do not export those targets.
set(HACDCPF_OPF_SUITESPARSE_LIBRARIES "")
if(HACDCPF_USE_SUITESPARSE AND
   TARGET mipsolvers::umfpack_vendored AND
   TARGET mipsolvers::klu_vendored)
  target_link_libraries(hacdcpf PRIVATE
    mipsolvers::umfpack_vendored
    mipsolvers::klu_vendored)
  target_compile_definitions(hacdcpf PRIVATE
    HACDCPF_OPF_HAVE_UMFPACK=1
    HACDCPF_OPF_HAVE_KLU=1)
  message(STATUS
    "hacdcpf: OPF sparse KKT = MIPSolvers vendored UMFPACK/KLU")
else()
set(_HACDCPF_SUITESPARSE_HINTS
  ${HACDCPF_SUITESPARSE_ROOT}
  $ENV{SUITESPARSE_ROOT}
  /opt/homebrew/opt/suite-sparse
  /opt/homebrew
  /usr/local/opt/suite-sparse
  /usr/local
  /usr
  "C:/vcpkg/installed/x64-windows"
  "C:/vcpkg/installed/x64-windows-static"
  "C:/vcpkg/installed/arm64-windows"
  "C:/SuiteSparse")
list(REMOVE_DUPLICATES _HACDCPF_SUITESPARSE_HINTS)
find_path(HACDCPF_SUITESPARSE_INCLUDE_DIR
  NAMES umfpack.h klu.h
  HINTS ${_HACDCPF_SUITESPARSE_HINTS}
  PATH_SUFFIXES include include/suitesparse)
find_library(HACDCPF_UMFPACK_LIBRARY
  NAMES umfpack
  HINTS ${_HACDCPF_SUITESPARSE_HINTS}
  PATH_SUFFIXES lib lib64)
find_library(HACDCPF_KLU_LIBRARY
  NAMES klu
  HINTS ${_HACDCPF_SUITESPARSE_HINTS}
  PATH_SUFFIXES lib lib64)

if(HACDCPF_USE_SUITESPARSE AND HACDCPF_SUITESPARSE_INCLUDE_DIR)
  find_library(HACDCPF_AMD_LIBRARY
    NAMES amd
    HINTS ${_HACDCPF_SUITESPARSE_HINTS}
    PATH_SUFFIXES lib lib64)
  find_library(HACDCPF_COLAMD_LIBRARY
    NAMES colamd
    HINTS ${_HACDCPF_SUITESPARSE_HINTS}
    PATH_SUFFIXES lib lib64)
  find_library(HACDCPF_CHOLMOD_LIBRARY
    NAMES cholmod
    HINTS ${_HACDCPF_SUITESPARSE_HINTS}
    PATH_SUFFIXES lib lib64)
  find_library(HACDCPF_SUITESPARSECONFIG_LIBRARY
    NAMES suitesparseconfig
    HINTS ${_HACDCPF_SUITESPARSE_HINTS}
    PATH_SUFFIXES lib lib64)

  target_include_directories(hacdcpf PRIVATE ${HACDCPF_SUITESPARSE_INCLUDE_DIR})
  foreach(_hacdcpf_ss_lib IN ITEMS
      HACDCPF_AMD_LIBRARY
      HACDCPF_COLAMD_LIBRARY
      HACDCPF_CHOLMOD_LIBRARY
      HACDCPF_SUITESPARSECONFIG_LIBRARY)
    if(NOT "${${_hacdcpf_ss_lib}}" STREQUAL "" AND
       NOT "${${_hacdcpf_ss_lib}}" MATCHES "-NOTFOUND$")
      list(APPEND HACDCPF_OPF_SUITESPARSE_LIBRARIES ${${_hacdcpf_ss_lib}})
    endif()
  endforeach()

  if(HACDCPF_UMFPACK_LIBRARY AND
     EXISTS "${HACDCPF_SUITESPARSE_INCLUDE_DIR}/umfpack.h")
    list(APPEND HACDCPF_OPF_SUITESPARSE_LIBRARIES ${HACDCPF_UMFPACK_LIBRARY})
    target_compile_definitions(hacdcpf PRIVATE HACDCPF_OPF_HAVE_UMFPACK=1)
    message(STATUS
      "hacdcpf: OPF sparse KKT can use UMFPACK "
      "(${HACDCPF_UMFPACK_LIBRARY})")
  endif()

  if(HACDCPF_KLU_LIBRARY AND
     EXISTS "${HACDCPF_SUITESPARSE_INCLUDE_DIR}/klu.h")
    list(APPEND HACDCPF_OPF_SUITESPARSE_LIBRARIES ${HACDCPF_KLU_LIBRARY})
    target_compile_definitions(hacdcpf PRIVATE HACDCPF_OPF_HAVE_KLU=1)
    message(STATUS
      "hacdcpf: OPF sparse KKT can use KLU (${HACDCPF_KLU_LIBRARY})")
  endif()

  if(HACDCPF_OPF_SUITESPARSE_LIBRARIES)
    list(REMOVE_DUPLICATES HACDCPF_OPF_SUITESPARSE_LIBRARIES)
    target_link_libraries(hacdcpf PRIVATE ${HACDCPF_OPF_SUITESPARSE_LIBRARIES})
  else()
    message(STATUS
      "hacdcpf: SuiteSparse headers found, but UMFPACK/KLU libraries were not; "
      "OPF sparse KKT will use Eigen SparseLU unless forced otherwise.")
  endif()
else()
  message(STATUS
    "hacdcpf: SuiteSparse disabled or not found; OPF sparse KKT will use Eigen SparseLU")
endif()
endif()

# ── ETAP Excel I/O backend (OpenXLSX) ────────────────────────────────────
if(HACDCPF_ENABLE_ETAP AND TARGET OpenXLSX::OpenXLSX)
  target_link_libraries(hacdcpf PUBLIC OpenXLSX::OpenXLSX)
endif()
if(HACDCPF_HAVE_OPENDSS)
  target_link_libraries(hacdcpf PUBLIC dss_capi::dss_capi)
endif()
if(HACDCPF_HAVE_OPENDSS_COMPARE)
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_HAVE_OPENDSS_COMPARE=1)

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/opendss_snapshot_adapter.cpp")
    add_executable(opendss_snapshot_adapter tools/opendss_snapshot_adapter.cpp)
    target_link_libraries(opendss_snapshot_adapter PRIVATE hacdcpf::hacdcpf)
    target_include_directories(opendss_snapshot_adapter PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/tools")
    target_compile_definitions(opendss_snapshot_adapter PRIVATE
      HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
    hacdcpf_copy_opendss_runtime(opendss_snapshot_adapter)
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/opendss_pf_compare.cpp")
    add_executable(opendss_pf_compare tools/opendss_pf_compare.cpp)
    target_link_libraries(opendss_pf_compare PRIVATE hacdcpf::hacdcpf)
    target_include_directories(opendss_pf_compare PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/tools")
    target_compile_definitions(opendss_pf_compare PRIVATE
      HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
    hacdcpf_copy_opendss_runtime(opendss_pf_compare)
  endif()
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
  hacdcpf_copy_opendss_runtime(etap_convert)
endif()

# ── AC OPF numerical-performance benchmark (diagnostic CLI) ────────────────────
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/opf_numerical_benchmark.cpp")
  add_executable(opf_numerical_benchmark tools/opf_numerical_benchmark.cpp)
  target_link_libraries(opf_numerical_benchmark PRIVATE hacdcpf::hacdcpf)
  target_compile_options(opf_numerical_benchmark PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-O3 -DNDEBUG>
    $<$<CXX_COMPILER_ID:MSVC>:/O2 /DNDEBUG>)
  hacdcpf_copy_opendss_runtime(opf_numerical_benchmark)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/matpower_pf_compare.cpp")
  add_executable(matpower_pf_compare tools/matpower_pf_compare.cpp)
  target_link_libraries(matpower_pf_compare PRIVATE hacdcpf::hacdcpf)
  hacdcpf_copy_opendss_runtime(matpower_pf_compare)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/bpa_dat_convert.cpp")
  add_executable(bpa_dat_convert tools/bpa_dat_convert.cpp)
  target_link_libraries(bpa_dat_convert PRIVATE hacdcpf::hacdcpf)
  hacdcpf_copy_opendss_runtime(bpa_dat_convert)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/gridlabd_validation_matrix.cpp")
  add_executable(gridlabd_validation_matrix tools/gridlabd_validation_matrix.cpp)
  target_link_libraries(gridlabd_validation_matrix PRIVATE hacdcpf::hacdcpf)
  hacdcpf_copy_opendss_runtime(gridlabd_validation_matrix)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/transient_validation_matrix.cpp")
  add_executable(transient_validation_matrix tools/transient_validation_matrix.cpp)
  target_link_libraries(transient_validation_matrix PRIVATE hacdcpf::hacdcpf)
  set(_HACDCPF_TRANSIENT_VALIDATION_PY
      "${CMAKE_CURRENT_SOURCE_DIR}/tools/harmonics_validation/.venv/bin/python")
  if(TARGET gridlabd_validation_matrix AND
     EXISTS "${_HACDCPF_TRANSIENT_VALIDATION_PY}" AND
     EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/transient_validation/run_cross_engine_matrix.py")
    add_custom_target(validate_transient_cross_engine_matrix
      COMMAND "${_HACDCPF_TRANSIENT_VALIDATION_PY}"
              "${CMAKE_CURRENT_SOURCE_DIR}/tools/transient_validation/run_cross_engine_matrix.py"
              --native-bin "$<TARGET_FILE:transient_validation_matrix>"
              --gridlabd-matrix-bin "$<TARGET_FILE:gridlabd_validation_matrix>"
      DEPENDS transient_validation_matrix gridlabd_validation_matrix
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      COMMENT "Running native/GridLAB-D transient disturbance matrix")
  endif()
endif()

# Paper experiment: deterministic small-case dynamic-security baseline.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/cyber_dynamic_safe_restoration_small.cpp")
  add_executable(cyber_dynamic_safe_restoration_small
    tools/cyber_dynamic_safe_restoration_small.cpp)
  target_link_libraries(cyber_dynamic_safe_restoration_small
    PRIVATE hacdcpf::hacdcpf)
  target_compile_definitions(cyber_dynamic_safe_restoration_small PRIVATE
    HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
  hacdcpf_copy_opendss_runtime(cyber_dynamic_safe_restoration_small)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/cyber_dynamic_safe_restoration_scale.cpp")
  add_executable(cyber_dynamic_safe_restoration_scale
    tools/cyber_dynamic_safe_restoration_scale.cpp)
  target_link_libraries(cyber_dynamic_safe_restoration_scale
    PRIVATE hacdcpf::hacdcpf)
  target_compile_definitions(cyber_dynamic_safe_restoration_scale PRIVATE
    HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
  hacdcpf_copy_opendss_runtime(cyber_dynamic_safe_restoration_scale)
endif()

# Reproducible end-to-end case study for the distribution-resilience review.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/distribution_resilience_review_case.cpp")
  add_executable(distribution_resilience_review_case
    tools/distribution_resilience_review_case.cpp)
  target_link_libraries(distribution_resilience_review_case
    PRIVATE hacdcpf::hacdcpf)
  target_compile_definitions(distribution_resilience_review_case PRIVATE
    HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
  hacdcpf_copy_opendss_runtime(distribution_resilience_review_case)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/short_circuit_validation_matrix.cpp")
  add_executable(short_circuit_validation_matrix
    tools/short_circuit_validation_matrix.cpp)
  target_link_libraries(short_circuit_validation_matrix PRIVATE hacdcpf::hacdcpf)
endif()

# ── SPPT MR3 certificate corpus emitter (LaTeX + CSV) ─────────────────────────
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/sppt_certify.cpp")
  add_executable(sppt_certify tools/sppt_certify.cpp)
  target_link_libraries(sppt_certify PRIVATE hacdcpf::hacdcpf)
  target_compile_definitions(sppt_certify PRIVATE
    HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
  hacdcpf_copy_opendss_runtime(sppt_certify)
endif()

# ── SPPT Pillar-4 ablation study emitter (LaTeX + CSV) ────────────────────────
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/sppt_ablation.cpp")
  add_executable(sppt_ablation tools/sppt_ablation.cpp)
  target_link_libraries(sppt_ablation PRIVATE hacdcpf::hacdcpf)
  target_compile_definitions(sppt_ablation PRIVATE
    HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
  hacdcpf_copy_opendss_runtime(sppt_ablation)
endif()

# ── SPPT Pillar-4 scale/accuracy benchmark emitter (LaTeX + CSV) ──────────────
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/sppt_benchmark.cpp")
  add_executable(sppt_benchmark tools/sppt_benchmark.cpp)
  target_link_libraries(sppt_benchmark PRIVATE hacdcpf::hacdcpf)
  target_compile_definitions(sppt_benchmark PRIVATE
    HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_options(sppt_benchmark PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-O3 -DNDEBUG>
    $<$<CXX_COMPILER_ID:MSVC>:/O2 /DNDEBUG>)
  hacdcpf_copy_opendss_runtime(sppt_benchmark)
endif()

# ── Native hybrid AC/DC PF evidence generator for SPPT paper ─────────────────
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/hybrid_acdc_pf_study.cpp")
  add_executable(hybrid_acdc_pf_study tools/hybrid_acdc_pf_study.cpp)
  target_link_libraries(hybrid_acdc_pf_study PRIVATE hacdcpf::hacdcpf)
  target_compile_definitions(hybrid_acdc_pf_study PRIVATE
    HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_options(hybrid_acdc_pf_study PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-O3 -DNDEBUG>
    $<$<CXX_COMPILER_ID:MSVC>:/O2 /DNDEBUG>)
  hacdcpf_copy_opendss_runtime(hybrid_acdc_pf_study)
endif()

# ── Three-phase hybrid AC/DC distribution PF evidence generator ───────────────
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/three_phase_hybrid_acdc_pf_study.cpp")
  add_executable(three_phase_hybrid_acdc_pf_study
    tools/three_phase_hybrid_acdc_pf_study.cpp)
  target_link_libraries(three_phase_hybrid_acdc_pf_study PRIVATE hacdcpf::hacdcpf)
  target_compile_options(three_phase_hybrid_acdc_pf_study PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-O3 -DNDEBUG>
    $<$<CXX_COMPILER_ID:MSVC>:/O2 /DNDEBUG>)
  hacdcpf_copy_opendss_runtime(three_phase_hybrid_acdc_pf_study)
endif()

add_executable(three_phase_hybrid_pf_external_case
  tools/three_phase_hybrid_pf_external_case.cpp)
target_link_libraries(three_phase_hybrid_pf_external_case PRIVATE hacdcpf::hacdcpf)
hacdcpf_copy_opendss_runtime(three_phase_hybrid_pf_external_case)

if(HACDCPF_HAVE_OPENDSS)
  add_executable(sioux_falls_ieee123_experiment
    tools/sioux_falls_ieee123_experiment.cpp)
  target_link_libraries(sioux_falls_ieee123_experiment PRIVATE hacdcpf::hacdcpf)
  target_include_directories(sioux_falls_ieee123_experiment PRIVATE
    "${MIPSOLVERS_SOURCE_DIR}/highs"
    "${CMAKE_BINARY_DIR}/_deps/mipsolvers_build/_deps/embedded_highs")
  target_compile_definitions(sioux_falls_ieee123_experiment PRIVATE
    HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
  hacdcpf_copy_opendss_runtime(sioux_falls_ieee123_experiment)
  add_test(
    NAME evpt_sioux_route_search_smoke
    COMMAND $<TARGET_FILE:sioux_falls_ieee123_experiment>
      --route-search --search-exhaustive --typhoon-road-impact
      --selected-od-pairs 2 --paths-per-od 3
      --search-scenarios 1 --search-threads 1 --search-max-nodes 100
      --num-steps 12 --road-capacity-scale 0.08 --station-power-kw 2400)
  set_tests_properties(evpt_sioux_route_search_smoke PROPERTIES
    PASS_REGULAR_EXPRESSION "\\\"optimality_proven\\\": true")

  add_test(
    NAME evpt_sioux_route_search_exhaustive_regression
    COMMAND $<TARGET_FILE:sioux_falls_ieee123_experiment>
      --route-search --search-exhaustive --typhoon-road-impact
      --selected-od-pairs 3 --paths-per-od 3 --od-demand-scale 0.20
      --search-scenarios 1 --search-threads 1 --search-max-nodes 200
      --num-steps 12 --road-capacity-scale 0.08 --station-power-kw 2400)
  set_tests_properties(evpt_sioux_route_search_exhaustive_regression PROPERTIES
    PASS_REGULAR_EXPRESSION "\\\"search_matches\\\": true")
endif()

add_executable(weather_resilience_case_study
  tools/weather_resilience_case_study.cpp)
target_link_libraries(weather_resilience_case_study PRIVATE hacdcpf::hacdcpf)
target_compile_definitions(weather_resilience_case_study PRIVATE
  HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")

# Phase-node graph-reduction paper benchmark and reproducibility audit.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/phase_graph_reduction_benchmark.cpp")
  add_executable(phase_graph_reduction_benchmark
    tools/phase_graph_reduction_benchmark.cpp)
  target_link_libraries(phase_graph_reduction_benchmark PRIVATE hacdcpf::hacdcpf)
  target_compile_definitions(phase_graph_reduction_benchmark PRIVATE
    HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_options(phase_graph_reduction_benchmark PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-O3 -DNDEBUG>
    $<$<CXX_COMPILER_ID:MSVC>:/O2 /DNDEBUG>)
  hacdcpf_copy_opendss_runtime(phase_graph_reduction_benchmark)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/phase_hybrid_opf_benchmark.cpp")
  add_executable(phase_hybrid_opf_benchmark
    tools/phase_hybrid_opf_benchmark.cpp)
  target_link_libraries(phase_hybrid_opf_benchmark PRIVATE hacdcpf::hacdcpf)
  target_compile_definitions(phase_hybrid_opf_benchmark PRIVATE
    HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
  target_compile_options(phase_hybrid_opf_benchmark PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-O3 -DNDEBUG>
    $<$<CXX_COMPILER_ID:MSVC>:/O2 /DNDEBUG>)
  hacdcpf_copy_opendss_runtime(phase_hybrid_opf_benchmark)
endif()

# ── SPPT Verified-Intelligent-Modeling agent-loop demo (Alg. 2 / Thm. 8.9) ───
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tools/sppt_agent_demo.cpp")
  add_executable(sppt_agent_demo tools/sppt_agent_demo.cpp)
  target_link_libraries(sppt_agent_demo PRIVATE hacdcpf::hacdcpf)
  target_compile_definitions(sppt_agent_demo PRIVATE
    HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
  hacdcpf_copy_opendss_runtime(sppt_agent_demo)
endif()

# ── Tests ─────────────────────────────────────────────────────────────────────
if(HACDCPF_BUILD_TESTS)
  include(CTest)
  enable_testing()
  add_subdirectory(tests)
endif()
