cmake_minimum_required(VERSION 3.20)

project(HybridACDCDistributionSystemsSimulation VERSION 0.1.0 LANGUAGES CXX C)

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
option(HACDCPF_ENABLE_OPENDSS_COMPARE
       "Build OpenDSS comparison tools and tests (requires dss_capi)"       OFF)
option(HACDCPF_ENABLE_ETAP    "Build ETAP Excel import/export (requires OpenXLSX)" OFF)
set(HACDCPF_DSS_CAPI_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/dss_capi/dss_capi"
    CACHE PATH "Vendored DSS C-API root used when HACDCPF_ENABLE_OPENDSS=ON")
set(HACDCPF_DSS_CAPI_PLATFORM "win_x64" CACHE STRING
    "Vendored DSS C-API platform directory under lib/")

# ── Dependencies (pulls in MIPSolvers + transitive: Eigen3, fmt, nlohmann_json,
#                 HiGHS, optional Ipopt/SCIP) ─────────────────────────────────
# Embedded SCIP uses POSIX sigaction(), which the Windows/UCRT CRT does not
# provide. Define SCIP_NO_SIGACTION so SCIP falls back to signal() there.
# SCIP also calls POSIX strtok_r(); MSVC provides the compatible strtok_s().
# Applied before the dependency add_subdirectory so it propagates to SCIP.
if(WIN32)
  add_compile_definitions(SCIP_NO_SIGACTION)
  add_compile_definitions(strtok_r=strtok_s)
endif()
include(cmake/Dependencies.cmake)

set(HACDCPF_HAVE_OPENDSS OFF)
set(HACDCPF_DSS_CAPI_RUNTIME_DLLS)
if(HACDCPF_ENABLE_OPENDSS OR HACDCPF_ENABLE_OPENDSS_COMPARE)
  if(NOT WIN32)
    message(FATAL_ERROR
      "The vendored DSS C-API bundle is currently Windows x64 only. "
      "Add a platform-specific bundle before enabling OpenDSS here.")
  endif()

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
    PATHS "${_hacdcpf_dss_capi_lib_dir}" "${HACDCPF_DSS_CAPI_ROOT}/lib"
    NO_DEFAULT_PATH)
  find_file(_hacdcpf_dss_capi_runtime
    NAMES altdss_capi.dll dss_capi.dll
    PATHS "${_hacdcpf_dss_capi_lib_dir}" "${HACDCPF_DSS_CAPI_ROOT}/bin"
    NO_DEFAULT_PATH)

  if(NOT _hacdcpf_dss_capi_implib OR NOT _hacdcpf_dss_capi_runtime)
    message(FATAL_ERROR
      "OpenDSS is enabled, but the vendored DSS C-API library/runtime was not "
      "found under ${HACDCPF_DSS_CAPI_ROOT}. Expected altdss_capi.lib/.dll.")
  endif()

  add_library(dss_capi::dss_capi SHARED IMPORTED GLOBAL)
  set_target_properties(dss_capi::dss_capi PROPERTIES
    IMPORTED_IMPLIB "${_hacdcpf_dss_capi_implib}"
    IMPORTED_LOCATION "${_hacdcpf_dss_capi_runtime}"
    INTERFACE_INCLUDE_DIRECTORIES "${_hacdcpf_dss_capi_include}")

  get_filename_component(_hacdcpf_dss_capi_runtime_dir
    "${_hacdcpf_dss_capi_runtime}" DIRECTORY)
  file(GLOB _hacdcpf_dss_capi_all_dlls
    "${_hacdcpf_dss_capi_runtime_dir}/*.dll")
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
  src/power_flow/distribution_power_flow.cpp
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
  # ── short circuit analysis ───────────────────────────────────────────────
  src/short_circuit/short_circuit.cpp
  # ── scenario generation ───────────────────────────────────────────────────────
  src/scenario_generation/scenario_generation.cpp

  # ── analysis ────────────────────────────────────────────────────────
  src/scenario_generation/typhoon_resilience.cpp
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
if(HACDCPF_HAVE_OPENDSS)
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
if(HACDCPF_HAVE_OPENDSS)
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_HAVE_OPENDSS=1)
else()
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_NO_OPENDSS)
endif()

# ── Link against MIPSolvers (brings Eigen3, fmt, nlohmann_json, HiGHS, Ipopt) ─
target_link_libraries(hacdcpf PUBLIC mipsolvers::mipsolvers)

# ── ETAP Excel I/O backend (OpenXLSX) ────────────────────────────────────
if(HACDCPF_ENABLE_ETAP AND TARGET OpenXLSX::OpenXLSX)
  target_link_libraries(hacdcpf PUBLIC OpenXLSX::OpenXLSX)
endif()
if(HACDCPF_HAVE_OPENDSS)
  target_link_libraries(hacdcpf PUBLIC dss_capi::dss_capi)
endif()
if(HACDCPF_HAVE_OPENDSS_COMPARE)
  target_compile_definitions(hacdcpf PUBLIC HACDCPF_HAVE_OPENDSS_COMPARE=1)

  add_executable(opendss_snapshot_adapter tools/opendss_snapshot_adapter.cpp)
  target_link_libraries(opendss_snapshot_adapter PRIVATE hacdcpf::hacdcpf)
  target_include_directories(opendss_snapshot_adapter PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/tools")
  target_compile_definitions(opendss_snapshot_adapter PRIVATE
    HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
  hacdcpf_copy_opendss_runtime(opendss_snapshot_adapter)

  add_executable(opendss_pf_compare tools/opendss_pf_compare.cpp)
  target_link_libraries(opendss_pf_compare PRIVATE hacdcpf::hacdcpf)
  target_include_directories(opendss_pf_compare PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/tools")
  target_compile_definitions(opendss_pf_compare PRIVATE
    HACDCPF_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
  hacdcpf_copy_opendss_runtime(opendss_pf_compare)
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

# ── Tests ─────────────────────────────────────────────────────────────────────
if(HACDCPF_BUILD_TESTS)
  include(CTest)
  enable_testing()
  add_subdirectory(tests)
endif()
