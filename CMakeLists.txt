cmake_minimum_required(VERSION 3.20)

project(MIPSolvers VERSION 0.1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Build type defaults ───────────────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Build type (Debug, Release, RelWithDebInfo)" FORCE)
endif()

# ── Options ───────────────────────────────────────────────────────────────────
option(MIPSOLVERS_BUILD_TESTS     "Build unit/integration tests"               ON)
option(MIPSOLVERS_ENABLE_WERROR   "Treat warnings as errors for core library"  OFF)
if(APPLE)
  set(_MIPSOLVERS_ENABLE_IPO_DEFAULT OFF)
else()
  set(_MIPSOLVERS_ENABLE_IPO_DEFAULT ON)
endif()
option(MIPSOLVERS_ENABLE_IPO
       "Enable interprocedural/link-time optimization for optimized builds"
       ${_MIPSOLVERS_ENABLE_IPO_DEFAULT})
unset(_MIPSOLVERS_ENABLE_IPO_DEFAULT)
option(MIPSOLVERS_BUILD_HFACTOR   "Build vendored HiGHS HFactor static lib"    ON)
option(MIPSOLVERS_EIGEN_VECTORIZE "Enable Eigen vectorization intrinsics"      ON)
option(MIPSOLVERS_ENABLE_NATIVE_ARCH
       "Enable -march=native for host-specific builds"                         OFF)
option(MIPSOLVERS_BUILD_SCUC "Build scuc_solve command-line tool"              ON)
option(MIPSOLVERS_BUILD_SCUC_CASE_BUILDER "Build scuc_case_builder tool"       ON)
option(MIPSOLVERS_BUILD_PYTHON "Build Python (pybind11) bindings"              OFF)
set(MIPSOLVERS_EIGEN_MAX_ALIGN_BYTES "32" CACHE STRING
    "Eigen max alignment bytes (16/32/64; empty = Eigen default)")

# Link-time optimization for optimized builds. AppleClang embeds SDK-version
# module flags in LTO objects; mixing static libraries built under nearby SDKs
# emits noisy linker warnings, so macOS defaults this option off.
if(MIPSOLVERS_ENABLE_IPO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT _ipo_ok OUTPUT _ipo_out)
  if(_ipo_ok)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON)
  else()
    message(STATUS "mipsolvers: IPO/LTO requested but unavailable: ${_ipo_out}")
  endif()
endif()

# ── Dependencies ─────────────────────────────────────────────────────────────
include(cmake/Dependencies.cmake)

# ── Engine source files ───────────────────────────────────────────────────────
add_library(mipsolvers STATIC
  src/engine/api/solver.cpp
  src/engine/strategy/dispatcher.cpp
  src/engine/strategy/warmstart.cpp
  src/engine/strategy/highs_presolve_side_state.cpp
  src/engine/strategy/presolve_manager.cpp
  src/engine/strategy/postsolve_manager.cpp
  src/engine/strategy/papilo_presolve.cpp
  src/engine/solver/solver_adapter.cpp
  src/engine/solver/adapter_registry.cpp
  src/engine/solver/native/native_adapters.cpp
  src/engine/solver/external/adapters.cpp
  src/engine/solver/native/lp/pdlp_solver.cpp
  src/engine/solver/native/nle/nle_solver.cpp
  src/engine/kernel/linear_algebra/linear_solver.cpp
  src/engine/kernel/linear_algebra/sparse_lu_factor.cpp
  src/engine/kernel/linear_algebra/hfactor_backend.cpp
  src/engine/kernel/lp_kernel/dual_simplex.cpp
  src/engine/kernel/lp_kernel/dual_simplex_api.cpp
  src/engine/kernel/kkt/kkt_system.cpp
  src/engine/kernel/ipm/lcqp_solver.cpp
  src/engine/kernel/ipm/ipm_lp_solver.cpp
  src/engine/kernel/ipm/ipm_solver.cpp
  src/engine/kernel/ipm/ipm_filter.cpp
  src/engine/kernel/ipm/ipm_scaling.cpp
  src/engine/kernel/ipm/ipm_restoration.cpp
  src/engine/util/problem_validation.cpp
  src/l2o/branching_policy.cpp
  src/l2o/feature_schema.cpp
  src/l2o/model_fingerprint.cpp
  src/l2o/policy.cpp
  src/l2o/scuc_warm_start.cpp
  src/l2o/solver_config_policy.cpp
  src/l2o/trace_event.cpp
  src/l2o/trace_writer.cpp
  src/aml/model.cpp
  src/aml/set.cpp
)

add_library(mipsolvers::mipsolvers ALIAS mipsolvers)

# ── Include directories ───────────────────────────────────────────────────────
target_include_directories(mipsolvers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ── Mandatory link libraries ──────────────────────────────────────────────────
target_link_libraries(mipsolvers PUBLIC Eigen3::Eigen fmt::fmt nlohmann_json::nlohmann_json)

# ── Compile definitions ───────────────────────────────────────────────────────
if(NOT MIPSOLVERS_EIGEN_VECTORIZE)
  target_compile_definitions(mipsolvers PUBLIC
    EIGEN_DONT_VECTORIZE=1 EIGEN_UNALIGNED_VECTORIZE=0)
endif()
if(MIPSOLVERS_EIGEN_MAX_ALIGN_BYTES)
  target_compile_definitions(mipsolvers PUBLIC
    EIGEN_MAX_ALIGN_BYTES=${MIPSOLVERS_EIGEN_MAX_ALIGN_BYTES})
endif()
if(WIN32)
  target_compile_definitions(mipsolvers PUBLIC NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

# ── HFactor vendored library ──────────────────────────────────────────────────
if(MIPSOLVERS_BUILD_HFACTOR)
  add_subdirectory(src/engine/kernel/linear_algebra/highs_factor)
  target_link_libraries(mipsolvers PRIVATE mipsolvers_hfactor)
  target_compile_definitions(mipsolvers PUBLIC MIPSOLVERS_HAVE_HFACTOR=1 HACDCPF_HAVE_HFACTOR=1)
endif()

# ── HiGHS in-process library ─────────────────────────────────────────────────
# The native branch-and-cut solver requires the in-process HiGHS library for
# LP relaxation and dynamic cut generation (strict-HiGHS mode).  When HiGHS is
# not available the B&C sources are excluded entirely, following the same
# pattern as HybridACDCPowerSystemsPlanning/CMakeLists.txt.
if(MIPSOLVERS_HAVE_HIGHS_LIB)
  target_link_libraries(mipsolvers PRIVATE highs)
  target_compile_definitions(mipsolvers PUBLIC
    MIPSOLVERS_HAVE_HIGHS_LIB=1 HACDCPF_HAVE_HIGHS_LIB=1)
  target_compile_definitions(mipsolvers PRIVATE
    MIPSOLVERS_HIGHS_LIB_SOURCE="${MIPSOLVERS_HIGHS_LIB_SOURCE}"
    HACDCPF_HIGHS_LIB_SOURCE="${MIPSOLVERS_HIGHS_LIB_SOURCE}")

  # Native B&C: strict HiGHS + dynamic cutting planes
  target_sources(mipsolvers PRIVATE
    src/engine/solver/native/milp/bc/legacy/branch_and_cut.cpp
    src/engine/solver/native/milp/bc/legacy/bc_parallel.cpp
    src/engine/solver/native/milp/bc/legacy/bc_utils.cpp
    src/engine/solver/native/milp/bc/legacy/bc_branching.cpp
    src/engine/solver/native/milp/bc/legacy/bc_cuts.cpp
    src/engine/solver/native/milp/bc/legacy/bc_cglp.cpp
    src/engine/solver/native/milp/bc/legacy/bc_cglp_model.cpp
    src/engine/solver/native/milp/bc/legacy/bc_clique_table.cpp
    src/engine/solver/native/milp/bc/legacy/bc_relaxation.cpp
    src/engine/solver/native/milp/bc/legacy/bc_conformance_trace.cpp
    src/engine/solver/native/milp/bc/legacy/bc_implied_bounds.cpp
    src/engine/solver/native/milp/bc/legacy/bc_legacy_helpers.cpp
    src/engine/solver/native/milp/bc/legacy/bc_minlp_legacy.cpp
    src/engine/solver/native/milp/bc/legacy/bc_proof_artifacts.cpp
    src/engine/solver/native/milp/bc/legacy/bc_root_audit.cpp
    src/engine/solver/native/milp/bc/legacy/bc_uc_trace.cpp
    src/engine/solver/native/milp/bc/legacy/bc_validation.cpp
    src/engine/solver/native/milp/bc/milp_presolve.cpp
    src/engine/solver/native/milp/bc/parallel/shared_state.cpp
    src/engine/solver/native/milp/bc/parallel/parallel_search.cpp
    src/engine/solver/native/milp/bc/api.cpp
    src/engine/solver/native/milp/bc/search/node_evaluator.cpp
    src/engine/solver/native/milp/bc/root/root_solve.cpp
    src/engine/solver/native/milp/bc/search/sequential_search.cpp
  )
  target_compile_definitions(mipsolvers PUBLIC
    MIPSOLVERS_NATIVE_BC_STRICT_HIGHS=1
    MIPSOLVERS_NATIVE_BC_DYNAMIC_CUTS=1)

  # MILP benchmark runner: tests native B&C performance with strict HiGHS LP
  # kernel and dynamic cutting planes.  Only built when the benchmark source
  # exists and HiGHS is available (mirrors the pattern in
  # HybridACDCPowerSystemsPlanning/CMakeLists.txt).
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmark/milp_benchmark_runner.cpp")
    add_executable(milp_benchmark_runner
      benchmark/milp_benchmark_runner.cpp
      src/scuc/scuc.cpp
      src/scuc/case_builder.cpp)
    target_include_directories(milp_benchmark_runner PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(milp_benchmark_runner PRIVATE mipsolvers highs::highs)
    if(NOT MSVC)
      target_compile_options(milp_benchmark_runner PRIVATE -Wall -Wextra
        $<$<CONFIG:Release>:-O3 -DNDEBUG>
        $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>)
    endif()

    # CTest integration: run the --full suite (6 UC cases, ~30 s on CI) with
    # B&C[StrictHiGHS] only (no external solvers).  JSON output is written to
    # the build tree so a post-step can compare against the stored baseline.
    if(MIPSOLVERS_BUILD_TESTS)
      add_test(NAME milp_benchmark
        COMMAND milp_benchmark_runner
                --full
                --no-highs-adapter
                --json "${CMAKE_CURRENT_BINARY_DIR}/milp_benchmark_result.json")
      set_tests_properties(milp_benchmark PROPERTIES
        LABELS   "benchmark"
        TIMEOUT  300)

      # Cross-platform perf-regression check: compare the JSON output from the
      # run above against the checked-in baseline using the Python helper.
      if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmark/check_perf_regression.py")
        find_program(_PYTHON_EXE NAMES python3 python)
        if(_PYTHON_EXE)
          add_test(NAME milp_benchmark_regression
            COMMAND "${_PYTHON_EXE}"
              "${CMAKE_CURRENT_SOURCE_DIR}/benchmark/check_perf_regression.py"
              "--result"    "${CMAKE_CURRENT_BINARY_DIR}/milp_benchmark_result.json"
              "--baseline"  "${CMAKE_CURRENT_SOURCE_DIR}/benchmark/milp_benchmark_baseline.json"
              "--tolerance" "0.25")
          set_tests_properties(milp_benchmark_regression PROPERTIES
            LABELS   "benchmark"
            TIMEOUT  30
            DEPENDS  milp_benchmark)
        endif()
      endif()
    endif()
  endif()

  # Cross-solver comparison driver: runs the same SCUC MIP instances through
  # Native, Gurobi, HiGHS and SCIP and prints a side-by-side performance table.
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmark/solver_comparison.cpp")
    add_executable(solver_comparison
      benchmark/solver_comparison.cpp
      src/scuc/scuc.cpp
      src/scuc/case_builder.cpp)
    target_include_directories(solver_comparison PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(solver_comparison PRIVATE mipsolvers)
    if(NOT MSVC)
      target_compile_options(solver_comparison PRIVATE -Wall -Wextra
        $<$<CONFIG:Release>:-O3 -DNDEBUG>
        $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>)
    endif()
  endif()
endif()

# ── SCIP in-process library ───────────────────────────────────────────────────
if(MIPSOLVERS_HAVE_SCIP_LIB)
  target_link_libraries(mipsolvers PRIVATE libscip)
  target_compile_definitions(mipsolvers PUBLIC
    MIPSOLVERS_HAVE_SCIP_LIB=1 HACDCPF_HAVE_SCIP_LIB=1)
  target_compile_definitions(mipsolvers PRIVATE
    MIPSOLVERS_SCIP_LIB_SOURCE="${MIPSOLVERS_SCIP_LIB_SOURCE}"
    HACDCPF_SCIP_LIB_SOURCE="${MIPSOLVERS_SCIP_LIB_SOURCE}")
endif()

# ── SuiteSparse ───────────────────────────────────────────────────────────────
# PRIVATE: 公共 API 头文件不暴露 SuiteSparse 类型；消费者不需要直接链接这些库。
# 将绝对路径限定在本项目构建内，避免平台相关路径写入导出的 targets 文件。
if(MIPSOLVERS_HAVE_SUITESPARSE)
  target_include_directories(mipsolvers PRIVATE ${MIPSOLVERS_SUITESPARSE_INCLUDE_DIRS})
  # BUILD_INTERFACE: SuiteSparse 是平台特定共享库，不向消费者导出绝对路径；
  # mipsolversConfig.cmake 会在消费者机器上用 find_package 重新找到它们。
  foreach(_ss_lib IN LISTS MIPSOLVERS_SUITESPARSE_LIBRARIES)
    target_link_libraries(mipsolvers PRIVATE "$<BUILD_INTERFACE:${_ss_lib}>")
  endforeach()
  target_compile_definitions(mipsolvers PUBLIC
    MIPSOLVERS_HAVE_SUITESPARSE=1 HACDCPF_HAVE_SUITESPARSE=1)
  if(MIPSOLVERS_HAVE_UMFPACK)
    target_compile_definitions(mipsolvers PUBLIC
      MIPSOLVERS_HAVE_UMFPACK=1 HACDCPF_HAVE_UMFPACK=1)
  endif()
  if(MIPSOLVERS_HAVE_KLU)
    target_compile_definitions(mipsolvers PUBLIC
      MIPSOLVERS_HAVE_KLU=1 HACDCPF_HAVE_KLU=1)
  endif()
  if(APPLE)
    # PRIVATE rpath: 仅影响本项目产出物，不传播给消费者
    target_link_options(mipsolvers PRIVATE
      "-Wl,-rpath,/opt/homebrew/opt/suite-sparse/lib"
      "-Wl,-rpath,/usr/local/opt/suite-sparse/lib")
  endif()
endif()

# ── SuperLU ───────────────────────────────────────────────────────────────────
# PRIVATE：同 SuiteSparse，不向消费者传播绝对路径。
if(MIPSOLVERS_HAVE_SUPERLU)
  target_include_directories(mipsolvers PRIVATE ${MIPSOLVERS_SUPERLU_INCLUDE_DIR})
  foreach(_slu_lib IN LISTS MIPSOLVERS_SUPERLU_LIBRARIES)
    target_link_libraries(mipsolvers PRIVATE "$<BUILD_INTERFACE:${_slu_lib}>")
  endforeach()
  target_compile_definitions(mipsolvers PUBLIC
    MIPSOLVERS_HAVE_SUPERLU=1 HACDCPF_HAVE_SUPERLU=1)
  if(UNIX AND NOT APPLE)
    # PRIVATE rpath：仅影响本项目产出物，不传播给消费者
    get_filename_component(_SLU_RPATH "${_MIPSOLVERS_SUPERLU_LIB}" DIRECTORY)
    target_link_options(mipsolvers PRIVATE "-Wl,-rpath,${_SLU_RPATH}")
  endif()
endif()

# ── Intel MKL PARDISO ─────────────────────────────────────────────────────────
# PRIVATE：MKL 路径平台相关（Linux/Windows 完全不同），不向消费者传播。
if(MIPSOLVERS_HAVE_MKL_PARDISO)
  target_include_directories(mipsolvers PRIVATE ${MIPSOLVERS_MKL_INCLUDE_DIRS})
  foreach(_mkl_lib IN LISTS MIPSOLVERS_MKL_LIBRARIES)
    target_link_libraries(mipsolvers PRIVATE "$<BUILD_INTERFACE:${_mkl_lib}>")
  endforeach()
  target_compile_definitions(mipsolvers PUBLIC
    MIPSOLVERS_HAVE_MKL_PARDISO=1 HACDCPF_HAVE_MKL_PARDISO=1)
  if(UNIX AND NOT APPLE)
    get_filename_component(_MKL_RPATH "${MIPSOLVERS_MKL_LP64_LIB}" DIRECTORY)
    target_link_options(mipsolvers PRIVATE "-Wl,-rpath,${_MKL_RPATH}")
  endif()
endif()

# ── Ipopt ─────────────────────────────────────────────────────────────────────
# PRIVATE：Ipopt 及其 MUMPS/OpenBLAS/gfortran 传递依赖路径均为平台特定的
# Homebrew/apt 绝对路径，不应写入导出的 targets 文件。
if(MIPSOLVERS_HAVE_IPOPT)
  target_include_directories(mipsolvers PRIVATE ${MIPSOLVERS_IPOPT_INCLUDE_DIRS})
  target_link_libraries(mipsolvers PRIVATE ${MIPSOLVERS_IPOPT_LIBRARIES})
  target_compile_definitions(mipsolvers PUBLIC
    MIPSOLVERS_HAVE_IPOPT=1 HACDCPF_HAVE_IPOPT=1)
endif()

# ── Gurobi ────────────────────────────────────────────────────────────────────
# PRIVATE：Gurobi 动态库路径（/Library/gurobi.../macOS, C:\gurobi.../Windows）
# 完全平台相关，且是可选外部适配器，不向消费者传播。
if(MIPSOLVERS_HAVE_GUROBI)
  target_include_directories(mipsolvers PRIVATE ${MIPSOLVERS_GUROBI_INCLUDE_DIRS})
  # BUILD_INTERFACE: Gurobi 是可选外部适配器，路径因版本/平台而异，
  # 消费者需自行安装 Gurobi 并提供链接路径。
  foreach(_gurobi_lib IN LISTS MIPSOLVERS_GUROBI_LIBRARIES)
    target_link_libraries(mipsolvers PRIVATE "$<BUILD_INTERFACE:${_gurobi_lib}>")
  endforeach()
  target_compile_definitions(mipsolvers PUBLIC
    MIPSOLVERS_HAVE_GUROBI=1 HACDCPF_HAVE_GUROBI=1)
endif()

# ── PaPILO ────────────────────────────────────────────────────────────────────
# PRIVATE：papilo_presolve.hpp 使用完全前向声明，不 #include 任何 papilo 头文件，
# 消费者无需 papilo 头文件或链接路径。
if(MIPSOLVERS_HAVE_PAPILO)
  target_link_libraries(mipsolvers PRIVATE papilo)
  target_compile_definitions(mipsolvers PUBLIC
    MIPSOLVERS_HAVE_PAPILO=1 HACDCPF_HAVE_PAPILO=1)
endif()

# ── Apple Accelerate ─────────────────────────────────────────────────────────
# 使用 generator expression 而非裸字符串，使导出的 targets 文件在非 Apple
# 平台（Linux/Windows）上不会尝试链接 -framework Accelerate。
# GCC on macOS does not provide the same Accelerate sparse API surface used by
# the IPM fast path, so only enable this path with Clang-family compilers.
if(APPLE AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang")
  target_compile_definitions(mipsolvers PUBLIC
    MIPSOLVERS_HAVE_ACCELERATE=1 HACDCPF_HAVE_ACCELERATE=1)
endif()
target_link_libraries(mipsolvers PUBLIC
  "$<$<PLATFORM_ID:Darwin>:-framework Accelerate>")

# ── Bundled single static archive ─────────────────────────────────────────────
# Merge libmipsolvers.a with every static dependency we build in-tree (HiGHS,
# SCIP, Ipopt, MUMPS, HiGHS-factor) plus the static LUSOL archive into a single
# self-contained archive `libmipsolvers_bundled.a`, for downstream applications
# that want one .a to link.  External *shared* libraries (SuiteSparse, OpenBLAS,
# fmt, GMP, TBB, Boost, Gurobi, gfortran/quadmath, Accelerate) cannot be merged
# into a static archive and must still be provided by the consumer at link time;
# see docs/build_and_deploy.md for the full consumer link line.
option(MIPSOLVERS_BUILD_BUNDLED_ARCHIVE
  "Produce a single bundled static archive (libmipsolvers_bundled.a)" ON)
if(MIPSOLVERS_BUILD_BUNDLED_ARCHIVE)
  set(_bundle_inputs "$<TARGET_FILE:mipsolvers>")
  foreach(_dep IN ITEMS mipsolvers_hfactor highs libscip ipopt_local dmumps)
    if(TARGET ${_dep})
      # Only static-library targets have an archive file to bundle.  When a
      # dependency is provided as an INTERFACE target (e.g. dmumps wrapping a
      # system/Homebrew MUMPS shared library), it has no $<TARGET_FILE> and is
      # supplied to consumers at link time like the other external shared libs.
      get_target_property(_dep_type ${_dep} TYPE)
      if(_dep_type STREQUAL "STATIC_LIBRARY")
        list(APPEND _bundle_inputs "$<TARGET_FILE:${_dep}>")
      endif()
    endif()
  endforeach()
  # Static LUSOL archive pulled in transitively via PaPILO (not a CMake target).
  find_library(MIPSOLVERS_CLUSOL_STATIC_LIB
    NAMES clusol libclusol
    PATHS /opt/homebrew/lib /usr/local/lib
    NO_CACHE)
  if(MIPSOLVERS_CLUSOL_STATIC_LIB AND MIPSOLVERS_CLUSOL_STATIC_LIB MATCHES "\\.a$")
    list(APPEND _bundle_inputs "${MIPSOLVERS_CLUSOL_STATIC_LIB}")
  endif()

  set(_bundle_output "${CMAKE_BINARY_DIR}/libmipsolvers_bundled.a")
  if(APPLE)
    # macOS libtool cleanly merges archives and de-dups member names.
    add_custom_command(
      OUTPUT "${_bundle_output}"
      COMMAND libtool -static -o "${_bundle_output}" ${_bundle_inputs}
      DEPENDS mipsolvers
      COMMENT "Bundling single static archive: ${_bundle_output}"
      VERBATIM)
  else()
    # GNU ar MRI script: ADDLIB each input, then SAVE.
    set(_mri "${CMAKE_BINARY_DIR}/mipsolvers_bundle.mri")
    set(_mri_lines "CREATE ${_bundle_output}")
    foreach(_in IN LISTS _bundle_inputs)
      string(APPEND _mri_lines "\nADDLIB ${_in}")
    endforeach()
    string(APPEND _mri_lines "\nSAVE\nEND\n")
    file(GENERATE OUTPUT "${_mri}" CONTENT "${_mri_lines}")
    add_custom_command(
      OUTPUT "${_bundle_output}"
      COMMAND "${CMAKE_AR}" -M < "${_mri}"
      DEPENDS mipsolvers "${_mri}"
      COMMENT "Bundling single static archive: ${_bundle_output}"
      VERBATIM)
  endif()
  add_custom_target(mipsolvers_bundled ALL DEPENDS "${_bundle_output}")
endif()

# ── Compiler warnings ────────────────────────────────────────────────────────
if(MSVC)
  target_compile_options(mipsolvers PRIVATE /W4)
  if(MIPSOLVERS_ENABLE_WERROR)
    target_compile_options(mipsolvers PRIVATE /WX)
  endif()
  target_compile_options(mipsolvers PRIVATE
    $<$<CONFIG:Release>:/O2>
    $<$<CONFIG:Release>:/Oi>
    $<$<CONFIG:Release>:/fp:fast>
    $<$<CONFIG:Release>:/DNDEBUG>
    $<$<CONFIG:RelWithDebInfo>:/O2>
    $<$<CONFIG:RelWithDebInfo>:/Oi>
    $<$<CONFIG:RelWithDebInfo>:/Zi>
    $<$<CONFIG:RelWithDebInfo>:/DNDEBUG>)
else()
  target_compile_options(mipsolvers PRIVATE -Wall -Wextra -Wpedantic)
  if(MIPSOLVERS_ENABLE_WERROR)
    target_compile_options(mipsolvers PRIVATE -Werror)
  endif()
  target_compile_options(mipsolvers PRIVATE
    $<$<CONFIG:Release>:-O3>
    $<$<CONFIG:Release>:-DNDEBUG>
    $<$<CONFIG:RelWithDebInfo>:-O2>
    $<$<CONFIG:RelWithDebInfo>:-g>
    $<$<CONFIG:RelWithDebInfo>:-DNDEBUG>)
  if(MIPSOLVERS_ENABLE_NATIVE_ARCH)
    target_compile_options(mipsolvers PRIVATE
      $<$<CONFIG:Release>:-march=native>
      $<$<CONFIG:RelWithDebInfo>:-march=native>)
  endif()
endif()

# ── Tests ─────────────────────────────────────────────────────────────────────
if(MIPSOLVERS_BUILD_TESTS)
  enable_testing()
  include(CTest)

  macro(mipsolvers_add_test _name)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/${_name}.cpp")
      add_executable(${_name} tests/${_name}.cpp)
      target_link_libraries(${_name} PRIVATE mipsolvers Catch2::Catch2WithMain)
      target_include_directories(${_name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/tests)
      add_test(NAME ${_name} COMMAND ${_name})
      set(_tier "")
      set(_margs ${ARGN})
      list(LENGTH _margs _nargs)
      if(_nargs GREATER 1)
        list(GET _margs 0 _kw)
        if(_kw STREQUAL "TIER")
          list(GET _margs 1 _tier)
        endif()
      endif()
      if(_tier)
        set_tests_properties(${_name} PROPERTIES LABELS "${_tier}")
      endif()
    endif()
  endmacro()

  mipsolvers_add_test(test_engine_api          TIER unit)
  mipsolvers_add_test(test_lp_solver           TIER unit)
  mipsolvers_add_test(test_milp_solver         TIER unit)
  mipsolvers_add_test(test_ipm_solver          TIER unit)
  mipsolvers_add_test(test_dual_simplex        TIER unit)
  mipsolvers_add_test(test_branch_and_cut      TIER integration)
  mipsolvers_add_test(test_problem_validation  TIER unit)
  mipsolvers_add_test(test_adapter_registry    TIER unit)
  mipsolvers_add_test(test_presolve            TIER unit)
  mipsolvers_add_test(test_l2o_trace           TIER unit)

  # SCUC module test requires scuc.cpp (not in base mipsolvers lib)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_scuc_module.cpp" AND MIPSOLVERS_BUILD_SCUC)
    add_executable(test_scuc_module
      tests/test_scuc_module.cpp
      src/scuc/scuc.cpp
    )
    target_link_libraries(test_scuc_module PRIVATE mipsolvers Catch2::Catch2WithMain)
    target_include_directories(test_scuc_module PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    add_test(NAME test_scuc_module COMMAND test_scuc_module)
    set_tests_properties(test_scuc_module PROPERTIES LABELS "integration")
  endif()

  # SCUC 市场模拟基准 & 可视化测试
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_market_simulation.cpp" AND MIPSOLVERS_BUILD_SCUC)
    add_executable(test_market_simulation
      tests/test_market_simulation.cpp
      src/scuc/scuc.cpp
      src/scuc/case_builder.cpp
    )
    target_link_libraries(test_market_simulation PRIVATE mipsolvers Catch2::Catch2WithMain)
    target_include_directories(test_market_simulation PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    if(NOT MSVC)
      target_compile_options(test_market_simulation PRIVATE -Wall -Wextra
        $<$<CONFIG:Release>:-O3 -DNDEBUG>
        $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>)
    endif()
    add_test(NAME test_market_simulation COMMAND test_market_simulation)
    set_tests_properties(test_market_simulation PROPERTIES
      LABELS "benchmark"
      TIMEOUT 600)
  endif()
endif()

# ── SCUC market simulation executable ────────────────────────────────────────
if(MIPSOLVERS_BUILD_SCUC)
  add_executable(scuc_solve
    src/scuc/main.cpp
    src/scuc/scuc.cpp
  )
  target_include_directories(scuc_solve PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(scuc_solve PRIVATE mipsolvers)
  if(NOT MSVC)
    target_compile_options(scuc_solve PRIVATE -Wall -Wextra
      $<$<CONFIG:Release>:-O3 -DNDEBUG>
      $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>)
  endif()
endif()

# ── SCUC case builder tool ─────────────────────────────────────────────────────
if(MIPSOLVERS_BUILD_SCUC_CASE_BUILDER AND MIPSOLVERS_BUILD_SCUC)
  add_executable(scuc_case_builder
    src/scuc/case_builder_main.cpp
    src/scuc/case_builder.cpp
    src/scuc/scuc.cpp
  )
  target_include_directories(scuc_case_builder PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(scuc_case_builder PRIVATE mipsolvers)
  if(NOT MSVC)
    target_compile_options(scuc_case_builder PRIVATE -Wall -Wextra
      $<$<CONFIG:Release>:-O3 -DNDEBUG>
      $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>)
  endif()
endif()

# ── Python bindings (pybind11) ────────────────────────────────────────────────
if(MIPSOLVERS_BUILD_PYTHON)
  # Locate the Python interpreter + pybind11 installed into it.
  # Override with -DPYTHON_EXECUTABLE=/path/to/python3 if needed.
  if(NOT DEFINED PYTHON_EXECUTABLE)
    find_program(PYTHON_EXECUTABLE
      NAMES python3.8 python3.9 python3 python
      HINTS
        "$ENV{HOME}/.pyenv/versions/3.8.20/bin"
        /opt/homebrew/bin
        /usr/local/bin
        /usr/bin)
  endif()
  if(NOT PYTHON_EXECUTABLE)
    message(FATAL_ERROR
      "MIPSOLVERS_BUILD_PYTHON=ON but no Python interpreter found. "
      "Set -DPYTHON_EXECUTABLE=/path/to/python3.8")
  endif()

  # Ask pybind11 where its CMake config lives.
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c
      "import pybind11, os; print(os.path.dirname(pybind11.get_cmake_dir()))"
    OUTPUT_VARIABLE _pb11_cmake_root
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _pb11_ret)
  if(_pb11_ret)
    message(FATAL_ERROR
      "pybind11 not found for ${PYTHON_EXECUTABLE}. "
      "Install it with: ${PYTHON_EXECUTABLE} -m pip install pybind11")
  endif()

  # Use pybind11's own CMake integration.
  list(APPEND CMAKE_PREFIX_PATH "${_pb11_cmake_root}")
  set(Python3_EXECUTABLE "${PYTHON_EXECUTABLE}" CACHE FILEPATH
      "Python interpreter used for MIPSolvers Python bindings" FORCE)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
  set(PYBIND11_FINDPYTHON ON)
  find_package(pybind11 REQUIRED CONFIG)

  # Build the extension module.
  pybind11_add_module(mipsolvers_py MODULE
    src/python/mipsolvers_py.cpp
    src/python/aml_bindings.cpp
    src/scuc/scuc.cpp          # SCUC is header-only-ish; include impl
    src/scuc/case_builder.cpp
  )

  target_include_directories(mipsolvers_py PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

  target_link_libraries(mipsolvers_py PRIVATE mipsolvers)

  if(NOT MSVC)
    target_compile_options(mipsolvers_py PRIVATE -Wall -Wextra
      $<$<CONFIG:Release>:-O3 -DNDEBUG>
      $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>)
  endif()

  # Derive extension suffix from Python (e.g. .cpython-38-darwin.so).
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c
      "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'))"
    OUTPUT_VARIABLE _py_ext_suffix
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  set_target_properties(mipsolvers_py PROPERTIES
    OUTPUT_NAME "mipsolvers"
    SUFFIX "${_py_ext_suffix}")

  message(STATUS "Python bindings: ${PYTHON_EXECUTABLE}, suffix: ${_py_ext_suffix}")
endif()

# ── Install rules ─────────────────────────────────────────────────────────────
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(MIPSOLVERS_INSTALL_CMAKEDIR
    "${CMAKE_INSTALL_LIBDIR}/cmake/mipsolvers"
    CACHE STRING "Install directory for mipsolvers CMake package files")

# ── Locate header-only dep sources for self-contained bundled install ─────────
# Eigen3 和 nlohmann_json 均为纯头文件库，可随安装包一起分发，
# 让消费者无需单独安装这两个依赖。
if(TARGET Eigen3::Eigen)
  get_target_property(_ms_eigen3_incs Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
  foreach(_inc IN LISTS _ms_eigen3_incs)
    if(EXISTS "${_inc}/Eigen/Core")
      set(_ms_eigen3_bundle_src "${_inc}")
      break()
    endif()
  endforeach()
  unset(_inc)
endif()

if(TARGET nlohmann_json::nlohmann_json)
  get_target_property(_ms_json_incs nlohmann_json::nlohmann_json INTERFACE_INCLUDE_DIRECTORIES)
  foreach(_inc IN LISTS _ms_json_incs)
    if(EXISTS "${_inc}/nlohmann/json.hpp")
      set(_ms_json_bundle_src "${_inc}")
      break()
    endif()
  endforeach()
  unset(_inc)
endif()

install(TARGETS mipsolvers
  EXPORT mipsolversTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
if(MIPSOLVERS_BUILD_HFACTOR)
  install(TARGETS mipsolvers_hfactor
    EXPORT mipsolversTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()
if(TARGET highs)
  install(TARGETS highs
    EXPORT mipsolversTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
if(TARGET libscip)
  install(TARGETS libscip
    EXPORT mipsolversTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
if(TARGET ipopt_local)
  install(TARGETS ipopt_local
    EXPORT mipsolversTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
install(DIRECTORY include/mipsolvers
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# ── Bundle header-only deps ───────────────────────────────────────────────────
# 安装包含 Eigen3 和 nlohmann_json 头文件，使外部项目无需系统安装即可使用。
# mipsolversConfig.cmake 优先使用系统包，找不到时自动回退到此处的副本。
if(_ms_eigen3_bundle_src)
  install(DIRECTORY "${_ms_eigen3_bundle_src}/Eigen"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/mipsolvers-deps/eigen3")
  if(EXISTS "${_ms_eigen3_bundle_src}/unsupported")
    install(DIRECTORY "${_ms_eigen3_bundle_src}/unsupported"
      DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/mipsolvers-deps/eigen3")
  endif()
  message(STATUS "mipsolvers: bundling Eigen3 headers from ${_ms_eigen3_bundle_src}")
endif()

if(_ms_json_bundle_src)
  install(DIRECTORY "${_ms_json_bundle_src}/nlohmann"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/mipsolvers-deps/nlohmann_json")
  message(STATUS "mipsolvers: bundling nlohmann_json headers from ${_ms_json_bundle_src}")
endif()

install(EXPORT mipsolversTargets
  FILE mipsolversTargets.cmake
  NAMESPACE mipsolvers::
  DESTINATION ${MIPSOLVERS_INSTALL_CMAKEDIR})

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/mipsolversConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/mipsolversConfig.cmake
  INSTALL_DESTINATION ${MIPSOLVERS_INSTALL_CMAKEDIR}
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/mipsolversConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/mipsolversConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/mipsolversConfigVersion.cmake
  DESTINATION ${MIPSOLVERS_INSTALL_CMAKEDIR})
