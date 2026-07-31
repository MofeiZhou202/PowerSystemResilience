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
option(MIPSOLVERS_BUILD_HFACTOR   "Enable the HFactor basis backend"           ON)
option(MIPSOLVERS_EIGEN_VECTORIZE "Enable Eigen vectorization intrinsics"      ON)
option(MIPSOLVERS_ENABLE_NATIVE_ARCH
       "Enable -march=native for host-specific builds"                         OFF)
option(MIPSOLVERS_BUILD_SCUC "Build scuc_solve command-line tool"              ON)
option(MIPSOLVERS_BUILD_SCUC_CASE_BUILDER "Build scuc_case_builder tool"       ON)
option(MIPSOLVERS_BUILD_PYTHON "Build Python (pybind11) bindings"              OFF)
option(MIPSOLVERS_THIRD_PARTY_ONLY
       "Build/install only the vendored third-party dependencies"           OFF)
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
# Export set used by the Build*.cmake modules for their vendored targets:
# mipsolversTargets normally, mipsolversThirdPartyTargets in third-party-only
# mode (installed with no namespace so imported names equal in-tree names).
if(MIPSOLVERS_THIRD_PARTY_ONLY)
  # Catch2 is part of the third-party package; make Dependencies.cmake build it
  # even though this mode defines no test targets.
  set(MIPSOLVERS_BUILD_TESTS ON)
  set(MIPSOLVERS_THIRD_PARTY_EXPORT_SET mipsolversThirdPartyTargets)
else()
  set(MIPSOLVERS_THIRD_PARTY_EXPORT_SET mipsolversTargets)
endif()
include(cmake/Dependencies.cmake)

# ── Third-party-only mode: install the vendored deps and stop ────────────────
# No mipsolvers library, tests, tools, benchmarks, SCUC apps, bundled archive
# or python bindings are defined in this mode; `cmake --install` produces the
# prebuilt package consumed via MIPSOLVERS_USE_PREBUILT_THIRD_PARTY.
if(MIPSOLVERS_THIRD_PARTY_ONLY)
  include(cmake/InstallThirdParty.cmake)
  return()
endif()

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
  src/engine/solver/native/conic_ipm_adapter.cpp
  src/engine/solver/external/adapters.cpp
  src/engine/solver/native/lp/pdlp_solver.cpp
  src/engine/solver/native/nle/nle_solver.cpp
  src/engine/kernel/linear_algebra/linear_solver.cpp
  src/engine/kernel/linear_algebra/sparse_lu_factor.cpp
  src/engine/kernel/linear_algebra/hfactor_backend.cpp
  src/engine/kernel/linear_algebra/cholmod_ldlt.cpp
  src/engine/kernel/lp_kernel/dual_simplex.cpp
  src/engine/kernel/lp_kernel/dual_simplex_api.cpp
  src/engine/kernel/lp_kernel/native_dual/factor.cpp
  src/engine/kernel/lp_kernel/native_dual/state.cpp
  src/engine/kernel/lp_kernel/native_dual/pricing.cpp
  src/engine/kernel/lp_kernel/native_dual/primal.cpp
  src/engine/kernel/lp_kernel/native_dual/certificate.cpp
  src/engine/kernel/lp_kernel/native_dual/solver.cpp
  src/engine/kernel/kkt/kkt_system.cpp
  src/engine/kernel/ipm/lcqp_solver.cpp
  src/engine/kernel/ipm/chordal_decomposition.cpp
  src/engine/kernel/ipm/cones.cpp
  src/engine/kernel/ipm/conic_ipm_solver.cpp
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

# Public headers use C++20 language/library features.  Propagate that contract
# to installed consumers instead of relying only on this project's global
# CMAKE_CXX_STANDARD setting.
target_compile_features(mipsolvers PUBLIC cxx_std_20)

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
  # Embedded HiGHS already defines HFactor. Building and linking the standalone
  # mirror as well creates duplicate global symbols whose selected
  # implementation depends on archive order. Use the mirror only as a fallback
  # when the complete HiGHS library is unavailable.
  if(NOT MIPSOLVERS_HAVE_HIGHS_LIB)
    add_subdirectory(src/engine/kernel/linear_algebra/highs_factor)
    target_link_libraries(mipsolvers PRIVATE mipsolvers_hfactor)
  endif()
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
    src/engine/solver/native/milp/bc/legacy/bc_env_options.cpp
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
    set_target_properties(milp_benchmark_runner PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    target_include_directories(milp_benchmark_runner PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(milp_benchmark_runner PRIVATE mipsolvers highs::highs)
    if(NOT MSVC)
      target_compile_options(milp_benchmark_runner PRIVATE -Wall -Wextra
        $<$<CONFIG:Release>:-O3 -DNDEBUG>
        $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>)
    endif()

    # CTest integration uses the deterministic 6-bus smoke case. The default
    # and --full modes remain available for longer performance experiments.
    # JSON output is written to the build tree for optional post-processing.
    if(MIPSOLVERS_BUILD_TESTS)
      add_test(NAME milp_benchmark
        COMMAND milp_benchmark_runner
                --smoke
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
    set_target_properties(solver_comparison PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    target_include_directories(solver_comparison PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    target_link_libraries(solver_comparison PRIVATE mipsolvers)
    if(NOT MSVC)
      target_compile_options(solver_comparison PRIVATE -Wall -Wextra
        $<$<CONFIG:Release>:-O3 -DNDEBUG>
        $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>)
    endif()
  endif()

  # Pure-native vs HiGHS comparison driver: benchmarks the native B&C with
  # the experimental native dual simplex / IPM as LP kernel
  # (LpKernelBackend::ExperimentalNative) against raw-API HiGHS at both MILP
  # and LP level, including
  # adversarially scaled LPs as a numerical-stability probe.
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmark/native_kernel_comparison.cpp")
    add_executable(native_kernel_comparison
      benchmark/native_kernel_comparison.cpp
      src/scuc/scuc.cpp
      src/scuc/case_builder.cpp)
    set_target_properties(native_kernel_comparison PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    target_include_directories(native_kernel_comparison PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${CMAKE_CURRENT_SOURCE_DIR}/highs
      ${CMAKE_CURRENT_BINARY_DIR}/_deps/embedded_highs)
    target_link_libraries(native_kernel_comparison PRIVATE mipsolvers highs::highs)
    if(NOT MSVC)
      target_compile_options(native_kernel_comparison PRIVATE -Wall -Wextra
        $<$<CONFIG:Release>:-O3 -DNDEBUG>
        $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>)
    endif()

    # CTest integration: standing pure-native A/B harness.  --check turns the
    # must-pass rows (natDualSimplex NETLIB + scaled probes, NativeBC-vs-HiGHS
    # optimum agreement on the smoke MILP) into an exit code; natIPM rows stay
    # informational. Not part of the CI-critical set (CI runs -L unit and
    # -R milp_benchmark$ only). Default and --full retain the larger cases.
    if(MIPSOLVERS_BUILD_TESTS)
      add_test(NAME native_kernel_comparison
        COMMAND native_kernel_comparison --smoke --check --time-limit 60)
      set_tests_properties(native_kernel_comparison PROPERTIES
        LABELS   "benchmark"
        TIMEOUT  600)
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
  # Vendored SuiteSparse targets (offline) take precedence when built from
  # in-tree sources; otherwise link the system SuiteSparse libraries found by
  # Dependencies.cmake (BUILD_INTERFACE keeps their absolute paths out of the
  # exported targets).
  if(TARGET umfpack_vendored)
    target_link_libraries(mipsolvers PRIVATE umfpack_vendored)
  endif()
  if(TARGET klu_vendored)
    target_link_libraries(mipsolvers PRIVATE klu_vendored)
  endif()
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
endif()

# ── Vendored CHOLMOD ──────────────────────────────────────────────────────────
# Built from in-tree SuiteSparse sources (cmake/BuildCHOLMOD.cmake); PRIVATE
# include (public API headers never expose cholmod types) and exported with
# the mipsolversTargets set like the other vendored libraries.
if(MIPSOLVERS_HAVE_CHOLMOD)
  # BEFORE: the vendored headers must win over any system SuiteSparse dir.
  target_include_directories(mipsolvers BEFORE PRIVATE ${MIPSOLVERS_CHOLMOD_INCLUDE_DIRS})
  target_link_libraries(mipsolvers PRIVATE cholmod_vendored)
  target_compile_definitions(mipsolvers PUBLIC
    MIPSOLVERS_HAVE_CHOLMOD=1 HACDCPF_HAVE_CHOLMOD=1)
endif()

# ── MUMPS (multifrontal symmetric-indefinite direct solver) ──────────────────
# MUMPS::MUMPS is provided by cmake/BuildMUMPS.cmake (vendored sources or a
# system/Homebrew MUMPS wrapped as an INTERFACE target on Apple).  Links the
# headers (dmumps_c.h) and libraries into mipsolvers for the MumpsSolver
# sparse-direct backend used by the OPF/KKT paths.
option(MIPSOLVERS_USE_MUMPS "Enable the MUMPS sparse-direct backend" ON)
if(MIPSOLVERS_USE_MUMPS AND TARGET MUMPS::MUMPS)
  target_link_libraries(mipsolvers PRIVATE MUMPS::MUMPS)
  target_compile_definitions(mipsolvers PUBLIC
    MIPSOLVERS_HAVE_MUMPS=1 HACDCPF_HAVE_MUMPS=1)
  message(STATUS "mipsolvers: MUMPS sparse-direct backend enabled")
else()
  message(STATUS "mipsolvers: MUMPS backend unavailable (no MUMPS::MUMPS target)")
endif()

# ── OpenMP (optional) ─────────────────────────────────────────────────────────
# Parallelizes the big embarrassingly-parallel kernel loops (SpMV, theta/residual
# builds) with threshold guards; serial fallback when unavailable.
option(MIPSOLVERS_USE_OPENMP "Enable OpenMP in numerical kernels" ON)
# Recorded for the consumer config: when ON the exported static archive has a
# LINK_ONLY reference to OpenMP::OpenMP_CXX that consumers must recreate.
set(MIPSOLVERS_OPENMP_ENABLED OFF)
if(MIPSOLVERS_USE_OPENMP)
  find_package(OpenMP QUIET COMPONENTS CXX)
  if(NOT OpenMP_CXX_FOUND AND APPLE)
    # Apple Clang: needs -Xpreprocessor -fopenmp + explicit libomp paths.
    find_path(_OMP_INCLUDE omp.h
      HINTS /opt/homebrew/opt/libomp/include /usr/local/opt/libomp/include)
    find_library(_OMP_LIB NAMES omp
      HINTS /opt/homebrew/opt/libomp/lib /usr/local/opt/libomp/lib)
    if(_OMP_INCLUDE AND _OMP_LIB)
      set(OpenMP_CXX_FOUND ON)
      add_library(OpenMP::OpenMP_CXX INTERFACE IMPORTED)
      set_target_properties(OpenMP::OpenMP_CXX PROPERTIES
        INTERFACE_COMPILE_OPTIONS "-Xpreprocessor;-fopenmp"
        INTERFACE_INCLUDE_DIRECTORIES "${_OMP_INCLUDE}"
        INTERFACE_LINK_LIBRARIES "${_OMP_LIB}")
      unset(_OMP_INCLUDE)
      unset(_OMP_LIB)
    endif()
  endif()
  if(OpenMP_CXX_FOUND)
    target_link_libraries(mipsolvers PRIVATE OpenMP::OpenMP_CXX)
    target_compile_definitions(mipsolvers PRIVATE MIPSOLVERS_USE_OPENMP=1)
    set(MIPSOLVERS_OPENMP_ENABLED ON)
    message(STATUS "mipsolvers: OpenMP enabled in numerical kernels")
  else()
    message(STATUS "mipsolvers: OpenMP not found; numerical kernels stay serial")
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
  if(MIPSOLVERS_PAPILO_LOCAL_SOURCE)
    target_include_directories(mipsolvers SYSTEM PRIVATE
      ${MIPSOLVERS_PAPILO_INCLUDE_DIRS})
    target_compile_definitions(mipsolvers PRIVATE PAPILO_NO_CMAKE_CONFIG=1)
  else()
    target_link_libraries(mipsolvers PRIVATE papilo)
  endif()
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
# SCIP, Ipopt, MUMPS, HiGHS-factor, vendored fmt, vendored reference LAPACK)
# into a single self-contained archive `libmipsolvers_bundled.a`, for
# downstream applications that want one .a to link.  External *shared*
# libraries (SuiteSparse when not vendored, OpenBLAS, Gurobi,
# gfortran/quadmath on non-Apple, Accelerate) cannot be merged into a static
# archive and must still be provided by the consumer at link time; see
# docs/build_and_deploy.md for the full consumer link line.
option(MIPSOLVERS_BUILD_BUNDLED_ARCHIVE
  "Produce a single bundled static archive (libmipsolvers_bundled.a)" ON)
if(MIPSOLVERS_BUILD_BUNDLED_ARCHIVE)
  set(_bundle_inputs "$<TARGET_FILE:mipsolvers>")
  foreach(_dep IN ITEMS mipsolvers_hfactor highs libscip ipopt_local dmumps
                       fmt reflapack_vendored)
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
  # Multi-config generators (e.g., Visual Studio) need config-specific outputs
  # so generated command files do not conflict across Debug/Release variants.
  set(_bundle_output "${CMAKE_BINARY_DIR}/$<CONFIG>/libmipsolvers_bundled.a")
  if(APPLE)
    # macOS libtool cleanly merges archives and de-dups member names.
    add_custom_command(
      OUTPUT "${_bundle_output}"
      COMMAND ${CMAKE_COMMAND} -E make_directory "$<CONFIG>"
      COMMAND libtool -static -o "${_bundle_output}" ${_bundle_inputs}
      DEPENDS mipsolvers
      COMMENT "Bundling single static archive: ${_bundle_output}"
      VERBATIM)
  else()
    # GNU ar MRI script: ADDLIB each input, then SAVE.
    set(_mri "${CMAKE_BINARY_DIR}/$<CONFIG>/mipsolvers_bundle.mri")
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
      # Test executables live in tests/ (gitignored), not the repo root.
      set_target_properties(${_name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests")
      target_link_libraries(${_name} PRIVATE mipsolvers Catch2::Catch2WithMain)
      target_include_directories(${_name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/tests)
      if(MIPSOLVERS_THIRD_PARTY_PREBUILT)
        # A few regression tests include HiGHS headers directly. Use the
        # installed target's complete interface here; its include directory
        # contains the generated HConfig.h that is absent from a fresh dev
        # build's _deps directory.
        target_include_directories(${_name} PRIVATE
          $<TARGET_PROPERTY:highs,INTERFACE_INCLUDE_DIRECTORIES>)
      else()
        target_include_directories(${_name} PRIVATE
          ${CMAKE_CURRENT_SOURCE_DIR}/highs
          ${CMAKE_CURRENT_BINARY_DIR}/_deps/embedded_highs)
      endif()
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
  mipsolvers_add_test(test_ipopt_parameter_stability TIER integration)
  mipsolvers_add_test(test_lp_solver           TIER unit)
  mipsolvers_add_test(test_milp_solver         TIER unit)
  mipsolvers_add_test(test_ipm_solver          TIER unit)
  mipsolvers_add_test(test_conic_ipm           TIER unit)
  mipsolvers_add_test(test_aml_conic           TIER unit)
  # The OpenMP guard macro is PRIVATE to the library; expose it to the conic
  # test so its OpenMP-only sections (parallel determinism) are compiled.
  if(TARGET test_conic_ipm AND OpenMP_CXX_FOUND)
    target_compile_definitions(test_conic_ipm PRIVATE MIPSOLVERS_USE_OPENMP=1)
  endif()
  mipsolvers_add_test(test_dual_simplex        TIER unit)
  if(TARGET test_dual_simplex AND TARGET highs)
    target_link_libraries(test_dual_simplex PRIVATE highs::highs)
  endif()
  mipsolvers_add_test(test_branch_and_cut      TIER integration)
  mipsolvers_add_test(test_problem_validation  TIER unit)
  mipsolvers_add_test(test_adapter_registry    TIER unit)
  mipsolvers_add_test(test_presolve            TIER unit)
  mipsolvers_add_test(test_l2o_trace           TIER unit)
  mipsolvers_add_test(test_numerical_stability TIER unit)
  mipsolvers_add_test(test_netlib_regression   TIER integration)

  # Temporary OPF scalability probe (not a test; removed after use).
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmark/opf_scale_probe.cpp")
    add_executable(opf_scale_probe benchmark/opf_scale_probe.cpp)
    set_target_properties(opf_scale_probe PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    target_link_libraries(opf_scale_probe PRIVATE mipsolvers)
    target_include_directories(opf_scale_probe PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()

  # Conic IPM benchmark driver: synthetic SOCP/SDP suites with planted
  # self-dual solutions; prints per-case timing and a thread-scaling table,
  # optionally writes JSON.  Doubles as a smoke test (exits non-zero unless
  # every run reports status "optimal").
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/benchmark/conic_benchmark.cpp")
    add_executable(conic_benchmark benchmark/conic_benchmark.cpp)
    set_target_properties(conic_benchmark PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    target_link_libraries(conic_benchmark PRIVATE mipsolvers)
    target_include_directories(conic_benchmark PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
    if(NOT MSVC)
      target_compile_options(conic_benchmark PRIVATE -Wall -Wextra -Wpedantic
        $<$<CONFIG:Release>:-O3 -DNDEBUG>
        $<$<CONFIG:RelWithDebInfo>:-O2 -g -DNDEBUG>)
    endif()
    # The OpenMP guard macro is PRIVATE to the library; expose it to the
    # benchmark (mirrors test_conic_ipm above) so it can pick an
    # OpenMP-aware default --threads-list.  Thread control itself goes
    # through ConicIPMOptions::num_threads.
    if(OpenMP_CXX_FOUND)
      target_compile_definitions(conic_benchmark PRIVATE MIPSOLVERS_USE_OPENMP=1)
    endif()
  endif()

  # SCUC module test requires scuc.cpp (not in base mipsolvers lib)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_scuc_module.cpp" AND MIPSOLVERS_BUILD_SCUC)
    add_executable(test_scuc_module
      tests/test_scuc_module.cpp
      src/scuc/scuc.cpp
    )
    set_target_properties(test_scuc_module PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests")
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
    set_target_properties(test_market_simulation PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests")
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
# In prebuilt third-party mode the vendored dependencies are IMPORTED targets:
# they cannot be installed/re-exported, and install(EXPORT) would reject the
# imported targets referenced by the mipsolvers link interface.  Installable
# packages are produced by the normal in-tree build (or by a
# MIPSOLVERS_THIRD_PARTY_ONLY build for the third-party package alone).
if(MIPSOLVERS_THIRD_PARTY_PREBUILT)
  message(STATUS
    "mipsolvers: prebuilt third-party mode — install/export rules skipped "
    "(use an in-tree build to produce installable packages)")
  return()
endif()
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
if(TARGET mipsolvers_hfactor)
  install(TARGETS mipsolvers_hfactor
    EXPORT mipsolversTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()
if(MIPSOLVERS_HAVE_CHOLMOD)
  install(TARGETS cholmod_vendored
    EXPORT mipsolversTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()
if(TARGET umfpack_vendored)
  install(TARGETS umfpack_vendored
    EXPORT mipsolversTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()
if(TARGET klu_vendored)
  install(TARGETS klu_vendored
    EXPORT mipsolversTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()
if(MIPSOLVERS_FMT_VENDORED)
  # Vendored fmt is exported with the package so consumers need no system fmt.
  # Public mipsolvers headers include <fmt/...>, so ship the headers too
  # (matches the fmt target's INSTALL_INTERFACE include dir set in
  # cmake/Dependencies.cmake).
  install(TARGETS fmt
    EXPORT mipsolversTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/third_party/fmt/include/fmt"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
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
