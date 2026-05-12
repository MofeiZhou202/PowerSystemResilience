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
