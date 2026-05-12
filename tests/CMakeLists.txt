cmake_minimum_required(VERSION 3.20)

function(hacdcpf_add_test target)
  add_executable(${target} ${ARGN})
  target_link_libraries(${target} PRIVATE hacdcpf::hacdcpf Catch2::Catch2WithMain)
  catch_discover_tests(${target})
endfunction()

hacdcpf_add_test(test_hacdcpf test_hacdcpf.cpp)
