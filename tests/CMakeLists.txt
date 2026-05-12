cmake_minimum_required(VERSION 3.20)

function(hacdcdss_add_test target)
  add_executable(${target} ${ARGN})
  target_link_libraries(${target} PRIVATE hacdcdss::hacdcdss Catch2::Catch2WithMain)
  catch_discover_tests(${target})
endfunction()

hacdcdss_add_test(test_hacdcdss test_hacdcdss.cpp)
