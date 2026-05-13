# CheckAtomic.cmake
# Check whether the compiler provides lock-free 64-bit atomics or needs -latomic.
# Sets HAVE_CXX_ATOMICS64_WITH_LIB if the extra library is required.

include(CheckCXXSourceCompiles)

function(check_working_cxx_atomics64 varname)
  set(OLD_CMAKE_REQUIRED_FLAGS ${CMAKE_REQUIRED_FLAGS})
  set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} -std=c++11")
  check_cxx_source_compiles("
#include <atomic>
#include <cstdint>
std::atomic<uint64_t> x(0);
int main() {
  uint64_t i = x.load(std::memory_order_relaxed);
  (void)i;
  return 0;
}
" ${varname})
  set(CMAKE_REQUIRED_FLAGS ${OLD_CMAKE_REQUIRED_FLAGS})
endfunction()

function(check_working_cxx_atomics64_with_lib varname)
  set(OLD_CMAKE_REQUIRED_FLAGS ${CMAKE_REQUIRED_FLAGS})
  set(OLD_CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES})
  set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} -std=c++11")
  set(CMAKE_REQUIRED_LIBRARIES "${CMAKE_REQUIRED_LIBRARIES} atomic")
  check_cxx_source_compiles("
#include <atomic>
#include <cstdint>
std::atomic<uint64_t> x(0);
int main() {
  uint64_t i = x.load(std::memory_order_relaxed);
  (void)i;
  return 0;
}
" ${varname})
  set(CMAKE_REQUIRED_FLAGS ${OLD_CMAKE_REQUIRED_FLAGS})
  set(CMAKE_REQUIRED_LIBRARIES ${OLD_CMAKE_REQUIRED_LIBRARIES})
endfunction()

check_working_cxx_atomics64(HAVE_CXX_ATOMICS64_WITHOUT_LIB)
if(NOT HAVE_CXX_ATOMICS64_WITHOUT_LIB)
  check_working_cxx_atomics64_with_lib(HAVE_CXX_ATOMICS64_WITH_LIB)
endif()
