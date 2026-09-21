cmake_minimum_required(VERSION 3.20)
project(MIPSolversConsumerExample LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 用法：cmake -B build -DCMAKE_PREFIX_PATH=/path/to/mipsolvers/install
# 也可以通过 add_subdirectory(path/to/MIPSolvers EXCLUDE_FROM_ALL) 方式集成

find_package(mipsolvers REQUIRED)

add_executable(consumer_example main.cpp)
target_link_libraries(consumer_example PRIVATE mipsolvers::mipsolvers)
mipsolvers_deploy_runtime(consumer_example)

# 可选：开启警告
if(NOT MSVC)
  target_compile_options(consumer_example PRIVATE -Wall -Wextra)
endif()
