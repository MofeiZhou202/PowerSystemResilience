# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/tianyangzhao/Codes/MIPSolvers/_deps/mumps_upstream-src")
  file(MAKE_DIRECTORY "/Users/tianyangzhao/Codes/MIPSolvers/_deps/mumps_upstream-src")
endif()
file(MAKE_DIRECTORY
  "/Users/tianyangzhao/Codes/MIPSolvers/_deps/mumps_upstream-build"
  "/Users/tianyangzhao/Codes/MIPSolvers/_deps/mumps_upstream-subbuild/mumps_upstream-populate-prefix"
  "/Users/tianyangzhao/Codes/MIPSolvers/_deps/mumps_upstream-subbuild/mumps_upstream-populate-prefix/tmp"
  "/Users/tianyangzhao/Codes/MIPSolvers/_deps/mumps_upstream-subbuild/mumps_upstream-populate-prefix/src/mumps_upstream-populate-stamp"
  "/Users/tianyangzhao/Codes/MIPSolvers/_deps/mumps_upstream-subbuild/mumps_upstream-populate-prefix/src"
  "/Users/tianyangzhao/Codes/MIPSolvers/_deps/mumps_upstream-subbuild/mumps_upstream-populate-prefix/src/mumps_upstream-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/tianyangzhao/Codes/MIPSolvers/_deps/mumps_upstream-subbuild/mumps_upstream-populate-prefix/src/mumps_upstream-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/tianyangzhao/Codes/MIPSolvers/_deps/mumps_upstream-subbuild/mumps_upstream-populate-prefix/src/mumps_upstream-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
