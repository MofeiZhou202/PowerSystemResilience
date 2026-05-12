# cmake/Dependencies.cmake
#
# Resolves all external dependencies for hacdcdss.
#
# Primary dependency: MIPSolvers (local sibling directory ../MIPSolvers).
# MIPSolvers transitively provides Eigen3, fmt, nlohmann_json, HiGHS, and
# optional Ipopt/SCIP — all compiled from local vendored sources.
# No Homebrew or system-installed solver library is required or searched.

# ── MIPSolvers ────────────────────────────────────────────────────────────────
set(_HACDCDSS_MIPSOLVERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../MIPSolvers")

if(NOT EXISTS "${_HACDCDSS_MIPSOLVERS_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "MIPSolvers source not found at ${_HACDCDSS_MIPSOLVERS_DIR}.\n"
    "Clone or symlink the MIPSolvers repository alongside this project so that\n"
    "  ${_HACDCDSS_MIPSOLVERS_DIR}/CMakeLists.txt  exists.")
endif()

# Disable MIPSolvers' own tests and Python bindings when building as a
# sub-project so that only the mipsolvers static library target is compiled.
set(MIPSOLVERS_BUILD_TESTS OFF CACHE BOOL
    "Disable MIPSolvers tests when used as dependency" FORCE)

if(NOT TARGET mipsolvers::mipsolvers)
  add_subdirectory(
    "${_HACDCDSS_MIPSOLVERS_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/_deps/mipsolvers_build"
    EXCLUDE_FROM_ALL)
endif()

if(NOT TARGET mipsolvers::mipsolvers)
  message(FATAL_ERROR
    "add_subdirectory(MIPSolvers) did not produce the mipsolvers::mipsolvers "
    "target.  Check the MIPSolvers CMakeLists.txt.")
endif()

# ── Catch2 (test framework – fetched only when tests are enabled) ─────────────
if(HACDCDSS_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.4
    EXCLUDE_FROM_ALL)
  FetchContent_MakeAvailable(Catch2)
  list(APPEND CMAKE_MODULE_PATH "${catch2_SOURCE_DIR}/extras")
  include(CTest)
  include(Catch)
endif()
