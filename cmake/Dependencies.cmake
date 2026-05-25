# cmake/Dependencies.cmake
#
# Resolves all external dependencies for hacdcdss.
#
# Primary dependency: MIPSolvers (local sibling directory ../MIPSolvers).
# MIPSolvers transitively provides Eigen3, fmt, nlohmann_json, HiGHS, and
# optional Ipopt/SCIP — all compiled from local vendored sources.
# No Homebrew or system-installed solver library is required or searched.

# ── MIPSolvers ────────────────────────────────────────────────────────────────
# MIPSolvers is consumed as a local sibling directory rather than through a
# package manager or network fetch.  To keep the build reproducible:
#
#   * Record the expected commit hash below.  Update it whenever MIPSolvers is
#     intentionally upgraded so reviewers can see the dependency version bump.
#   * If the working tree does not match, CMake emits a WARNING (not an error)
#     so that developers whose local clone is slightly ahead/behind can still
#     build.  CI can be configured to treat the warning as an error.
#
# Last verified compatible commit (update when upgrading MIPSolvers):
set(_HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT ""
    CACHE STRING "Expected MIPSolvers HEAD commit (empty = skip check)")

set(_HACDCDSS_MIPSOLVERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../MIPSolvers")

if(NOT EXISTS "${_HACDCDSS_MIPSOLVERS_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "MIPSolvers source not found at ${_HACDCDSS_MIPSOLVERS_DIR}.\n"
    "Clone or symlink the MIPSolvers repository alongside this project so that\n"
    "  ${_HACDCDSS_MIPSOLVERS_DIR}/CMakeLists.txt  exists.")
endif()

# Optionally verify the MIPSolvers commit matches the recorded pin.
if(_HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT)
  find_package(Git QUIET)
  if(Git_FOUND)
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" -C "${_HACDCDSS_MIPSOLVERS_DIR}"
              rev-parse --verify HEAD
      OUTPUT_VARIABLE _MIPSOLVERS_ACTUAL_COMMIT
      ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT _MIPSOLVERS_ACTUAL_COMMIT STREQUAL _HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT)
      message(WARNING
        "MIPSolvers HEAD (${_MIPSOLVERS_ACTUAL_COMMIT}) does not match the\n"
        "expected pin (${_HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT}).\n"
        "Update _HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT in cmake/Dependencies.cmake\n"
        "if this upgrade is intentional.")
    endif()
  endif()
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
if(HACDCPF_BUILD_TESTS)
  include(FetchContent)

  # Prefer a pre-existing local source tree over a network download to avoid
  # depending on GitHub connectivity.
  set(_CATCH2_LOCAL_CANDIDATES
    "${CMAKE_CURRENT_SOURCE_DIR}/../HybridACDCPowerSystemsPlanning/build_rel/_deps/catch2-src"
    "${CMAKE_CURRENT_SOURCE_DIR}/../HybridACDCPowerSystemsPlanning/build/_deps/catch2-src"
    "${CMAKE_CURRENT_SOURCE_DIR}/../MIPSolvers/build_mipsolvers/_deps/catch2-src"
    "${CMAKE_CURRENT_SOURCE_DIR}/../MIPSolvers/_deps/catch2-src")

  set(_CATCH2_LOCAL_DIR "")
  foreach(_dir IN LISTS _CATCH2_LOCAL_CANDIDATES)
    if(EXISTS "${_dir}/CMakeLists.txt")
      set(_CATCH2_LOCAL_DIR "${_dir}")
      break()
    endif()
  endforeach()

  if(_CATCH2_LOCAL_DIR)
    message(STATUS "hacdcdss: using local Catch2 source at ${_CATCH2_LOCAL_DIR}")
    FetchContent_Declare(
      Catch2
      SOURCE_DIR "${_CATCH2_LOCAL_DIR}"
      EXCLUDE_FROM_ALL)
  else()
    message(STATUS "hacdcdss: downloading Catch2 v3.5.4 from GitHub")
    FetchContent_Declare(
      Catch2
      GIT_REPOSITORY https://github.com/catchorg/Catch2.git
      GIT_TAG        v3.5.4
      EXCLUDE_FROM_ALL)
  endif()

  FetchContent_MakeAvailable(Catch2)
  list(APPEND CMAKE_MODULE_PATH "${catch2_SOURCE_DIR}/extras")
  include(CTest)
  include(Catch)
endif()
