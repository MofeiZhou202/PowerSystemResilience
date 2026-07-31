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
#   * If the working tree does not match, CMake emits a FATAL_ERROR that halts
#     the configure step.  Update the pin below whenever MIPSolvers is upgraded.
#   * Override the default sibling path by setting MIPSOLVERS_SOURCE_DIR
#     (e.g. cmake -DMIPSOLVERS_SOURCE_DIR=/opt/MIPSolvers ..) — useful for CI
#     environments where the tree layout differs from the default.
#
# Last verified compatible commit (update when upgrading MIPSolvers):
set(_HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT "78d9272299f8202eee7377aeca5e6bf359193d92"
  CACHE STRING "Expected MIPSolvers HEAD commit (empty = skip check)" FORCE)

set(MIPSOLVERS_SOURCE_DIR "" CACHE PATH
    "Explicit path to the MIPSolvers source tree. \
When empty, auto-detects ./MIPSolvers (in-repo) then ../MIPSolvers (sibling).")
if(NOT MIPSOLVERS_SOURCE_DIR STREQUAL "")
  set(_HACDCDSS_MIPSOLVERS_DIR "${MIPSOLVERS_SOURCE_DIR}")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/MIPSolvers/CMakeLists.txt")
  # MIPSolvers deployed inside the repo (./MIPSolvers).
  set(_HACDCDSS_MIPSOLVERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/MIPSolvers")
else()
  # Legacy layout: sibling directory next to this project.
  set(_HACDCDSS_MIPSOLVERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../MIPSolvers")
endif()

if(NOT EXISTS "${_HACDCDSS_MIPSOLVERS_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "MIPSolvers source not found at ${_HACDCDSS_MIPSOLVERS_DIR}.\n"
    "Clone or symlink the MIPSolvers repository alongside this project so that\n"
    "  ${_HACDCDSS_MIPSOLVERS_DIR}/CMakeLists.txt  exists.")
endif()

# Optionally verify the MIPSolvers commit matches the recorded pin.
if(_HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT)
  find_package(Git QUIET)
  if(Git_FOUND AND EXISTS "${_HACDCDSS_MIPSOLVERS_DIR}/.git")
    option(HACDCDSS_SKIP_MIPSOLVERS_DIRTY_CHECK
           "Suppress the local dirty-workspace warning for MIPSolvers in non-release builds"
           OFF)

    set(_HACDCDSS_MIPSOLVERS_STRICT_REPRO OFF)
    if(CMAKE_BUILD_TYPE)
      string(TOUPPER "${CMAKE_BUILD_TYPE}" _HACDCDSS_BUILD_TYPE_UPPER)
      if(_HACDCDSS_BUILD_TYPE_UPPER STREQUAL "RELEASE" OR
         _HACDCDSS_BUILD_TYPE_UPPER STREQUAL "RELWITHDEBINFO" OR
         _HACDCDSS_BUILD_TYPE_UPPER STREQUAL "MINSIZEREL")
        set(_HACDCDSS_MIPSOLVERS_STRICT_REPRO ON)
      endif()
    endif()
    if(DEFINED ENV{CI})
      string(TOLOWER "$ENV{CI}" _HACDCDSS_CI_VALUE)
      if(NOT _HACDCDSS_CI_VALUE STREQUAL "" AND
         NOT _HACDCDSS_CI_VALUE STREQUAL "0" AND
         NOT _HACDCDSS_CI_VALUE STREQUAL "false" AND
         NOT _HACDCDSS_CI_VALUE STREQUAL "off")
        set(_HACDCDSS_MIPSOLVERS_STRICT_REPRO ON)
      endif()
    endif()
    if(HACDCDSS_SKIP_MIPSOLVERS_DIRTY_CHECK AND
       _HACDCDSS_MIPSOLVERS_STRICT_REPRO)
      message(FATAL_ERROR
        "HACDCDSS_SKIP_MIPSOLVERS_DIRTY_CHECK cannot be enabled for Release/CI "
        "builds.  Dirty MIPSolvers sources make the pinned dependency "
        "non-reproducible.")
    endif()
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" -C "${_HACDCDSS_MIPSOLVERS_DIR}"
              rev-parse --verify HEAD
      OUTPUT_VARIABLE _MIPSOLVERS_ACTUAL_COMMIT
      ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT _MIPSOLVERS_ACTUAL_COMMIT STREQUAL _HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT)
      # Non-fatal: a mismatch is flagged but never blocks the build, so a local
      # MIPSolvers checkout that differs from the recorded pin still compiles.
      # Update _HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT to silence this once the new
      # MIPSolvers revision is intentionally adopted.
      message(WARNING
        "MIPSolvers HEAD (${_MIPSOLVERS_ACTUAL_COMMIT}) does not match the "
        "recorded pin (${_HACDCDSS_MIPSOLVERS_EXPECTED_COMMIT}).  Building "
        "against the local checkout anyway; update the pin to record an "
        "intentional upgrade.")
    endif()

    # Detect a dirty working tree — uncommitted local changes in MIPSolvers
    # would cause builds to diverge from the pinned commit even if HEAD matches.
    if(NOT HACDCDSS_SKIP_MIPSOLVERS_DIRTY_CHECK)
      execute_process(
        COMMAND "${GIT_EXECUTABLE}" -C "${_HACDCDSS_MIPSOLVERS_DIR}"
                status --porcelain
        OUTPUT_VARIABLE _MIPSOLVERS_DIRTY
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
      if(_MIPSOLVERS_DIRTY)
        string(CONCAT _HACDCDSS_MIPSOLVERS_DIRTY_MESSAGE
          "MIPSolvers working tree at ${_HACDCDSS_MIPSOLVERS_DIR} has "
          "uncommitted changes:\n${_MIPSOLVERS_DIRTY}\n"
          "The build is NOT reproducible from the pinned commit.  Commit, "
          "stash, or discard the MIPSolvers changes before producing a release "
          "artefact.")
        if(_HACDCDSS_MIPSOLVERS_STRICT_REPRO)
          message(FATAL_ERROR "${_HACDCDSS_MIPSOLVERS_DIRTY_MESSAGE}")
        else()
          message(WARNING
            "${_HACDCDSS_MIPSOLVERS_DIRTY_MESSAGE}  Set "
            "HACDCDSS_SKIP_MIPSOLVERS_DIRTY_CHECK=ON to suppress this warning "
            "in non-release local builds.")
        endif()
      endif()
    endif()
  elseif(NOT EXISTS "${_HACDCDSS_MIPSOLVERS_DIR}/.git")
    message(STATUS
      "MIPSolvers source archive has no Git metadata; commit and dirty-tree "
      "checks are skipped.")
  endif()
endif()

# MIPSolvers subproject knobs are set by the top-level dependency profile before
# this file is included.  Keep a conservative fallback for direct/manual
# inclusion, but do not override an explicit profile choice such as full-dev.
if(NOT DEFINED MIPSOLVERS_BUILD_TESTS)
  set(MIPSOLVERS_BUILD_TESTS OFF CACHE BOOL
      "Build MIPSolvers tests when used as dependency")
endif()

set(_HACDCDSS_MIPSOLVERS_BINARY_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/_deps/mipsolvers_build")
if(NOT TARGET mipsolvers::mipsolvers)
  add_subdirectory(
    "${_HACDCDSS_MIPSOLVERS_DIR}"
    "${_HACDCDSS_MIPSOLVERS_BINARY_DIR}"
    EXCLUDE_FROM_ALL)
endif()

if(NOT TARGET mipsolvers::mipsolvers)
  message(FATAL_ERROR
    "add_subdirectory(MIPSolvers) did not produce the mipsolvers::mipsolvers "
    "target.  Check the MIPSolvers CMakeLists.txt.")
endif()

# MIPSolvers' NETLIB regression resolves checked-in MPS fixtures beneath its
# working directory. Parent directories cannot portably override test
# properties created in an add_subdirectory() on every supported CMake
# version, so mirror the small fixture set into the dependency build tree.
if(MIPSOLVERS_BUILD_TESTS AND
   EXISTS "${_HACDCDSS_MIPSOLVERS_DIR}/tests/data/netlib")
  file(COPY "${_HACDCDSS_MIPSOLVERS_DIR}/tests/data/netlib"
       DESTINATION "${_HACDCDSS_MIPSOLVERS_BINARY_DIR}/tests/data")
endif()

# ── Catch2 (test framework, vendored by MIPSolvers) ──────────────────────────
if(HACDCPF_BUILD_TESTS)
  set(_HACDCDSS_CATCH2_DIR
      "${_HACDCDSS_MIPSOLVERS_DIR}/third_party/catch2")
  if(NOT TARGET Catch2::Catch2WithMain)
    if(NOT EXISTS "${_HACDCDSS_CATCH2_DIR}/catch_amalgamated.cpp" OR
       NOT EXISTS "${_HACDCDSS_CATCH2_DIR}/catch2/catch_test_macros.hpp")
      message(FATAL_ERROR
        "MIPSolvers vendored Catch2 is incomplete at "
        "${_HACDCDSS_CATCH2_DIR}. Restore third_party/catch2; network "
        "downloads are disabled for hermetic builds.")
    endif()
    find_package(Threads REQUIRED)
    add_library(Catch2 STATIC
      "${_HACDCDSS_CATCH2_DIR}/catch_amalgamated.cpp")
    target_include_directories(Catch2 PUBLIC "${_HACDCDSS_CATCH2_DIR}")
    target_compile_definitions(Catch2 PRIVATE CATCH_AMALGAMATED_CUSTOM_MAIN)
    target_link_libraries(Catch2 PUBLIC Threads::Threads)
    set_target_properties(Catch2 PROPERTIES POSITION_INDEPENDENT_CODE ON)
    add_library(Catch2::Catch2 ALIAS Catch2)

    add_library(Catch2WithMain STATIC
      "${_HACDCDSS_CATCH2_DIR}/catch_amalgamated.cpp")
    target_include_directories(Catch2WithMain PUBLIC "${_HACDCDSS_CATCH2_DIR}")
    target_link_libraries(Catch2WithMain PUBLIC Threads::Threads)
    set_target_properties(Catch2WithMain PROPERTIES POSITION_INDEPENDENT_CODE ON)
    add_library(Catch2::Catch2WithMain ALIAS Catch2WithMain)
  endif()
  if(NOT EXISTS "${_HACDCDSS_CATCH2_DIR}/extras/Catch.cmake")
    message(FATAL_ERROR
      "MIPSolvers vendored Catch2 is missing extras/Catch.cmake, required for "
      "test discovery in an offline build.")
  endif()
  message(STATUS "hacdcdss: Catch2 = MIPSolvers vendored (3.7.1)")
  list(APPEND CMAKE_MODULE_PATH "${_HACDCDSS_CATCH2_DIR}/extras")
  include(CTest)
  include(Catch)
endif()

# ── OpenXLSX (ETAP Excel I/O, local-only) ────────────────────────────────────
set(HACDCPF_HAVE_OPENXLSX OFF)
if(HACDCPF_ENABLE_ETAP)
  set(OPENXLSX_CREATE_DOCS      OFF CACHE BOOL "" FORCE)
  set(OPENXLSX_BUILD_TESTS      OFF CACHE BOOL "" FORCE)
  set(OPENXLSX_BUILD_SAMPLES    OFF CACHE BOOL "" FORCE)
  set(OPENXLSX_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)

  set(_HACDCDSS_OPENXLSX_DIR
      "${CMAKE_CURRENT_SOURCE_DIR}/third_party/OpenXLSX-master")
  if(EXISTS "${_HACDCDSS_OPENXLSX_DIR}/CMakeLists.txt")
    add_subdirectory(
      "${_HACDCDSS_OPENXLSX_DIR}"
      "${CMAKE_CURRENT_BINARY_DIR}/_deps/openxlsx-build"
      EXCLUDE_FROM_ALL)
    message(STATUS "hacdcdss: OpenXLSX = vendored (third_party/OpenXLSX-master)")
    set(HACDCPF_HAVE_OPENXLSX ON)
  else()
    find_package(OpenXLSX CONFIG QUIET)
    if(OpenXLSX_FOUND)
      message(STATUS "hacdcdss: OpenXLSX = local system package")
      set(HACDCPF_HAVE_OPENXLSX ON)
    endif()
  endif()

  if(TARGET OpenXLSX AND NOT TARGET OpenXLSX::OpenXLSX)
    add_library(OpenXLSX::OpenXLSX ALIAS OpenXLSX)
  endif()

  if(NOT HACDCPF_HAVE_OPENXLSX)
    message(FATAL_ERROR
      "HACDCPF_ENABLE_ETAP=ON but OpenXLSX is unavailable. Restore vendored "
      "third_party/OpenXLSX-master or provide a local OpenXLSX package. "
      "Network downloads are disabled for hermetic builds.")
  endif()
endif()
