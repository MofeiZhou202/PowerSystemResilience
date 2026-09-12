# cmake/BuildHiGHS.cmake
# Compiles HiGHS LP/MIP solver from the local highs/ source directory
# (which contains only the src/ content of a full HiGHS 1.15.1 checkout,
# plus the MIPSolvers HACDCPF customizations rebased onto 1.15.1).
#
# After inclusion, the following CMake targets exist:
#   highs         — the HiGHS static library (FAST_BUILD path)
#   highs::highs  — ALIAS (or Highs::highs, depending on PROJECT_NAMESPACE)
#
# This file must be included BEFORE any target in the main build uses HiGHS.

# Required CMake modules used by highs/CMakeLists.txt
include(CheckCXXCompilerFlag)
include(CheckCXXSourceCompiles)
include(GenerateExportHeader)

set(_HIGHS_SRC "${CMAKE_CURRENT_SOURCE_DIR}/highs")
set(_HIGHS_BIN "${CMAKE_CURRENT_BINARY_DIR}/_deps/embedded_highs")
file(MAKE_DIRECTORY "${_HIGHS_BIN}")

# ── 1. Version ────────────────────────────────────────────────────────────────
set(HIGHS_VERSION_MAJOR 1)
set(HIGHS_VERSION_MINOR 15)
set(HIGHS_VERSION_PATCH 1)
set(HIGHS_VERSION "${HIGHS_VERSION_MAJOR}.${HIGHS_VERSION_MINOR}.${HIGHS_VERSION_PATCH}")

# ── 2. Variables consumed by highs/CMakeLists.txt ────────────────────────────
set(FAST_BUILD ON)
set(HIGHS_BINARY_DIR "${_HIGHS_BIN}")
set(PROJECT_NAMESPACE "highs")   # creates alias highs::highs
set(BUILD_SHARED_LIBS OFF)

# Feature flags
set(ZLIB OFF)
set(ZLIB_FOUND FALSE)
set(HIPO OFF)
# 64-bit HighsInt across the whole embedded HiGHS build — must match the
# standalone HFactor configuration (HIGHSINT64) since the two share HVectorBase/
# HighsSparseMatrix template sources; mixing int32/int64 ABIs is an ODR trap.
set(HIGHSINT64 ON)
set(CUPDLP_CPU ON)
set(CUPDLP_GPU OFF)
set(CUPDLP_FIND_CUDA OFF)
set(BUILD_CXX ON)
set(BUILD_TESTING OFF)   # don't pull in HiGHS test targets

# ── 3. Generate HConfig.h from template ──────────────────────────────────────
set(GITHASH "embedded_1.15.1")
set(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}")  # pass through
set(HIGHS_HAVE_BUILTIN_CLZ OFF)
set(HIGHS_HAVE_BITSCAN_REVERSE OFF)
if(MSVC)
  set(HIGHS_HAVE_BITSCAN_REVERSE ON)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
  set(HIGHS_HAVE_BUILTIN_CLZ ON)
endif()
configure_file("${_HIGHS_SRC}/HConfig.h.in" "${_HIGHS_BIN}/HConfig.h" @ONLY)

# ── 4. CMAKE_MODULE_PATH: let include(sources) find highs/sources.cmake ──────
list(PREPEND CMAKE_MODULE_PATH "${_HIGHS_SRC}")

# ── 5. Suppress install() rules from the subdirectory ────────────────────────
# We use EXCLUDE_FROM_ALL so install targets aren't added automatically.

# ── 6. Add the HiGHS src/ as a subdirectory ──────────────────────────────────
# HiGHS 1.15 links `highs PUBLIC highs_extras` (highs/CMakeLists.txt, the
# BUILD_SHARED_EXTRAS_LIB=OFF branch).  The embedded build compiles the
# extern/HighsExtras* sources directly into the highs target (see
# highs/sources.cmake), so highs_extras is provided as an empty INTERFACE
# target to satisfy that reference.
add_library(highs_extras INTERFACE)

# Temporarily hide the parent PROJECT_SOURCE_DIR so highs/CMakeLists.txt
# paths like ${PROJECT_SOURCE_DIR}/highs/... still work.  They resolve to
# MIPSolvers_root/highs which is exactly our _HIGHS_SRC.
add_subdirectory("${_HIGHS_SRC}" "${_HIGHS_BIN}" EXCLUDE_FROM_ALL)

# ── 7. Post-setup: expose public headers ─────────────────────────────────────
# The FAST_BUILD path in highs/CMakeLists.txt already adds PUBLIC includes
# via target_include_directories with $<INSTALL_INTERFACE:...> and
# $<BUILD_INTERFACE:...> generator expressions.  We add the generated
# HConfig.h location as a PUBLIC include.
target_include_directories(highs PUBLIC
  $<BUILD_INTERFACE:${_HIGHS_BIN}>
  $<BUILD_INTERFACE:${_HIGHS_SRC}>)

# HiGHS 1.15 extern/HighsExtrasApi.cpp expects HIGHS_EXTRAS_VERSION, which
# upstream defines on its separate highs_extras target (extern/CMakeLists.txt
# line 69).  In the embedded build the file is compiled into the highs target,
# so the definition is attached to the source file instead.
set_source_files_properties("${_HIGHS_SRC}/extern/HighsExtrasApi.cpp"
  TARGET_DIRECTORY highs
  PROPERTIES COMPILE_DEFINITIONS "HIGHS_EXTRAS_VERSION=\"${HIGHS_VERSION}\"")

# Create the expected alias if not already created by the subdirectory
if(NOT TARGET highs::highs)
  add_library(highs::highs ALIAS highs)
endif()

message(STATUS "mipsolvers: building embedded HiGHS ${HIGHS_VERSION} from ${_HIGHS_SRC}")
