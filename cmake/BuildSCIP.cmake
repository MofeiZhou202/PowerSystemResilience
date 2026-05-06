# cmake/BuildSCIP.cmake
# Sets up the embedded SCIP build from scip/ source directory.
# Include this from cmake/Dependencies.cmake BEFORE calling add_subdirectory(scip).
#
# After inclusion, the following CMake targets will exist:
#   libscip  / SCIP::libscip   — the SCIP static library

# ── Source and binary directories (defined first, used throughout) ─────────────
set(_SCIP_SRC "${CMAKE_CURRENT_SOURCE_DIR}/scip")
set(_SCIP_BIN "${CMAKE_CURRENT_BINARY_DIR}/_deps/embedded_scip")
file(MAKE_DIRECTORY "${_SCIP_BIN}/scip")

# ── SCIP version ─────────────────────────────────────────────────────────────
set(SCIP_VERSION_MAJOR 9)
set(SCIP_VERSION_MINOR 0)
set(SCIP_VERSION_PATCH 0)
set(SCIP_VERSION_API  102)

# ── Required flags ────────────────────────────────────────────────────────────
set(SHARED OFF)
set(LTO_AVAILABLE OFF)
set(CXXONLY OFF)
set(SCIP_EMBEDDED_BUILD ON)  # suppress install/export rules for embedded build

# Optional features all OFF for minimal embedded build
set(SCIP_WITH_ZLIB    OFF)
set(SCIP_WITH_GMP     OFF)
set(SCIP_WITH_MPFR    OFF)
set(SCIP_WITH_READLINE OFF)
set(SCIP_WITH_ZIMPL   OFF)
set(SCIP_WITH_AMPL    OFF)
set(SCIP_WITH_EXACTSOLVE OFF)
set(SCIP_WITH_PAPILO  OFF)
set(SCIP_WITH_LAPACK  OFF)
set(SCIP_WITH_BOOST   OFF)
set(SOPLEX_FOUND      OFF)
set(ZIMPL_FOUND       OFF)

# ── LP interface: use HiGHS when available, otherwise none ────────────────────
if(MIPSOLVERS_HAVE_HIGHS_LIB)
  set(lpi lpi/lpi_highs.cpp)
  set(LPS_LIBRARIES highs::highs)
  set(LPS_PIC_LIBRARIES highs::highs)
  set(SCIP_WITH_LPS "highs")
else()
  set(lpi lpi/lpi_none.c)
  set(LPS_LIBRARIES "")
  set(LPS_PIC_LIBRARIES "")
  set(SCIP_WITH_LPS "none")
endif()

# ── Exact LP / lpiexact interface ─────────────────────────────────────────────
# Use the "none" implementation that provides stub symbols for all SCIPlpiExact* calls
set(lpiexact "lpiexact/lpiexact_none.c")

# ── Threading: none ───────────────────────────────────────────────────────────
set(tpisources
  tpi/tpi_tnycthrd.c
  tinycthread/tinycthread.c)
set(THREAD_LIBRARIES "")
set(TPI_NONE ON)

# ── Symmetry: no external (dejavu / bliss / nauty bundled in scip/) ───────────
set(sym "")
set(SYM_LIBRARIES "")
set(NLPI_LIBRARIES "")
set(GMP_LIBRARIES "")
set(MPFR_LIBRARIES "")
set(LAPACK_LIBRARIES "")
set(PAPILO_IMPORTED_TARGETS "")
set(ZIMPL_LIBRARIES "")
set(ZIMPL_PIC_LIBRARIES "")
set(Readline_LIBRARY "")

# ── Generate scip/config.h in binary dir ─────────────────────────────────────
# Build type
if(NOT CMAKE_BUILD_TYPE)
  set(_SCIP_BUILD_TYPE "Release")
else()
  set(_SCIP_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
endif()

# Generate scip/config.h
configure_file(
  "${_SCIP_SRC}/scip/config.h.in"
  "${_SCIP_BIN}/scip/config.h"
  @ONLY)

# ── scip_update_githash: create a static githash.c in the build dir ───────────
set(_SCIP_GITHASH_C "${_SCIP_BIN}/scip/githash.c")
if(NOT EXISTS "${_SCIP_GITHASH_C}")
  file(WRITE "${_SCIP_GITHASH_C}"
    "/* auto-generated: no git history */\n"
    "#define SCIP_GITHASH \"embedded\"\n")
endif()
add_custom_target(scip_update_githash)  # empty target (no git)

# ── Add scip as a subdirectory ────────────────────────────────────────────────
set(PROJECT_BINARY_DIR "${_SCIP_BIN}")

# Tell scip/CMakeLists.txt where to find its own include dirs relative to
# the binary dir (scip/config.h lives in _SCIP_BIN).
add_subdirectory("${_SCIP_SRC}" "${_SCIP_BIN}" EXCLUDE_FROM_ALL)

# Restore PROJECT_BINARY_DIR to the true top-level build dir
set(PROJECT_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")

# ── Add missing optional-solver stub files to libscip ────────────────────────
# scipsources in scip/CMakeLists.txt doesn't include these; scipdefplugins.c
# calls them unconditionally, so we must provide stub/none implementations.
if(TARGET libscip)
  target_sources(libscip PRIVATE
    # Expression interpreter (mandatory; "none" provides stubs)
    "${_SCIP_SRC}/scip/exprinterpret_none.c"
    # NLP solver stubs (call returns SCIP_PLUGINNOTFOUND)
    "${_SCIP_SRC}/scip/nlpi_ipopt_dummy.c"
    "${_SCIP_SRC}/scip/nlpi_conopt_dummy.c"
    "${_SCIP_SRC}/scip/nlpi_filtersqp_dummy.c"
    "${_SCIP_SRC}/scip/nlpi_worhp_dummy.c"
    # Symmetry stub (no external symmetry library)
    "${_SCIP_SRC}/symmetry/compute_symmetry_none.cpp")

  # Include the lpiexact headers directory
  target_include_directories(libscip PUBLIC
    $<BUILD_INTERFACE:${_SCIP_BIN}>
    $<BUILD_INTERFACE:${_SCIP_SRC}/lpiexact>
    $<BUILD_INTERFACE:${_SCIP_SRC}/symmetry>)
endif()
