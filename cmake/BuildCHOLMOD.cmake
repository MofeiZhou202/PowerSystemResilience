# cmake/BuildCHOLMOD.cmake
# Builds CHOLMOD 5.3.4 (from SuiteSparse 7.12.2) out of the VENDORED sources
# in suitesparse/ — no network access is required at configure/build/deploy
# time.  The sources are compiled directly (same approach as BuildMUMPS.cmake)
# rather than via the upstream umbrella CMake, which is not usable standalone.
#
# Components compiled (all that CHOLMOD needs):
#   SuiteSparse_config, AMD, CAMD, COLAMD, CCOLAMD, and CHOLMOD's
#   Check/Cholesky/Utility(Core)/Supernodal modules — double precision,
#   both int32 (cholmod_*) and int64 (cholmod_l_*) entry points.
#   Excluded: Partition/Modify/MatrixOps (NPARTITION/NMODIFY/NMATRIXOPS),
#   GPU/CUDA, and the Fortran-only AMD helpers.
#
# BLAS/LAPACK (supernodal kernels): Accelerate framework on macOS;
# find_package(BLAS/LAPACK) elsewhere (system OpenBLAS etc. — no download).
#
# Targets/vars created:
#   cholmod_vendored           — static library
#   MIPSOLVERS_HAVE_CHOLMOD    — set ON when the vendored build is enabled
#   MIPSOLVERS_CHOLMOD_INCLUDE_DIRS — public header dirs (cholmod.h)

option(MIPSOLVERS_USE_VENDORED_CHOLMOD
  "Build vendored CHOLMOD from in-tree SuiteSparse sources (offline)" ON)

set(MIPSOLVERS_HAVE_CHOLMOD OFF)
set(MIPSOLVERS_CHOLMOD_INCLUDE_DIRS "")

if(NOT MIPSOLVERS_USE_VENDORED_CHOLMOD)
  message(STATUS "mipsolvers: vendored CHOLMOD disabled (MIPSOLVERS_USE_VENDORED_CHOLMOD=OFF)")
  return()
endif()

set(_SS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/suitesparse")
if(NOT EXISTS "${_SS_ROOT}/CHOLMOD/Include/cholmod.h")
  message(STATUS "mipsolvers: vendored SuiteSparse not found at ${_SS_ROOT}; CHOLMOD disabled")
  return()
endif()

# ── Sources ───────────────────────────────────────────────────────────────────
file(GLOB _SSCONFIG_SOURCES "${_SS_ROOT}/SuiteSparse_config/SuiteSparse_config.c")
file(GLOB _AMD_SOURCES      "${_SS_ROOT}/AMD/Source/amd_*.c")
file(GLOB _CAMD_SOURCES     "${_SS_ROOT}/CAMD/Source/camd_*.c")
file(GLOB _COLAMD_SOURCES   "${_SS_ROOT}/COLAMD/Source/colamd*.c")
file(GLOB _CCOLAMD_SOURCES  "${_SS_ROOT}/CCOLAMD/Source/ccolamd*.c")
file(GLOB _CHOLMOD_SOURCES
  "${_SS_ROOT}/CHOLMOD/Check/cholmod_*.c"
  "${_SS_ROOT}/CHOLMOD/Cholesky/cholmod_*.c"
  "${_SS_ROOT}/CHOLMOD/Utility/cholmod_*.c"
  "${_SS_ROOT}/CHOLMOD/Supernodal/cholmod_*.c")

add_library(cholmod_vendored STATIC
  ${_SSCONFIG_SOURCES}
  ${_AMD_SOURCES}
  ${_CAMD_SOURCES}
  ${_COLAMD_SOURCES}
  ${_CCOLAMD_SOURCES}
  ${_CHOLMOD_SOURCES})

set_target_properties(cholmod_vendored PROPERTIES
  C_STANDARD 11
  C_STANDARD_REQUIRED ON
  POSITION_INDEPENDENT_CODE ON)

target_include_directories(cholmod_vendored PRIVATE
  "${_SS_ROOT}/SuiteSparse_config"
  "${_SS_ROOT}/AMD/Include"
  "${_SS_ROOT}/CAMD/Include"
  "${_SS_ROOT}/COLAMD/Include"
  "${_SS_ROOT}/CCOLAMD/Include"
  "${_SS_ROOT}/CHOLMOD/Include"
  "${_SS_ROOT}/CHOLMOD/Utility")

# Build without METIS (Partition) and without the Modify/MatrixOps modules —
# their sources are not compiled above; these defines keep internal headers
# consistent with that.
target_compile_definitions(cholmod_vendored PRIVATE
  NPARTITION NMODIFY NMATRIXOPS)

# ── BLAS/LAPACK for the supernodal kernels ────────────────────────────────────
if(APPLE)
  target_link_libraries(cholmod_vendored PRIVATE "-framework Accelerate")
else()
  find_package(BLAS QUIET)
  find_package(LAPACK QUIET)
  if(BLAS_FOUND AND LAPACK_FOUND)
    target_link_libraries(cholmod_vendored PRIVATE
      ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
  else()
    # Degrade gracefully to the simplicial method (no BLAS) — CHOLMOD stays
    # usable, just without the supernodal BLAS-3 kernels.
    message(STATUS "mipsolvers: BLAS/LAPACK not found — CHOLMOD supernodal module disabled")
    target_compile_definitions(cholmod_vendored PRIVATE NSUPERNODAL)
  endif()
endif()
if(UNIX AND NOT APPLE)
  target_link_libraries(cholmod_vendored PRIVATE m)
endif()

set(MIPSOLVERS_HAVE_CHOLMOD ON)
set(MIPSOLVERS_CHOLMOD_INCLUDE_DIRS "${_SS_ROOT}/CHOLMOD/Include")
message(STATUS "mipsolvers: building vendored CHOLMOD 5.3.4 (SuiteSparse 7.12.2) from suitesparse/")
