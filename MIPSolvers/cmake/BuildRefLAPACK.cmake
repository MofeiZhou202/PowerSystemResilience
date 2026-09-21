# cmake/BuildRefLAPACK.cmake
# Builds the vendored reference BLAS+LAPACK 3.12.1 (third_party/lapack) as a
# single static PIC library.  Offline fallback used when no system BLAS/LAPACK
# is available — see MIPSOLVERS_USE_VENDORED_BLAS / MIPSOLVERS_FORCE_VENDORED_BLAS
# in cmake/Dependencies.cmake.
#
# Targets created:
#   reflapack_vendored — static library: reference BLAS + full LAPACK

set(_RL "${CMAKE_CURRENT_SOURCE_DIR}/third_party/lapack")
if(NOT EXISTS "${_RL}/BLAS/SRC/dgemm.f" OR NOT EXISTS "${_RL}/SRC/dpotrf.f")
  message(FATAL_ERROR
    "vendored reference LAPACK not found at third_party/lapack (expected "
    "third_party/lapack/BLAS/SRC/dgemm.f and third_party/lapack/SRC/dpotrf.f) "
    "— restore it from the repository (see third_party/lapack/README.local).")
endif()

# Fortran compiler detection shared with the vendored MUMPS build.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/DetectFortranCompiler.cmake")

# ── Sources ───────────────────────────────────────────────────────────────────
# Reference BLAS.  BLAS/SRC/xerbla.f is excluded: SRC/xerbla.f provides the
# same symbol (one definition per static archive).
file(GLOB _RL_BLAS_SOURCES "${_RL}/BLAS/SRC/*.f")
list(REMOVE_ITEM _RL_BLAS_SOURCES "${_RL}/BLAS/SRC/xerbla.f")
# Full LAPACK (all precisions).  The *.f90 files are required: in 3.12 the
# *lartg/*lassq/*gedmd routines exist only as F90 sources and use the
# la_constants / la_xisnan modules — CMake's Fortran module dependency
# scanner orders the compilation automatically (same as the MUMPS build).
file(GLOB _RL_LAPACK_SOURCES
  "${_RL}/SRC/*.f"
  "${_RL}/SRC/*.F"
  "${_RL}/SRC/*.f90"
  "${_RL}/SRC/*.F90")

add_library(reflapack_vendored STATIC ${_RL_BLAS_SOURCES} ${_RL_LAPACK_SOURCES})

set_target_properties(reflapack_vendored PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  Fortran_MODULE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/_reflapack_mods")

# Reference BLAS/LAPACK is legacy F77/F90 code: not warning-clean, and some
# routines trip GCC>=10 argument-type checking (same flags as the MUMPS build).
target_compile_options(reflapack_vendored PRIVATE
  $<$<OR:$<Fortran_COMPILER_ID:GNU>,$<Fortran_COMPILER_ID:LLVMFlang>>:
    -w
    -fno-strict-aliasing
    $<$<VERSION_GREATER_EQUAL:${CMAKE_Fortran_COMPILER_VERSION},10>:
      -fallow-argument-mismatch
      -fallow-invalid-boz
    >
  >
)

# Link the Fortran runtime the same way the vendored MUMPS build does (static
# archives on macOS, implicit runtime elsewhere), so a CXX-only final link
# resolves Fortran runtime symbols.
mipsolvers_link_fortran_runtime(reflapack_vendored PUBLIC)

# Exported with ${MIPSOLVERS_THIRD_PARTY_EXPORT_SET} (mipsolversTargets in a
# normal build): dmumps/cholmod_vendored/ipopt_local link this target when the
# vendored BLAS fallback is active.
include(GNUInstallDirs)
if(NOT DEFINED MIPSOLVERS_THIRD_PARTY_EXPORT_SET)
  set(MIPSOLVERS_THIRD_PARTY_EXPORT_SET mipsolversTargets)
endif()
install(TARGETS reflapack_vendored
  EXPORT  ${MIPSOLVERS_THIRD_PARTY_EXPORT_SET}
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")

message(STATUS
  "mipsolvers: building vendored reference BLAS+LAPACK 3.12.1 from third_party/lapack")
