# cmake/BuildMUMPS.cmake
# Builds MUMPS 5.7.3 from source (sequential, double-precision only).
# Uses only the vendored mumps/ source tree; no network fallback exists.
# All CMake build logic is inlined here.
#
# Requirements:
#   Fortran compiler  — gfortran from 'brew install gcc' on macOS CI
#
# Targets created:
#   dmumps        — static library: PORD + mpiseq + mumps_common + dmumps (double)
#   MUMPS::MUMPS  — interface alias for dmumps

include(GNUInstallDirs)

# Export set for the vendored targets installed below: mipsolversTargets in a
# normal build, mipsolversThirdPartyTargets in MIPSOLVERS_THIRD_PARTY_ONLY mode
# (set by the top-level CMakeLists.txt before Dependencies.cmake is included).
if(NOT DEFINED MIPSOLVERS_THIRD_PARTY_EXPORT_SET)
  set(MIPSOLVERS_THIRD_PARTY_EXPORT_SET mipsolversTargets)
endif()

# ── Enable Fortran (project uses only CXX C; we enable Fortran here) ──────────
# Compiler detection shared with the vendored reference-LAPACK build.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/DetectFortranCompiler.cmake")

# ── MUMPS 5.7.3 — vendored in-tree sources only (offline; no network) ─────────
# The MUMPS source tree is committed under mumps/ (mirrors the highs/, scip/,
# ipopt/, suitesparse/ vendoring convention) so configure/build/deploy never
# touch the network.  There is intentionally NO download fallback: a missing
# vendored tree is a checkout error, not a reason to hit the network.
set(_M "${CMAKE_CURRENT_SOURCE_DIR}/mumps")
if(NOT EXISTS "${_M}/include/dmumps_c.h")
  message(FATAL_ERROR
    "vendored MUMPS sources missing (expected ${_M}/include/dmumps_c.h). "
    "The mumps/ tree (MUMPS 5.7.3) is committed to this repository — restore "
    "it. Network downloads are disabled by design (hermetic build).")
endif()

# ── Generate mumps_int_def.h (32-bit integer variant) ────────────────────────
# Avoid touching the tracked header on every configure: changing its timestamp
# needlessly recompiles all MUMPS C sources and the Ipopt MUMPS interface.
set(_ms_mumps_int_def
  "#ifndef MUMPS_INT_H\n#define MUMPS_INT_H\n#define MUMPS_INTSIZE32\n#endif\n")
set(_ms_write_mumps_int_def TRUE)
if(EXISTS "${_M}/include/mumps_int_def.h")
  file(READ "${_M}/include/mumps_int_def.h" _ms_existing_mumps_int_def)
  if(_ms_existing_mumps_int_def STREQUAL _ms_mumps_int_def)
    set(_ms_write_mumps_int_def FALSE)
  endif()
endif()
if(_ms_write_mumps_int_def)
  file(WRITE "${_M}/include/mumps_int_def.h" "${_ms_mumps_int_def}")
endif()
unset(_ms_existing_mumps_int_def)
unset(_ms_mumps_int_def)
unset(_ms_write_mumps_int_def)

# ─────────────────────────────────────────────────────────────────────────────
# MUMPS is built from the vendored in-tree sources (mumps/) by default — the
# deployment environment has no network and no Homebrew, so the from-source
# build is the only supported path.  A system/Homebrew MUMPS is used only when
# explicitly opted back in via -DMIPSOLVERS_FORCE_BUILD_MUMPS=OFF.
#
# Note: the from-source static libdmumps.a previously exhibited *unresolvable*
# Fortran symbols when linked into an executable on arm64 macOS; that trace is
# stale for the current 5.7.3 vendored build, which links and runs correctly
# (see the MumpsSolver tests).
# ─────────────────────────────────────────────────────────────────────────────
option(MIPSOLVERS_FORCE_BUILD_MUMPS
  "Build MUMPS from the vendored in-tree sources (offline default)" ON)

# Records (for the installed mipsolversConfig.cmake) whether the MUMPS runtime
# came from the Homebrew ipopt cellar — only then may the consumer config
# append those /opt/homebrew paths.
set(MIPSOLVERS_CONSUMER_NEEDS_BREW_MUMPS OFF)

set(_ms_use_brew_mumps OFF)
if(APPLE AND NOT MIPSOLVERS_FORCE_BUILD_MUMPS)
  set(_ms_brew_mumps_dir "/opt/homebrew/opt/ipopt/lib")
  set(_ms_brew_mumps_libs "")
  set(_ms_brew_mumps_ok TRUE)
  foreach(_ml dmumps mumps_common mpiseq pord)
    find_library(_ms_brew_lib_${_ml}
      NAMES ${_ml}
      PATHS "${_ms_brew_mumps_dir}"
      NO_DEFAULT_PATH)
    if(_ms_brew_lib_${_ml})
      list(APPEND _ms_brew_mumps_libs "${_ms_brew_lib_${_ml}}")
    else()
      set(_ms_brew_mumps_ok FALSE)
    endif()
  endforeach()
  set(_ms_use_brew_mumps ${_ms_brew_mumps_ok})
endif()

if(_ms_use_brew_mumps)
  # dmumps as an INTERFACE target: vendored headers for build-time includes,
  # Homebrew dylibs for linking.  No MUMPS sources are compiled.
  add_library(dmumps INTERFACE)
  target_include_directories(dmumps INTERFACE
    "$<BUILD_INTERFACE:${_M}/include>"
    "$<BUILD_INTERFACE:${_M}/libseq>")

  # The Homebrew MUMPS dylibs need the gfortran/quadmath runtime.  A from-source
  # build pulls these in automatically via the enabled Fortran language; an
  # INTERFACE target must add them explicitly so the final executable link
  # resolves Fortran runtime symbols (e.g. __gfortran_generate_error).
  set(_ms_fortran_runtime "")
  foreach(_fl gfortran quadmath)
    find_library(_ms_fortran_${_fl}
      NAMES ${_fl}
      PATHS /opt/homebrew/opt/gcc/lib/gcc/current
            /opt/homebrew/lib/gcc/current
      NO_DEFAULT_PATH)
    if(_ms_fortran_${_fl})
      list(APPEND _ms_fortran_runtime "${_ms_fortran_${_fl}}")
    endif()
  endforeach()

  target_link_libraries(dmumps INTERFACE
    ${_ms_brew_mumps_libs}
    ${_ms_fortran_runtime}
    "$<$<PLATFORM_ID:Darwin>:-framework Accelerate>")

  if(NOT TARGET MUMPS)
    add_library(MUMPS INTERFACE)
    target_link_libraries(MUMPS INTERFACE dmumps)
    add_library(MUMPS::MUMPS ALIAS MUMPS)
  endif()

  install(TARGETS dmumps MUMPS
    EXPORT  ${MIPSOLVERS_THIRD_PARTY_EXPORT_SET}
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")

  set(MIPSOLVERS_CONSUMER_NEEDS_BREW_MUMPS ON)
  message(WARNING
    "mipsolvers: using Homebrew MUMPS at ${_ms_brew_mumps_dir}. This is "
    "NON-HERMETIC (depends on Homebrew dylibs and the gcc Fortran runtime) "
    "and intended for local debugging only — do not use for deployable "
    "builds. Use -DMIPSOLVERS_FORCE_BUILD_MUMPS=ON (the default) for the "
    "vendored, offline MUMPS build.")
  message(STATUS
    "mipsolvers: using Homebrew MUMPS at ${_ms_brew_mumps_dir} "
    "(from-source build skipped; -DMIPSOLVERS_FORCE_BUILD_MUMPS=ON to override)")
  return()
endif()

# ─────────────────────────────────────────────────────────────────────────────
# Single dmumps STATIC library — all MUMPS sources compiled directly in.
# CMake's built-in Fortran module dependency scanner handles ordering of
# module-definition files automatically; no manual OBJECT-target chains needed.
# ─────────────────────────────────────────────────────────────────────────────
if(WIN32 AND CMAKE_Fortran_COMPILER_ID MATCHES "^(Intel|IntelLLVM)$")
  set(_MIPSOLVERS_FORTRAN_MAIN_STUB "${CMAKE_CURRENT_BINARY_DIR}/_mumps_main_stub.c")
  file(WRITE "${_MIPSOLVERS_FORTRAN_MAIN_STUB}"
    "void MAIN__(void) {}\n"
    "void MAIN_(void) {}\n")
  add_library(mipsolvers_fortran_main_stub STATIC "${_MIPSOLVERS_FORTRAN_MAIN_STUB}")
endif()

add_library(dmumps STATIC

  # ── PORD fill-reducing ordering library ──────────────────────────────────
  "${_M}/PORD/lib/graph.c"
  "${_M}/PORD/lib/gbipart.c"
  "${_M}/PORD/lib/gbisect.c"
  "${_M}/PORD/lib/ddcreate.c"
  "${_M}/PORD/lib/ddbisect.c"
  "${_M}/PORD/lib/nestdiss.c"
  "${_M}/PORD/lib/multisector.c"
  "${_M}/PORD/lib/gelim.c"
  "${_M}/PORD/lib/bucket.c"
  "${_M}/PORD/lib/tree.c"
  "${_M}/PORD/lib/symbfac.c"
  "${_M}/PORD/lib/interface.c"
  "${_M}/PORD/lib/sort.c"
  "${_M}/PORD/lib/minpriority.c"

  # ── Sequential MPI stub (libseq) ─────────────────────────────────────────
  "${_M}/libseq/elapse.c"
  "${_M}/libseq/mpic.c"
  "${_M}/libseq/mpi.f"

  # ── MUMPS common — Fortran module definitions ─────────────────────────────
  "${_M}/src/mumps_memory_mod.F"
  "${_M}/src/double_linked_list.F"
  "${_M}/src/lr_common.F"
  "${_M}/src/ana_orderings_wrappers_m.F"
  "${_M}/src/omp_tps_common_m.F"
  "${_M}/src/mumps_l0_omp_m.F"
  "${_M}/src/ana_omp_m.F"
  "${_M}/src/fac_maprow_data_m.F"
  "${_M}/src/fac_future_niv2_mod.F"
  "${_M}/src/fac_descband_data_m.F"
  "${_M}/src/front_data_mgt_m.F"
  "${_M}/src/fac_asm_build_sort_index_m.F"
  "${_M}/src/fac_asm_build_sort_index_ELT_m.F"
  "${_M}/src/mumps_static_mapping.F"
  "${_M}/src/mumps_ooc_common.F"
  "${_M}/src/ana_blk_m.F"            # >= 5.3
  "${_M}/src/mumps_pivnul_mod.F"     # >= 5.6
  "${_M}/src/sol_ds_common_m.F"      # >= 5.7
  "${_M}/src/fac_ibct_data_m.F"      # < 5.8 (5.7.x: yes)
  "${_M}/src/mumps_comm_ibcast.F"    # < 5.8 (5.7.x: yes)
  "${_M}/src/mumps_mpitoomp_m.F"

  # ── MUMPS common — other Fortran routines ─────────────────────────────────
  "${_M}/src/ana_orderings.F"
  "${_M}/src/ana_set_ordering.F"
  "${_M}/src/ana_AMDMF.F"
  "${_M}/src/bcast_errors.F"
  "${_M}/src/estim_flops.F"
  "${_M}/src/mumps_type2_blocking.F"
  "${_M}/src/mumps_version.F"
  "${_M}/src/mumps_print_defined.F"
  "${_M}/src/tools_common.F"
  "${_M}/src/ana_blk.F"              # >= 5.3
  "${_M}/src/sol_common.F"

  # ── MUMPS common — C sources ──────────────────────────────────────────────
  "${_M}/src/mumps_common.c"
  "${_M}/src/mumps_io_basic.c"
  "${_M}/src/mumps_io_thread.c"
  "${_M}/src/mumps_io_err.c"
  "${_M}/src/mumps_io.c"
  "${_M}/src/mumps_numa.c"
  "${_M}/src/mumps_pord.c"
  "${_M}/src/mumps_thread.c"
  "${_M}/src/mumps_save_restore_C.c"
  "${_M}/src/mumps_addr.c"             # >= 5.6
  "${_M}/src/mumps_config_file_C.c"
  "${_M}/src/mumps_thread_affinity.c"
  "${_M}/src/mumps_register_thread.c"  # >= 5.4

  # ── dmumps double-precision Fortran sources ───────────────────────────────
  "${_M}/src/dsol_distrhs.F"
  "${_M}/src/dmumps_comm_buffer.F"
  "${_M}/src/dmumps_ooc_buffer.F"
  "${_M}/src/dmumps_ooc.F"
  "${_M}/src/dmumps_struc_def.F"
  "${_M}/src/dana_aux.F"
  "${_M}/src/dana_aux_par.F"
  "${_M}/src/dana_lr.F"
  "${_M}/src/dfac_asm_master_ELT_m.F"
  "${_M}/src/dfac_asm_master_m.F"
  "${_M}/src/dfac_front_aux.F"
  "${_M}/src/dfac_front_LU_type1.F"
  "${_M}/src/dfac_front_LU_type2.F"
  "${_M}/src/dfac_front_LDLT_type1.F"
  "${_M}/src/dfac_front_LDLT_type2.F"
  "${_M}/src/dfac_front_type2_aux.F"
  "${_M}/src/dfac_lr.F"
  "${_M}/src/dfac_omp_m.F"
  "${_M}/src/dfac_par_m.F"
  "${_M}/src/dlr_core.F"
  "${_M}/src/dmumps_lr_data_m.F"
  "${_M}/src/domp_tps_m.F"
  "${_M}/src/dstatic_ptr_m.F"
  "${_M}/src/dlr_type.F"
  "${_M}/src/dmumps_save_restore.F"
  "${_M}/src/dmumps_save_restore_files.F"
  "${_M}/src/dfac_mem_dynamic.F"
  "${_M}/src/dmumps_config_file.F"
  "${_M}/src/dmumps_sol_es.F"
  "${_M}/src/dsol_lr.F"
  "${_M}/src/dfac_sispointers_m.F"   # >= 5.3
  "${_M}/src/dfac_sol_l0omp_m.F"    # >= 5.3
  "${_M}/src/dsol_omp_m.F"          # >= 5.3
  "${_M}/src/dmumps_mpi3_mod.F"     # >= 5.6
  "${_M}/src/dlr_stats.F"           # < 5.8 (5.7.x: yes)
  "${_M}/src/dmumps_load.F"         # < 5.8 (5.7.x: yes)
  "${_M}/src/dini_driver.F"
  "${_M}/src/dana_driver.F"
  "${_M}/src/dfac_driver.F"
  "${_M}/src/dsol_driver.F"
  "${_M}/src/dend_driver.F"
  "${_M}/src/dana_aux_ELT.F"
  "${_M}/src/dana_dist_m.F"
  "${_M}/src/dana_LDLT_preprocess.F"
  "${_M}/src/dana_reordertree.F"
  "${_M}/src/darrowheads.F"
  "${_M}/src/dbcast_int.F"
  "${_M}/src/dfac_asm_ELT.F"
  "${_M}/src/dfac_asm.F"
  "${_M}/src/dfac_b.F"
  "${_M}/src/dfac_distrib_distentry.F"
  "${_M}/src/dfac_distrib_ELT.F"
  "${_M}/src/dfac_lastrtnelind.F"
  "${_M}/src/dfac_mem_alloc_cb.F"
  "${_M}/src/dfac_mem_compress_cb.F"
  "${_M}/src/dfac_mem_free_block_cb.F"
  "${_M}/src/dfac_mem_stack_aux.F"
  "${_M}/src/dfac_mem_stack.F"
  "${_M}/src/dfac_process_band.F"
  "${_M}/src/dfac_process_blfac_slave.F"
  "${_M}/src/dfac_process_blocfacto_LDLT.F"
  "${_M}/src/dfac_process_blocfacto.F"
  "${_M}/src/dfac_process_bf.F"
  "${_M}/src/dfac_process_end_facto_slave.F"
  "${_M}/src/dfac_process_contrib_type1.F"
  "${_M}/src/dfac_process_contrib_type2.F"
  "${_M}/src/dfac_process_contrib_type3.F"
  "${_M}/src/dfac_process_maprow.F"
  "${_M}/src/dfac_process_master2.F"
  "${_M}/src/dfac_process_message.F"
  "${_M}/src/dfac_process_root2slave.F"
  "${_M}/src/dfac_process_root2son.F"
  "${_M}/src/dfac_process_rtnelind.F"
  "${_M}/src/dfac_root_parallel.F"
  "${_M}/src/dfac_scalings.F"
  "${_M}/src/dfac_determinant.F"
  "${_M}/src/dfac_scalings_simScaleAbs.F"
  "${_M}/src/dfac_scalings_simScale_util.F"
  "${_M}/src/dfac_sol_pool.F"
  "${_M}/src/dfac_type3_symmetrize.F"
  "${_M}/src/dini_defaults.F"
  "${_M}/src/dmumps_driver.F"
  "${_M}/src/dmumps_f77.F"
  "${_M}/src/dmumps_iXamax.F"
  "${_M}/src/dana_mtrans.F"
  "${_M}/src/dooc_panel_piv.F"
  "${_M}/src/drank_revealing.F"
  "${_M}/src/dsol_aux.F"
  "${_M}/src/dsol_bwd_aux.F"
  "${_M}/src/dsol_bwd.F"
  "${_M}/src/dsol_c.F"
  "${_M}/src/dsol_fwd_aux.F"
  "${_M}/src/dsol_fwd.F"
  "${_M}/src/dsol_matvec.F"
  "${_M}/src/dsol_root_parallel.F"
  "${_M}/src/dtools.F"
  "${_M}/src/dtype3_root.F"
  "${_M}/src/dsol_distsol.F"         # >= 5.7
  "${_M}/src/dfac_diag.F"            # >= 5.7
  "${_M}/src/dfac_dist_arrowheads_omp.F"  # >= 5.7

  # ── dmumps double-precision C interface ───────────────────────────────────
  "${_M}/src/mumps_c.c"
  "${_M}/src/dmumps_gpu.c"
)

# ── Include paths ─────────────────────────────────────────────────────────────
# PUBLIC with BUILD_INTERFACE: ipopt_local needs MUMPS headers at build time
# (dmumps_c.h, mpi.h from libseq), but they must not be exported with absolute
# build-tree paths — consumers of an installed mipsolvers don't use MUMPS directly.
target_include_directories(dmumps
  PUBLIC
    "$<BUILD_INTERFACE:${_M}/include>"
    "$<BUILD_INTERFACE:${_M}/libseq>"
  PRIVATE
    "${_M}/PORD/include"
)

# ── Fortran module output directory ───────────────────────────────────────────
set_target_properties(dmumps PROPERTIES
  Fortran_MODULE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/_mumps_mods")

# ── Compile options (language-guarded so C flags don't reach Fortran and v.v.) ─
# -w                       suppress all warnings (MUMPS sources are not warning-clean)
# -fno-strict-aliasing     prevents aliasing-related miscompilation
# -fallow-argument-mismatch  required for GCC>=10 (MUMPS uses non-standard Fortran)
# -fallow-invalid-boz        ditto
# -Werror-implicit-function-declaration  catch missing C prototypes early
target_compile_options(dmumps PRIVATE
  $<$<COMPILE_LANGUAGE:Fortran>:
    $<$<OR:$<Fortran_COMPILER_ID:GNU>,$<Fortran_COMPILER_ID:LLVMFlang>>:
      -w
      -fno-strict-aliasing
      $<$<VERSION_GREATER_EQUAL:${CMAKE_Fortran_COMPILER_VERSION},10>:
        -fallow-argument-mismatch
        -fallow-invalid-boz
      >
    >
    $<$<AND:$<PLATFORM_ID:Windows>,$<OR:$<Fortran_COMPILER_ID:Intel>,$<Fortran_COMPILER_ID:IntelLLVM>>>:
      /nofor_main
    >
  >
  $<$<COMPILE_LANGUAGE:C>:
    $<$<OR:$<C_COMPILER_ID:GNU>,$<C_COMPILER_ID:Clang>,$<C_COMPILER_ID:AppleClang>>:
      -fno-strict-aliasing
      -Werror-implicit-function-declaration
    >
  >
)

# ── Compile definitions ───────────────────────────────────────────────────────
# Intel Fortran on Windows uses uppercase external names without a trailing
# underscore; GNU/MinGW and Unix compilers use the Add_ convention here.
# pord  = use the PORD fill-reducing ordering (the only one we build)
if(WIN32 AND CMAKE_Fortran_COMPILER_ID MATCHES "^(Intel|IntelLLVM)$")
  set(_MIPSOLVERS_MUMPS_C_MANGLING MUMPS_WIN32)
else()
  set(_MIPSOLVERS_MUMPS_C_MANGLING Add_)
endif()
target_compile_definitions(dmumps PRIVATE
  $<$<COMPILE_LANGUAGE:C>:${_MIPSOLVERS_MUMPS_C_MANGLING}>
  $<$<COMPILE_LANGUAGE:C>:pord>
  $<$<COMPILE_LANGUAGE:Fortran>:pord>
)
unset(_MIPSOLVERS_MUMPS_C_MANGLING)
# MUMPS_ARITH=MUMPS_ARITH_d selects double-precision types in mumps_c.c / dmumps_gpu.c
set_source_files_properties(
  "${_M}/src/mumps_c.c"
  "${_M}/src/dmumps_gpu.c"
  PROPERTIES COMPILE_DEFINITIONS "MUMPS_ARITH=MUMPS_ARITH_d"
)

# ── Runtime dependencies: BLAS/LAPACK (unified via MIPSOLVERS_BLAS_LIBRARIES,
# resolved in cmake/Dependencies.cmake), plus the Fortran runtime for the
# from-source Fortran objects (see mipsolvers_link_fortran_runtime in
# cmake/DetectFortranCompiler.cmake — static archives on macOS, implicit
# runtime elsewhere).
if(MIPSOLVERS_BLAS_LIBRARIES)
  target_link_libraries(dmumps PUBLIC ${MIPSOLVERS_BLAS_LIBRARIES})
else()
  message(FATAL_ERROR
    "The vendored MUMPS build requires BLAS/LAPACK, but "
    "MIPSOLVERS_BLAS_LIBRARIES is empty. On Windows, either use the default "
    "Ipopt MKL/Pardiso backend, provide a system BLAS/LAPACK discoverable by "
    "CMake, or configure with -DMIPSOLVERS_FORCE_VENDORED_BLAS=ON and a "
    "supported Fortran compiler (Intel oneAPI ifx/ifort or MinGW gfortran).")
endif()

# Static-link the GCC Fortran runtime into the final executables on Linux
# (deployment convenience); macOS always uses static runtime archives.
option(MIPSOLVERS_STATIC_LIBGFORTRAN
  "Link libgfortran/libgcc statically for Linux deployment" OFF)

mipsolvers_link_fortran_runtime(dmumps PUBLIC)

if(NOT APPLE AND NOT WIN32 AND MIPSOLVERS_STATIC_LIBGFORTRAN)
  target_link_options(dmumps INTERFACE -static-libgfortran -static-libgcc)
endif()

if(TARGET mipsolvers_fortran_main_stub)
  target_link_libraries(dmumps PUBLIC mipsolvers_fortran_main_stub)
endif()

# ── MUMPS::MUMPS — interface alias (matches scivision's exported target name) ─
if(NOT TARGET MUMPS)
  add_library(MUMPS INTERFACE)
  target_link_libraries(MUMPS INTERFACE dmumps)
  add_library(MUMPS::MUMPS ALIAS MUMPS)
endif()

# ── Export dmumps and MUMPS so ipopt_local's install(EXPORT) succeeds ─────────
install(TARGETS dmumps MUMPS
  EXPORT  ${MIPSOLVERS_THIRD_PARTY_EXPORT_SET}
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)

if(TARGET mipsolvers_fortran_main_stub)
  install(TARGETS mipsolvers_fortran_main_stub
  EXPORT  ${MIPSOLVERS_THIRD_PARTY_EXPORT_SET}
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)

endif()

message(STATUS "mipsolvers: MUMPS 5.7.3 built from source (sequential, double precision)")
