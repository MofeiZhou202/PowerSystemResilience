# cmake/DetectFortranCompiler.cmake
# Detects a Fortran compiler and enables the Fortran language.
# Shared by BuildMUMPS.cmake (vendored MUMPS) and BuildRefLAPACK.cmake
# (vendored reference BLAS/LAPACK fallback) so both resolve the compiler
# identically.  Safe to include multiple times: once CMAKE_Fortran_COMPILER
# is set the detection is skipped and enable_language() is a no-op.

# Support explicit toolchain wiring via cache/env so the same repo can be
# configured consistently on Windows/macOS/Linux.
if(NOT CMAKE_Fortran_COMPILER)
  set(_MIPSOLVERS_FORTRAN_HINTS)

  if(DEFINED MIPSOLVERS_FORTRAN_COMPILER AND NOT MIPSOLVERS_FORTRAN_COMPILER STREQUAL "")
    list(APPEND _MIPSOLVERS_FORTRAN_HINTS "${MIPSOLVERS_FORTRAN_COMPILER}")
  endif()
  if(DEFINED ENV{MIPSOLVERS_FORTRAN_COMPILER} AND NOT "$ENV{MIPSOLVERS_FORTRAN_COMPILER}" STREQUAL "")
    list(APPEND _MIPSOLVERS_FORTRAN_HINTS "$ENV{MIPSOLVERS_FORTRAN_COMPILER}")
  endif()
  if(DEFINED ENV{FC} AND NOT "$ENV{FC}" STREQUAL "")
    list(APPEND _MIPSOLVERS_FORTRAN_HINTS "$ENV{FC}")
  endif()

  if(WIN32)
    list(APPEND _MIPSOLVERS_FORTRAN_HINTS
      "C:/Program Files (x86)/Intel/oneAPI/compiler/latest/bin/ifx.exe"
      "C:/Program Files (x86)/Intel/oneAPI/compiler/latest/bin/ifort.exe"
      "C:/Program Files/Intel/oneAPI/compiler/latest/bin/ifx.exe"
      "C:/Program Files/Intel/oneAPI/compiler/latest/bin/ifort.exe"
      "C:/msys64/mingw64/bin/gfortran.exe")
    set(_MIPSOLVERS_FORTRAN_NAMES ifx ifort gfortran)
  elseif(APPLE)
    list(APPEND _MIPSOLVERS_FORTRAN_HINTS
      "/opt/homebrew/bin/gfortran"
      "/usr/local/bin/gfortran")
    set(_MIPSOLVERS_FORTRAN_NAMES gfortran ifx ifort)
  else()
    list(APPEND _MIPSOLVERS_FORTRAN_HINTS
      "/usr/bin/gfortran"
      "/usr/local/bin/gfortran")
    set(_MIPSOLVERS_FORTRAN_NAMES gfortran ifx ifort)
  endif()

  set(_MIPSOLVERS_FORTRAN_CANDIDATE "")
  foreach(_fc_hint IN LISTS _MIPSOLVERS_FORTRAN_HINTS)
    if(EXISTS "${_fc_hint}")
      set(_MIPSOLVERS_FORTRAN_CANDIDATE "${_fc_hint}")
      break()
    endif()
  endforeach()

  if(NOT _MIPSOLVERS_FORTRAN_CANDIDATE)
    find_program(_MIPSOLVERS_FORTRAN_CANDIDATE NAMES ${_MIPSOLVERS_FORTRAN_NAMES})
  endif()

  if(_MIPSOLVERS_FORTRAN_CANDIDATE)
    set(CMAKE_Fortran_COMPILER "${_MIPSOLVERS_FORTRAN_CANDIDATE}" CACHE FILEPATH "Fortran compiler for embedded MUMPS/Ipopt/LAPACK" FORCE)
    message(STATUS "mipsolvers: using Fortran compiler ${CMAKE_Fortran_COMPILER}")
  endif()
endif()

enable_language(Fortran)

# Snapshot the detected implicit Fortran runtime for
# mipsolvers_link_fortran_runtime(): on Apple the static-archive path clears
# CMAKE_Fortran_IMPLICIT_LINK_LIBRARIES to silence CMake's automatic (dynamic)
# runtime injection, so later callers must use this snapshot.  Snapshotted
# only once — a later include() of this module (e.g. from BuildMUMPS after
# BuildRefLAPACK already cleared the CMAKE_ variables) must not re-capture
# the cleared values.
if(NOT DEFINED MIPSOLVERS_FORTRAN_IMPLICIT_LINK_LIBRARIES)
  set(MIPSOLVERS_FORTRAN_IMPLICIT_LINK_LIBRARIES
    "${CMAKE_Fortran_IMPLICIT_LINK_LIBRARIES}")
  set(MIPSOLVERS_FORTRAN_IMPLICIT_LINK_DIRECTORIES
    "${CMAKE_Fortran_IMPLICIT_LINK_DIRECTORIES}")
endif()

# mipsolvers_link_fortran_runtime(<target> <scope>)
# Links the GCC/gfortran runtime needed by Fortran objects into <target> with
# the given visibility (e.g. PUBLIC).  CMake only injects the implicit Fortran
# link libraries automatically when the *final* link is Fortran-aware; a
# CXX-only executable (e.g. a downstream consumer's server binary) fails with
# undefined __gfortran_os_error_at/__gfortran_runtime_error_at without this
# explicit dependency.
#
# Apple: link the runtime as STATIC archives (libgfortran.a, libquadmath.a,
# libgcc.a, ...) resolved inside the compiler's own directories, so
# executables carry no /opt/homebrew dylib dependencies (hermetic deploy), and
# clear the implicit-link variables so CMake's automatic dynamic injection
# adds nothing.  Falls back to the implicit dynamic runtime with a WARNING
# when the static archives are not found.
# Other platforms: link the implicit runtime discovered by CMake (unchanged
# historical behavior).
function(mipsolvers_link_fortran_runtime _target _scope)
  if(APPLE)
    set(_msfrt_libs "")
    set(_msfrt_missing "")
    foreach(_fl IN LISTS MIPSOLVERS_FORTRAN_IMPLICIT_LINK_LIBRARIES)
      find_library(_msfrt_static_${_fl}
        NAMES "lib${_fl}.a"
        PATHS ${MIPSOLVERS_FORTRAN_IMPLICIT_LINK_DIRECTORIES}
        NO_DEFAULT_PATH NO_CACHE)
      if(_msfrt_static_${_fl})
        list(APPEND _msfrt_libs "${_msfrt_static_${_fl}}")
      else()
        list(APPEND _msfrt_missing "${_fl}")
      endif()
    endforeach()
    if(_msfrt_missing)
      message(WARNING
        "mipsolvers: static GCC Fortran runtime archives not found for: "
        "${_msfrt_missing} — falling back to the implicit (dynamic) Fortran "
        "runtime. Executables will depend on Homebrew gcc dylibs "
        "(non-hermetic).")
      target_link_libraries(${_target} ${_scope}
        ${MIPSOLVERS_FORTRAN_IMPLICIT_LINK_LIBRARIES})
      target_link_directories(${_target} ${_scope}
        ${MIPSOLVERS_FORTRAN_IMPLICIT_LINK_DIRECTORIES})
    else()
      target_link_libraries(${_target} ${_scope} ${_msfrt_libs})
      # Suppress CMake's automatic (dynamic) Fortran runtime injection for all
      # subsequently generated targets in this directory scope: the static
      # archives above already provide the runtime.
      set(CMAKE_Fortran_IMPLICIT_LINK_LIBRARIES "" PARENT_SCOPE)
      set(CMAKE_Fortran_IMPLICIT_LINK_DIRECTORIES "" PARENT_SCOPE)
      message(STATUS
        "mipsolvers: linking GCC Fortran runtime statically (hermetic)")
    endif()
  else()
    if(MIPSOLVERS_FORTRAN_IMPLICIT_LINK_LIBRARIES)
      target_link_libraries(${_target} ${_scope}
        ${MIPSOLVERS_FORTRAN_IMPLICIT_LINK_LIBRARIES})
      target_link_directories(${_target} ${_scope}
        ${MIPSOLVERS_FORTRAN_IMPLICIT_LINK_DIRECTORIES})
    endif()
  endif()
endfunction()
