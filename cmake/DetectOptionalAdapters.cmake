# Optional adapters are independent of whether the numeric third-party stack
# is built in-tree or loaded from a local prebuilt package.

option(MIPSOLVERS_USE_GUROBI
  "Enable Gurobi detection and native C API adapter when available" OFF)
option(MIPSOLVERS_USE_PAPILO
  "Enable local PaPILO presolve when available" ON)

set(MIPSOLVERS_HAVE_GUROBI OFF)
set(MIPSOLVERS_GUROBI_INCLUDE_DIRS "")
set(MIPSOLVERS_GUROBI_LIBRARIES "")
if(MIPSOLVERS_USE_GUROBI)
  set(_mipsolvers_gurobi_hints
      "$ENV{GUROBI_HOME}"
      /Library/gurobi1300/macos_universal2
      /Library/gurobi1200/macos_universal2
      /opt/gurobi/macos_universal2
      /opt/gurobi1300/linux64
      /opt/gurobi1200/linux64
      /opt/gurobi1100/linux64
      /opt/gurobi/linux64
      "C:/gurobi1300/win64"
      "C:/gurobi1200/win64"
      "C:/gurobi1100/win64")
  find_path(MIPSOLVERS_GUROBI_INCLUDE_DIR NAMES gurobi_c.h
    HINTS ${_mipsolvers_gurobi_hints}
    PATH_SUFFIXES include)
  foreach(_grb_ver 130 120 110 100 95)
    if(NOT MIPSOLVERS_GUROBI_LIBRARY)
      find_library(MIPSOLVERS_GUROBI_LIBRARY NAMES gurobi${_grb_ver}
        HINTS ${_mipsolvers_gurobi_hints}
        PATH_SUFFIXES lib)
    endif()
  endforeach()
  if(MIPSOLVERS_GUROBI_INCLUDE_DIR AND MIPSOLVERS_GUROBI_LIBRARY)
    set(MIPSOLVERS_HAVE_GUROBI ON)
    set(MIPSOLVERS_GUROBI_INCLUDE_DIRS ${MIPSOLVERS_GUROBI_INCLUDE_DIR})
    set(MIPSOLVERS_GUROBI_LIBRARIES ${MIPSOLVERS_GUROBI_LIBRARY})
    message(STATUS "mipsolvers: Gurobi detected: ${MIPSOLVERS_GUROBI_LIBRARY}")
  else()
    message(STATUS
      "mipsolvers: Gurobi enabled but not detected; packaged solvers remain active")
  endif()
  unset(_mipsolvers_gurobi_hints)
else()
  message(STATUS "mipsolvers: Gurobi disabled (MIPSOLVERS_USE_GUROBI=OFF)")
endif()

if(MIPSOLVERS_USE_PAPILO)
  include(${CMAKE_CURRENT_LIST_DIR}/BuildPaPILO.cmake)
else()
  set(MIPSOLVERS_HAVE_PAPILO OFF)
  set(MIPSOLVERS_PAPILO_LOCAL_SOURCE OFF)
  set(MIPSOLVERS_PAPILO_INCLUDE_DIRS "")
  message(STATUS "mipsolvers: PaPILO disabled (MIPSOLVERS_USE_PAPILO=OFF)")
endif()
