# Resolve PaPILO without network access. The bundled integration uses PaPILO's
# template implementation directly, so no Boost/PaPILO binary libraries leak
# into the installed mipsolvers target.

set(MIPSOLVERS_PAPILO_SOURCE_DIR "" CACHE PATH
    "Local PaPILO source tree (must contain src/papilo/Config.hpp)")
set(MIPSOLVERS_PAPILO_BOOST_DIR "" CACHE PATH
    "Local Boost include root used by source-built PaPILO")
set(MIPSOLVERS_PAPILO_ROOT "" CACHE PATH
    "Local installed PaPILO prefix (used only when no source tree is selected)")

set(MIPSOLVERS_HAVE_PAPILO OFF)
set(MIPSOLVERS_PAPILO_LOCAL_SOURCE OFF)
set(MIPSOLVERS_PAPILO_INCLUDE_DIRS "")

set(_mipsolvers_bundled_papilo
    "${CMAKE_CURRENT_LIST_DIR}/../third_party/papilo")
set(_mipsolvers_bundled_papilo_boost
    "${CMAKE_CURRENT_LIST_DIR}/../third_party/boost_papilo")

set(_mipsolvers_papilo_source "")
if(MIPSOLVERS_PAPILO_SOURCE_DIR)
  set(_mipsolvers_papilo_source "${MIPSOLVERS_PAPILO_SOURCE_DIR}")
elseif(EXISTS "${_mipsolvers_bundled_papilo}/src/papilo/Config.hpp")
  set(_mipsolvers_papilo_source "${_mipsolvers_bundled_papilo}")
endif()

if(_mipsolvers_papilo_source)
  if(NOT EXISTS "${_mipsolvers_papilo_source}/src/papilo/Config.hpp")
    message(FATAL_ERROR
      "MIPSOLVERS_PAPILO_SOURCE_DIR does not contain src/papilo/Config.hpp: "
      "${_mipsolvers_papilo_source}")
  endif()

  if(MIPSOLVERS_PAPILO_BOOST_DIR)
    set(_mipsolvers_papilo_boost "${MIPSOLVERS_PAPILO_BOOST_DIR}")
  else()
    set(_mipsolvers_papilo_boost "${_mipsolvers_bundled_papilo_boost}")
  endif()
  if(NOT EXISTS "${_mipsolvers_papilo_boost}/boost/version.hpp")
    message(FATAL_ERROR
      "PaPILO source integration requires a local Boost include tree. Set "
      "MIPSOLVERS_PAPILO_BOOST_DIR or restore third_party/boost_papilo.")
  endif()

  set(MIPSOLVERS_HAVE_PAPILO ON)
  set(MIPSOLVERS_PAPILO_LOCAL_SOURCE ON)
  set(MIPSOLVERS_PAPILO_INCLUDE_DIRS
      "${_mipsolvers_papilo_source}/src"
      "${_mipsolvers_papilo_boost}")
  message(STATUS
    "mipsolvers: PaPILO enabled from local source: ${_mipsolvers_papilo_source}")
elseif(MIPSOLVERS_PAPILO_ROOT OR DEFINED ENV{PAPILO_ROOT})
  set(_mipsolvers_papilo_hints "${MIPSOLVERS_PAPILO_ROOT}" "$ENV{PAPILO_ROOT}")
  find_package(papilo CONFIG QUIET
    HINTS ${_mipsolvers_papilo_hints}
    PATH_SUFFIXES lib/cmake/papilo
    NO_DEFAULT_PATH)
  if(papilo_FOUND AND TARGET papilo)
    set(MIPSOLVERS_HAVE_PAPILO ON)
    message(STATUS "mipsolvers: PaPILO enabled from explicit local package")
  else()
    message(STATUS
      "mipsolvers: explicit PaPILO package not found; using native MILP presolve")
  endif()
else()
  message(STATUS
    "mipsolvers: PaPILO source/package unavailable; using native MILP presolve")
endif()

unset(_mipsolvers_bundled_papilo)
unset(_mipsolvers_bundled_papilo_boost)
unset(_mipsolvers_papilo_source)
unset(_mipsolvers_papilo_boost)
unset(_mipsolvers_papilo_hints)
