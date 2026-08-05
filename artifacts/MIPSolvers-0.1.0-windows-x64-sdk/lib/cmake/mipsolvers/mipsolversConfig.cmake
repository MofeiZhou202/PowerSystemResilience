
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was mipsolversConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

include(CMakeFindDependencyMacro)

# Threads is always needed (HiGHS uses it)
find_dependency(Threads)

# ── Eigen3 (header-only) ──────────────────────────────────────────────────────
# 优先使用系统安装的 Eigen3；若找不到，回退到随本包附带的副本。
if(NOT TARGET Eigen3::Eigen)
  find_package(Eigen3 3.3 CONFIG QUIET)
endif()
if(NOT TARGET Eigen3::Eigen)
  set(_mipsolvers_eigen3_bundled
      "${PACKAGE_PREFIX_DIR}/include/mipsolvers-deps/eigen3")
  if(EXISTS "${_mipsolvers_eigen3_bundled}/Eigen/Core")
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${_mipsolvers_eigen3_bundled}")
  else()
    message(FATAL_ERROR
      "mipsolvers: Eigen3 未找到。请通过以下方式安装：\n"
      "  macOS:   brew install eigen\n"
      "  Ubuntu:  apt install libeigen3-dev\n"
      "  Windows: vcpkg install eigen3\n"
      "或在 cmake 时传入 -DCMAKE_PREFIX_PATH=<Eigen3 安装路径>")
  endif()
  unset(_mipsolvers_eigen3_bundled)
endif()

# ── fmt ───────────────────────────────────────────────────────────────────────
# 当 fmt 以 vendored 方式构建时，它已作为本包导出目标（mipsolvers::fmt）随
# mipsolversTargets 一起安装，消费者无需系统 fmt；否则在消费者环境查找系统 fmt。
if(NOT ON)
  if(NOT TARGET fmt::fmt AND NOT TARGET fmt::fmt-header-only)
    find_package(fmt QUIET)
    if(NOT fmt_FOUND)
      message(FATAL_ERROR
        "mipsolvers: fmt 未找到。请通过以下方式安装：\n"
        "  macOS:   brew install fmt\n"
        "  Ubuntu:  apt install libfmt-dev\n"
        "  Windows: vcpkg install fmt")
    endif()
  endif()
endif()

# ── nlohmann_json (header-only) ───────────────────────────────────────────────
# 优先使用系统安装的 nlohmann_json；若找不到，回退到随本包附带的副本。
if(NOT TARGET nlohmann_json::nlohmann_json)
  find_package(nlohmann_json 3.11 CONFIG QUIET)
endif()
if(NOT TARGET nlohmann_json::nlohmann_json)
  set(_mipsolvers_json_bundled
      "${PACKAGE_PREFIX_DIR}/include/mipsolvers-deps/nlohmann_json")
  if(EXISTS "${_mipsolvers_json_bundled}/nlohmann/json.hpp")
    add_library(nlohmann_json::nlohmann_json INTERFACE IMPORTED)
    set_target_properties(nlohmann_json::nlohmann_json PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${_mipsolvers_json_bundled}")
  else()
    message(FATAL_ERROR
      "mipsolvers: nlohmann_json 未找到。请通过以下方式安装：\n"
      "  macOS:   brew install nlohmann-json\n"
      "  Ubuntu:  apt install nlohmann-json3-dev\n"
      "  Windows: vcpkg install nlohmann-json")
  endif()
  unset(_mipsolvers_json_bundled)
endif()

# ── OpenMP（静态归档的 LINK_ONLY 传递依赖）────────────────────────────────────
# 构建时 OpenMP::OpenMP_CXX 以 PRIVATE 链接进静态库，导出接口中保留为
# $<LINK_ONLY:OpenMP::OpenMP_CXX>，消费者侧必须重建该目标，否则 generate 报错。
if(OFF AND NOT TARGET OpenMP::OpenMP_CXX)
  find_package(OpenMP QUIET COMPONENTS CXX)
  if(NOT OpenMP_CXX_FOUND AND APPLE)
    # Apple Clang：需要 -Xpreprocessor -fopenmp + 显式 libomp 路径（与构建侧一致）。
    find_path(_ms_omp_include omp.h
      HINTS /opt/homebrew/opt/libomp/include /usr/local/opt/libomp/include)
    find_library(_ms_omp_lib NAMES omp
      HINTS /opt/homebrew/opt/libomp/lib /usr/local/opt/libomp/lib)
    if(_ms_omp_include AND _ms_omp_lib)
      add_library(OpenMP::OpenMP_CXX INTERFACE IMPORTED)
      set_target_properties(OpenMP::OpenMP_CXX PROPERTIES
        INTERFACE_COMPILE_OPTIONS "-Xpreprocessor;-fopenmp"
        INTERFACE_INCLUDE_DIRECTORIES "${_ms_omp_include}"
        INTERFACE_LINK_LIBRARIES "${_ms_omp_lib}")
    endif()
  endif()
  if(NOT TARGET OpenMP::OpenMP_CXX)
    message(WARNING
      "mipsolvers: 本包构建时启用了 OpenMP，但当前环境未找到 OpenMP，最终链接可能失败。")
  endif()
endif()

include("${CMAKE_CURRENT_LIST_DIR}/mipsolversTargets.cmake")

# ── 可选系统依赖：在消费者机器上重新发现 ─────────────────────────────────────────
# 这些库在构建时通过 $<BUILD_INTERFACE:...> 链接（路径不写入导出文件）。
# 消费者需要在自己的系统上安装对应库，此处尝试自动定位并注入到链接接口。

# ── SuiteSparse ────────────────────────────────────────────────────────────────
# 当 SuiteSparse 支持来自 vendored 导出目标（umfpack_vendored/klu_vendored，
# 随 mipsolversTargets 安装）时，跳过系统 SuiteSparse 的重新发现。
if(OFF AND NOT OFF)
  set(_ms_ss_libs)
  find_package(PkgConfig QUIET)
  if(PKG_CONFIG_FOUND)
    pkg_check_modules(_ms_umfpack QUIET umfpack)
    pkg_check_modules(_ms_klu     QUIET klu)
    if(_ms_umfpack_FOUND)
      list(APPEND _ms_ss_libs ${_ms_umfpack_LIBRARIES})
    endif()
    if(_ms_klu_FOUND)
      list(APPEND _ms_ss_libs ${_ms_klu_LIBRARIES})
    endif()
  endif()
  if(NOT _ms_ss_libs)
    # 回退：直接在系统路径和常见位置查找
    foreach(_ss_name umfpack klu amd colamd cholmod suitesparseconfig)
      find_library(_ms_ss_${_ss_name} ${_ss_name}
        HINTS /opt/homebrew/opt/suite-sparse/lib /usr/local/lib /usr/lib
              /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu)
      if(_ms_ss_${_ss_name})
        list(APPEND _ms_ss_libs "${_ms_ss_${_ss_name}}")
      endif()
    endforeach()
  endif()
  if(_ms_ss_libs)
    set_property(TARGET mipsolvers::mipsolvers APPEND PROPERTY
      INTERFACE_LINK_LIBRARIES "${_ms_ss_libs}")
  else()
    message(WARNING
      "mipsolvers: 构建时启用了 SuiteSparse 支持，但在当前系统未找到 SuiteSparse。\n"
      "如需完整功能，请安装：macOS: brew install suite-sparse  |  Ubuntu: apt install libsuitesparse-dev")
  endif()
  unset(_ms_ss_libs)
endif()

# ── Gurobi（可选外部适配器）──────────────────────────────────────────────────
if(OFF)
  # Gurobi 路径因版本和平台而异，尝试通用查找
  find_library(_ms_gurobi_lib
    NAMES gurobi130 gurobi120 gurobi110 gurobi100 gurobi gurobi_c++
    HINTS
      /Library/gurobi1300/macos_universal2/lib
      /Library/gurobi1200/macos_universal2/lib
      $ENV{GUROBI_HOME}/lib
      /opt/gurobi/lib
    NO_DEFAULT_PATH)
  if(NOT _ms_gurobi_lib)
    # 也在系统路径里找
    find_library(_ms_gurobi_lib NAMES gurobi130 gurobi120 gurobi110 gurobi100 gurobi)
  endif()
  if(_ms_gurobi_lib)
    set_property(TARGET mipsolvers::mipsolvers APPEND PROPERTY
      INTERFACE_LINK_LIBRARIES "${_ms_gurobi_lib}")
  else()
    message(STATUS
      "mipsolvers: Gurobi 未找到（非致命错误：Gurobi 是可选适配器，运行时不可用）。\n"
      "如需 Gurobi 支持，请设置 GUROBI_HOME 环境变量。")
  endif()
  unset(_ms_gurobi_lib)
endif()

# ── Ipopt / MUMPS（Apple 平台）────────────────────────────────────────────────
# 仅当构建时确实使用了 Homebrew ipopt cellar 的 MUMPS（非 vendored 构建）时，
# 才在消费者机器上追加这些 /opt/homebrew 传递依赖路径。
if(OFF AND APPLE AND OFF)
  # 在 Homebrew ipopt cellar 中查找 MUMPS 等传递依赖
  set(_ms_ipopt_mumps_dir "/opt/homebrew/opt/ipopt/lib")
  set(_ms_ipopt_deps)
  foreach(_ml libdmumps.dylib libmumps_common.dylib libmpiseq.dylib libpord.dylib)
    if(EXISTS "${_ms_ipopt_mumps_dir}/${_ml}")
      list(APPEND _ms_ipopt_deps "${_ms_ipopt_mumps_dir}/${_ml}")
    endif()
  endforeach()
  find_library(_ms_gfortran gfortran
    HINTS /opt/homebrew/opt/gcc/lib/gcc/current /opt/homebrew/lib
    NO_DEFAULT_PATH)
  if(_ms_gfortran)
    list(APPEND _ms_ipopt_deps "${_ms_gfortran}")
  endif()
  find_library(_ms_openblas openblas
    HINTS /opt/homebrew/opt/openblas/lib NO_DEFAULT_PATH)
  if(_ms_openblas)
    list(APPEND _ms_ipopt_deps "${_ms_openblas}")
  endif()
  if(_ms_ipopt_deps)
    set_property(TARGET mipsolvers::ipopt_local APPEND PROPERTY
      INTERFACE_LINK_LIBRARIES "-framework Accelerate;${_ms_ipopt_deps}")
  endif()
  unset(_ms_ipopt_mumps_dir)
  unset(_ms_ipopt_deps)
  unset(_ms_gfortran)
  unset(_ms_openblas)
endif()

check_required_components(mipsolvers)
