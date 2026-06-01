# ─────────────────────────────────────────────────────────────────────────────
# mipsolvers_hfactor — vendored HiGHS HFactor as a standalone static library
# Part of U.7.118 Phase 1 (port HiGHS HFactor as faster basis-LU backend).
#
# This target compiles the vendored HFactor sources WITHOUT any modification.
# The header search path is set to THIS directory so the vendored files see
# their includes (`#include "util/HFactor.h"`, `"io/HighsIO.h"`, `"HConfig.h"`)
# as they would inside the HiGHS source tree.
#
# Phase 1 = compile only. No integration with mipsolvers yet.
# ─────────────────────────────────────────────────────────────────────────────

set(HFACTOR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")

set(HFACTOR_SOURCES
    util/HFactor.cpp
    util/HFactorRefactor.cpp
    util/HFactorExtend.cpp
    util/HFactorUtils.cpp
    util/HFactorDebug.cpp
    # NOTE: all other .cpp files that were previously listed here
    # (HVectorBase, HSet, HighsSparseMatrix, HighsSort, HighsUtils,
    #  HighsMatrixUtils, HighsHash, stringutil, HighsIO, HighsOptions)
    # are already compiled by the main `highs` target in highs/sources.cmake.
    # Compiling them again here with a different version of HighsOptions.h
    # triggers -Wodr (C++ One-Definition-Rule) violations at link time.
    # The symbols are resolved at final link time via the `highs` dependency.
)

add_library(mipsolvers_hfactor STATIC ${HFACTOR_SOURCES})

# HFactor sources do `#include "util/HFactor.h"`, `#include "HConfig.h"`, etc.
# ${PROJECT_SOURCE_DIR}/highs is listed FIRST so hfactor sees the same
# HighsOptions.h (with hacdcpf custom fields) as the rest of the project.
# Without this, the local highs_factor/lp_data/HighsOptions.h (an older
# snapshot) would be picked up, causing -Wodr violations at link time.
target_include_directories(mipsolvers_hfactor PRIVATE
    ${PROJECT_SOURCE_DIR}/highs
    ${HFACTOR_ROOT})

# Match HiGHS' C++ standard (C++17 used by mipsolvers as well).
target_compile_features(mipsolvers_hfactor PUBLIC cxx_std_17)

# HiGHS uses some non-standard pragmas / unused warnings that we don't want
# to inherit. Treat the vendored code as third-party: relax warnings.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(mipsolvers_hfactor PRIVATE
      -Wno-unused-parameter
      -Wno-unused-variable
      -Wno-sign-compare
      -Wno-unused-but-set-variable
      -Wno-deprecated-declarations
  )
endif()

# Position-independent code so we can later link this into mipsolvers (a shared
# or large static lib).
set_target_properties(mipsolvers_hfactor PROPERTIES POSITION_INDEPENDENT_CODE ON)
