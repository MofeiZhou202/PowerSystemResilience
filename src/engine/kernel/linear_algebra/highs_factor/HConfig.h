// Vendored HConfig stub for HFactor port (U.7.118 Phase 1).
// Mirrors HiGHS/build/HConfig.h but excludes integration-specific defines
// we don't need (FAST_BUILD, ZLIB_FOUND, CUPDLP_*, HIPO, CMAKE_BUILD_TYPE).
#ifndef HCONFIG_H_
#define HCONFIG_H_

#if defined(__GNUC__) || defined(__clang__)
#define HIGHS_HAVE_BUILTIN_CLZ
#elif defined(_MSC_VER)
#define HIGHS_HAVE_BITSCAN_REVERSE
#endif

#define HIGHS_GITHASH "vendored-from-dcc25308d"
#define HIGHS_VERSION_MAJOR 1
#define HIGHS_VERSION_MINOR 14
#define HIGHS_VERSION_PATCH 0

// 64-bit index type (HighsInt = int64_t): raises the factor-nnz ceiling
// from 2^31 to 2^63 for the vendored HFactor simplex backend.
#define HIGHSINT64

#endif /* HCONFIG_H_ */
