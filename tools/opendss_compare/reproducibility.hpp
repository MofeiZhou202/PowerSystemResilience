#pragma once

namespace hacdcpf_compare {

inline constexpr const char* kCanonicalOpenDSSCompareBuildCommand =
    "cmake --build build/opendss_compare --target opendss_snapshot_adapter opendss_pf_compare "
    "test_opendss_pf_compare -j4";

inline constexpr const char* kCanonicalOpenDSSCompareTestCommand =
    "build/opendss_compare/test_opendss_pf_compare";

inline constexpr const char* kCanonicalOpenDSSSnapshotAdapterCommand =
    "build/opendss_compare/opendss_snapshot_adapter --master "
    "tests/data/opendss/minimal_regulator_1ph/Master.dss";

inline constexpr const char* kCanonicalOpenDSSCompareToolCommand =
    "build/opendss_compare/opendss_pf_compare";

inline constexpr int kAcceptedCaseGateExitCode = 2;

}  // namespace hacdcpf_compare
