// Unit-discipline contract for the MATPOWER importer (§6.3).
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>

#include "hacdcpf/io/matpower_parser.hpp"

namespace {

// A minimal but valid MATPOWER case that also carries the two NON-STANDARD unit
// markers the importer recognizes (kW loads, ohmic branch impedance).
// Matrices come first (so the block parser latches onto the real data); the
// non-standard unit markers are trailing comments that has_*_conversion still
// finds via a plain text search.
constexpr const char* kCaseWithUnitMarkers = R"(function mpc = p3case
mpc.version = '2';
mpc.baseMVA = 100;
mpc.bus = [
  1  3  90  30  0  0  1  1.0  0  110  1  1.1  0.9;
  2  1  0   0   0  0  1  1.0  0  110  1  1.1  0.9;
];
mpc.gen = [
  1  0  0  100  -100  1.0  100  1  100  0;
];
mpc.branch = [
  1  2  1.21  12.1  0.0  100  100  100  0  0  1  -360  360;
];
% non-standard unit markers (loads in kW, branch impedance in ohms):
% mpc.bus(:, [PD, QD]) = mpc.bus(:, [PD, QD]) / 1e3;
% mpc.branch(:, [BR_R BR_X]) = ohms;
)";

std::string write_temp_case() {
  const auto path =
      std::filesystem::temp_directory_path() / "hacdcpf_p3_matpower.m";
  std::ofstream(path) << kCaseWithUnitMarkers;
  return path.string();
}

}  // namespace

TEST_CASE("MATPOWER twin path refuses heuristic unit inference",
          "[io][matpower][units]") {
  using namespace hacdcpf::io;
  const std::string path = write_temp_case();

  SECTION("default (twin path) does not apply markers and is not strict-clean") {
    const auto result =
        parse_matpower(path, MatpowerImportOptions{ImportMode::Strict, false});
    CHECK(result.report.unit_assertion == UnitAssertion::Inferred);
    CHECK(result.report.binding_level == ImportBindingLevel::Rich);
    // Non-standard units => Strict rejects, Permissive accepts-and-records.
    CHECK_FALSE(passes_mode(result.report, ImportMode::Strict));
    CHECK(passes_mode(result.report, ImportMode::Permissive));
    // The kW/ohm markers were recorded as skipped inferences.
    const bool skipped_inference = std::any_of(
        result.report.records.begin(), result.report.records.end(),
        [](const ImportRecord& r) {
          return r.disposition == ImportDisposition::Skipped &&
                 r.reason_code == ImportReasonCode::UnitInferred;
        });
    CHECK(skipped_inference);
  }

  SECTION("best_effort_import applies markers under reduced provenance") {
    const auto result = parse_matpower(
        path, MatpowerImportOptions{ImportMode::Permissive, true});
    CHECK(result.report.unit_assertion == UnitAssertion::BestEffort);
    CHECK_FALSE(passes_mode(result.report, ImportMode::Strict));
    const bool coerced = std::any_of(
        result.report.records.begin(), result.report.records.end(),
        [](const ImportRecord& r) {
          return r.disposition == ImportDisposition::Coerced &&
                 r.reason_code == ImportReasonCode::UnitInferred;
        });
    CHECK(coerced);
  }
}
