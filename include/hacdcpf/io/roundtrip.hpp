#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"

/// @file roundtrip.hpp
/// Round-trip / IO conformance contract (docs/digital_twin_data_io_architecture.md §7).
///
/// A working IO layer must satisfy import(export(M)) ≈ M within a declared
/// tolerance.  This header provides a generic, self-contained field-level diff
/// (via the native JSON contract) that produces stored evidence the digital-twin
/// NumericalValidation dimension consumes (§7.2).  It is dependency-light: the
/// nlohmann/json machinery lives entirely in roundtrip.cpp.

namespace hacdcpf::io {

/// One field that failed to survive a round-trip.
struct FieldDiff {
  std::string path;    ///< JSON path, e.g. "ac.branches[3].x_pu".
  std::string before;
  std::string after;
};

/// Stored evidence from one round-trip conformance run (§7.2).
struct RoundTripEvidence {
  std::string adapter{"json"};  ///< "json" | "canonical" | "etap" | ...
  std::string level{"rich"};    ///< "rich" | "canonical" binding level.
  bool lossless{false};
  bool passed{false};           ///< lossless within @c epsilon.
  double epsilon{1e-6};
  std::size_t fields_checked{0};
  std::size_t fields_mismatched{0};
  std::vector<FieldDiff> mismatches;  ///< capped sample of mismatches.
};

/// Field-level diff of two systems via their native JSON projection.
///
/// Numbers compare within @p epsilon (absolute, scaled by magnitude); strings,
/// booleans, and null compare exactly.  Missing/extra keys count as mismatches.
RoundTripEvidence diff_systems(const HybridPowerSystem& before,
                               const HybridPowerSystem& after,
                               double epsilon = 1e-6,
                               std::string adapter = "json",
                               std::string level = "rich");

/// Self-contained JSON round-trip: diff_systems(sys, from_json(to_json(sys))).
/// This is the gate evidence for fidelity level F3 (§4, §7).
RoundTripEvidence json_roundtrip(const HybridPowerSystem& sys,
                                 double epsilon = 1e-6);

}  // namespace hacdcpf::io
