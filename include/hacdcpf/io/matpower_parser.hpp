#pragma once
#include <string>

#include "hacdcpf/io/import_report.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::io {

/// Options controlling MATPOWER import unit discipline (§6.3).
struct MatpowerImportOptions {
  /// Strict rejects any inferred / best-effort unit handling; Permissive
  /// records it and proceeds.
  ImportMode mode{ImportMode::Permissive};
  /// When true, non-standard unit markers (kW loads, ohmic branch impedance)
  /// are converted heuristically and the model is stamped
  /// UnitAssertion::BestEffort.  When false (the twin path), such heuristics
  /// are forbidden: markers are reported but not applied.
  bool best_effort_import{false};
};

/// MATPOWER import result carrying the reconstructed system and a uniform
/// import report (§8).
struct MatpowerImportResult {
  HybridPowerSystem system;
  ImportReport report;
};

/// Parse a MATPOWER .m case file and return a HybridPowerSystem.
///
/// Legacy convenience overload: applies the non-standard kW/ohm conversion
/// markers if present (best-effort behavior), and emits no report.  Prefer the
/// options overload on the digital-twin path.
HybridPowerSystem parse_matpower(const std::string& filepath);

/// Parse a MATPOWER .m case file under an explicit unit-discipline policy.
///
/// MATPOWER is per-unit on @c mpc.baseMVA, so a standard file is
/// UnitAssertion::Asserted.  Non-standard kW/ohm markers are only applied under
/// @c best_effort_import (stamped BestEffort); otherwise they are reported and
/// left unapplied, and Strict mode does not accept the result.
MatpowerImportResult parse_matpower(const std::string& filepath,
                                    const MatpowerImportOptions& options);

}  // namespace hacdcpf::io
