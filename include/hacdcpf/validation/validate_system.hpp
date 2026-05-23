#pragma once

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/validation/validation_report.hpp"

namespace hacdcpf::validation {

// ── Validation level ─────────────────────────────────────────────────────────

/// Controls which checks are performed and how warnings are treated.
enum class ValidationLevel {
    /// Only Error-severity structural checks: required fields, bus
    /// references, branch endpoints.  All Warnings are suppressed.
    Basic,
    /// Errors + system-level topology and slack-bus checks.
    /// Most useful for importers and converters.
    Electrical,
    /// Full validation — all checks at their natural severity.
    /// This is the behaviour of the zero-argument validate() overload.
    SolverReady,
    /// Full validation with all Warnings promoted to Errors.
    /// Use before a final solve when no ambiguity should be tolerated.
    Strict,
};

/// Run all static checks on \p sys and return a ValidationReport.
///
/// Checks performed:
///   - Duplicate bus IDs (AC and DC)
///   - Branches referencing missing buses
///   - Isolated buses (no connected branch)
///   - Invalid voltage limits (Vmin >= Vmax, Vmin <= 0)
///   - Generator limits inconsistent with initial dispatch
///   - Generator Qmin > Qmax or Pmin > Pmax
///   - Slack bus missing (no SLACK-type bus and no is_slack generator)
///   - Multiple slack buses without distributed-slack being configured
///   - Negative or zero branch impedance where not allowed
///   - Transformer tap ratio outside (0.5, 2.0)
///   - VSC converter references to missing AC or DC buses
///   - DC branch references to missing DC buses
///   - Negative resistance on AC branches
///   - base_mva <= 0
///   - ACSystem / DCSystem base_mva mismatch vs HybridPowerSystem base_mva
ValidationReport validate(const HybridPowerSystem& sys);

/// Run checks appropriate for \p level.
///
/// - Basic      : only Error-severity field/reference/limit checks.
/// - Electrical : same as Basic plus slack-bus and topology Warnings.
/// - SolverReady: identical to validate(sys) (all checks, all severities).
/// - Strict     : all checks, with Warnings promoted to Errors.
ValidationReport validate(const HybridPowerSystem& sys, ValidationLevel level);

}  // namespace hacdcpf::validation
