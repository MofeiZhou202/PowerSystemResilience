#pragma once

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/validation/validation_report.hpp"

namespace hacdcpf::validation {

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

}  // namespace hacdcpf::validation
