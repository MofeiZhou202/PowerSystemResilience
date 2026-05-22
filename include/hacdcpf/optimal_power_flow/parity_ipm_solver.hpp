#pragma once

/// optimal_power_flow/parity_ipm_solver.hpp
/// ==========================================
/// High-level driver: assemble parity Problem, run IPM, and map back to
/// ACOPFResult.  Companion to native_ipm_solver.hpp.
/// Replaces the high-level path in: optimal_power_flow/ac_opf.hpp.

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/formulation.hpp"
#include "hacdcpf/optimal_power_flow/native_ipm_solver.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"

namespace hacdcpf::opf::parity {

/// Solve AC OPF via the parity full-space IPM path.
ACOPFResult solve_ac_opf_parity(const HybridPowerSystem& sys,
                                 const ACOPFOptions& opts = {});

}  // namespace hacdcpf::opf::parity
