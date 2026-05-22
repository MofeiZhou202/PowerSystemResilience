#pragma once

/// optimal_power_flow/ac_opf_solver.hpp
/// =====================================
/// Forward declarations of AC OPF solver entry points.
/// Replaces: optimal_power_flow/ac_opf.hpp

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"

namespace hacdcpf::opf {

/// Solve the AC OPF for the given system.
ACOPFResult solve_ac_opf(const HybridPowerSystem& sys,
                         const ACOPFOptions& opt = {});

/// Compute Jacobian diagnostic metrics at the current operating point.
ACOPFJacobianDiagnostics compute_jacobian_diagnostics(const HybridPowerSystem& sys,
                                                       const ACOPFOptions& opt = {});

}  // namespace hacdcpf::opf
