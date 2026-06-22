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

/// Assemble and solve AC OPF through the parity full-space IPM path.
///
/// This high-level driver calls `build_problem`, `build_variable_bounds`,
/// `build_initial_point`, and `solve_primal_dual_ipm`, then maps the solved
/// vector back into `ACOPFResult`.  It is the preferred path for code review of
/// the unified hybrid formulation because all variables and constraint rows are
/// explicit in `Problem::vidx` and `Problem::cidx`.
///
/// @param sys Rich hybrid power-system input.
/// @param opts AC OPF tolerances and converter constraint-family toggles.
/// @return OPF result with `solver_path == OPFSolverPath::ParityIPM` when this
/// path is used successfully.
ACOPFResult solve_ac_opf_parity(const HybridPowerSystem& sys,
                                 const ACOPFOptions& opts = {});

}  // namespace hacdcpf::opf::parity
