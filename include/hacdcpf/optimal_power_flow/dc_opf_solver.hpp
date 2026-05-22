#pragma once

/// optimal_power_flow/dc_opf_solver.hpp
/// =======================================
/// Top-level DC OPF solver entry-point.
/// Replaces: optimal_power_flow/dc_opf.hpp.

#include <tuple>
#include <string>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"

namespace hacdcpf::opf {

DCOPFResult solve_dc_opf(const HybridPowerSystem& sys,
                         const DCOPFOptions& opts = {});

/// Check DC OPF feasibility against a solved result.
/// Returns: {feasible, max_violation, status_message}
std::tuple<bool, double, std::string>
check_dc_opf_feasibility(const HybridPowerSystem& sys,
                         const DCOPFResult& result,
                         double tol = 1e-6);

}  // namespace hacdcpf::opf
