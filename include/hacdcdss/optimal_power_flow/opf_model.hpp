#pragma once
// hacdcdss/optimal_power_flow/opf_model.hpp
//
// Module: optimal_power_flow
//
// OPFModel provides two formulations:
//
//   DC OPF (run_dc)
//     Linearised DC approximation — lossless, ignores reactive power.
//     Formulated as a Linear Program (LPModel) and solved via HiGHS through
//     the MIPSolvers engine.
//     Decision variables: generator dispatch P_gen[i], bus angles θ[k].
//     Minimises: Σ_i (cost_b[i] * P_gen[i])   (linear cost, ignoring cost_a)
//     Subject to:
//       Power balance: B' * θ = P_gen − P_load  (net injection equations)
//       Branch limits: −rate ≤ b_ij*(θ_i − θ_j) ≤ rate
//       Generator limits: P_gen_min ≤ P_gen ≤ P_gen_max
//       Slack angle reference: θ_slack = 0
//
//   AC OPF (run_ac)
//     Full nonlinear AC OPF formulated as a Nonlinear Program (NLPModel) and
//     solved via Ipopt through the MIPSolvers engine (requires Ipopt present).
//     Decision variables: P_gen[i], Q_gen[i], V[k], θ[k].
//     Minimises: Σ_i (cost_a[i]*P_gen[i]² + cost_b[i]*P_gen[i] + cost_c[i])
//     Subject to: AC power-balance equations, voltage bounds, thermal limits.
//
// Both solvers use mipsolvers::engine::SolverEngine for dispatch to the
// appropriate backend (HiGHS for LP; Ipopt for NLP).

#include <hacdcdss/model/network_model.hpp>
#include <hacdcdss/model/scenario_data.hpp>
#include <hacdcdss/optimal_power_flow/opf_result.hpp>

#include <mipsolvers/engine/api/solver.hpp>

namespace hacdcdss::optimal_power_flow {

// ── OPF options ───────────────────────────────────────────────────────────────
struct OPFOptions {
    double  cost_a_scale    = 1.0;   // scaling factor for quadratic cost terms
    double  branch_rate_slack = 0.0; // relax thermal limits by this fraction
    bool    verbose         = false;
};

// ── OPF model and solver ──────────────────────────────────────────────────────
class OPFModel {
public:
    explicit OPFModel(mipsolvers::engine::SolverEngine& engine);

    /// Solve DC OPF (LP) for the nominal operating point.
    OPFResult run_dc(const model::NetworkModel& net,
                     const OPFOptions&          opts = {}) const;

    /// Solve DC OPF (LP) for a specific scenario (scales load/generation).
    OPFResult run_dc(const model::NetworkModel& net,
                     const model::Scenario&     scenario,
                     const OPFOptions&          opts = {}) const;

    /// Solve AC OPF (NLP) for the nominal operating point.
    /// Requires Ipopt to be compiled into the MIPSolvers library.
    OPFResult run_ac(const model::NetworkModel& net,
                     const OPFOptions&          opts = {}) const;

    /// Solve AC OPF (NLP) for a specific scenario.
    OPFResult run_ac(const model::NetworkModel& net,
                     const model::Scenario&     scenario,
                     const OPFOptions&          opts = {}) const;

private:
    mipsolvers::engine::SolverEngine& engine_;

    // Internal helpers.
    OPFResult solve_dc_impl(const model::NetworkModel& net,
                            const std::vector<double>& load_scale,
                            const OPFOptions&          opts) const;

    OPFResult solve_ac_impl(const model::NetworkModel& net,
                            const std::vector<double>& load_scale,
                            const OPFOptions&          opts) const;
};

} // namespace hacdcdss::optimal_power_flow
