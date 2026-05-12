#pragma once
// hacdcdss/power_flow/pf_solver.hpp
//
// Module: power_flow
//
// PFSolver implements Newton-Raphson power flow for hybrid AC/DC distribution
// networks.  The algorithm follows these steps:
//
//   AC-only (no DC buses or converters):
//     Standard Newton-Raphson on the [ΔP; ΔQ] mismatch equations, solved
//     using the NLESolver from MIPSolvers.
//
//   Hybrid AC/DC (with VSC converters):
//     Fully coupled Newton-Raphson where the state vector is extended with
//     DC bus voltages.  VSC converters appear as controllable injections that
//     link the AC and DC residual equations.
//
//   For each scenario, load and generation are scaled before solving; the
//   nominal operating point can be solved without scaling via run_nominal().
//
// Solver backend: mipsolvers::engine::NLESolver (domain-agnostic NR with
// dogleg trust-region globalisation, sparse Jacobian).

#include <hacdcdss/model/network_model.hpp>
#include <hacdcdss/model/scenario_data.hpp>
#include <hacdcdss/power_flow/pf_result.hpp>

#include <mipsolvers/engine/solver/native/nle/nle_solver.hpp>

namespace hacdcdss::power_flow {

// ── Solver options ────────────────────────────────────────────────────────────
struct PFOptions {
    int    max_iter   = 50;
    double tol        = 1e-6;   // convergence criterion: ‖F(x)‖∞ < tol [p.u.]
    bool   flat_start = true;   // initialise V = 1∠0 if true
    bool   verbose    = false;  // print iteration log to stdout
};

// ── Power-flow solver ─────────────────────────────────────────────────────────
class PFSolver {
public:
    PFSolver() = default;

    /// Run power flow for a given network and scenario.
    /// Scenario load_scale / pv_scale / wind_scale are applied multiplicatively
    /// to the base-case injections before solving.
    PFResult run(const model::NetworkModel& net,
                 const model::Scenario&     scenario,
                 const PFOptions&           opts = {}) const;

    /// Run power flow at the nominal operating point (no scenario scaling).
    PFResult run_nominal(const model::NetworkModel& net,
                         const PFOptions&           opts = {}) const;

private:
    // AC-only Newton-Raphson.
    PFResult solve_ac(const model::NetworkModel& net,
                      const PFOptions&           opts) const;

    // Coupled AC/DC Newton-Raphson.
    PFResult solve_ac_dc(const model::NetworkModel& net,
                         const PFOptions&           opts) const;
};

} // namespace hacdcdss::power_flow
