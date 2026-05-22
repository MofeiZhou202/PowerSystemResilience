#pragma once

/// PowerFlowProblem
/// ================
/// A discrete, self-contained description of one power-flow solve task.
/// It is produced by the projection layer and consumed by any
/// IPowerFlowSolver implementation, decoupling "what to solve" from
/// "how to solve it".
///
/// Usage:
///   HybridPowerSystem sys = ...;
///   PowerFlowOptions  opt = ...;
///   PowerFlowProblem  prob = build_power_flow_problem(sys, opt);
///   auto result = solver.solve(prob);

#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/assembly/solver_data.hpp"

namespace hacdcpf {

// ── Problem descriptor ────────────────────────────────────────────────────────

struct PowerFlowProblem {
    powerflow::SolverData network;   ///< Canonicalised network in solver-assembly format
    PowerFlowOptions      options;   ///< Solver settings for this particular problem

    // ── Metadata ─────────────────────────────────────────────────────────────
    std::string name;                ///< Optional human-readable case name
    int         n_ac_buses{0};
    int         n_dc_buses{0};
    int         n_converters{0};
};

// ── Factory function ──────────────────────────────────────────────────────────

/// Project \p sys to a \c PowerFlowProblem ready for solving.
/// This encapsulates:
///   1. Bus-merging (switches, zero-impedance branches)
///   2. Transformer/device projection to canonical AC/DC branches
///   3. Component aggregation (loads, ZIP, storage, REN, EV, etc.)
///   4. Slack generator assignment
///
/// Throws if the system is structurally invalid.
PowerFlowProblem build_power_flow_problem(
    const HybridPowerSystem& sys,
    const PowerFlowOptions& options = {});

}  // namespace hacdcpf
