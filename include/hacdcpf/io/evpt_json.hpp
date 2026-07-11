#pragma once

// io/evpt_json.hpp
// ──────────────────────────────────────────────────────────────────────────
// JSON serialisation for the EV power-traffic coupling layer (hacdcpf::evpt).
//
// Problem side: the traffic graph, routes, demands, and station prices are
// parsed from JSON; the power system (EVPowerTrafficProblem::system) is
// supplied separately by the caller (typically the GUI session's current
// system or a built-in demo scenario).
//
// Result side: emits GUI-oriented JSON.  Heavy fields (per-step power-system
// copies, full OPF results) are reduced to compact summaries; per-link CTM
// time series are downsampled to a bounded cell budget with a "truncated"
// flag so the client knows when it is looking at a reduced grid.

#include <nlohmann/json.hpp>

#include "hacdcpf/ev_power_traffic/options.hpp"
#include "hacdcpf/ev_power_traffic/simulation.hpp"
#include "hacdcpf/ev_power_traffic/types.hpp"

namespace hacdcpf::io {

// ── Problem / scenario ─────────────────────────────────────────────────────

/// Parse traffic graph, routes, demands, ICV demands, and station prices from
/// `j` into `problem`.  Does NOT touch problem.system.  Unknown fields are
/// ignored; missing fields keep struct defaults.  Throws nlohmann::json
/// exceptions on type mismatches.
void evpt_problem_from_json(const nlohmann::json& j,
                            evpt::EVPowerTrafficProblem& problem);

/// Scenario echo for rendering: traffic nodes (with coordinates when set),
/// links, routes, demands, and the charging-station → bus mapping taken from
/// problem.system.
nlohmann::json evpt_scenario_to_json(const evpt::EVPowerTrafficProblem& problem);

// ── Options ────────────────────────────────────────────────────────────────

evpt::EVPowerTrafficOptions evpt_options_from_json(const nlohmann::json& j);
evpt::CTMOptions evpt_ctm_options_from_json(const nlohmann::json& j);
evpt::DUEOptions evpt_due_options_from_json(const nlohmann::json& j);
evpt::CTMJointWelfareOptions evpt_ctm_joint_options_from_json(
    const nlohmann::json& j);
evpt::JointOptimizerOptions evpt_joint_optimizer_options_from_json(
    const nlohmann::json& j);

// ── Results ────────────────────────────────────────────────────────────────

/// Cap on links × steps cells emitted for the per-link CTM time series.
/// Above the cap, steps are strided and "truncated": true is set.
inline constexpr int kEvptMaxLinkStepCells = 200000;

nlohmann::json evpt_result_to_json(const evpt::EVPowerTrafficResult& r);
nlohmann::json evpt_ctm_due_result_to_json(
    const evpt::CTMDUEResult& r, int max_link_step_cells = kEvptMaxLinkStepCells);
nlohmann::json evpt_ctm_joint_result_to_json(
    const evpt::CTMJointWelfareResult& r,
    int max_link_step_cells = kEvptMaxLinkStepCells);
nlohmann::json evpt_joint_optimizer_result_to_json(
    const evpt::JointOptimizerResult& r);

}  // namespace hacdcpf::io
