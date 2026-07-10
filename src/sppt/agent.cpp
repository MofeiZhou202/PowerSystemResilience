/// sppt/agent.cpp
/// ==============
/// Verified Intelligent Modeling agent loop (see agent.hpp).

#include "hacdcpf/sppt/agent.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/model/enums/bus_types.hpp"
#include "hacdcpf/sppt/guard.hpp"

namespace hacdcpf::sppt {
namespace {

AgentEdit make_scale_loads_10pct() {
  return {"scale all loads +10%",
          [](HybridPowerSystem& s) {
            for (auto& ld : s.ac.loads) {
              ld.p_mw *= 1.10;
              ld.q_mvar *= 1.10;
            }
          },
          /*expected_admissible=*/true};
}

AgentEdit make_remove_ac_references() {
  return {"remove all AC angle references (hallucinated deletion)",
          [](HybridPowerSystem& s) {
            for (auto& b : s.ac.buses)
              if (b.bus_type == BusType::SLACK) b.bus_type = BusType::PQ;
            for (auto& g : s.ac.generators) g.is_slack = false;
            for (auto& eg : s.ac.external_grids) eg.in_service = false;
          },
          /*expected_admissible=*/false};
}

AgentEdit make_restore_slack_reference() {
  return {"restore a slack reference at the first bus",
          [](HybridPowerSystem& s) {
            if (!s.ac.buses.empty())
              s.ac.buses.front().bus_type = BusType::SLACK;
            if (!s.ac.generators.empty()) {
              s.ac.generators.front().is_slack = true;
              if (!s.ac.buses.empty())
                s.ac.generators.front().bus = s.ac.buses.front().index;
            }
          },
          /*expected_admissible=*/true};
}

AgentEdit make_trim_generation_headroom_5pct() {
  return {"trim generation headroom -5%",
          [](HybridPowerSystem& s) {
            for (auto& g : s.ac.generators) g.pg_mw *= 0.95;
          },
          /*expected_admissible=*/true};
}

AgentEdit make_no_op() {
  return {"no operation",
          [](HybridPowerSystem&) {},
          /*expected_admissible=*/true};
}

}  // namespace

AgentTrajectory run_agent_loop(const HybridPowerSystem& seed,
                               const std::vector<AgentEdit>& edits) {
  AgentTrajectory traj;
  HybridPowerSystem current = seed;

  for (const auto& edit : edits) {
    AgentStep step;
    step.edit_name = edit.name;

    // Propose the edit on a candidate copy (Alg. 2, line 1).
    HybridPowerSystem candidate = current;
    if (edit.apply) edit.apply(candidate);

    // Guard gates (Alg. 2, lines 2-7).
    const GuardVerdict v = guard_system(candidate);
    step.accepted = v.accepted;
    step.reason = v.reason;

    // Score the guard against the ground-truth label (Pillar 5).
    if (edit.expected_admissible && v.accepted) ++traj.metrics.tp;
    else if (!edit.expected_admissible && v.accepted) ++traj.metrics.fp;
    else if (!edit.expected_admissible && !v.accepted) ++traj.metrics.tn;
    else ++traj.metrics.fn;

    if (v.accepted) {
      // Commit and analyze (Alg. 2, lines 8-9).
      current = candidate;
      step.applied = true;
      const PowerFlowResult pf = solve_power_flow(current);
      step.analysis_ran = true;
      step.analysis_converged = pf.converged;
      step.attributed_buses = static_cast<int>(pf.vm.size());
    }

    // Loop soundness (Thm. 8.9): a committed step must be admissible, and any
    // analyzed committed state must be attributable.
    if (step.applied && !step.accepted) traj.sound = false;
    if (step.accepted && step.analysis_ran && step.attributed_buses == 0)
      traj.sound = false;

    traj.steps.push_back(std::move(step));
  }
  return traj;
}

std::vector<std::string> agent_action_ids() {
  return {"scale_loads_10pct",
          "remove_ac_references",
          "restore_slack_reference",
          "trim_generation_headroom_5pct",
          "no_op"};
}

AgentEdit agent_edit_from_action_id(const std::string& id) {
  if (id == "scale_loads_10pct") return make_scale_loads_10pct();
  if (id == "remove_ac_references") return make_remove_ac_references();
  if (id == "restore_slack_reference") return make_restore_slack_reference();
  if (id == "trim_generation_headroom_5pct")
    return make_trim_generation_headroom_5pct();
  if (id == "no_op") return make_no_op();
  throw std::invalid_argument("unknown SPPT agent action id: " + id);
}

std::vector<AgentEdit> agent_script_from_action_ids(
    const std::vector<std::string>& ids) {
  std::vector<AgentEdit> script;
  script.reserve(ids.size());
  for (const auto& id : ids) script.push_back(agent_edit_from_action_id(id));
  return script;
}

std::vector<AgentEdit> default_agent_script() {
  return agent_script_from_action_ids({"scale_loads_10pct",
                                       "remove_ac_references",
                                       "restore_slack_reference",
                                       "trim_generation_headroom_5pct"});
}

}  // namespace hacdcpf::sppt
