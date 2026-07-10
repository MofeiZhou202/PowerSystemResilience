#pragma once

/// sppt/agent.hpp
/// ==============
/// Verified Intelligent Modeling agent loop (Def. 8.7 / Alg. 2 / Thm. 8.9 of
/// docs/latex/sppt_theory.tex).  An agent proposes a sequence of edits to a
/// model; each candidate passes through the admissibility guard, and only
/// admissible edits are committed and analyzed.  The trajectory therefore never
/// certifies an ill-posed or unattributable state (loop soundness), and the
/// guard verdicts scored against ground-truth labels give scripted agent-edit
/// metrics for the LLM-ready interface (Pillar 5).

#include <functional>
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/sppt/guard.hpp"

namespace hacdcpf::sppt {

/// One proposed edit (a scripted stand-in for a future LLM/agent action).
struct AgentEdit {
  std::string name;                                 ///< human/NL description
  std::function<void(HybridPowerSystem&)> apply;    ///< the mutation
  bool expected_admissible{true};                   ///< ground-truth label
};

/// The outcome of processing one edit through the guard.
struct AgentStep {
  std::string edit_name;
  bool accepted{false};        ///< guard verdict
  std::string reason;          ///< failing gate when rejected
  bool applied{false};         ///< committed to the running model?
  bool analysis_ran{false};    ///< analysis executed on the committed model?
  bool analysis_converged{false};
  int  attributed_buses{0};    ///< bus voltages attributed back through R
};

/// A full agent trajectory plus its soundness flag and guard metrics.
struct AgentTrajectory {
  std::vector<AgentStep> steps;
  GuardMetrics metrics;   ///< guard verdicts vs expected_admissible labels
  bool sound{true};       ///< no committed step was ill-posed / unattributable
};

/// Run the guarded loop: apply only guard-admissible edits, analyze each
/// committed state, and score the guard against the edits' labels.
AgentTrajectory run_agent_loop(const HybridPowerSystem& seed,
                               const std::vector<AgentEdit>& edits);

/// Stable action identifiers exposed to tool-calling LLM adapters.
std::vector<std::string> agent_action_ids();

/// Convert one stable action identifier into the corresponding typed edit.
AgentEdit agent_edit_from_action_id(const std::string& id);

/// Convert a sequence of stable action identifiers into typed edits.
std::vector<AgentEdit> agent_script_from_action_ids(
    const std::vector<std::string>& ids);

/// A canned edit script (admissible + inadmissible) usable on any loaded AC
/// model, for the CLI and GUI demonstrations.
std::vector<AgentEdit> default_agent_script();

}  // namespace hacdcpf::sppt
