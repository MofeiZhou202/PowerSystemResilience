/// test_sppt_agent.cpp
/// ===================
/// Verified Intelligent Modeling agent loop (Alg. 2 / Thm. 8.9): the guard admits
/// good edits and rejects hallucinated ones, the trajectory stays sound, and the
/// scripted agent-edit metrics are perfect on the labeled script.

#include <string>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/sppt/agent.hpp"
#include "hacdcpf/sppt/intelligent_simulation.hpp"

#ifndef HACDCPF_TEST_DATA_DIR
#define HACDCPF_TEST_DATA_DIR "../../data"
#endif

using namespace hacdcpf;

namespace {
std::string data_path(const std::string& name) {
  return std::string(HACDCPF_TEST_DATA_DIR) + "/" + name;
}
}  // namespace

TEST_CASE("Agent loop admits good edits, rejects hallucinated ones, stays sound",
          "[sppt][agent]") {
  HybridPowerSystem sys = io::parse_matpower(data_path("case9.m"));
  const auto traj = sppt::run_agent_loop(sys, sppt::default_agent_script());

  REQUIRE(traj.steps.size() == 4);

  // 1: scale loads — admissible, committed, analyzed, attributable.
  CHECK(traj.steps[0].accepted);
  CHECK(traj.steps[0].applied);
  CHECK(traj.steps[0].analysis_converged);
  CHECK(traj.steps[0].attributed_buses > 0);

  // 2: remove all references — hallucinated, must be rejected and NOT committed.
  CHECK_FALSE(traj.steps[1].accepted);
  CHECK_FALSE(traj.steps[1].applied);

  // 3, 4: admissible again.
  CHECK(traj.steps[2].accepted);
  CHECK(traj.steps[3].accepted);

  // Loop soundness (Thm. 8.9).
  CHECK(traj.sound);

  // Scripted agent-edit metrics on the labeled script.
  CHECK(traj.metrics.tp == 3);
  CHECK(traj.metrics.tn == 1);
  CHECK(traj.metrics.fp == 0);
  CHECK(traj.metrics.fn == 0);
  CHECK(traj.metrics.accuracy() == 1.0);
  CHECK(traj.metrics.catch_rate() == 1.0);
}

TEST_CASE("Agent loop never commits an inadmissible state", "[sppt][agent][soundness]") {
  HybridPowerSystem sys = io::parse_matpower(data_path("case14.m"));
  const auto traj = sppt::run_agent_loop(sys, sppt::default_agent_script());
  for (const auto& s : traj.steps) {
    if (s.applied) CHECK(s.accepted);              // committed => admissible
    if (s.accepted && s.analysis_ran) CHECK(s.attributed_buses > 0);
  }
  CHECK(traj.sound);
}

TEST_CASE("intelligent simulation contract gates uncertainty and authority",
          "[sppt][intelligent-simulation][uncertainty]") {
  sppt::IntelligentSimulationContract contract;
  contract.id = "hybrid-pf-read-only-v1";
  contract.requested_observables = {ObservableKind::ACBusVoltage,
                                    ObservableKind::DCBusVoltage};

  sppt::IntelligentSimulationEvidence evidence;
  evidence.guard.accepted = true;
  evidence.projection.mode = ProjectionMode::ExactIdeal;
  evidence.solver_converged = true;
  evidence.solver_residual = 1e-9;
  evidence.independent.supported = true;
  evidence.independent.converged = true;
  evidence.independent.total_residual = 2e-9;
  evidence.uncertainty.push_back(
      {"load forecast", sppt::UncertaintyKind::StandardDeviation,
       0.03, 0.99, "SCADA/forecast-v4"});

  const auto read_only =
      sppt::evaluate_intelligent_simulation_contract(contract, evidence);
  CHECK(read_only.disposition ==
        sppt::IntelligentSimulationDisposition::AdmitReadOnly);

  contract.authority = sppt::ActionAuthority::PhysicalControl;
  const auto control =
      sppt::evaluate_intelligent_simulation_contract(contract, evidence);
  CHECK(control.disposition ==
        sppt::IntelligentSimulationDisposition::RequireHumanReview);

  contract.authority = sppt::ActionAuthority::ReadOnlyAnalysis;
  evidence.uncertainty.front().relative_uncertainty = 0.30;
  const auto uncertain =
      sppt::evaluate_intelligent_simulation_contract(contract, evidence);
  CHECK(uncertain.disposition == sppt::IntelligentSimulationDisposition::Reject);
}
