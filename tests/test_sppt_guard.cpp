/// test_sppt_guard.cpp
/// ===================
/// The SPPT admissibility guard (Def. 8.7, Alg. 2) and scripted agent-edit
/// evaluation metrics for the LLM-ready interface (Pillar 5).

#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/model/enums/bus_types.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/sppt/guard.hpp"

#ifndef HACDCPF_TEST_DATA_DIR
#define HACDCPF_TEST_DATA_DIR "../../data"
#endif

using namespace hacdcpf;

namespace {

std::string data_path(const std::string& name) {
  return std::string(HACDCPF_TEST_DATA_DIR) + "/" + name;
}

// Remove the AC angle reference — the canonical inadmissible edit.
HybridPowerSystem break_reference(HybridPowerSystem sys) {
  for (auto& b : sys.ac.buses)
    if (b.bus_type == BusType::SLACK) b.bus_type = BusType::PQ;
  for (auto& g : sys.ac.generators) g.is_slack = false;
  for (auto& eg : sys.ac.external_grids) eg.in_service = false;
  return sys;
}

}  // namespace

TEST_CASE("guard accepts a well-posed, attributable system", "[sppt][guard]") {
  HybridPowerSystem sys = io::parse_matpower(data_path("case9.m"));
  const auto v = sppt::guard_system(sys);
  INFO("reason=" << v.reason);
  CHECK(v.validation_ok);
  CHECK(v.well_posed);
  CHECK(v.attribution_total);
  CHECK(v.accepted);
}

TEST_CASE("guard rejects a reference-less system at the well-posedness gate",
          "[sppt][guard]") {
  HybridPowerSystem sys = break_reference(io::parse_matpower(data_path("case9.m")));
  const auto v = sppt::guard_system(sys);
  INFO("reason=" << v.reason);
  CHECK_FALSE(v.accepted);
  // Either the validation gate or the well-posedness gate must catch it.
  CHECK((v.reason == "validation" || v.reason == "ill-posed reference"));
  CHECK_FALSE(v.well_posed);
}

TEST_CASE("guard metrics over a labeled edit set", "[sppt][guard][metrics]") {
  std::vector<sppt::LabeledEdit> edits;
  for (const char* c : {"case9.m", "case14.m", "case30.m"}) {
    edits.push_back({std::string("ok_") + c, io::parse_matpower(data_path(c)), true});
    edits.push_back({std::string("broken_") + c,
                     break_reference(io::parse_matpower(data_path(c))), false});
  }

  const auto m = sppt::evaluate_guard(edits);
  INFO("tp=" << m.tp << " fp=" << m.fp << " tn=" << m.tn << " fn=" << m.fn);
  CHECK(m.tp == 3);   // all admissible accepted
  CHECK(m.tn == 3);   // all inadmissible caught
  CHECK(m.fp == 0);
  CHECK(m.fn == 0);
  CHECK(m.accuracy() == 1.0);
  CHECK(m.catch_rate() == 1.0);   // hallucination-catch rate
  CHECK(m.precision() == 1.0);
  CHECK(m.recall() == 1.0);
}

TEST_CASE("guard evaluates attribution for the requested observable",
          "[sppt][guard][attribution]") {
  HybridPowerSystem sys = io::parse_matpower(data_path("case9.m"));
  const auto accepted = sppt::guard_system(
      sys, {ObservableKind::ACBusVoltage, ObservableKind::NodalDual});
  CHECK(accepted.accepted);
  REQUIRE(accepted.attribution.size() == 2);
  CHECK(accepted.attribution_total);

  const auto rejected =
      sppt::guard_system(sys, {ObservableKind::ServiceLoss});
  CHECK_FALSE(rejected.accepted);
  CHECK_FALSE(rejected.attribution_total);
  REQUIRE(rejected.attribution.size() == 1);
  CHECK(rejected.attribution.front().recovery == RecoveryClass::Unsupported);
}
