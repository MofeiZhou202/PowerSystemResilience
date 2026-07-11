/// test_sppt_metamorphic.cpp
/// =========================
/// Executable metamorphic relations MR1-MR7 realizing the theorems of
/// docs/latex/sppt_theory.tex (Pillars 1-2 of the verification program).

#include <string>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"
#include "hacdcpf/sppt/sppt.hpp"

#ifndef HACDCPF_TEST_DATA_DIR
#define HACDCPF_TEST_DATA_DIR "../../data"
#endif

using namespace hacdcpf;

namespace {

std::string data_path(const std::string& name) {
  return std::string(HACDCPF_TEST_DATA_DIR) + "/" + name;
}

ACBus make_bus(int idx, BusType type) {
  ACBus b;
  b.index = idx;
  b.bus_type = type;
  b.vm_pu = 1.0;
  b.va_deg = 0.0;
  b.in_service = true;
  return b;
}

Generator make_slack_gen(int idx, int bus) {
  Generator g;
  g.index = idx;
  g.bus = bus;
  g.is_slack = true;
  g.in_service = true;
  g.vg_pu = 1.0;
  g.pg_mw = 40.0;
  g.pmax_mw = 200.0;
  g.pmin_mw = 0.0;
  g.qmax_mvar = 100.0;
  g.qmin_mvar = -100.0;
  return g;
}

ACBranch make_branch(int idx, int f, int t, double r, double x) {
  ACBranch br;
  br.index = idx;
  br.from_bus = f;
  br.to_bus = t;
  br.r_pu = r;
  br.x_pu = x;
  br.b_pu = 0.0;
  br.tap = 1.0;
  br.in_service = true;
  return br;
}

Load make_load(int idx, int bus, double p, double q) {
  Load ld;
  ld.index = idx;
  ld.bus = bus;
  ld.p_mw = p;
  ld.q_mvar = q;
  ld.in_service = true;
  return ld;
}

// 3-bus feeder with a closed switch (bus 1--2) that projection contracts into a
// single canonical bus (switch presence is what enables the zero-impedance
// merge, per project_to_canonical_models).
HybridPowerSystem make_zero_impedance_case() {
  HybridPowerSystem sys;
  sys.name = "sppt_zeroZ";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ),
                  make_bus(3, BusType::PQ)};
  sys.ac.branches = {make_branch(2, 2, 3, 0.01, 0.05)};  // real line 2--3

  Switch sw;  // closed switch 1--2: contracted into one canonical bus
  sw.index = 1;
  sw.bus_from = 1;
  sw.bus_to = 2;
  sw.closed = true;
  sw.in_service = true;
  sys.ac.switches = {sw};

  sys.ac.generators = {make_slack_gen(1, 1)};
  sys.ac.loads = {make_load(1, 3, 30.0, 10.0)};
  return sys;
}

// One self-contained area on a disjoint bus-id range.
HybridPowerSystem make_area(int base_id) {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;
  sys.ac.buses = {make_bus(base_id + 1, BusType::SLACK),
                  make_bus(base_id + 2, BusType::PQ)};
  sys.ac.branches = {make_branch(base_id + 1, base_id + 1, base_id + 2, 0.01, 0.05)};
  sys.ac.generators = {make_slack_gen(base_id + 1, base_id + 1)};
  sys.ac.loads = {make_load(base_id + 1, base_id + 2, 25.0, 8.0)};
  return sys;
}

}  // namespace

TEST_CASE("MR1: projection is idempotent", "[sppt][metamorphic][mr1]") {
  for (const char* c : {"case9.m", "case14.m", "case30.m"}) {
    HybridPowerSystem sys = io::parse_matpower(data_path(c));
    const auto r = sppt::mr1_projection_idempotence(sys);
    INFO(c << ": " << r.detail);
    CHECK(r.passed);
    CHECK(r.residual == 0.0);
  }
}

TEST_CASE("MR3: power-flow semantics are preserved through projection",
          "[sppt][metamorphic][mr3]") {
  for (const char* c : {"case9.m", "case14.m", "case30.m", "case57.m"}) {
    HybridPowerSystem sys = io::parse_matpower(data_path(c));
    const auto r = sppt::mr3_semantic_preservation_pf(sys, 1e-6);
    INFO(c << ": residual=" << r.residual << " (" << r.detail << ")");
    CHECK(r.passed);
    CHECK(r.residual < 1e-6);
  }
}

TEST_CASE("MR3d: OPF nodal prices are preserved through projection",
          "[sppt][metamorphic][mr3][opf]") {
  HybridPowerSystem sys = io::parse_matpower(data_path("case9.m"));
  const auto r = sppt::mr3_semantic_preservation_opf_dual(sys, 1e-6);
  INFO("case9 LMP: residual=" << r.residual << " (" << r.detail << ")");
  CHECK(r.passed);
}

TEST_CASE("MR2 / MR4: attribution round-trip and merged-bus equipotential",
          "[sppt][metamorphic][mr2][mr4]") {
  HybridPowerSystem sys = make_zero_impedance_case();
  // Sanity: the zero-impedance tie really collapses under projection.
  CHECK(n_ac_buses(project_to_canonical_models(sys)) < n_ac_buses(sys));

  const auto mr2 = sppt::mr2_attribution_roundtrip(sys);
  INFO("MR2: " << mr2.detail);
  CHECK(mr2.passed);

  const auto mr4 = sppt::mr4_merged_bus_equipotential(sys);
  INFO("MR4: " << mr4.detail);
  CHECK(mr4.passed);
}

TEST_CASE("projection modes classify ideal and threshold contractions",
          "[sppt][projection][provenance]") {
  HybridPowerSystem ideal = make_zero_impedance_case();
  ProjectionOptions exact_options;
  exact_options.mode = ProjectionMode::ExactIdeal;
  const HybridPowerSystem exact = project_to_canonical_models(ideal, exact_options);
  REQUIRE(exact.projection_certificate.has_value());
  CHECK(exact.projection_certificate->exact_merge_count() == 1);
  CHECK(exact.projection_certificate->approximate_merge_count() == 0);
  CHECK(n_ac_buses(exact) == 2);

  HybridPowerSystem nonideal = ideal;
  nonideal.ac.switches.front().r_contact_ohm = 1e-8;
  nonideal.ac.switches.front().z_ohm = 1e-8;
  const HybridPowerSystem exact_nonideal =
      project_to_canonical_models(nonideal, exact_options);
  CHECK(n_ac_buses(exact_nonideal) == 3);

  ProjectionOptions approximate_options;
  approximate_options.mode = ProjectionMode::ThresholdApproximate;
  const HybridPowerSystem approximate =
      project_to_canonical_models(nonideal, approximate_options);
  REQUIRE(approximate.projection_certificate.has_value());
  CHECK(approximate.projection_certificate->approximate_merge_count() == 1);
  CHECK(n_ac_buses(approximate) == 2);

  const auto voltage =
      evaluate_attribution(nonideal, approximate, ObservableKind::ACBusVoltage);
  CHECK(voltage.total());
  CHECK(voltage.recovery == RecoveryClass::Approximate);
  const auto switch_flow =
      evaluate_attribution(nonideal, approximate, ObservableKind::SwitchTerminalFlow);
  CHECK(switch_flow.total());
  CHECK(switch_flow.recovery == RecoveryClass::Approximate);
}

TEST_CASE("MR5: solutions are invariant to bus relabeling",
          "[sppt][metamorphic][mr5]") {
  for (const char* c : {"case9.m", "case14.m"}) {
    HybridPowerSystem sys = io::parse_matpower(data_path(c));
    const auto r = sppt::mr5_relabel_invariance(sys, 1e-6);
    INFO(c << ": residual=" << r.residual << " (" << r.detail << ")");
    CHECK(r.passed);
  }
}

TEST_CASE("MR6: projection distributes over disjoint union",
          "[sppt][metamorphic][mr6]") {
  const HybridPowerSystem a = make_area(0);
  const HybridPowerSystem b = make_area(100);
  const auto r = sppt::mr6_compositionality(a, b);
  INFO("MR6: " << r.detail);
  CHECK(r.passed);
  CHECK(r.residual == 0.0);
}

TEST_CASE("MR7: removing a reference produces a typed rejection, not a silent answer",
          "[sppt][metamorphic][mr7]") {
  for (const char* c : {"case9.m", "case14.m"}) {
    HybridPowerSystem sys = io::parse_matpower(data_path(c));
    // Precondition: the authored system is admissible.
    REQUIRE(validate_full(sys).is_valid());
    const auto r = sppt::mr7_wellposedness_gate(sys);
    INFO(c << ": " << r.detail);
    CHECK(r.passed);
  }
}

TEST_CASE("SPPT core suite runs and reports on a hybrid case",
          "[sppt][metamorphic][suite]") {
  HybridPowerSystem sys = io::parse_matpower(data_path("case14.m"));
  const auto results = sppt::run_core_metamorphic_suite(sys);
  REQUIRE(results.size() == 7);
  for (const auto& r : results) {
    INFO(r.id << " " << r.name << ": " << r.detail);
    CHECK(r.passed);
  }
}
