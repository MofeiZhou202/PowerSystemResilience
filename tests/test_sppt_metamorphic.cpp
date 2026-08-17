/// test_sppt_metamorphic.cpp
/// =========================
/// Executable metamorphic relations MR1-MR7 realizing the theorems of
/// docs/latex/sppt_theory.tex (Pillars 1-2 of the verification program).

#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"
#include "hacdcpf/sppt/sppt.hpp"

#ifndef HACDCPF_MATPOWER_DATA_DIR
#define HACDCPF_MATPOWER_DATA_DIR "../../external_data/matpower"
#endif

using namespace hacdcpf;

namespace {

std::string data_path(const std::string& name) {
  return std::string(HACDCPF_MATPOWER_DATA_DIR) + "/" + name;
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

HybridPowerSystem make_merge_strip_case() {
  HybridPowerSystem sys;
  sys.name = "sppt_merge_strip";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ),
                  make_bus(3, BusType::PQ), make_bus(4, BusType::PQ),
                  make_bus(5, BusType::PQ)};
  // Positions 0 and 1 survive. Position 2 belongs to the dead island.
  // Projection appends the switch branch at position 3, then contracts it.
  sys.ac.branches = {make_branch(1, 2, 3, 0.01, 0.05),
                     make_branch(2, 1, 3, 0.02, 0.08),
                     make_branch(3, 4, 5, 0.01, 0.04)};
  Switch sw;
  sw.index = 1;
  sw.bus_from = 1;
  sw.bus_to = 2;
  sw.closed = true;
  sw.in_service = true;
  sys.ac.switches = {sw};
  sys.ac.generators = {make_slack_gen(1, 1)};
  sys.ac.loads = {make_load(1, 3, 20.0, 5.0),
                  make_load(2, 5, 4.0, 1.0)};
  return sys;
}

HybridPowerSystem make_rich_idempotence_case() {
  HybridPowerSystem sys;
  sys.name = "sppt_rich_idempotence";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ),
                  make_bus(3, BusType::PQ)};
  // Match the 20/10 kV transformer nameplate voltages so the canonical model
  // is physically consistent (off-nominal tap and impedance voltage-base
  // scale are both 1). ACBus::base_kv defaults to 110 kV otherwise.
  sys.ac.buses[0].base_kv = 20.0;
  sys.ac.buses[1].base_kv = 10.0;
  sys.ac.buses[2].base_kv = 10.0;
  sys.ac.generators = {make_slack_gen(1, 1)};

  Transformer2W tr;
  tr.index = 1;
  tr.hv_bus = 1;
  tr.lv_bus = 2;
  tr.sn_mva = 40.0;
  tr.vn_hv_kv = 20.0;
  tr.vn_lv_kv = 10.0;
  tr.vk_percent = 6.0;
  tr.vkr_percent = 0.6;
  sys.ac.transformers_2w = {tr};

  Switch sw;
  sw.index = 1;
  sw.bus_from = 2;
  sw.bus_to = 3;
  sw.closed = true;
  sys.ac.switches = {sw};

  FlexibleLoad fl;
  fl.index = 1;
  fl.bus = 3;
  fl.p_mw = 5.0;
  fl.q_mvar = 2.0;
  sys.ac.flexible_loads = {fl};

  EnergyRouter er;
  er.index = 1;
  er.name = "ER1";
  er.num_ports = 2;
  er.p_rated_mw = 2.0;
  EnergyRouterPort p1;
  p1.index = 1;
  p1.bus = 1;
  p1.side = 0;
  p1.port_type = ERPortType::AC;
  p1.p_set_mw = -0.5;
  EnergyRouterPort p2 = p1;
  p2.index = 2;
  p2.bus = 3;
  p2.side = 1;
  p2.p_set_mw = 0.49;
  er.ports = {p1, p2};
  sys.energy_routers = {er};
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

TEST_CASE("merge then dead-island strip composes branch provenance",
          "[sppt][projection][provenance][regression]") {
  const HybridPowerSystem projected =
      project_to_canonical_models(make_merge_strip_case());
  REQUIRE(projected.bus_merge_map.has_value());
  const auto& map = *projected.bus_merge_map;

  REQUIRE(map.n_original_branches == 4);
  REQUIRE(map.branch_orig_to_proj.size() == 4);
  CHECK(map.branch_orig_to_proj.at(0) == 0);
  CHECK(map.branch_orig_to_proj.at(1) == 1);
  CHECK(map.branch_orig_to_proj.at(2) == -1);
  CHECK(map.branch_orig_to_proj.at(3) == -1);
  CHECK(projected.ac.branches.size() == 2);
}

TEST_CASE("rich projection is idempotent for expansion-sensitive components",
          "[sppt][metamorphic][mr1][regression]") {
  const HybridPowerSystem authored = make_rich_idempotence_case();
  const HybridPowerSystem once = project_to_canonical_models(authored);
  const HybridPowerSystem twice = project_to_canonical_models(once);

  REQUIRE(once.projection_certificate.has_value());
  CHECK(once.ac.branches.size() == twice.ac.branches.size());
  CHECK(once.ac.loads.size() == twice.ac.loads.size());
  CHECK(once.vsc_converters.size() == twice.vsc_converters.size());
  CHECK(once.dc.dcdc_converters.size() == twice.dc.dcdc_converters.size());
  CHECK(std::abs(total_load_p_mw(once) - total_load_p_mw(twice)) < 1e-12);
  CHECK(std::abs(total_load_q_mvar(once) - total_load_q_mvar(twice)) < 1e-12);

  const auto mr1 = sppt::mr1_projection_idempotence(authored);
  INFO(mr1.detail);
  CHECK(mr1.passed);
  CHECK(mr1.residual <= mr1.tolerance);
}

TEST_CASE("projection index-space mismatches fail loudly",
          "[sppt][projection][contract][regression]") {
  BusMergeMap map;
  map.n_original = 3;
  map.n_merged = 2;
  map.ext_to_int = {{1, 0}, {2, 0}, {3, 1}};
  map.ext_to_orig_pos = {{1, 0}, {2, 1}, {3, 2}};

  CHECK_THROWS_AS(
      unproject_bus_vector({1.0}, map, BusVectorSemantics::Intensive),
      std::invalid_argument);

  ProjectionOptions exact;
  const HybridPowerSystem canonical =
      project_to_canonical_models(make_rich_idempotence_case(), exact);
  ProjectionOptions incompatible = exact;
  incompatible.impedance_threshold *= 2.0;
  CHECK_THROWS_AS(project_to_canonical_models(canonical, incompatible),
                  std::invalid_argument);
}

TEST_CASE("extensive bus reprojection conserves and splits quantities",
          "[sppt][projection][reprojection][regression]") {
  BusMergeMap map;
  map.n_original = 3;
  map.n_merged = 2;
  map.ext_to_int = {{1, 0}, {2, 0}, {3, 1}};
  map.ext_to_orig_pos = {{1, 0}, {2, 1}, {3, 2}};
  map.extensive_participation = {{1, 0.25}, {2, 0.75}, {3, 1.0}};

  const auto intensive = unproject_bus_vector(
      {8.0, 5.0}, map, BusVectorSemantics::Intensive);
  const auto extensive = unproject_bus_vector(
      {8.0, 5.0}, map, BusVectorSemantics::Extensive);
  CHECK(intensive == std::vector<double>{8.0, 8.0, 5.0});
  CHECK(extensive == std::vector<double>{2.0, 6.0, 5.0});
  CHECK(std::abs(extensive[0] + extensive[1] - 8.0) < 1e-12);
}

TEST_CASE("public linear AC flow returns authored bus and branch spaces",
          "[sppt][projection][reprojection][api][regression]") {
  const HybridPowerSystem authored = make_merge_strip_case();
  const auto result = solve_ac_dc_power_flow(authored);
  REQUIRE(result.success);
  CHECK(result.va.size() == authored.ac.buses.size());
  CHECK(result.pf_mw.size() == authored.ac.branches.size());
  CHECK(result.pf_mw.back() == 0.0);
}

TEST_CASE("public PF hides energy-router internal DC buses",
          "[sppt][projection][reprojection][api][regression]") {
  const HybridPowerSystem authored = make_rich_idempotence_case();
  const auto result = solve_power_flow(authored);
  REQUIRE(result.converged);
  CHECK(result.vdc.size() == authored.dc.buses.size());
  CHECK(result.branch_flows.size() == authored.ac.branches.size());
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
  if (r.observation == sppt::MetamorphicObservation::Observed) {
    CHECK(r.passed);
  } else {
    CHECK_FALSE(r.passed);
    CHECK(r.detail.find("not observed") != std::string::npos);
  }
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

TEST_CASE("MR1-DC: DC dead-island strip is idempotent",
          "[sppt][metamorphic][mr1][dc][regression]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = sys.dc.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK), make_bus(2, BusType::PQ)};
  ACBranch line;
  line.index = 1;
  line.from_bus = 1;
  line.to_bus = 2;
  line.r_pu = 0.01;
  line.x_pu = 0.10;
  sys.ac.branches = {line};
  ExternalGrid grid;
  grid.index = 1;
  grid.bus = 1;
  sys.ac.external_grids = {grid};

  // Live DC island (DC_V source + load via DC branch) plus one dead DC bus.
  DCBus dcv;
  dcv.index = 1;
  dcv.bus_type = DCBusType::DC_V;
  dcv.base_kv = 0.75;
  DCBus dcp;
  dcp.index = 2;
  dcp.bus_type = DCBusType::DC_P;
  dcp.base_kv = 0.75;
  DCBus dead;
  dead.index = 5;
  dead.bus_type = DCBusType::DC_P;
  dead.base_kv = 0.75;
  sys.dc.buses = {dcv, dcp, dead};
  DCBranch dcbr;
  dcbr.index = 1;
  dcbr.from_bus = 1;
  dcbr.to_bus = 2;
  dcbr.r_pu = 0.02;
  sys.dc.branches = {dcbr};

  const HybridPowerSystem once = project_to_canonical_models(sys);
  const HybridPowerSystem twice = project_to_canonical_models(once);

  // First projection strips exactly the one dead DC island.
  REQUIRE(once.dc.buses.size() == 2);
  REQUIRE(once.projection_certificate.has_value());
  CHECK(once.projection_certificate->has_dc_strip());

  // Re-projecting the canonical model removes nothing further (idempotent).
  REQUIRE(twice.dc.buses.size() == once.dc.buses.size());
  for (size_t i = 0; i < once.dc.buses.size(); ++i)
    CHECK(twice.dc.buses[i].index == once.dc.buses[i].index);
}
