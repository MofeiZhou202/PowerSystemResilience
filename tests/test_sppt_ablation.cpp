/// test_sppt_ablation.cpp
/// ======================
/// Pillar-4 ablation study: each disabled SPPT mechanism produces a measurable
/// failure (docs/latex/sppt_theory.tex).

#include <string>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/model/enums/bus_types.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/sppt/ablation.hpp"

#ifndef HACDCPF_TEST_DATA_DIR
#define HACDCPF_TEST_DATA_DIR "../../data"
#endif

using namespace hacdcpf;

namespace {

std::string data_path(const std::string& name) {
  return std::string(HACDCPF_TEST_DATA_DIR) + "/" + name;
}

HybridPowerSystem make_switch_demo() {
  HybridPowerSystem sys;
  sys.name = "switch_demo";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;
  auto bus = [](int idx, BusType t) {
    ACBus b;
    b.index = idx;
    b.bus_type = t;
    b.vm_pu = 1.0;
    b.in_service = true;
    return b;
  };
  sys.ac.buses = {bus(1, BusType::SLACK), bus(2, BusType::PQ), bus(3, BusType::PQ)};
  ACBranch line;
  line.index = 1;
  line.from_bus = 2;
  line.to_bus = 3;
  line.r_pu = 0.01;
  line.x_pu = 0.05;
  line.tap = 1.0;
  line.in_service = true;
  sys.ac.branches = {line};
  Switch sw;
  sw.index = 1;
  sw.bus_from = 1;
  sw.bus_to = 2;
  sw.closed = true;
  sw.in_service = true;
  sys.ac.switches = {sw};
  Generator g;
  g.index = 1;
  g.bus = 1;
  g.is_slack = true;
  g.in_service = true;
  g.vg_pu = 1.0;
  g.pg_mw = 30.0;
  g.pmax_mw = 200.0;
  g.pmin_mw = 0.0;
  g.qmax_mvar = 100.0;
  g.qmin_mvar = -100.0;
  sys.ac.generators = {g};
  Load ld;
  ld.index = 1;
  ld.bus = 3;
  ld.p_mw = 30.0;
  ld.q_mvar = 10.0;
  ld.in_service = true;
  sys.ac.loads = {ld};
  return sys;
}

}  // namespace

TEST_CASE("Ablation B: role typing catches reference-less edits the raw solver may not",
          "[sppt][ablation][role]") {
  for (const char* c : {"case9.m", "case14.m", "case30.m"}) {
    HybridPowerSystem sys = io::parse_matpower(data_path(c));
    const auto row = sppt::run_ablation_case(sys, c);
    INFO(c << ": guard_rejects=" << row.role_guard_rejects
           << " raw_silent=" << row.role_raw_silent);
    // The well-posedness gate ALWAYS rejects a reference-less system.
    CHECK(row.role_guard_rejects);
  }
}

TEST_CASE("Ablation C: removing the merge guard blows up the Ybus scale",
          "[sppt][ablation][merge]") {
  const auto row = sppt::run_ablation_case(make_switch_demo(), "switch_demo");
  INFO("merged max|Yii|=" << row.merge_max_diag_merged
       << " unmerged=" << row.merge_max_diag_unmerged
       << " blowup=" << row.merge_blowup);
  REQUIRE(row.merge_applicable);
  CHECK(row.merge_max_diag_unmerged > row.merge_max_diag_merged);
  CHECK(row.merge_blowup > 100.0);   // tiny-Z branch injects ~1e6 self-admittance
}

TEST_CASE("Ablation A: provenance is needed exactly where structure is collapsed",
          "[sppt][ablation][provenance]") {
  // A clean MATPOWER case (no switches/merges/dead islands) needs no provenance.
  HybridPowerSystem clean = io::parse_matpower(data_path("case9.m"));
  const auto clean_row = sppt::run_ablation_case(clean, "case9");
  CHECK(clean_row.prov_unattributable_ablated == 0);

  // The switch case collapses a bus and expands a device, so without provenance
  // those canonical entities cannot be attributed back.
  const auto sw_row = sppt::run_ablation_case(make_switch_demo(), "switch_demo");
  INFO("switch_demo unattributable=" << sw_row.prov_unattributable_ablated);
  CHECK(sw_row.prov_unattributable_ablated > 0);
  CHECK(sw_row.prov_unattributable_guarded == 0);
}

TEST_CASE("Ablation study aggregates and renders", "[sppt][ablation][study]") {
  sppt::AblationStudy study;
  study.rows.push_back(sppt::run_ablation_case(
      io::parse_matpower(data_path("case9.m")), "case9"));
  study.rows.push_back(sppt::run_ablation_case(make_switch_demo(), "switch_demo"));

  CHECK(study.total_unattributable_without_provenance() > 0);
  CHECK(study.max_merge_blowup() > 100.0);

  const std::string csv = study.to_csv();
  CHECK(csv.find("case,prov_unattributable_ablated") != std::string::npos);
  const std::string tex = study.to_latex();
  CHECK(tex.find("\\begin{tabular}") != std::string::npos);
  CHECK(tex.find("\\bottomrule") != std::string::npos);
}
