#pragma once

// Internal compare/test fixture — NOT a public hacdcpf API header.
// Lives in tools/opendss_compare/ to avoid polluting either the public hacdcpf
// API (include/) or the test-only directory (tests/).
//
// Consumers:
//   tools/opendss_pf_compare.cpp   (compare tool + report generation)
//   tests/test_opendss_pf_compare.cpp (integration tests)
//
// Any builder drift between consumers must be fixed HERE, not in individual
// consumer files.

#include <array>
#include <string>
#include <vector>

#include hacdcpf/model/hybrid_power_system.hpp
#include hacdcpf/model/hybrid_power_system.hpp
#include hacdcpf/model/ac_components.hpp

namespace hacdcpf_compare_fixtures {

using hacdcpf::ACBranch;
using hacdcpf::ACBus;
using hacdcpf::BusType;
using hacdcpf::Generator;
using hacdcpf::HybridPowerSystem;
using hacdcpf::PhaseMask;
using hacdcpf::RegulatorControl;
using hacdcpf::Switch;
using hacdcpf::ThreePhaseACBus;
using hacdcpf::ThreePhaseACLine;
using hacdcpf::ThreePhaseACSystem;
using hacdcpf::ThreePhaseGenerator;
using hacdcpf::ThreePhaseLoad;
using hacdcpf::ThreePhaseRegulatorControl;
using hacdcpf::ThreePhaseTransformer;
using hacdcpf::Transformer2W;
using hacdcpf::Transformer3W;

struct FixedTapOpenDSSCase {
  std::string case_id;
  std::string acceptance_tier;
  std::string relative_master_dss;
  std::string transformer_name;
  std::string line_name;
  double configured_transformer_tap_pu{1.0};
  int transformer_origin_index{0};
  int tap_side{0};
  int tap_pos{0};
  int tap_neutral{0};
  double tap_step_percent{0.0};
};

struct RegulatorOpenDSSCase {
  std::string case_id;
  std::string acceptance_tier;
  std::string relative_master_dss;
  std::string transformer_name;
  std::string line_name;
  std::string regcontrol_name;
  std::string monitored_bus_name;
  int monitored_bus_node{1};
  int winding{0};
  int transformer_origin_index{0};
  int tap_winding{0};
  int tap_side{0};
  int tap_min{0};
  int tap_max{0};
  int tap_neutral{0};
  int expected_final_tap_number{0};
  double expected_final_tap_pu{1.0};
  double tap_step_percent{0.0};
  double forward_vreg_volts{0.0};
  double forward_band_volts{0.0};
  double ptratio{0.0};
  double remote_ptratio{0.0};
  double ct_primary_amps{0.0};
  double forward_r_volts{0.0};
  double forward_x_volts{0.0};
  int max_tap_change{1};
  bool implicit_local_bus_monitoring{false};
};

struct Transformer3WCrossCheckCase {
  std::string case_id;
  std::string acceptance_tier;
  std::string relative_master_dss;
  std::string transformer_name;
  std::vector<std::string> bus_names;
  std::array<int, 3> bus_nodes;
  std::vector<std::string> observed_bus_names;
  std::vector<int> observed_bus_nodes;
  std::string external_line_name;
  int transformer_origin_index{0};
  int tap_side{0};
  int tap_pos{0};
  double tap_step_percent{0.0};
};

struct ThreePhaseNROpenDSSCase {
  std::string case_id;
  std::string acceptance_tier;
  std::string relative_master_dss;
  std::vector<std::string> bus_names;
  std::vector<std::string> line_names;
};

struct ThreePhaseRegulatorPhaseSpec {
  std::string phase_name;
  std::string transformer_name;
  std::string regcontrol_name;
  std::string monitored_bus_name;
  int monitored_bus_node{1};
  int transformer_index{0};
  int winding{0};
  int tap_winding{0};
  int tap_side{0};
  int tap_min{0};
  int tap_max{0};
  int tap_neutral{0};
  int expected_final_tap_number{0};
  double expected_final_tap_pu{1.0};
  double forward_vreg_volts{0.0};
  double forward_band_volts{0.0};
  double ptratio{0.0};
  double remote_ptratio{0.0};
  double ct_primary_amps{0.0};
  double forward_r_volts{0.0};
  double forward_x_volts{0.0};
  int max_tap_change{1};
  bool implicit_local_bus_monitoring{false};
};

struct ThreePhaseRegulatorBankOpenDSSCase {
  std::string case_id;
  std::string acceptance_tier;
  std::string relative_master_dss;
  std::vector<std::string> bus_names;
  std::vector<ThreePhaseRegulatorPhaseSpec> regulators;
};

// ---------------------------------------------------------------------------
// build_3bus_radial
//
// Minimal 3-bus single-phase radial (minimal_radial_3bus_1ph).
//
// Mirror of: tests/data/opendss/minimal_radial_3bus_1ph/Master.dss
//   Slack:  sourcebus @ 7.2 kV LN, V=1.0 pu, angle=0
//   Bus 2:  500 kW + 300 kVar load
//   Bus 3:  800 kW + 400 kVar load
//   L1:  sourcebus→bus2, r1=0.5184 Ω, x1=1.0368 Ω
//        → r_pu=0.10, x_pu=0.20 on 10 MVA base (Z_base=5.184 Ω at 7.2 kV LN)
//   L2:  bus2→bus3, r1=0.41472 Ω, x1=0.7776 Ω
//        → r_pu=0.08, x_pu=0.15 on 10 MVA base
// ---------------------------------------------------------------------------
inline HybridPowerSystem build_3bus_radial() {
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.name = "minimal_radial_3bus_1ph";
  auto& ac = sys.ac;
  ac.base_mva = 10.0;

  ACBus b1, b2, b3;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.vm_pu = 1.0;
  b1.va_deg = 0.0;
  b1.in_service = true;

  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.vm_pu = 1.0;
  b2.va_deg = 0.0;
  b2.pd_mw = 0.5;
  b2.qd_mvar = 0.3;
  b2.in_service = true;

  b3.index = 3;
  b3.bus_type = BusType::PQ;
  b3.vm_pu = 1.0;
  b3.va_deg = 0.0;
  b3.pd_mw = 0.8;
  b3.qd_mvar = 0.4;
  b3.in_service = true;

  ac.buses = {b1, b2, b3};

  ACBranch l1, l2;
  l1.index = 1;
  l1.from_bus = 1;
  l1.to_bus = 2;
  l1.r_pu = 0.10;
  l1.x_pu = 0.20;
  l1.tap = 1.0;
  l1.in_service = true;

  l2.index = 2;
  l2.from_bus = 2;
  l2.to_bus = 3;
  l2.r_pu = 0.08;
  l2.x_pu = 0.15;
  l2.tap = 1.0;
  l2.in_service = true;

  ac.branches = {l1, l2};

  Generator slack;
  slack.index = 1;
  slack.bus = 1;
  slack.in_service = true;
  slack.is_slack = true;
  slack.vg_pu = 1.0;
  slack.pmax_mw = 100.0;
  slack.pmin_mw = 0.0;
  slack.qmax_mvar = 100.0;
  slack.qmin_mvar = -100.0;
  ac.generators = {slack};

  return sys;
}

// ---------------------------------------------------------------------------
// build_3bus_radial_with_shunt
//
// Minimal 3-bus single-phase radial with a 150 kvar constant shunt at bus2.
//
// Mirror of: tests/data/opendss/minimal_radial_3bus_1ph_shunt/Master.dss
//   Same topology as build_3bus_radial().
//   Shunt: ACBus.bs_mvar = 0.15 at bus2 (= 150 kvar / 1000, base_mva=10).
//
// Modeling note: BFS treats bs_mvar as voltage-INDEPENDENT fixed injection.
// OpenDSS mirror uses model=1 kvar=-150 constant-power load (identical
// voltage-independent assumption on both sides → no V^2 bias).
// ---------------------------------------------------------------------------
inline HybridPowerSystem build_3bus_radial_with_shunt() {
  HybridPowerSystem sys = build_3bus_radial();
  sys.name = "minimal_radial_3bus_1ph_shunt";
  for (auto& b : sys.ac.buses) {
    if (b.index == 2) {
      b.bs_mvar = 0.15;  // 150 kvar; BFS uses this as fixed injection
    }
  }
  return sys;
}

// ---------------------------------------------------------------------------
// build_3bus_radial_with_cap_physical
//
// Repo side: same as build_3bus_radial_with_shunt — bs_mvar=0.15 (closest
// BFS can express for a capacitor element).
//
// OpenDSS mirror: tests/data/opendss/minimal_radial_3bus_1ph_cap/Master.dss
//   Uses a physical "Capacitor" element (kvar=150, kv=7.2) which delivers
//   Q = V^2 * B = V^2 * 150 kvar.
//
// Systematic bias: BFS injects fixed 0.15 MVAr; OpenDSS delivers V²*0.15 MVAr.
//   Δ ≈ (1 - V²) * 0.15 MVAr ≈ 0.009 MVAr at V≈0.97 pu (≈6% of cap rating).
//   This case is classified exploratory_physical_component: the bias is
//   expected, quantified, and documented — not suppressed.
// ---------------------------------------------------------------------------
inline HybridPowerSystem build_3bus_radial_with_cap_physical() {
  HybridPowerSystem sys = build_3bus_radial();
  sys.name = "minimal_radial_3bus_1ph_cap";
  for (auto& b : sys.ac.buses) {
    if (b.index == 2) {
      b.bs_mvar = 0.15;  // BFS approximation of a physical capacitor
    }
  }
  return sys;
}

// ---------------------------------------------------------------------------
// build_balanced_3bus_3phase
//
// Minimal balanced 3-bus 3-phase radial (minimal_radial_3bus_balanced_3ph).
//
// Mirror of: tests/data/opendss/minimal_radial_3bus_balanced_3ph/Master.dss
//   Three balanced phases (a=0°, b=-120°, c=+120°).
//   Per-phase loads:
//     Bus 2: 0.5 MW + 0.3 MVAr per phase (total 1.5 MW + 0.9 MVAr)
//     Bus 3: 0.8 MW + 0.4 MVAr per phase (total 2.4 MW + 1.2 MVAr)
// ---------------------------------------------------------------------------
inline ThreePhaseACSystem build_balanced_3bus_3phase() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "minimal_radial_3bus_balanced_3ph";

  ThreePhaseACBus b1, b2, b3;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.vm_a_pu = 1.0;  b1.va_a_deg = 0.0;
  b1.vm_b_pu = 1.0;  b1.va_b_deg = -120.0;
  b1.vm_c_pu = 1.0;  b1.va_c_deg = 120.0;
  b1.in_service = true;

  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.vm_a_pu = 1.0;  b2.va_a_deg = 0.0;
  b2.vm_b_pu = 1.0;  b2.va_b_deg = -120.0;
  b2.vm_c_pu = 1.0;  b2.va_c_deg = 120.0;
  b2.pd_a_mw = 0.5;  b2.qd_a_mvar = 0.3;
  b2.pd_b_mw = 0.5;  b2.qd_b_mvar = 0.3;
  b2.pd_c_mw = 0.5;  b2.qd_c_mvar = 0.3;
  b2.in_service = true;

  b3.index = 3;
  b3.bus_type = BusType::PQ;
  b3.vm_a_pu = 1.0;  b3.va_a_deg = 0.0;
  b3.vm_b_pu = 1.0;  b3.va_b_deg = -120.0;
  b3.vm_c_pu = 1.0;  b3.va_c_deg = 120.0;
  b3.pd_a_mw = 0.8;  b3.qd_a_mvar = 0.4;
  b3.pd_b_mw = 0.8;  b3.qd_b_mvar = 0.4;
  b3.pd_c_mw = 0.8;  b3.qd_c_mvar = 0.4;
  b3.in_service = true;

  sys.buses = {b1, b2, b3};

  ThreePhaseACLine l1, l2;
  l1.index = 1;  l1.from_bus = 1;  l1.to_bus = 2;
  l1.r1_pu = 0.10;  l1.x1_pu = 0.20;
  l1.r0_pu = 0.10;  l1.x0_pu = 0.20;
  l1.in_service = true;

  l2.index = 2;  l2.from_bus = 2;  l2.to_bus = 3;
  l2.r1_pu = 0.08;  l2.x1_pu = 0.15;
  l2.r0_pu = 0.08;  l2.x0_pu = 0.15;
  l2.in_service = true;

  sys.lines = {l1, l2};

  ThreePhaseGenerator g;
  g.index = 1;
  g.bus = 1;
  g.in_service = true;
  g.is_slack = true;
  g.vm_pu = 1.0;
  g.pmax_mw = 100.0;
  g.pmin_mw = 0.0;
  g.qmax_mvar = 100.0;
  g.qmin_mvar = -100.0;
  sys.generators = {g};

  return sys;
}

// ---------------------------------------------------------------------------
// build_case33bw_radial
//
// Baran & Wu (1989) 33-bus radial distribution system — single-phase
// equivalent on a 10 MVA, 12.66 kV (L-L) base.
//
// Source: W. H. Kersting, "A comprehensive distribution test feeder,"
//   plus W. Baran & F. Wu, "Network reconfiguration in distribution systems,"
//   IEEE Trans. Power Delivery, 4(3), 1989.
//   Parameters verified against the publicly available MATPOWER case33bw.m.
//
// Mirror of: tests/data/opendss/case33bw_radial_3ph/Master.dss
//   OpenDSS model is 3-phase balanced (phases=3, basekv=12.66).
//   Each 3-phase balanced load matches the 3-phase total here.
//   Phase A per-unit voltage equals the BFS single-phase-equivalent voltage.
//   3-phase total terminal power = 3 × per-phase = BFS p_branch_mw.
//
// Topology (32 radial branches):
//   Main feeder: b1→b2→b3→...→b18
//   Lateral-1:   b2→b19→b20→b21→b22
//   Lateral-2:   b3→b23→b24→b25
//   Lateral-3:   b6→b26→b27→b28→b29→b30→b31→b32→b33
//
// Z_base = 12.66^2 / 10 = 16.028 Ω   (branch impedances: R_pu = R_Ω / Z_base)
// Total load: 3715 kW + 2300 kVAr (matches Baran & Wu Table I)
// ---------------------------------------------------------------------------
inline HybridPowerSystem build_case33bw_radial() {
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.name = "case33bw_radial_3ph";
  auto& ac = sys.ac;
  ac.base_mva = 10.0;

  // Z_base at 10 MVA, 12.66 kV L-L
  const double z_base = 12.66 * 12.66 / 10.0;  // 16.028 Ω

  // Bus 1: slack
  {
    ACBus b;
    b.index = 1;
    b.bus_type = BusType::SLACK;
    b.vm_pu = 1.0;
    b.va_deg = 0.0;
    b.in_service = true;
    ac.buses.push_back(b);
  }

  // Buses 2-33: PQ loads.
  // Load data {pd_kW, qd_kVAr} from Baran & Wu (1989) Table I, indexed bus-2..bus-33.
  static const double kLoadsKW[32] = {
    100, 90, 120, 60, 60, 200, 200, 60, 60, 45, 60, 60,
    120, 60, 60, 60, 90, 90, 90, 90, 90, 90, 420, 420,
    60, 60, 60, 120, 200, 150, 210, 60
  };
  static const double kLoadsKVAr[32] = {
    60, 40, 80, 30, 20, 100, 100, 20, 20, 30, 35, 35,
    80, 10, 20, 20, 40, 40, 40, 40, 40, 50, 200, 200,
    25, 25, 20, 70, 600, 70, 100, 40
  };

  for (int i = 0; i < 32; ++i) {
    ACBus b;
    b.index = i + 2;
    b.bus_type = BusType::PQ;
    b.vm_pu = 1.0;
    b.va_deg = 0.0;
    b.pd_mw   = kLoadsKW[i]   / 1000.0;
    b.qd_mvar = kLoadsKVAr[i] / 1000.0;
    b.in_service = true;
    ac.buses.push_back(b);
  }

  // Branch data {from_bus, to_bus, R_Ω, X_Ω} from Baran & Wu Table I.
  struct BranchData { int from; int to; double r_ohm; double x_ohm; };
  static const BranchData kBr[32] = {
    {1,  2,  0.0922, 0.0477}, {2,  3,  0.4930, 0.2511}, {3,  4,  0.3660, 0.1864},
    {4,  5,  0.3811, 0.1941}, {5,  6,  0.8190, 0.7070}, {6,  7,  0.1872, 0.6188},
    {7,  8,  0.7114, 0.2351}, {8,  9,  1.0300, 0.7400}, {9,  10, 1.0440, 0.7400},
    {10, 11, 0.1966, 0.0650}, {11, 12, 0.3744, 0.1238}, {12, 13, 1.4680, 1.1550},
    {13, 14, 0.5416, 0.7129}, {14, 15, 0.5910, 0.5260}, {15, 16, 0.7463, 0.5450},
    {16, 17, 1.2890, 1.7210}, {17, 18, 0.7320, 0.5740},
    {2,  19, 0.1640, 0.1565}, {19, 20, 1.5042, 1.3554}, {20, 21, 0.4095, 0.4784},
    {21, 22, 0.7089, 0.9373},
    {3,  23, 0.4512, 0.3083}, {23, 24, 0.8980, 0.7091}, {24, 25, 0.8960, 0.7011},
    {6,  26, 0.2030, 0.1034}, {26, 27, 0.2842, 0.1447}, {27, 28, 1.0590, 0.9337},
    {28, 29, 0.8042, 0.7006}, {29, 30, 0.5075, 0.2585}, {30, 31, 0.9744, 0.9630},
    {31, 32, 0.3105, 0.3619}, {32, 33, 0.3410, 0.5302},
  };

  for (int i = 0; i < 32; ++i) {
    ACBranch br;
    br.index = i + 1;
    br.from_bus = kBr[i].from;
    br.to_bus   = kBr[i].to;
    br.r_pu = kBr[i].r_ohm / z_base;
    br.x_pu = kBr[i].x_ohm / z_base;
    br.tap = 1.0;
    br.in_service = true;
    ac.branches.push_back(br);
  }

  Generator slack;
  slack.index = 1;
  slack.bus = 1;
  slack.in_service = true;
  slack.is_slack = true;
  slack.vg_pu = 1.0;
  slack.pmax_mw = 100.0;
  slack.pmin_mw = 0.0;
  slack.qmax_mvar = 100.0;
  slack.qmin_mvar = -100.0;
  ac.generators = {slack};

  return sys;
}

// ---------------------------------------------------------------------------
// build_unbalanced_3bus_3phase
//
// Minimal 3-bus 3-phase unbalanced radial (minimal_unbalanced_3bus_3ph).
//
// Mirror of: tests/data/opendss/minimal_unbalanced_3bus_3ph/Master.dss
//   Same topology as build_balanced_3bus_3phase() but with unequal per-phase
//   loads AND different zero-sequence impedances (r0 ≠ r1).
//
//   Load unbalance (per phase, MW + MVAr):
//     Bus 2: A=0.3/0.15, B=0.2/0.10, C=0.1/0.05
//     Bus 3: A=0.4/0.20, B=0.3/0.15, C=0.2/0.10
//
//   Line impedances (per-unit on 10 MVA, Z_base=5.184 Ω = 7.2²/10):
//     L1: r1=0.10, x1=0.20, r0=0.30, x0=0.50  (→ Ohm: 0.5184,1.0368,1.5552,2.5920)
//     L2: r1=0.08, x1=0.15, r0=0.25, x0=0.40  (→ Ohm: 0.41472,0.77760,1.29600,2.07360)
//
// This case is the accepted unbalanced 3-phase baseline. It verifies:
//   - Per-phase Vm and Va for all 3 buses
//   - Per-phase terminal-1 P/Q for both lines
//   - Total P/Q losses
// ---------------------------------------------------------------------------
inline ThreePhaseACSystem build_unbalanced_3bus_3phase() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.name = "minimal_unbalanced_3bus_3ph";

  ThreePhaseACBus b1, b2, b3;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.vm_a_pu = 1.0;  b1.va_a_deg = 0.0;
  b1.vm_b_pu = 1.0;  b1.va_b_deg = -120.0;
  b1.vm_c_pu = 1.0;  b1.va_c_deg = 120.0;
  b1.in_service = true;

  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.vm_a_pu = 1.0;  b2.va_a_deg = 0.0;
  b2.vm_b_pu = 1.0;  b2.va_b_deg = -120.0;
  b2.vm_c_pu = 1.0;  b2.va_c_deg = 120.0;
  b2.pd_a_mw = 0.3;  b2.qd_a_mvar = 0.15;
  b2.pd_b_mw = 0.2;  b2.qd_b_mvar = 0.10;
  b2.pd_c_mw = 0.1;  b2.qd_c_mvar = 0.05;
  b2.in_service = true;

  b3.index = 3;
  b3.bus_type = BusType::PQ;
  b3.vm_a_pu = 1.0;  b3.va_a_deg = 0.0;
  b3.vm_b_pu = 1.0;  b3.va_b_deg = -120.0;
  b3.vm_c_pu = 1.0;  b3.va_c_deg = 120.0;
  b3.pd_a_mw = 0.4;  b3.qd_a_mvar = 0.20;
  b3.pd_b_mw = 0.3;  b3.qd_b_mvar = 0.15;
  b3.pd_c_mw = 0.2;  b3.qd_c_mvar = 0.10;
  b3.in_service = true;

  sys.buses = {b1, b2, b3};

  ThreePhaseACLine l1, l2;
  l1.index = 1;  l1.from_bus = 1;  l1.to_bus = 2;
  l1.r1_pu = 0.10;  l1.x1_pu = 0.20;
  l1.r0_pu = 0.30;  l1.x0_pu = 0.50;
  l1.in_service = true;

  l2.index = 2;  l2.from_bus = 2;  l2.to_bus = 3;
  l2.r1_pu = 0.08;  l2.x1_pu = 0.15;
  l2.r0_pu = 0.25;  l2.x0_pu = 0.40;
  l2.in_service = true;

  sys.lines = {l1, l2};

  ThreePhaseGenerator g;
  g.index = 1;
  g.bus = 1;
  g.in_service = true;
  g.is_slack = true;
  g.vm_pu = 1.0;
  g.pmax_mw = 100.0;
  g.pmin_mw = 0.0;
  g.qmax_mvar = 100.0;
  g.qmin_mvar = -100.0;
  sys.generators = {g};

  return sys;
}

// ---------------------------------------------------------------------------
// build_3bus_passthr_transformer
//
// Minimal 3-bus single-phase case with a pass-through transformer (tap=1.0).
//
// Mirror of: tests/data/opendss/minimal_passthr_transformer_1ph/Master.dss
//   Topology: sourcebus -[Transformer t12]- bus2 -[Line l1]- bus3
//   Transformer: r_pu=0.02 (2% copper loss, %loadloss=2.0), x_pu=0.06 (xhl=6.0%), tap=1.0.
//   Line l1: r_pu=0.08, x_pu=0.15 (same as base 1ph case line 2).
//   Loads: bus2=500 kW/300 kVAr, bus3=800 kW/400 kVAr.
//
// With tap=1.0, the ACBranch tap-aware solver reduces to the same pure series
// impedance (r_pu=0.02, x_pu=0.06). The OpenDSS Transformer with
// %noloadloss=0 and %imag=0 is also a pure series impedance.
// This case verifies that the two are equivalent for tap=1.0.
//
// The generic PD-element bridge now captures Transformer terminal powers
// directly. line_results remains only as a compatibility projection.
// ---------------------------------------------------------------------------
inline HybridPowerSystem build_3bus_passthr_transformer() {
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.name = "minimal_passthr_transformer_1ph";
  auto& ac = sys.ac;
  ac.base_mva = 10.0;

  ACBus b1, b2, b3;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.vm_pu = 1.0;
  b1.va_deg = 0.0;
  b1.in_service = true;

  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.vm_pu = 1.0;
  b2.va_deg = 0.0;
  b2.pd_mw = 0.5;
  b2.qd_mvar = 0.3;
  b2.in_service = true;

  b3.index = 3;
  b3.bus_type = BusType::PQ;
  b3.vm_pu = 1.0;
  b3.va_deg = 0.0;
  b3.pd_mw = 0.8;
  b3.qd_mvar = 0.4;
  b3.in_service = true;

  ac.buses = {b1, b2, b3};

  // Branch 0 (index 1): represents the transformer as a series impedance.
  // r_pu=0.02 (2% resistance), x_pu=0.06 (6% leakage), tap=1.0 (BFS ignores tap).
  ACBranch t12, l1;
  t12.index = 1;
  t12.from_bus = 1;
  t12.to_bus = 2;
  t12.r_pu = 0.02;
  t12.x_pu = 0.06;
  t12.tap = 1.0;
  t12.in_service = true;

  // Branch 1 (index 2): line bus2→bus3.
  // Same as build_3bus_radial() line 2: r_pu=0.08, x_pu=0.15.
  l1.index = 2;
  l1.from_bus = 2;
  l1.to_bus = 3;
  l1.r_pu = 0.08;
  l1.x_pu = 0.15;
  l1.tap = 1.0;
  l1.in_service = true;

  ac.branches = {t12, l1};

  Generator slack;
  slack.index = 1;
  slack.bus = 1;
  slack.in_service = true;
  slack.is_slack = true;
  slack.vg_pu = 1.0;
  slack.pmax_mw = 100.0;
  slack.pmin_mw = 0.0;
  slack.qmax_mvar = 100.0;
  slack.qmin_mvar = -100.0;
  ac.generators = {slack};

  return sys;
}

// ---------------------------------------------------------------------------
// build_3bus_fixed_tap_transformer
//
// Minimal 3-bus single-phase case with an off-nominal fixed transformer tap.
//
// Mirror of: tests/data/opendss/minimal_tap_transformer_1ph/Master.dss
//
// Repository-side representation:
//   - Transformer2W stores the original tap-side semantics.
//   - project_to_canonical_models() normalizes the tap onto ACBranch.from_bus.
//   - solve_distribution_pf(const HybridPowerSystem&) then solves the projected
//     ACBranch with tap-aware BFS equations.
//
// Parameter note:
//   Transformer2W uses vk_percent / vkr_percent, where vk is the impedance
//   magnitude.  To match the OpenDSS xhl=6.0%, %loadloss=2.0% series branch
//   used in the validation case, choose:
//     r_pu = 0.02
//     x_pu = 0.06
//     |z_pu| = sqrt(r^2 + x^2) = 0.0632455532
//   so vk_percent = 6.32455532 and vkr_percent = 2.0.
//
// Tap semantics:
//   tap_side = 0 (HV / from-side)
//   tap_pos = 5, tap_neutral = 0, tap_step_percent = 1.0
//   → physical winding tap = 1.05 pu
//
// ---------------------------------------------------------------------------
inline HybridPowerSystem build_3bus_fixed_tap_transformer() {
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.name = "minimal_tap_transformer_1ph";
  auto& ac = sys.ac;
  ac.base_mva = 10.0;

  ACBus b1, b2, b3;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.vm_pu = 1.0;
  b1.va_deg = 0.0;
  b1.in_service = true;

  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.vm_pu = 1.0;
  b2.va_deg = 0.0;
  b2.pd_mw = 0.5;
  b2.qd_mvar = 0.3;
  b2.in_service = true;

  b3.index = 3;
  b3.bus_type = BusType::PQ;
  b3.vm_pu = 1.0;
  b3.va_deg = 0.0;
  b3.pd_mw = 0.8;
  b3.qd_mvar = 0.4;
  b3.in_service = true;

  ac.buses = {b1, b2, b3};

  ACBranch l1;
  l1.index = 1;
  l1.from_bus = 2;
  l1.to_bus = 3;
  l1.r_pu = 0.08;
  l1.x_pu = 0.15;
  l1.tap = 1.0;
  l1.in_service = true;
  l1.name = "l1";
  ac.branches = {l1};

  Transformer2W t12;
  t12.index = 1;
  t12.name = "t12";
  t12.hv_bus = 1;
  t12.lv_bus = 2;
  t12.in_service = true;
  t12.sn_mva = 10.0;
  t12.vn_hv_kv = 7.2;
  t12.vn_lv_kv = 7.2;
  t12.vk_percent = 6.324555320336759;
  t12.vkr_percent = 2.0;
  t12.tap_side = 0;
  t12.tap_pos = 5;
  t12.tap_min = -16;
  t12.tap_max = 16;
  t12.tap_neutral = 0;
  t12.tap_step_percent = 1.0;
  t12.shift_deg = 0.0;
  ac.transformers_2w = {t12};

  Generator slack;
  slack.index = 1;
  slack.bus = 1;
  slack.in_service = true;
  slack.is_slack = true;
  slack.vg_pu = 1.0;
  slack.pmax_mw = 100.0;
  slack.pmin_mw = 0.0;
  slack.qmax_mvar = 100.0;
  slack.qmin_mvar = -100.0;
  ac.generators = {slack};

  return sys;
}

inline HybridPowerSystem build_3bus_fixed_tap_transformer_lv_side() {
  HybridPowerSystem sys = build_3bus_fixed_tap_transformer();
  sys.name = "minimal_tap_transformer_1ph_lv_side";
  auto& tr = sys.ac.transformers_2w.front();
  tr.tap_side = 1;
  return sys;
}

inline FixedTapOpenDSSCase fixed_tap_transformer_case() {
  return {
      .case_id = "minimal_tap_transformer_1ph",
      .acceptance_tier = "accepted_fixed_tap_xfmr",
      .relative_master_dss =
          "tests/data/opendss/minimal_tap_transformer_1ph/Master.dss",
      .transformer_name = "t12",
      .line_name = "l1",
      .configured_transformer_tap_pu = 1.05,
      .transformer_origin_index = 1,
      .tap_side = 0,
      .tap_pos = 5,
      .tap_neutral = 0,
      .tap_step_percent = 1.0,
  };
}

// ---------------------------------------------------------------------------
// build_minimal_regulator_1ph
//
// Minimal 3-bus single-phase closed-loop regulator case.
//
// Mirror of:
//   tests/data/opendss/minimal_regulator_1ph/Master.dss
//
// Topology:
//   sourcebus -[Transformer t12 + RegControl reg1]- bus2 -[Line l1]- bus3
//
// Control semantics:
//   - tapwinding = 2, so the physical tap lives on the LV/to-side winding.
//   - tap_step_percent = 0.625 => 32 taps from 0.90 to 1.10, matching OpenDSS.
//   - monitored bus is remote bus3.1 (remote PT path, no LDC in the accepted case).
// ---------------------------------------------------------------------------
inline HybridPowerSystem build_minimal_regulator_1ph() {
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.name = "minimal_regulator_1ph";
  auto& ac = sys.ac;
  ac.base_mva = 10.0;

  ACBus b1, b2, b3;
  b1.index = 1;
  b1.name = "sourcebus";
  b1.base_kv = 7.2;
  b1.bus_type = BusType::SLACK;
  b1.vm_pu = 1.0;
  b1.va_deg = 0.0;
  b1.in_service = true;

  b2.index = 2;
  b2.name = "bus2";
  b2.base_kv = 7.2;
  b2.bus_type = BusType::PQ;
  b2.vm_pu = 1.0;
  b2.va_deg = 0.0;
  b2.in_service = true;

  b3.index = 3;
  b3.name = "bus3";
  b3.base_kv = 7.2;
  b3.bus_type = BusType::PQ;
  b3.vm_pu = 1.0;
  b3.va_deg = 0.0;
  b3.pd_mw = 1.6;
  b3.qd_mvar = 0.9;
  b3.in_service = true;

  ac.buses = {b1, b2, b3};

  ACBranch l1;
  l1.index = 1;
  l1.from_bus = 2;
  l1.to_bus = 3;
  l1.r_pu = 0.08;
  l1.x_pu = 0.15;
  l1.tap = 1.0;
  l1.in_service = true;
  l1.name = "l1";
  ac.branches = {l1};

  Transformer2W t12;
  t12.index = 1;
  t12.name = "t12";
  t12.hv_bus = 1;
  t12.lv_bus = 2;
  t12.in_service = true;
  t12.sn_mva = 10.0;
  t12.vn_hv_kv = 7.2;
  t12.vn_lv_kv = 7.2;
  t12.vk_percent = 6.324555320336759;
  t12.vkr_percent = 2.0;
  t12.tap_side = 1;
  t12.tap_pos = 0;
  t12.tap_min = -16;
  t12.tap_max = 16;
  t12.tap_neutral = 0;
  t12.tap_step_percent = 0.625;
  t12.shift_deg = 0.0;
  ac.transformers_2w = {t12};

  RegulatorControl reg1;
  reg1.index = 1;
  reg1.name = "reg1";
  reg1.transformer_index = 1;
  reg1.transformer_name = "t12";
  reg1.winding = 2;
  reg1.tap_winding = 2;
  reg1.monitored_bus = 3;
  reg1.monitored_node = 1;
  reg1.vreg_volts = 122.0;
  reg1.band_volts = 2.0;
  reg1.ptratio = 60.0;
  reg1.remote_ptratio = 60.0;
  reg1.ct_primary_amps = 300.0;
  reg1.r_volts = 0.0;
  reg1.x_volts = 0.0;
  reg1.max_tap_change = 1;
  reg1.reversible = false;
  reg1.enabled = true;
  ac.regulator_controls = {reg1};

  Generator slack;
  slack.index = 1;
  slack.bus = 1;
  slack.in_service = true;
  slack.is_slack = true;
  slack.vg_pu = 1.0;
  slack.pmax_mw = 100.0;
  slack.pmin_mw = 0.0;
  slack.qmax_mvar = 100.0;
  slack.qmin_mvar = -100.0;
  ac.generators = {slack};

  return sys;
}

inline HybridPowerSystem build_minimal_regulator_1ph_local_pt_with_ldc() {
  HybridPowerSystem sys = build_minimal_regulator_1ph();
  sys.name = "minimal_regulator_local_pt_ldc_1ph";
  auto& reg = sys.ac.regulator_controls.front();
  reg.monitored_bus = 0;
  reg.vreg_volts = 122.0;
  reg.band_volts = 2.0;
  reg.ptratio = 60.0;
  reg.remote_ptratio = 60.0;
  reg.ct_primary_amps = 300.0;
  reg.r_volts = 0.1;
  reg.x_volts = 0.2;
  reg.max_tap_change = 1;
  return sys;
}

inline HybridPowerSystem build_minimal_regulator_1ph_remote_ptratio() {
  HybridPowerSystem sys = build_minimal_regulator_1ph();
  sys.name = "minimal_regulator_remote_ptratio_1ph";
  auto& reg = sys.ac.regulator_controls.front();
  reg.remote_ptratio = 55.0;
  return sys;
}

inline HybridPowerSystem build_minimal_regulator_1ph_with_merged_monitor_bus() {
  HybridPowerSystem sys = build_minimal_regulator_1ph();

  ACBus b4;
  b4.index = 4;
  b4.name = "bus4";
  b4.base_kv = 7.2;
  b4.bus_type = BusType::PQ;
  b4.vm_pu = 1.0;
  b4.va_deg = 0.0;
  b4.in_service = true;
  sys.ac.buses.push_back(b4);

  hacdcpf::Switch sw34;
  sw34.index = 1;
  sw34.name = "sw34";
  sw34.bus_from = 3;
  sw34.bus_to = 4;
  sw34.in_service = true;
  sw34.closed = true;
  sw34.r_contact_ohm = 0.0;
  sw34.z_ohm = 0.0;
  sys.ac.switches = {sw34};

  sys.ac.regulator_controls.front().monitored_bus = 4;
  return sys;
}

inline ThreePhaseACSystem build_minimal_regulator_bank_3ph_nr_case() {
  auto make_bus = [](
                      int index,
                      const std::string& name,
                      BusType type,
                      PhaseMask mask) {
    ThreePhaseACBus bus;
    bus.index = index;
    bus.name = name;
    bus.bus_type = type;
    bus.phase_mask = mask;
    bus.base_kv = 7.2;
    bus.in_service = true;
    bus.vm_a_pu = 1.0;
    bus.va_a_deg = 0.0;
    bus.vm_b_pu = 1.0;
    bus.va_b_deg = -120.0;
    bus.vm_c_pu = 1.0;
    bus.va_c_deg = 120.0;
    return bus;
  };

  auto make_transformer = [](
                              int index,
                              const std::string& name,
                              PhaseMask mask) {
    ThreePhaseTransformer transformer;
    transformer.index = index;
    transformer.name = name;
    transformer.hv_bus = 1;
    transformer.lv_bus = 2;
    transformer.in_service = true;
    transformer.hv_phase_mask = mask;
    transformer.lv_phase_mask = mask;
    transformer.sn_mva = 5.0;
    transformer.vn_hv_kv = 7.2;
    transformer.vn_lv_kv = 7.2;
    transformer.vk_percent = 1.0;
    transformer.vkr_percent = 0.1;
    transformer.vector_group = "YNyn0";
    transformer.tap_side = 1;
    transformer.tap_min = -16;
    transformer.tap_max = 16;
    transformer.tap_neutral = 0;
    transformer.tap_step_percent = 0.625;
    return transformer;
  };

  auto make_control = [](
                           int index,
                           const std::string& name,
                           const std::string& transformer_name,
                           int transformer_index,
                           int monitored_node,
                           double vreg_volts,
                           double band_volts) {
    ThreePhaseRegulatorControl control;
    control.index = index;
    control.name = name;
    control.transformer_index = transformer_index;
    control.transformer_name = transformer_name;
    control.winding = 2;
    control.tap_winding = 2;
    control.monitored_bus = 0;
    control.monitored_node = monitored_node;
    control.vreg_volts = vreg_volts;
    control.band_volts = band_volts;
    control.ptratio = 60.0;
    control.remote_ptratio = 60.0;
    control.ct_primary_amps = 300.0;
    control.max_tap_change = 1;
    control.enabled = true;
    return control;
  };

  ThreePhaseACSystem sys;
  sys.base_mva = 5.0;
  sys.name = "minimal_regulator_bank_3ph_nr";
  sys.buses = {
      make_bus(1, "sourcebus", BusType::SLACK, PhaseMask::abc()),
      make_bus(2, "bus2", BusType::PQ, PhaseMask::abc()),
  };
  sys.transformers = {
      make_transformer(1, "rega", PhaseMask::a()),
      make_transformer(2, "regb", PhaseMask::b()),
      make_transformer(3, "regc", PhaseMask::c()),
  };
  sys.regulator_controls = {
      make_control(1, "rega_ctl", "rega", 1, 1, 121.5, 1.0),
      make_control(2, "regb_ctl", "regb", 2, 2, 120.75, 0.5),
      make_control(3, "regc_ctl", "regc", 3, 3, 119.25, 0.5),
  };
  return sys;
}

inline ThreePhaseRegulatorBankOpenDSSCase accepted_three_phase_regulator_bank_case() {
  return {
      .case_id = "minimal_regulator_bank_3ph_nr",
      .acceptance_tier = "accepted_regulator_bank_3ph",
      .relative_master_dss =
          "tests/data/opendss/minimal_regulator_bank_3ph_nr/Master.dss",
      .bus_names = {"sourcebus", "bus2"},
      .regulators = {
          {
              .phase_name = "a",
              .transformer_name = "rega",
              .regcontrol_name = "rega_ctl",
              .monitored_bus_name = "bus2",
              .monitored_bus_node = 1,
              .transformer_index = 1,
              .winding = 2,
              .tap_winding = 2,
              .tap_side = 1,
              .tap_min = -16,
              .tap_max = 16,
              .tap_neutral = 0,
              .expected_final_tap_number = 2,
              .expected_final_tap_pu = 1.0125,
              .forward_vreg_volts = 121.5,
              .forward_band_volts = 1.0,
              .ptratio = 60.0,
              .remote_ptratio = 60.0,
              .ct_primary_amps = 300.0,
              .forward_r_volts = 0.0,
              .forward_x_volts = 0.0,
              .max_tap_change = 1,
              .implicit_local_bus_monitoring = true,
          },
          {
              .phase_name = "b",
              .transformer_name = "regb",
              .regcontrol_name = "regb_ctl",
              .monitored_bus_name = "bus2",
              .monitored_bus_node = 2,
              .transformer_index = 2,
              .winding = 2,
              .tap_winding = 2,
              .tap_side = 1,
              .tap_min = -16,
              .tap_max = 16,
              .tap_neutral = 0,
              .expected_final_tap_number = 1,
              .expected_final_tap_pu = 1.00625,
              .forward_vreg_volts = 120.75,
              .forward_band_volts = 0.5,
              .ptratio = 60.0,
              .remote_ptratio = 60.0,
              .ct_primary_amps = 300.0,
              .forward_r_volts = 0.0,
              .forward_x_volts = 0.0,
              .max_tap_change = 1,
              .implicit_local_bus_monitoring = true,
          },
          {
              .phase_name = "c",
              .transformer_name = "regc",
              .regcontrol_name = "regc_ctl",
              .monitored_bus_name = "bus2",
              .monitored_bus_node = 3,
              .transformer_index = 3,
              .winding = 2,
              .tap_winding = 2,
              .tap_side = 1,
              .tap_min = -16,
              .tap_max = 16,
              .tap_neutral = 0,
              .expected_final_tap_number = -1,
              .expected_final_tap_pu = 0.99375,
              .forward_vreg_volts = 119.25,
              .forward_band_volts = 0.5,
              .ptratio = 60.0,
              .remote_ptratio = 60.0,
              .ct_primary_amps = 300.0,
              .forward_r_volts = 0.0,
              .forward_x_volts = 0.0,
              .max_tap_change = 1,
              .implicit_local_bus_monitoring = true,
          },
      },
  };
}

inline HybridPowerSystem build_minimal_regulator_cycle_1ph_with_merged_monitor_bus() {
  HybridPowerSystem sys = build_minimal_regulator_1ph_with_merged_monitor_bus();
  auto& reg = sys.ac.regulator_controls.front();
  // Force a discrete tap ping-pong between tap 0 and tap 1:
  // tap 0 ≈ 115.61 V, tap 1 ≈ 116.39 V on the PT secondary.
  reg.vreg_volts = 116.0;
  reg.band_volts = 0.1;
  reg.max_tap_change = 1;
  return sys;
}

inline HybridPowerSystem build_transformer3w_tap_projection_case(int tap_side) {
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.name = "transformer3w_tap_projection_case";
  auto& ac = sys.ac;
  ac.base_mva = 10.0;

  ACBus b1, b2, b3;
  b1.index = 1;
  b1.name = "hv";
  b1.base_kv = 33.0;
  b1.bus_type = BusType::SLACK;
  b1.vm_pu = 1.0;
  b1.in_service = true;

  b2.index = 2;
  b2.name = "mv";
  b2.base_kv = 11.0;
  b2.bus_type = BusType::PQ;
  b2.vm_pu = 1.0;
  b2.in_service = true;

  b3.index = 3;
  b3.name = "lv";
  b3.base_kv = 6.6;
  b3.bus_type = BusType::PQ;
  b3.vm_pu = 1.0;
  b3.in_service = true;

  ac.buses = {b1, b2, b3};

  Transformer3W tr;
  tr.index = 1;
  tr.name = "t135";
  tr.hv_bus = 1;
  tr.mv_bus = 2;
  tr.lv_bus = 3;
  tr.in_service = true;
  tr.sn_hv_mva = 10.0;
  tr.sn_mv_mva = 6.0;
  tr.sn_lv_mva = 4.0;
  tr.vn_hv_kv = 33.0;
  tr.vn_mv_kv = 11.0;
  tr.vn_lv_kv = 6.6;
  tr.vk_hv_mv_percent = 10.0;
  tr.vk_hv_lv_percent = 11.0;
  tr.vk_mv_lv_percent = 12.0;
  tr.vkr_hv_mv_percent = 1.0;
  tr.vkr_hv_lv_percent = 1.1;
  tr.vkr_mv_lv_percent = 1.2;
  tr.tap_side = tap_side;
  tr.tap_pos = 4;
  tr.tap_step_percent = 1.25;
  ac.transformers_3w = {tr};

  Generator slack;
  slack.index = 1;
  slack.bus = 1;
  slack.in_service = true;
  slack.is_slack = true;
  slack.vg_pu = 1.0;
  slack.pmax_mw = 100.0;
  slack.pmin_mw = 0.0;
  slack.qmax_mvar = 100.0;
  slack.qmin_mvar = -100.0;
  ac.generators = {slack};

  return sys;
}

inline HybridPowerSystem build_transformer3w_tap_smoke_case() {
  HybridPowerSystem sys = build_transformer3w_tap_projection_case(2);
  sys.name = "transformer3w_tap_smoke_case";
  auto& tr = sys.ac.transformers_3w.front();
  tr.vk_mv_lv_percent = 0.0;
  tr.vkr_mv_lv_percent = 0.0;

  sys.ac.buses[1].pd_mw = 0.9;
  sys.ac.buses[1].qd_mvar = 0.3;
  sys.ac.buses[2].pd_mw = 0.6;
  sys.ac.buses[2].qd_mvar = 0.2;
  return sys;
}

inline HybridPowerSystem build_minimal_transformer3w_cross_check_case() {
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.name = "minimal_transformer3w_tap_1ph";
  auto& ac = sys.ac;
  ac.base_mva = 10.0;

  ACBus hv, mv, lv;
  hv.index = 1;
  hv.name = "hv";
  hv.base_kv = 33.0;
  hv.bus_type = BusType::SLACK;
  hv.vm_pu = 1.0;
  hv.va_deg = 0.0;
  hv.in_service = true;

  mv.index = 2;
  mv.name = "mv";
  mv.base_kv = 11.0;
  mv.bus_type = BusType::PQ;
  mv.vm_pu = 1.0;
  mv.va_deg = 0.0;
  mv.pd_mw = 0.8;
  mv.qd_mvar = 0.25;
  mv.in_service = true;

  lv.index = 3;
  lv.name = "lv";
  lv.base_kv = 6.6;
  lv.bus_type = BusType::PQ;
  lv.vm_pu = 1.0;
  lv.va_deg = 0.0;
  lv.pd_mw = 0.6;
  lv.qd_mvar = 0.2;
  lv.in_service = true;

  ac.buses = {hv, mv, lv};

  Transformer3W tr;
  tr.index = 1;
  tr.name = "t135";
  tr.hv_bus = 1;
  tr.mv_bus = 2;
  tr.lv_bus = 3;
  tr.in_service = true;
  tr.sn_hv_mva = 10.0;
  tr.sn_mv_mva = 10.0;
  tr.sn_lv_mva = 10.0;
  tr.vn_hv_kv = 33.0;
  tr.vn_mv_kv = 11.0;
  tr.vn_lv_kv = 6.6;
  tr.vk_hv_mv_percent = 10.0;
  tr.vk_hv_lv_percent = 11.0;
  tr.vk_mv_lv_percent = 12.0;
  tr.vkr_hv_mv_percent = 0.0;
  tr.vkr_hv_lv_percent = 0.0;
  tr.vkr_mv_lv_percent = 0.0;
  tr.tap_side = 1;
  tr.tap_pos = 4;
  tr.tap_step_percent = 1.25;
  ac.transformers_3w = {tr};

  Generator slack;
  slack.index = 1;
  slack.bus = 1;
  slack.in_service = true;
  slack.is_slack = true;
  slack.vg_pu = 1.0;
  slack.pmax_mw = 100.0;
  slack.pmin_mw = 0.0;
  slack.qmax_mvar = 100.0;
  slack.qmin_mvar = -100.0;
  ac.generators = {slack};

  return sys;
}

inline HybridPowerSystem build_transformer3w_hv_spur_cross_check_case() {
  HybridPowerSystem sys = build_minimal_transformer3w_cross_check_case();
  sys.name = "minimal_transformer3w_tap_1ph_hv_spur";

  ACBus hv_spur;
  hv_spur.index = 4;
  hv_spur.name = "hv_spur";
  hv_spur.base_kv = 33.0;
  hv_spur.bus_type = BusType::PQ;
  hv_spur.vm_pu = 1.0;
  hv_spur.va_deg = 0.0;
  hv_spur.pd_mw = 0.15;
  hv_spur.qd_mvar = 0.05;
  hv_spur.in_service = true;
  sys.ac.buses.push_back(hv_spur);

  ACBranch l_hv_spur;
  l_hv_spur.index = 1;
  l_hv_spur.name = "l_hv_spur";
  l_hv_spur.from_bus = 1;
  l_hv_spur.to_bus = 4;
  l_hv_spur.r_pu = 0.02;
  l_hv_spur.x_pu = 0.04;
  l_hv_spur.tap = 1.0;
  l_hv_spur.in_service = true;
  sys.ac.branches = {l_hv_spur};

  return sys;
}

inline RegulatorOpenDSSCase accepted_regulator_case() {
  return {
      .case_id = "minimal_regulator_1ph",
      .acceptance_tier = "accepted_regulator",
      .relative_master_dss =
          "tests/data/opendss/minimal_regulator_1ph/Master.dss",
      .transformer_name = "t12",
      .line_name = "l1",
      .regcontrol_name = "reg1",
      .monitored_bus_name = "bus3",
      .monitored_bus_node = 1,
      .winding = 2,
      .transformer_origin_index = 1,
      .tap_winding = 2,
      .tap_side = 1,
      .tap_min = -16,
      .tap_max = 16,
      .tap_neutral = 0,
      .expected_final_tap_number = 8,
      .expected_final_tap_pu = 1.05,
      .tap_step_percent = 0.625,
      .forward_vreg_volts = 122.0,
      .forward_band_volts = 2.0,
      .ptratio = 60.0,
      .remote_ptratio = 60.0,
      .ct_primary_amps = 300.0,
      .forward_r_volts = 0.0,
      .forward_x_volts = 0.0,
      .max_tap_change = 1,
      .implicit_local_bus_monitoring = false,
  };
}

inline RegulatorOpenDSSCase accepted_local_pt_with_ldc_regulator_case() {
  return {
      .case_id = "minimal_regulator_local_pt_ldc_1ph",
      .acceptance_tier = "accepted_regulator_local_pt_ldc",
      .relative_master_dss =
          "tests/data/opendss/minimal_regulator_local_pt_ldc_1ph/Master.dss",
      .transformer_name = "t12",
      .line_name = "l1",
      .regcontrol_name = "reg1",
      .monitored_bus_name = "bus2",
      .monitored_bus_node = 1,
      .winding = 2,
      .transformer_origin_index = 1,
      .tap_winding = 2,
      .tap_side = 1,
      .tap_min = -16,
      .tap_max = 16,
      .tap_neutral = 0,
      .expected_final_tap_number = 4,
      .expected_final_tap_pu = 1.025,
      .tap_step_percent = 0.625,
      .forward_vreg_volts = 122.0,
      .forward_band_volts = 2.0,
      .ptratio = 60.0,
      .remote_ptratio = 60.0,
      .ct_primary_amps = 300.0,
      .forward_r_volts = 0.1,
      .forward_x_volts = 0.2,
      .max_tap_change = 1,
      .implicit_local_bus_monitoring = true,
  };
}

inline RegulatorOpenDSSCase accepted_remote_ptratio_regulator_case() {
  return {
      .case_id = "minimal_regulator_remote_ptratio_1ph",
      .acceptance_tier = "accepted_regulator_remote_ptratio",
      .relative_master_dss =
          "tests/data/opendss/minimal_regulator_remote_ptratio_1ph/Master.dss",
      .transformer_name = "t12",
      .line_name = "l1",
      .regcontrol_name = "reg1",
      .monitored_bus_name = "bus3",
      .monitored_bus_node = 1,
      .winding = 2,
      .transformer_origin_index = 1,
      .tap_winding = 2,
      .tap_side = 1,
      .tap_min = -16,
      .tap_max = 16,
      .tap_neutral = 0,
      .expected_final_tap_number = -4,
      .expected_final_tap_pu = 0.975,
      .tap_step_percent = 0.625,
      .forward_vreg_volts = 122.0,
      .forward_band_volts = 2.0,
      .ptratio = 60.0,
      .remote_ptratio = 55.0,
      .ct_primary_amps = 300.0,
      .forward_r_volts = 0.0,
      .forward_x_volts = 0.0,
      .max_tap_change = 1,
      .implicit_local_bus_monitoring = false,
  };
}

inline Transformer3WCrossCheckCase transformer3w_formal_cross_check_case() {
  return {
      .case_id = "minimal_transformer3w_tap_1ph",
      .acceptance_tier = "accepted_transformer3w_isolated_triangle",
      .relative_master_dss =
          "tests/data/opendss/minimal_transformer3w_tap_1ph/Master.dss",
      .transformer_name = "t135",
      .bus_names = {"hv", "mv", "lv"},
      .bus_nodes = {1, 1, 1},
      .observed_bus_names = {"hv", "mv", "lv"},
      .observed_bus_nodes = {1, 1, 1},
      .external_line_name = "",
      .transformer_origin_index = 1,
      .tap_side = 1,
      .tap_pos = 4,
      .tap_step_percent = 1.25,
  };
}

inline Transformer3WCrossCheckCase transformer3w_hv_spur_formal_cross_check_case() {
  return {
      .case_id = "minimal_transformer3w_tap_1ph_hv_spur",
      .acceptance_tier = "accepted_transformer3w_hv_spur_embed",
      .relative_master_dss =
          "tests/data/opendss/minimal_transformer3w_tap_1ph_hv_spur/Master.dss",
      .transformer_name = "t135",
      .bus_names = {"hv", "mv", "lv"},
      .bus_nodes = {1, 1, 1},
      .observed_bus_names = {"hv", "mv", "lv", "hv_spur"},
      .observed_bus_nodes = {1, 1, 1, 1},
      .external_line_name = "l_hv_spur",
      .transformer_origin_index = 1,
      .tap_side = 1,
      .tap_pos = 4,
      .tap_step_percent = 1.25,
  };
}

// ---------------------------------------------------------------------------
// NR p.u. convention adapter
//
// The NR solver uses phase_domain_power_base_scale = 3/base_mva, which
// implies a per-phase power base S_base_1ph = base_mva/3 and therefore
// Z_base = V_LN² / (base_mva/3) = V_LL² / base_mva.
//
// The BFS-convention fixtures (build_balanced_3bus_3phase, etc.) compute
// impedances using Z_base = V_LN² / base_mva, which is 3× smaller.
//
// This helper rescales line impedances from BFS to NR convention:
//   - Series impedance (r, x): divide by 3
//   - Shunt susceptance (b):   multiply by 3
// ---------------------------------------------------------------------------
inline void rescale_lines_bfs_to_nr(ThreePhaseACSystem& sys) {
  constexpr double kBfsToNrZ = 1.0 / 3.0;
  constexpr double kBfsToNrB = 3.0;
  for (auto& line : sys.lines) {
    line.r1_pu *= kBfsToNrZ;
    line.x1_pu *= kBfsToNrZ;
    line.r0_pu *= kBfsToNrZ;
    line.x0_pu *= kBfsToNrZ;
    line.b1_pu *= kBfsToNrB;
    for (auto& v : line.r_matrix_pu) v *= kBfsToNrZ;
    for (auto& v : line.x_matrix_pu) v *= kBfsToNrZ;
    for (auto& v : line.b_matrix_pu) v *= kBfsToNrB;
  }
}

inline ThreePhaseACSystem build_meshed_3bus_3phase_nr_case() {
  ThreePhaseACSystem sys = build_balanced_3bus_3phase();
  sys.name = "minimal_meshed_3bus_3ph_nr";

  ThreePhaseACLine l3;
  l3.index = 3;
  l3.name = "l3";
  l3.from_bus = 1;
  l3.to_bus = 3;
  l3.r1_pu = 0.12;
  l3.x1_pu = 0.25;
  l3.r0_pu = 0.12;
  l3.x0_pu = 0.25;
  l3.in_service = true;
  l3.rate_a_mva = 100.0;
  sys.lines.push_back(l3);
  rescale_lines_bfs_to_nr(sys);
  return sys;
}

inline ThreePhaseNROpenDSSCase accepted_meshed_3bus_3phase_nr_case() {
  return {
      .case_id = "minimal_meshed_3bus_3ph_nr",
      .acceptance_tier = "accepted_nr_meshed_3ph",
      .relative_master_dss =
          "tests/data/opendss/minimal_meshed_3bus_3ph_nr/Master.dss",
      .bus_names = {"sourcebus", "bus2", "bus3"},
      .line_names = {"l1", "l2", "l3"},
  };
}

inline ThreePhaseACSystem build_phase_matrix_line_3bus_3phase_nr_case() {
  ThreePhaseACSystem sys = build_balanced_3bus_3phase();
  sys.name = "minimal_phase_matrix_line_3bus_3ph_nr";

  sys.lines[0].use_phase_matrix = true;
  sys.lines[0].r_matrix_pu = {
      0.100, 0.030, 0.020,
      0.030, 0.115, 0.025,
      0.020, 0.025, 0.095,
  };
  sys.lines[0].x_matrix_pu = {
      0.220, 0.080, 0.060,
      0.080, 0.210, 0.070,
      0.060, 0.070, 0.235,
  };
  sys.lines[0].b_matrix_pu = {
      0.0018, -0.0004, -0.0002,
      -0.0004, 0.0016, -0.0003,
      -0.0002, -0.0003, 0.0019,
  };

  sys.lines[1].use_phase_matrix = true;
  sys.lines[1].r_matrix_pu = {
      0.080, 0.022, 0.016,
      0.022, 0.087, 0.018,
      0.016, 0.018, 0.076,
  };
  sys.lines[1].x_matrix_pu = {
      0.160, 0.055, 0.040,
      0.055, 0.170, 0.048,
      0.040, 0.048, 0.150,
  };
  sys.lines[1].b_matrix_pu = {
      0.0013, -0.0003, -0.00015,
      -0.0003, 0.0012, -0.0002,
      -0.00015, -0.0002, 0.0014,
  };
  rescale_lines_bfs_to_nr(sys);
  return sys;
}

inline ThreePhaseACSystem build_three_phase_nr_load_compare_base_case(
    const std::string& case_name) {
  ThreePhaseACSystem sys = build_balanced_3bus_3phase();
  sys.name = case_name;
  for (auto& bus : sys.buses) {
    bus.base_kv = 12.47;
    bus.pd_a_mw = 0.0;
    bus.qd_a_mvar = 0.0;
    bus.pd_b_mw = 0.0;
    bus.qd_b_mvar = 0.0;
    bus.pd_c_mw = 0.0;
    bus.qd_c_mvar = 0.0;
  }
  rescale_lines_bfs_to_nr(sys);
  return sys;
}

inline ThreePhaseACSystem build_three_phase_nr_single_phase_lateral_load_compare_base_case(
    const std::string& case_name) {
  ThreePhaseACSystem sys = build_three_phase_nr_load_compare_base_case(case_name);
  sys.buses[2].phase_mask = PhaseMask::a();
  sys.buses[2].base_kv = 7.2;
  sys.lines[1].phase_mask = PhaseMask::a();
  return sys;
}

inline ThreePhaseACSystem build_three_phase_nr_two_phase_lateral_load_compare_base_case(
    const std::string& case_name) {
  ThreePhaseACSystem sys = build_three_phase_nr_load_compare_base_case(case_name);
  sys.buses[2].phase_mask = PhaseMask::ab();
  sys.buses[2].base_kv = 7.2;
  sys.lines[1].phase_mask = PhaseMask::ab();
  return sys;
}

inline ThreePhaseLoad make_three_phase_compare_load(
    int index,
    int bus,
    const std::string& connection) {
  ThreePhaseLoad load;
  load.index = index;
  load.bus = bus;
  load.phase_mask = PhaseMask::abc();
  load.connection = connection;
  load.grounded = true;
  load.in_service = true;
  load.vmin_pu = 1e-6;
  load.vmax_pu = 2.0;
  load.zipv_cutoff_pu = 0.0;
  return load;
}

inline void set_balanced_three_phase_slack_vm(ThreePhaseACSystem& sys, double vm_pu) {
  sys.buses[0].vm_a_pu = vm_pu;
  sys.buses[0].vm_b_pu = vm_pu;
  sys.buses[0].vm_c_pu = vm_pu;
  if (!sys.generators.empty()) {
    sys.generators[0].vm_pu = vm_pu;
  }
}

inline ThreePhaseACSystem build_load_wye_zip_3bus_3phase_nr_case() {
  ThreePhaseACSystem sys =
      build_three_phase_nr_load_compare_base_case("minimal_load_wye_zip_3bus_3ph_nr");
  ThreePhaseLoad load = make_three_phase_compare_load(1, 3, "wye");
  load.p_a_mw = 0.60; load.q_a_mvar = 0.30;
  load.p_b_mw = 0.60; load.q_b_mvar = 0.30;
  load.p_c_mw = 0.60; load.q_c_mvar = 0.30;
  load.p_const_z_percent = 20.0;
  load.p_const_i_percent = 30.0;
  load.p_const_p_percent = 50.0;
  load.q_const_z_percent = 40.0;
  load.q_const_i_percent = 20.0;
  load.q_const_p_percent = 40.0;
  sys.loads = {load};
  return sys;
}

inline ThreePhaseACSystem build_load_wye_open_neutral_3bus_1phase_lateral_nr_case() {
  ThreePhaseACSystem sys = build_three_phase_nr_single_phase_lateral_load_compare_base_case(
      "minimal_load_wye_open_neutral_3bus_1ph_lateral_nr");
  ThreePhaseLoad load = make_three_phase_compare_load(1, 3, "wye");
  load.phase_mask = PhaseMask::a();
  load.grounded = false;
  load.p_a_mw = 0.55;
  load.q_a_mvar = 0.22;
  load.const_z_percent = 0.0;
  load.const_i_percent = 0.0;
  load.const_p_percent = 100.0;
  sys.loads = {load};
  return sys;
}

inline ThreePhaseACSystem build_load_wye_impedance_grounded_3bus_1phase_lateral_nr_case() {
  ThreePhaseACSystem sys = build_three_phase_nr_single_phase_lateral_load_compare_base_case(
      "minimal_load_wye_impedance_grounded_3bus_1ph_lateral_nr");
  ThreePhaseLoad load = make_three_phase_compare_load(1, 3, "wye");
  load.phase_mask = PhaseMask::a();
  load.p_a_mw = 0.55;
  load.q_a_mvar = 0.22;
  load.r_neut_ohm = 5.0;
  load.x_neut_ohm = 1.5;
  load.const_z_percent = 100.0;
  load.const_i_percent = 0.0;
  load.const_p_percent = 0.0;
  sys.loads = {load};
  return sys;
}

inline ThreePhaseACSystem build_load_wye_vmax_3bus_1phase_lateral_nr_case() {
  ThreePhaseACSystem sys = build_three_phase_nr_single_phase_lateral_load_compare_base_case(
      "minimal_load_wye_vmax_3bus_1ph_lateral_nr");
  set_balanced_three_phase_slack_vm(sys, 1.10);
  ThreePhaseLoad load = make_three_phase_compare_load(1, 3, "wye");
  load.phase_mask = PhaseMask::a();
  load.p_a_mw = 0.30;
  load.q_a_mvar = 0.12;
  load.vmax_pu = 1.01;
  load.const_z_percent = 0.0;
  load.const_i_percent = 0.0;
  load.const_p_percent = 100.0;
  sys.loads = {load};
  return sys;
}

inline ThreePhaseACSystem build_load_delta_power_3bus_3phase_nr_case() {
  ThreePhaseACSystem sys =
      build_three_phase_nr_load_compare_base_case("minimal_load_delta_power_3bus_3ph_nr");
  ThreePhaseLoad load = make_three_phase_compare_load(1, 3, "delta");
  load.p_a_mw = 0.50; load.q_a_mvar = 0.20;
  load.p_b_mw = 0.50; load.q_b_mvar = 0.20;
  load.p_c_mw = 0.50; load.q_c_mvar = 0.20;
  load.const_z_percent = 0.0;
  load.const_i_percent = 0.0;
  load.const_p_percent = 100.0;
  sys.loads = {load};
  return sys;
}

inline ThreePhaseACSystem build_load_delta_ab_power_3bus_2phase_lateral_nr_case() {
  ThreePhaseACSystem sys = build_three_phase_nr_two_phase_lateral_load_compare_base_case(
      "minimal_load_delta_ab_power_3bus_2ph_lateral_nr");
  ThreePhaseLoad load = make_three_phase_compare_load(1, 3, "delta");
  load.phase_mask = PhaseMask::ab();
  load.p_a_mw = 0.45;
  load.q_a_mvar = 0.18;
  load.p_b_mw = 0.30;
  load.q_b_mvar = 0.12;
  load.const_z_percent = 0.0;
  load.const_i_percent = 0.0;
  load.const_p_percent = 100.0;
  sys.loads = {load};
  return sys;
}

inline ThreePhaseACSystem build_load_delta_zip_3bus_3phase_nr_case() {
  ThreePhaseACSystem sys =
      build_three_phase_nr_load_compare_base_case("minimal_load_delta_zip_3bus_3ph_nr");
  ThreePhaseLoad load = make_three_phase_compare_load(1, 3, "delta");
  load.p_a_mw = 0.48; load.q_a_mvar = 0.18;
  load.p_b_mw = 0.48; load.q_b_mvar = 0.18;
  load.p_c_mw = 0.48; load.q_c_mvar = 0.18;
  load.p_const_z_percent = 25.0;
  load.p_const_i_percent = 25.0;
  load.p_const_p_percent = 50.0;
  load.q_const_z_percent = 35.0;
  load.q_const_i_percent = 15.0;
  load.q_const_p_percent = 50.0;
  sys.loads = {load};
  return sys;
}

inline ThreePhaseNROpenDSSCase accepted_load_wye_zip_3bus_3phase_nr_case() {
  return {
      .case_id = "minimal_load_wye_zip_3bus_3ph_nr",
      .acceptance_tier = "accepted_nr_load_semantics_3ph",
      .relative_master_dss =
          "tests/data/opendss/minimal_load_wye_zip_3bus_3ph_nr/Master.dss",
      .bus_names = {"sourcebus", "bus2", "bus3"},
      .line_names = {"l1", "l2"},
  };
}

inline ThreePhaseNROpenDSSCase accepted_load_wye_open_neutral_3bus_1phase_lateral_nr_case() {
  return {
      .case_id = "minimal_load_wye_open_neutral_3bus_1ph_lateral_nr",
      .acceptance_tier = "accepted_nr_load_semantics_3ph",
      .relative_master_dss =
          "tests/data/opendss/minimal_load_wye_open_neutral_3bus_1ph_lateral_nr/Master.dss",
      .bus_names = {"sourcebus", "bus2", "bus3"},
      .line_names = {"l1", "l2"},
  };
}

inline ThreePhaseNROpenDSSCase accepted_load_wye_impedance_grounded_3bus_1phase_lateral_nr_case() {
  return {
      .case_id = "minimal_load_wye_impedance_grounded_3bus_1ph_lateral_nr",
      .acceptance_tier = "accepted_nr_load_semantics_3ph",
      .relative_master_dss =
          "tests/data/opendss/minimal_load_wye_impedance_grounded_3bus_1ph_lateral_nr/Master.dss",
      .bus_names = {"sourcebus", "bus2", "bus3"},
      .line_names = {"l1", "l2"},
  };
}

inline ThreePhaseNROpenDSSCase accepted_load_wye_vmax_3bus_1phase_lateral_nr_case() {
  return {
      .case_id = "minimal_load_wye_vmax_3bus_1ph_lateral_nr",
      .acceptance_tier = "accepted_nr_load_semantics_3ph",
      .relative_master_dss =
          "tests/data/opendss/minimal_load_wye_vmax_3bus_1ph_lateral_nr/Master.dss",
      .bus_names = {"sourcebus", "bus2", "bus3"},
      .line_names = {"l1", "l2"},
  };
}

inline ThreePhaseNROpenDSSCase accepted_load_delta_power_3bus_3phase_nr_case() {
  return {
      .case_id = "minimal_load_delta_power_3bus_3ph_nr",
      .acceptance_tier = "accepted_nr_load_semantics_3ph",
      .relative_master_dss =
          "tests/data/opendss/minimal_load_delta_power_3bus_3ph_nr/Master.dss",
      .bus_names = {"sourcebus", "bus2", "bus3"},
      .line_names = {"l1", "l2"},
  };
}

inline ThreePhaseNROpenDSSCase accepted_load_delta_zip_3bus_3phase_nr_case() {
  return {
      .case_id = "minimal_load_delta_zip_3bus_3ph_nr",
      .acceptance_tier = "accepted_nr_load_semantics_3ph",
      .relative_master_dss =
          "tests/data/opendss/minimal_load_delta_zip_3bus_3ph_nr/Master.dss",
      .bus_names = {"sourcebus", "bus2", "bus3"},
      .line_names = {"l1", "l2"},
  };
}

inline ThreePhaseNROpenDSSCase accepted_phase_matrix_line_3bus_3phase_nr_case() {
  return {
      .case_id = "minimal_phase_matrix_line_3bus_3ph_nr",
      .acceptance_tier = "accepted_nr_phase_matrix_line_3ph",
      .relative_master_dss =
          "tests/data/opendss/minimal_phase_matrix_line_3bus_3ph_nr/Master.dss",
      .bus_names = {"sourcebus", "bus2", "bus3"},
      .line_names = {"l1", "l2"},
  };
}

inline ThreePhaseACSystem build_multi_source_3bus_3phase_nr_case() {
  ThreePhaseACSystem sys = build_balanced_3bus_3phase();
  sys.name = "minimal_multi_source_3bus_3ph_nr";

  sys.buses[1].pd_a_mw = 0.4;
  sys.buses[1].qd_a_mvar = 0.15;
  sys.buses[1].pd_b_mw = 0.4;
  sys.buses[1].qd_b_mvar = 0.15;
  sys.buses[1].pd_c_mw = 0.4;
  sys.buses[1].qd_c_mvar = 0.15;

  sys.buses[2].pd_a_mw = 0.9;
  sys.buses[2].qd_a_mvar = 0.35;
  sys.buses[2].pd_b_mw = 0.9;
  sys.buses[2].qd_b_mvar = 0.35;
  sys.buses[2].pd_c_mw = 0.9;
  sys.buses[2].qd_c_mvar = 0.35;

  ThreePhaseGenerator dg;
  dg.index = 2;
  dg.name = "dg3";
  dg.bus = 3;
  dg.in_service = true;
  dg.is_slack = false;
  dg.p_mw = 0.0;
  dg.q_mvar = 0.0;
  dg.p_a_mw = 0.60;
  dg.q_a_mvar = 0.15;
  dg.p_b_mw = 0.40;
  dg.q_b_mvar = 0.10;
  dg.p_c_mw = 0.20;
  dg.q_c_mvar = 0.05;
  dg.vm_pu = 1.0;
  dg.pmax_mw = 2.0;
  dg.pmin_mw = 0.0;
  dg.qmax_mvar = 1.0;
  dg.qmin_mvar = -1.0;
  dg.mbase_mva = 2.0;
  sys.generators.push_back(dg);
  rescale_lines_bfs_to_nr(sys);
  return sys;
}

inline ThreePhaseNROpenDSSCase accepted_multi_source_3bus_3phase_nr_case() {
  return {
      .case_id = "minimal_multi_source_3bus_3ph_nr",
      .acceptance_tier = "accepted_nr_multi_source_3ph",
      .relative_master_dss =
          "tests/data/opendss/minimal_multi_source_3bus_3ph_nr/Master.dss",
      .bus_names = {"sourcebus", "bus2", "bus3"},
      .line_names = {"l1", "l2"},
  };
}

// ---------------------------------------------------------------------------
// IEEE 13-node test feeder: 4.16 kV backbone subset for three-phase NR
//
// Modeled subset: 8 three-phase buses (rg60 → 632 → 633/670 → 671 → 680/692 → 675)
// Slack bus rg60 voltage set to OpenDSS regulator output.
//
// Known limitations documented in compare result:
//   1. Sequence impedance approximation of asymmetric 3×3 phase impedance matrices
//   2. Substation and distribution transformers omitted (no NR transformer support)
//   3. 2-phase/1-phase laterals (645,646,684,611,652) excluded
//   4. Bus 634 (0.48 kV secondary) excluded; its load reflected to bus 633
//   5. Regulators modeled as known slack voltage, not as closed-loop control
//   6. Capacitor at bus 675 modeled as per-phase bus shunt susceptance
// ---------------------------------------------------------------------------
inline ThreePhaseACSystem build_ieee13_4kv_backbone_3phase() {
  ThreePhaseACSystem sys;
  sys.base_mva = 10.0;
  sys.base_freq_hz = 60.0;
  sys.name = "ieee_13_node_4kv_backbone_3ph_nr";

  const double base_kv = 4.16;
  const double zbase = base_kv * base_kv / sys.base_mva;  // 1.73056 Ω

  // === BUSES ===
  // 1: rg60 (slack, regulator output)
  ThreePhaseACBus rg60;
  rg60.index = 1; rg60.name = "rg60";
  rg60.bus_type = BusType::SLACK;
  rg60.base_kv = base_kv;
  rg60.vm_a_pu = 1.0560331204; rg60.va_a_deg = -0.01308692;
  rg60.vm_b_pu = 1.0373856704; rg60.va_b_deg = -120.01262142;
  rg60.vm_c_pu = 1.0560496847; rg60.va_c_deg = 119.98408438;

  // 2: 632
  ThreePhaseACBus b632;
  b632.index = 2; b632.name = "632";
  b632.bus_type = BusType::PQ;
  b632.base_kv = base_kv;
  // Reflected loads from 2-phase laterals 645 and 646:
  //   645: phase B, wye, 170+j125 kW
  //   646: phases B+C, delta, 230+j132 kW → per-phase ~115+j66
  b632.pd_b_mw = 0.170 + 0.115; b632.qd_b_mvar = 0.125 + 0.066;
  b632.pd_c_mw = 0.115;         b632.qd_c_mvar = 0.066;

  // 3: 633 (loads include reflected 634 secondary-side loads through XFM1)
  ThreePhaseACBus b633;
  b633.index = 3; b633.name = "633";
  b633.bus_type = BusType::PQ;
  b633.base_kv = base_kv;
  // 634 loads (wye, 0.277 kV LN): 160+j110, 120+j90, 120+j90 kW/kvar
  // Reflected to 4.16 kV primary as equivalent (ignore XFM1 impedance for now)
  b633.pd_a_mw = 0.160; b633.qd_a_mvar = 0.110;
  b633.pd_b_mw = 0.120; b633.qd_b_mvar = 0.090;
  b633.pd_c_mw = 0.120; b633.qd_c_mvar = 0.090;

  // 4: 670 (distributed load: 1/3 of 632→671 line load)
  ThreePhaseACBus b670;
  b670.index = 4; b670.name = "670";
  b670.bus_type = BusType::PQ;
  b670.base_kv = base_kv;
  b670.pd_a_mw = 0.017; b670.qd_a_mvar = 0.010;
  b670.pd_b_mw = 0.066; b670.qd_b_mvar = 0.038;
  b670.pd_c_mw = 0.117; b670.qd_c_mvar = 0.068;

  // 5: 671 (large delta load 1155+j660, split per-phase for our model)
  // Delta load at 4.16 kV: 3-phase total 1155 kW + 660 kVAR
  // Per-phase for balanced delta: P/3 each
  // Reflected loads from 1-phase/2-phase laterals via 684:
  //   652: phase A, wye, 128+j86 kW (const-Z model 2 - approx as const-P)
  //   611: phase C, wye, 170+j80 kW (const-I model 5 - approx as const-P)
  ThreePhaseACBus b671;
  b671.index = 5; b671.name = "671";
  b671.bus_type = BusType::PQ;
  b671.base_kv = base_kv;
  b671.pd_a_mw = 0.385 + 0.128; b671.qd_a_mvar = 0.220 + 0.086;
  b671.pd_b_mw = 0.385;         b671.qd_b_mvar = 0.220;
  b671.pd_c_mw = 0.385 + 0.170; b671.qd_c_mvar = 0.220 + 0.080;
  // 611 capacitor: 100 kVAR on phase C
  b671.bs_c_mvar = 0.100;

  // 6: 680 (no load, dead-end bus)
  ThreePhaseACBus b680;
  b680.index = 6; b680.name = "680";
  b680.bus_type = BusType::PQ;
  b680.base_kv = base_kv;

  // 7: 692 (delta load on phases C-A: 170+j151)
  // For simplicity, split evenly between A and C
  ThreePhaseACBus b692;
  b692.index = 7; b692.name = "692";
  b692.bus_type = BusType::PQ;
  b692.base_kv = base_kv;
  b692.pd_a_mw = 0.085; b692.qd_a_mvar = 0.0755;
  b692.pd_c_mw = 0.085; b692.qd_c_mvar = 0.0755;

  // 8: 675 (per-phase wye loads + 600 kVAR capacitor bank)
  ThreePhaseACBus b675;
  b675.index = 8; b675.name = "675";
  b675.bus_type = BusType::PQ;
  b675.base_kv = base_kv;
  b675.pd_a_mw = 0.485; b675.qd_a_mvar = 0.190;
  b675.pd_b_mw = 0.068; b675.qd_b_mvar = 0.060;
  b675.pd_c_mw = 0.290; b675.qd_c_mvar = 0.212;
  // 600 kVAR 3-phase cap at 4.16 kV → per-phase: 200 kVAR
  b675.bs_a_mvar = 0.200;
  b675.bs_b_mvar = 0.200;
  b675.bs_c_mvar = 0.200;

  sys.buses = {rg60, b632, b633, b670, b671, b680, b692, b675};

  // === GENERATORS (slack) ===
  ThreePhaseGenerator gen;
  gen.index = 1; gen.name = "source";
  gen.bus = 1; gen.is_slack = true;
  gen.vm_pu = 1.0;  // per-phase Vm set on bus
  sys.generators = {gen};

  // === LINES ===
  // Sequence impedances derived from IEEE 13-node line code matrices.
  // Z1 = Zs - Zm,  Z0 = Zs + 2*Zm  (Zs=avg diagonal, Zm=avg off-diagonal)
  // All in Ω/mile. Length in feet → miles: /5280. Then pu: * len_mi / zbase.

  auto make_line = [&](int idx, const std::string& name,
                        int from, int to,
                        double r1_opm, double x1_opm,
                        double r0_opm, double x0_opm,
                        double len_ft) -> ThreePhaseACLine {
    ThreePhaseACLine l;
    l.index = idx; l.name = name;
    l.from_bus = from; l.to_bus = to;
    double len_mi = len_ft / 5280.0;
    l.r1_pu = r1_opm * len_mi / zbase;
    l.x1_pu = x1_opm * len_mi / zbase;
    l.r0_pu = r0_opm * len_mi / zbase;
    l.x0_pu = x0_opm * len_mi / zbase;
    return l;
  };

  // mtx601: Rs=0.3418, Rm=0.1558, Xs=1.0335, Xm=0.4367
  //  → r1=0.1860, x1=0.5968, r0=0.6534, x0=1.9070
  auto l_rg60_632 = make_line(1, "650632",  1, 2,
      0.1860, 0.5968, 0.6534, 1.9070, 2000.0);

  auto l_632_670 = make_line(2, "632670",  2, 4,
      0.1860, 0.5968, 0.6534, 1.9070, 667.0);

  auto l_670_671 = make_line(3, "670671",  4, 5,
      0.1860, 0.5968, 0.6534, 1.9070, 1333.0);

  auto l_671_680 = make_line(4, "671680",  5, 6,
      0.1860, 0.5968, 0.6534, 1.9070, 1000.0);

  // mtx602: Rs=0.7479, Rm=0.1558, Xs=1.1970, Xm=0.4367
  //  → r1=0.5921, x1=0.7603, r0=1.0595, x0=2.0704
  auto l_632_633 = make_line(5, "632633",  2, 3,
      0.5921, 0.7603, 1.0595, 2.0704, 500.0);

  // 671→692 switch (R1=1e-4 Ω total, X1=0)
  ThreePhaseACLine l_switch;
  l_switch.index = 6; l_switch.name = "671692";
  l_switch.from_bus = 5; l_switch.to_bus = 7;
  l_switch.r1_pu = 1.0e-4 / zbase;
  l_switch.x1_pu = 1.0e-6 / zbase;  // small nonzero
  l_switch.r0_pu = 1.0e-4 / zbase;
  l_switch.x0_pu = 1.0e-6 / zbase;

  // mtx606 (underground cable): Rs=0.7917, Rm=0.3067, Xs=0.4244, Xm=0.0027
  //  → r1=0.4850, x1=0.4217, r0=1.4051, x0=0.4298
  auto l_692_675 = make_line(7, "692675",  7, 8,
      0.4850, 0.4217, 1.4051, 0.4298, 500.0);

  sys.lines = {l_rg60_632, l_632_670, l_670_671, l_671_680,
               l_632_633, l_switch, l_692_675};

  return sys;
}

inline ThreePhaseNROpenDSSCase ieee13_4kv_backbone_3phase_nr_case() {
  return {
      .case_id = "ieee_13_node_4kv_backbone_3ph_nr",
      .acceptance_tier = "deferred_ieee13_feeder_formal_cross_check",
      .relative_master_dss =
          "external_data/opendss_ieee_pes/opendss_reference/13_node/"
          "official_full/IEEE13Nodeckt.dss",
      .bus_names = {"rg60", "632", "633", "670", "671", "680", "692", "675"},
      .line_names = {"650632", "632670", "670671", "671680",
                     "632633", "671692", "692675"},
  };
}

}  // namespace hacdcpf_compare_fixtures
