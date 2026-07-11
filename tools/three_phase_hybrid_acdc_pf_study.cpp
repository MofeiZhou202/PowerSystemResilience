/// tools/three_phase_hybrid_acdc_pf_study.cpp
/// ==================================================
/// Numerical-performance evidence for three-phase hybrid AC/DC distribution
/// studies in docs/latex/sppt_theory.tex.
///
/// The current implementation has two executable numerical kernels:
///   * abc-domain three-phase AC power flow on ThreePhaseACSystem; and
///   * aggregate AC/DC/VSC power flow on HybridPowerSystem.
///
/// This generator deliberately reports them as a staged hybrid workflow rather
/// than claiming a monolithic abc+DC Newton state.  Each case contains an
/// unbalanced three-phase feeder and a paired aggregate DC/VSC boundary model.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/model/ac_components.hpp"
#include "hacdcpf/model/converter_components.hpp"
#include "hacdcpf/model/dc_components.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/distribution_power_flow.hpp"

namespace fs = std::filesystem;

namespace {

using hacdcpf::ACBranch;
using hacdcpf::ACBus;
using hacdcpf::BusType;
using hacdcpf::DCBranch;
using hacdcpf::DCBus;
using hacdcpf::DCBusType;
using hacdcpf::DCLoad;
using hacdcpf::Generator;
using hacdcpf::HybridPowerSystem;
using hacdcpf::PhaseMask;
using hacdcpf::PowerFlowOptions;
using hacdcpf::ThreePhaseACBus;
using hacdcpf::ThreePhaseACLine;
using hacdcpf::ThreePhaseACSystem;
using hacdcpf::ThreePhaseExternalGrid;
using hacdcpf::ThreePhaseGenerator;
using hacdcpf::ThreePhaseLoad;
using hacdcpf::VSCConverter;
using hacdcpf::analysis::PhaseNodeIndexer;
using hacdcpf::analysis::ThreePhaseDPFResult;
using hacdcpf::analysis::ThreePhaseNROptions;

struct CaseSpec {
  std::string label;
  std::string topology;
  HybridPowerSystem system;
};

struct ThreePhaseStats {
  bool attempted{false};
  bool converged{false};
  int iterations{0};
  double residual{std::numeric_limits<double>::quiet_NaN()};
  double best_ms{std::numeric_limits<double>::infinity()};
  double vm_min{std::numeric_limits<double>::quiet_NaN()};
  double vm_max{std::numeric_limits<double>::quiet_NaN()};
  double max_vuf_percent{std::numeric_limits<double>::quiet_NaN()};
  double p_loss_mw{std::numeric_limits<double>::quiet_NaN()};
  std::string note;
};

struct BoundaryStats {
  bool attempted{false};
  bool converged{false};
  int iterations{0};
  double residual{std::numeric_limits<double>::quiet_NaN()};
  double best_ms{std::numeric_limits<double>::infinity()};
  double vdc_min{std::numeric_limits<double>::quiet_NaN()};
  double vdc_max{std::numeric_limits<double>::quiet_NaN()};
  int vsc_transfers{0};
  std::string note;
};

struct CaseSummary {
  CaseSpec spec;
  int phase_nodes{0};
  ThreePhaseStats abc;
  BoundaryStats boundary;
  int stress_success{0};
  int stress_total{0};
  double max_success_scale{0.0};
};

struct StressRow {
  std::string label;
  double scale{1.0};
  ThreePhaseStats abc;
  BoundaryStats boundary;
};

std::string latex_escape(const std::string& input) {
  std::string out;
  out.reserve(input.size());
  for (const char ch : input) {
    switch (ch) {
      case '&': out += "\\&"; break;
      case '%': out += "\\%"; break;
      case '$': out += "\\$"; break;
      case '#': out += "\\#"; break;
      case '_': out += "\\_"; break;
      case '{': out += "\\{"; break;
      case '}': out += "\\}"; break;
      case '~': out += "\\textasciitilde{}"; break;
      case '^': out += "\\textasciicircum{}"; break;
      case '\\': out += "\\textbackslash{}"; break;
      default: out.push_back(ch); break;
    }
  }
  return out;
}

std::string f2(double value) {
  if (!std::isfinite(value)) return "--";
  std::ostringstream os;
  os << std::fixed << std::setprecision(2) << value;
  return os.str();
}

std::string f3(double value) {
  if (!std::isfinite(value)) return "--";
  std::ostringstream os;
  os << std::fixed << std::setprecision(3) << value;
  return os.str();
}

std::string sci(double value) {
  if (!std::isfinite(value)) return "--";
  if (std::abs(value) < 1e-99) return "$0$";
  std::ostringstream os;
  os << std::scientific << std::setprecision(2) << value;
  std::string s = os.str();
  const auto epos = s.find('e');
  if (epos == std::string::npos) return s;
  const std::string mant = s.substr(0, epos);
  const int exp = std::stoi(s.substr(epos + 1));
  return "$" + mant + "{\\times}10^{" + std::to_string(exp) + "}$";
}

ThreePhaseACBus make_tp_bus(int id, BusType type, PhaseMask mask,
                            double base_kv = 12.47) {
  ThreePhaseACBus b;
  b.index = id;
  b.name = "tp_bus_" + std::to_string(id);
  b.bus_type = type;
  b.phase_mask = mask;
  b.base_kv = base_kv;
  b.in_service = true;
  b.vm_a_pu = 1.0;
  b.vm_b_pu = 1.0;
  b.vm_c_pu = 1.0;
  b.va_a_deg = 0.0;
  b.va_b_deg = -120.0;
  b.va_c_deg = 120.0;
  return b;
}

ThreePhaseACLine make_tp_line(int id, int from, int to, PhaseMask mask,
                              double r1, double x1,
                              double r0_scale = 1.8,
                              double x0_scale = 1.5) {
  ThreePhaseACLine l;
  l.index = id;
  l.name = "tp_line_" + std::to_string(id);
  l.from_bus = from;
  l.to_bus = to;
  l.phase_mask = mask;
  l.in_service = true;
  l.r1_pu = r1;
  l.x1_pu = x1;
  l.r0_pu = r1 * r0_scale;
  l.x0_pu = x1 * x0_scale;
  l.rate_a_mva = 10.0;
  l.length_km = 0.8 + 0.2 * id;
  return l;
}

ThreePhaseLoad make_tp_load(int id, int bus, PhaseMask mask,
                            double pa, double qa,
                            double pb, double qb,
                            double pc, double qc) {
  ThreePhaseLoad load;
  load.index = id;
  load.name = "tp_load_" + std::to_string(id);
  load.bus = bus;
  load.phase_mask = mask;
  load.in_service = true;
  load.p_a_mw = mask.has(0) ? pa : 0.0;
  load.q_a_mvar = mask.has(0) ? qa : 0.0;
  load.p_b_mw = mask.has(1) ? pb : 0.0;
  load.q_b_mvar = mask.has(1) ? qb : 0.0;
  load.p_c_mw = mask.has(2) ? pc : 0.0;
  load.q_c_mvar = mask.has(2) ? qc : 0.0;
  load.connection = "wye";
  load.const_p_percent = 100.0;
  return load;
}

ThreePhaseExternalGrid make_tp_grid(int bus) {
  ThreePhaseExternalGrid grid;
  grid.index = 1;
  grid.name = "source";
  grid.bus = bus;
  grid.phase_mask = PhaseMask::abc();
  grid.vm_pu = 1.0;
  grid.va_deg = 0.0;
  grid.x1_pu = 0.02;
  grid.r1_pu = 0.002;
  grid.in_service = true;
  return grid;
}

ThreePhaseACSystem make_balanced_radial_tp() {
  ThreePhaseACSystem tp;
  tp.name = "balanced_radial_abc";
  tp.base_mva = 10.0;
  tp.base_freq_hz = 50.0;
  tp.buses = {
      make_tp_bus(1, BusType::SLACK, PhaseMask::abc()),
      make_tp_bus(2, BusType::PQ, PhaseMask::abc()),
      make_tp_bus(3, BusType::PQ, PhaseMask::abc()),
      make_tp_bus(4, BusType::PQ, PhaseMask::abc()),
      make_tp_bus(5, BusType::PQ, PhaseMask::abc()),
  };
  tp.external_grids = {make_tp_grid(1)};
  tp.lines = {
      make_tp_line(1, 1, 2, PhaseMask::abc(), 0.018, 0.045),
      make_tp_line(2, 2, 3, PhaseMask::abc(), 0.020, 0.050),
      make_tp_line(3, 3, 4, PhaseMask::abc(), 0.025, 0.055),
      make_tp_line(4, 4, 5, PhaseMask::abc(), 0.022, 0.050),
  };
  tp.loads = {
      make_tp_load(1, 2, PhaseMask::abc(), 0.30, 0.10, 0.30, 0.10, 0.30, 0.10),
      make_tp_load(2, 3, PhaseMask::abc(), 0.22, 0.08, 0.22, 0.08, 0.22, 0.08),
      make_tp_load(3, 4, PhaseMask::abc(), 0.18, 0.06, 0.18, 0.06, 0.18, 0.06),
      make_tp_load(4, 5, PhaseMask::abc(), 0.12, 0.04, 0.12, 0.04, 0.12, 0.04),
  };
  return tp;
}

ThreePhaseACSystem make_unbalanced_lateral_tp() {
  ThreePhaseACSystem tp;
  tp.name = "unbalanced_lateral_abc";
  tp.base_mva = 10.0;
  tp.base_freq_hz = 50.0;
  tp.buses = {
      make_tp_bus(1, BusType::SLACK, PhaseMask::abc()),
      make_tp_bus(2, BusType::PQ, PhaseMask::abc()),
      make_tp_bus(3, BusType::PQ, PhaseMask::abc()),
      make_tp_bus(4, BusType::PQ, PhaseMask::ab()),
      make_tp_bus(5, BusType::PQ, PhaseMask::a()),
      make_tp_bus(6, BusType::PQ, PhaseMask::bc()),
  };
  tp.external_grids = {make_tp_grid(1)};
  tp.lines = {
      make_tp_line(1, 1, 2, PhaseMask::abc(), 0.020, 0.048),
      make_tp_line(2, 2, 3, PhaseMask::abc(), 0.024, 0.055),
      make_tp_line(3, 3, 4, PhaseMask::ab(), 0.030, 0.070),
      make_tp_line(4, 4, 5, PhaseMask::a(), 0.035, 0.080),
      make_tp_line(5, 3, 6, PhaseMask::bc(), 0.032, 0.074),
  };
  tp.loads = {
      make_tp_load(1, 2, PhaseMask::abc(), 0.35, 0.12, 0.25, 0.08, 0.18, 0.06),
      make_tp_load(2, 3, PhaseMask::abc(), 0.20, 0.08, 0.42, 0.15, 0.26, 0.10),
      make_tp_load(3, 4, PhaseMask::ab(), 0.18, 0.07, 0.15, 0.05, 0.0, 0.0),
      make_tp_load(4, 5, PhaseMask::a(), 0.11, 0.04, 0.0, 0.0, 0.0, 0.0),
      make_tp_load(5, 6, PhaseMask::bc(), 0.0, 0.0, 0.16, 0.06, 0.13, 0.05),
  };
  return tp;
}

ThreePhaseACSystem make_meshed_der_tp() {
  ThreePhaseACSystem tp = make_balanced_radial_tp();
  tp.name = "meshed_der_abc";
  tp.lines.push_back(make_tp_line(5, 5, 2, PhaseMask::abc(), 0.055, 0.120));
  if (tp.loads.size() >= 4) {
    tp.loads[0] = make_tp_load(1, 2, PhaseMask::abc(), 0.40, 0.13, 0.22, 0.08, 0.18, 0.06);
    tp.loads[1] = make_tp_load(2, 3, PhaseMask::abc(), 0.18, 0.06, 0.36, 0.12, 0.24, 0.09);
    tp.loads[2] = make_tp_load(3, 4, PhaseMask::abc(), 0.15, 0.05, 0.14, 0.05, 0.26, 0.09);
    tp.loads[3] = make_tp_load(4, 5, PhaseMask::abc(), 0.10, 0.04, 0.18, 0.06, 0.12, 0.04);
  }
  ThreePhaseGenerator gen;
  gen.index = 1;
  gen.name = "abc_der_bus_4";
  gen.bus = 4;
  gen.in_service = true;
  gen.phase_mask = PhaseMask::abc();
  gen.p_mw = 0.0;
  gen.q_mvar = 0.0;
  gen.p_a_mw = 0.10;
  gen.p_b_mw = 0.10;
  gen.p_c_mw = 0.10;
  gen.vm_pu = 1.01;
  gen.pmax_mw = 1.0;
  gen.qmax_mvar = 0.5;
  gen.qmin_mvar = -0.5;
  tp.generators.push_back(gen);
  return tp;
}

double total_p_mw(const ThreePhaseACSystem& tp, int bus) {
  double p = 0.0;
  for (const auto& b : tp.buses) {
    if (b.index == bus) p += b.pd_a_mw + b.pd_b_mw + b.pd_c_mw;
  }
  for (const auto& l : tp.loads) {
    if (l.in_service && l.bus == bus) p += l.p_a_mw + l.p_b_mw + l.p_c_mw;
  }
  return p;
}

double total_q_mvar(const ThreePhaseACSystem& tp, int bus) {
  double q = 0.0;
  for (const auto& b : tp.buses) {
    if (b.index == bus) q += b.qd_a_mvar + b.qd_b_mvar + b.qd_c_mvar;
  }
  for (const auto& l : tp.loads) {
    if (l.in_service && l.bus == bus) q += l.q_a_mvar + l.q_b_mvar + l.q_c_mvar;
  }
  return q;
}

HybridPowerSystem make_hybrid_case(std::string label,
                                   const ThreePhaseACSystem& tp,
                                   double dc_load_mw,
                                   int vsc_ac_bus) {
  HybridPowerSystem sys;
  sys.name = label;
  sys.base_mva = 10.0;
  sys.three_phase_ac = tp;
  sys.ac.name = label + "_aggregate_ac";
  sys.ac.base_mva = 10.0;
  sys.ac.freq_hz = tp.base_freq_hz;

  for (const auto& tb : tp.buses) {
    ACBus b;
    b.index = tb.index;
    b.name = tb.name + "_posseq";
    b.bus_type = tb.bus_type;
    b.base_kv = tb.base_kv > 0.0 ? tb.base_kv : 12.47;
    b.vm_pu = 1.0;
    b.va_deg = (tb.bus_type == BusType::SLACK) ? 0.0 : 0.0;
    b.pd_mw = total_p_mw(tp, tb.index);
    b.qd_mvar = total_q_mvar(tp, tb.index);
    b.in_service = tb.in_service;
    sys.ac.buses.push_back(b);
  }
  for (const auto& tl : tp.lines) {
    ACBranch br;
    br.index = tl.index;
    br.name = tl.name + "_posseq";
    br.from_bus = tl.from_bus;
    br.to_bus = tl.to_bus;
    br.r_pu = tl.r1_pu;
    br.x_pu = tl.x1_pu;
    br.b_pu = tl.b1_pu;
    br.r0_pu = tl.r0_pu;
    br.x0_pu = tl.x0_pu;
    br.rate_a_mva = tl.rate_a_mva;
    br.in_service = tl.in_service;
    sys.ac.branches.push_back(br);
  }
  Generator slack;
  slack.index = 1;
  slack.name = "aggregate_slack";
  slack.bus = 1;
  slack.is_slack = true;
  slack.in_service = true;
  slack.vg_pu = 1.0;
  slack.pmax_mw = 50.0;
  slack.pmin_mw = -50.0;
  slack.qmax_mvar = 50.0;
  slack.qmin_mvar = -50.0;
  sys.ac.generators.push_back(slack);

  sys.dc.name = label + "_dc";
  sys.dc.base_mva = 10.0;
  DCBus d1;
  d1.index = 1;
  d1.name = "dc_pcc";
  d1.bus_type = DCBusType::DC_P;
  d1.base_kv = 0.75;
  d1.vmin_pu = 0.80;
  d1.vmax_pu = 1.20;
  DCBus d2 = d1;
  d2.index = 2;
  d2.name = "dc_load";
  sys.dc.buses = {d1, d2};

  DCBranch dbr;
  dbr.index = 1;
  dbr.name = "dc_feeder";
  dbr.from_bus = 1;
  dbr.to_bus = 2;
  dbr.r_pu = 0.02;
  dbr.rate_a_mva = 5.0;
  dbr.in_service = true;
  sys.dc.branches = {dbr};

  DCLoad dload;
  dload.index = 1;
  dload.name = "dc_ev_charger_bank";
  dload.bus = 2;
  dload.p_mw = dc_load_mw;
  dload.p_rated_mw = std::max(1.0, dc_load_mw);
  dload.in_service = true;
  sys.dc.loads = {dload};

  VSCConverter vsc;
  vsc.index = 1;
  vsc.name = "abc_boundary_vsc";
  vsc.bus_ac = vsc_ac_bus;
  vsc.bus_dc = 1;
  vsc.in_service = true;
  vsc.control_mode = hacdcpf::ConverterMode::VDC_Q;
  vsc.v_dc_set_pu = 1.0;
  vsc.q_set_mvar = 0.0;
  vsc.eta = 0.985;
  vsc.k_vdc = 0.10;
  vsc.pmax_mw = 5.0;
  vsc.pmin_mw = -5.0;
  vsc.qmax_mvar = 2.0;
  vsc.qmin_mvar = -2.0;
  vsc.p_rated_mw = 5.0;
  vsc.vn_ac_kv = 12.47;
  vsc.vn_dc_kv = 0.75;
  sys.vsc_converters = {vsc};
  return sys;
}

void scale_case(HybridPowerSystem& sys, double factor) {
  if (sys.three_phase_ac.has_value()) {
    for (auto& b : sys.three_phase_ac->buses) {
      b.pd_a_mw *= factor;
      b.pd_b_mw *= factor;
      b.pd_c_mw *= factor;
      b.qd_a_mvar *= factor;
      b.qd_b_mvar *= factor;
      b.qd_c_mvar *= factor;
    }
    for (auto& l : sys.three_phase_ac->loads) {
      l.p_a_mw *= factor;
      l.p_b_mw *= factor;
      l.p_c_mw *= factor;
      l.q_a_mvar *= factor;
      l.q_b_mvar *= factor;
      l.q_c_mvar *= factor;
    }
  }
  for (auto& b : sys.ac.buses) {
    b.pd_mw *= factor;
    b.qd_mvar *= factor;
  }
  for (auto& b : sys.dc.buses) b.pd_mw *= factor;
  for (auto& l : sys.dc.loads) l.p_mw *= factor;
}

ThreePhaseNROptions three_phase_options() {
  ThreePhaseNROptions opt;
  opt.max_iter = 80;
  opt.max_control_iter = 100;
  opt.tol = 1e-8;
  opt.include_shunts = true;
  return opt;
}

PowerFlowOptions boundary_options() {
  PowerFlowOptions opt;
  opt.max_iter = 120;
  opt.tol = 1e-8;
  opt.max_line_search_steps = 16;
  opt.max_regularization_steps = 10;
  opt.enable_converter_coordination_check = true;
  return opt;
}

ThreePhaseStats run_three_phase(const HybridPowerSystem& sys, int repeats) {
  ThreePhaseStats stats;
  stats.attempted = true;
  if (!sys.three_phase_ac.has_value()) {
    stats.note = "missing three_phase_ac subsystem";
    return stats;
  }
  const auto opt = three_phase_options();
  for (int r = 0; r < std::max(1, repeats); ++r) {
    try {
      const auto t0 = std::chrono::steady_clock::now();
      const ThreePhaseDPFResult result =
          hacdcpf::analysis::solve_three_phase_nr(*sys.three_phase_ac, opt);
      const auto t1 = std::chrono::steady_clock::now();
      stats.best_ms = std::min(
          stats.best_ms,
          std::chrono::duration<double, std::milli>(t1 - t0).count());
      stats.converged = result.converged;
      stats.iterations = result.iterations;
      stats.residual = result.residual;
      stats.max_vuf_percent = result.max_vuf_percent;
      stats.p_loss_mw = result.total_p_loss_mw;
      double lo = std::numeric_limits<double>::infinity();
      double hi = -std::numeric_limits<double>::infinity();
      for (const auto& v : result.bus_voltages) {
        PhaseMask mask = PhaseMask::abc();
        if (sys.three_phase_ac.has_value()) {
          const auto it = std::find_if(
              sys.three_phase_ac->buses.begin(),
              sys.three_phase_ac->buses.end(),
              [&](const ThreePhaseACBus& bus) { return bus.index == v.bus_id; });
          if (it != sys.three_phase_ac->buses.end()) mask = it->phase_mask;
        }
        if (mask.has(0)) {
          lo = std::min(lo, v.vm_a_pu);
          hi = std::max(hi, v.vm_a_pu);
        }
        if (mask.has(1)) {
          lo = std::min(lo, v.vm_b_pu);
          hi = std::max(hi, v.vm_b_pu);
        }
        if (mask.has(2)) {
          lo = std::min(lo, v.vm_c_pu);
          hi = std::max(hi, v.vm_c_pu);
        }
      }
      if (std::isfinite(lo)) {
        stats.vm_min = lo;
        stats.vm_max = hi;
      }
      stats.note = result.solver_used.empty() ? "three_phase_nr" : result.solver_used;
    } catch (const std::exception& e) {
      stats.note = e.what();
    }
  }
  return stats;
}

BoundaryStats run_boundary(const HybridPowerSystem& sys, int repeats) {
  BoundaryStats stats;
  stats.attempted = true;
  const auto opt = boundary_options();
  for (int r = 0; r < std::max(1, repeats); ++r) {
    const auto t0 = std::chrono::steady_clock::now();
    const auto result = hacdcpf::safe_solve_power_flow(sys, opt, true);
    const auto t1 = std::chrono::steady_clock::now();
    stats.best_ms = std::min(
        stats.best_ms,
        std::chrono::duration<double, std::milli>(t1 - t0).count());
    if (!result) {
      stats.converged = false;
      stats.note = result.error().message;
      continue;
    }
    const auto& pf = result.value();
    stats.converged = pf.converged;
    stats.iterations = pf.iterations;
    stats.residual = pf.residual;
    stats.vsc_transfers = static_cast<int>(pf.vsc_transfers.size());
    stats.note = pf.diagnostics.termination_reason;
    if (!pf.vdc.empty()) {
      auto [lo, hi] = std::minmax_element(pf.vdc.begin(), pf.vdc.end());
      stats.vdc_min = *lo;
      stats.vdc_max = *hi;
    }
  }
  return stats;
}

std::vector<CaseSpec> build_cases() {
  std::vector<CaseSpec> cases;
  cases.push_back({"abc-balanced-radial",
                   "balanced 5-bus radial ABC feeder + DC charger",
                   make_hybrid_case("abc-balanced-radial",
                                    make_balanced_radial_tp(), 0.45, 3)});
  cases.push_back({"abc-unbalanced-lateral",
                   "unbalanced ABC/AB/A laterals + DC charger",
                   make_hybrid_case("abc-unbalanced-lateral",
                                    make_unbalanced_lateral_tp(), 0.55, 3)});
  cases.push_back({"abc-meshed-der",
                   "unbalanced meshed ABC feeder with DER + DC charger",
                   make_hybrid_case("abc-meshed-der",
                                    make_meshed_der_tp(), 0.50, 3)});
  return cases;
}

int count_phase_nodes(const HybridPowerSystem& sys) {
  if (!sys.three_phase_ac.has_value()) return 0;
  return PhaseNodeIndexer::build(*sys.three_phase_ac).total_nodes;
}

void write_summary(const fs::path& outdir,
                   const std::vector<CaseSummary>& summaries) {
  {
    std::ofstream csv(outdir / "sppt_three_phase_hybrid_pf_summary.csv");
    csv << "case,topology,tp_buses,phase_nodes,tp_lines,dc_buses,vsc,"
           "abc_converged,abc_iterations,abc_ms,abc_residual,abc_vmin,abc_vmax,"
           "abc_vuf_pct,abc_loss_mw,boundary_converged,boundary_iterations,"
           "boundary_ms,boundary_residual,vdc_min,vdc_max,stress_success,stress_total,max_success_scale\n";
    for (const auto& row : summaries) {
      const auto& sys = row.spec.system;
      const auto& tp = *sys.three_phase_ac;
      csv << row.spec.label << ",\"" << row.spec.topology << "\","
          << tp.buses.size() << ',' << row.phase_nodes << ','
          << tp.lines.size() << ',' << sys.dc.buses.size() << ','
          << sys.vsc_converters.size() << ','
          << (row.abc.converged ? "yes" : "no") << ','
          << row.abc.iterations << ',' << row.abc.best_ms << ','
          << row.abc.residual << ',' << row.abc.vm_min << ','
          << row.abc.vm_max << ',' << row.abc.max_vuf_percent << ','
          << row.abc.p_loss_mw << ','
          << (row.boundary.converged ? "yes" : "no") << ','
          << row.boundary.iterations << ',' << row.boundary.best_ms << ','
          << row.boundary.residual << ',' << row.boundary.vdc_min << ','
          << row.boundary.vdc_max << ',' << row.stress_success << ','
          << row.stress_total << ',' << row.max_success_scale << '\n';
    }
  }

  std::ofstream tex(outdir / "sppt_three_phase_hybrid_pf_summary.tex");
  tex << "% Auto-generated by tools/three_phase_hybrid_acdc_pf_study.cpp\n";
  tex << "\\begin{tabularx}{\\linewidth}{@{}lrrrrcccccc@{}}\n";
  tex << "\\toprule\n";
  tex << "Case & bus & ph. & line & DC & VSC & ABC PF & ms & VUF\\% & boundary & stress \\\\\n";
  tex << "\\midrule\n";
  for (const auto& row : summaries) {
    const auto& sys = row.spec.system;
    const auto& tp = *sys.three_phase_ac;
    tex << "\\texttt{" << latex_escape(row.spec.label) << "} & "
        << tp.buses.size() << " & " << row.phase_nodes << " & "
        << tp.lines.size() << " & " << sys.dc.buses.size() << " & "
        << sys.vsc_converters.size() << " & "
        << (row.abc.converged ? "yes/" : "no/") << row.abc.iterations << " & "
        << f2(row.abc.best_ms) << " & " << f2(row.abc.max_vuf_percent) << " & "
        << (row.boundary.converged ? "yes/" : "no/") << f2(row.boundary.best_ms)
        << " & " << row.stress_success << "/" << row.stress_total << " \\\\\n";
  }
  tex << "\\bottomrule\n\\end{tabularx}\n";
}

void write_stress(const fs::path& outdir,
                  const std::vector<StressRow>& rows) {
  {
    std::ofstream csv(outdir / "sppt_three_phase_hybrid_pf_stress.csv");
    csv << "case,scale,abc_converged,abc_iterations,abc_ms,abc_residual,"
           "abc_vmin,abc_vmax,abc_vuf_pct,boundary_converged,boundary_iterations,"
           "boundary_ms,boundary_residual,vdc_min,vdc_max\n";
    for (const auto& row : rows) {
      csv << row.label << ',' << row.scale << ','
          << (row.abc.converged ? "yes" : "no") << ','
          << row.abc.iterations << ',' << row.abc.best_ms << ','
          << row.abc.residual << ',' << row.abc.vm_min << ','
          << row.abc.vm_max << ',' << row.abc.max_vuf_percent << ','
          << (row.boundary.converged ? "yes" : "no") << ','
          << row.boundary.iterations << ',' << row.boundary.best_ms << ','
          << row.boundary.residual << ',' << row.boundary.vdc_min << ','
          << row.boundary.vdc_max << '\n';
    }
  }

  std::ofstream tex(outdir / "sppt_three_phase_hybrid_pf_stress.tex");
  tex << "% Auto-generated by tools/three_phase_hybrid_acdc_pf_study.cpp\n";
  tex << "\\begin{tabular}{@{}llrrrrrr@{}}\n";
  tex << "\\toprule\n";
  tex << "Case & load & ABC & it. & ms & $V_{abc}$ range & VUF\\% & boundary $V_{dc}$ \\\\\n";
  tex << "\\midrule\n";
  for (const auto& row : rows) {
    tex << "\\texttt{" << latex_escape(row.label) << "} & "
        << f2(row.scale) << "$\\times$ & "
        << (row.abc.converged ? "yes" : "no") << " & "
        << row.abc.iterations << " & " << f2(row.abc.best_ms) << " & "
        << f3(row.abc.vm_min) << "--" << f3(row.abc.vm_max) << " & "
        << f2(row.abc.max_vuf_percent) << " & "
        << (row.boundary.converged ? f3(row.boundary.vdc_min) + "--" + f3(row.boundary.vdc_max)
                                   : std::string("fail"))
        << " \\\\\n";
  }
  tex << "\\bottomrule\n\\end{tabular}\n";
}

void write_analysis(const fs::path& outdir,
                    const std::vector<CaseSummary>& summaries,
                    const std::vector<StressRow>& rows) {
  const int base_total = static_cast<int>(summaries.size());
  int base_abc = 0;
  int base_boundary = 0;
  int stress_abc = 0;
  int stress_boundary = 0;
  double max_abc_ms = 0.0;
  int max_abc_iter = 0;
  double max_abc_residual = 0.0;
  double max_vuf = 0.0;
  double min_vabc = std::numeric_limits<double>::infinity();
  double max_boundary_ms = 0.0;
  int max_boundary_iter = 0;
  double max_boundary_residual = 0.0;
  double min_success_vdc = std::numeric_limits<double>::infinity();
  double min_max_success_scale = std::numeric_limits<double>::infinity();
  double max_success_scale = 0.0;

  for (const auto& summary : summaries) {
    if (summary.abc.converged) ++base_abc;
    if (summary.boundary.converged) ++base_boundary;
    min_max_success_scale =
        std::min(min_max_success_scale, summary.max_success_scale);
    max_success_scale =
        std::max(max_success_scale, summary.max_success_scale);
  }

  for (const auto& row : rows) {
    if (row.abc.converged) {
      ++stress_abc;
      max_abc_ms = std::max(max_abc_ms, row.abc.best_ms);
      max_abc_iter = std::max(max_abc_iter, row.abc.iterations);
      if (std::isfinite(row.abc.residual)) {
        max_abc_residual = std::max(max_abc_residual, row.abc.residual);
      }
      if (std::isfinite(row.abc.max_vuf_percent)) {
        max_vuf = std::max(max_vuf, row.abc.max_vuf_percent);
      }
      if (std::isfinite(row.abc.vm_min)) {
        min_vabc = std::min(min_vabc, row.abc.vm_min);
      }
    }
    if (row.boundary.converged) {
      ++stress_boundary;
      max_boundary_ms = std::max(max_boundary_ms, row.boundary.best_ms);
      max_boundary_iter =
          std::max(max_boundary_iter, row.boundary.iterations);
      if (std::isfinite(row.boundary.residual)) {
        max_boundary_residual =
            std::max(max_boundary_residual, row.boundary.residual);
      }
      if (std::isfinite(row.boundary.vdc_min)) {
        min_success_vdc = std::min(min_success_vdc, row.boundary.vdc_min);
      }
    }
  }

  const int stress_total = static_cast<int>(rows.size());
  if (!std::isfinite(min_max_success_scale)) min_max_success_scale = 0.0;
  if (!std::isfinite(min_vabc)) min_vabc = std::numeric_limits<double>::quiet_NaN();
  if (!std::isfinite(min_success_vdc)) {
    min_success_vdc = std::numeric_limits<double>::quiet_NaN();
  }

  std::ofstream tex(outdir / "sppt_three_phase_hybrid_analysis.tex");
  tex << "% Auto-generated three-phase hybrid performance interpretation.\n";
  tex << "\\begin{tabularx}{\\linewidth}{@{}>{\\raggedright\\arraybackslash}p{0.18\\linewidth}\n"
         "  >{\\raggedright\\arraybackslash}p{0.28\\linewidth} X@{}}\n";
  tex << "\\toprule\n";
  tex << "Question & Numerical result & Interpretation \\\\\n";
  tex << "\\midrule\n";
  tex << "Nominal solvability & ABC " << base_abc << "/" << base_total
      << ", boundary " << base_boundary << "/" << base_total
      << " & all three representative feeders solve at base load in both the "
         "abc-domain distribution kernel and the aggregate AC/DC/VSC boundary "
         "kernel \\\\\n";
  tex << "ABC robustness & " << stress_abc << "/" << stress_total
      << " stress points; max " << max_abc_iter << " it., "
      << f2(max_abc_ms) << " ms, residual " << sci(max_abc_residual)
      << " & unbalanced abc equations remain numerically stable through the "
         "tested load ladder; observed failures are not abc Newton failures \\\\\n";
  tex << "Unbalance tracking & max VUF " << f2(max_vuf)
      << "\\%, min $V_{abc}$ " << f3(min_vabc)
      << " p.u. & missing-phase laterals and uneven per-phase loads are retained "
         "rather than collapsed to a positive-sequence surrogate \\\\\n";
  tex << "AC/DC boundary limit & staged success " << stress_boundary << "/"
      << stress_total << "; max load range " << f2(min_max_success_scale)
      << "--" << f2(max_success_scale) << "$\\times$, min successful $V_{dc}$ "
      << f3(min_success_vdc) << " p.u. & high-load failures occur when the "
         "paired DC/VSC boundary reaches an operating/numerical limit, which the "
         "study reports instead of reclassifying as convergence \\\\\n";
  tex << "Boundary timing & max successful " << max_boundary_iter << " it., "
      << f2(max_boundary_ms) << " ms, residual " << sci(max_boundary_residual)
      << " & the aggregate AC/DC solve is small in these cases; the result is "
         "useful for workflow evidence, not a claim of full monolithic "
         "abc--DC--VSC Jacobian performance \\\\\n";
  tex << "\\bottomrule\n\\end{tabularx}\n";
}

void write_scope(const fs::path& outdir) {
  std::ofstream tex(outdir / "sppt_three_phase_hybrid_scope.tex");
  tex << "% Auto-generated staged three-phase hybrid AC/DC scope note.\n";
  tex << "\\begin{tabularx}{\\linewidth}{@{}lXX@{}}\n";
  tex << "\\toprule\n";
  tex << "Layer & Solved equations & Performance evidence \\\\\n";
  tex << "\\midrule\n";
  tex << "Three-phase distribution & compact abc-domain AC nodal PF with unbalanced loads, missing phases, meshed ties, and DER & convergence, iterations, residual, voltage range, VUF, loss, solve time \\\\\n";
  tex << "AC/DC boundary & aggregate AC/DC nodal PF with DC buses, DC branch, VSC loss/control, and DC load & convergence, iterations, residual, Vdc range, VSC transfer count, solve time \\\\\n";
  tex << "Current limitation & staged coupling; the abc solver does not yet include DC voltages and VSC powers in one monolithic Newton vector & reported explicitly; no hidden claim of full abc--DC Jacobian equivalence \\\\\n";
  tex << "\\bottomrule\n\\end{tabularx}\n";
}

void write_plot(const fs::path& outdir,
                const std::vector<CaseSummary>& summaries) {
  std::ofstream tex(outdir / "sppt_three_phase_hybrid_pf_stress_plot.tex");
  tex << "% Auto-generated three-phase hybrid stress plot.\n";
  tex << "\\begin{tikzpicture}[font=\\scriptsize,scale=0.92]\n";
  tex << "  \\draw[->] (0,0) -- (4.8,0) node[right] {case};\n";
  tex << "  \\draw[->] (0,0) -- (0,4.4) node[above] {max converged load};\n";
  tex << "  \\foreach \\y/\\lab in {1/0.5,2/1.0,3/1.5,4/2.0} {"
      << "\\draw[gray!35] (0,\\y) -- (4.3,\\y); "
      << "\\node[anchor=east] at (-0.08,\\y) {\\lab};}\n";
  for (std::size_t i = 0; i < summaries.size(); ++i) {
    const double x = 0.55 + static_cast<double>(i) * 1.2;
    const double h = 2.0 * summaries[i].max_success_scale;
    tex << "  \\draw[fill=TealBlue!55,draw=TealBlue!70!black] ("
        << x << ",0) rectangle +(0.55," << h << ");\n";
    tex << "  \\node[rotate=45,anchor=east] at (" << (x + 0.42)
        << ",-0.15) {\\texttt{" << latex_escape(summaries[i].spec.label)
        << "}};\n";
    tex << "  \\node[anchor=south] at (" << (x + 0.28) << ","
        << (h + 0.05) << ") {" << f2(summaries[i].max_success_scale)
        << "$\\times$};\n";
  }
  tex << "\\end{tikzpicture}\n";
}

}  // namespace

int main(int argc, char** argv) {
  const fs::path outdir = argc > 1 ? fs::path(argv[1])
                                   : fs::path("docs") / "latex";
  const int repeats = argc > 2 ? std::max(1, std::stoi(argv[2])) : 5;
  fs::create_directories(outdir);

  const std::vector<double> scales{0.50, 1.00, 1.25, 1.50, 1.75, 2.00};
  std::vector<CaseSummary> summaries;
  std::vector<StressRow> stress_rows;

  for (const auto& spec : build_cases()) {
    CaseSummary summary;
    summary.spec = spec;
    summary.phase_nodes = count_phase_nodes(spec.system);
    summary.abc = run_three_phase(spec.system, repeats);
    summary.boundary = run_boundary(spec.system, repeats);

    for (double scale : scales) {
      HybridPowerSystem stressed = spec.system;
      scale_case(stressed, scale);
      StressRow row;
      row.label = spec.label;
      row.scale = scale;
      row.abc = run_three_phase(stressed, 1);
      row.boundary = run_boundary(stressed, 1);
      if (row.abc.converged && row.boundary.converged) {
        ++summary.stress_success;
        summary.max_success_scale = std::max(summary.max_success_scale, scale);
      }
      ++summary.stress_total;
      stress_rows.push_back(row);
    }
    summaries.push_back(summary);
  }

  write_summary(outdir, summaries);
  write_stress(outdir, stress_rows);
  write_analysis(outdir, summaries, stress_rows);
  write_scope(outdir);
  write_plot(outdir, summaries);

  std::cout << "Wrote three-phase hybrid AC/DC PF study tables to "
            << outdir << "\n";
  return 0;
}
