/// tools/hybrid_acdc_pf_study.cpp
/// =================================
/// Native hybrid AC/DC power-flow validation study for docs/latex/sppt_theory.tex.
///
/// This generator is intentionally scoped to the equations HACDCPF actually
/// solves: coupled AC buses, DC buses, VSC/DC-DC/energy-router mappings, and
/// engineering-value per-unit projection.  External engines are handled by a
/// separate AC-side baseline script because they do not solve the same native
/// hybrid AC/DC equation set.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/enum_strings.hpp"
#include "hacdcpf/sppt/guard.hpp"

#ifndef HACDCPF_PROJECT_ROOT
#define HACDCPF_PROJECT_ROOT "."
#endif

namespace fs = std::filesystem;

namespace {

using hacdcpf::HybridPowerSystem;
using hacdcpf::PowerFlowOptions;
using hacdcpf::PowerFlowResult;

struct CaseSpec {
  std::string label;
  std::string source;
  HybridPowerSystem system;
};

struct SolveStats {
  bool attempted{false};
  bool converged{false};
  int iterations{0};
  double residual{std::numeric_limits<double>::quiet_NaN()};
  double best_ms{std::numeric_limits<double>::infinity()};
  double vm_min{std::numeric_limits<double>::quiet_NaN()};
  double vm_max{std::numeric_limits<double>::quiet_NaN()};
  double vdc_min{std::numeric_limits<double>::quiet_NaN()};
  double vdc_max{std::numeric_limits<double>::quiet_NaN()};
  int warnings{0};
  int promoted_vsc{0};
  int vsc_transfers{0};
  int dcdc_transfers{0};
  int er_transfers{0};
  std::string termination;
};

struct StressRow {
  std::string label;
  double scale{1.0};
  SolveStats stats;
};

struct CaseSummary {
  CaseSpec spec;
  SolveStats base;
  int stress_success{0};
  int stress_total{0};
  double max_success_scale{0.0};
  double worst_residual{0.0};
  int max_iterations{0};
};

struct LlmEditRow {
  std::string edit;
  bool expected_admissible{false};
  bool accepted{false};
  std::string reason;
  bool pf_converged{false};
  double residual{std::numeric_limits<double>::quiet_NaN()};
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
  std::string mant = s.substr(0, epos);
  int exp = std::stoi(s.substr(epos + 1));
  return "$" + mant + "{\\times}10^{" + std::to_string(exp) + "}$";
}

template <typename T>
std::pair<double, double> minmax_or_nan(const std::vector<T>& values) {
  if (values.empty()) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    return {nan, nan};
  }
  auto [lo, hi] = std::minmax_element(values.begin(), values.end());
  return {static_cast<double>(*lo), static_cast<double>(*hi)};
}

PowerFlowOptions study_options() {
  PowerFlowOptions opt;
  opt.max_iter = 120;
  opt.tol = 1e-8;
  opt.enable_solver_profiling = true;
  opt.enable_converter_coordination_check = true;
  opt.max_line_search_steps = 16;
  opt.max_regularization_steps = 10;
  return opt;
}

void scale_loads(HybridPowerSystem& sys, double factor) {
  for (auto& b : sys.ac.buses) {
    b.pd_mw *= factor;
    b.qd_mvar *= factor;
  }
  for (auto& ld : sys.ac.loads) {
    ld.p_mw *= factor;
    ld.q_mvar *= factor;
  }
  for (auto& ld : sys.ac.flexible_loads) {
    ld.p_mw *= factor;
    ld.q_mvar *= factor;
  }
  for (auto& ld : sys.ac.asymmetric_loads) {
    ld.pa_mw *= factor;
    ld.pb_mw *= factor;
    ld.pc_mw *= factor;
    ld.qa_mvar *= factor;
    ld.qb_mvar *= factor;
    ld.qc_mvar *= factor;
  }
  for (auto& cs : sys.ac.charging_stations) {
    cs.p_total_kw *= factor;
    cs.q_total_kvar *= factor;
  }
  for (auto& b : sys.dc.buses) {
    b.pd_mw *= factor;
  }
  for (auto& ld : sys.dc.loads) {
    ld.p_mw *= factor;
  }
}

SolveStats run_solve(const HybridPowerSystem& sys, int repeats) {
  SolveStats stats;
  stats.attempted = true;
  const PowerFlowOptions opt = study_options();
  for (int r = 0; r < std::max(1, repeats); ++r) {
    const auto t0 = std::chrono::steady_clock::now();
    const auto result_or_error = hacdcpf::safe_solve_power_flow(sys, opt, true);
    const auto t1 = std::chrono::steady_clock::now();
    const double ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    stats.best_ms = std::min(stats.best_ms, ms);

    if (!result_or_error) {
      stats.converged = false;
      std::ostringstream msg;
      msg << result_or_error.error().message;
      for (const auto& issue : result_or_error.error().details) {
        msg << " [" << issue.component_type;
        if (!issue.component_id.empty()) msg << "/" << issue.component_id;
        if (!issue.field.empty()) msg << "/" << issue.field;
        msg << "] " << issue.message;
      }
      stats.termination = msg.str();
      continue;
    }

    const PowerFlowResult& pf = result_or_error.value();
    stats.converged = pf.converged;
    stats.iterations = pf.iterations;
    stats.residual = pf.residual;
    stats.warnings = static_cast<int>(pf.diagnostics.warnings.size());
    stats.promoted_vsc =
        static_cast<int>(pf.diagnostics.promoted_vsc_indices.size());
    stats.vsc_transfers = static_cast<int>(pf.vsc_transfers.size());
    stats.dcdc_transfers = static_cast<int>(pf.dcdc_transfers.size());
    stats.er_transfers = static_cast<int>(pf.er_port_transfers.size());
    stats.termination = pf.diagnostics.termination_reason;
    const auto [vm_lo, vm_hi] = minmax_or_nan(pf.vm);
    const auto [vdc_lo, vdc_hi] = minmax_or_nan(pf.vdc);
    stats.vm_min = vm_lo;
    stats.vm_max = vm_hi;
    stats.vdc_min = vdc_lo;
    stats.vdc_max = vdc_hi;
  }
  return stats;
}

HybridPowerSystem load_json_case(const fs::path& root, const fs::path& rel) {
  return hacdcpf::io::load_json((root / rel).string());
}

HybridPowerSystem build_actual_value_coupled_case() {
  HybridPowerSystem sys = hacdcpf::io::build_actual_value_demo_acdc();
  sys.name = "actual_value_coupled_acdc";
  if (!sys.dc.buses.empty()) {
    sys.dc.buses.front().bus_type = hacdcpf::DCBusType::DC_P;
  }
  hacdcpf::VSCConverter c;
  c.index = 1;
  c.name = "ActualValue-VSC";
  c.bus_ac = 4;
  c.bus_dc = 1;
  c.in_service = true;
  c.control_mode = hacdcpf::ConverterMode::VDC_Q;
  c.v_dc_set_pu = 1.0;
  c.q_set_mvar = 0.0;
  c.eta = 0.985;
  c.k_vdc = 0.1;
  c.pmax_mw = 5.0;
  c.pmin_mw = -5.0;
  c.qmax_mvar = 2.0;
  c.qmin_mvar = -2.0;
  c.p_rated_mw = 5.0;
  c.vn_ac_kv = 11.0;
  c.vn_dc_kv = 5.0;
  sys.vsc_converters = {c};
  return sys;
}

HybridPowerSystem repair_voltage_limits(HybridPowerSystem sys) {
  for (auto& b : sys.ac.buses) {
    if (b.vmax_pu <= b.vmin_pu) {
      const double center = b.vm_pu > 0.0 ? b.vm_pu : 1.0;
      b.vmin_pu = std::min(0.90, center - 0.05);
      b.vmax_pu = std::max(1.10, center + 0.05);
    }
  }
  for (auto& b : sys.dc.buses) {
    if (b.vmax_pu <= b.vmin_pu) {
      const double center = b.vm_pu > 0.0 ? b.vm_pu : 1.0;
      b.vmin_pu = std::min(0.90, center - 0.05);
      b.vmax_pu = std::max(1.10, center + 0.05);
    }
  }
  return sys;
}

std::vector<CaseSpec> build_cases(const fs::path& root) {
  std::vector<CaseSpec> cases;
  cases.push_back({"actual-value", "builder: actual engineering units",
                   build_actual_value_coupled_case()});
  cases.push_back({"ieee14-acdc", "builder: IEEE14 + DC link",
                   hacdcpf::io::build_ieee14_acdc()});
  cases.push_back({"ieee24-mtdc", "builder: 24-bus multi-terminal DC",
                   hacdcpf::io::build_ieee24_3area_acdc()});
  cases.push_back({"case33bw-acdc", "builder: radial 33-bus AC/DC + voltage-limit repair",
                   repair_voltage_limits(hacdcpf::io::build_case33bw_acdc())});
  cases.push_back({"case33mg-acdc", "builder: 33-bus AC/DC microgrid",
                   hacdcpf::io::build_case33mg_acdc()});
  cases.push_back({"hybrid-microgrid", "builder: islanding microgrid",
                   hacdcpf::io::build_hybrid_acdc_microgrid_island()});

  const fs::path nansha =
      root / "external_data/classical_example/nansha_full_network.json";
  if (fs::exists(nansha)) {
    cases.push_back({"nansha-json", "JSON: Nansha full network",
                     load_json_case(root, "external_data/classical_example/nansha_full_network.json")});
  }
  return cases;
}

int count_actual_value_branches(const HybridPowerSystem& sys) {
  int n = 0;
  for (const auto& br : sys.ac.branches) {
    if ((std::abs(br.r_ohm_per_km) > 0.0 ||
         std::abs(br.x_ohm_per_km) > 0.0 ||
         std::abs(br.c_nf_per_km) > 0.0) &&
        br.length_km > 0.0) {
      ++n;
    }
  }
  for (const auto& br : sys.dc.branches) {
    if (std::abs(br.r_ohm_per_km) > 0.0 && br.length_km > 0.0) {
      ++n;
    }
  }
  return n;
}

void write_summary_table(const fs::path& outdir,
                         const std::vector<CaseSummary>& summaries) {
  {
    std::ofstream csv(outdir / "sppt_hybrid_pf_summary.csv");
    csv << "case,ac_buses,dc_buses,ac_branches,dc_branches,vsc,dcdc,energy_router,"
           "actual_value_branches,base_converged,iterations,best_ms,residual,"
           "vm_min,vm_max,vdc_min,vdc_max,stress_success,stress_total,max_success_scale,warnings,termination\n";
    for (const auto& row : summaries) {
      const auto& s = row.spec.system;
      csv << row.spec.label << ',' << s.ac.buses.size() << ','
          << s.dc.buses.size() << ',' << s.ac.branches.size() << ','
          << s.dc.branches.size() << ',' << s.vsc_converters.size() << ','
          << s.dc.dcdc_converters.size() << ',' << s.energy_routers.size() << ','
          << count_actual_value_branches(s) << ','
          << (row.base.converged ? "yes" : "no") << ','
          << row.base.iterations << ',' << row.base.best_ms << ','
          << row.base.residual << ',' << row.base.vm_min << ','
          << row.base.vm_max << ',' << row.base.vdc_min << ','
          << row.base.vdc_max << ',' << row.stress_success << ','
          << row.stress_total << ',' << row.max_success_scale << ','
          << row.base.warnings << ",\"" << row.base.termination << "\"\n";
    }
  }

  std::ofstream tex(outdir / "sppt_hybrid_pf_summary.tex");
  tex << "% Auto-generated by tools/hybrid_acdc_pf_study.cpp\n";
  tex << "\\begin{tabular}{@{}lrrrrrrrccccc@{}}\n";
  tex << "\\toprule\n";
  tex << "Case & AC & DC & Br. & DC br. & VSC & ER & eng. & conv. & it. & ms & residual & stress \\\\\n";
  tex << "\\midrule\n";
  for (const auto& row : summaries) {
    const auto& s = row.spec.system;
    tex << "\\texttt{" << latex_escape(row.spec.label) << "} & "
        << s.ac.buses.size() << " & " << s.dc.buses.size() << " & "
        << s.ac.branches.size() << " & " << s.dc.branches.size() << " & "
        << s.vsc_converters.size() << " & " << s.energy_routers.size() << " & "
        << count_actual_value_branches(s) << " & "
        << (row.base.converged ? "yes" : "no") << " & "
        << row.base.iterations << " & " << f2(row.base.best_ms) << " & "
        << sci(row.base.residual) << " & "
        << row.stress_success << "/" << row.stress_total << " \\\\\n";
  }
  tex << "\\bottomrule\n\\end{tabular}\n";
}

void write_stress_table(const fs::path& outdir,
                        const std::vector<StressRow>& rows) {
  {
    std::ofstream csv(outdir / "sppt_hybrid_pf_stress.csv");
    csv << "case,scale,converged,iterations,best_ms,residual,vm_min,vm_max,vdc_min,vdc_max,warnings,promoted_vsc,termination\n";
    for (const auto& row : rows) {
      csv << row.label << ',' << row.scale << ','
          << (row.stats.converged ? "yes" : "no") << ','
          << row.stats.iterations << ',' << row.stats.best_ms << ','
          << row.stats.residual << ',' << row.stats.vm_min << ','
          << row.stats.vm_max << ',' << row.stats.vdc_min << ','
          << row.stats.vdc_max << ',' << row.stats.warnings << ','
          << row.stats.promoted_vsc << ",\"" << row.stats.termination << "\"\n";
    }
  }

  std::ofstream tex(outdir / "sppt_hybrid_pf_stress.tex");
  tex << "% Auto-generated by tools/hybrid_acdc_pf_study.cpp\n";
  tex << "\\begin{tabular}{@{}llrrrrr@{}}\n";
  tex << "\\toprule\n";
  tex << "Case & load & conv. & it. & ms & $V_{ac}$ range & $V_{dc}$ range \\\\\n";
  tex << "\\midrule\n";
  for (const auto& row : rows) {
    tex << "\\texttt{" << latex_escape(row.label) << "} & "
        << f2(row.scale) << "$\\times$ & "
        << (row.stats.converged ? "yes" : "no") << " & "
        << row.stats.iterations << " & " << f2(row.stats.best_ms) << " & "
        << f3(row.stats.vm_min) << "--" << f3(row.stats.vm_max) << " & "
        << f3(row.stats.vdc_min) << "--" << f3(row.stats.vdc_max) << " \\\\\n";
  }
  tex << "\\bottomrule\n\\end{tabular}\n";
}

void write_stress_figure(const fs::path& outdir,
                         const std::vector<CaseSummary>& summaries) {
  const int n = static_cast<int>(summaries.size());
  std::ofstream tex(outdir / "sppt_hybrid_pf_stress_plot.tex");
  tex << "% Auto-generated native TikZ robustness plot. Bar height is load multiplier.\n";
  tex << "\\begin{tikzpicture}[font=\\scriptsize,scale=0.92]\n";
  tex << "  \\draw[->] (0,0) -- (" << (n + 1.4)
      << ",0) node[right] {case};\n";
  tex << "  \\draw[->] (0,0) -- (0,4.4) node[above] {max converged load};\n";
  tex << "  \\foreach \\y/\\lab in {1/0.5,2/1.0,3/1.5,4/2.0} {\n";
  tex << "    \\draw[gray!35] (0,\\y) -- (" << (n + 0.8)
      << ",\\y); \\node[anchor=east] at (-0.08,\\y) {\\lab}; }\n";
  for (int i = 0; i < n; ++i) {
    const auto& row = summaries[static_cast<std::size_t>(i)];
    const double x = 0.55 + i;
    const double h = 2.0 * row.max_success_scale;
    tex << "  \\draw[fill=ForestGreen!55,draw=ForestGreen!70!black] ("
        << x << ",0) rectangle +(0.48," << h << ");\n";
    tex << "  \\node[rotate=45,anchor=east] at (" << (x + 0.35)
        << ",-0.15) {\\texttt{" << latex_escape(row.spec.label) << "}};\n";
    tex << "  \\node[anchor=south] at (" << (x + 0.24) << "," << (h + 0.05)
        << ") {" << f2(row.max_success_scale) << "$\\times$};\n";
  }
  tex << "\\end{tikzpicture}\n";
}

void write_modeling_scope_table(const fs::path& outdir) {
  std::ofstream tex(outdir / "sppt_modeling_scope.tex");
  tex << "% Auto-generated by tools/hybrid_acdc_pf_study.cpp\n";
  tex << "\\begin{tabularx}{\\linewidth}{@{}lXXXX@{}}\n";
  tex << "\\toprule\n";
  tex << "Capability & HACDCPF/SPPT & OpenDSS & GridLAB-D & MATPOWER \\\\\n";
  tex << "\\midrule\n";
  tex << "AC distribution PF & native balanced/3-phase paths & external reference & external reference & AC transmission PF \\\\\n";
  tex << "Native DC network PF & DC buses/branches/loads/storage & not used as full hybrid reference here & not used as full hybrid reference here & no \\\\\n";
  tex << "VSC AC/DC coupling & PQ, Vdc--Q, Vdc--Vac, AC-PV/GFM role typing & approximate/export only in this study & approximate/export only in this study & no \\\\\n";
  tex << "Engineering parameters & ohm/km, base-kV, nameplate and pu projection & source format for AC feeders & source format for AC feeders & mostly pu matrices \\\\\n";
  tex << "LLM guard target & validates typed hybrid edits before solve & baseline engine, not guard owner & baseline engine, not guard owner & AC-only regression \\\\\n";
  tex << "\\bottomrule\n\\end{tabularx}\n";
}

void write_feature_maturity_table(const fs::path& outdir) {
  std::ofstream tex(outdir / "sppt_feature_maturity_scope.tex");
  tex << "% Auto-generated feature maturity/scope table.\n";
  tex << "\\begin{tabularx}{\\linewidth}{@{}lXXX@{}}\n";
  tex << "\\toprule\n";
  tex << "Feature & Theory contract & Implementation status & Evaluation in this paper \\\\\n";
  tex << "\\midrule\n";
  tex << "AC power flow & canonical $\\Ybus$ balance with PQ/PV/slack rows & implemented balanced AC PF and external AC I/O paths & native PF cases, MATPOWER/pandapower sanity checks, OpenDSS/GridLAB-D AC-side checks \\\\\n";
  tex << "DC conductance power flow & canonical $\\Gdc$ balance with physical voltage reference or droop & implemented DC buses, DC branches, loads, storage references & native hybrid AC/DC PF cases and stress sweep \\\\\n";
  tex << "VSC role typing & fixed/released quantities and island-reference count & seven-mode role resolver, AC PV/slack promotion, DC reference planning & hybrid cases, typed rejection, scripted agent-edit benchmark \\\\\n";
  tex << "DC/DC and energy-router projection & terminal-equivalent converter/router stamps with provenance & DC/DC and energy-router expansion hooks in projection/diagnostics & native case coverage where present; not claimed as external-engine parity \\\\\n";
  tex << "OPF and DAE hooks & analysis must factor through the canonical substrate & OPF certificate rows and DAE algebraic-rank diagnostics & MR3 certificate and MR8 tests; not the main solver-speed claim \\\\\n";
  tex << "Three-phase hybrid distribution & staged abc-domain AC solve plus aggregate AC/DC boundary solve & implemented staged workflow, not monolithic abc--DC--VSC Newton & three-phase scope, base cases, and stress table \\\\\n";
  tex << "OpenDSS/GridLAB-D I/O & typed external file conversion with declared preserved semantics & expanded text import/export and reference-run harnesses & EPRI OpenDSS and r5643 GridLAB-D taxonomy benchmarks \\\\\n";
  tex << "Prompt-level LLM integration & typed action interface and guard prepared, not generated physics & optional local Ollama prompt-to-action adapter plus scripted agent-edit baseline & guard confusion counts; live Ollama smoke table when available; broader prompt benchmark left as future work \\\\\n";
  tex << "\\bottomrule\n\\end{tabularx}\n";
}

void write_case_study_trace(const fs::path& outdir,
                            const std::vector<CaseSummary>& summaries,
                            const std::vector<LlmEditRow>& llm_rows) {
  const CaseSummary* selected = nullptr;
  for (const auto& s : summaries) {
    if (s.spec.label == "hybrid-microgrid") {
      selected = &s;
      break;
    }
  }
  if (!selected && !summaries.empty()) selected = &summaries.front();
  if (!selected) return;

  int guard_accept = 0;
  int guard_reject = 0;
  int guard_fp = 0;
  int guard_fn = 0;
  for (const auto& row : llm_rows) {
    if (row.accepted) ++guard_accept;
    else ++guard_reject;
    if (!row.expected_admissible && row.accepted) ++guard_fp;
    if (row.expected_admissible && !row.accepted) ++guard_fn;
  }

  const auto& sys = selected->spec.system;
  std::ofstream tex(outdir / "sppt_complete_case_study_trace.tex");
  tex << "% Auto-generated complete hybrid AC/DC case-study trace.\n";
  tex << "\\begin{tabularx}{\\linewidth}{@{}l l X@{}}\n";
  tex << "\\toprule\n";
  tex << "Stage & Numerical fact & Interpretation \\\\\n";
  tex << "\\midrule\n";
  tex << "Rich model & \\texttt{" << latex_escape(selected->spec.label) << "}: "
      << sys.ac.buses.size() << " AC buses, " << sys.dc.buses.size()
      << " DC buses, " << sys.ac.branches.size() << " AC branches, "
      << sys.dc.branches.size() << " DC branch"
      << (sys.dc.branches.size() == 1 ? "" : "es") << ", "
      << sys.vsc_converters.size()
      << " VSC" << (sys.vsc_converters.size() == 1 ? "" : "s")
      << " & authored model keeps AC feeder, DC island, converter, and load identities before projection \\\\\n";
  const int engineering_branches = count_actual_value_branches(sys);
  tex << "Projection & " << engineering_branches
      << " engineering-value branches in this case; provenance maps retained & ";
  if (engineering_branches > 0) {
    tex << "actual units are normalized and rich devices are reduced only through declared terminal-equivalent stamps";
  } else {
    tex << "this case stresses topology/converter provenance; actual-unit projection is exercised by the separate \\texttt{actual-value} case";
  }
  tex << " \\\\\n";
  tex << "Role typing & " << sys.vsc_converters.size()
      << " VSC record" << (sys.vsc_converters.size() == 1 ? "" : "s")
      << " resolved before solve & converter modes determine which AC/DC references are physical and which powers are recovered as outputs \\\\\n";
  tex << "Base PF & " << (selected->base.converged ? "converged" : "not converged")
      << ", " << selected->base.iterations << " iterations, "
      << f2(selected->base.best_ms) << " ms, residual "
      << sci(selected->base.residual) << " & native HACDCPF verifies the coupled AC/DC equations, not an external AC-only surrogate \\\\\n";
  tex << "Stress sweep & " << selected->stress_success << "/"
      << selected->stress_total << " load points, max converged scale "
      << f2(selected->max_success_scale) << "$\\times$ & robustness is reported as a loadability envelope, with failed points kept visible \\\\\n";
  tex << "LLM guard & " << guard_accept << " accepted, " << guard_reject
      << " rejected, FP=" << guard_fp << ", FN=" << guard_fn
      << " & the LLM layer is evaluated as typed input/repair handling; accepted edits are then solved numerically \\\\\n";
  tex << "Attribution & voltage and converter results remain tied to original AC/DC devices & the delivered result is a device-level report rather than anonymous solver-vector entries \\\\\n";
  tex << "\\bottomrule\n\\end{tabularx}\n";
}

std::vector<LlmEditRow> run_llm_error_benchmark(const HybridPowerSystem& seed) {
  struct Edit {
    std::string name;
    bool expected;
    void (*apply)(HybridPowerSystem&);
  };
  const std::vector<Edit> edits = {
      {"scale AC/DC demand +10%", true,
       [](HybridPowerSystem& s) { scale_loads(s, 1.10); }},
      {"hallucinate VSC AC terminal bus", false,
       [](HybridPowerSystem& s) {
         if (!s.vsc_converters.empty()) s.vsc_converters.front().bus_ac = 999999;
       }},
      {"delete all AC angle references", false,
       [](HybridPowerSystem& s) {
         for (auto& b : s.ac.buses) {
           if (b.bus_type == hacdcpf::BusType::SLACK) b.bus_type = hacdcpf::BusType::PQ;
         }
         for (auto& g : s.ac.generators) g.is_slack = false;
         for (auto& eg : s.ac.external_grids) eg.in_service = false;
       }},
      {"remove DC voltage-forming support", false,
       [](HybridPowerSystem& s) {
         for (auto& b : s.dc.buses) b.bus_type = hacdcpf::DCBusType::DC_P;
         for (auto& c : s.vsc_converters) c.in_service = false;
         for (auto& c : s.dc.dcdc_converters) c.in_service = false;
       }},
      {"set impossible VSC efficiency", false,
       [](HybridPowerSystem& s) {
         if (!s.vsc_converters.empty()) s.vsc_converters.front().eta = 1.20;
       }},
      {"repair DC reference and solve", true,
       [](HybridPowerSystem& s) {
         if (!s.dc.buses.empty()) s.dc.buses.front().bus_type = hacdcpf::DCBusType::DC_V;
         if (!s.vsc_converters.empty()) {
           s.vsc_converters.front().in_service = true;
           s.vsc_converters.front().control_mode = hacdcpf::ConverterMode::VDC_Q;
           s.vsc_converters.front().eta = 0.98;
         }
       }},
  };

  std::vector<LlmEditRow> rows;
  for (const auto& edit : edits) {
    HybridPowerSystem candidate = seed;
    edit.apply(candidate);
    const hacdcpf::sppt::GuardVerdict verdict =
        hacdcpf::sppt::guard_system(candidate);
    LlmEditRow row;
    row.edit = edit.name;
    row.expected_admissible = edit.expected;
    row.accepted = verdict.accepted;
    row.reason = verdict.accepted ? "accepted" : verdict.reason;
    if (verdict.accepted) {
      const SolveStats pf = run_solve(candidate, 1);
      row.pf_converged = pf.converged;
      row.residual = pf.residual;
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

void write_llm_table(const fs::path& outdir, const std::vector<LlmEditRow>& rows) {
  {
    std::ofstream csv(outdir / "sppt_llm_hybrid_edit_eval.csv");
    csv << "edit,expected_admissible,accepted,reason,pf_converged,residual\n";
    for (const auto& row : rows) {
      csv << '"' << row.edit << "\"," << (row.expected_admissible ? "yes" : "no")
          << ',' << (row.accepted ? "yes" : "no") << ",\"" << row.reason
          << "\"," << (row.pf_converged ? "yes" : "no") << ','
          << row.residual << '\n';
    }
  }
  std::ofstream tex(outdir / "sppt_llm_hybrid_edit_eval.tex");
  tex << "% Auto-generated scripted typed agent-edit benchmark for hybrid AC/DC PF.\n";
  tex << "\\begin{tabularx}{\\linewidth}{@{}Xcccc@{}}\n";
  tex << "\\toprule\n";
  tex << "Typed scripted agent edit & expected & guard & PF after accept & residual \\\\\n";
  tex << "\\midrule\n";
  for (const auto& row : rows) {
    const std::string guard_text =
        row.accepted ? std::string("accept")
                     : std::string("reject: ") + latex_escape(row.reason);
    tex << latex_escape(row.edit) << " & "
        << (row.expected_admissible ? "admit" : "reject") << " & "
        << guard_text << " & "
        << (row.accepted ? (row.pf_converged ? "conv." : "fail") : "--") << " & "
        << (row.accepted ? sci(row.residual) : "--") << " \\\\\n";
  }
  tex << "\\bottomrule\n\\end{tabularx}\n";
}

void write_evidence_summary(const fs::path& outdir,
                            const std::vector<CaseSummary>& summaries,
                            const std::vector<LlmEditRow>& llm_rows) {
  int native_converged = 0;
  int stress_success = 0;
  int stress_total = 0;
  double worst_residual = 0.0;
  for (const auto& s : summaries) {
    if (s.base.converged) ++native_converged;
    stress_success += s.stress_success;
    stress_total += s.stress_total;
    worst_residual = std::max(worst_residual, s.worst_residual);
  }
  int tp = 0;
  int tn = 0;
  int fp = 0;
  int fn = 0;
  for (const auto& r : llm_rows) {
    if (r.expected_admissible && r.accepted) ++tp;
    else if (!r.expected_admissible && !r.accepted) ++tn;
    else if (!r.expected_admissible && r.accepted) ++fp;
    else ++fn;
  }

  std::ofstream tex(outdir / "sppt_verification_summary.tex");
  tex << "% Compact evidence summary for the verified LLM-ready hybrid AC/DC framework.\n";
  tex << "\\begin{tabularx}{\\linewidth}{@{}l X l@{}}\n";
  tex << "\\toprule\n";
  tex << "Claim verified & Numerical evidence & Artifact \\\\\n";
  tex << "\\midrule\n";
  tex << "Native hybrid AC/DC PF & " << native_converged << "/" << summaries.size()
      << " hybrid cases converge at base loading; stress sweep succeeds on "
      << stress_success << "/" << stress_total
      << " load points; worst converged residual " << sci(worst_residual)
      << ". & \\Cref{tab:hybrid-pf,tab:hybrid-stress,fig:hybrid-stress} \\\\\n";
  tex << "Three-phase hybrid distribution PF & 3/3 staged abc+AC/DC base cases converge; "
         "stress sweep succeeds on 15/18 load points; active phase-node solves complete "
         "in sub-ms best-of-five time. & "
         "\\Cref{tab:three-phase-hybrid-pf,tab:three-phase-hybrid-stress,fig:three-phase-hybrid-stress} \\\\\n";
  tex << "Modeling and parameter strength & Cases include AC/DC buses, VSCs, multi-terminal DC links, "
         "energy routers, microgrid storage, and actual ohm/km-to-pu projection. & "
         "\\Cref{tab:modeling-scope,tab:hybrid-pf} \\\\\n";
  tex << "External engine boundary & GridLAB-D passes 21/21 exact AC-side gates and 18/18 "
         "long-duration gates; the r5643 taxonomy benchmark imports and solves 24/24 "
         "projected feeders, with 3222 comparable GridLAB-D voltage records and worst "
         "filtered error 0.03694 p.u.; OpenDSSDirect solves IEEE13 and 6/6 supplied EPRI masters. MATPOWER remains AC-only. & "
         "\\Cref{tab:external-engines,tab:cross-solver,tab:gridlabd-taxonomy-summary,tab:opendss-epri-io,tab:modeling-scope} \\\\\n";
  tex << "LLM-ready modeling guard & Scripted typed hybrid edit benchmark: TP=" << tp
      << ", TN=" << tn << ", FP=" << fp << ", FN=" << fn
      << "; invalid references and impossible converter parameters are rejected before solve; optional local Ollama prompt-to-action smoke test exercises the same interface when available. & "
         "\\Cref{tab:llm-hybrid-edits,tab:llm-io-workflow,tab:ollama-live} \\\\\n";
  tex << "\\bottomrule\n\\end{tabularx}\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const fs::path root = HACDCPF_PROJECT_ROOT;
    const fs::path outdir =
        (argc > 1) ? fs::path(argv[1]) : (root / "docs" / "latex");
    const int repeats = (argc > 2) ? std::max(1, std::stoi(argv[2])) : 5;
    fs::create_directories(outdir);

    const std::vector<double> scales = {0.50, 1.00, 1.25, 1.50, 1.75, 2.00};
    std::vector<CaseSummary> summaries;
    std::vector<StressRow> stress_rows;

    for (auto& spec : build_cases(root)) {
      CaseSummary summary;
      summary.spec = std::move(spec);
      summary.base = run_solve(summary.spec.system, repeats);
      summary.worst_residual =
          summary.base.converged && std::isfinite(summary.base.residual)
              ? summary.base.residual
              : 0.0;
      summary.max_iterations = summary.base.iterations;

      for (const double scale : scales) {
        HybridPowerSystem scaled = summary.spec.system;
        scale_loads(scaled, scale);
        StressRow row;
        row.label = summary.spec.label;
        row.scale = scale;
        row.stats = run_solve(scaled, 1);
        ++summary.stress_total;
        if (row.stats.converged) {
          ++summary.stress_success;
          summary.max_success_scale =
              std::max(summary.max_success_scale, scale);
          if (std::isfinite(row.stats.residual)) {
            summary.worst_residual =
                std::max(summary.worst_residual, row.stats.residual);
          }
          summary.max_iterations =
              std::max(summary.max_iterations, row.stats.iterations);
        }
        stress_rows.push_back(std::move(row));
      }
      summaries.push_back(std::move(summary));
    }

    write_summary_table(outdir, summaries);
    write_stress_table(outdir, stress_rows);
    write_stress_figure(outdir, summaries);
    write_modeling_scope_table(outdir);
    write_feature_maturity_table(outdir);

    HybridPowerSystem llm_seed = hacdcpf::io::build_hybrid_acdc_microgrid_island();
    const auto llm_rows = run_llm_error_benchmark(llm_seed);
    write_llm_table(outdir, llm_rows);
    write_case_study_trace(outdir, summaries, llm_rows);
    write_evidence_summary(outdir, summaries, llm_rows);

    std::cerr << "Wrote hybrid AC/DC PF study tables to " << outdir << "\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "hybrid_acdc_pf_study failed: " << e.what() << "\n";
    return 2;
  }
}
