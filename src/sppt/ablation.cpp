/// sppt/ablation.cpp
/// =================
/// Pillar-4 ablation study implementation (see ablation.hpp).

#include "hacdcpf/sppt/ablation.hpp"

#include <cmath>
#include <complex>
#include <exception>
#include <iomanip>
#include <sstream>
#include <string>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/model/enums/bus_types.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"
#include "hacdcpf/sppt/guard.hpp"

namespace hacdcpf::sppt {
namespace {

std::string sci(double v) {
  std::ostringstream os;
  os << std::scientific << std::setprecision(2) << v;
  return os.str();
}

// Demote every AC angle reference — the canonical inadmissible edit.
HybridPowerSystem break_reference(HybridPowerSystem sys) {
  for (auto& b : sys.ac.buses)
    if (b.bus_type == BusType::SLACK) b.bus_type = BusType::PQ;
  for (auto& g : sys.ac.generators) g.is_slack = false;
  for (auto& eg : sys.ac.external_grids) eg.in_service = false;
  return sys;
}

double max_abs_diagonal(const Eigen::SparseMatrix<std::complex<double>>& y) {
  double m = 0.0;
  for (int k = 0; k < y.outerSize(); ++k)
    m = std::max(m, std::abs(y.coeff(k, k)));
  return m;
}

// ── Ablation A: provenance removed ─────────────────────────────────────────
void ablate_provenance(const HybridPowerSystem& sys, AblationRow& row) {
  int merged_lost = 0;
  int dead = 0;
  try {
    powerflow::SolverData sd = powerflow::make_solver_data(sys);
    if (sd.bus_merge_map) {
      for (const auto& group : sd.bus_merge_map->groups)
        if (group.size() > 1) merged_lost += static_cast<int>(group.size()) - 1;
      dead = static_cast<int>(sd.bus_merge_map->dead_bus_indices.size());
    }
  } catch (const std::exception&) {
    // fall through with zero counts
  }
  // Rich devices that become canonical branches and thus need BranchExpandMap
  // to be attributed back to their origin.
  const int expanded = static_cast<int>(sys.ac.switches.size()) +
                       static_cast<int>(sys.ac.circuit_breakers.size()) +
                       static_cast<int>(sys.ac.transformers_2w.size()) +
                       3 * static_cast<int>(sys.ac.transformers_3w.size());

  row.prov_unattributable_ablated = merged_lost + dead + expanded;
  row.prov_unattributable_guarded = 0;  // provenance recovers all of them
  const int denom = std::max(1, n_ac_buses(sys) + n_ac_branches(sys));
  row.prov_fraction =
      static_cast<double>(row.prov_unattributable_ablated) / denom;
}

// ── Ablation B: role typing / well-posedness gate removed ──────────────────
void ablate_role_typing(const HybridPowerSystem& sys, AblationRow& row) {
  const HybridPowerSystem broken = break_reference(sys);
  row.role_guard_rejects = !guard_system(broken).accepted;
  try {
    const PowerFlowResult pf = solve_power_flow(broken);
    row.role_raw_silent = pf.converged;  // "converged" despite no reference
  } catch (const std::exception&) {
    row.role_raw_silent = false;  // raw solver failed loudly — not silent
  }
}

// ── Ablation C: merge fill-guard removed ───────────────────────────────────
void ablate_merge_guard(const HybridPowerSystem& sys, AblationRow& row) {
  const bool has_switch =
      !sys.ac.switches.empty() || !sys.ac.circuit_breakers.empty();
  row.merge_applicable = has_switch;
  if (!has_switch) return;

  try {
    // Guarded: the switch triggers the zero-impedance merge.
    powerflow::SolverData merged = powerflow::make_solver_data(sys);
    row.merge_max_diag_merged = max_abs_diagonal(merged.ybus);

    // Ablated: replace switches/CBs with near-zero-impedance branches so the
    // merge does NOT fire, leaving the ill-conditioning in place.
    HybridPowerSystem unmerged = sys;
    int next_idx = 0;
    for (const auto& b : unmerged.ac.branches) next_idx = std::max(next_idx, b.index);
    auto add_tiny_branch = [&](int from, int to, bool closed) {
      ACBranch br;
      br.index = ++next_idx;
      br.from_bus = from;
      br.to_bus = to;
      br.r_pu = 1e-6;
      br.x_pu = 1e-6;
      br.b_pu = 0.0;
      br.tap = 1.0;
      br.in_service = closed;
      unmerged.ac.branches.push_back(br);
    };
    for (const auto& sw : unmerged.ac.switches)
      add_tiny_branch(sw.bus_from, sw.bus_to, sw.closed);
    for (const auto& cb : unmerged.ac.circuit_breakers)
      add_tiny_branch(cb.bus_from, cb.bus_to, cb.closed);
    unmerged.ac.switches.clear();
    unmerged.ac.circuit_breakers.clear();

    powerflow::SolverData um = powerflow::make_solver_data(unmerged);
    row.merge_max_diag_unmerged = max_abs_diagonal(um.ybus);
    row.merge_blowup =
        row.merge_max_diag_merged > 0.0
            ? row.merge_max_diag_unmerged / row.merge_max_diag_merged
            : 0.0;
  } catch (const std::exception&) {
    row.merge_applicable = false;
  }
}

}  // namespace

AblationRow run_ablation_case(const HybridPowerSystem& sys, std::string case_name) {
  AblationRow row;
  row.case_name = std::move(case_name);
  ablate_provenance(sys, row);
  ablate_role_typing(sys, row);
  ablate_merge_guard(sys, row);
  return row;
}

AblationStudy run_ablation_study(
    const std::vector<std::pair<std::string, std::string>>& cases) {
  AblationStudy study;
  for (const auto& [name, path] : cases) {
    try {
      HybridPowerSystem sys;
      if (path.size() >= 2 && path.substr(path.size() - 2) == ".m")
        sys = io::parse_matpower(path);
      else
        sys = io::load_json(path);
      study.rows.push_back(run_ablation_case(sys, name));
    } catch (const std::exception& e) {
      AblationRow row;
      row.case_name = name + " (load failed: " + e.what() + ")";
      study.rows.push_back(row);
    }
  }
  return study;
}

int AblationStudy::total_unattributable_without_provenance() const {
  int n = 0;
  for (const auto& r : rows) n += r.prov_unattributable_ablated;
  return n;
}

int AblationStudy::silent_solves_without_role_typing() const {
  int n = 0;
  for (const auto& r : rows)
    if (r.role_raw_silent) ++n;
  return n;
}

double AblationStudy::max_merge_blowup() const {
  double m = 0.0;
  for (const auto& r : rows)
    if (r.merge_applicable) m = std::max(m, r.merge_blowup);
  return m;
}

std::string AblationStudy::to_csv() const {
  std::ostringstream os;
  os << "case,prov_unattributable_ablated,prov_fraction,role_guard_rejects,"
        "role_raw_silent,merge_applicable,merge_max_diag_merged,"
        "merge_max_diag_unmerged,merge_blowup\n";
  for (const auto& r : rows) {
    os << r.case_name << ',' << r.prov_unattributable_ablated << ','
       << sci(r.prov_fraction) << ',' << (r.role_guard_rejects ? 1 : 0) << ','
       << (r.role_raw_silent ? 1 : 0) << ',' << (r.merge_applicable ? 1 : 0) << ','
       << sci(r.merge_max_diag_merged) << ',' << sci(r.merge_max_diag_unmerged)
       << ',' << sci(r.merge_blowup) << '\n';
  }
  return os.str();
}

std::string AblationStudy::to_latex() const {
  std::ostringstream os;
  os << "% Auto-generated by hacdcpf::sppt::AblationStudy::to_latex()\n"
     << "\\begin{tabular}{@{}lrccr@{}}\n\\toprule\n"
     << "Case & $\\Att$: unattrib. & $\\rho$: gate & $\\rho$: raw silent & "
        "merge: $Y_{ii}$ blowup \\\\\n\\midrule\n";
  for (const auto& r : rows) {
    std::string cn = r.case_name;
    std::string esc;
    for (char c : cn) {
      if (c == '_') esc.push_back('\\');
      esc.push_back(c);
    }
    os << "\\texttt{" << esc << "} & " << r.prov_unattributable_ablated << " & "
       << (r.role_guard_rejects ? "reject" : "--") << " & "
       << (r.role_raw_silent ? "silent" : "fails") << " & "
       << (r.merge_applicable ? sci(r.merge_blowup) + "$\\times$" : std::string("--"))
       << " \\\\\n";
  }
  os << "\\bottomrule\n\\end{tabular}\n";
  return os.str();
}

}  // namespace hacdcpf::sppt
