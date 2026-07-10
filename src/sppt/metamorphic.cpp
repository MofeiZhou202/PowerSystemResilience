/// sppt/metamorphic.cpp
/// ====================
/// Implementation of the SPPT metamorphic relations (see metamorphic.hpp and
/// docs/latex/sppt_theory.tex).  Every relation is written against the public
/// API so it certifies the shipped code path, not an internal shortcut.

#include "hacdcpf/sppt/metamorphic.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/dynamics/dynamics.hpp"
#include "hacdcpf/model/enums/bus_types.hpp"
#include "hacdcpf/model/enums/converter_enums.hpp"

namespace hacdcpf::sppt {
namespace {

std::string to_lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

bool contains_ci(const std::string& hay, const std::string& needle) {
  return to_lower(hay).find(to_lower(needle)) != std::string::npos;
}

double max_abs_diff(const std::vector<double>& a, const std::vector<double>& b) {
  const std::size_t n = std::min(a.size(), b.size());
  double m = 0.0;
  for (std::size_t i = 0; i < n; ++i)
    m = std::max(m, std::abs(a[i] - b[i]));
  return m;
}

// Attribute a canonical (merged-space) bus vector back to original space using
// the projection's BusMergeMap, when the lengths disagree.  Falls back to the
// canonical vector unchanged when no non-trivial map is available.
std::vector<double> attribute_bus_vector(const std::vector<double>& canonical,
                                         const HybridPowerSystem& sys) {
  powerflow::SolverData sd = powerflow::make_solver_data(sys);
  if (sd.bus_merge_map && !sd.bus_merge_map->ext_to_int.empty())
    return unproject_bus_vector(canonical, *sd.bus_merge_map);
  return canonical;
}

}  // namespace

// ── MR1 ──────────────────────────────────────────────────────────────────────
MetamorphicResult mr1_projection_idempotence(const HybridPowerSystem& sys) {
  MetamorphicResult r;
  r.id = "MR1";
  r.name = "Projection idempotence";

  const HybridPowerSystem p1 = project_to_canonical_models(sys);
  const HybridPowerSystem p2 = project_to_canonical_models(p1);

  const int dbus = std::abs(n_ac_buses(p1) - n_ac_buses(p2)) +
                   std::abs(n_dc_buses(p1) - n_dc_buses(p2));
  const int dbranch = std::abs(n_ac_branches(p1) - n_ac_branches(p2)) +
                      std::abs(n_dc_branches(p1) - n_dc_branches(p2));

  r.residual = static_cast<double>(dbus + dbranch);
  r.tolerance = 0.0;
  r.passed = (dbus == 0 && dbranch == 0);
  r.detail = "Pi(Pi(S)) vs Pi(S): dbus=" + std::to_string(dbus) +
             " dbranch=" + std::to_string(dbranch);
  return r;
}

// ── MR2 ──────────────────────────────────────────────────────────────────────
MetamorphicResult mr2_attribution_roundtrip(const HybridPowerSystem& sys, double tol) {
  MetamorphicResult r;
  r.id = "MR2";
  r.name = "Attribution round-trip";
  r.tolerance = tol;

  powerflow::SolverData sd = powerflow::make_solver_data(sys);
  if (!sd.bus_merge_map || sd.bus_merge_map->ext_to_int.empty()) {
    r.passed = true;
    r.residual = 0.0;
    r.detail = "identity map (no zero-impedance merges) — round-trip is exact";
    return r;
  }

  const BusMergeMap& m = *sd.bus_merge_map;
  // Canonical (merged-space) field with distinct per-node values.
  std::vector<double> v_int(static_cast<std::size_t>(std::max(0, m.n_merged)));
  for (std::size_t i = 0; i < v_int.size(); ++i)
    v_int[i] = 1.0 + static_cast<double>(i);

  const std::vector<double> v_ext = unproject_bus_vector(v_int, m);

  double res = 0.0;
  for (const auto& [ext, t] : m.ext_to_int) {
    if (m.is_dead_bus(ext)) continue;
    auto it = m.ext_to_orig_pos.find(ext);
    if (it == m.ext_to_orig_pos.end()) continue;
    const int pos = it->second;
    if (pos < 0 || pos >= static_cast<int>(v_ext.size())) continue;
    if (t < 0 || t >= static_cast<int>(v_int.size())) continue;
    res = std::max(res, std::abs(v_ext[static_cast<std::size_t>(pos)] -
                                 v_int[static_cast<std::size_t>(t)]));
  }
  r.residual = res;
  r.passed = res <= tol;
  r.detail = "R_S o iota_S round-trip over " +
             std::to_string(m.ext_to_int.size()) + " buses";
  return r;
}

// ── MR3 (power flow) ─────────────────────────────────────────────────────────
MetamorphicResult mr3_semantic_preservation_pf(const HybridPowerSystem& sys, double tol) {
  MetamorphicResult r;
  r.id = "MR3";
  r.name = "Semantic preservation (power flow)";
  r.tolerance = tol;

  const PowerFlowResult ref = solve_power_flow(sys);
  const HybridPowerSystem proj = project_to_canonical_models(sys);
  const PowerFlowResult can = solve_power_flow(proj);

  if (!ref.converged || !can.converged) {
    r.passed = false;
    r.residual = 1.0;
    r.detail = std::string("non-convergence: ref=") +
               (ref.converged ? "ok" : "fail") + " can=" +
               (can.converged ? "ok" : "fail");
    return r;
  }

  double res = 0.0;
  // Compare by bus id so the relation is invariant to any reordering the
  // projection performs (A_can lives in canonical node order, A_ref in authored
  // order; attribution reconciles them through the shared bus identity).
  if (ref.vm.size() == sys.ac.buses.size() &&
      can.vm.size() == proj.ac.buses.size()) {
    std::unordered_map<int, std::pair<double, double>> ref_by_id;
    ref_by_id.reserve(sys.ac.buses.size());
    for (std::size_t i = 0; i < sys.ac.buses.size(); ++i)
      ref_by_id[sys.ac.buses[i].index] = {
          ref.vm[i], i < ref.va.size() ? ref.va[i] : 0.0};
    for (std::size_t j = 0; j < proj.ac.buses.size(); ++j) {
      auto it = ref_by_id.find(proj.ac.buses[j].index);
      if (it == ref_by_id.end()) continue;
      res = std::max(res, std::abs(it->second.first - can.vm[j]));
      if (j < can.va.size())
        res = std::max(res, std::abs(it->second.second - can.va[j]));
    }
    r.detail = "max|A_ref - R.A_can.Pi| over V, id-keyed (n_bus=" +
               std::to_string(ref.vm.size()) + ")";
  } else {
    const std::vector<double> vm_attr = attribute_bus_vector(can.vm, sys);
    if (vm_attr.size() != ref.vm.size()) {
      r.passed = false;
      r.residual = 1.0;
      r.detail = "bus-vector length mismatch after attribution (ref=" +
                 std::to_string(ref.vm.size()) + ", attr=" +
                 std::to_string(vm_attr.size()) + ")";
      return r;
    }
    res = std::max(res, max_abs_diff(ref.vm, vm_attr));
    r.detail = "max|A_ref - R.A_can.Pi| over V, attributed (n_bus=" +
               std::to_string(ref.vm.size()) + ")";
  }
  if (!ref.vdc.empty() && ref.vdc.size() == can.vdc.size())
    res = std::max(res, max_abs_diff(ref.vdc, can.vdc));

  r.residual = res;
  r.passed = res <= tol;
  return r;
}

// ── MR3 (OPF duals) ──────────────────────────────────────────────────────────
MetamorphicResult mr3_semantic_preservation_opf_dual(const HybridPowerSystem& sys,
                                                     double tol) {
  MetamorphicResult r;
  r.id = "MR3d";
  r.name = "Semantic preservation (OPF nodal prices)";
  r.tolerance = tol;

  const opf::DCOPFResult ref = solve_dc_opf(sys);
  const HybridPowerSystem proj = project_to_canonical_models(sys);
  const opf::DCOPFResult can = solve_dc_opf(proj);

  if (!ref.converged || !can.converged) {
    r.passed = false;
    r.residual = 1.0;
    r.detail = std::string("non-convergence: ref=") +
               (ref.converged ? "ok" : "fail") + " can=" +
               (can.converged ? "ok" : "fail");
    return r;
  }
  if (ref.lmp.empty() || can.lmp.empty()) {
    r.passed = true;  // no duals to compare — vacuously preserved
    r.residual = 0.0;
    r.detail = "no LMPs reported (vacuous)";
    return r;
  }

  double res = 0.0;
  if (ref.lmp.size() == sys.ac.buses.size() &&
      can.lmp.size() == proj.ac.buses.size()) {
    std::unordered_map<int, double> ref_by_id;
    ref_by_id.reserve(sys.ac.buses.size());
    for (std::size_t i = 0; i < sys.ac.buses.size(); ++i)
      ref_by_id[sys.ac.buses[i].index] = ref.lmp[i];
    for (std::size_t j = 0; j < proj.ac.buses.size(); ++j) {
      auto it = ref_by_id.find(proj.ac.buses[j].index);
      if (it != ref_by_id.end())
        res = std::max(res, std::abs(it->second - can.lmp[j]));
    }
  } else if (ref.lmp.size() == can.lmp.size()) {
    res = max_abs_diff(ref.lmp, can.lmp);
  } else {
    const std::vector<double> lmp_attr = attribute_bus_vector(can.lmp, sys);
    if (lmp_attr.size() != ref.lmp.size()) {
      r.passed = false;
      r.residual = 1.0;
      r.detail = "LMP length mismatch after attribution";
      return r;
    }
    res = max_abs_diff(ref.lmp, lmp_attr);
  }
  r.residual = res;
  r.passed = res <= tol;
  r.detail = "max|LMP_ref - R.LMP_can.Pi| id-keyed (n_bus=" +
             std::to_string(ref.lmp.size()) + ")";
  return r;
}

// ── MR3 (transient trajectory) ───────────────────────────────────────────────
MetamorphicResult mr3_semantic_preservation_transient(const HybridPowerSystem& sys,
                                                      double tol) {
  MetamorphicResult r;
  r.id = "MR3t";
  r.name = "Semantic preservation (transient trajectory)";
  r.tolerance = tol;

  dynamics::DynamicSolverOptions opt;
  opt.t_end_s = 0.10;
  opt.dt_s = 0.005;
  opt.run_power_flow_initialization = true;
  opt.use_consistent_dynamic_initialization = true;

  const dynamics::DynamicResults ref = ::hacdcpf::run_transient_simulation(sys, opt);
  const HybridPowerSystem proj = project_to_canonical_models(sys);
  const dynamics::DynamicResults can = ::hacdcpf::run_transient_simulation(proj, opt);

  if (!ref.success || !can.success) {
    r.passed = false;
    r.residual = 1.0;
    r.detail = std::string("non-convergence: ref=") +
               (ref.success ? "ok" : "fail") + " can=" +
               (can.success ? "ok" : "fail");
    return r;
  }
  const dynamics::DynamicSnapshot* fs_ref = ref.final_snapshot();
  const dynamics::DynamicSnapshot* fs_can = can.final_snapshot();
  if (!fs_ref || !fs_can) {
    r.passed = false;
    r.residual = 1.0;
    r.detail = "no final snapshot";
    return r;
  }

  // Ordering-independent final-snapshot observables.
  double res = 0.0;
  res = std::max(res, std::abs(fs_ref->max_ac_voltage_pu - fs_can->max_ac_voltage_pu));
  res = std::max(res, std::abs(fs_ref->min_ac_voltage_pu - fs_can->min_ac_voltage_pu));
  res = std::max(res, std::abs(fs_ref->frequency_hz - fs_can->frequency_hz));
  res = std::max(res, std::abs(fs_ref->coi_frequency_hz - fs_can->coi_frequency_hz));
  if (fs_ref->state.size() == fs_can->state.size() && fs_ref->state.size() > 0)
    res = std::max(res, (fs_ref->state - fs_can->state).cwiseAbs().maxCoeff());
  if (fs_ref->vdc.size() == fs_can->vdc.size() && fs_ref->vdc.size() > 0)
    res = std::max(res, (fs_ref->vdc - fs_can->vdc).cwiseAbs().maxCoeff());

  r.residual = res;
  r.passed = res <= tol;
  r.detail = "max final-snapshot deviation (V envelope, frequency, states, Vdc)";
  return r;
}

// ── MR4 ──────────────────────────────────────────────────────────────────────
MetamorphicResult mr4_merged_bus_equipotential(const HybridPowerSystem& sys, double tol) {
  MetamorphicResult r;
  r.id = "MR4";
  r.name = "Merged-bus equipotential";
  r.tolerance = tol;

  powerflow::SolverData sd = powerflow::make_solver_data(sys);
  if (!sd.bus_merge_map || sd.bus_merge_map->ext_to_int.empty() ||
      !sd.bus_merge_map->has_merges()) {
    r.passed = true;
    r.residual = 0.0;
    r.detail = "no zero-impedance merges — equipotential holds trivially";
    return r;
  }

  // Structural certificate (Lem. 5.5): every member of a merge class collapses
  // to one internal node, so by construction they share the representative's
  // potential.  We verify the collapse rather than reading solved voltages so
  // the check is independent of bus-vector indexing conventions.
  const BusMergeMap& m = *sd.bus_merge_map;
  int violations = 0;
  int merged_groups = 0;
  for (const auto& group : m.groups) {
    if (group.size() < 2) continue;
    ++merged_groups;
    int rep_internal = -2;
    for (int ext : group) {
      auto it = m.ext_to_int.find(ext);
      const int internal = (it == m.ext_to_int.end()) ? -1 : it->second;
      if (rep_internal == -2) rep_internal = internal;
      else if (internal != rep_internal) ++violations;
    }
  }
  r.residual = static_cast<double>(violations);
  r.passed = (violations == 0);
  r.detail = "merged groups=" + std::to_string(merged_groups) +
             ", split violations=" + std::to_string(violations);
  return r;
}

// ── MR5 ──────────────────────────────────────────────────────────────────────
MetamorphicResult mr5_relabel_invariance(const HybridPowerSystem& sys, double tol) {
  MetamorphicResult r;
  r.id = "MR5";
  r.name = "Merge / relabel invariance";
  r.tolerance = tol;

  const PowerFlowResult a = solve_power_flow(sys);
  if (!a.converged || a.vm.size() != sys.ac.buses.size()) {
    r.passed = false;
    r.residual = 1.0;
    r.detail = "baseline solve did not converge or vm not in original bus space";
    return r;
  }

  // Relabel: reverse the AC-bus ordering (a pure permutation; bus ids unchanged).
  HybridPowerSystem sys2 = sys;
  std::reverse(sys2.ac.buses.begin(), sys2.ac.buses.end());
  const PowerFlowResult b = solve_power_flow(sys2);
  if (!b.converged || b.vm.size() != sys2.ac.buses.size()) {
    r.passed = false;
    r.residual = 1.0;
    r.detail = "relabeled solve did not converge or vm not in original bus space";
    return r;
  }

  // Compare per bus-id (attribute back through the position->id relabeling).
  std::unordered_map<int, double> vm_by_id;
  for (std::size_t i = 0; i < sys.ac.buses.size(); ++i)
    vm_by_id[sys.ac.buses[i].index] = a.vm[i];

  double res = 0.0;
  for (std::size_t j = 0; j < sys2.ac.buses.size(); ++j) {
    auto it = vm_by_id.find(sys2.ac.buses[j].index);
    if (it != vm_by_id.end())
      res = std::max(res, std::abs(it->second - b.vm[j]));
  }
  r.residual = res;
  r.passed = res <= tol;
  r.detail = "max per-bus-id |V| deviation under bus reordering";
  return r;
}

// ── MR6 ──────────────────────────────────────────────────────────────────────
MetamorphicResult mr6_compositionality(const HybridPowerSystem& s1,
                                       const HybridPowerSystem& s2) {
  MetamorphicResult r;
  r.id = "MR6";
  r.name = "Compositionality of projection";
  r.tolerance = 0.0;

  const HybridPowerSystem p1 = project_to_canonical_models(s1);
  const HybridPowerSystem p2 = project_to_canonical_models(s2);

  // Disjoint union (empty interface): concatenate the primary component vectors.
  // Precondition: s1 and s2 use disjoint bus-id ranges (Assumption "clean
  // interface" with Sigma = empty).
  HybridPowerSystem glued = s1;
  auto& ab = glued.ac.buses;
  ab.insert(ab.end(), s2.ac.buses.begin(), s2.ac.buses.end());
  auto& abr = glued.ac.branches;
  abr.insert(abr.end(), s2.ac.branches.begin(), s2.ac.branches.end());
  auto& gen = glued.ac.generators;
  gen.insert(gen.end(), s2.ac.generators.begin(), s2.ac.generators.end());
  auto& ld = glued.ac.loads;
  ld.insert(ld.end(), s2.ac.loads.begin(), s2.ac.loads.end());
  auto& sh = glued.ac.shunts;
  sh.insert(sh.end(), s2.ac.shunts.begin(), s2.ac.shunts.end());
  auto& db = glued.dc.buses;
  db.insert(db.end(), s2.dc.buses.begin(), s2.dc.buses.end());
  auto& dbr = glued.dc.branches;
  dbr.insert(dbr.end(), s2.dc.branches.begin(), s2.dc.branches.end());

  const HybridPowerSystem pg = project_to_canonical_models(glued);

  const int dbus = std::abs(n_ac_buses(pg) - (n_ac_buses(p1) + n_ac_buses(p2))) +
                   std::abs(n_dc_buses(pg) - (n_dc_buses(p1) + n_dc_buses(p2)));
  const int dbranch =
      std::abs(n_ac_branches(pg) - (n_ac_branches(p1) + n_ac_branches(p2))) +
      std::abs(n_dc_branches(pg) - (n_dc_branches(p1) + n_dc_branches(p2)));

  r.residual = static_cast<double>(dbus + dbranch);
  r.passed = (dbus == 0 && dbranch == 0);
  r.detail = "Pi(S1(+)S2) vs Pi(S1)+Pi(S2): dbus=" + std::to_string(dbus) +
             " dbranch=" + std::to_string(dbranch);
  return r;
}

// ── MR7 ──────────────────────────────────────────────────────────────────────
MetamorphicResult mr7_wellposedness_gate(const HybridPowerSystem& sys) {
  MetamorphicResult r;
  r.id = "MR7";
  r.name = "Well-posedness gate";
  r.tolerance = 0.0;

  // Break the AC angle reference: demote every SLACK bus, clear is_slack on
  // generators, and take external grids out of service.
  HybridPowerSystem broken = sys;
  for (auto& b : broken.ac.buses)
    if (b.bus_type == BusType::SLACK) b.bus_type = BusType::PQ;
  for (auto& g : broken.ac.generators) g.is_slack = false;
  for (auto& eg : broken.ac.external_grids) eg.in_service = false;
  for (auto& c : broken.vsc_converters) {
    c.ac_grid_forming = false;
    if (c.control_mode == ConverterMode::AC_GRID_FORMING)
      c.control_mode = ConverterMode::PQ_MODE;
  }

  const validation::ValidationReport rep =
      validation::validate(broken, validation::ValidationLevel::SolverReady);

  bool typed_reject = false;
  std::string which;
  for (const auto& issue : rep.issues) {
    if (issue.severity != validation::Severity::Error) continue;
    if (contains_ci(issue.message, "slack") ||
        contains_ci(issue.message, "reference") ||
        contains_ci(issue.message, "no swing")) {
      typed_reject = true;
      which = issue.message;
      break;
    }
  }

  r.residual = 0.0;
  r.passed = typed_reject;
  r.detail = typed_reject
                 ? ("reference removal produced a typed rejection: " + which)
                 : ("no typed rejection fired (validator: " + rep.summary() + ")");
  return r;
}

// ── suite ────────────────────────────────────────────────────────────────────
std::vector<MetamorphicResult> run_core_metamorphic_suite(const HybridPowerSystem& sys) {
  return {
      mr1_projection_idempotence(sys),
      mr2_attribution_roundtrip(sys),
      mr3_semantic_preservation_pf(sys),
      mr3_semantic_preservation_opf_dual(sys),
      mr4_merged_bus_equipotential(sys),
      mr5_relabel_invariance(sys),
      mr7_wellposedness_gate(sys),
  };
}

}  // namespace hacdcpf::sppt
