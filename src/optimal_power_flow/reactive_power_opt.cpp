/// @file  reactive_power_opt.cpp
/// @brief Mixed discrete/continuous reactive-power optimisation heuristic.
///
/// Solves the RPO problem using local sensitivity planes combined with
/// coordinate search over discrete variables
/// (OLTC tap positions and switchable shunt steps).
///
///   Phase 0 – Sensitivity ranking: ±1 perturbation of each variable
///             around baseline to rank by |Δobj|.  Variables are then
///             processed most-sensitive-first in all subsequent phases.
///   Phase 1 – Ternary-search sweep: for each variable, exploit
///             approximate unimodality via discrete ternary search
///             (O(log R) evals), accumulating local sensitivity data.
///   Phase 2 – Coordinate descent over every admissible value.
///   Phase 3 – Pairwise neighbourhood refinement: check ±1
///             perturbations of variable pairs.
///
/// Additional per-solve optimizations:
///   • In-place system modification (no deep copy)
///   • Solution cache (avoid redundant NLP evaluations)

#include "hacdcpf/optimal_power_flow/reactive_power_opt.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"
#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/pv_power_curve.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <map>
#include <numeric>
#include <vector>

namespace hacdcpf::opf {

namespace {

// ───────────────────────── helpers ──────────────────────────────

struct TapInfo {
  int trafo_idx;
  int tap_pos, tap_min, tap_max, tap_neutral;
  double tap_step_pct;
  double ratio_before;
  std::string name;
};

struct ShuntInfo {
  int shunt_idx;
  int current_step, n_steps;
  double bs_per_step;
  double bs_before;
  std::string name;
};

inline double tap_ratio(int tap_pos, int tap_neutral, double step_pct) {
  return 1.0 + (tap_pos - tap_neutral) * step_pct / 100.0;
}

/// Collect transformers that have an adjustable OLTC range.
std::vector<TapInfo> collect_taps(const HybridPowerSystem& sys) {
  std::vector<TapInfo> out;
  for (size_t i = 0; i < sys.ac.transformers_2w.size(); ++i) {
    const auto& t = sys.ac.transformers_2w[i];
    if (!t.in_service) continue;
    if (t.tap_max <= t.tap_min || t.tap_step_percent <= 0.0) continue;
    TapInfo ti;
    ti.trafo_idx    = static_cast<int>(i);
    ti.tap_pos      = t.tap_pos;
    ti.tap_min      = t.tap_min;
    ti.tap_max      = t.tap_max;
    ti.tap_neutral  = t.tap_neutral;
    ti.tap_step_pct = t.tap_step_percent;
    ti.ratio_before = tap_ratio(t.tap_pos, t.tap_neutral, t.tap_step_percent);
    ti.name         = t.name.empty() ? "Trafo" + std::to_string(i) : t.name;
    out.push_back(ti);
  }
  return out;
}

/// Collect switchable shunts with more than one step.
std::vector<ShuntInfo> collect_shunts(const HybridPowerSystem& sys) {
  std::vector<ShuntInfo> out;
  for (size_t i = 0; i < sys.ac.shunts.size(); ++i) {
    const auto& sh = sys.ac.shunts[i];
    if (!sh.in_service || !sh.switchable) continue;
    if (sh.n_steps <= 1) continue;
    ShuntInfo si;
    si.shunt_idx    = static_cast<int>(i);
    si.current_step = sh.current_step;
    si.n_steps      = sh.n_steps;
    si.bs_per_step  = sh.bs_per_step;
    si.bs_before    = sh.bs_mvar;
    si.name         = sh.name.empty()
                        ? "Shunt" + std::to_string(i) + "@B" + std::to_string(sh.bus)
                        : sh.name;
    out.push_back(si);
  }
  return out;
}

/// Compute total active-power loss = sum(all generation) - sum(all demand).
///
/// Generation sources in a hybrid AC/DC system:
///   pg_mw    – conventional (dispatchable) generators
///   pren_mw  – curtailable renewables (wind, hydro, PV)
///   pstor_mw – storage discharge (positive = injection)
///   pac_mw   – VSC converter AC-side injection (positive = into AC)
///   static_generators – fixed DER injections (not in OPF result vectors)
double compute_loss(const ACOPFResult& r, const HybridPowerSystem& sys) {
  double gen_total = 0.0;

  // Conventional generators
  for (double p : r.pg_mw) gen_total += p;

  // External grids are genuine boundary injections and must be included.
  for (double p : r.external_grid_p_mw) gen_total += p;

  // Curtailable renewables (wind, hydro, PV dispatched by OPF)
  for (double p : r.pren_mw) gen_total += p;

  // Storage dispatch (positive = discharge/injection into grid)
  for (double p : r.pstor_mw) gen_total += p;

  // VSC/DCDC/energy-router powers are internal transfers, not generation, and
  // therefore must not be added to the system source ledger.

  // Static generators (fixed output, not in OPF result vectors)
  for (const auto& sg : sys.ac.static_generators)
    if (sg.in_service) gen_total += sg.p_mw * sg.scaling;

  // Non-curtailable renewables (fixed output, not in OPF result vectors)
  for (const auto& rg : sys.ac.renewable_gens)
    if (rg.in_service && !rg.curtailable) gen_total += rg.p_mw;

  // Non-controllable PV systems (fixed output, not OPF variables)
  for (const auto& pv : sys.ac.pv_systems)
    if (pv.in_service && !pv.controllable)
      gen_total += powerflow::compute_pv_power_mw(pv);

  for (const auto& sg : sys.dc.static_generators)
    if (sg.in_service) gen_total += sg.p_mw * sg.scaling;
  for (const auto& sg : sys.dc.dc_static_generators)
    if (sg.in_service) gen_total += sg.p_set_mw * sg.scaling;
  for (const auto& pv : sys.dc.pv_arrays)
    if (pv.in_service) gen_total += pv.p_set_mw;
  for (const auto& mobile : sys.mobile_storage)
    if (mobile.in_service) gen_total += mobile.p_mw;
  for (const auto& vpp : sys.vpps)
    if (vpp.in_service) gen_total += vpp.p_output_mw;
  for (const auto& microgrid : sys.microgrids)
    if (microgrid.in_service) gen_total -= microgrid.p_exchange_mw;

  double pd_total = 0.0;
  for (const auto& bus : sys.ac.buses)
    if (bus.in_service) pd_total += bus.pd_mw;
  for (const auto& ld : sys.ac.loads)
    if (ld.in_service) pd_total += ld.p_mw * ld.scaling;
  for (const auto& ld : sys.ac.asymmetric_loads)
    if (ld.in_service) pd_total += ld.pa_mw + ld.pb_mw + ld.pc_mw;
  if (!r.pflex_mw.empty()) {
    for (double p : r.pflex_mw) pd_total += p;
  } else {
    for (const auto& ld : sys.ac.flexible_loads)
      if (ld.in_service) pd_total += ld.p_mw;
  }
  for (const auto& station : sys.ac.charging_stations) {
    if (!station.in_service) continue;
    pd_total += station.p_total_kw > 0.0
        ? station.p_total_kw / 1000.0
        : station.max_power_kw * station.utilization_rate *
              station.simultaneity_factor / 1000.0;
  }
  for (const auto& motor : sys.ac.motors) {
    if (!motor.in_service) continue;
    pd_total += motor.efficiency > 1e-9
        ? motor.sn_mva * motor.cos_phi / motor.efficiency
        : motor.sn_mva * motor.cos_phi;
  }
  for (const auto& bus : sys.dc.buses)
    if (bus.in_service) pd_total += bus.pd_mw;
  for (const auto& ld : sys.dc.loads)
    if (ld.in_service) pd_total += ld.p_mw * ld.scaling;
  for (double shed : r.dpd_mw) pd_total -= std::max(0.0, shed);

  return gen_total - pd_total;
}

/// RPO objective evaluated from an OPF solution.
double rpo_objective(const ACOPFResult& r,
                     const HybridPowerSystem& sys,
                     const RPOOptions& opt) {
  double obj = 0.0;
  if (opt.objective == RPOObjective::MinVoltageDeviation ||
      opt.objective == RPOObjective::Combined) {
    for (double v : r.vm) {
      const double d = v - opt.v_target;
      obj += opt.vdev_weight * d * d;
    }
  }
  if (opt.objective == RPOObjective::MinActiveLoss ||
      opt.objective == RPOObjective::Combined) {
    obj += opt.loss_weight * compute_loss(r, sys);
  }
  return obj;
}

double max_voltage_deviation(const std::vector<double>& vm, double vt) {
  double mx = 0.0;
  for (double v : vm) mx = std::max(mx, std::abs(v - vt));
  return mx;
}

/// Apply discrete settings to a mutable system (in-place).
void apply_settings(HybridPowerSystem& sys,
                    const std::vector<TapInfo>& taps,
                    const std::vector<int>& tap_pos,
                    const std::vector<ShuntInfo>& shunts,
                    const std::vector<int>& shunt_steps) {
  for (size_t k = 0; k < taps.size(); ++k) {
    auto& t = sys.ac.transformers_2w.at(
        static_cast<size_t>(taps[k].trafo_idx));
    t.tap_pos = tap_pos[k];
  }
  for (size_t k = 0; k < shunts.size(); ++k) {
    auto& sh = sys.ac.shunts.at(
        static_cast<size_t>(shunts[k].shunt_idx));
    sh.current_step = shunt_steps[k];
    sh.bs_mvar = sh.bs_per_step * shunt_steps[k];
  }
}

/// Solve AC OPF using in-place modification (avoids deep copy).
/// Restores the original settings on the mutable copy before returning.
ACOPFResult solve_opf_inplace(
    HybridPowerSystem& mut_sys,
    const std::vector<TapInfo>& taps,   const std::vector<int>& tap_pos,
    const std::vector<ShuntInfo>& shunts, const std::vector<int>& shunt_steps,
    const std::vector<int>& orig_tap_pos,
    const std::vector<int>& orig_shunt_steps,
    const RPOOptions& rpo_opt)
{
  apply_settings(mut_sys, taps, tap_pos, shunts, shunt_steps);

  ACOPFOptions opf_opt;
  opf_opt.ac_solver_backend = ACOPFSolverBackend::Ipopt;
  opf_opt.enable_primal_dual = true;
  opf_opt.use_parity_ipm     = true;
  opf_opt.allow_fallback     = true;
  if (rpo_opt.objective == RPOObjective::MinActiveLoss) {
    opf_opt.objective = ACOPFObjective::ActiveLoss;
  } else if (rpo_opt.objective == RPOObjective::Combined) {
    opf_opt.objective = ACOPFObjective::VoltageDeviationAndLoss;
  } else {
    opf_opt.objective = ACOPFObjective::VoltageDeviation;
  }
  opf_opt.voltage_target_pu = rpo_opt.v_target;
  opf_opt.voltage_deviation_weight = rpo_opt.vdev_weight;
  opf_opt.active_loss_weight = rpo_opt.loss_weight;
  opf_opt.max_inner_iterations = rpo_opt.max_ipm_iter;
  opf_opt.feasibility_tol = rpo_opt.ipm_tol;
  opf_opt.enforce_branch_limits = rpo_opt.enforce_branch_limits;
  opf_opt.enforce_converter_capacity = rpo_opt.enforce_converter_capacity;
  opf_opt.enforce_converter_current_limits =
      rpo_opt.enforce_converter_current_limits;
  opf_opt.enforce_converter_modulation_limits =
      rpo_opt.enforce_converter_modulation_limits;
  opf_opt.verbose = rpo_opt.verbose;

  ACOPFResult r = solve_ac_opf(mut_sys, opf_opt);

  // Restore original settings so mut_sys is clean for next call
  apply_settings(mut_sys, taps, orig_tap_pos, shunts, orig_shunt_steps);
  return r;
}

// ───────── Local sensitivity planes (diagnostic, never pruning) ──────────

/// A local linear sensitivity model generated at an evaluated point y_k:
///   f(y) >= f_k + g_k^T (y - y_k)
/// where g_k is a finite-difference gradient estimate.
struct LocalSensitivityPlane {
  std::vector<double> y;          // evaluation point
  double              f{1e30};    // objective at y
  std::vector<double> g;          // gradient estimate

  double predict(const std::vector<double>& yt) const {
    double val = f;
    for (size_t i = 0; i < g.size(); ++i)
      val += g[i] * (yt[i] - y[i]);
    return val;
  }
};

}  // anonymous namespace

// ════════════════════════════════════════════════════════════════
//  Public API
// ════════════════════════════════════════════════════════════════

RPOResult solve_rpo(const HybridPowerSystem& sys, const RPOOptions& opt) {
  RPOResult out;
  const auto t0 = std::chrono::steady_clock::now();

  const auto taps   = collect_taps(sys);
  const auto shunts = collect_shunts(sys);
  const int n_taps   = static_cast<int>(taps.size());
  const int n_shunts = static_cast<int>(shunts.size());
  const int n_disc   = n_taps + n_shunts;

  // ── Helper lambdas for discrete-variable access ──
  auto var_lo = [&](int vi) -> int {
    return vi < n_taps ? taps[vi].tap_min : 0;
  };
  auto var_hi = [&](int vi) -> int {
    return vi < n_taps ? taps[vi].tap_max : shunts[vi - n_taps].n_steps;
  };
  auto get_var = [&](const std::vector<int>& tp,
                     const std::vector<int>& ss, int vi) -> int {
    return vi < n_taps ? tp[vi] : ss[vi - n_taps];
  };
  auto set_var = [&](std::vector<int>& tp,
                     std::vector<int>& ss, int vi, int v) {
    if (vi < n_taps) tp[vi] = v;
    else             ss[vi - n_taps] = v;
  };
  auto to_dvec = [&](const std::vector<int>& tp,
                     const std::vector<int>& ss) -> std::vector<double> {
    std::vector<double> y;
    y.reserve(static_cast<size_t>(n_disc));
    for (int v : tp) y.push_back(static_cast<double>(v));
    for (int v : ss) y.push_back(static_cast<double>(v));
    return y;
  };

  // ── Original settings for restore-after-solve ──
  std::vector<int> orig_tp;
  for (const auto& t : taps)   orig_tp.push_back(t.tap_pos);
  std::vector<int> orig_ss;
  for (const auto& s : shunts) orig_ss.push_back(s.current_step);

  // ── Mutable copy of system for in-place modification ──
  HybridPowerSystem mut_sys = sys;

  // ── Baseline AC OPF ("before" state) ──
  std::vector<int> base_tp = orig_tp;
  std::vector<int> base_ss = orig_ss;
  ACOPFResult base_r = solve_opf_inplace(mut_sys, taps, base_tp,
                                          shunts, base_ss,
                                          orig_tp, orig_ss, opt);
  out.nlp_solves = 1;
  out.baseline_opf = base_r;
  if (!base_r.converged) {
    out.status = "Baseline AC OPF failed: " + base_r.status;
    out.runtime_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    return out;
  }
  out.vm_before            = base_r.vm;
  out.va_before            = base_r.va;
  out.qg_mvar_before       = base_r.qg_mvar;
  out.pg_mw_before         = base_r.pg_mw;
  out.total_loss_before_mw = compute_loss(base_r, sys);
  out.max_vdev_before      = max_voltage_deviation(base_r.vm, opt.v_target);

  // ── No discrete devices → baseline is optimal ──
  if (n_disc == 0) {
    out.converged = true;
    out.vm_after            = out.vm_before;
    out.va_after            = out.va_before;
    out.qg_mvar_after       = out.qg_mvar_before;
    out.pg_mw_after         = out.pg_mw_before;
    out.total_loss_after_mw = out.total_loss_before_mw;
    out.max_vdev_after      = out.max_vdev_before;
    out.objective           = rpo_objective(base_r, sys, opt);
    out.optimized_opf       = base_r;
    out.gap                 = 1.0;
    out.nodes_explored = 1;
    out.runtime_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    out.status = "Feasible OPF point (no adjustable discrete devices)";
    return out;
  }

  // ═══════════════════════════════════════════════════════════════
  //  Sensitivity-guided coordinate optimisation
  // ═══════════════════════════════════════════════════════════════

  // -- Incumbent tracking --
  double      incumbent_obj = rpo_objective(base_r, sys, opt);
  ACOPFResult incumbent_result = base_r;
  std::vector<int> incumbent_tp = base_tp;
  std::vector<int> incumbent_ss = base_ss;
  out.nodes_explored = 1;

  // -- Local sensitivity-plane pool & solution cache --
  std::vector<LocalSensitivityPlane> planes;
  std::map<std::vector<int>, double> eval_cache;

  // Cache the baseline evaluation
  {
    std::vector<int> key;
    key.insert(key.end(), base_tp.begin(), base_tp.end());
    key.insert(key.end(), base_ss.begin(), base_ss.end());
    eval_cache[key] = incumbent_obj;
  }

  auto budget_ok = [&]() -> bool {
    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    return elapsed < opt.time_limit_sec &&
           out.nlp_solves < opt.max_nodes;
  };

  // Evaluate NLP at a candidate; updates incumbent if improved.
  // Returns objective (1e30 if infeasible or already cached-infeasible).
  auto evaluate = [&](const std::vector<int>& tp,
                      const std::vector<int>& ss) -> double {
    std::vector<int> key;
    key.insert(key.end(), tp.begin(), tp.end());
    key.insert(key.end(), ss.begin(), ss.end());
    auto it = eval_cache.find(key);
    if (it != eval_cache.end()) return it->second;

    ACOPFResult r = solve_opf_inplace(mut_sys, taps, tp,
                                      shunts, ss,
                                      orig_tp, orig_ss, opt);
    ++out.nlp_solves;
    ++out.nodes_explored;

    double obj = 1e30;
    if (r.converged) {
      obj = rpo_objective(r, sys, opt);
      if (obj < incumbent_obj) {
        incumbent_obj    = obj;
        incumbent_result = std::move(r);
        incumbent_tp     = tp;
        incumbent_ss     = ss;
      }
    }
    eval_cache[key] = obj;
    return obj;
  };

  // Helper: retain a local 1-D sensitivity plane for diagnostics.
  auto add_1d_plane = [&](const std::vector<int>& tp,
                          const std::vector<int>& ss,
                          int vi, double obj, double dobj_dvar) {
    LocalSensitivityPlane plane;
    plane.y = to_dvec(tp, ss);
    plane.f = obj;
    plane.g.assign(static_cast<size_t>(n_disc), 0.0);
    plane.g[vi] = dobj_dvar;
    planes.push_back(std::move(plane));
  };

  // ─── Phase 0: Sensitivity ranking ────────────────────────────
  //
  // Perturb each variable by ±1 from baseline (2 solves per var),
  // compute |Δobj|, and order variables by sensitivity (descending).
  // This ensures the most impactful variables are optimised first.

  struct SensEntry { int var_idx; double sensitivity; };
  std::vector<SensEntry> sensitivities;

  for (int vi = 0; vi < n_disc && budget_ok(); ++vi) {
    int cur = get_var(base_tp, base_ss, vi);
    double best_delta = 0.0;

    for (int dir : {-1, 1}) {
      int nv = cur + dir;
      if (nv < var_lo(vi) || nv > var_hi(vi)) continue;
      auto trial_tp = base_tp;
      auto trial_ss = base_ss;
      set_var(trial_tp, trial_ss, vi, nv);
      double obj = evaluate(trial_tp, trial_ss);
      if (obj < 1e20) {
        best_delta = std::max(best_delta, std::abs(obj - incumbent_obj));

        // Generate OA cut from this perturbation
        double grad = (obj - incumbent_obj) / dir;
        add_1d_plane(trial_tp, trial_ss, vi, obj, grad);
      }
    }
    sensitivities.push_back({vi, best_delta});
  }

  // Sort by decreasing sensitivity
  std::sort(sensitivities.begin(), sensitivities.end(),
            [](const SensEntry& a, const SensEntry& b) {
              return a.sensitivity > b.sensitivity;
            });

  std::vector<int> var_order;
  for (const auto& se : sensitivities)
    var_order.push_back(se.var_idx);

  // ─── Phase 1: Ternary-search sweep per variable ───────────────
  //
  // For each variable (in sensitivity order), perform a discrete
  // ternary search exploiting approximate unimodality of the RPO
  // objective in each variable.  This reduces O(R) to O(log R)
  // evaluations per variable.  Falls back to golden-section-style
  // narrowing on the integer grid.

  for (int vi : var_order) {
    if (!budget_ok()) break;
    int lo = var_lo(vi);
    int hi = var_hi(vi);

    // Ternary search on integer grid
    while (hi - lo > 2 && budget_ok()) {
      int m1 = lo + (hi - lo) / 3;
      int m2 = hi - (hi - lo) / 3;

      auto tp1 = incumbent_tp, ss1 = incumbent_ss;
      set_var(tp1, ss1, vi, m1);
      double f1 = evaluate(tp1, ss1);

      auto tp2 = incumbent_tp, ss2 = incumbent_ss;
      set_var(tp2, ss2, vi, m2);
      double f2 = evaluate(tp2, ss2);

      // Retain local sensitivity planes from both evaluations.
      if (f1 < 1e20 && f2 < 1e20 && m2 != m1) {
        double grad = (f2 - f1) / (m2 - m1);
        add_1d_plane(tp1, ss1, vi, f1, grad);
        add_1d_plane(tp2, ss2, vi, f2, grad);
      }

      if (f1 < f2)
        hi = m2 - 1;
      else
        lo = m1 + 1;
    }

    // Exhaustive search in the remaining small window [lo, hi]
    for (int v = lo; v <= hi && budget_ok(); ++v) {
      auto sweep_tp = incumbent_tp;
      auto sweep_ss = incumbent_ss;
      set_var(sweep_tp, sweep_ss, vi, v);
      evaluate(sweep_tp, sweep_ss);
    }
  }

  // ─── Phase 2: Coordinate descent ──────────────────────────────
  //
  // Every admissible coordinate value is evaluated.  The local sensitivity
  // planes are not globally valid lower bounds for this non-convex MINLP and
  // therefore are never used to prune candidates.

  constexpr int kMaxCDCycles = 3;
  for (int cycle = 0; cycle < kMaxCDCycles && budget_ok(); ++cycle) {
    bool improved_this_cycle = false;
    for (int vi : var_order) {
      if (!budget_ok()) break;
      int cur_val = get_var(incumbent_tp, incumbent_ss, vi);
      auto sweep_tp = incumbent_tp;
      auto sweep_ss = incumbent_ss;

      for (int v = var_lo(vi); v <= var_hi(vi) && budget_ok(); ++v) {
        if (v == cur_val) continue;
        set_var(sweep_tp, sweep_ss, vi, v);

        auto y_cand = to_dvec(sweep_tp, sweep_ss);

        double obj = evaluate(sweep_tp, sweep_ss);

        // Add OA cut from this evaluation
        if (obj < 1e20) {
          auto y_inc = to_dvec(incumbent_tp, incumbent_ss);
          double dy = y_cand[vi] - y_inc[vi];
          double grad = (std::abs(dy) > 0.5)
                          ? (obj - incumbent_obj) / dy : 0.0;
          add_1d_plane(sweep_tp, sweep_ss, vi, obj, grad);

          if (obj < incumbent_obj - opt.gap_tol)
            improved_this_cycle = true;
        }
      }
    }
    if (!improved_this_cycle) break;
  }

  // ─── Phase 3: Pairwise neighbourhood polishing ────────────────
  //
  // Check ±1 perturbations of every pair of discrete variables
  // around the incumbent, to capture 2-variable interactions that
  // coordinate descent may miss.

  for (size_t ii = 0; ii < var_order.size() && budget_ok(); ++ii) {
    for (size_t jj = ii + 1; jj < var_order.size() && budget_ok(); ++jj) {
      int vi = var_order[ii];
      int vj = var_order[jj];
      int ci = get_var(incumbent_tp, incumbent_ss, vi);
      int cj = get_var(incumbent_tp, incumbent_ss, vj);
      for (int di : {-1, 1}) {
        int ni = ci + di;
        if (ni < var_lo(vi) || ni > var_hi(vi)) continue;
        for (int dj : {-1, 1}) {
          if (!budget_ok()) break;
          int nj = cj + dj;
          if (nj < var_lo(vj) || nj > var_hi(vj)) continue;

          auto trial_tp = incumbent_tp;
          auto trial_ss = incumbent_ss;
          set_var(trial_tp, trial_ss, vi, ni);
          set_var(trial_tp, trial_ss, vj, nj);

          evaluate(trial_tp, trial_ss);
        }
      }
    }
  }

  // ── Fill results ──
  if (incumbent_obj < 1e20) {
    out.converged           = true;
    out.objective           = incumbent_obj;
    out.vm_after            = incumbent_result.vm;
    out.va_after            = incumbent_result.va;
    out.qg_mvar_after       = incumbent_result.qg_mvar;
    out.pg_mw_after         = incumbent_result.pg_mw;
    out.optimized_opf       = incumbent_result;
    out.total_loss_after_mw = compute_loss(incumbent_result, sys);
    out.max_vdev_after      = max_voltage_deviation(incumbent_result.vm, opt.v_target);

    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    if (elapsed >= opt.time_limit_sec)
      out.status = "Feasible incumbent at time limit";
    else if (out.nlp_solves >= opt.max_nodes)
      out.status = "Feasible incumbent at evaluation limit";
    else
      out.status = "Feasible local optimum (heuristic search)";
    out.gap = 1.0;
  } else {
    out.converged = false;
    out.vm_after            = out.vm_before;
    out.va_after            = out.va_before;
    out.qg_mvar_after       = out.qg_mvar_before;
    out.pg_mw_after         = out.pg_mw_before;
    out.total_loss_after_mw = out.total_loss_before_mw;
    out.max_vdev_after      = out.max_vdev_before;
    out.status = "No feasible configuration found";
  }

  // ── Tap / shunt detail entries ──
  for (int k = 0; k < n_taps; ++k) {
    TapResult tr;
    tr.trafo_index  = taps[k].trafo_idx;
    tr.name         = taps[k].name;
    tr.tap_before   = taps[k].tap_pos;
    tr.tap_after    = incumbent_obj < 1e20 ? incumbent_tp[k] : taps[k].tap_pos;
    tr.ratio_before = taps[k].ratio_before;
    tr.ratio_after  = tap_ratio(tr.tap_after, taps[k].tap_neutral,
                                taps[k].tap_step_pct);
    out.taps.push_back(tr);
  }
  for (int k = 0; k < n_shunts; ++k) {
    ShuntResult sr;
    sr.shunt_index    = shunts[k].shunt_idx;
    sr.name           = shunts[k].name;
    sr.step_before    = shunts[k].current_step;
    sr.step_after     = incumbent_obj < 1e20 ? incumbent_ss[k] : shunts[k].current_step;
    sr.bs_mvar_before = shunts[k].bs_before;
    sr.bs_mvar_after  = shunts[k].bs_per_step * sr.step_after;
    out.shunts.push_back(sr);
  }

  // ── Legacy statistics envelope ──
  out.bc_stats.nodes_explored = out.nodes_explored;
  out.bc_stats.lp_solves      = out.nlp_solves;
  out.bc_stats.cuts_added     = static_cast<int>(planes.size());
  out.bc_stats.best_obj       = incumbent_obj < 1e20 ? incumbent_obj : 0.0;
  out.bc_stats.gap            = out.gap;
  out.bc_stats.status         = out.status;

  out.runtime_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t0).count();
  out.bc_stats.runtime_sec = out.runtime_sec;

  return out;
}

}  // namespace hacdcpf::opf
