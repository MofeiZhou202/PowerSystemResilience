/// Security-Constrained Unit Commitment — standalone MILP formulation.
///
/// Implements Southern China Regional Electricity Market §2.6 (SCUC/SCED/LMP).
/// All HybridACDCPF dependencies removed; uses mipsolvers::engine::SolverEngine.

#include "mipsolvers/scuc/scuc.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <stdexcept>

#include <Eigen/LU>
#include <Eigen/Sparse>
#include <nlohmann/json.hpp>

#include "mipsolvers/engine/engine.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/api/result.hpp"
#include "mipsolvers/engine/solver/native/native_adapters.hpp"
#include "mipsolvers/scuc/case_builder.hpp"

namespace mipsolvers::scuc {

using json = nlohmann::json;
using Clock = std::chrono::steady_clock;

namespace {

constexpr double kEps = 1e-9;

engine::BCOptions make_scuc_bc_options(const SCUCConfig& cfg) {
  engine::BCOptions opt;
  opt.time_limit_sec = cfg.time_limit_sec;
  opt.gap_tol = cfg.mip_gap;
  opt.verbose = cfg.verbose;
  return engine::make_strict_highs_production_options(opt);
}

inline double clampd(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

// ─────────────────────────────────────────────────────────────────────────────
// Variable index — flat variable layout for SCUC MILP
// ─────────────────────────────────────────────────────────────────────────────
struct VarIndex {
  int ng{0}, nb{0}, nl{0}, nw{0}, npv{0}, nstorage{0}, ndc{0}, nsec{0};
  int T{24}, T_commit{24}, n_segments{3}, intervals_per_hour{1};
  bool use_sto_binaries{false};  ///< xi+/xi- binary vars for storage

  // Block start/end indices
  int SU_start{0}, SU_end{0};       // startup binary [ng × T_commit]
  int SD_start{0}, SD_end{0};       // shutdown binary [ng × T_commit]
  int IG_start{0}, IG_end{0};       // commitment binary [ng × T_commit]

  std::vector<int> SEG_start, SEG_end;  // dispatch segment [ng × T] each
  int PG_start{0}, PG_end{0};       // total dispatch [ng × T]
  int RG_spin_start{0}, RG_spin_end{0};
  int RG_reg_up_start{0}, RG_reg_up_end{0};
  int RG_reg_down_start{0}, RG_reg_down_end{0};

  int PF_start{0}, PF_end{0};
  int PW_start{0}, PW_end{0};       // wind dispatch [nw × T]
  int PPV_start{0}, PPV_end{0};     // solar dispatch [npv × T]
  int WIND_CURT_start{0}, WIND_CURT_end{0};  // wind curtailment [nw × T]
  int SOL_CURT_start{0},  SOL_CURT_end{0};   // solar curtailment [npv × T]
  int PSTO_IN_start{0},  PSTO_IN_end{0};
  int PSTO_OUT_start{0}, PSTO_OUT_end{0};
  int SOC_start{0}, SOC_end{0};
  // Binary storage mode indicators (only allocated when use_sto_binaries=true)
  int XI_CH_start{0},  XI_CH_end{0};   // xi+ [nstorage × T]
  int XI_DIS_start{0}, XI_DIS_end{0};  // xi- [nstorage × T]
  int PDC_start{0}, PDC_end{0};

  int n_areas{1};
  int LOAD_SHED_start{0}, LOAD_SHED_end{0};  // [n_areas × T]
  int GEN_CURT_start{0},  GEN_CURT_end{0};   // [n_areas × T]

  int SL_LINE_POS_start{0}, SL_LINE_POS_end{0};
  int SL_LINE_NEG_start{0}, SL_LINE_NEG_end{0};
  int SL_SEC_POS_start{0},  SL_SEC_POS_end{0};
  int SL_SEC_NEG_start{0},  SL_SEC_NEG_end{0};

  int nx{0};

  int idx(int base, int unit, int period, [[maybe_unused]] int n_units) const {
    return base + unit * T + period;
  }
  int idx_commit(int base, int unit, int period) const {
    return base + unit * T_commit + period;
  }
};

VarIndex setup_var_index(const SCUCInput& inp) {
  VarIndex v;
  const auto& cfg = inp.config;
  v.ng = static_cast<int>(inp.generators.size());
  v.nb = inp.num_buses;
  v.nl = static_cast<int>(inp.branches.size());
  v.nw = static_cast<int>(inp.wind.size());
  v.npv = static_cast<int>(inp.solar.size());
  v.nstorage = static_cast<int>(inp.storage.size());
  v.ndc = static_cast<int>(inp.dc_lines.size());
  v.nsec = static_cast<int>(inp.sections.size());
  v.T = cfg.num_periods;
  v.n_segments = std::max(1, cfg.n_segments);
  v.intervals_per_hour = std::max(1, static_cast<int>(std::llround(1.0 / std::max(1e-6, cfg.period_length_hr))));
  v.T_commit = std::max(1, v.T / v.intervals_per_hour);

  // Use binary storage indicators if any unit requests them
  for (const auto& s : inp.storage)
    if (s.use_binary_indicators) { v.use_sto_binaries = true; break; }

  int off = 0;
  auto alloc = [&](int n, int& s, int& e) { s = off; e = off + n; off = e; };

  alloc(v.ng * v.T_commit, v.SU_start, v.SU_end);
  alloc(v.ng * v.T_commit, v.SD_start, v.SD_end);
  alloc(v.ng * v.T_commit, v.IG_start, v.IG_end);

  v.SEG_start.resize(static_cast<size_t>(v.n_segments));
  v.SEG_end.resize(static_cast<size_t>(v.n_segments));
  for (int k = 0; k < v.n_segments; ++k)
    alloc(v.ng * v.T, v.SEG_start[static_cast<size_t>(k)], v.SEG_end[static_cast<size_t>(k)]);

  alloc(v.ng * v.T, v.PG_start, v.PG_end);
  alloc(v.ng * v.T, v.RG_spin_start, v.RG_spin_end);
  alloc(v.ng * v.T, v.RG_reg_up_start, v.RG_reg_up_end);
  alloc(v.ng * v.T, v.RG_reg_down_start, v.RG_reg_down_end);
  alloc(v.nl * v.T, v.PF_start, v.PF_end);
  alloc(v.nw  * v.T, v.PW_start,  v.PW_end);
  alloc(v.npv * v.T, v.PPV_start, v.PPV_end);
  // Renewable curtailment variables (only allocated if M2 > 0)
  if (cfg.M2_renewable_curtail_penalty > kEps) {
    alloc(v.nw  * v.T, v.WIND_CURT_start, v.WIND_CURT_end);
    alloc(v.npv * v.T, v.SOL_CURT_start,  v.SOL_CURT_end);
  } else {
    alloc(0, v.WIND_CURT_start, v.WIND_CURT_end);
    alloc(0, v.SOL_CURT_start,  v.SOL_CURT_end);
  }
  alloc(v.nstorage * v.T, v.PSTO_IN_start, v.PSTO_IN_end);
  alloc(v.nstorage * v.T, v.PSTO_OUT_start, v.PSTO_OUT_end);
  alloc(v.nstorage * v.T, v.SOC_start, v.SOC_end);
  if (v.use_sto_binaries) {
    alloc(v.nstorage * v.T, v.XI_CH_start,  v.XI_CH_end);
    alloc(v.nstorage * v.T, v.XI_DIS_start, v.XI_DIS_end);
  } else {
    alloc(0, v.XI_CH_start,  v.XI_CH_end);
    alloc(0, v.XI_DIS_start, v.XI_DIS_end);
  }
  alloc(v.ndc * v.T, v.PDC_start, v.PDC_end);

  v.n_areas = 1;
  alloc(v.n_areas * v.T, v.LOAD_SHED_start, v.LOAD_SHED_end);
  alloc(v.n_areas * v.T, v.GEN_CURT_start,  v.GEN_CURT_end);

  alloc(v.nl   * v.T, v.SL_LINE_POS_start, v.SL_LINE_POS_end);
  alloc(v.nl   * v.T, v.SL_LINE_NEG_start, v.SL_LINE_NEG_end);
  alloc(v.nsec * v.T, v.SL_SEC_POS_start,  v.SL_SEC_POS_end);
  alloc(v.nsec * v.T, v.SL_SEC_NEG_start,  v.SL_SEC_NEG_end);

  v.nx = off;
  return v;
}

// ─────────────────────────────────────────────────────────────────────────────
// PTDF computation (DC approximation)
// ─────────────────────────────────────────────────────────────────────────────
Eigen::MatrixXd compute_ptdf(const SCUCInput& inp) {
  const int nl = static_cast<int>(inp.branches.size());
  const int nb = inp.num_buses;

  if (nl == 0 || nb <= 1) return Eigen::MatrixXd();

  Eigen::MatrixXd Bbus = Eigen::MatrixXd::Zero(nb, nb);
  for (const auto& br : inp.branches) {
    if (!br.in_service) continue;
    const int f = br.from, t = br.to;
    if (f < 0 || t < 0 || f >= nb || t >= nb) continue;
    const double b = 1.0 / (std::abs(br.reactance) > 1e-6 ? br.reactance : 1e-3);
    Bbus(f, f) += b; Bbus(t, t) += b; Bbus(f, t) -= b; Bbus(t, f) -= b;
  }

  constexpr int slack = 0;
  std::vector<int> keep;
  keep.reserve(static_cast<size_t>(nb - 1));
  for (int i = 0; i < nb; ++i) if (i != slack) keep.push_back(i);

  const int nr = static_cast<int>(keep.size());
  Eigen::MatrixXd Bred(nr, nr);
  for (int r = 0; r < nr; ++r)
    for (int c = 0; c < nr; ++c)
      Bred(r, c) = Bbus(keep[r], keep[c]);

  Eigen::MatrixXd Bred_inv = Eigen::MatrixXd::Zero(nr, nr);
  if (nr > 0) {
    Eigen::FullPivLU<Eigen::MatrixXd> lu(Bred);
    if (lu.isInvertible()) Bred_inv = lu.inverse();
  }

  auto red_idx = [&](int bus) -> int {
    if (bus == slack) return -1;
    auto it = std::find(keep.begin(), keep.end(), bus);
    return it == keep.end() ? -1 : static_cast<int>(std::distance(keep.begin(), it));
  };

  Eigen::MatrixXd PTDF = Eigen::MatrixXd::Zero(nl, nb);
  for (int l = 0; l < nl; ++l) {
    const auto& br = inp.branches[static_cast<size_t>(l)];
    const double b = 1.0 / (std::abs(br.reactance) > 1e-6 ? br.reactance : 1e-3);
    const int rf = red_idx(br.from), rt = red_idx(br.to);
    for (int n = 0; n < nb; ++n) {
      const int rn = red_idx(n);
      if (rn < 0) { PTDF(l, n) = 0.0; continue; }
      PTDF(l, n) = b * (((rf >= 0) ? Bred_inv(rf, rn) : 0.0) -
                        ((rt >= 0) ? Bred_inv(rt, rn) : 0.0));
    }
  }
  return PTDF;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers to get profile values
// ─────────────────────────────────────────────────────────────────────────────
double load_at(const SCUCInput& inp, int d, int t) {
  const double base = inp.loads[static_cast<size_t>(d)].p_mw;
  if (d < static_cast<int>(inp.profiles.load.size()) &&
      t < static_cast<int>(inp.profiles.load[static_cast<size_t>(d)].size()))
    return base * inp.profiles.load[static_cast<size_t>(d)][static_cast<size_t>(t)];
  return base;
}

double wind_at(const SCUCInput& inp, int w, int t) {
  if (w < static_cast<int>(inp.profiles.wind.size()) &&
      t < static_cast<int>(inp.profiles.wind[static_cast<size_t>(w)].size()))
    return inp.profiles.wind[static_cast<size_t>(w)][static_cast<size_t>(t)];
  return inp.wind[static_cast<size_t>(w)].pmax;
}

double solar_at(const SCUCInput& inp, int s, int t) {
  if (s < static_cast<int>(inp.profiles.solar.size()) &&
      t < static_cast<int>(inp.profiles.solar[static_cast<size_t>(s)].size()))
    return inp.profiles.solar[static_cast<size_t>(s)][static_cast<size_t>(t)];
  return inp.solar[static_cast<size_t>(s)].pmax;
}

bool scuc_primal_repair_enabled(const SCUCConfig& cfg) {
  if (!cfg.enable_primal_repair) return false;
  return std::getenv("MIPSOLVERS_DISABLE_SCUC_PRIMAL_REPAIR") == nullptr;
}

double scuc_thermal_target_mw(const SCUCInput& inp, int dispatch_period) {
  const auto& cfg = inp.config;
  double load = 0.0;
  for (int load_idx = 0; load_idx < static_cast<int>(inp.loads.size()); ++load_idx)
    load += load_at(inp, load_idx, dispatch_period);

  double renewable = 0.0;
  for (int wind_idx = 0; wind_idx < static_cast<int>(inp.wind.size()); ++wind_idx)
    renewable += std::max(0.0, wind_at(inp, wind_idx, dispatch_period));
  for (int solar_idx = 0; solar_idx < static_cast<int>(inp.solar.size()); ++solar_idx)
    renewable += std::max(0.0, solar_at(inp, solar_idx, dispatch_period));

  const double upward_reserve =
      (cfg.spinning_reserve_req + cfg.regulation_up_req) * load;
  const double net_load = std::max(0.0, load - renewable);
  return net_load + upward_reserve;
}

double scuc_segment_capacity(const Generator& gen, int n_segments) {
  double capacity = 0.0;
  for (int seg = 0; seg < n_segments; ++seg) {
    if (seg < static_cast<int>(gen.bid_segments.size()))
      capacity += std::max(0.0, gen.bid_segments[static_cast<size_t>(seg)].quantity);
  }
  return capacity;
}

bool scuc_seed_satisfies(const engine::LPModel& lp, const Eigen::VectorXd& seed,
                         double tol, double* max_violation_out = nullptr) {
  if (seed.size() != static_cast<int>(lp.vars.size())) {
    if (max_violation_out != nullptr) *max_violation_out = std::numeric_limits<double>::infinity();
    return false;
  }
  double max_violation = 0.0;
  for (int col = 0; col < seed.size(); ++col) {
    const auto& var = lp.vars[static_cast<size_t>(col)];
    max_violation = std::max(max_violation, var.lb - seed[col]);
    max_violation = std::max(max_violation, seed[col] - var.ub);
    if (!std::isfinite(seed[col])) max_violation = std::numeric_limits<double>::infinity();
  }
  if (lp.A.rows() > 0) {
    const Eigen::VectorXd act = lp.A * seed;
    for (int row = 0; row < act.size(); ++row) {
      max_violation = std::max(max_violation, act[row] - lp.b[row]);
    }
  }
  if (lp.Aeq.rows() > 0) {
    const Eigen::VectorXd act = lp.Aeq * seed;
    for (int row = 0; row < act.size(); ++row) {
      max_violation = std::max(max_violation, std::abs(act[row] - lp.beq[row]));
    }
  }
  if (max_violation_out != nullptr) *max_violation_out = max_violation;
  return max_violation <= tol;
}

Eigen::VectorXd scuc_initial_seed_vector(const VarIndex& v,
                                         const engine::LPModel& lp) {
  Eigen::VectorXd seed(v.nx);
  for (int col = 0; col < v.nx; ++col) {
    const auto& var = lp.vars[static_cast<size_t>(col)];
    double value = std::isfinite(var.lb) ? var.lb : 0.0;
    value = std::max(var.lb, std::min(var.ub, value));
    seed[col] = std::isfinite(value) ? value : 0.0;
  }
  return seed;
}

void scuc_fill_segments_for_dispatch(const SCUCInput& inp, const VarIndex& v,
                                     int gen_idx, int dispatch_period,
                                     Eigen::VectorXd& seed) {
  const auto& gen = inp.generators[static_cast<size_t>(gen_idx)];
  const int commit_period = dispatch_period / v.intervals_per_hour;
  const double committed = seed[v.IG_start + gen_idx * v.T_commit + commit_period];
  double remaining = std::max(0.0, seed[v.PG_start + gen_idx * v.T + dispatch_period] -
                                       std::max(0.0, gen.pmin) * committed);
  for (int seg = 0; seg < v.n_segments; ++seg) {
    const int col = v.SEG_start[static_cast<size_t>(seg)] + gen_idx * v.T + dispatch_period;
    double cap = 0.0;
    if (seg < static_cast<int>(gen.bid_segments.size()))
      cap = std::max(0.0, gen.bid_segments[static_cast<size_t>(seg)].quantity);
    const double value = std::min(cap, remaining);
    seed[col] = value;
    remaining -= value;
  }
}

void scuc_repair_network_slacks(const VarIndex& v, const engine::LPModel& lp,
                                Eigen::VectorXd& seed) {
  auto is_network_slack = [&](int col) {
    return (col >= v.SL_LINE_POS_start && col < v.SL_LINE_POS_end) ||
           (col >= v.SL_LINE_NEG_start && col < v.SL_LINE_NEG_end) ||
           (col >= v.SL_SEC_POS_start && col < v.SL_SEC_POS_end) ||
           (col >= v.SL_SEC_NEG_start && col < v.SL_SEC_NEG_end);
  };
  if (lp.A.rows() == 0) return;
  Eigen::SparseMatrix<double, Eigen::RowMajor> row_major = lp.A;
  for (int pass = 0; pass < 2; ++pass) {
    const Eigen::VectorXd activity = lp.A * seed;
    bool changed = false;
    for (int row = 0; row < row_major.rows(); ++row) {
      double violation = activity[row] - lp.b[row];
      if (violation <= 1e-7) continue;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(row_major, row); it; ++it) {
        const int col = static_cast<int>(it.col());
        const double coeff = it.value();
        if (!is_network_slack(col) || !(coeff < -kEps)) continue;
        const double room = lp.vars[static_cast<size_t>(col)].ub - seed[col];
        if (room <= kEps) continue;
        const double delta = std::min(room, violation / (-coeff));
        seed[col] += delta;
        violation -= (-coeff) * delta;
        changed = true;
        break;
      }
    }
    if (!changed) break;
  }
}

Eigen::VectorXd recover_scuc_continuous_seed(
    const SCUCInput& inp, const VarIndex& v, const engine::LPModel& lp,
    const std::vector<std::vector<int>>& online) {
  const auto& cfg = inp.config;
  const int T = cfg.num_periods;
  const double dt = cfg.period_length_hr;
  const bool has_ren_curt = cfg.M2_renewable_curtail_penalty > kEps;
  Eigen::VectorXd seed = scuc_initial_seed_vector(v, lp);

  for (int gen_idx = 0; gen_idx < v.ng; ++gen_idx) {
    int previous =
        (gen_idx < static_cast<int>(inp.initial_status.commitment.size()) &&
         inp.initial_status.commitment[static_cast<size_t>(gen_idx)] > 0.5) ? 1 : 0;
    for (int commit_period = 0; commit_period < v.T_commit; ++commit_period) {
      const int now = online[static_cast<size_t>(gen_idx)][static_cast<size_t>(commit_period)];
      seed[v.IG_start + gen_idx * v.T_commit + commit_period] = static_cast<double>(now);
      seed[v.SU_start + gen_idx * v.T_commit + commit_period] =
          (previous == 0 && now == 1) ? 1.0 : 0.0;
      seed[v.SD_start + gen_idx * v.T_commit + commit_period] =
          (previous == 1 && now == 0) ? 1.0 : 0.0;
      previous = now;
    }
  }

  for (int period = 0; period < T; ++period) {
    for (int wind_idx = 0; wind_idx < v.nw; ++wind_idx) {
      const double forecast = std::max(0.0, wind_at(inp, wind_idx, period));
      const int pw_col = v.PW_start + wind_idx * T + period;
      seed[pw_col] = std::max(lp.vars[static_cast<size_t>(pw_col)].lb,
                              std::min(lp.vars[static_cast<size_t>(pw_col)].ub, forecast));
      if (has_ren_curt && v.WIND_CURT_start < v.WIND_CURT_end) {
        seed[v.WIND_CURT_start + wind_idx * T + period] =
            std::max(0.0, forecast - seed[pw_col]);
      }
    }
    for (int solar_idx = 0; solar_idx < v.npv; ++solar_idx) {
      const double forecast = std::max(0.0, solar_at(inp, solar_idx, period));
      const int pv_col = v.PPV_start + solar_idx * T + period;
      seed[pv_col] = std::max(lp.vars[static_cast<size_t>(pv_col)].lb,
                              std::min(lp.vars[static_cast<size_t>(pv_col)].ub, forecast));
      if (has_ren_curt && v.SOL_CURT_start < v.SOL_CURT_end) {
        seed[v.SOL_CURT_start + solar_idx * T + period] =
            std::max(0.0, forecast - seed[pv_col]);
      }
    }
  }

  for (int storage_idx = 0; storage_idx < v.nstorage; ++storage_idx) {
    const auto& sto = inp.storage[static_cast<size_t>(storage_idx)];
    const double e_cap = std::max(0.0, sto.energy_capacity_mwh);
    const double eta_rt = clampd(sto.efficiency, 0.1, 1.0);
    const double eta_sq = std::sqrt(eta_rt);
    const double eta_ch = (sto.eta_charge > 0) ? clampd(sto.eta_charge, 0.01, 1.0) : eta_sq;
    const double eta_dis = (sto.eta_discharge > 0) ? clampd(sto.eta_discharge, 0.01, 1.0) : eta_sq;
    const double soc_lo = (sto.soc_min >= 0) ? clampd(sto.soc_min, 0.0, 1.0) * e_cap : 0.1 * e_cap;
    double energy = sto.soc_init * e_cap;
    if (storage_idx < static_cast<int>(inp.initial_status.storage_soc.size())) {
      const double initial = std::max(0.0, inp.initial_status.storage_soc[static_cast<size_t>(storage_idx)]);
      energy = (initial <= 1.0 + 1e-8) ? initial * e_cap : initial;
    }
    energy = clampd(energy, 0.0, e_cap);
    const double soc_fin = (sto.soc_final >= 0) ? clampd(sto.soc_final, 0.0, 1.0) * e_cap : energy;
    for (int period = 0; period < T; ++period) {
      const int p_in = v.PSTO_IN_start + storage_idx * T + period;
      const int p_out = v.PSTO_OUT_start + storage_idx * T + period;
      const int soc = v.SOC_start + storage_idx * T + period;
      const double period_target = (period == T - 1) ? std::max(soc_lo, soc_fin) : soc_lo;
      double charge = 0.0;
      if (energy + 1e-8 < period_target && eta_ch > kEps && dt > kEps) {
        charge = (period_target - energy) / (eta_ch * dt);
        if (sto.use_binary_indicators && sto.pmin_charge > kEps) charge = std::max(charge, sto.pmin_charge);
        charge = std::min(std::max(0.0, sto.pmax_charge), charge);
      }
      seed[p_in] = charge;
      seed[p_out] = 0.0;
      if (v.use_sto_binaries && sto.use_binary_indicators) {
        seed[v.XI_CH_start + storage_idx * T + period] = charge > kEps ? 1.0 : 0.0;
        seed[v.XI_DIS_start + storage_idx * T + period] = 0.0;
      }
      energy = clampd(energy + eta_ch * charge * dt, 0.0, e_cap);
      (void)eta_dis;
      seed[soc] = energy;
    }
  }

  for (int dc_idx = 0; dc_idx < v.ndc; ++dc_idx) {
    for (int period = 0; period < T; ++period) {
      const int col = v.PDC_start + dc_idx * T + period;
      seed[col] = std::max(lp.vars[static_cast<size_t>(col)].lb,
                           std::min(lp.vars[static_cast<size_t>(col)].ub, 0.0));
    }
  }

  struct DispatchUnit {
    int gen_idx{0};
    double score{0.0};
    double pmin{0.0};
    double pmax{0.0};
    double ramp_up{0.0};
    double ramp_down{0.0};
    double lower{0.0};
    double upper{0.0};
  };
  std::vector<double> prev_pg(static_cast<size_t>(v.ng), 0.0);
  for (int gen_idx = 0; gen_idx < v.ng; ++gen_idx) {
    if (gen_idx < static_cast<int>(inp.initial_status.dispatch.size()))
      prev_pg[static_cast<size_t>(gen_idx)] = std::max(0.0, inp.initial_status.dispatch[static_cast<size_t>(gen_idx)]);
  }

  for (int period = 0; period < T; ++period) {
    const int commit_period = period / v.intervals_per_hour;
    double load = 0.0;
    for (int load_idx = 0; load_idx < static_cast<int>(inp.loads.size()); ++load_idx)
      load += load_at(inp, load_idx, period);
    const double spin_req = cfg.spinning_reserve_req * load;
    const double reg_up_req = cfg.regulation_up_req * load;
    const double reg_down_req = cfg.regulation_down_req * load;

    std::vector<DispatchUnit> units;
    units.reserve(static_cast<size_t>(v.ng));
    for (int gen_idx = 0; gen_idx < v.ng; ++gen_idx) {
      const auto& gen = inp.generators[static_cast<size_t>(gen_idx)];
      const int online_now = online[static_cast<size_t>(gen_idx)][static_cast<size_t>(commit_period)];
      const double committed = static_cast<double>(online_now);
      const double pmin = std::max(0.0, gen.pmin) * committed;
      const double seg_cap = scuc_segment_capacity(gen, v.n_segments);
      const double dispatch_max = committed * std::min(std::max(0.0, gen.pmax), std::max(0.0, gen.pmin) + seg_cap);
      const double pmax = committed * std::max(0.0, gen.pmax);
      const double ramp_up = std::max(0.0, gen.ramp_up_mw_min * 60.0 * dt);
      const double ramp_down = std::max(0.0, gen.ramp_dn_mw_min * 60.0 * dt);
      const double su = seed[v.SU_start + gen_idx * v.T_commit + commit_period];
      const double sd = seed[v.SD_start + gen_idx * v.T_commit + commit_period];
      const double prev_commit = period == 0
          ? ((gen_idx < static_cast<int>(inp.initial_status.commitment.size()) &&
              inp.initial_status.commitment[static_cast<size_t>(gen_idx)] > 0.5) ? 1.0 : 0.0)
          : seed[v.IG_start + gen_idx * v.T_commit + ((period - 1) / v.intervals_per_hour)];
      double lower = pmin;
      double upper = dispatch_max;
      upper = std::min(upper, prev_pg[static_cast<size_t>(gen_idx)] + ramp_up + std::max(0.0, gen.pmax) * su);
      lower = std::max(lower, prev_pg[static_cast<size_t>(gen_idx)] - ramp_down - std::max(0.0, gen.pmax) * sd);
      if (period > 0) {
        upper = std::min(upper, prev_pg[static_cast<size_t>(gen_idx)] +
                                    std::min(std::max(0.0, gen.pmax), ramp_up) * prev_commit +
                                    std::max(0.0, gen.pmax) * su);
        lower = std::max(lower, prev_pg[static_cast<size_t>(gen_idx)] -
                                    std::min(std::max(0.0, gen.pmax), ramp_down) * committed -
                                    std::max(0.0, gen.pmax) * sd);
      }
      lower = std::max(0.0, lower);
      upper = std::max(lower, upper);
      double marginal = 0.0;
      if (!gen.bid_segments.empty()) marginal = gen.bid_segments.front().price;
      const double score = marginal + gen.no_load_cost / std::max(1.0, std::max(0.0, gen.pmax));
      units.push_back({gen_idx, score, pmin, pmax, ramp_up, ramp_down, lower, upper});
      seed[v.PG_start + gen_idx * T + period] = lower;
    }
    std::sort(units.begin(), units.end(), [](const DispatchUnit& lhs, const DispatchUnit& rhs) {
      if (lhs.score != rhs.score) return lhs.score < rhs.score;
      return lhs.pmax > rhs.pmax;
    });

    double down_available = 0.0;
    for (const DispatchUnit& unit : units) {
      const double pg = seed[v.PG_start + unit.gen_idx * T + period];
      const auto& gen = inp.generators[static_cast<size_t>(unit.gen_idx)];
      const double down_cap = gen.ramp_dn_mw_min > kEps ? gen.ramp_dn_mw_min * 10.0 : std::numeric_limits<double>::infinity();
      down_available += std::min(std::max(0.0, pg - unit.pmin), down_cap);
    }
    double down_deficit = std::max(0.0, reg_down_req - down_available);
    for (const DispatchUnit& unit : units) {
      if (down_deficit <= kEps) break;
      const auto& gen = inp.generators[static_cast<size_t>(unit.gen_idx)];
      const int pg_col = v.PG_start + unit.gen_idx * T + period;
      const double down_cap = gen.ramp_dn_mw_min > kEps ? gen.ramp_dn_mw_min * 10.0 : unit.upper - unit.pmin;
      const double current_down = std::min(std::max(0.0, seed[pg_col] - unit.pmin), down_cap);
      const double useful_room = std::max(0.0, std::min(unit.upper - seed[pg_col], down_cap - current_down));
      const double delta = std::min(useful_room, down_deficit);
      seed[pg_col] += delta;
      down_deficit -= delta;
    }

    for (int gen_idx = 0; gen_idx < v.ng; ++gen_idx)
      scuc_fill_segments_for_dispatch(inp, v, gen_idx, period, seed);

    double reg_down_remaining = reg_down_req;
    double reg_up_remaining = reg_up_req;
    double spin_remaining = spin_req;
    for (const DispatchUnit& unit : units) {
      const auto& gen = inp.generators[static_cast<size_t>(unit.gen_idx)];
      const double pg = seed[v.PG_start + unit.gen_idx * T + period];
      const int rd_col = v.RG_reg_down_start + unit.gen_idx * T + period;
      const int ru_col = v.RG_reg_up_start + unit.gen_idx * T + period;
      const int rs_col = v.RG_spin_start + unit.gen_idx * T + period;
      const double down_cap = gen.ramp_dn_mw_min > kEps ? gen.ramp_dn_mw_min * 10.0 : std::numeric_limits<double>::infinity();
      const double rd = std::min({reg_down_remaining, std::max(0.0, pg - unit.pmin), down_cap});
      seed[rd_col] = rd;
      reg_down_remaining -= rd;

      double headroom = std::max(0.0, unit.pmax - pg);
      const double up_cap = gen.ramp_up_mw_min > kEps ? gen.ramp_up_mw_min * 10.0 : std::numeric_limits<double>::infinity();
      const double ru = std::min({reg_up_remaining, headroom, up_cap});
      seed[ru_col] = ru;
      reg_up_remaining -= ru;
      headroom -= ru;
      const double spin = std::min({spin_remaining, headroom, std::max(0.0, unit.pmax - unit.pmin)});
      seed[rs_col] = spin;
      spin_remaining -= spin;
    }

    double supply = 0.0;
    for (int gen_idx = 0; gen_idx < v.ng; ++gen_idx) supply += seed[v.PG_start + gen_idx * T + period];
    for (int wind_idx = 0; wind_idx < v.nw; ++wind_idx) supply += seed[v.PW_start + wind_idx * T + period];
    for (int solar_idx = 0; solar_idx < v.npv; ++solar_idx) supply += seed[v.PPV_start + solar_idx * T + period];
    for (int storage_idx = 0; storage_idx < v.nstorage; ++storage_idx) {
      supply += seed[v.PSTO_OUT_start + storage_idx * T + period];
      supply -= seed[v.PSTO_IN_start + storage_idx * T + period];
    }
    for (int dc_idx = 0; dc_idx < v.ndc; ++dc_idx) supply += seed[v.PDC_start + dc_idx * T + period];
    const double imbalance = load - supply;
    seed[v.LOAD_SHED_start + period] = std::max(0.0, imbalance);
    seed[v.GEN_CURT_start + period] = std::max(0.0, -imbalance);

    for (const DispatchUnit& unit : units)
      prev_pg[static_cast<size_t>(unit.gen_idx)] = seed[v.PG_start + unit.gen_idx * T + period];
  }

  scuc_repair_network_slacks(v, lp, seed);
  for (int col = 0; col < v.nx; ++col) {
    const auto& var = lp.vars[static_cast<size_t>(col)];
    seed[col] = std::max(var.lb, std::min(var.ub, seed[col]));
  }
  return seed;
}

Eigen::VectorXd build_scuc_primal_repair_seed(const SCUCInput& inp,
                                              const VarIndex& v,
                                              const engine::LPModel& lp) {
  if (!scuc_primal_repair_enabled(inp.config) || v.ng <= 0 || v.T_commit <= 0 ||
      static_cast<int>(lp.vars.size()) != v.nx) {
    return Eigen::VectorXd();
  }

  struct SeedUnit {
    int gen_idx{0};
    double score{0.0};
    double pmax{0.0};
  };

  std::vector<SeedUnit> merit;
  merit.reserve(static_cast<size_t>(v.ng));
  for (int gen_idx = 0; gen_idx < v.ng; ++gen_idx) {
    const auto& gen = inp.generators[static_cast<size_t>(gen_idx)];
    const double pmax = std::max(0.0, gen.pmax);
    if (pmax <= kEps) continue;
    double marginal = 0.0;
    if (!gen.bid_segments.empty()) {
      marginal = gen.bid_segments.front().price;
      for (const auto& segment : gen.bid_segments) {
        if (segment.quantity > kEps) {
          marginal = segment.price;
          break;
        }
      }
    }
    const double startup_adder = gen.startup_cost /
        std::max(1.0, std::ceil(std::max(1.0, gen.min_up_time_hr)));
    const double score = (gen.must_run ? -1.0e12 : 0.0) +
                         marginal + (gen.no_load_cost + startup_adder) /
                                        std::max(1.0, pmax);
    merit.push_back({gen_idx, score, pmax});
  }
  if (merit.empty()) return Eigen::VectorXd();

  std::sort(merit.begin(), merit.end(),
            [](const SeedUnit& lhs, const SeedUnit& rhs) {
              if (lhs.score != rhs.score) return lhs.score < rhs.score;
              return lhs.pmax > rhs.pmax;
            });
  std::vector<SeedUnit> expensive = merit;
  std::sort(expensive.begin(), expensive.end(),
            [](const SeedUnit& lhs, const SeedUnit& rhs) {
              if (lhs.score != rhs.score) return lhs.score > rhs.score;
              return lhs.pmax < rhs.pmax;
            });

  std::vector<std::vector<int>> online(static_cast<size_t>(v.ng),
                                       std::vector<int>(static_cast<size_t>(v.T_commit), 0));
  std::vector<int> state(static_cast<size_t>(v.ng), 0);
  std::vector<int> lock_on(static_cast<size_t>(v.ng), 0);
  std::vector<int> lock_off(static_cast<size_t>(v.ng), 0);
  for (int gen_idx = 0; gen_idx < v.ng; ++gen_idx) {
    const auto& gen = inp.generators[static_cast<size_t>(gen_idx)];
    const int min_up = std::max(0, static_cast<int>(std::ceil(gen.min_up_time_hr)));
    const int min_down = std::max(0, static_cast<int>(std::ceil(gen.min_dn_time_hr)));
    const bool initially_on =
        gen.must_run ||
        (gen_idx < static_cast<int>(inp.initial_status.commitment.size()) &&
         inp.initial_status.commitment[static_cast<size_t>(gen_idx)] > 0.5);
    state[static_cast<size_t>(gen_idx)] = initially_on ? 1 : 0;
    if (gen_idx < static_cast<int>(inp.initial_status.time_in_state.size())) {
      const double time_in_state = inp.initial_status.time_in_state[static_cast<size_t>(gen_idx)];
      if (initially_on && time_in_state > 0.0) {
        lock_on[static_cast<size_t>(gen_idx)] =
            std::max(0, min_up - static_cast<int>(std::floor(time_in_state)));
      } else if (!initially_on && time_in_state < 0.0) {
        lock_off[static_cast<size_t>(gen_idx)] =
            std::max(0, min_down - static_cast<int>(std::floor(-time_in_state)));
      }
    }
  }

  struct CommitmentCapability {
    double capacity{0.0};
    double up_headroom{0.0};
    double spinning{0.0};
    double reg_up{0.0};
    double reg_down{0.0};
  };

  auto reserve_capability = [](const Generator& gen) {
    CommitmentCapability cap;
    cap.capacity = std::max(0.0, gen.pmax);
    cap.up_headroom = std::max(0.0, gen.pmax - gen.pmin);
    cap.spinning = cap.up_headroom;
    const double r10u = std::max(0.0, gen.ramp_up_mw_min * 10.0);
    const double r10d = std::max(0.0, gen.ramp_dn_mw_min * 10.0);
    cap.reg_up = r10u > kEps ? std::min(cap.up_headroom, r10u) : cap.up_headroom;
    cap.reg_down = r10d > kEps ? std::min(cap.up_headroom, r10d) : cap.up_headroom;
    return cap;
  };

  auto commitment_capability = [&](const std::vector<int>& commitment) {
    CommitmentCapability cap;
    for (int gen_idx = 0; gen_idx < v.ng; ++gen_idx) {
      if (commitment[static_cast<size_t>(gen_idx)] == 0) continue;
      const CommitmentCapability unit =
          reserve_capability(inp.generators[static_cast<size_t>(gen_idx)]);
      cap.capacity += unit.capacity;
      cap.up_headroom += unit.up_headroom;
      cap.spinning += unit.spinning;
      cap.reg_up += unit.reg_up;
      cap.reg_down += unit.reg_down;
    }
    return cap;
  };

  for (int commit_period = 0; commit_period < v.T_commit; ++commit_period) {
    std::vector<int> current = state;
    for (int gen_idx = 0; gen_idx < v.ng; ++gen_idx) {
      const auto& gen = inp.generators[static_cast<size_t>(gen_idx)];
      if (gen.must_run) current[static_cast<size_t>(gen_idx)] = 1;
      if (lock_on[static_cast<size_t>(gen_idx)] > 0) current[static_cast<size_t>(gen_idx)] = 1;
      if (lock_off[static_cast<size_t>(gen_idx)] > 0 && !gen.must_run)
        current[static_cast<size_t>(gen_idx)] = 0;
    }

    double target = 0.0;
    double spin_req = 0.0;
    double rup_req = 0.0;
    double rdn_req = 0.0;
    const int first_t = commit_period * v.intervals_per_hour;
    const int last_t = std::min(v.T - 1, first_t + v.intervals_per_hour - 1);
    for (int dispatch_period = first_t; dispatch_period <= last_t; ++dispatch_period) {
      target = std::max(target, scuc_thermal_target_mw(inp, dispatch_period));
      double load_t = 0.0;
      for (int load_idx = 0; load_idx < static_cast<int>(inp.loads.size()); ++load_idx)
        load_t += load_at(inp, load_idx, dispatch_period);
      spin_req = std::max(spin_req, inp.config.spinning_reserve_req * load_t);
      rup_req = std::max(rup_req, inp.config.regulation_up_req * load_t);
      rdn_req = std::max(rdn_req, inp.config.regulation_down_req * load_t);
    }
    // Reserve capability is checked separately below; adding an extra capacity
    // margin here tends to keep one more expensive peaker online in large UC
    // cases without improving feasibility.

    CommitmentCapability capacity = commitment_capability(current);
    auto meets_requirements = [&]() {
      return capacity.capacity + kEps >= target &&
             capacity.up_headroom + kEps >= spin_req + rup_req &&
             capacity.spinning + kEps >= spin_req &&
             capacity.reg_up + kEps >= rup_req &&
             capacity.reg_down + kEps >= rdn_req;
    };
    auto try_start_units = [&](bool allow_locked_off) {
      for (const SeedUnit& unit : merit) {
        const int gen_idx = unit.gen_idx;
        if (current[static_cast<size_t>(gen_idx)] != 0) continue;
        if (!allow_locked_off && lock_off[static_cast<size_t>(gen_idx)] > 0) continue;
        current[static_cast<size_t>(gen_idx)] = 1;
        lock_off[static_cast<size_t>(gen_idx)] = 0;
        const CommitmentCapability added =
            reserve_capability(inp.generators[static_cast<size_t>(gen_idx)]);
        capacity.capacity += added.capacity;
        capacity.up_headroom += added.up_headroom;
        capacity.spinning += added.spinning;
        capacity.reg_up += added.reg_up;
        capacity.reg_down += added.reg_down;
        if (meets_requirements()) return true;
      }
      return meets_requirements();
    };
    if (!meets_requirements() && !try_start_units(false)) {
      (void)try_start_units(true);
    }

    for (const SeedUnit& unit : expensive) {
      const int gen_idx = unit.gen_idx;
      const auto& gen = inp.generators[static_cast<size_t>(gen_idx)];
      if (current[static_cast<size_t>(gen_idx)] == 0 || gen.must_run) continue;
      if (lock_on[static_cast<size_t>(gen_idx)] > 0) continue;
      const CommitmentCapability removed = reserve_capability(gen);
      capacity.capacity -= removed.capacity;
      capacity.up_headroom -= removed.up_headroom;
      capacity.spinning -= removed.spinning;
      capacity.reg_up -= removed.reg_up;
      capacity.reg_down -= removed.reg_down;
      if (!meets_requirements()) {
        capacity.capacity += removed.capacity;
        capacity.up_headroom += removed.up_headroom;
        capacity.spinning += removed.spinning;
        capacity.reg_up += removed.reg_up;
        capacity.reg_down += removed.reg_down;
        continue;
      }
      current[static_cast<size_t>(gen_idx)] = 0;
    }

    for (int gen_idx = 0; gen_idx < v.ng; ++gen_idx) {
      online[static_cast<size_t>(gen_idx)][static_cast<size_t>(commit_period)] =
          current[static_cast<size_t>(gen_idx)];
      const auto& gen = inp.generators[static_cast<size_t>(gen_idx)];
      const int min_up = std::max(0, static_cast<int>(std::ceil(gen.min_up_time_hr)));
      const int min_down = std::max(0, static_cast<int>(std::ceil(gen.min_dn_time_hr)));
      const int previous = state[static_cast<size_t>(gen_idx)];
      const int now = current[static_cast<size_t>(gen_idx)];
      if (previous == 0 && now == 1) {
        lock_on[static_cast<size_t>(gen_idx)] = std::max(0, min_up);
        lock_off[static_cast<size_t>(gen_idx)] = 0;
      } else if (previous == 1 && now == 0) {
        lock_off[static_cast<size_t>(gen_idx)] = std::max(0, min_down);
        lock_on[static_cast<size_t>(gen_idx)] = 0;
      }
    }

    for (int gen_idx = 0; gen_idx < v.ng; ++gen_idx) {
      if (current[static_cast<size_t>(gen_idx)] != 0) {
        if (lock_on[static_cast<size_t>(gen_idx)] > 0) --lock_on[static_cast<size_t>(gen_idx)];
      } else if (lock_off[static_cast<size_t>(gen_idx)] > 0) {
        --lock_off[static_cast<size_t>(gen_idx)];
      }
    }
    state = std::move(current);
  }

  double max_violation = std::numeric_limits<double>::infinity();
  Eigen::VectorXd seed = recover_scuc_continuous_seed(inp, v, lp, online);
  if (scuc_seed_satisfies(lp, seed, 1e-5, &max_violation)) return seed;

  std::vector<std::vector<int>> dense_online(static_cast<size_t>(v.ng),
                                             std::vector<int>(static_cast<size_t>(v.T_commit), 1));
  for (int gen_idx = 0; gen_idx < v.ng; ++gen_idx) {
    if (inp.generators[static_cast<size_t>(gen_idx)].must_run) continue;
    for (int commit_period = 0; commit_period < v.T_commit; ++commit_period) {
      dense_online[static_cast<size_t>(gen_idx)][static_cast<size_t>(commit_period)] = 1;
    }
  }
  Eigen::VectorXd dense_seed = recover_scuc_continuous_seed(inp, v, lp, dense_online);
  double dense_violation = std::numeric_limits<double>::infinity();
  if (scuc_seed_satisfies(lp, dense_seed, 1e-5, &dense_violation)) return dense_seed;

  if (std::getenv("MIPSOLVERS_SCUC_PRIMAL_REPAIR_LOG") != nullptr) {
    std::cerr << "[SCUC-REPAIR] seed remains infeasible: merit_violation="
              << max_violation << " dense_violation=" << dense_violation << "\n";
  }
  return max_violation <= dense_violation ? seed : dense_seed;
}

// ─────────────────────────────────────────────────────────────────────────────
// Formulation: builds MIPModel for SCUC (or SCED when binaries fixed)
// ─────────────────────────────────────────────────────────────────────────────
struct Formulation {
  engine::MIPModel mip;
  VarIndex v;
  Eigen::MatrixXd PTDF;       // [nl × nb]
  Eigen::MatrixXd SEC_PTDF;   // [nsec × nb] = weighted combination of PTDF rows
  int n_ineq_rows{0};
  int n_eq_rows{0};
  int power_balance_eq_start{0};
  std::vector<int> line_fwd_row;  // size = nl*T, -1 = skipped
  std::vector<int> line_rev_row;
  std::vector<int> sec_fwd_row;   // size = nsec*T
  std::vector<int> sec_rev_row;
  int n_cuts{0};
};

Formulation build_formulation(const SCUCInput& inp) {
  Formulation f;
  f.v = setup_var_index(inp);
  const VarIndex& v = f.v;
  const auto& cfg = inp.config;
  const int T = cfg.num_periods;
  const int iph = v.intervals_per_hour;
  const double dt = cfg.period_length_hr;

  auto& mip = f.mip;
  auto& lp = mip.linear_part;
  lp.sense = engine::Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(v.nx);
  lp.vars.resize(static_cast<size_t>(v.nx));
  for (int i = 0; i < v.nx; ++i) {
    lp.vars[static_cast<size_t>(i)].type = engine::VarType::Continuous;
    lp.vars[static_cast<size_t>(i)].lb = 0.0;
    lp.vars[static_cast<size_t>(i)].ub = 1e20;
  }

  // ── Binary variables ─────────────────────────────────────────────────────
  for (int i = v.SU_start; i < v.SU_end; ++i) {
    lp.vars[static_cast<size_t>(i)] = {engine::VarType::Binary, 0.0, 1.0, {}};
    mip.binary_idx.push_back(i);
  }
  for (int i = v.SD_start; i < v.SD_end; ++i) {
    lp.vars[static_cast<size_t>(i)] = {engine::VarType::Binary, 0.0, 1.0, {}};
    mip.binary_idx.push_back(i);
  }
  for (int i = v.IG_start; i < v.IG_end; ++i) {
    lp.vars[static_cast<size_t>(i)] = {engine::VarType::Binary, 0.0, 1.0, {}};
    mip.binary_idx.push_back(i);
  }
  if (v.use_sto_binaries) {
    for (int i = v.XI_CH_start; i < v.XI_CH_end; ++i) {
      lp.vars[static_cast<size_t>(i)] = {engine::VarType::Binary, 0.0, 1.0, {}};
      mip.binary_idx.push_back(i);
    }
    for (int i = v.XI_DIS_start; i < v.XI_DIS_end; ++i) {
      lp.vars[static_cast<size_t>(i)] = {engine::VarType::Binary, 0.0, 1.0, {}};
      mip.binary_idx.push_back(i);
    }
  }

  // ── Objective ────────────────────────────────────────────────────────────
  // §2.6.3 Objective: generation cost + startup/no-load + wheeling fee
  //                 + M1 slacks + M2 renewable curtailment + storage bids
  const double M1 = cfg.M1_line_slack_penalty;
  const double M2 = cfg.M2_renewable_curtail_penalty;
  const double wf = cfg.wheeling_fee_per_mwh;  // P_gwf wheeling fee (§ obj)

  for (int g = 0; g < v.ng; ++g) {
    const auto& gen = inp.generators[static_cast<size_t>(g)];
    const double wf_coeff = wf * dt;  // wheeling fee per MW per period
    for (int t = 0; t < T; ++t) {
      // Piecewise-linear bid cost (§2.6.3.7)
      for (int k = 0; k < v.n_segments; ++k) {
        const int idx = v.SEG_start[static_cast<size_t>(k)] + g * T + t;
        double price = 0.0;
        if (k < static_cast<int>(gen.bid_segments.size()))
          price = gen.bid_segments[static_cast<size_t>(k)].price;
        lp.c(idx) = price * dt;
      }
      // Wheeling fee on total generator output (§2.6.3 obj P_gwf term)
      if (wf > kEps)
        lp.c(v.PG_start + g * T + t) += wf_coeff;

      lp.c(v.RG_spin_start   + g * T + t) = gen.spinning_reserve_price * dt;
      lp.c(v.RG_reg_up_start + g * T + t) = gen.regulation_up_price    * dt;
      lp.c(v.RG_reg_down_start + g * T + t) = gen.regulation_down_price * dt;
    }
    // Startup cost (§2.6.3.13) and no-load cost (§2.6.3 min-output cost)
    for (int h = 0; h < v.T_commit; ++h) {
      lp.c(v.SU_start + g * v.T_commit + h) += gen.startup_cost;  // hot cost used as default
      lp.c(v.IG_start + g * v.T_commit + h) += gen.no_load_cost * (1.0 / iph);
    }
  }

  // LOAD_SHED / GEN_CURT penalty
  for (int t = 0; t < T; ++t) {
    lp.c(v.LOAD_SHED_start + t) = cfg.voll  * dt;
    lp.vars[static_cast<size_t>(v.LOAD_SHED_start + t)].ub = 1e6;
    lp.c(v.GEN_CURT_start + t) = cfg.vocc * dt;
    lp.vars[static_cast<size_t>(v.GEN_CURT_start + t)].ub = 1e6;
  }
  for (int i = v.SL_LINE_POS_start; i < v.SL_LINE_POS_end; ++i) { lp.c(i) = M1; lp.vars[static_cast<size_t>(i)].ub = 1e6; }
  for (int i = v.SL_LINE_NEG_start; i < v.SL_LINE_NEG_end; ++i) { lp.c(i) = M1; lp.vars[static_cast<size_t>(i)].ub = 1e6; }
  for (int i = v.SL_SEC_POS_start;  i < v.SL_SEC_POS_end;  ++i) { lp.c(i) = M1; lp.vars[static_cast<size_t>(i)].ub = 1e6; }
  for (int i = v.SL_SEC_NEG_start;  i < v.SL_SEC_NEG_end;  ++i) { lp.c(i) = M1; lp.vars[static_cast<size_t>(i)].ub = 1e6; }

  // Renewable curtailment penalty (§2.6.3 obj M2 term)
  if (M2 > kEps) {
    for (int i = v.WIND_CURT_start; i < v.WIND_CURT_end; ++i) { lp.c(i) = M2 * dt; lp.vars[static_cast<size_t>(i)].ub = 1e6; }
    for (int i = v.SOL_CURT_start;  i < v.SOL_CURT_end;  ++i) { lp.c(i) = M2 * dt; lp.vars[static_cast<size_t>(i)].ub = 1e6; }
  }

  // Storage bid price costs (§2.6.3 obj last term)
  for (int s = 0; s < v.nstorage; ++s) {
    const auto& sto = inp.storage[static_cast<size_t>(s)];
    for (int t = 0; t < T; ++t) {
      if (std::abs(sto.discharge_bid_price) > kEps)
        lp.c(v.PSTO_OUT_start + s * T + t) += sto.discharge_bid_price * dt;
      if (std::abs(sto.charge_bid_price) > kEps)
        lp.c(v.PSTO_IN_start  + s * T + t) += sto.charge_bid_price    * dt;
    }
  }

  // ── Build constraint matrices ─────────────────────────────────────────────
  std::vector<Eigen::Triplet<double>> Aeq_trip, A_trip;
  std::vector<double> beq_vec, b_vec;

  auto add_eq = [&](const std::vector<std::pair<int,double>>& terms, double rhs) {
    const int r = static_cast<int>(beq_vec.size());
    for (const auto& [c, cv] : terms)
      if (c >= 0 && c < v.nx && std::isfinite(cv) && std::abs(cv) > 0.0)
        Aeq_trip.emplace_back(r, c, cv);
    beq_vec.push_back(std::isfinite(rhs) ? rhs : 0.0);
  };

  auto add_le = [&](const std::vector<std::pair<int,double>>& terms, double rhs) {
    const int r = static_cast<int>(b_vec.size());
    for (const auto& [c, cv] : terms)
      if (c >= 0 && c < v.nx && std::isfinite(cv) && std::abs(cv) > 0.0)
        A_trip.emplace_back(r, c, cv);
    b_vec.push_back(std::isfinite(rhs) ? rhs : 0.0);
  };

  // ── PG = Pmin*IG + Σ segments (§2.6.3.6) ─────────────────────────────────
  for (int g = 0; g < v.ng; ++g) {
    const double pmin = inp.generators[static_cast<size_t>(g)].pmin;
    const double pmax = inp.generators[static_cast<size_t>(g)].pmax;
    for (int t = 0; t < T; ++t) {
      const int h = t / iph;
      const int pg = v.PG_start + g * T + t;
      const int ig = v.IG_start + g * v.T_commit + h;

      // PG = Pmin*IG + Σ SEG_k
      {
        std::vector<std::pair<int,double>> row{{pg, 1.0}};
        if (pmin > kEps) row.emplace_back(ig, -pmin);
        for (int k = 0; k < v.n_segments; ++k)
          row.emplace_back(v.SEG_start[static_cast<size_t>(k)] + g * T + t, -1.0);
        add_eq(row, 0.0);
      }

      // PG <= Pmax * IG  (§2.6.3.8)
      add_le({{pg, 1.0}, {ig, -pmax}}, 0.0);

      // PG >= Pmin * IG  (§2.6.3.8, lower coupling)
      if (pmin > kEps)
        add_le({{pg, -1.0}, {ig, pmin}}, 0.0);

      // Segment capacity upper bounds (§2.6.3.7)
      for (int k = 0; k < v.n_segments; ++k) {
        const int seg_idx = v.SEG_start[static_cast<size_t>(k)] + g * T + t;
        const auto& gen = inp.generators[static_cast<size_t>(g)];
        double q = 0.0;
        if (k < static_cast<int>(gen.bid_segments.size()))
          q = gen.bid_segments[static_cast<size_t>(k)].quantity;
        q = std::max(0.0, q);
        lp.vars[static_cast<size_t>(seg_idx)].ub = q;
        if (q > kEps) {
          add_le({{seg_idx, 1.0}, {ig, -q}}, 0.0);
        }
      }
    }
  }

  // ── Commitment transitions (§2.6.3.12) ───────────────────────────────────
  for (int g = 0; g < v.ng; ++g) {
    const auto& gen = inp.generators[static_cast<size_t>(g)];
    for (int h = 0; h < v.T_commit; ++h) {
      const int su = v.SU_start + g * v.T_commit + h;
      const int sd = v.SD_start + g * v.T_commit + h;
      const int ig = v.IG_start + g * v.T_commit + h;

      if (h == 0) {
        double ig0 = 0.0;
        if (g < static_cast<int>(inp.initial_status.commitment.size()))
          ig0 = clampd(inp.initial_status.commitment[static_cast<size_t>(g)], 0.0, 1.0);
        add_eq({{ig, 1.0}, {su, -1.0}, {sd, 1.0}}, ig0);
      } else {
        const int ig_prev = v.IG_start + g * v.T_commit + (h - 1);
        add_eq({{ig, 1.0}, {ig_prev, -1.0}, {su, -1.0}, {sd, 1.0}}, 0.0);
      }
      add_le({{su, 1.0}, {sd, 1.0}}, 1.0);

      // Must-run (§2.6.3.5)
      if (gen.must_run) {
        lp.vars[static_cast<size_t>(ig)].lb = 1.0;
        lp.vars[static_cast<size_t>(ig)].ub = 1.0;
      }
    }
  }

  // ── Min up/down time (§2.6.3.12) ─────────────────────────────────────────
  for (int g = 0; g < v.ng; ++g) {
    const auto& gen = inp.generators[static_cast<size_t>(g)];
    const int min_up = std::max(0, static_cast<int>(std::ceil(gen.min_up_time_hr)));
    const int min_dn = std::max(0, static_cast<int>(std::ceil(gen.min_dn_time_hr)));
    for (int h = 0; h < v.T_commit; ++h) {
      if (min_up > 0) {
        std::vector<std::pair<int,double>> row;
        for (int tau = std::max(0, h - min_up + 1); tau <= h; ++tau)
          row.emplace_back(v.SU_start + g * v.T_commit + tau, 1.0);
        row.emplace_back(v.IG_start + g * v.T_commit + h, -1.0);
        add_le(row, 0.0);
      }
      if (min_dn > 0) {
        std::vector<std::pair<int,double>> row;
        for (int tau = std::max(0, h - min_dn + 1); tau <= h; ++tau)
          row.emplace_back(v.SD_start + g * v.T_commit + tau, 1.0);
        row.emplace_back(v.IG_start + g * v.T_commit + h, 1.0);
        add_le(row, 1.0);
      }
    }
  }

  // ── Max startups/shutdowns (§2.6.3.13) ───────────────────────────────────
  for (int g = 0; g < v.ng; ++g) {
    const auto& gen = inp.generators[static_cast<size_t>(g)];
    if (gen.max_startups > 0) {
      std::vector<std::pair<int,double>> row;
      for (int h = 0; h < v.T_commit; ++h)
        row.emplace_back(v.SU_start + g * v.T_commit + h, 1.0);
      add_le(row, static_cast<double>(gen.max_startups));
    }
    if (gen.max_shutdowns > 0) {
      std::vector<std::pair<int,double>> row;
      for (int h = 0; h < v.T_commit; ++h)
        row.emplace_back(v.SD_start + g * v.T_commit + h, 1.0);
      add_le(row, static_cast<double>(gen.max_shutdowns));
    }
  }

  // ── Ramping constraints (§2.6.3.11) ──────────────────────────────────────
  for (int g = 0; g < v.ng; ++g) {
    const auto& gen = inp.generators[static_cast<size_t>(g)];
    const double pmax = gen.pmax;
    const double rup = gen.ramp_up_mw_min * 60.0 * dt;
    const double rdn = gen.ramp_dn_mw_min * 60.0 * dt;

    for (int t = 0; t < T; ++t) {
      const int h = t / iph;
      const int pg_t = v.PG_start + g * T + t;
      const int su_h = v.SU_start + g * v.T_commit + h;
      const int sd_h = v.SD_start + g * v.T_commit + h;

      if (t == 0) {
        double p0 = 0.0;
        if (g < static_cast<int>(inp.initial_status.dispatch.size()))
          p0 = std::max(0.0, inp.initial_status.dispatch[static_cast<size_t>(g)]);
        add_le({{pg_t, 1.0}, {su_h, -pmax}}, p0 + rup);
        add_le({{pg_t, -1.0}, {sd_h, -pmax}}, -p0 + rdn);
      } else {
        const int pg_prev = v.PG_start + g * T + (t - 1);
        add_le({{pg_t, 1.0}, {pg_prev, -1.0}, {su_h, -pmax}}, rup);
        add_le({{pg_prev, 1.0}, {pg_t, -1.0}, {sd_h, -pmax}}, rdn);
      }
    }
  }

  // ── Renewable variable bounds (§2.6.3.20) ─────────────────────────────────
  const double alpha_n = cfg.renewable_min_output_coeff;
  const bool has_ren_curt = (M2 > kEps);
  for (int t = 0; t < T; ++t) {
    for (int w = 0; w < v.nw; ++w) {
      const double fw = wind_at(inp, w, t);
      const int pw_idx = v.PW_start + w * T + t;
      if (has_ren_curt) {
        // P_w[t] + WIND_CURT[w][t] = forecast[w][t]; curt >= 0
        const int wc_idx = v.WIND_CURT_start + w * T + t;
        lp.vars[static_cast<size_t>(pw_idx)].lb = std::max(0.0, alpha_n * fw);
        lp.vars[static_cast<size_t>(pw_idx)].ub = 1e20;  // freed; equality constraint handles it
        lp.vars[static_cast<size_t>(wc_idx)].ub = std::max(0.0, fw);  // curt <= forecast
        add_eq({{pw_idx, 1.0}, {wc_idx, 1.0}}, std::max(0.0, fw));
      } else {
        lp.vars[static_cast<size_t>(pw_idx)].lb = std::max(0.0, alpha_n * fw);
        lp.vars[static_cast<size_t>(pw_idx)].ub = std::max(0.0, fw);
      }
    }
    for (int s = 0; s < v.npv; ++s) {
      const double fs = solar_at(inp, s, t);
      const int pv_idx = v.PPV_start + s * T + t;
      if (has_ren_curt) {
        const int sc_idx = v.SOL_CURT_start + s * T + t;
        lp.vars[static_cast<size_t>(pv_idx)].lb = std::max(0.0, alpha_n * fs);
        lp.vars[static_cast<size_t>(pv_idx)].ub = 1e20;
        lp.vars[static_cast<size_t>(sc_idx)].ub = std::max(0.0, fs);
        add_eq({{pv_idx, 1.0}, {sc_idx, 1.0}}, std::max(0.0, fs));
      } else {
        lp.vars[static_cast<size_t>(pv_idx)].lb = std::max(0.0, alpha_n * fs);
        lp.vars[static_cast<size_t>(pv_idx)].ub = std::max(0.0, fs);
      }
    }
  }

  // ── Storage constraints (§2.6.3.16) ──────────────────────────────────────
  for (int s = 0; s < v.nstorage; ++s) {
    const auto& sto = inp.storage[static_cast<size_t>(s)];
    const double p_ch_max  = std::max(0.0, sto.pmax_charge);
    const double p_dis_max = std::max(0.0, sto.pmax_discharge);
    const double p_ch_min  = std::max(0.0, sto.pmin_charge);
    const double p_dis_min = std::max(0.0, sto.pmin_discharge);
    const double e_cap = std::max(0.0, sto.energy_capacity_mwh);
    const double eta_rt = clampd(sto.efficiency, 0.1, 1.0);
    const double eta_sq = std::sqrt(eta_rt);
    // Separate charge / discharge efficiencies (§2.6.3.16(2))
    const double eta_ch  = (sto.eta_charge   > 0) ? clampd(sto.eta_charge,  0.01, 1.0) : eta_sq;
    const double eta_dis = (sto.eta_discharge > 0) ? clampd(sto.eta_discharge, 0.01, 1.0) : eta_sq;
    // SOC bounds (§2.6.3.16(2))
    const double soc_lo = (sto.soc_min >= 0) ? clampd(sto.soc_min, 0.0, 1.0) * e_cap : 0.1 * e_cap;

    // Initial SOC (§2.6.3.16(3))
    double soc0 = sto.soc_init * e_cap;
    if (s < static_cast<int>(inp.initial_status.storage_soc.size())) {
      const double iv = std::max(0.0, inp.initial_status.storage_soc[static_cast<size_t>(s)]);
      soc0 = (iv <= 1.0 + 1e-8) ? iv * e_cap : iv;
    }
    soc0 = clampd(soc0, 0.0, e_cap);

    // Required final SOC (§2.6.3.16(3))
    const double soc_fin = (sto.soc_final >= 0) ? clampd(sto.soc_final, 0.0, 1.0) * e_cap : soc0;

    const bool use_xi = v.use_sto_binaries && sto.use_binary_indicators;

    for (int t = 0; t < T; ++t) {
      const int p_in  = v.PSTO_IN_start  + s * T + t;
      const int p_out = v.PSTO_OUT_start + s * T + t;
      const int soc   = v.SOC_start      + s * T + t;

      lp.vars[static_cast<size_t>(soc)].lb = soc_lo;
      lp.vars[static_cast<size_t>(soc)].ub = e_cap;

      if (use_xi) {
        // (a) Binary mode indicators: xi+, xi- (§2.6.3.16(1))
        const int xi_ch  = v.XI_CH_start  + s * T + t;
        const int xi_dis = v.XI_DIS_start + s * T + t;
        // P_ch_min * xi+ <= P_ch <= P_ch_max * xi+
        lp.vars[static_cast<size_t>(p_in)].ub  = p_ch_max;
        lp.vars[static_cast<size_t>(p_out)].ub = p_dis_max;
        if (p_ch_min > kEps)
          add_le({{p_in,  -1.0}, {xi_ch,  p_ch_min}},  0.0);  // p_in >= p_ch_min * xi+
        add_le({{p_in,   1.0}, {xi_ch,  -p_ch_max}},  0.0);   // p_in <= p_ch_max * xi+
        if (p_dis_min > kEps)
          add_le({{p_out, -1.0}, {xi_dis, p_dis_min}}, 0.0);  // p_out >= p_dis_min * xi-
        add_le({{p_out,  1.0}, {xi_dis, -p_dis_max}}, 0.0);   // p_out <= p_dis_max * xi-
        // Mutual exclusion: xi+ + xi- <= 1 (§2.6.3.16(1))
        add_le({{xi_ch, 1.0}, {xi_dis, 1.0}}, 1.0);
      } else {
        // LP relaxation: simple bounds + soft anti-simultaneous
        lp.vars[static_cast<size_t>(p_in)].ub  = p_ch_max;
        lp.vars[static_cast<size_t>(p_out)].ub = p_dis_max;
        const double p_max_sim = std::max(p_ch_max, p_dis_max);
        add_le({{p_in, 1.0}, {p_out, 1.0}}, p_max_sim);
      }

      // (b) SOC dynamics: E_t = E_{t-1} + eta_ch * P_ch * dt - P_dis / eta_dis * dt (§2.6.3.16(2))
      if (t == 0) {
        add_eq({{soc, 1.0}, {p_in, -eta_ch * dt}, {p_out, dt / eta_dis}}, soc0);
      } else {
        const int soc_prev = v.SOC_start + s * T + (t - 1);
        add_eq({{soc, 1.0}, {soc_prev, -1.0}, {p_in, -eta_ch * dt}, {p_out, dt / eta_dis}}, 0.0);
      }
    }

    // (c) Final SOC constraint: E_T >= soc_fin (§2.6.3.16(3))
    if (T > 0) {
      add_le({{v.SOC_start + s * T + (T - 1), -1.0}}, -soc_fin);
    }

    // (e) Cycle limit (§2.6.3.16(5))
    if (sto.cycle_limit > kEps && e_cap > kEps) {
      std::vector<std::pair<int,double>> row;
      for (int t = 0; t < T; ++t) {
        row.emplace_back(v.PSTO_OUT_start + s * T + t, dt / eta_dis);
        row.emplace_back(v.PSTO_IN_start  + s * T + t, eta_ch * dt);
      }
      add_le(row, 2.0 * sto.cycle_limit * e_cap);
    }
  }

  // ── DC line limits and ramping (§2.6.3.17) ───────────────────────────────
  for (int dcl = 0; dcl < v.ndc; ++dcl) {
    const auto& dc = inp.dc_lines[static_cast<size_t>(dcl)];
    for (int t = 0; t < T; ++t) {
      const int pdc = v.PDC_start + dcl * T + t;
      lp.vars[static_cast<size_t>(pdc)].lb = dc.pmin;
      lp.vars[static_cast<size_t>(pdc)].ub = dc.pmax;
      if (t == 0) {
        add_le({{pdc, 1.0}}, dc.ramp_up);
        add_le({{pdc, -1.0}}, dc.ramp_dn);
      } else {
        const int pdc_prev = v.PDC_start + dcl * T + (t - 1);
        add_le({{pdc, 1.0}, {pdc_prev, -1.0}}, dc.ramp_up);
        add_le({{pdc_prev, 1.0}, {pdc, -1.0}}, dc.ramp_dn);
      }
    }
  }

  // ── System power balance (§2.6.3.1) ──────────────────────────────────────
  f.power_balance_eq_start = static_cast<int>(beq_vec.size());
  for (int t = 0; t < T; ++t) {
    std::vector<std::pair<int,double>> row;
    for (int g = 0; g < v.ng; ++g)
      row.emplace_back(v.PG_start + g * T + t, 1.0);
    for (int w = 0; w < v.nw; ++w)
      row.emplace_back(v.PW_start + w * T + t, 1.0);
    for (int s = 0; s < v.npv; ++s)
      row.emplace_back(v.PPV_start + s * T + t, 1.0);
    for (int s = 0; s < v.nstorage; ++s) {
      row.emplace_back(v.PSTO_OUT_start + s * T + t,  1.0);
      row.emplace_back(v.PSTO_IN_start  + s * T + t, -1.0);
    }
    for (int dcl = 0; dcl < v.ndc; ++dcl)
      row.emplace_back(v.PDC_start + dcl * T + t, 1.0);
    row.emplace_back(v.LOAD_SHED_start + t,  1.0);
    row.emplace_back(v.GEN_CURT_start  + t, -1.0);

    double load_t = 0.0;
    for (int d = 0; d < static_cast<int>(inp.loads.size()); ++d)
      load_t += load_at(inp, d, t);
    add_eq(row, load_t);
  }

  // ── PTDF-based line flow limits (§2.6.3.14) ───────────────────────────────
  const Eigen::MatrixXd PTDF = compute_ptdf(inp);
  f.PTDF = PTDF;
  f.line_fwd_row.assign(static_cast<size_t>(v.nl * T), -1);
  f.line_rev_row.assign(static_cast<size_t>(v.nl * T), -1);

  // Build per-bus incidence lists for fast PTDF row construction
  const int nb = v.nb;
  std::vector<std::vector<int>> gens_at(static_cast<size_t>(nb)),
                                 loads_at(static_cast<size_t>(nb)),
                                 wind_at_b(static_cast<size_t>(nb)),
                                 solar_at_b(static_cast<size_t>(nb)),
                                 sto_at(static_cast<size_t>(nb));
  for (int g = 0; g < v.ng; ++g) {
    int b = inp.generators[static_cast<size_t>(g)].bus;
    if (b >= 0 && b < nb) gens_at[static_cast<size_t>(b)].push_back(g);
  }
  for (int d = 0; d < static_cast<int>(inp.loads.size()); ++d) {
    int b = inp.loads[static_cast<size_t>(d)].bus;
    if (b >= 0 && b < nb) loads_at[static_cast<size_t>(b)].push_back(d);
  }
  for (int w = 0; w < v.nw; ++w) {
    int b = inp.wind[static_cast<size_t>(w)].bus;
    if (b >= 0 && b < nb) wind_at_b[static_cast<size_t>(b)].push_back(w);
  }
  for (int s = 0; s < v.npv; ++s) {
    int b = inp.solar[static_cast<size_t>(s)].bus;
    if (b >= 0 && b < nb) solar_at_b[static_cast<size_t>(b)].push_back(s);
  }
  for (int s = 0; s < v.nstorage; ++s) {
    int b = inp.storage[static_cast<size_t>(s)].bus;
    if (b >= 0 && b < nb) sto_at[static_cast<size_t>(b)].push_back(s);
  }

  // Line flow constraints
  if (PTDF.rows() == v.nl && PTDF.cols() == nb) {
    for (int l = 0; l < v.nl; ++l) {
      const auto& br = inp.branches[static_cast<size_t>(l)];
      const double lim = std::max(0.0, br.rating_mw);
      if (!br.in_service || lim > 1e5) continue;

      for (int t = 0; t < T; ++t) {
        // Add slack variables to the row vectors
        const int sl_pos_idx = v.SL_LINE_POS_start + l * T + t;
        const int sl_neg_idx = v.SL_LINE_NEG_start + l * T + t;

        int fwd_r, rev_r;
        // Build raw flow row, then append slacks
        std::vector<std::pair<int,double>> fwd, rev;
        double load_ptdf = 0.0;
        for (int b = 0; b < nb; ++b) {
          const double ptdf = PTDF(l, b);
          if (std::abs(ptdf) <= 1e-12) continue;
          for (int g : gens_at[static_cast<size_t>(b)]) {
            fwd.emplace_back(v.PG_start + g * T + t, ptdf);
            rev.emplace_back(v.PG_start + g * T + t, -ptdf);
          }
          for (int w : wind_at_b[static_cast<size_t>(b)]) {
            fwd.emplace_back(v.PW_start + w * T + t, ptdf);
            rev.emplace_back(v.PW_start + w * T + t, -ptdf);
          }
          for (int s : solar_at_b[static_cast<size_t>(b)]) {
            fwd.emplace_back(v.PPV_start + s * T + t, ptdf);
            rev.emplace_back(v.PPV_start + s * T + t, -ptdf);
          }
          for (int s : sto_at[static_cast<size_t>(b)]) {
            fwd.emplace_back(v.PSTO_OUT_start + s * T + t,  ptdf);
            fwd.emplace_back(v.PSTO_IN_start  + s * T + t, -ptdf);
            rev.emplace_back(v.PSTO_OUT_start + s * T + t, -ptdf);
            rev.emplace_back(v.PSTO_IN_start  + s * T + t,  ptdf);
          }
          for (int d : loads_at[static_cast<size_t>(b)])
            load_ptdf += ptdf * load_at(inp, d, t);
        }
        for (int dcl = 0; dcl < v.ndc; ++dcl) {
          const auto& dc = inp.dc_lines[static_cast<size_t>(dcl)];
          if (dc.from < 0 || dc.from >= nb || dc.to < 0 || dc.to >= nb) continue;
          const double coeff = PTDF(l, dc.to) - PTDF(l, dc.from);
          if (std::abs(coeff) > 1e-12) {
            fwd.emplace_back(v.PDC_start + dcl * T + t, coeff);
            rev.emplace_back(v.PDC_start + dcl * T + t, -coeff);
          }
        }
        fwd.emplace_back(sl_pos_idx, -1.0);
        rev.emplace_back(sl_neg_idx, -1.0);
        fwd_r = static_cast<int>(b_vec.size());
        add_le(fwd, lim + load_ptdf);
        rev_r = static_cast<int>(b_vec.size());
        add_le(rev, lim - load_ptdf);
        f.line_fwd_row[static_cast<size_t>(l * T + t)] = fwd_r;
        f.line_rev_row[static_cast<size_t>(l * T + t)] = rev_r;
      }
    }
  }

  // ── Section flow constraints (§2.6.3.15) ─────────────────────────────────
  // Section PTDF = weighted sum of constituent line PTDFs
  f.sec_fwd_row.assign(static_cast<size_t>(v.nsec * T), -1);
  f.sec_rev_row.assign(static_cast<size_t>(v.nsec * T), -1);
  if (v.nsec > 0 && PTDF.rows() == v.nl && PTDF.cols() == nb) {
    f.SEC_PTDF = Eigen::MatrixXd::Zero(v.nsec, nb);
    for (int s = 0; s < v.nsec; ++s) {
      const auto& sec = inp.sections[static_cast<size_t>(s)];
      for (const auto& [li, w] : sec.line_weights) {
        if (li >= 0 && li < v.nl)
          f.SEC_PTDF.row(s) += w * PTDF.row(li);
      }
    }
    for (int s = 0; s < v.nsec; ++s) {
      const auto& sec = inp.sections[static_cast<size_t>(s)];
      const double fwd_lim = std::max(0.0, sec.rating_fwd_mw);
      const double rev_lim = std::max(0.0, sec.rating_rev_mw);
      for (int t = 0; t < T; ++t) {
        const int sl_pos_idx = v.SL_SEC_POS_start + s * T + t;
        const int sl_neg_idx = v.SL_SEC_NEG_start + s * T + t;

        std::vector<std::pair<int,double>> fwd, rev;
        double load_ptdf = 0.0;
        for (int b = 0; b < nb; ++b) {
          const double ptdf = f.SEC_PTDF(s, b);
          if (std::abs(ptdf) <= 1e-12) continue;
          for (int g : gens_at[static_cast<size_t>(b)]) {
            fwd.emplace_back(v.PG_start + g * T + t, ptdf);
            rev.emplace_back(v.PG_start + g * T + t, -ptdf);
          }
          for (int w : wind_at_b[static_cast<size_t>(b)]) {
            fwd.emplace_back(v.PW_start + w * T + t, ptdf);
            rev.emplace_back(v.PW_start + w * T + t, -ptdf);
          }
          for (int ss : solar_at_b[static_cast<size_t>(b)]) {
            fwd.emplace_back(v.PPV_start + ss * T + t, ptdf);
            rev.emplace_back(v.PPV_start + ss * T + t, -ptdf);
          }
          for (int ss : sto_at[static_cast<size_t>(b)]) {
            fwd.emplace_back(v.PSTO_OUT_start + ss * T + t,  ptdf);
            fwd.emplace_back(v.PSTO_IN_start  + ss * T + t, -ptdf);
            rev.emplace_back(v.PSTO_OUT_start + ss * T + t, -ptdf);
            rev.emplace_back(v.PSTO_IN_start  + ss * T + t,  ptdf);
          }
          for (int d : loads_at[static_cast<size_t>(b)])
            load_ptdf += ptdf * load_at(inp, d, t);
        }
        fwd.emplace_back(sl_pos_idx, -1.0);
        rev.emplace_back(sl_neg_idx, -1.0);
        f.sec_fwd_row[static_cast<size_t>(s * T + t)] = static_cast<int>(b_vec.size());
        add_le(fwd, fwd_lim + load_ptdf);
        f.sec_rev_row[static_cast<size_t>(s * T + t)] = static_cast<int>(b_vec.size());
        add_le(rev, rev_lim - load_ptdf);
      }
    }
  }

  // ── Reserve requirements (§2.6.3.2, §2.6.3.3, §2.6.3.4) ─────────────────
  for (int t = 0; t < T; ++t) {
    double load_t = 0.0;
    for (int d = 0; d < static_cast<int>(inp.loads.size()); ++d)
      load_t += load_at(inp, d, t);

    const double spin_req  = cfg.spinning_reserve_req  * load_t;
    const double rup_req   = cfg.regulation_up_req     * load_t;
    const double rdn_req   = cfg.regulation_down_req   * load_t;
    const double neg_req   = cfg.neg_reserve_req       * load_t;  // §2.6.3.3
    const double pfr_req   = cfg.pfr_reserve_req_mw;              // §2.6.3.4

    std::vector<std::pair<int,double>> spin_row, rup_row, rdn_row;
    // Negative reserve: sum_g (P_{g,t} - Pmin * IG) <= load - neg_req
    std::vector<std::pair<int,double>> neg_row;
    // PFR: sum_g alpha_pf * Pmax * IG >= pfr_req
    std::vector<std::pair<int,double>> pfr_row;

    for (int g = 0; g < v.ng; ++g) {
      const auto& gen = inp.generators[static_cast<size_t>(g)];
      const double pmax = gen.pmax, pmin = gen.pmin;
      const int h = t / iph;
      const int pg = v.PG_start + g * T + t;
      const int ig = v.IG_start + g * v.T_commit + h;
      const int rs = v.RG_spin_start    + g * T + t;
      const int ru = v.RG_reg_up_start  + g * T + t;
      const int rd = v.RG_reg_down_start + g * T + t;

      spin_row.emplace_back(rs, -1.0);
      rup_row.emplace_back(ru,  -1.0);
      rdn_row.emplace_back(rd,  -1.0);

      // Reserve coupling: P + R_spin + R_up <= Pmax * IG
      add_le({{pg, 1.0}, {rs, 1.0}, {ru, 1.0}, {ig, -pmax}}, 0.0);
      // Regulation down: R_down <= P - Pmin * IG
      add_le({{rd, 1.0}, {pg, -1.0}, {ig, pmin}}, 0.0);
      // Spinning cap: R_spin <= (Pmax - Pmin) * IG
      add_le({{rs, 1.0}, {ig, -(pmax - pmin)}}, 0.0);

      const double r10u = gen.ramp_up_mw_min * 10.0;
      if (r10u > kEps) add_le({{ru, 1.0}, {ig, -r10u}}, 0.0);
      const double r10d = gen.ramp_dn_mw_min * 10.0;
      if (r10d > kEps) add_le({{rd, 1.0}, {ig, -r10d}}, 0.0);

      // §2.6.3.3 Negative reserve: P - Pmin * IG (dispatchable downward room)
      if (neg_req > 0.0) {
        neg_row.emplace_back(pg,  1.0);
        neg_row.emplace_back(ig, -pmin);
      }

      // §2.6.3.4 PFR: alpha_pf * Pmax * IG (simplified capacity bound)
      if (pfr_req > 0.0 && gen.pfr_alpha > kEps) {
        pfr_row.emplace_back(ig, -gen.pfr_alpha * pmax);
      }
    }

    // Positive reserve requirements
    if (spin_req > 0.0) add_le(spin_row, -spin_req);
    if (rup_req  > 0.0) add_le(rup_row,  -rup_req);
    if (rdn_req  > 0.0) add_le(rdn_row,  -rdn_req);

    // §2.6.3.3 Negative reserve: sum(P - Pmin*IG) <= load - neg_req
    // Equivalent to: sum_g(P_g - Pmin_g * IG_g) <= load_t - neg_req
    if (neg_req > 0.0 && !neg_row.empty())
      add_le(neg_row, load_t - neg_req);

    // §2.6.3.4 PFR requirement: sum_g alpha^pf_g * Pmax_g * IG_g >= pfr_req
    // Written as: -sum ... <= -pfr_req
    if (pfr_req > 0.0 && !pfr_row.empty())
      add_le(pfr_row, -pfr_req);
  }

  // ── Generator group output limits (§2.6.3.9) ─────────────────────────────
  for (const auto& grp : inp.generator_groups) {
    if (grp.gen_indices.empty()) continue;
    for (int t = 0; t < T; ++t) {
      // Group output sum = sum of PG for members
      std::vector<std::pair<int,double>> row;
      for (int gi : grp.gen_indices) {
        if (gi >= 0 && gi < v.ng)
          row.emplace_back(v.PG_start + gi * T + t, 1.0);
      }
      if (row.empty()) continue;

      // Lower bound
      double gp_min = 0.0;
      if (!grp.pmin_t.empty()) {
        const int tidx = (grp.pmin_t.size() == 1) ? 0 : std::min(t, static_cast<int>(grp.pmin_t.size()) - 1);
        gp_min = grp.pmin_t[static_cast<size_t>(tidx)];
      }
      // Upper bound
      double gp_max = 1e18;
      if (!grp.pmax_t.empty()) {
        const int tidx = (grp.pmax_t.size() == 1) ? 0 : std::min(t, static_cast<int>(grp.pmax_t.size()) - 1);
        gp_max = grp.pmax_t[static_cast<size_t>(tidx)];
      }
      if (gp_min > 0.0) {
        std::vector<std::pair<int,double>> lb_row;
        for (const auto& [c, cv] : row) lb_row.emplace_back(c, -cv);
        add_le(lb_row, -gp_min);
      }
      if (gp_max < 1e17)
        add_le(row, gp_max);
    }
  }

  // ── Generator group energy limits (§2.6.3.10) ────────────────────────────
  for (const auto& grp : inp.generator_groups) {
    if (grp.gen_indices.empty()) continue;
    if (grp.emax < kEps && grp.emin < kEps) continue;

    std::vector<std::pair<int,double>> row;
    for (int gi : grp.gen_indices) {
      if (gi < 0 || gi >= v.ng) continue;
      for (int t = 0; t < T; ++t)
        row.emplace_back(v.PG_start + gi * T + t, dt);
    }
    if (row.empty()) continue;
    if (grp.emax > kEps)
      add_le(row, grp.emax);
    if (grp.emin > kEps) {
      std::vector<std::pair<int,double>> lb_row;
      for (const auto& [c, cv] : row) lb_row.emplace_back(c, -cv);
      add_le(lb_row, -grp.emin);
    }
  }

  // ── LP-valid pre-formulation market cutting planes ─────────────────────────
  int n_cuts = 0;
  if (cfg.enable_market_cuts) {
    // For large UC problems the root LP solve is already the binding constraint
    // on solver time.  Adding extended cut families (T, H, R) inflates the LP
    // by O(G × T_commit) rows each; beyond the threshold below they make the
    // root LP slower without tightening the bound (since the LP never finishes
    // within the time budget anyway).  Small cases (< threshold) benefit from
    // these families via presolve/clique detection, so they remain enabled there.
    //
    // Threshold separation (2025-05):
    //   large_uc (> 800): disables Families T, H, and R (adds 4×G×T +
    //   conflict-cover + 2×G×(T-1) rows; row overhead dominates LP-tightening
    //   gain for the 118-bus case 54×24=1296 where even with warm-start the
    //   harder per-node LP degrades primal quality and stochastic B&B bound).
    const bool large_uc = (v.ng * v.T_commit > 800);

    // Family 6: symmetry-breaking for identical generators
    {
      struct GenKey { int pmax_bkt, pmin_bkt, tu, td;
        bool operator<(const GenKey& o) const {
          return std::tie(pmax_bkt,pmin_bkt,tu,td) < std::tie(o.pmax_bkt,o.pmin_bkt,o.tu,o.td);
        }};
      std::map<GenKey, std::vector<int>> groups;
      for (int g = 0; g < v.ng; ++g) {
        const auto& gen = inp.generators[static_cast<size_t>(g)];
        groups[{static_cast<int>(std::round(gen.pmax)),
                static_cast<int>(std::round(gen.pmin)),
                static_cast<int>(std::round(gen.min_up_time_hr)),
                static_cast<int>(std::round(gen.min_dn_time_hr))}].push_back(g);
      }
      for (auto& [key, gens] : groups) {
        if (static_cast<int>(gens.size()) < 2) continue;
        for (int ji = 0; ji + 1 < static_cast<int>(gens.size()); ++ji) {
          for (int h = 0; h < v.T_commit; ++h) {
            add_le({{v.IG_start + gens[static_cast<size_t>(ji+1)] * v.T_commit + h,  1.0},
                    {v.IG_start + gens[static_cast<size_t>(ji)]   * v.T_commit + h, -1.0}}, 0.0);
            ++n_cuts;
          }
        }
      }
    }

    // Family A: cumulative segment coupling cuts.
    // For each generator g, dispatch period t, and segment prefix m:
    //   SEG_0 + SEG_1 + ... + SEG_m <= cumQ_m * IG
    // where cumQ_m = Σ_{k=0}^{m} q_k.  These LP-valid inequalities tighten the
    // root LP relaxation; HiGHS extracts them as warm root cuts for re-injection
    // at every B&C node, yielding fast per-node LP solves.
    for (int g = 0; g < v.ng; ++g) {
      const auto& gen = inp.generators[static_cast<size_t>(g)];
      for (int t = 0; t < T; ++t) {
        const int h = t / iph;
        const int ig = v.IG_start + g * v.T_commit + h;
        double cum_q = 0.0;
        for (int m = 0; m < v.n_segments; ++m) {
          double q_m = 0.0;
          if (m < static_cast<int>(gen.bid_segments.size()))
            q_m = gen.bid_segments[static_cast<size_t>(m)].quantity;
          cum_q += q_m;
          if (m == 0) continue;
          if (cum_q > kEps) {
            std::vector<std::pair<int,double>> row;
            for (int k = 0; k <= m; ++k)
              row.emplace_back(v.SEG_start[static_cast<size_t>(k)] + g * T + t, 1.0);
            row.emplace_back(ig, -cum_q);
            add_le(row, 0.0);
            ++n_cuts;
          }
        }
      }
    }

    // Family T: transition hull cuts.  Provides the convex-hull
    // characterisation of SU/SD vs IG transitions; adds 4 × G × T_commit rows.
    // Disabled for large_uc (> 800) — row overhead hurts primal quality for
    // 118-bus-scale instances (degrades cold incumbent, increases objective gap).
    if (!large_uc) {
    for (int g = 0; g < v.ng; ++g) {
      const double ig0 =
          (g < static_cast<int>(inp.initial_status.commitment.size()) &&
           inp.initial_status.commitment[static_cast<size_t>(g)] > 0.5) ? 1.0 : 0.0;
      for (int h = 0; h < v.T_commit; ++h) {
        const int su = v.SU_start + g * v.T_commit + h;
        const int sd = v.SD_start + g * v.T_commit + h;
        const int ig = v.IG_start + g * v.T_commit + h;
        add_le({{su, 1.0}, {ig, -1.0}}, 0.0);
        add_le({{sd, 1.0}, {ig, 1.0}}, 1.0);
        if (h == 0) {
          add_le({{su, 1.0}}, 1.0 - ig0);
          add_le({{sd, 1.0}}, ig0);
        } else {
          const int ig_prev = v.IG_start + g * v.T_commit + (h - 1);
          add_le({{su, 1.0}, {ig_prev, 1.0}}, 1.0);
          add_le({{sd, 1.0}, {ig_prev, -1.0}}, 0.0);
        }
        n_cuts += 4;
      }
    }
    } // !large_uc (Family T)

    // Family G: extended startup clique (TU + TD window)
    for (int g = 0; g < v.ng; ++g) {
      const auto& gen = inp.generators[static_cast<size_t>(g)];
      const int TU = std::max(0, static_cast<int>(std::ceil(gen.min_up_time_hr)));
      const int TD = std::max(0, static_cast<int>(std::ceil(gen.min_dn_time_hr)));
      const int window = TU + TD;
      if (window < 2) continue;
      for (int h = 0; h + window - 1 < v.T_commit; ++h) {
        std::vector<std::pair<int,double>> row;
        for (int hp = h; hp < h + window; ++hp)
          row.emplace_back(v.SU_start + g * v.T_commit + hp, 1.0);
        add_le(row, 1.0);
        ++n_cuts;
      }
    }

    // Family H: transition-conflict cover cuts.  Fully implied by Family G + T,
    // so it has zero independent LP value but inflates the LP significantly.
    // Disabled for large_uc (> 800) — pure overhead at this scale.
    if (!large_uc) {
    for (int g = 0; g < v.ng; ++g) {
      const auto& gen = inp.generators[static_cast<size_t>(g)];
      const int TU = std::max(0, static_cast<int>(std::ceil(gen.min_up_time_hr)));
      const int TD = std::max(0, static_cast<int>(std::ceil(gen.min_dn_time_hr)));
      if (TU > 1) {
        for (int start = 0; start < v.T_commit; ++start) {
          const int su = v.SU_start + g * v.T_commit + start;
          const int last = std::min(v.T_commit - 1, start + TU - 1);
          for (int shut = start + 1; shut <= last; ++shut) {
            const int sd = v.SD_start + g * v.T_commit + shut;
            add_le({{su, 1.0}, {sd, 1.0}}, 1.0);
            ++n_cuts;
          }
        }
      }
      if (TD > 1) {
        for (int shut = 0; shut < v.T_commit; ++shut) {
          const int sd = v.SD_start + g * v.T_commit + shut;
          const int last = std::min(v.T_commit - 1, shut + TD - 1);
          for (int start = shut + 1; start <= last; ++start) {
            const int su = v.SU_start + g * v.T_commit + start;
            add_le({{sd, 1.0}, {su, 1.0}}, 1.0);
            ++n_cuts;
          }
        }
      }
    }
    } // !large_uc (Family H)

    // Family R: tight ramp-perspective cuts.  Uses startup/shutdown capacity
    // coefficients min(Pmin+R, Pmax) instead of big-M Pmax in the SU/SD term:
    //   P_t - P_{t-1} <= R · IG_{t-1} + min(Pmin+R, Pmax) · SU_t
    //   P_{t-1} - P_t <= R · IG_t     + min(Pmin+R, Pmax) · SD_t
    // At startup (IG_{t-1}=0, SU_t=1): P_t <= min(Pmin+R, Pmax) instead of
    // Pmax — a factor 2-4× tighter for typical slow-ramping generators.
    // Adds 2 × G × (T-1) rows.  Disabled for large_uc (> 800): for the 118-bus
    // 54G×24T case these generators have ramp rates large enough that su_cap ≈
    // Pmax so tightening is negligible, while the extra rows degrade node LP
    // performance and primal heuristic quality.
    if (!large_uc) {
    for (int g = 0; g < v.ng; ++g) {
      const auto& gen = inp.generators[static_cast<size_t>(g)];
      const double pmax = std::max(0.0, gen.pmax);
      const double pmin = std::max(0.0, gen.pmin);
      if (pmax <= kEps) continue;
      const double rup = std::min(pmax, std::max(0.0, gen.ramp_up_mw_min * 60.0 * dt));
      const double rdn = std::min(pmax, std::max(0.0, gen.ramp_dn_mw_min * 60.0 * dt));
      // Tight coefficients: at startup/shutdown the unit can only produce
      // up to Pmin + one-period ramp capacity (not the big-M Pmax).
      const double su_cap = std::min(pmax, pmin + rup);
      const double sd_cap = std::min(pmax, pmin + rdn);
      for (int t = 1; t < T; ++t) {
        const int h      = t / iph;
        const int h_prev = (t - 1) / iph;
        const int pg_t    = v.PG_start + g * T + t;
        const int pg_prev = v.PG_start + g * T + (t - 1);
        const int ig_prev = v.IG_start + g * v.T_commit + h_prev;
        const int ig_t    = v.IG_start + g * v.T_commit + h;
        const int su_h    = v.SU_start + g * v.T_commit + h;
        const int sd_h    = v.SD_start + g * v.T_commit + h;
        add_le({{pg_t, 1.0}, {pg_prev, -1.0}, {ig_prev, -rup}, {su_h, -su_cap}}, 0.0);
        add_le({{pg_prev, 1.0}, {pg_t, -1.0}, {ig_t,    -rdn}, {sd_h, -sd_cap}}, 0.0);
        n_cuts += 2;
      }
    }
    } // !large_uc (Family R)

    // Family P: startup-ramp capacity upper bounds (multi-lag).
    // If the unit started k periods ago (SU_{g,s}=1 at s=h-k+1), its output
    // at period h is bounded by Pmin + k·Rup (k ramp steps from Pmin):
    //   P_{g,h} + max(0, Pmax-Pmin-k·Rup) · SU_{g,h-k+1} <= Pmax · IG_{g,h}
    // For k=1 this directly bounds the startup period itself.  For k=2..k_max
    // it tightens the LP relaxation beyond what Family R provides when IG is
    // fractional across periods (Family R only uses adjacent periods).
    // Disabled for large_uc: same reasoning as Family R (generators have large
    // ramp rates, excess≈0, negligible LP tightening but significant overhead).
    // Only active when T == T_commit (iph=1) for exact period alignment.
    if (v.T == v.T_commit && !large_uc) {
      for (int g = 0; g < v.ng; ++g) {
        const auto& gen = inp.generators[static_cast<size_t>(g)];
        const double pmax = std::max(0.0, gen.pmax);
        const double pmin = std::max(0.0, gen.pmin);
        if (pmax <= pmin + kEps) continue;
        const double rup = std::max(0.0, gen.ramp_up_mw_min * 60.0 * dt);
        const int tu = std::max(0, static_cast<int>(std::ceil(gen.min_up_time_hr)));
        const int k_limit = std::min(std::max(0, tu - 1), 8);
        for (int k = 1; k <= k_limit; ++k) {
          const double excess = pmax - pmin - static_cast<double>(k) * rup;
          if (excess < kEps) break;  // k·Rup covers full range — no tightening
          for (int h = k - 1; h < v.T_commit; ++h) {
            const int su_per = h - k + 1;   // startup period (>= 0)
            add_le({{v.PG_start + g * T + h,            1.0},
                    {v.SU_start + g * v.T_commit + su_per, excess},
                    {v.IG_start + g * v.T_commit + h,    -pmax}}, 0.0);
            ++n_cuts;
          }
        }
      }
    }

    // Family Q: shutdown-ramp capacity upper bounds (multi-lag).
    // Symmetric to P: if the unit shuts down in k periods (SD_{g,h+k}=1),
    // output at period h must leave room to ramp down to Pmin before shutdown:
    //   P_{g,h} + max(0, Pmax-Pmin-k·Rdn) · SD_{g,h+k} <= Pmax · IG_{g,h}
    // Disabled for large_uc: same reasoning as Family P.
    if (v.T == v.T_commit && !large_uc) {
      for (int g = 0; g < v.ng; ++g) {
        const auto& gen = inp.generators[static_cast<size_t>(g)];
        const double pmax = std::max(0.0, gen.pmax);
        const double pmin = std::max(0.0, gen.pmin);
        if (pmax <= pmin + kEps) continue;
        const double rdn = std::max(0.0, gen.ramp_dn_mw_min * 60.0 * dt);
        const int td = std::max(0, static_cast<int>(std::ceil(gen.min_dn_time_hr)));
        const int k_limit = std::min(std::max(0, td - 1), 8);
        for (int k = 1; k <= k_limit; ++k) {
          const double excess = pmax - pmin - static_cast<double>(k) * rdn;
          if (excess < kEps) break;
          for (int h = 0; h + k < v.T_commit; ++h) {
            const int sd_per = h + k;       // shutdown period
            add_le({{v.PG_start + g * T + h,            1.0},
                    {v.SD_start + g * v.T_commit + sd_per, excess},
                    {v.IG_start + g * v.T_commit + h,    -pmax}}, 0.0);
            ++n_cuts;
          }
        }
      }
    }

    // Family C: system reserve-cover aggregations.  Summing the individual
    // headroom rows with the reserve requirements and power balance yields
    // compact cover inequalities that expose system capacity and downward
    // reserve pressure directly in the root LP.
    for (int t = 0; t < T; ++t) {
      double load_t = 0.0;
      for (int d = 0; d < static_cast<int>(inp.loads.size()); ++d)
        load_t += load_at(inp, d, t);
      const double up_req = (cfg.spinning_reserve_req + cfg.regulation_up_req) * load_t;
      const double down_req = cfg.regulation_down_req * load_t;
      if (up_req > kEps) {
        std::vector<std::pair<int,double>> row;
        for (int g = 0; g < v.ng; ++g) {
          const int h = t / iph;
          row.emplace_back(v.IG_start + g * v.T_commit + h,
                           -std::max(0.0, inp.generators[static_cast<size_t>(g)].pmax));
        }
        for (int w = 0; w < v.nw; ++w) row.emplace_back(v.PW_start + w * T + t, -1.0);
        for (int s = 0; s < v.npv; ++s) row.emplace_back(v.PPV_start + s * T + t, -1.0);
        for (int s = 0; s < v.nstorage; ++s) {
          row.emplace_back(v.PSTO_OUT_start + s * T + t, -1.0);
          row.emplace_back(v.PSTO_IN_start  + s * T + t,  1.0);
        }
        for (int dcl = 0; dcl < v.ndc; ++dcl) row.emplace_back(v.PDC_start + dcl * T + t, -1.0);
        row.emplace_back(v.LOAD_SHED_start + t, -1.0);
        row.emplace_back(v.GEN_CURT_start  + t,  1.0);
        add_le(row, -(load_t + up_req));
        ++n_cuts;
      }
      if (down_req > kEps) {
        std::vector<std::pair<int,double>> row;
        for (int g = 0; g < v.ng; ++g) {
          const int h = t / iph;
          row.emplace_back(v.IG_start + g * v.T_commit + h,
                           std::max(0.0, inp.generators[static_cast<size_t>(g)].pmin));
        }
        for (int w = 0; w < v.nw; ++w) row.emplace_back(v.PW_start + w * T + t, 1.0);
        for (int s = 0; s < v.npv; ++s) row.emplace_back(v.PPV_start + s * T + t, 1.0);
        for (int s = 0; s < v.nstorage; ++s) {
          row.emplace_back(v.PSTO_OUT_start + s * T + t,  1.0);
          row.emplace_back(v.PSTO_IN_start  + s * T + t, -1.0);
        }
        for (int dcl = 0; dcl < v.ndc; ++dcl) row.emplace_back(v.PDC_start + dcl * T + t, 1.0);
        row.emplace_back(v.LOAD_SHED_start + t,  1.0);
        row.emplace_back(v.GEN_CURT_start  + t, -1.0);
        add_le(row, load_t - down_req);
        ++n_cuts;
      }
    }

    // Family V: reserve deliverability covers.  These are projected cover
    // inequalities obtained by summing reserve requirements with each unit's
    // certified reserve capability.  They remove the continuous reserve
    // variables from the row, so fractional commitment must already carry
    // enough online capability at the root node.
    for (int t = 0; t < T; ++t) {
      double load_t = 0.0;
      for (int d = 0; d < static_cast<int>(inp.loads.size()); ++d)
        load_t += load_at(inp, d, t);
      const double spin_req = cfg.spinning_reserve_req * load_t;
      const double rup_req = cfg.regulation_up_req * load_t;
      const double rdn_req = cfg.regulation_down_req * load_t;
      const int h = t / iph;

      auto add_reserve_cover = [&](double req, auto cap_of_unit) {
        if (!(req > kEps)) return;
        std::vector<std::pair<int,double>> row;
        row.reserve(static_cast<size_t>(v.ng));
        for (int g = 0; g < v.ng; ++g) {
          const double cap = std::max(0.0, cap_of_unit(inp.generators[static_cast<size_t>(g)]));
          if (cap > kEps)
            row.emplace_back(v.IG_start + g * v.T_commit + h, -cap);
        }
        if (!row.empty()) {
          add_le(row, -req);
          ++n_cuts;
        }
      };

      add_reserve_cover(spin_req + rup_req, [](const Generator& gen) {
        return std::max(0.0, gen.pmax - gen.pmin);
      });
      add_reserve_cover(spin_req, [](const Generator& gen) {
        return std::max(0.0, gen.pmax - gen.pmin);
      });
      add_reserve_cover(rup_req, [](const Generator& gen) {
        const double headroom = std::max(0.0, gen.pmax - gen.pmin);
        const double r10 = std::max(0.0, gen.ramp_up_mw_min * 10.0);
        return r10 > kEps ? std::min(headroom, r10) : headroom;
      });
      add_reserve_cover(rdn_req, [](const Generator& gen) {
        const double headroom = std::max(0.0, gen.pmax - gen.pmin);
        const double r10 = std::max(0.0, gen.ramp_dn_mw_min * 10.0);
        return r10 > kEps ? std::min(headroom, r10) : headroom;
      });
    }

    // Family 11: cyclic SOC bound for storage (corrected for separate eta)
    for (int s = 0; s < v.nstorage; ++s) {
      const auto& sto = inp.storage[static_cast<size_t>(s)];
      const double e_cap = std::max(0.0, sto.energy_capacity_mwh);
      if (e_cap < kEps) continue;
      const double eta_rt = clampd(sto.efficiency, 0.1, 1.0);
      const double eta_sq = std::sqrt(eta_rt);
      const double eta_ch  = (sto.eta_charge   > 0) ? clampd(sto.eta_charge,  0.01, 1.0) : eta_sq;
      const double soc0_frac = clampd(sto.soc_init, 0.0, 1.0);
      const double soc_fin = (sto.soc_final >= 0) ? clampd(sto.soc_final, 0.0, 1.0) * e_cap
                                                   : soc0_frac * e_cap;
      const double rhs = eta_ch * sto.pmax_charge * static_cast<double>(T) * dt
                       + soc0_frac * e_cap - soc_fin;
      if (rhs < kEps) continue;
      std::vector<std::pair<int,double>> row;
      for (int t = 0; t < T; ++t) {
        const double eta_dis = (sto.eta_discharge > 0) ? clampd(sto.eta_discharge, 0.01, 1.0) : eta_sq;
        row.emplace_back(v.PSTO_OUT_start + s * T + t, dt / eta_dis);
      }
      add_le(row, rhs);
      ++n_cuts;
    }
  }
  f.n_cuts = n_cuts;

  // ── Assemble sparse matrices ──────────────────────────────────────────────
  lp.A.resize(static_cast<int>(b_vec.size()), v.nx);
  lp.A.setFromTriplets(A_trip.begin(), A_trip.end());
  lp.b = Eigen::Map<Eigen::VectorXd>(b_vec.data(), static_cast<int>(b_vec.size()));

  lp.Aeq.resize(static_cast<int>(beq_vec.size()), v.nx);
  lp.Aeq.setFromTriplets(Aeq_trip.begin(), Aeq_trip.end());
  lp.beq = Eigen::Map<Eigen::VectorXd>(beq_vec.data(), static_cast<int>(beq_vec.size()));

  f.n_ineq_rows = static_cast<int>(b_vec.size());
  f.n_eq_rows   = static_cast<int>(beq_vec.size());

  mip.initial_solution = build_scuc_primal_repair_seed(inp, v, lp);

  // UC hint for native B&C
  {
    engine::MIPModel::UCGenHint uc;
    uc.ng = v.ng; uc.T = v.T_commit;
    uc.period_hours = cfg.period_length_hr;
    uc.pg_start = v.PG_start;
    uc.ig_start = v.IG_start; uc.su_start = v.SU_start; uc.sd_start = v.SD_start;
    uc.min_up.resize(static_cast<size_t>(v.ng));
    uc.min_down.resize(static_cast<size_t>(v.ng));
    uc.ig0.resize(static_cast<size_t>(v.ng));
    uc.pmin.resize(static_cast<size_t>(v.ng));
    uc.pmax.resize(static_cast<size_t>(v.ng));
    uc.ramp.resize(static_cast<size_t>(v.ng));
    uc.up_reserve_headroom_cap.resize(static_cast<size_t>(v.ng));
    uc.spinning_reserve_cap.resize(static_cast<size_t>(v.ng));
    uc.regulation_up_cap.resize(static_cast<size_t>(v.ng));
    uc.regulation_down_cap.resize(static_cast<size_t>(v.ng));
    const int block_size = v.ng * v.T_commit;
    uc.ig_cols.assign(static_cast<size_t>(block_size), -1);
    uc.su_cols.assign(static_cast<size_t>(block_size), -1);
    uc.sd_cols.assign(static_cast<size_t>(block_size), -1);
    if (v.T == v.T_commit) {
      uc.pg_cols.assign(static_cast<size_t>(block_size), -1);
    }
    uc.gen_bus.resize(static_cast<size_t>(v.ng), -1);
    for (int g = 0; g < v.ng; ++g) {
      const auto& gen = inp.generators[static_cast<size_t>(g)];
      uc.gen_bus[static_cast<size_t>(g)] = gen.bus;
      uc.min_up[static_cast<size_t>(g)]   = std::max(0, static_cast<int>(std::ceil(gen.min_up_time_hr)));
      uc.min_down[static_cast<size_t>(g)] = std::max(0, static_cast<int>(std::ceil(gen.min_dn_time_hr)));
      uc.ig0[static_cast<size_t>(g)] =
          (g < static_cast<int>(inp.initial_status.commitment.size()) &&
           inp.initial_status.commitment[static_cast<size_t>(g)] > 0.5) ? 1 : 0;
      uc.pmin[static_cast<size_t>(g)] = gen.pmin;
      uc.pmax[static_cast<size_t>(g)] = gen.pmax;
      uc.ramp[static_cast<size_t>(g)] = std::max(gen.ramp_up_mw_min, gen.ramp_dn_mw_min) * 60.0 * dt;
        const double headroom = std::max(0.0, gen.pmax - gen.pmin);
        const double r10u = std::max(0.0, gen.ramp_up_mw_min * 10.0);
        const double r10d = std::max(0.0, gen.ramp_dn_mw_min * 10.0);
        uc.up_reserve_headroom_cap[static_cast<size_t>(g)] = headroom;
        uc.spinning_reserve_cap[static_cast<size_t>(g)] = headroom;
        uc.regulation_up_cap[static_cast<size_t>(g)] =
          r10u > kEps ? std::min(headroom, r10u) : headroom;
        uc.regulation_down_cap[static_cast<size_t>(g)] =
          r10d > kEps ? std::min(headroom, r10d) : headroom;
      for (int h = 0; h < v.T_commit; ++h) {
        const size_t pos = static_cast<size_t>(h * v.ng + g);
        uc.ig_cols[pos] = v.IG_start + g * v.T_commit + h;
        uc.su_cols[pos] = v.SU_start + g * v.T_commit + h;
        uc.sd_cols[pos] = v.SD_start + g * v.T_commit + h;
        if (!uc.pg_cols.empty()) {
          uc.pg_cols[pos] = v.PG_start + g * T + h;
        }
      }
    }
    uc.demand.assign(static_cast<size_t>(T), 0.0);
    uc.reserve_requirement.assign(static_cast<size_t>(T), 0.0);
    uc.spinning_requirement.assign(static_cast<size_t>(T), 0.0);
    uc.regulation_up_requirement.assign(static_cast<size_t>(T), 0.0);
    uc.regulation_down_requirement.assign(static_cast<size_t>(T), 0.0);
    for (int t = 0; t < T; ++t) {
      double load_t = 0.0;
      for (int d = 0; d < static_cast<int>(inp.loads.size()); ++d)
        load_t += load_at(inp, d, t);
      uc.demand[static_cast<size_t>(t)] = load_t;
      uc.reserve_requirement[static_cast<size_t>(t)] =
          (cfg.spinning_reserve_req + cfg.regulation_up_req) * load_t;
      uc.spinning_requirement[static_cast<size_t>(t)] =
          cfg.spinning_reserve_req * load_t;
      uc.regulation_up_requirement[static_cast<size_t>(t)] =
          cfg.regulation_up_req * load_t;
      uc.regulation_down_requirement[static_cast<size_t>(t)] =
          cfg.regulation_down_req * load_t;
    }
    uc.n_segments = v.n_segments;
    uc.segment_cols.assign(static_cast<size_t>(v.ng * v.T_commit * v.n_segments), -1);
    uc.segment_cap.assign(static_cast<size_t>(v.ng * v.n_segments), 0.0);
    if (v.T == v.T_commit) {
      for (int t = 0; t < T; ++t) {
        for (int g = 0; g < v.ng; ++g) {
          const auto& gen = inp.generators[static_cast<size_t>(g)];
          for (int k = 0; k < v.n_segments; ++k) {
            const size_t pos = static_cast<size_t>((t * v.ng + g) * v.n_segments + k);
            uc.segment_cols[pos] = v.SEG_start[static_cast<size_t>(k)] + g * T + t;
            if (k < static_cast<int>(gen.bid_segments.size())) {
              uc.segment_cap[static_cast<size_t>(g * v.n_segments + k)] =
                  std::max(0.0, gen.bid_segments[static_cast<size_t>(k)].quantity);
            }
          }
        }
      }
    }
    uc.certifies_power_balance_rows = true;
    uc.certifies_generation_capacity_rows = true;
    uc.certifies_ramping_rows = true;
    uc.certifies_min_up_down_rows = true;
    uc.certifies_segment_bound_rows = true;
    uc.certifies_system_reserve_rows = true;
    uc.certifies_storage_cycle_rows = (v.nstorage > 0);
    mip.uc_hint = std::move(uc);
  }

  // [Improvement (7)]: per-variable branching priorities for HiGHS/native B&B.
  // The UC state variable u(g,t) drives capacity, ramping, reserve, and the
  // min-up/down transition logic, so it gets the highest structural priority.
  // Startup/shutdown binaries encode transitions and are prioritized next.
  // Within each class, earlier periods dominate later periods.
  {
    const int n_vars = static_cast<int>(v.nx);
    const int T_c = v.T_commit;
    const int class_stride = T_c + 1;
    mip.branching_priority.assign(static_cast<size_t>(n_vars), 0);
    for (int g = 0; g < v.ng; ++g) {
      for (int t = 0; t < T_c; ++t) {
        const int time_prio = T_c - t;   // t=0 -> highest within class
        mip.branching_priority[static_cast<size_t>(v.IG_start + g * T_c + t)] =
            3 * class_stride + time_prio;
        mip.branching_priority[static_cast<size_t>(v.SU_start + g * T_c + t)] =
            2 * class_stride + time_prio;
        mip.branching_priority[static_cast<size_t>(v.SD_start + g * T_c + t)] =
            2 * class_stride + time_prio;
      }
    }
  }

  return f;
}

// ─────────────────────────────────────────────────────────────────────────────
// Extract result from solution vector
// ─────────────────────────────────────────────────────────────────────────────
SCUCSolveResult extract_result(
    const Eigen::VectorXd& x,
    const SCUCInput& inp,
    const Formulation& fm,
    bool converged,
    double objective)
{
  const VarIndex& v = fm.v;
  const int T = inp.config.num_periods;
  const int iph = v.intervals_per_hour;
  const double dt = inp.config.period_length_hr;

  auto zero2d = [](int r, int c) -> Matrix2D {
    return Matrix2D(static_cast<size_t>(r), std::vector<double>(static_cast<size_t>(c), 0.0));
  };

  SCUCSolveResult out;
  out.converged  = converged;
  out.objective  = objective;

  if (x.size() != static_cast<Eigen::Index>(v.nx)) return out;

  out.commitment = zero2d(v.ng, v.T_commit);
  out.startup    = zero2d(v.ng, v.T_commit);
  out.shutdown   = zero2d(v.ng, v.T_commit);
  out.dispatch   = zero2d(v.ng, T);
  out.spinning_reserve  = zero2d(v.ng, T);
  out.regulation_up     = zero2d(v.ng, T);
  out.regulation_down   = zero2d(v.ng, T);
  out.segment_dispatch.assign(static_cast<size_t>(v.n_segments), zero2d(v.ng, T));
  out.wind_generation  = zero2d(v.nw, T);
  out.solar_generation = zero2d(v.npv, T);
  out.storage_charging    = zero2d(v.nstorage, T);
  out.storage_discharging = zero2d(v.nstorage, T);
  out.storage_soc         = zero2d(v.nstorage, T);
  out.line_flows = zero2d(v.nl, T);
  out.wind_curtailment  = zero2d(v.nw, T);
  out.solar_curtailment = zero2d(v.npv, T);
  out.section_flows     = zero2d(v.nsec, T);
  out.load_shedding   = std::vector<double>(static_cast<size_t>(T), 0.0);
  out.gen_curtailment = std::vector<double>(static_cast<size_t>(T), 0.0);

  for (int g = 0; g < v.ng; ++g) {
    for (int h = 0; h < v.T_commit; ++h) {
      out.commitment[static_cast<size_t>(g)][static_cast<size_t>(h)] = x(v.IG_start + g * v.T_commit + h);
      out.startup[static_cast<size_t>(g)][static_cast<size_t>(h)]    = x(v.SU_start + g * v.T_commit + h);
      out.shutdown[static_cast<size_t>(g)][static_cast<size_t>(h)]   = x(v.SD_start + g * v.T_commit + h);
    }
    for (int t = 0; t < T; ++t) {
      out.dispatch[static_cast<size_t>(g)][static_cast<size_t>(t)] = x(v.PG_start + g * T + t);
      out.spinning_reserve[static_cast<size_t>(g)][static_cast<size_t>(t)] = x(v.RG_spin_start + g * T + t);
      out.regulation_up[static_cast<size_t>(g)][static_cast<size_t>(t)]    = x(v.RG_reg_up_start + g * T + t);
      out.regulation_down[static_cast<size_t>(g)][static_cast<size_t>(t)]  = x(v.RG_reg_down_start + g * T + t);
      for (int k = 0; k < v.n_segments; ++k)
        out.segment_dispatch[static_cast<size_t>(k)][static_cast<size_t>(g)][static_cast<size_t>(t)] =
            x(v.SEG_start[static_cast<size_t>(k)] + g * T + t);
    }
  }
  for (int w = 0; w < v.nw; ++w)
    for (int t = 0; t < T; ++t)
      out.wind_generation[static_cast<size_t>(w)][static_cast<size_t>(t)] = x(v.PW_start + w * T + t);
  for (int s = 0; s < v.npv; ++s)
    for (int t = 0; t < T; ++t)
      out.solar_generation[static_cast<size_t>(s)][static_cast<size_t>(t)] = x(v.PPV_start + s * T + t);
  for (int s = 0; s < v.nstorage; ++s) {
    for (int t = 0; t < T; ++t) {
      out.storage_charging[static_cast<size_t>(s)][static_cast<size_t>(t)]    = x(v.PSTO_IN_start + s * T + t);
      out.storage_discharging[static_cast<size_t>(s)][static_cast<size_t>(t)] = x(v.PSTO_OUT_start + s * T + t);
      out.storage_soc[static_cast<size_t>(s)][static_cast<size_t>(t)]         = x(v.SOC_start + s * T + t);
    }
  }

  // Line flows via PTDF
  if (fm.PTDF.rows() == v.nl && fm.PTDF.cols() == v.nb) {
    for (int t = 0; t < T; ++t) {
      Eigen::VectorXd inj = Eigen::VectorXd::Zero(v.nb);
      for (int g = 0; g < v.ng; ++g) {
        int b = inp.generators[static_cast<size_t>(g)].bus;
        if (b >= 0 && b < v.nb) inj(b) += out.dispatch[static_cast<size_t>(g)][static_cast<size_t>(t)];
      }
      for (int w = 0; w < v.nw; ++w) {
        int b = inp.wind[static_cast<size_t>(w)].bus;
        if (b >= 0 && b < v.nb) inj(b) += out.wind_generation[static_cast<size_t>(w)][static_cast<size_t>(t)];
      }
      for (int s = 0; s < v.npv; ++s) {
        int b = inp.solar[static_cast<size_t>(s)].bus;
        if (b >= 0 && b < v.nb) inj(b) += out.solar_generation[static_cast<size_t>(s)][static_cast<size_t>(t)];
      }
      for (int s = 0; s < v.nstorage; ++s) {
        int b = inp.storage[static_cast<size_t>(s)].bus;
        if (b >= 0 && b < v.nb) {
          inj(b) += x(v.PSTO_OUT_start + s * T + t);
          inj(b) -= x(v.PSTO_IN_start + s * T + t);
        }
      }
      for (int d = 0; d < static_cast<int>(inp.loads.size()); ++d) {
        int b = inp.loads[static_cast<size_t>(d)].bus;
        if (b >= 0 && b < v.nb) inj(b) -= load_at(inp, d, t);
      }
      for (int dcl = 0; dcl < v.ndc; ++dcl) {
        const auto& dc = inp.dc_lines[static_cast<size_t>(dcl)];
        const double pdc = x(v.PDC_start + dcl * T + t);
        if (dc.from >= 0 && dc.from < v.nb) inj(dc.from) -= pdc;
        if (dc.to   >= 0 && dc.to   < v.nb) inj(dc.to)   += pdc;
      }
      Eigen::VectorXd flows = fm.PTDF * inj;
      for (int l = 0; l < v.nl; ++l)
        out.line_flows[static_cast<size_t>(l)][static_cast<size_t>(t)] = flows(l);
      // Section flows via section PTDF
      if (fm.SEC_PTDF.rows() == v.nsec && fm.SEC_PTDF.cols() == v.nb) {
        Eigen::VectorXd sflows = fm.SEC_PTDF * inj;
        for (int s = 0; s < v.nsec; ++s)
          out.section_flows[static_cast<size_t>(s)][static_cast<size_t>(t)] = sflows(s);
      }
    }
  }

  // Wind / solar curtailment
  if (v.WIND_CURT_start < v.WIND_CURT_end) {
    for (int w = 0; w < v.nw; ++w)
      for (int t = 0; t < T; ++t)
        out.wind_curtailment[static_cast<size_t>(w)][static_cast<size_t>(t)] =
            x(v.WIND_CURT_start + w * T + t);
  } else {
    // Curtailment = forecast - actual when not explicitly tracked
    for (int w = 0; w < v.nw; ++w)
      for (int t = 0; t < T; ++t)
        out.wind_curtailment[static_cast<size_t>(w)][static_cast<size_t>(t)] =
            std::max(0.0, wind_at(inp, w, t) - out.wind_generation[static_cast<size_t>(w)][static_cast<size_t>(t)]);
  }
  if (v.SOL_CURT_start < v.SOL_CURT_end) {
    for (int s = 0; s < v.npv; ++s)
      for (int t = 0; t < T; ++t)
        out.solar_curtailment[static_cast<size_t>(s)][static_cast<size_t>(t)] =
            x(v.SOL_CURT_start + s * T + t);
  } else {
    for (int s = 0; s < v.npv; ++s)
      for (int t = 0; t < T; ++t)
        out.solar_curtailment[static_cast<size_t>(s)][static_cast<size_t>(t)] =
            std::max(0.0, solar_at(inp, s, t) - out.solar_generation[static_cast<size_t>(s)][static_cast<size_t>(t)]);
  }

  for (int t = 0; t < T; ++t) {
    out.load_shedding[static_cast<size_t>(t)]   = x(v.LOAD_SHED_start + t);
    out.gen_curtailment[static_cast<size_t>(t)] = x(v.GEN_CURT_start + t);
  }

  // Cost decomposition
  double ec = 0, su_c = 0, nl_c = 0, res_c = 0, pen = 0;
  for (int g = 0; g < v.ng; ++g) {
    const auto& gen = inp.generators[static_cast<size_t>(g)];
    for (int h = 0; h < v.T_commit; ++h) {
      su_c += out.startup[static_cast<size_t>(g)][static_cast<size_t>(h)] * gen.startup_cost;
      nl_c += out.commitment[static_cast<size_t>(g)][static_cast<size_t>(h)] * gen.no_load_cost * (1.0 / iph);
    }
    for (int t = 0; t < T; ++t) {
      for (int k = 0; k < v.n_segments; ++k) {
        double p = 0.0;
        if (k < static_cast<int>(gen.bid_segments.size()))
          p = gen.bid_segments[static_cast<size_t>(k)].price;
        ec += out.segment_dispatch[static_cast<size_t>(k)][static_cast<size_t>(g)][static_cast<size_t>(t)] * p * dt;
      }
      res_c += (out.spinning_reserve[static_cast<size_t>(g)][static_cast<size_t>(t)] * gen.spinning_reserve_price +
                out.regulation_up[static_cast<size_t>(g)][static_cast<size_t>(t)] * gen.regulation_up_price +
                out.regulation_down[static_cast<size_t>(g)][static_cast<size_t>(t)] * gen.regulation_down_price) * dt;
    }
  }
  for (int t = 0; t < T; ++t)
    pen += (out.load_shedding[static_cast<size_t>(t)] * inp.config.voll +
            out.gen_curtailment[static_cast<size_t>(t)] * inp.config.vocc) * dt;

  out.energy_cost  = ec;
  out.startup_cost = su_c;
  out.no_load_cost = nl_c;
  out.reserve_cost = res_c;
  out.penalty_cost = pen;
  out.total_cost   = ec + su_c + nl_c + res_c + pen;
  out.n_cuts_added = fm.n_cuts;

  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Solver factory: configure SolverEngine from config.solver name
// ─────────────────────────────────────────────────────────────────────────────
engine::api::Result run_milp(const engine::MIPModel& mip, const SCUCConfig& cfg) {
  engine::SolverEngine eng(false);
  const engine::BCOptions bc_opt = make_scuc_bc_options(cfg);
  eng.register_adapter(std::make_shared<engine::StrictHighsBranchAndCutAdapter>(bc_opt));
  eng.register_adapter(std::make_shared<engine::NativeBranchAndCutAdapter>(bc_opt));
  eng.register_default_adapters();

  engine::SolveOptions opts;
  if (cfg.solver != "Auto") opts.preferred_solver = cfg.solver;
  opts.allow_fallback = cfg.allow_fallback;

  // Pass time limit / gap as solver options via the preferred solver name.
  // The SolveOptions struct keeps these simple; detailed options go through
  // the BCOptions / adapter constructors. For now, rely on default settings
  // while respecting the solver preference.

  return eng.solve_milp(mip, opts);
}

/// LP solve that returns constraint duals (needed for LMP extraction).
/// Prefers solvers known to return duals: Gurobi, then native IPM/dual-simplex.
/// Falls back to the user-preferred solver if those are unavailable.
engine::api::Result run_lp_with_duals(const engine::LPModel& lp, const SCUCConfig& cfg) {
  engine::SolverEngine eng;
  eng.register_default_adapters();

  // Solvers that return constraint_duals from solve_lp():
  // NativeIPMLPAdapter, NativeBranchAndCut (LP path), Gurobi.
  // HiGHS file-based adapter does NOT return duals.
  const std::vector<std::string> dual_solvers = {
      "Gurobi", "NativeBranchAndCut", "NativeIPMLPAdapter"};

  auto available = eng.list_solvers(engine::ProblemClass::LP);
  std::string preferred;
  for (const auto& ds : dual_solvers) {
    if (std::find(available.begin(), available.end(), ds) != available.end()) {
      preferred = ds;
      break;
    }
  }
  if (preferred.empty() && cfg.solver != "Auto") preferred = cfg.solver;

  engine::SolveOptions opts;
  opts.preferred_solver = preferred;
  opts.allow_fallback = cfg.allow_fallback;

  return eng.solve_lp(lp, opts);
}

engine::api::Result run_lp(const engine::LPModel& lp, const SCUCConfig& cfg) {
  engine::SolverEngine eng;
  eng.register_default_adapters();

  engine::SolveOptions opts;
  if (cfg.solver != "Auto") opts.preferred_solver = cfg.solver;
  opts.allow_fallback = cfg.allow_fallback;

  return eng.solve_lp(lp, opts);
}

// ─────────────────────────────────────────────────────────────────────────────
// SCUC stage (MILP)
// ─────────────────────────────────────────────────────────────────────────────
SCUCSolveResult solve_scuc_stage(const SCUCInput& inp) {
  const auto tic = Clock::now();

  auto fm = build_formulation(inp);
  auto sol = run_milp(fm.mip, inp.config);

  auto out = extract_result(sol.x, inp, fm, sol.stats.success, sol.stats.objective);
  out.solver_name  = sol.stats.solver_name.empty() ? inp.config.solver : sol.stats.solver_name;
  out.mip_gap      = sol.stats.mip_gap;

  const auto toc = Clock::now();
  out.solve_time_sec =
      std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic).count();
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// SCED stage (LP with fixed commitment)
// ─────────────────────────────────────────────────────────────────────────────
SCUCSolveResult solve_sced_stage(const SCUCInput& inp, const SCUCSolveResult& scuc_res) {
  const auto tic = Clock::now();

  auto fm = build_formulation(inp);
  const VarIndex& v = fm.v;
  auto& vars = fm.mip.linear_part.vars;

  // Fix binaries to SCUC commitment
  for (int g = 0; g < v.ng; ++g) {
    for (int h = 0; h < v.T_commit; ++h) {
      auto fix = [&](int idx, double val) {
        vars[static_cast<size_t>(idx)].lb = val;
        vars[static_cast<size_t>(idx)].ub = val;
        vars[static_cast<size_t>(idx)].type = engine::VarType::Continuous;
      };
      double ig = 0.0, su = 0.0, sd = 0.0;
      if (g < static_cast<int>(scuc_res.commitment.size()) &&
          h < static_cast<int>(scuc_res.commitment[static_cast<size_t>(g)].size()))
        ig = std::round(scuc_res.commitment[static_cast<size_t>(g)][static_cast<size_t>(h)]);
      if (g < static_cast<int>(scuc_res.startup.size()) &&
          h < static_cast<int>(scuc_res.startup[static_cast<size_t>(g)].size()))
        su = std::round(scuc_res.startup[static_cast<size_t>(g)][static_cast<size_t>(h)]);
      if (g < static_cast<int>(scuc_res.shutdown.size()) &&
          h < static_cast<int>(scuc_res.shutdown[static_cast<size_t>(g)].size()))
        sd = std::round(scuc_res.shutdown[static_cast<size_t>(g)][static_cast<size_t>(h)]);
      fix(v.IG_start + g * v.T_commit + h, ig);
      fix(v.SU_start + g * v.T_commit + h, su);
      fix(v.SD_start + g * v.T_commit + h, sd);
    }
  }
  fm.mip.binary_idx.clear();
  fm.mip.integer_idx.clear();

  auto sol = run_lp(fm.mip.linear_part, inp.config);
  auto out = extract_result(sol.x, inp, fm, sol.stats.success, sol.stats.objective);
  out.solver_name = (sol.stats.solver_name.empty() ? inp.config.solver : sol.stats.solver_name) + " (SCED-LP)";

  const auto toc = Clock::now();
  out.solve_time_sec =
      std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic).count();
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// LMP stage (delta-neighbourhood LP, dual extraction)
// ─────────────────────────────────────────────────────────────────────────────
SCUCLMPResult solve_lmp_stage(const SCUCInput& inp, const SCUCSolveResult& sced_res) {
  const auto tic = Clock::now();
  SCUCLMPResult out;
  const int T = inp.config.num_periods;
  const double delta = inp.config.lmp_delta;

  auto fm = build_formulation(inp);
  const VarIndex& v = fm.v;
  auto& vars = fm.mip.linear_part.vars;

  // Fix binaries
  for (int g = 0; g < v.ng; ++g) {
    for (int h = 0; h < v.T_commit; ++h) {
      auto fix = [&](int idx, double val) {
        vars[static_cast<size_t>(idx)].lb = val;
        vars[static_cast<size_t>(idx)].ub = val;
        vars[static_cast<size_t>(idx)].type = engine::VarType::Continuous;
      };
      double ig = 0.0, su = 0.0, sd = 0.0;
      if (g < static_cast<int>(sced_res.commitment.size()) && h < static_cast<int>(sced_res.commitment[g].size()))
        ig = std::round(sced_res.commitment[static_cast<size_t>(g)][static_cast<size_t>(h)]);
      if (g < static_cast<int>(sced_res.startup.size()) && h < static_cast<int>(sced_res.startup[g].size()))
        su = std::round(sced_res.startup[static_cast<size_t>(g)][static_cast<size_t>(h)]);
      if (g < static_cast<int>(sced_res.shutdown.size()) && h < static_cast<int>(sced_res.shutdown[g].size()))
        sd = std::round(sced_res.shutdown[static_cast<size_t>(g)][static_cast<size_t>(h)]);
      fix(v.IG_start + g * v.T_commit + h, ig);
      fix(v.SU_start + g * v.T_commit + h, su);
      fix(v.SD_start + g * v.T_commit + h, sd);
    }
  }

  // Delta-neighbourhood bounds on PG
  for (int g = 0; g < v.ng; ++g) {
    const auto& gen = inp.generators[static_cast<size_t>(g)];
    for (int t = 0; t < T; ++t) {
      const int pg_idx = v.PG_start + g * T + t;
      const int h = t / v.intervals_per_hour;
      double ig = 0.0;
      if (g < static_cast<int>(sced_res.commitment.size()) && h < static_cast<int>(sced_res.commitment[g].size()))
        ig = std::round(sced_res.commitment[static_cast<size_t>(g)][static_cast<size_t>(h)]);
      if (ig < 0.5) {
        vars[static_cast<size_t>(pg_idx)].lb = 0.0;
        vars[static_cast<size_t>(pg_idx)].ub = 0.0;
        continue;
      }
      double pg_sced = 0.0;
      if (g < static_cast<int>(sced_res.dispatch.size()) && t < static_cast<int>(sced_res.dispatch[g].size()))
        pg_sced = sced_res.dispatch[static_cast<size_t>(g)][static_cast<size_t>(t)];
      double lb = std::max(gen.pmin, (1.0 - delta) * pg_sced);
      double ub = std::min(gen.pmax, (1.0 + delta) * pg_sced);
      if (lb > ub + 1e-6) { lb = gen.pmin; ub = gen.pmax; }
      vars[static_cast<size_t>(pg_idx)].lb = lb;
      vars[static_cast<size_t>(pg_idx)].ub = ub;
    }
  }

  fm.mip.binary_idx.clear();
  fm.mip.integer_idx.clear();

  auto sol = run_lp_with_duals(fm.mip.linear_part, inp.config);

  out.converged = sol.stats.success;
  if (!sol.stats.success) {
    out.solve_time_sec =
        std::chrono::duration_cast<std::chrono::duration<double>>(Clock::now() - tic).count();
    return out;
  }

  const int nb = v.nb;
  const int n_ineq = fm.n_ineq_rows;
  const auto& duals = sol.constraint_duals;
  const int m_total = static_cast<int>(duals.size());

  out.nodal_lmp      = Matrix2D(static_cast<size_t>(nb), std::vector<double>(static_cast<size_t>(T), 0.0));
  out.energy_lmp     = Matrix2D(static_cast<size_t>(nb), std::vector<double>(static_cast<size_t>(T), 0.0));
  out.congestion_lmp = Matrix2D(static_cast<size_t>(nb), std::vector<double>(static_cast<size_t>(T), 0.0));

  // SMP from power-balance equality duals
  std::vector<double> smp(static_cast<size_t>(T), 0.0);
  for (int t = 0; t < T; ++t) {
    const int row = n_ineq + fm.power_balance_eq_start + t;
    if (row >= 0 && row < m_total) smp[static_cast<size_t>(t)] = duals(row);
  }

  for (int t = 0; t < T; ++t) {
    for (int b = 0; b < nb; ++b) {
      double cong = 0.0;
      for (int l = 0; l < v.nl; ++l) {
        const int idx_fwd = (l * T + t < static_cast<int>(fm.line_fwd_row.size())) ? fm.line_fwd_row[static_cast<size_t>(l * T + t)] : -1;
        const int idx_rev = (l * T + t < static_cast<int>(fm.line_rev_row.size())) ? fm.line_rev_row[static_cast<size_t>(l * T + t)] : -1;
        double mu_fwd = (idx_fwd >= 0 && idx_fwd < m_total) ? -duals(idx_fwd) : 0.0;
        double mu_rev = (idx_rev >= 0 && idx_rev < m_total) ? -duals(idx_rev) : 0.0;
        if (fm.PTDF.rows() > l && fm.PTDF.cols() > b)
          cong += (mu_rev - mu_fwd) * fm.PTDF(l, b);
      }
      out.energy_lmp[static_cast<size_t>(b)][static_cast<size_t>(t)]     = smp[static_cast<size_t>(t)];
      out.congestion_lmp[static_cast<size_t>(b)][static_cast<size_t>(t)] = cong;
      out.nodal_lmp[static_cast<size_t>(b)][static_cast<size_t>(t)]      = smp[static_cast<size_t>(t)] + cong;
    }
  }

  double sum = 0, mn = 1e18, mx = -1e18;
  for (int b = 0; b < nb; ++b) for (int t = 0; t < T; ++t) {
    double v_lmp = out.nodal_lmp[static_cast<size_t>(b)][static_cast<size_t>(t)];
    sum += v_lmp;
    mn = std::min(mn, v_lmp);
    mx = std::max(mx, v_lmp);
  }
  out.avg_lmp = (nb * T > 0) ? sum / (nb * T) : 0.0;
  out.min_lmp = mn < 1e17 ? mn : 0.0;
  out.max_lmp = mx > -1e17 ? mx : 0.0;

  out.solve_time_sec =
      std::chrono::duration_cast<std::chrono::duration<double>>(Clock::now() - tic).count();
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// JSON helper: 2-D matrix → JSON array of arrays
// ─────────────────────────────────────────────────────────────────────────────
static json matrix2d_to_json(const Matrix2D& m) {
  json arr = json::array();
  for (const auto& row : m) arr.push_back(json(row));
  return arr;
}

static json solve_result_to_json(const SCUCSolveResult& r) {
  json j;
  j["converged"]     = r.converged;
  j["objective"]     = r.objective;
  j["solver_name"]   = r.solver_name;
  j["solve_time_sec"]= r.solve_time_sec;
  j["mip_gap"]       = r.mip_gap;
  j["n_cuts_added"]  = r.n_cuts_added;
  j["commitment"]    = matrix2d_to_json(r.commitment);
  j["startup"]       = matrix2d_to_json(r.startup);
  j["shutdown"]      = matrix2d_to_json(r.shutdown);
  j["dispatch"]      = matrix2d_to_json(r.dispatch);
  j["spinning_reserve"]  = matrix2d_to_json(r.spinning_reserve);
  j["regulation_up"]     = matrix2d_to_json(r.regulation_up);
  j["regulation_down"]   = matrix2d_to_json(r.regulation_down);
  j["wind_generation"]   = matrix2d_to_json(r.wind_generation);
  j["solar_generation"]  = matrix2d_to_json(r.solar_generation);
  j["storage_charging"]       = matrix2d_to_json(r.storage_charging);
  j["storage_discharging"]    = matrix2d_to_json(r.storage_discharging);
  j["storage_soc"]            = matrix2d_to_json(r.storage_soc);
  j["line_flows"]             = matrix2d_to_json(r.line_flows);
  j["load_shedding"]          = r.load_shedding;
  j["gen_curtailment"]        = r.gen_curtailment;
  j["cost"]["energy"]   = r.energy_cost;
  j["cost"]["startup"]  = r.startup_cost;
  j["cost"]["no_load"]  = r.no_load_cost;
  j["cost"]["reserve"]  = r.reserve_cost;
  j["cost"]["penalty"]  = r.penalty_cost;
  j["cost"]["total"]    = r.total_cost;
  return j;
}

}  // namespace (anonymous)

// ═════════════════════════════════════════════════════════════════════════════
// Public API implementations
// ═════════════════════════════════════════════════════════════════════════════

SCUCInput scuc_from_json(const std::string& json_str) {
  const json j = json::parse(json_str);
  SCUCInput inp;

  // ── Config ───────────────────────────────────────────────────────────────
  if (j.contains("config")) {
    const auto& jc = j["config"];
    auto gs = [&](const char* k, auto& v) { if (jc.contains(k)) v = jc[k].get<std::decay_t<decltype(v)>>(); };
    gs("solver",                    inp.config.solver);
    gs("allow_fallback",            inp.config.allow_fallback);
    gs("num_periods",               inp.config.num_periods);
    gs("period_length_hr",          inp.config.period_length_hr);
    gs("n_segments",                inp.config.n_segments);
    gs("mip_gap",                   inp.config.mip_gap);
    gs("time_limit_sec",            inp.config.time_limit_sec);
    gs("verbose",                   inp.config.verbose);
    gs("spinning_reserve_req",      inp.config.spinning_reserve_req);
    gs("regulation_up_req",         inp.config.regulation_up_req);
    gs("regulation_down_req",       inp.config.regulation_down_req);
    gs("voll",                      inp.config.voll);
    gs("vocc",                      inp.config.vocc);
    gs("renewable_min_output_coeff",inp.config.renewable_min_output_coeff);
    gs("enable_market_cuts",        inp.config.enable_market_cuts);
    gs("enable_primal_repair",      inp.config.enable_primal_repair);
    gs("M1_line_slack_penalty",            inp.config.M1_line_slack_penalty);
    gs("M2_renewable_curtail_penalty",      inp.config.M2_renewable_curtail_penalty);
    gs("neg_reserve_req",                   inp.config.neg_reserve_req);
    gs("pfr_reserve_req_mw",               inp.config.pfr_reserve_req_mw);
    gs("wheeling_fee_per_mwh",             inp.config.wheeling_fee_per_mwh);
    gs("solve_sced",                       inp.config.solve_sced);
    gs("solve_lmp",                        inp.config.solve_lmp);
    gs("lmp_delta",                        inp.config.lmp_delta);
  }

  // ── Generators ───────────────────────────────────────────────────────────
  if (j.contains("generators")) {
    for (const auto& jg : j["generators"]) {
      Generator g;
      auto gs2 = [&](const char* k, auto& v) { if (jg.contains(k)) v = jg[k].get<std::decay_t<decltype(v)>>(); };
      gs2("name",                  g.name);
      gs2("bus",                   g.bus);
      gs2("pmin",                  g.pmin);
      gs2("pmax",                  g.pmax);
      gs2("ramp_up_mw_min",        g.ramp_up_mw_min);
      gs2("ramp_dn_mw_min",        g.ramp_dn_mw_min);
      gs2("min_up_time_hr",        g.min_up_time_hr);
      gs2("min_dn_time_hr",        g.min_dn_time_hr);
      gs2("must_run",              g.must_run);
      gs2("max_startups",          g.max_startups);
      gs2("max_shutdowns",         g.max_shutdowns);
      gs2("startup_cost",          g.startup_cost);
      gs2("no_load_cost",          g.no_load_cost);
      gs2("spinning_reserve_price",g.spinning_reserve_price);
      gs2("regulation_up_price",   g.regulation_up_price);
      gs2("regulation_down_price", g.regulation_down_price);
      gs2("startup_cost_warm",      g.startup_cost_warm);
      gs2("startup_cost_cold",      g.startup_cost_cold);
      gs2("hot_start_threshold_hr", g.hot_start_threshold_hr);
      gs2("warm_start_threshold_hr",g.warm_start_threshold_hr);
      gs2("ud_periods",             g.ud_periods);
      gs2("dd_periods",             g.dd_periods);
      gs2("pfr_alpha",              g.pfr_alpha);
      gs2("group_id",              g.group_id);
      if (jg.contains("bid_segments")) {
        for (const auto& seg : jg["bid_segments"]) {
          BidSegment s;
          if (seg.contains("price"))    s.price    = seg["price"].get<double>();
          if (seg.contains("quantity")) s.quantity = seg["quantity"].get<double>();
          g.bid_segments.push_back(s);
        }
      }
      inp.generators.push_back(std::move(g));
    }
  }

  // ── Branches ─────────────────────────────────────────────────────────────
  if (j.contains("branches")) {
    for (const auto& jb : j["branches"]) {
      Branch b;
      if (jb.contains("from"))       b.from       = jb["from"].get<int>();
      if (jb.contains("to"))         b.to         = jb["to"].get<int>();
      if (jb.contains("reactance"))  b.reactance  = jb["reactance"].get<double>();
      if (jb.contains("rating_mw"))  b.rating_mw  = jb["rating_mw"].get<double>();
      if (jb.contains("in_service")) b.in_service = jb["in_service"].get<bool>();
      inp.branches.push_back(b);
    }
  }

  // ── Loads ────────────────────────────────────────────────────────────────
  if (j.contains("loads")) {
    for (const auto& jl : j["loads"]) {
      Load l;
      if (jl.contains("bus"))  l.bus  = jl["bus"].get<int>();
      if (jl.contains("p_mw")) l.p_mw = jl["p_mw"].get<double>();
      inp.loads.push_back(l);
    }
  }

  // ── Wind / Solar ─────────────────────────────────────────────────────────
  if (j.contains("wind")) {
    for (const auto& jw : j["wind"]) {
      WindUnit w;
      if (jw.contains("bus"))  w.bus  = jw["bus"].get<int>();
      if (jw.contains("pmax")) w.pmax = jw["pmax"].get<double>();
      inp.wind.push_back(w);
    }
  }
  if (j.contains("solar")) {
    for (const auto& js : j["solar"]) {
      SolarUnit s;
      if (js.contains("bus"))  s.bus  = js["bus"].get<int>();
      if (js.contains("pmax")) s.pmax = js["pmax"].get<double>();
      inp.solar.push_back(s);
    }
  }

  // ── Storage ──────────────────────────────────────────────────────────────
  if (j.contains("storage")) {
    for (const auto& js : j["storage"]) {
      StorageUnit s;
      auto gss = [&](const char* k, auto& v) { if (js.contains(k)) v = js[k].get<std::decay_t<decltype(v)>>(); };
      gss("bus",                   s.bus);
      gss("pmax_charge",           s.pmax_charge);
      gss("pmax_discharge",        s.pmax_discharge);
      gss("energy_capacity_mwh",   s.energy_capacity_mwh);
      gss("efficiency",            s.efficiency);
      gss("soc_init",              s.soc_init);
      gss("charge_bid_price",      s.charge_bid_price);
      gss("discharge_bid_price",   s.discharge_bid_price);
      gss("cycle_limit",           s.cycle_limit);
      gss("eta_charge",             s.eta_charge);
      gss("eta_discharge",          s.eta_discharge);
      gss("soc_min",                s.soc_min);
      gss("soc_final",              s.soc_final);
      gss("pmin_charge",            s.pmin_charge);
      gss("pmin_discharge",         s.pmin_discharge);
      gss("use_binary_indicators",  s.use_binary_indicators);
      inp.storage.push_back(s);
    }
  }

  // ── DC lines ─────────────────────────────────────────────────────────────
  if (j.contains("dc_lines")) {
    for (const auto& jd : j["dc_lines"]) {
      DCLine d;
      auto gsd = [&](const char* k, auto& v) { if (jd.contains(k)) v = jd[k].get<std::decay_t<decltype(v)>>(); };
      gsd("from",    d.from);
      gsd("to",      d.to);
      gsd("pmin",    d.pmin);
      gsd("pmax",    d.pmax);
      gsd("ramp_up", d.ramp_up);
      gsd("ramp_dn", d.ramp_dn);
      inp.dc_lines.push_back(d);
    }
  }

  // ── Profiles ─────────────────────────────────────────────────────────────
  if (j.contains("profiles")) {
    const auto& jp = j["profiles"];
    auto parse_matrix = [](const json& arr) -> std::vector<std::vector<double>> {
      std::vector<std::vector<double>> out;
      for (const auto& row : arr) out.push_back(row.get<std::vector<double>>());
      return out;
    };
    if (jp.contains("load"))  inp.profiles.load  = parse_matrix(jp["load"]);
    if (jp.contains("wind"))  inp.profiles.wind  = parse_matrix(jp["wind"]);
    if (jp.contains("solar")) inp.profiles.solar = parse_matrix(jp["solar"]);
  }

  // ── Initial status ────────────────────────────────────────────────────────
  // ── Generator groups ─────────────────────────────────────────────────────
  if (j.contains("generator_groups")) {
    for (const auto& jgrp : j["generator_groups"]) {
      GeneratorGroup grp;
      if (jgrp.contains("id"))   grp.id   = jgrp["id"].get<int>();
      if (jgrp.contains("name")) grp.name = jgrp["name"].get<std::string>();
      if (jgrp.contains("gen_indices")) grp.gen_indices = jgrp["gen_indices"].get<std::vector<int>>();
      if (jgrp.contains("pmin_t")) grp.pmin_t = jgrp["pmin_t"].get<std::vector<double>>();
      if (jgrp.contains("pmax_t")) grp.pmax_t = jgrp["pmax_t"].get<std::vector<double>>();
      if (jgrp.contains("emin"))  grp.emin = jgrp["emin"].get<double>();
      if (jgrp.contains("emax"))  grp.emax = jgrp["emax"].get<double>();
      inp.generator_groups.push_back(std::move(grp));
    }
  }

  // ── Sections ─────────────────────────────────────────────────────────────
  if (j.contains("sections")) {
    for (const auto& jsec : j["sections"]) {
      Section sec;
      if (jsec.contains("name")) sec.name = jsec["name"].get<std::string>();
      if (jsec.contains("rating_fwd_mw")) sec.rating_fwd_mw = jsec["rating_fwd_mw"].get<double>();
      if (jsec.contains("rating_rev_mw")) sec.rating_rev_mw = jsec["rating_rev_mw"].get<double>();
      if (jsec.contains("line_weights")) {
        for (const auto& lw : jsec["line_weights"]) {
          if (lw.is_array() && lw.size() >= 2)
            sec.line_weights.emplace_back(lw[0].get<int>(), lw[1].get<double>());
          else if (lw.is_object()) {
            int li = lw.contains("line") ? lw["line"].get<int>() : -1;
            double wt = lw.contains("weight") ? lw["weight"].get<double>() : 1.0;
            sec.line_weights.emplace_back(li, wt);
          }
        }
      }
      inp.sections.push_back(std::move(sec));
    }
  }

  if (j.contains("initial_status")) {
    const auto& ji = j["initial_status"];
    if (ji.contains("commitment"))   inp.initial_status.commitment  = ji["commitment"].get<std::vector<double>>();
    if (ji.contains("dispatch"))     inp.initial_status.dispatch    = ji["dispatch"].get<std::vector<double>>();
    if (ji.contains("storage_soc"))  inp.initial_status.storage_soc = ji["storage_soc"].get<std::vector<double>>();
    if (ji.contains("time_in_state")) inp.initial_status.time_in_state = ji["time_in_state"].get<std::vector<double>>();
  }

  // ── Infer num_buses if not provided ──────────────────────────────────────
  if (j.contains("num_buses")) {
    inp.num_buses = j["num_buses"].get<int>();
  } else {
    int max_bus = 0;
    for (const auto& g : inp.generators) max_bus = std::max(max_bus, g.bus);
    for (const auto& br : inp.branches)  { max_bus = std::max(max_bus, br.from); max_bus = std::max(max_bus, br.to); }
    for (const auto& l : inp.loads)      max_bus = std::max(max_bus, l.bus);
    for (const auto& w : inp.wind)       max_bus = std::max(max_bus, w.bus);
    for (const auto& s : inp.solar)      max_bus = std::max(max_bus, s.bus);
    for (const auto& s : inp.storage)    max_bus = std::max(max_bus, s.bus);
    inp.num_buses = std::max(1, max_bus + 1);
  }

  return inp;
}

SCUCOutput scuc_solve(const SCUCInput& input) {
  SCUCOutput out;

  // Stage 1: SCUC (MILP)
  out.scuc = solve_scuc_stage(input);

  // Stage 2: SCED (LP with fixed commitment)
  if (input.config.solve_sced && out.scuc.converged) {
    out.sced = solve_sced_stage(input, out.scuc);
  }

  // Stage 3: LMP (delta-neighbourhood LP, dual extraction)
  const auto& ref = out.sced.converged ? out.sced : out.scuc;
  if (input.config.solve_lmp && ref.converged) {
    out.lmp = solve_lmp_stage(input, ref);
  }

  return out;
}

std::string scuc_output_to_json(const SCUCOutput& output,
                                const SCUCInput& input,
                                int indent) {
  json j;

  // Echo back minimal input metadata for traceability
  j["meta"]["solver"]       = input.config.solver;
  j["meta"]["num_periods"]  = input.config.num_periods;
  j["meta"]["num_buses"]    = input.num_buses;
  j["meta"]["num_generators"] = static_cast<int>(input.generators.size());
  j["meta"]["num_branches"]   = static_cast<int>(input.branches.size());

  // SCUC result
  j["scuc"] = solve_result_to_json(output.scuc);

  // SCED result (only included if it was run)
  if (input.config.solve_sced) {
    j["sced"] = solve_result_to_json(output.sced);
  }

  // LMP result
  if (input.config.solve_lmp) {
    json jl;
    jl["converged"]     = output.lmp.converged;
    jl["solve_time_sec"]= output.lmp.solve_time_sec;
    jl["avg_lmp"]       = output.lmp.avg_lmp;
    jl["max_lmp"]       = output.lmp.max_lmp;
    jl["min_lmp"]       = output.lmp.min_lmp;
    jl["nodal"]         = matrix2d_to_json(output.lmp.nodal_lmp);
    jl["energy"]        = matrix2d_to_json(output.lmp.energy_lmp);
    jl["congestion"]    = matrix2d_to_json(output.lmp.congestion_lmp);
    j["lmp"] = std::move(jl);
  }

  return indent >= 0 ? j.dump(indent) : j.dump();
}

engine::MIPModel build_scuc_mip(const SCUCInput& inp) {
  return build_formulation(inp).mip;
}

}  // namespace mipsolvers::scuc
