/// Security-Constrained Unit Commitment — standalone MILP formulation.
///
/// Ported from HybridACDCPowerSystemsPlanning/src/analysis/market_simulation.cpp.
/// All HybridACDCPF dependencies removed; uses mipsolvers::engine::SolverEngine.

#include "mipsolvers/scuc/scuc.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <numeric>
#include <stdexcept>

#include <Eigen/LU>
#include <Eigen/Sparse>
#include <nlohmann/json.hpp>

#include "mipsolvers/engine/engine.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/api/result.hpp"

namespace mipsolvers::scuc {

using json = nlohmann::json;
using Clock = std::chrono::steady_clock;

namespace {

constexpr double kEps = 1e-9;

inline double clampd(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

// ─────────────────────────────────────────────────────────────────────────────
// Variable index — flat variable layout for SCUC MILP
// ─────────────────────────────────────────────────────────────────────────────
struct VarIndex {
  int ng{0}, nb{0}, nl{0}, nw{0}, npv{0}, nstorage{0}, ndc{0};
  int T{24}, T_commit{24}, n_segments{3}, intervals_per_hour{1};

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
  int PW_start{0}, PW_end{0};
  int PPV_start{0}, PPV_end{0};
  int PSTO_IN_start{0}, PSTO_IN_end{0};
  int PSTO_OUT_start{0}, PSTO_OUT_end{0};
  int SOC_start{0}, SOC_end{0};
  int PDC_start{0}, PDC_end{0};

  int n_areas{1};
  int LOAD_SHED_start{0}, LOAD_SHED_end{0};
  int GEN_CURT_start{0}, GEN_CURT_end{0};

  int SL_LINE_POS_start{0}, SL_LINE_POS_end{0};
  int SL_LINE_NEG_start{0}, SL_LINE_NEG_end{0};
  int n_sections{0};
  int SL_SEC_POS_start{0}, SL_SEC_POS_end{0};
  int SL_SEC_NEG_start{0}, SL_SEC_NEG_end{0};

  int nx{0};

  int idx(int base, int unit, int period, int n_units) const {
    return base + unit * T + period;
    (void)n_units;  // kept for documentation
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
  v.T = cfg.num_periods;
  v.n_segments = std::max(1, cfg.n_segments);
  v.intervals_per_hour = std::max(1, static_cast<int>(std::llround(1.0 / std::max(1e-6, cfg.period_length_hr))));
  v.T_commit = std::max(1, v.T / v.intervals_per_hour);

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
  alloc(v.nw * v.T, v.PW_start, v.PW_end);
  alloc(v.npv * v.T, v.PPV_start, v.PPV_end);
  alloc(v.nstorage * v.T, v.PSTO_IN_start, v.PSTO_IN_end);
  alloc(v.nstorage * v.T, v.PSTO_OUT_start, v.PSTO_OUT_end);
  alloc(v.nstorage * v.T, v.SOC_start, v.SOC_end);
  alloc(v.ndc * v.T, v.PDC_start, v.PDC_end);

  v.n_areas = 1;
  alloc(v.n_areas * v.T, v.LOAD_SHED_start, v.LOAD_SHED_end);
  alloc(v.n_areas * v.T, v.GEN_CURT_start, v.GEN_CURT_end);

  alloc(v.nl * v.T, v.SL_LINE_POS_start, v.SL_LINE_POS_end);
  alloc(v.nl * v.T, v.SL_LINE_NEG_start, v.SL_LINE_NEG_end);
  v.n_sections = 0;
  alloc(0, v.SL_SEC_POS_start, v.SL_SEC_POS_end);
  alloc(0, v.SL_SEC_NEG_start, v.SL_SEC_NEG_end);

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

// ─────────────────────────────────────────────────────────────────────────────
// Formulation: builds MIPModel for SCUC (or SCED when binaries fixed)
// ─────────────────────────────────────────────────────────────────────────────
struct Formulation {
  engine::MIPModel mip;
  VarIndex v;
  Eigen::MatrixXd PTDF;
  int n_ineq_rows{0};
  int n_eq_rows{0};
  int power_balance_eq_start{0};
  std::vector<int> line_fwd_row;  // size = nl*T, -1 = skipped
  std::vector<int> line_rev_row;
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

  // ── Objective ────────────────────────────────────────────────────────────
  for (int g = 0; g < v.ng; ++g) {
    const auto& gen = inp.generators[static_cast<size_t>(g)];
    for (int t = 0; t < T; ++t) {
      for (int k = 0; k < v.n_segments; ++k) {
        const int idx = v.SEG_start[static_cast<size_t>(k)] + g * T + t;
        double price = 0.0;
        if (k < static_cast<int>(gen.bid_segments.size()))
          price = gen.bid_segments[static_cast<size_t>(k)].price;
        lp.c(idx) = price * dt;
      }
      lp.c(v.RG_spin_start + g * T + t) = gen.spinning_reserve_price * dt;
      lp.c(v.RG_reg_up_start + g * T + t) = gen.regulation_up_price * dt;
      lp.c(v.RG_reg_down_start + g * T + t) = gen.regulation_down_price * dt;
    }
    for (int h = 0; h < v.T_commit; ++h) {
      lp.c(v.SU_start + g * v.T_commit + h) += gen.startup_cost;
      lp.c(v.IG_start + g * v.T_commit + h) += gen.no_load_cost * (1.0 / iph);
    }
  }

  // LOAD_SHED / GEN_CURT penalty in objective
  const double M1 = cfg.M1_line_slack_penalty;
  for (int t = 0; t < T; ++t) {
    lp.c(v.LOAD_SHED_start + t) = cfg.voll * dt;
    lp.vars[static_cast<size_t>(v.LOAD_SHED_start + t)].ub = 1e6;
    lp.c(v.GEN_CURT_start + t) = cfg.vocc * dt;
    lp.vars[static_cast<size_t>(v.GEN_CURT_start + t)].ub = 1e6;
  }
  for (int i = v.SL_LINE_POS_start; i < v.SL_LINE_POS_end; ++i) { lp.c(i) = M1; lp.vars[static_cast<size_t>(i)].ub = 1e6; }
  for (int i = v.SL_LINE_NEG_start; i < v.SL_LINE_NEG_end; ++i) { lp.c(i) = M1; lp.vars[static_cast<size_t>(i)].ub = 1e6; }

  // Storage bid price costs
  for (int s = 0; s < v.nstorage; ++s) {
    const auto& sto = inp.storage[static_cast<size_t>(s)];
    if (std::abs(sto.charge_bid_price) < kEps && std::abs(sto.discharge_bid_price) < kEps) continue;
    for (int t = 0; t < T; ++t) {
      lp.c(v.PSTO_IN_start + s * T + t) += sto.charge_bid_price * dt;
      lp.c(v.PSTO_OUT_start + s * T + t) += sto.discharge_bid_price * dt;
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

  // ── PG = Pmin*IG + Σ segments ─────────────────────────────────────────────
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

      // PG <= Pmax * IG
      add_le({{pg, 1.0}, {ig, -pmax}}, 0.0);
      // PG >= Pmin * IG  →  -PG + Pmin*IG <= 0
      add_le({{pg, -1.0}, {ig, pmin}}, 0.0);

      // Segment capacity upper bounds
      for (int k = 0; k < v.n_segments; ++k) {
        const auto& seg_idx = v.SEG_start[static_cast<size_t>(k)] + g * T + t;
        const auto& gen = inp.generators[static_cast<size_t>(g)];
        double q = 0.0;
        if (k < static_cast<int>(gen.bid_segments.size()))
          q = gen.bid_segments[static_cast<size_t>(k)].quantity;
        lp.vars[static_cast<size_t>(seg_idx)].ub = std::max(0.0, q);
      }
    }
  }

  // ── Commitment transitions ────────────────────────────────────────────────
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

      if (gen.must_run) {
        lp.vars[static_cast<size_t>(ig)].lb = 1.0;
        lp.vars[static_cast<size_t>(ig)].ub = 1.0;
      }
    }
  }

  // ── Min up/down time ─────────────────────────────────────────────────────
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

  // ── Max startups/shutdowns ────────────────────────────────────────────────
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

  // ── Ramping constraints ───────────────────────────────────────────────────
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

  // ── Renewable variable bounds ─────────────────────────────────────────────
  const double alpha_n = cfg.renewable_min_output_coeff;
  for (int t = 0; t < T; ++t) {
    for (int w = 0; w < v.nw; ++w) {
      const double fw = wind_at(inp, w, t);
      const int idx = v.PW_start + w * T + t;
      lp.vars[static_cast<size_t>(idx)].lb = std::max(0.0, alpha_n * fw);
      lp.vars[static_cast<size_t>(idx)].ub = std::max(0.0, fw);
    }
    for (int s = 0; s < v.npv; ++s) {
      const double fs = solar_at(inp, s, t);
      const int idx = v.PPV_start + s * T + t;
      lp.vars[static_cast<size_t>(idx)].lb = std::max(0.0, alpha_n * fs);
      lp.vars[static_cast<size_t>(idx)].ub = std::max(0.0, fs);
    }
  }

  // ── Storage constraints ───────────────────────────────────────────────────
  for (int s = 0; s < v.nstorage; ++s) {
    const auto& sto = inp.storage[static_cast<size_t>(s)];
    const double p_ch = std::max(0.0, sto.pmax_charge);
    const double p_dis = std::max(0.0, sto.pmax_discharge);
    const double e_cap = std::max(0.0, sto.energy_capacity_mwh);
    const double eta_rt = clampd(sto.efficiency, 0.1, 1.0);
    const double eta = std::sqrt(eta_rt);

    double soc0 = sto.soc_init * e_cap;
    if (s < static_cast<int>(inp.initial_status.storage_soc.size())) {
      const double iv = std::max(0.0, inp.initial_status.storage_soc[static_cast<size_t>(s)]);
      soc0 = (iv <= 1.0 + 1e-8) ? iv * e_cap : iv;
    }
    soc0 = clampd(soc0, 0.0, e_cap);

    for (int t = 0; t < T; ++t) {
      const int p_in = v.PSTO_IN_start + s * T + t;
      const int p_out = v.PSTO_OUT_start + s * T + t;
      const int soc = v.SOC_start + s * T + t;

      lp.vars[static_cast<size_t>(p_in)] = {engine::VarType::Continuous, 0.0, p_ch, {}};
      lp.vars[static_cast<size_t>(p_out)] = {engine::VarType::Continuous, 0.0, p_dis, {}};
      lp.vars[static_cast<size_t>(soc)] = {engine::VarType::Continuous, 0.1 * e_cap, e_cap, {}};

      if (t == 0) {
        add_eq({{soc, 1.0}, {p_in, -eta * dt},
                {p_out, dt / std::max(eta, 1e-6)}}, soc0);
      } else {
        const int soc_prev = v.SOC_start + s * T + (t - 1);
        add_eq({{soc, 1.0}, {soc_prev, -1.0},
                {p_in, -eta * dt}, {p_out, dt / std::max(eta, 1e-6)}}, 0.0);
      }
    }
    // Final SOC >= initial SOC
    if (T > 0) {
      const int soc_T = v.SOC_start + s * T + (T - 1);
      add_le({{soc_T, -1.0}}, -soc0);
    }
    // Simultaneous charge/discharge prevention
    const double p_max_sim = std::max(p_ch, p_dis);
    for (int t = 0; t < T; ++t)
      add_le({{v.PSTO_IN_start + s * T + t, 1.0},
              {v.PSTO_OUT_start + s * T + t, 1.0}}, p_max_sim);

    // Cycle limit
    if (sto.cycle_limit > kEps && e_cap > kEps) {
      std::vector<std::pair<int,double>> row;
      for (int t = 0; t < T; ++t) {
        row.emplace_back(v.PSTO_OUT_start + s * T + t, dt / std::max(eta, 1e-6));
        row.emplace_back(v.PSTO_IN_start + s * T + t, eta * dt);
      }
      add_le(row, 2.0 * sto.cycle_limit * e_cap);
    }
  }

  // ── DC line limits and ramping ────────────────────────────────────────────
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

  // ── System power balance ─────────────────────────────────────────────────
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
      row.emplace_back(v.PSTO_OUT_start + s * T + t, 1.0);
      row.emplace_back(v.PSTO_IN_start + s * T + t, -1.0);
    }
    row.emplace_back(v.LOAD_SHED_start + t, 1.0);   // shed reduces effective demand
    row.emplace_back(v.GEN_CURT_start + t, -1.0);   // curtailment absorbs surplus

    double load_t = 0.0;
    for (int d = 0; d < static_cast<int>(inp.loads.size()); ++d)
      load_t += load_at(inp, d, t);
    add_eq(row, load_t);
  }

  // ── PTDF-based line flow limits ───────────────────────────────────────────
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

  if (PTDF.rows() == v.nl && PTDF.cols() == nb) {
    for (int l = 0; l < v.nl; ++l) {
      const auto& br = inp.branches[static_cast<size_t>(l)];
      const double lim = std::max(0.0, br.rating_mw);
      if (!br.in_service || lim > 1e5) continue;  // unconstrained

      for (int t = 0; t < T; ++t) {
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
            fwd.emplace_back(v.PSTO_OUT_start + s * T + t, ptdf);
            fwd.emplace_back(v.PSTO_IN_start + s * T + t, -ptdf);
            rev.emplace_back(v.PSTO_OUT_start + s * T + t, -ptdf);
            rev.emplace_back(v.PSTO_IN_start + s * T + t, ptdf);
          }
          for (int d : loads_at[static_cast<size_t>(b)])
            load_ptdf += ptdf * load_at(inp, d, t);
        }
        // DC contribution
        for (int dcl = 0; dcl < v.ndc; ++dcl) {
          const auto& dc = inp.dc_lines[static_cast<size_t>(dcl)];
          if (dc.from < 0 || dc.from >= nb || dc.to < 0 || dc.to >= nb) continue;
          const double coeff = PTDF(l, dc.to) - PTDF(l, dc.from);
          if (std::abs(coeff) > 1e-12) {
            fwd.emplace_back(v.PDC_start + dcl * T + t, coeff);
            rev.emplace_back(v.PDC_start + dcl * T + t, -coeff);
          }
        }

        // Slack variables
        fwd.emplace_back(v.SL_LINE_POS_start + l * T + t, -1.0);
        rev.emplace_back(v.SL_LINE_NEG_start + l * T + t, -1.0);

        f.line_fwd_row[static_cast<size_t>(l * T + t)] = static_cast<int>(b_vec.size());
        add_le(fwd, lim + load_ptdf);
        f.line_rev_row[static_cast<size_t>(l * T + t)] = static_cast<int>(b_vec.size());
        add_le(rev, lim - load_ptdf);
      }
    }
  }

  // ── Reserve requirements ──────────────────────────────────────────────────
  for (int t = 0; t < T; ++t) {
    double load_t = 0.0;
    for (int d = 0; d < static_cast<int>(inp.loads.size()); ++d)
      load_t += load_at(inp, d, t);

    const double spin_req = cfg.spinning_reserve_req * load_t;
    const double rup_req  = cfg.regulation_up_req * load_t;
    const double rdn_req  = cfg.regulation_down_req * load_t;

    std::vector<std::pair<int,double>> spin_row, rup_row, rdn_row;

    for (int g = 0; g < v.ng; ++g) {
      const auto& gen = inp.generators[static_cast<size_t>(g)];
      const double pmax = gen.pmax, pmin = gen.pmin;
      const int h = t / iph;
      const int pg = v.PG_start + g * T + t;
      const int ig = v.IG_start + g * v.T_commit + h;
      const int rs = v.RG_spin_start + g * T + t;
      const int ru = v.RG_reg_up_start + g * T + t;
      const int rd = v.RG_reg_down_start + g * T + t;

      spin_row.emplace_back(rs, -1.0);
      rup_row.emplace_back(ru, -1.0);
      rdn_row.emplace_back(rd, -1.0);

      add_le({{pg, 1.0}, {rs, 1.0}, {ru, 1.0}, {ig, -pmax}}, 0.0);
      add_le({{rd, 1.0}, {pg, -1.0}, {ig, pmin}}, 0.0);
      add_le({{rs, 1.0}, {ig, -(pmax - pmin)}}, 0.0);

      const double r10 = gen.ramp_up_mw_min * 10.0;
      if (r10 > kEps) add_le({{ru, 1.0}, {ig, -r10}}, 0.0);
      const double r10d = gen.ramp_dn_mw_min * 10.0;
      if (r10d > kEps) add_le({{rd, 1.0}, {ig, -r10d}}, 0.0);
    }

    if (spin_req > 0.0) add_le(spin_row, -spin_req);
    if (rup_req  > 0.0) add_le(rup_row,  -rup_req);
    if (rdn_req  > 0.0) add_le(rdn_row,  -rdn_req);
  }

  // ── LP-valid pre-formulation market cutting planes ─────────────────────────
  int n_cuts = 0;
  if (cfg.enable_market_cuts) {
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

    // Family A: segment commitment coupling
    for (int g = 0; g < v.ng; ++g) {
      const auto& gen = inp.generators[static_cast<size_t>(g)];
      for (int t = 0; t < T; ++t) {
        const int h = t / iph;
        const int ig = v.IG_start + g * v.T_commit + h;
        for (int m = 0; m < v.n_segments; ++m) {
          double q = 0.0;
          if (m < static_cast<int>(gen.bid_segments.size()))
            q = gen.bid_segments[static_cast<size_t>(m)].quantity;
          if (q < kEps) continue;
          const int seg = v.SEG_start[static_cast<size_t>(m)] + g * T + t;
          add_le({{seg, 1.0}, {ig, -q}}, 0.0);
          ++n_cuts;
        }
      }
    }

    // Family G: extended startup clique
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

    // Family 11: cyclic SOC bound for storage
    for (int s = 0; s < v.nstorage; ++s) {
      const auto& sto = inp.storage[static_cast<size_t>(s)];
      const double e_cap = std::max(0.0, sto.energy_capacity_mwh);
      if (e_cap < kEps) continue;
      const double eta = std::sqrt(clampd(sto.efficiency, 0.1, 1.0));
      const double soc0 = clampd(sto.soc_init, 0.0, 1.0) * e_cap;
      const double rhs = eta * sto.pmax_charge * static_cast<double>(T) * dt + soc0;
      if (rhs < kEps) continue;
      std::vector<std::pair<int,double>> row;
      for (int t = 0; t < T; ++t)
        row.emplace_back(v.PSTO_OUT_start + s * T + t, dt / std::max(eta, 1e-6));
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

  // UC hint for native B&C
  {
    engine::MIPModel::UCGenHint uc;
    uc.ng = v.ng; uc.T = v.T_commit;
    uc.pg_start = v.PG_start;
    uc.ig_start = v.IG_start; uc.su_start = v.SU_start; uc.sd_start = v.SD_start;
    uc.min_up.resize(static_cast<size_t>(v.ng));
    uc.min_down.resize(static_cast<size_t>(v.ng));
    uc.ig0.resize(static_cast<size_t>(v.ng));
    uc.pmin.resize(static_cast<size_t>(v.ng));
    uc.pmax.resize(static_cast<size_t>(v.ng));
    uc.ramp.resize(static_cast<size_t>(v.ng));
    for (int g = 0; g < v.ng; ++g) {
      const auto& gen = inp.generators[static_cast<size_t>(g)];
      uc.min_up[static_cast<size_t>(g)] = std::max(0, static_cast<int>(std::ceil(gen.min_up_time_hr)));
      uc.min_down[static_cast<size_t>(g)] = std::max(0, static_cast<int>(std::ceil(gen.min_dn_time_hr)));
      uc.ig0[static_cast<size_t>(g)] =
          (g < static_cast<int>(inp.initial_status.commitment.size()) &&
           inp.initial_status.commitment[static_cast<size_t>(g)] > 0.5) ? 1 : 0;
      uc.pmin[static_cast<size_t>(g)] = gen.pmin;
      uc.pmax[static_cast<size_t>(g)] = gen.pmax;
      uc.ramp[static_cast<size_t>(g)] = std::max(gen.ramp_up_mw_min, gen.ramp_dn_mw_min) * 60.0 * dt;
    }
    mip.uc_hint = std::move(uc);
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
    }
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
  engine::SolverEngine eng;
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
    gs("M1_line_slack_penalty",     inp.config.M1_line_slack_penalty);
    gs("solve_sced",                inp.config.solve_sced);
    gs("solve_lmp",                 inp.config.solve_lmp);
    gs("lmp_delta",                 inp.config.lmp_delta);
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
  if (j.contains("initial_status")) {
    const auto& ji = j["initial_status"];
    if (ji.contains("commitment"))  inp.initial_status.commitment  = ji["commitment"].get<std::vector<double>>();
    if (ji.contains("dispatch"))    inp.initial_status.dispatch    = ji["dispatch"].get<std::vector<double>>();
    if (ji.contains("storage_soc")) inp.initial_status.storage_soc = ji["storage_soc"].get<std::vector<double>>();
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

}  // namespace mipsolvers::scuc
