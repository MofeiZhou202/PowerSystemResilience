#include "mipsolvers/engine/solver/native/lp/pdlp_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/util/problem_validation.hpp"

namespace mipsolvers::engine {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// ===================== CSR built from Eigen CSC =====================
struct CSRMatrix {
  int rows{0}, cols{0}, nnz{0};
  std::vector<int> row_ptr;
  std::vector<int> col_idx;
  std::vector<double> val;
};

CSRMatrix build_csr(const Eigen::SparseMatrix<double>& csc) {
  CSRMatrix r;
  r.rows = static_cast<int>(csc.rows());
  r.cols = static_cast<int>(csc.cols());
  r.nnz  = static_cast<int>(csc.nonZeros());
  r.row_ptr.assign(r.rows + 1, 0);
  r.col_idx.resize(r.nnz);
  r.val.resize(r.nnz);

  const int* outer = csc.outerIndexPtr();
  const int* inner = csc.innerIndexPtr();
  const double* v  = csc.valuePtr();

  for (int k = 0; k < r.nnz; ++k) ++r.row_ptr[inner[k] + 1];
  for (int i = 0; i < r.rows; ++i) r.row_ptr[i + 1] += r.row_ptr[i];

  std::vector<int> pos(r.row_ptr.begin(), r.row_ptr.begin() + r.rows);
  for (int j = 0; j < r.cols; ++j) {
    for (int k = outer[j]; k < outer[j + 1]; ++k) {
      const int i = inner[k];
      const int p = pos[i]++;
      r.col_idx[p] = j;
      r.val[p] = v[k];
    }
  }
  return r;
}

// ===================== Ruiz row+column equilibration =====================
// M → Dr·M·Dc.  row_bounds *= Dr, c *= Dc, var_bounds /= Dc.
// cum_dc accumulates column scaling for unscaling: x_orig = cum_dc .* x_scaled.
/// @brief Ruiz row+column equilibration with early termination.
/// @return Number of rounds actually performed.
int ruiz_equilibrate(double* csc_val, const int* csc_inner, const int* csc_outer,
                     double* csr_val, const int* csr_col, const int* csr_rp,
                     int m, int n, int nnz,
                     double* c, double* vl, double* vu,
                     double* rl, double* ru,
                     double* cum_dc, int rounds) {
  if (rounds <= 0 || nnz == 0) return 0;
  std::vector<double> rm(m), cm(n);

  // Early-stop threshold: if max deviation of scaling factors from 1.0
  // is below this, further rounds are unlikely to help.
  constexpr double kConvergenceTol = 1e-3;

  int rounds_done = 0;
  for (int rnd = 0; rnd < rounds; ++rnd) {
    std::fill(rm.begin(), rm.end(), 0.0);
    std::fill(cm.begin(), cm.end(), 0.0);
    for (int j = 0; j < n; ++j) {
      for (int k = csc_outer[j]; k < csc_outer[j + 1]; ++k) {
        const double av = std::abs(csc_val[k]);
        const int i = csc_inner[k];
        if (av > rm[i]) rm[i] = av;
        if (av > cm[j]) cm[j] = av;
      }
    }

    // Early termination: check if row/column maxima are already ~1.0.
    double max_dev = 0.0;
    for (int i = 0; i < m; ++i)
      if (rm[i] > 0.0) max_dev = std::max(max_dev, std::abs(rm[i] - 1.0));
    for (int j = 0; j < n; ++j)
      if (cm[j] > 0.0) max_dev = std::max(max_dev, std::abs(cm[j] - 1.0));
    if (rnd >= 2 && max_dev < kConvergenceTol) {
      rounds_done = rnd;
      break;
    }

    for (int i = 0; i < m; ++i) {
      const double dr = 1.0 / std::sqrt(std::max(rm[i], 1e-12));
      if (std::isfinite(rl[i])) rl[i] *= dr;
      if (std::isfinite(ru[i])) ru[i] *= dr;
      rm[i] = dr;
    }
    for (int j = 0; j < n; ++j) {
      const double dc = 1.0 / std::sqrt(std::max(cm[j], 1e-12));
      c[j] *= dc;
      if (std::isfinite(vl[j])) vl[j] /= dc;
      if (std::isfinite(vu[j])) vu[j] /= dc;
      cum_dc[j] *= dc;
      cm[j] = dc;
    }
    for (int j = 0; j < n; ++j) {
      const double dc_j = cm[j];
      for (int k = csc_outer[j]; k < csc_outer[j + 1]; ++k)
        csc_val[k] *= rm[csc_inner[k]] * dc_j;
    }
    for (int i = 0; i < m; ++i) {
      const double dr_i = rm[i];
      for (int k = csr_rp[i]; k < csr_rp[i + 1]; ++k)
        csr_val[k] *= dr_i * cm[csr_col[k]];
    }
    rounds_done = rnd + 1;
  }
  return rounds_done;
}

// ===================== Convergence helpers =====================

double primal_viol_inf(const double* ax, const double* rl, const double* ru, int m) {
  double w = 0.0;
  for (int i = 0; i < m; ++i) {
    if (std::isfinite(rl[i])) { double v = rl[i] - ax[i]; if (v > w) w = v; }
    if (std::isfinite(ru[i])) { double v = ax[i] - ru[i]; if (v > w) w = v; }
  }
  return w;
}

double dual_stat_inf(const double* g, const double* x,
                     const double* vl, const double* vu, int n) {
  constexpr double kTol = 1e-9;
  double w = 0.0;
  for (int j = 0; j < n; ++j) {
    double v;
    if (x[j] <= vl[j] + kTol) v = std::max(0.0, -g[j]);
    else if (x[j] >= vu[j] - kTol) v = std::max(0.0, g[j]);
    else v = std::abs(g[j]);
    if (v > w) w = v;
  }
  return w;
}

double support_interval(const double* rl, const double* ru, const double* y, int m) {
  double v = 0.0;
  for (int i = 0; i < m; ++i) {
    if (y[i] >= 0.0) {
      if (!std::isfinite(ru[i])) return kInf;
      v += y[i] * ru[i];
    } else {
      if (!std::isfinite(rl[i])) return kInf;
      v += y[i] * rl[i];
    }
  }
  return v;
}

double rel_gap(double p, double d) {
  return std::abs(p - d) / std::max({1.0, std::abs(p), std::abs(d)});
}

}  // namespace

// ===================== Adapter interface =====================

NativePDLPAdapter::NativePDLPAdapter(PDLPOptions opt) : opt_(std::move(opt)) {}
std::string NativePDLPAdapter::name() const { return "NativePDLP"; }
bool NativePDLPAdapter::supports(ProblemClass cls) const { return cls == ProblemClass::LP; }

SolveResult NativePDLPAdapter::solve_lp(const LPModel& prob) const {
  const auto t0 = std::chrono::steady_clock::now();
  SolveResult out;
  out.stats.solver_name = name();

  const ValidationReport vr = validate(prob);
  if (!vr.valid) {
    out.stats.status = vr.errors.empty() ? "Invalid LP" : vr.errors.front();
    return out;
  }

  const int n = static_cast<int>(prob.c.size());
  const int m_ineq = static_cast<int>(prob.A.rows());
  const int m_eq   = static_cast<int>(prob.Aeq.rows());
  const int m = m_ineq + m_eq;
  const bool is_min = (prob.sense == Sense::Minimize);
  const double* orig_c = prob.c.data();

  // --- Scaled problem vectors (owned, modified by Ruiz) ---
  std::vector<double> c_min(n), vl(n), vu(n);
  for (int j = 0; j < n; ++j) {
    c_min[j] = is_min ? orig_c[j] : -orig_c[j];
    vl[j] = prob.vars[static_cast<size_t>(j)].lb;
    vu[j] = prob.vars[static_cast<size_t>(j)].ub;
  }

  // --- Build combined constraint matrix M (CSC via Eigen) ---
  Eigen::SparseMatrix<double> M(m, n);
  {
    std::vector<Eigen::Triplet<double>> trips;
    trips.reserve(static_cast<size_t>(prob.A.nonZeros() + prob.Aeq.nonZeros()));
    for (int j = 0; j < prob.A.outerSize(); ++j)
      for (Eigen::SparseMatrix<double>::InnerIterator it(prob.A, j); it; ++it)
        trips.emplace_back(static_cast<int>(it.row()), static_cast<int>(it.col()), it.value());
    for (int j = 0; j < prob.Aeq.outerSize(); ++j)
      for (Eigen::SparseMatrix<double>::InnerIterator it(prob.Aeq, j); it; ++it)
        trips.emplace_back(m_ineq + static_cast<int>(it.row()), static_cast<int>(it.col()), it.value());
    M.setFromTriplets(trips.begin(), trips.end());
    M.makeCompressed();
  }

  // --- Row bounds ---
  std::vector<double> rl(m, -kInf), ru(m, kInf);
  for (int i = 0; i < m_ineq; ++i) ru[i] = prob.b[i];
  for (int i = 0; i < m_eq; ++i) {
    rl[m_ineq + i] = prob.beq[i];
    ru[m_ineq + i] = prob.beq[i];
  }

  const int nnz = static_cast<int>(M.nonZeros());

  // --- Trivial zero-structure case ---
  if (nnz == 0) {
    out.x.resize(n);
    for (int j = 0; j < n; ++j) {
      if (c_min[j] > 0) out.x[j] = vl[j];
      else if (c_min[j] < 0) out.x[j] = vu[j];
      else out.x[j] = 0.5 * (vl[j] + vu[j]);
    }
    bool feas = true;
    for (int i = 0; i < m && feas; ++i) {
      if (std::isfinite(rl[i]) && 0.0 < rl[i] - 1e-12) feas = false;
      if (std::isfinite(ru[i]) && 0.0 > ru[i] + 1e-12) feas = false;
    }
    out.stats.success = feas;
    out.stats.status = feas ? "Optimal" : "Infeasible";
    out.stats.objective = prob.c.dot(out.x);
    out.stats.runtime_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    return out;
  }

  // --- Build CSR from CSC ---
  CSRMatrix csr = build_csr(M);

  // --- Ruiz row+column equilibration ---
  std::vector<double> cum_dc(n, 1.0);
  ruiz_equilibrate(M.valuePtr(), M.innerIndexPtr(), M.outerIndexPtr(),
                   csr.val.data(), csr.col_idx.data(), csr.row_ptr.data(),
                   m, n, nnz,
                   c_min.data(), vl.data(), vu.data(),
                   rl.data(), ru.data(),
                   cum_dc.data(), opt_.ruiz_rounds);

  // --- Raw CSC/CSR pointers (stable after makeCompressed + in-place value edits) ---
  const double* csc_v  = M.valuePtr();
  const int*    csc_in = M.innerIndexPtr();
  const int*    csc_ou = M.outerIndexPtr();
  const int*    csr_rp = csr.row_ptr.data();
  const int*    csr_ci = csr.col_idx.data();
  const double* csr_v  = csr.val.data();

  // --- Pock-Chambolle diagonal preconditioning ---
  std::vector<double> col_asum(n, 0.0), row_asum(m, 0.0);
  for (int j = 0; j < n; ++j)
    for (int k = csc_ou[j]; k < csc_ou[j + 1]; ++k) {
      const double av = std::abs(csc_v[k]);
      col_asum[j] += av;
      row_asum[csc_in[k]] += av;
    }

  std::vector<double> tau(n), sigma(m);
  // Initialize primal_weight from the ratio of variable range to constraint scale.
  // This helps convergence on problems with mixed-scale variables (e.g., UC).
  double primal_weight = 1.0;
  {
    double avg_col = 0.0, avg_row = 0.0;
    int nc = 0, nr = 0;
    for (int j = 0; j < n; ++j) if (col_asum[j] > 0) { avg_col += col_asum[j]; ++nc; }
    for (int i = 0; i < m; ++i) if (row_asum[i] > 0) { avg_row += row_asum[i]; ++nr; }
    if (nc > 0 && nr > 0) {
      avg_col /= nc; avg_row /= nr;
      primal_weight = std::sqrt(avg_row / std::max(avg_col, 1e-12));
      primal_weight = std::max(0.01, std::min(100.0, primal_weight));
    }
  }
  // --- IP-PMM regularization parameters (must precede refresh_steps) ---
  const bool use_pmm = opt_.use_pmm_regularization;
  double rho = use_pmm ? opt_.pmm_rho_init : 0.0;
  double delta = use_pmm ? opt_.pmm_delta_init : 0.0;

  double step_gain = 1.0;
  auto refresh_steps = [&]() {
    const double eff = opt_.step_scale * step_gain;
    for (int j = 0; j < n; ++j)
      tau[j] = eff * primal_weight / std::max(col_asum[j] + rho, 1e-9);
    for (int i = 0; i < m; ++i)
      sigma[i] = eff / (primal_weight * std::max(row_asum[i] + delta, 1e-9));
  };
  refresh_steps();

  // --- Primal / dual initialisation ---
  std::vector<double> x(n), x_prev(n), x_bar(n), y(m, 0.0);
  std::vector<double> grad(n), ax_buf(m);

  // --- Ergodic (running-average) iterates for smoother convergence ---
  std::vector<double> x_avg(n), y_avg(m, 0.0);
  double avg_count = 0.0;

  for (int j = 0; j < n; ++j) {
    if (std::isfinite(vl[j]) && std::isfinite(vu[j]))
      x[j] = 0.5 * (vl[j] + vu[j]);
    else if (std::isfinite(vl[j])) x[j] = vl[j];
    else if (std::isfinite(vu[j])) x[j] = vu[j];
    else x[j] = 0.0;
    x_prev[j] = x[j];
    x_avg[j] = x[j];
  }

  // --- IP-PMM step-size regularization state ---
  // ρ and δ are added to step-size denominators to prevent blowup on
  // near-zero columns/rows, then decayed as the solver converges.
  double prev_pf = kInf, prev_df = kInf;

  // --- Best-iterate tracking ---
  double best_metric = kInf, best_pf = kInf, best_df = kInf, best_gap = kInf;
  std::vector<double> best_x = x;

  int iters = 0;
  bool converged = false;
  double restart_ref = kInf;
  bool do_extrap = true;

  // --- Local raw pointers for hot loop ---
  double*       xp   = x.data();
  double*       xpp  = x_prev.data();
  double*       xbp  = x_bar.data();
  double*       yp   = y.data();
  double*       gp   = grad.data();
  double*       axp  = ax_buf.data();
  const double* cp   = c_min.data();
  const double* vlp  = vl.data();
  const double* vup  = vu.data();
  const double* rlp  = rl.data();
  const double* rup  = ru.data();
  const double* tp   = tau.data();
  const double* sp   = sigma.data();

  // ===================== MAIN PDHG LOOP =====================
  for (int iter = 1; iter <= opt_.max_iter; ++iter) {
    iters = iter;

    // --- Extrapolation: x_bar = 2x - x_prev  (or just x after restart) ---
    const double* xb_use;
    if (do_extrap) {
      for (int j = 0; j < n; ++j)
        xbp[j] = 2.0 * xp[j] - xpp[j];
      xb_use = xbp;
    } else {
      xb_use = xp;
    }

    // --- Fused dual step: CSR row-pass over M ---
    //   dot_i = row_i(M) · x_bar
    //   q = y[i] + σ[i]·dot_i
    //   proj = clamp(q/σ, rl, ru)
    //   y[i] = q − σ·proj
    for (int i = 0; i < m; ++i) {
      double dot = 0.0;
      const int end = csr_rp[i + 1];
      for (int k = csr_rp[i]; k < end; ++k)
        dot += csr_v[k] * xb_use[csr_ci[k]];
      const double si = sp[i];
      const double q = yp[i] + si * dot;
      double proj = q / si;
      if (proj < rlp[i]) proj = rlp[i];
      if (proj > rup[i]) proj = rup[i];
      yp[i] = q - si * proj;
    }

    // --- Save x → x_prev, then fused primal step: CSC col-pass over M ---
    //   dot_j = col_j(M)^T · y  (= (M^T y)[j])
    //   grad_j = c[j] + dot_j
    //   x[j] = clamp(x[j] − τ[j]·grad_j, vl, vu)
    // Note: IP-PMM step-size regularization (ρ, δ in τ/σ denominators) provides
    // conditioning benefit without perturbing the gradient (see refresh_steps).
    std::memcpy(xpp, xp, static_cast<size_t>(n) * sizeof(double));
    for (int j = 0; j < n; ++j) {
      double dot = 0.0;
      const int end = csc_ou[j + 1];
      for (int k = csc_ou[j]; k < end; ++k)
        dot += csc_v[k] * yp[csc_in[k]];
      const double g = cp[j] + dot;
      gp[j] = g;
      double xnew = xp[j] - tp[j] * g;
      if (xnew < vlp[j]) xnew = vlp[j];
      if (xnew > vup[j]) xnew = vup[j];
      xp[j] = xnew;
    }

    // --- Ergodic average update ---
    avg_count += 1.0;
    {
      const double aw = 1.0 / avg_count;
      for (int j = 0; j < n; ++j) x_avg[j] += aw * (xp[j] - x_avg[j]);
      for (int i = 0; i < m; ++i) y_avg[i] += aw * (yp[i] - y_avg[i]);
    }

    // --- Periodic convergence check ---
    const bool do_check = (iter == opt_.max_iter) ||
        (opt_.check_interval > 0 && (iter % opt_.check_interval) == 0);
    if (!do_check) { do_extrap = true; continue; }

    // Compute ax = M·x via CSR (only at check iterations)
    for (int i = 0; i < m; ++i) {
      double dot = 0.0;
      const int end = csr_rp[i + 1];
      for (int k = csr_rp[i]; k < end; ++k)
        dot += csr_v[k] * xp[csr_ci[k]];
      axp[i] = dot;
    }

    const double pf = primal_viol_inf(axp, rlp, rup, m);
    const double df = dual_stat_inf(gp, xp, vlp, vup, n);

    // Duality gap
    double gap = kInf;
    {
      double pobj = 0.0;
      for (int j = 0; j < n; ++j) pobj += cp[j] * xp[j];
      const double sr = support_interval(rlp, rup, yp, m);
      if (std::isfinite(sr)) {
        double sv = 0.0;
        for (int j = 0; j < n; ++j) {
          const double sj = -gp[j];
          sv += (sj >= 0.0) ? sj * vup[j] : sj * vlp[j];
        }
        gap = rel_gap(pobj, -sv - sr);
      }
    }

    double metric = std::max(pf, df);
    if (std::isfinite(gap)) metric = std::max(metric, gap);

    if (metric < best_metric) {
      best_metric = metric;
      best_x.assign(xp, xp + n);
      best_pf  = pf;
      best_df  = df;
      best_gap = gap;
    }

    // --- Evaluate ergodic average (smoother, better worst-case rate) ---
    double avg_pf = kInf, avg_df = kInf, avg_gap = kInf;
    if (avg_count >= 20.0) {
      // ax_avg = M * x_avg via CSR (reuse axp)
      for (int i = 0; i < m; ++i) {
        double dot = 0.0;
        for (int k = csr_rp[i]; k < csr_rp[i + 1]; ++k)
          dot += csr_v[k] * x_avg[static_cast<size_t>(csr_ci[k])];
        axp[i] = dot;
      }
      avg_pf = primal_viol_inf(axp, rlp, rup, m);
      // grad_avg and dual stationarity in one CSC pass
      double wd = 0.0;
      double pobj_a = 0.0;
      for (int j = 0; j < n; ++j) {
        double dot = 0.0;
        for (int k = csc_ou[j]; k < csc_ou[j + 1]; ++k)
          dot += csc_v[k] * y_avg[static_cast<size_t>(csc_in[k])];
        const double ga = cp[j] + dot;
        double v;
        if (x_avg[j] <= vlp[j] + 1e-9) v = std::max(0.0, -ga);
        else if (x_avg[j] >= vup[j] - 1e-9) v = std::max(0.0, ga);
        else v = std::abs(ga);
        if (v > wd) wd = v;
        pobj_a += cp[j] * x_avg[j];
        gp[j] = ga;
      }
      avg_df = wd;
      const double sr_a = support_interval(rlp, rup, y_avg.data(), m);
      if (std::isfinite(sr_a)) {
        double sv_a = 0.0;
        for (int j = 0; j < n; ++j) {
          const double sj = -gp[j];
          sv_a += (sj >= 0.0) ? sj * vup[j] : sj * vlp[j];
        }
        avg_gap = rel_gap(pobj_a, -sv_a - sr_a);
      }
      double am = std::max(avg_pf, avg_df);
      if (std::isfinite(avg_gap)) am = std::max(am, avg_gap);
      if (am < best_metric) {
        best_metric = am;
        best_x = x_avg;
        best_pf = avg_pf;
        best_df = avg_df;
        best_gap = avg_gap;
      }
    }

    if (opt_.verbose) {
      std::printf("PDLP %6d: pf=%.3e df=%.3e gap=%.3e\n",
                  iter, pf, df, std::isfinite(gap) ? gap : 999.0);
    }

    // --- Adaptive restart ---
    if (!std::isfinite(restart_ref)) restart_ref = metric;
    bool did_restart = false;
    if (std::isfinite(metric) && metric < opt_.restart_threshold * restart_ref) {
      restart_ref = metric;
      did_restart = true;
    } else if (opt_.restart_interval > 0 && (iter % opt_.restart_interval) == 0) {
      restart_ref = metric;
      did_restart = true;
    }

    // --- Step-gain adaptation ---
    if (std::isfinite(metric) && std::isfinite(restart_ref) && restart_ref > 0.0) {
      if (metric < 0.90 * restart_ref) {
        step_gain = std::min(1.35, step_gain * 1.01);
        refresh_steps();
        tp = tau.data(); sp = sigma.data();
      } else if (metric > 1.10 * restart_ref) {
        step_gain = std::max(0.35, step_gain * 0.995);
        refresh_steps();
        tp = tau.data(); sp = sigma.data();
      }
    }

    // --- Primal-weight balancing ---
    if (pf > 0.0 && df > 0.0) {
      const double ratio = pf / std::max(df, 1e-12);
      if (ratio > opt_.primal_weight_update_threshold ||
          ratio < 1.0 / opt_.primal_weight_update_threshold) {
        primal_weight *= std::sqrt(std::max(1e-4, std::min(1e4, ratio)));
        primal_weight = std::max(1e-3, std::min(1e3, primal_weight));
        refresh_steps();
        tp = tau.data(); sp = sigma.data();
      }
    }

    // --- IP-PMM: Adaptive step-size regularization decay ---
    // (Inspired by Pougkakiotis & Gondzio 2021, adapted to PDHG)
    // The regularization ρ/δ in the step-size denominators prevents step-size
    // blowup on near-zero columns/rows. As convergence progresses, we decay
    // the regularization so it doesn't limit the final step sizes.
    if (use_pmm && std::isfinite(pf) && std::isfinite(df)) {
      const double old_rho = rho, old_delta = delta;

      // Faster decay when making progress, slower guaranteed decay otherwise
      if (pf <= opt_.pmm_update_threshold * prev_pf ||
          df <= opt_.pmm_update_threshold * prev_df) {
        rho *= 0.8;
        delta *= 0.8;
      } else {
        rho *= 0.97;
        delta *= 0.97;
      }
      rho = std::max(rho, opt_.pmm_reg_min);
      delta = std::max(delta, opt_.pmm_reg_min);

      // Refresh step sizes if regularization changed significantly
      if (rho < 0.5 * old_rho || delta < 0.5 * old_delta) {
        refresh_steps();
        tp = tau.data(); sp = sigma.data();
      }

      prev_pf = pf;
      prev_df = df;
    }

    do_extrap = !did_restart;
    if (did_restart) {
      avg_count = 0.0;
      x_avg.assign(xp, xp + n);
      y_avg.assign(yp, yp + m);
    }

    // --- convergence test (current OR averaged iterate) ---
    const bool gap_ok = !std::isfinite(gap) || gap <= opt_.tol_gap;
    const bool avg_gap_ok = !std::isfinite(avg_gap) || avg_gap <= opt_.tol_gap;
    if ((pf <= opt_.tol_primal && df <= opt_.tol_dual && gap_ok) ||
        (avg_pf <= opt_.tol_primal && avg_df <= opt_.tol_dual && avg_gap_ok)) {
      converged = true;
      break;
    }
    if (!std::isfinite(pf) || !std::isfinite(df)) break;
  }

  // ===================== UNSCALE & OUTPUT =====================
  out.x.resize(n);
  for (int j = 0; j < n; ++j)
    out.x[j] = cum_dc[j] * best_x[j];

  // Compute unscaled primal feasibility against original problem.
  double unscaled_pf = 0.0;
  {
    if (m_ineq > 0) {
      Eigen::VectorXd ax_ineq = prob.A * out.x;
      for (int i = 0; i < m_ineq; ++i) {
        double viol = ax_ineq[i] - prob.b[i];
        if (viol > unscaled_pf) unscaled_pf = viol;
      }
    }
    if (m_eq > 0) {
      Eigen::VectorXd ax_eq = prob.Aeq * out.x;
      for (int i = 0; i < m_eq; ++i) {
        double viol = std::abs(ax_eq[i] - prob.beq[i]);
        if (viol > unscaled_pf) unscaled_pf = viol;
      }
    }
  }

  out.stats.objective  = prob.c.dot(out.x);
  out.stats.iterations = iters;
  out.stats.primal_feas     = unscaled_pf;
  out.stats.dual_feas       = best_df;
  out.stats.complementarity = std::isfinite(best_gap) ? best_gap : 0.0;
  out.stats.residual_inf    = std::max(unscaled_pf, best_df);

  const bool bg_ok = !std::isfinite(best_gap) || best_gap <= opt_.tol_gap;
  out.stats.success = converged ||
      (best_pf <= opt_.tol_primal && best_df <= opt_.tol_dual && bg_ok);
  out.stats.status = out.stats.success ? "Optimal" : "Max iterations reached";

  out.stats.runtime_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t0).count();
  return out;
}

}  // namespace mipsolvers::engine
