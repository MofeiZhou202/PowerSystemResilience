/// \file two_stage.cpp
/// \brief L-shaped/Benders and Column-and-Constraint Generation for two-stage
///        stochastic and robust optimization.
///
/// Derivation and cut formulas: docs/two_stage_decomposition_design.md.
/// References: Van Slyke & Wets (1969); Benders (1962); Birge & Louveaux,
/// Introduction to Stochastic Programming 2nd ed. §5.1/§5.4; Zeng & Zhao,
/// Oper. Res. Lett. 41(5) (2013).

#include "mipsolvers/engine/decomposition/two_stage.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "mipsolvers/engine/api/solver.hpp"

namespace mipsolvers::engine::decomposition {
namespace {

using Clock = std::chrono::steady_clock;
using Triplet = Eigen::Triplet<double>;
constexpr double kInf = std::numeric_limits<double>::infinity();

/// Clamp the requested worker count to [1, n_items].
int resolve_workers(int requested, int n_items) {
  int w = requested > 0 ? requested : 1;
  return std::min(w, std::max(1, n_items));
}

/// A fixed set of independent SolverEngine instances, one per worker thread.
/// Each engine owns its own adapter instances, so concurrent solves touch no
/// shared mutable state (the engine already runs solves concurrently in its
/// portfolio/parallel B&C paths). engines[0] also serves the serial phases.
struct WorkerPool {
  std::vector<std::unique_ptr<SolverEngine>> engines;

  explicit WorkerPool(int workers) {
    const int w = std::max(1, workers);
    engines.reserve(static_cast<std::size_t>(w));
    for (int i = 0; i < w; ++i) {
      engines.push_back(std::make_unique<SolverEngine>());
      engines.back()->register_default_adapters();
    }
  }

  SolverEngine& main() const { return *engines.front(); }
  int size() const { return static_cast<int>(engines.size()); }

  /// Run fn(engine, i) for i in [0, n). Sequential (engines[0]) when size()==1,
  /// bit-identical to the pre-parallel path; otherwise dynamically balanced.
  template <class F>
  void map(int n, F&& fn) const {
    if (size() <= 1) {
      for (int i = 0; i < n; ++i) fn(*engines.front(), i);
      return;
    }
    std::atomic<int> next{0};
    std::vector<std::thread> pool;
    pool.reserve(static_cast<std::size_t>(size()));
    for (int w = 0; w < size(); ++w) {
      pool.emplace_back([&, w] {
        for (int i = next.fetch_add(1); i < n; i = next.fetch_add(1))
          fn(*engines[static_cast<std::size_t>(w)], i);
      });
    }
    for (auto& t : pool) t.join();
  }
};

double relative_gap(double upper, double lower) {
  if (!std::isfinite(upper) || !std::isfinite(lower)) return kInf;
  return std::max(0.0, upper - lower) / std::max(1.0, std::abs(upper));
}

bool has_integrality(const std::vector<VariableMeta>& vars) {
  for (const auto& v : vars)
    if (v.type != VarType::Continuous) return true;
  return false;
}

/// Append integer/binary column indices of `vars` (shifted by `offset`).
void collect_integrality(const std::vector<VariableMeta>& vars, int offset,
                         std::vector<int>& integer_idx,
                         std::vector<int>& binary_idx) {
  for (int j = 0; j < static_cast<int>(vars.size()); ++j) {
    if (vars[static_cast<std::size_t>(j)].type == VarType::Integer)
      integer_idx.push_back(offset + j);
    else if (vars[static_cast<std::size_t>(j)].type == VarType::Binary)
      binary_idx.push_back(offset + j);
  }
}

/// Copy the nonzeros of `M` (scaled) into `trips` at the given row/col offset.
void add_block(std::vector<Triplet>& trips, const Eigen::SparseMatrix<double>& M,
               int row_off, int col_off, double scale) {
  for (int k = 0; k < M.outerSize(); ++k)
    for (Eigen::SparseMatrix<double>::InnerIterator it(M, k); it; ++it)
      trips.emplace_back(row_off + static_cast<int>(it.row()),
                         col_off + static_cast<int>(it.col()),
                         scale * it.value());
}

/// One accumulated master inequality: sum_j coeff_j * col_j <= rhs.
struct CutRow {
  std::vector<std::pair<int, double>> coeffs;
  double rhs{0.0};
};

/// Prefer a native LP solver that returns constraint duals (LMP/Benders duals),
/// matching src/scuc/scuc.cpp::run_lp_with_duals. Empty => engine default.
std::string pick_dual_lp_solver(const SolverEngine& eng,
                                const std::string& user) {
  if (!user.empty()) return user;
  const auto avail = eng.list_solvers(ProblemClass::LP);
  for (const char* name : {"NativeBranchAndCut", "NativeIPMLP"}) {
    if (std::find(avail.begin(), avail.end(), name) != avail.end())
      return name;
  }
  return {};
}

/// Solve an LP/MILP built as an LPModel plus integrality index sets.
api::Result solve_generic(const SolverEngine& eng, const LPModel& lp,
                          const std::vector<int>& integer_idx,
                          const std::vector<int>& binary_idx,
                          const std::string& solver, double time_limit_sec) {
  SolveOptions opts;
  opts.preferred_solver = solver;
  opts.allow_fallback = true;
  if (time_limit_sec > 0.0) opts.time_limit_sec = time_limit_sec;
  if (integer_idx.empty() && binary_idx.empty()) return eng.solve_lp(lp, opts);
  MIPModel mip;
  mip.linear_part = lp;
  mip.integer_idx = integer_idx;
  mip.binary_idx = binary_idx;
  return eng.solve_milp(mip, opts);
}

/// Assemble an LPModel from base first-stage rows plus accumulated cuts.
///   variables: [x (n1) | epigraph columns] with objective `obj`.
///   inequality: [first.A | 0] x <= first.b, then each CutRow;
///   equality:   [first.Aeq | 0] x  = first.beq.
LPModel assemble_master(const FirstStage& first, int n_extra_cols,
                        const Eigen::VectorXd& obj,
                        const std::vector<VariableMeta>& extra_vars,
                        const std::vector<CutRow>& cuts) {
  const int n1 = static_cast<int>(first.c.size());
  const int ncols = n1 + n_extra_cols;

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = obj;
  lp.vars = first.vars;
  lp.vars.insert(lp.vars.end(), extra_vars.begin(), extra_vars.end());

  const int base_ineq = static_cast<int>(first.A.rows());
  const int n_cuts = static_cast<int>(cuts.size());
  std::vector<Triplet> ineq;
  add_block(ineq, first.A, 0, 0, 1.0);
  Eigen::VectorXd b(base_ineq + n_cuts);
  b.head(base_ineq) = first.b;
  for (int i = 0; i < n_cuts; ++i) {
    for (const auto& [col, val] : cuts[static_cast<std::size_t>(i)].coeffs)
      ineq.emplace_back(base_ineq + i, col, val);
    b[base_ineq + i] = cuts[static_cast<std::size_t>(i)].rhs;
  }
  Eigen::SparseMatrix<double> A(base_ineq + n_cuts, ncols);
  A.setFromTriplets(ineq.begin(), ineq.end());
  A.makeCompressed();
  lp.A = std::move(A);
  lp.b = std::move(b);

  const int base_eq = static_cast<int>(first.Aeq.rows());
  std::vector<Triplet> eq;
  add_block(eq, first.Aeq, 0, 0, 1.0);
  Eigen::SparseMatrix<double> Aeq(base_eq, ncols);
  Aeq.setFromTriplets(eq.begin(), eq.end());
  Aeq.makeCompressed();
  lp.Aeq = std::move(Aeq);
  lp.beq = first.beq;
  return lp;
}

/// Result of a fixed-x recourse solve for cut generation.
struct RecourseSolve {
  bool solved{false};        ///< solver returned a usable answer (optimal or infeasible)
  bool feasible{false};      ///< recourse (R_s) feasible at x
  double q{0.0};             ///< Q_s(x) when feasible
  Eigen::VectorXd y;         ///< recourse optimizer when feasible
  Eigen::VectorXd g;         ///< subgradient of Q_s (optimality) or w_s (feasibility): T_s^T mu
  double w{0.0};             ///< elastic infeasibility w_s(x) (>0 when infeasible)
  std::string status;
};

/// Solve recourse (R_s) at fixed x and, when feasible, form the optimality
/// subgradient g = T_s^T mu; when infeasible, solve the elastic feasibility
/// subproblem (FEAS_s) and form the feasibility subgradient. See design doc
/// §2.1–§2.2. `mu` is the engine dual of the negated (<=) coupling rows, so
/// grad Q_s = T_s^T mu directly.
RecourseSolve solve_recourse_for_cut(const SolverEngine& eng,
                                     const Recourse& r,
                                     const Eigen::VectorXd& x,
                                     const std::string& lp_solver,
                                     double time_limit_sec) {
  RecourseSolve out;
  const int m = static_cast<int>(r.W.rows());
  const int n = static_cast<int>(r.W.cols());
  const Eigen::VectorXd rhs = r.T * x - r.h;  // b_sub = T x - h for -W y <= T x - h

  // Primal recourse: min d'y s.t. (-W) y <= T x - h, y in [l,u].
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = r.d;
  lp.vars = r.vars;
  Eigen::SparseMatrix<double> negW = (-1.0) * r.W;
  negW.makeCompressed();
  lp.A = negW;
  lp.b = rhs;
  lp.Aeq.resize(0, n);
  lp.beq.resize(0);

  SolveOptions opts;
  opts.preferred_solver = lp_solver;
  opts.allow_fallback = true;
  if (time_limit_sec > 0.0) opts.time_limit_sec = time_limit_sec;
  api::Result res = eng.solve_lp(lp, opts);

  if (res.stats.success && res.x.size() == n) {
    if (res.constraint_duals.size() < m) {
      out.status = "recourse LP returned no dual certificate";
      return out;  // fail loudly upstream
    }
    out.solved = true;
    out.feasible = true;
    out.q = res.stats.objective;
    out.y = res.x;
    // grad Q_s(x) = T_s^T mu  (mu = duals of the <= rows).  Design doc §2.1.
    out.g = r.T.transpose() * res.constraint_duals.head(m);
    return out;
  }

  // Elastic feasibility subproblem (FEAS_s): min 1'v s.t. W y + v >= h - T x.
  // Engine <= form over [y (n) | v (m)]: [-W | -I][y;v] <= T x - h.  Birge &
  // Louveaux §5.1.  Always feasible and bounded; w>0 certifies infeasibility.
  LPModel fe;
  fe.sense = Sense::Minimize;
  fe.c = Eigen::VectorXd::Zero(n + m);
  fe.c.tail(m).setOnes();
  fe.vars = r.vars;
  fe.vars.resize(static_cast<std::size_t>(n + m));
  for (int i = 0; i < m; ++i)
    fe.vars[static_cast<std::size_t>(n + i)] = {VarType::Continuous, 0.0, 1e20, {}};
  std::vector<Triplet> trips;
  add_block(trips, r.W, 0, 0, -1.0);            // -W on y
  for (int i = 0; i < m; ++i) trips.emplace_back(i, n + i, -1.0);  // -I on v
  Eigen::SparseMatrix<double> Afe(m, n + m);
  Afe.setFromTriplets(trips.begin(), trips.end());
  Afe.makeCompressed();
  fe.A = std::move(Afe);
  fe.b = rhs;
  fe.Aeq.resize(0, n + m);
  fe.beq.resize(0);

  api::Result fres = eng.solve_lp(fe, opts);
  if (!fres.stats.success || fres.constraint_duals.size() < m) {
    out.status = "recourse feasibility subproblem failed: " + fres.stats.status;
    return out;
  }
  out.solved = true;
  out.feasible = false;
  out.w = fres.stats.objective;
  // grad w_s(x) = T_s^T mu^feas (same coupling rows).  Design doc §2.2.
  out.g = r.T.transpose() * fres.constraint_duals.head(m);
  return out;
}

/// Evaluate Q_s(x) exactly for the CCG oracle (LP or MILP recourse).
struct OracleSolve {
  bool feasible{false};
  double q{0.0};
  Eigen::VectorXd y;
  std::string status;
};

OracleSolve evaluate_recourse(const SolverEngine& eng, const Recourse& r,
                              const Eigen::VectorXd& x,
                              const std::string& solver, double time_limit_sec) {
  OracleSolve out;
  const int n = static_cast<int>(r.W.cols());
  const int m = static_cast<int>(r.W.rows());
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = r.d;
  lp.vars = r.vars;
  Eigen::SparseMatrix<double> negW = (-1.0) * r.W;
  negW.makeCompressed();
  lp.A = negW;
  lp.b = r.T * x - r.h;
  lp.Aeq.resize(0, n);
  lp.beq.resize(0);

  std::vector<int> int_idx, bin_idx;
  collect_integrality(r.vars, 0, int_idx, bin_idx);
  api::Result res =
      solve_generic(eng, lp, int_idx, bin_idx, solver, time_limit_sec);
  if (res.stats.success && res.x.size() == n) {
    out.feasible = true;
    out.q = res.stats.objective;
    out.y = res.x;
  } else {
    out.status = res.stats.status;
    (void)m;
  }
  return out;
}

/// Continuous under-estimator L_s = min_{x in relax(X), y} d_s'y
///   s.t. W_s y + T_s x >= h_s, A x <= b, Aeq x = beq, box(x), y in Y_s.
/// Valid lower bound on theta_s (design doc §3.1). Falls back to `fallback`.
double theta_under_estimator(const SolverEngine& eng, const FirstStage& first,
                             const Recourse& r, const std::string& lp_solver,
                             double fallback) {
  const int n1 = static_cast<int>(first.c.size());
  const int n = static_cast<int>(r.W.cols());
  const int m = static_cast<int>(r.W.rows());
  const int base_ineq = static_cast<int>(first.A.rows());
  const int base_eq = static_cast<int>(first.Aeq.rows());

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(n1 + n);
  lp.c.tail(n) = r.d;
  // Relax first-stage integrality (continuous box), keep recourse bounds.
  lp.vars.resize(static_cast<std::size_t>(n1 + n));
  for (int j = 0; j < n1; ++j)
    lp.vars[static_cast<std::size_t>(j)] = {VarType::Continuous, first.vars[static_cast<std::size_t>(j)].lb,
                                            first.vars[static_cast<std::size_t>(j)].ub, {}};
  for (int j = 0; j < n; ++j)
    lp.vars[static_cast<std::size_t>(n1 + j)] = {VarType::Continuous, r.vars[static_cast<std::size_t>(j)].lb,
                                                 r.vars[static_cast<std::size_t>(j)].ub, {}};

  std::vector<Triplet> ineq;
  add_block(ineq, first.A, 0, 0, 1.0);                 // A x <= b
  add_block(ineq, r.T, base_ineq, 0, -1.0);            // -T x
  add_block(ineq, r.W, base_ineq, n1, -1.0);           // -W y  <= -h
  Eigen::SparseMatrix<double> A(base_ineq + m, n1 + n);
  A.setFromTriplets(ineq.begin(), ineq.end());
  A.makeCompressed();
  lp.A = std::move(A);
  lp.b.resize(base_ineq + m);
  lp.b.head(base_ineq) = first.b;
  lp.b.tail(m) = -r.h;

  std::vector<Triplet> eq;
  add_block(eq, first.Aeq, 0, 0, 1.0);
  Eigen::SparseMatrix<double> Aeq(base_eq, n1 + n);
  Aeq.setFromTriplets(eq.begin(), eq.end());
  Aeq.makeCompressed();
  lp.Aeq = std::move(Aeq);
  lp.beq = first.beq;

  SolveOptions opts;
  opts.preferred_solver = lp_solver;
  opts.allow_fallback = true;
  api::Result res = eng.solve_lp(lp, opts);
  if (res.stats.success && std::isfinite(res.stats.objective))
    return res.stats.objective;
  return fallback;
}

std::string validate_common(const TwoStageModel& model) {
  const auto& f = model.first;
  const int n1 = static_cast<int>(f.c.size());
  if (n1 <= 0) return "first stage has no columns";
  if (static_cast<int>(f.vars.size()) != n1) return "first-stage vars size mismatch";
  if (f.A.cols() != n1 || f.Aeq.cols() != n1) return "first-stage matrix width mismatch";
  if (f.b.size() != f.A.rows() || f.beq.size() != f.Aeq.rows())
    return "first-stage rhs size mismatch";
  if (model.scenarios.empty()) return "no scenarios provided";
  for (std::size_t s = 0; s < model.scenarios.size(); ++s) {
    const auto& r = model.scenarios[s];
    const int m = static_cast<int>(r.W.rows());
    const int n = static_cast<int>(r.W.cols());
    if (n <= 0 || m <= 0) return "scenario has empty recourse";
    if (static_cast<int>(r.d.size()) != n) return "recourse cost size mismatch";
    if (static_cast<int>(r.vars.size()) != n) return "recourse vars size mismatch";
    if (r.T.rows() != m || r.T.cols() != n1) return "technology matrix shape mismatch";
    if (static_cast<int>(r.h.size()) != m) return "recourse rhs size mismatch";
  }
  return {};
}

/// Integer L-shaped method (Laporte & Louveaux 1993) for (SP) with integer
/// recourse and a pure-binary first stage.  Design doc §3.3.  Adds, per
/// scenario and iteration, the exact integer optimality cut (INT) plus the
/// valid LP-relaxation Benders cut (OPT'); the recourse value Q_s(x_k) is
/// evaluated exactly by solving the recourse MILP.
DecompositionResult benders_integer_lshaped(const TwoStageModel& model,
                                            const BendersOptions& options) {
  DecompositionResult out;
  const auto& f = model.first;
  const int n1 = static_cast<int>(f.c.size());
  const int S = static_cast<int>(model.scenarios.size());
  for (const auto& v : f.vars) {
    if (v.type != VarType::Binary) {
      out.status =
          "IntegerLShaped requires a pure-binary first stage (design doc §3.3)";
      return out;
    }
  }

  WorkerPool pool(resolve_workers(options.threads, S));
  SolverEngine& eng = pool.main();
  const std::string dual_lp = pick_dual_lp_solver(eng, options.subproblem_solver);
  const auto t0 = Clock::now();
  auto remaining = [&]() -> double {
    if (options.time_limit_sec <= 0.0) return 0.0;
    return std::max(1e-3, options.time_limit_sec -
                             std::chrono::duration<double>(Clock::now() - t0).count());
  };

  Eigen::VectorXd p(S);
  std::vector<VariableMeta> theta_vars(static_cast<std::size_t>(S));
  std::vector<double> Ls(static_cast<std::size_t>(S));
  for (int s = 0; s < S; ++s) {
    p[s] = model.scenarios[static_cast<std::size_t>(s)].probability;
    Ls[static_cast<std::size_t>(s)] = theta_under_estimator(
        eng, f, model.scenarios[static_cast<std::size_t>(s)], dual_lp,
        options.theta_lower_bound);
    theta_vars[static_cast<std::size_t>(s)] = {VarType::Continuous,
                                               Ls[static_cast<std::size_t>(s)], 1e20, "theta"};
  }
  Eigen::VectorXd master_obj(n1 + S);
  master_obj.head(n1) = f.c;
  master_obj.tail(S) = p;

  std::vector<int> int_idx, bin_idx;
  collect_integrality(f.vars, 0, int_idx, bin_idx);

  std::vector<CutRow> cuts;
  Eigen::VectorXd best_x;
  std::vector<Eigen::VectorXd> best_y;
  double UB = kInf, LB = -kInf;

  for (int it = 0; it < options.max_iterations; ++it) {
    out.iterations = it + 1;
    LPModel master = assemble_master(f, S, master_obj, theta_vars, cuts);
    api::Result mres = solve_generic(eng, master, int_idx, bin_idx,
                                     options.master_solver, remaining());
    if (!mres.stats.success || mres.x.size() != n1 + S) {
      out.status = "Benders master solve failed: " + mres.stats.status;
      return out;
    }
    LB = mres.stats.objective;
    Eigen::VectorXd xk = mres.x.head(n1);
    for (int j = 0; j < n1; ++j) xk[j] = xk[j] >= 0.5 ? 1.0 : 0.0;  // binary snap
    const int ones = static_cast<int>(std::lround(xk.sum()));

    // Per-scenario work: LP-relaxation cut + exact recourse MILP. Independent
    // across scenarios; parallel when threads > 1.
    std::vector<RecourseSolve> rs(static_cast<std::size_t>(S));
    std::vector<OracleSolve> os(static_cast<std::size_t>(S));
    pool.map(S, [&](SolverEngine& e, int s) {
      const auto& r = model.scenarios[static_cast<std::size_t>(s)];
      rs[static_cast<std::size_t>(s)] =
          solve_recourse_for_cut(e, r, xk, dual_lp, remaining());
      os[static_cast<std::size_t>(s)] =
          evaluate_recourse(e, r, xk, options.subproblem_solver, remaining());
    });

    double expectation = 0.0;
    std::vector<Eigen::VectorXd> yk(static_cast<std::size_t>(S));
    for (int s = 0; s < S; ++s) {
      const RecourseSolve& r = rs[static_cast<std::size_t>(s)];
      if (!r.solved) {
        out.status = r.status.empty() ? "recourse solve failed" : r.status;
        return out;
      }
      if (r.feasible) {
        CutRow cut;
        double rhs = -r.q;
        for (int j = 0; j < n1; ++j)
          if (r.g[j] != 0.0) {
            cut.coeffs.emplace_back(j, r.g[j]);
            rhs += r.g[j] * xk[j];
          }
        cut.coeffs.emplace_back(n1 + s, -1.0);
        cut.rhs = rhs;
        cuts.push_back(std::move(cut));
        ++out.cuts_added;
      } else {
        CutRow cut;
        double rhs = -r.w;
        for (int j = 0; j < n1; ++j)
          if (r.g[j] != 0.0) {
            cut.coeffs.emplace_back(j, r.g[j]);
            rhs += r.g[j] * xk[j];
          }
        cut.rhs = rhs;
        cuts.push_back(std::move(cut));
        ++out.cuts_added;
      }

      // (2) Exact recourse MILP => Q_s(x_k) and the integer optimality cut.
      const OracleSolve& o = os[static_cast<std::size_t>(s)];
      if (!o.feasible) {
        out.status =
            "integer recourse infeasible at incumbent; IntegerLShaped assumes "
            "relatively complete recourse (design doc §3.3)";
        return out;
      }
      expectation += p[s] * o.q;
      yk[static_cast<std::size_t>(s)] = o.y;
      // (INT):  theta_s >= (Q-L)(sum_{S1} x - sum_{S0} x - (|S1|-1)) + L.
      // Row form: sum_i a_i x_i - theta_s <= (Q-L)(|S1|-1) - L.
      const double L = Ls[static_cast<std::size_t>(s)];
      const double coef = o.q - L;
      CutRow icut;
      for (int j = 0; j < n1; ++j)
        icut.coeffs.emplace_back(j, xk[j] >= 0.5 ? coef : -coef);
      icut.coeffs.emplace_back(n1 + s, -1.0);
      icut.rhs = coef * (ones - 1) - L;
      cuts.push_back(std::move(icut));
      ++out.cuts_added;
    }

    const double cand = f.c.dot(xk) + expectation;
    if (cand < UB) {
      UB = cand;
      best_x = xk;
      best_y = yk;
    }
    out.lower_bound = LB;
    out.upper_bound = UB;
    out.relative_gap = relative_gap(UB, LB);
    if (options.verbose)
      std::fprintf(stderr, "[IntL] it=%d LB=%.10g UB=%.10g gap=%.3e cuts=%d\n",
                   it + 1, LB, UB, out.relative_gap, out.cuts_added);
    if (std::isfinite(UB) && LB > UB + 1e-6 * (1.0 + std::abs(UB))) {
      out.status = "integer L-shaped lower bound exceeded upper bound";
      return out;
    }
    if (out.relative_gap <= options.gap_tolerance) {
      out.success = true;
      out.status = "Optimal";
      break;
    }
    if (options.time_limit_sec > 0.0 &&
        std::chrono::duration<double>(Clock::now() - t0).count() >=
            options.time_limit_sec) {
      out.status = "time limit";
      break;
    }
  }
  if (out.status.empty()) out.status = "iteration limit";
  if (best_x.size() == n1) {
    out.x = best_x;
    out.y = best_y;
    out.objective = UB;
  }
  return out;
}

/// Inner MILP v_s(lambda) = min_{y in Y_s, z in {0,1}^n1} d'y - lambda'z
///   s.t. W y + T z >= h.  Returns v and the binary minimizer z. Design doc §3.6.
struct LagInner {
  bool ok{false};
  double v{0.0};
  Eigen::VectorXd z;
};

LagInner lagrangian_inner(const SolverEngine& eng, const Recourse& r,
                          const Eigen::VectorXd& lambda, int n1,
                          const std::string& solver, double tl) {
  LagInner out;
  const int n = static_cast<int>(r.W.cols());
  const int m = static_cast<int>(r.W.rows());
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(n + n1);
  lp.c.head(n) = r.d;
  lp.c.tail(n1) = -lambda;
  lp.vars = r.vars;
  lp.vars.resize(static_cast<std::size_t>(n + n1));
  for (int j = 0; j < n1; ++j)
    lp.vars[static_cast<std::size_t>(n + j)] = {VarType::Binary, 0.0, 1.0, {}};
  std::vector<Triplet> trips;  // [-W | -T][y;z] <= -h
  add_block(trips, r.W, 0, 0, -1.0);
  add_block(trips, r.T, 0, n, -1.0);
  Eigen::SparseMatrix<double> A(m, n + n1);
  A.setFromTriplets(trips.begin(), trips.end());
  A.makeCompressed();
  lp.A = std::move(A);
  lp.b = -r.h;
  lp.Aeq.resize(0, n + n1);
  lp.beq.resize(0);
  std::vector<int> int_idx, bin_idx;
  collect_integrality(r.vars, 0, int_idx, bin_idx);
  for (int j = 0; j < n1; ++j) bin_idx.push_back(n + j);
  api::Result res = solve_generic(eng, lp, int_idx, bin_idx, solver, tl);
  if (!res.stats.success || res.x.size() != n + n1) return out;
  out.ok = true;
  out.v = res.stats.objective;
  out.z = res.x.segment(n, n1);
  return out;
}

/// Maximize the Lagrangian dual max_lambda [lambda'xk + v_s(lambda)] over the
/// box [-B, B] by an inner cutting-plane loop. Returns the best (lambda, v).
struct LagCut {
  bool ok{false};
  Eigen::VectorXd lambda;
  double v{0.0};
};

LagCut lagrangian_dual(const SolverEngine& eng, const Recourse& r,
                       const Eigen::VectorXd& xk, int n1, int inner_iters,
                       double B, const std::string& solver, double tl) {
  LagCut out;
  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(n1);
  double best_phi = -kInf;
  std::vector<CutRow> planes;  // over [lambda(0..n1-1) | t(n1)]
  for (int it = 0; it < std::max(1, inner_iters); ++it) {
    LagInner in = lagrangian_inner(eng, r, lambda, n1, solver, tl);
    if (!in.ok) break;
    const double phi = lambda.dot(xk) + in.v;
    if (phi > best_phi) {
      best_phi = phi;
      out.lambda = lambda;
      out.v = in.v;
      out.ok = true;
    }
    // Supporting plane for concave phi: t <= phi + (xk - z)'(lambda' - lambda).
    const Eigen::VectorXd g = xk - in.z;
    CutRow plane;
    for (int j = 0; j < n1; ++j)
      if (g[j] != 0.0) plane.coeffs.emplace_back(j, -g[j]);
    plane.coeffs.emplace_back(n1, 1.0);
    plane.rhs = phi - g.dot(lambda);
    planes.push_back(std::move(plane));

    LPModel dm;  // max t s.t. planes, lambda in [-B,B]
    dm.sense = Sense::Maximize;
    dm.c = Eigen::VectorXd::Zero(n1 + 1);
    dm.c[n1] = 1.0;
    dm.vars.resize(static_cast<std::size_t>(n1 + 1));
    for (int j = 0; j < n1; ++j)
      dm.vars[static_cast<std::size_t>(j)] = {VarType::Continuous, -B, B, {}};
    dm.vars[static_cast<std::size_t>(n1)] = {VarType::Continuous, -1e15, 1e15, {}};
    std::vector<Triplet> trips;
    for (int i = 0; i < static_cast<int>(planes.size()); ++i)
      for (const auto& [col, val] : planes[static_cast<std::size_t>(i)].coeffs)
        trips.emplace_back(i, col, val);
    Eigen::SparseMatrix<double> A(static_cast<int>(planes.size()), n1 + 1);
    A.setFromTriplets(trips.begin(), trips.end());
    A.makeCompressed();
    dm.A = std::move(A);
    dm.b.resize(static_cast<int>(planes.size()));
    for (int i = 0; i < static_cast<int>(planes.size()); ++i)
      dm.b[i] = planes[static_cast<std::size_t>(i)].rhs;
    dm.Aeq.resize(0, n1 + 1);
    dm.beq.resize(0);
    api::Result dr = eng.solve_lp(dm, {});
    if (!dr.stats.success || dr.x.size() != n1 + 1) break;
    lambda = dr.x.head(n1);
    if (dr.x[n1] - best_phi <= 1e-7 * (1.0 + std::abs(best_phi))) break;
  }
  return out;
}

/// Lagrangian/SDDiP cut method (Zou, Ahmed & Sun 2019) for (SP) with integer
/// recourse and a pure-binary first stage. Design doc §3.6. Each iteration adds
/// the affine Lagrangian cut theta_s >= lambda_s'x + v_s(lambda_s); Q_s(x_k) is
/// evaluated exactly (recourse MILP) for the incumbent.
DecompositionResult benders_lagrangian(const TwoStageModel& model,
                                       const BendersOptions& options) {
  DecompositionResult out;
  const auto& f = model.first;
  const int n1 = static_cast<int>(f.c.size());
  const int S = static_cast<int>(model.scenarios.size());
  for (const auto& v : f.vars) {
    if (v.type != VarType::Binary) {
      out.status =
          "Lagrangian mode requires a pure-binary first stage (design doc §3.6)";
      return out;
    }
  }

  WorkerPool pool(resolve_workers(options.threads, S));
  SolverEngine& eng = pool.main();
  const auto t0 = Clock::now();
  auto remaining = [&]() -> double {
    if (options.time_limit_sec <= 0.0) return 0.0;
    return std::max(1e-3, options.time_limit_sec -
                             std::chrono::duration<double>(Clock::now() - t0).count());
  };
  const std::string dual_lp = pick_dual_lp_solver(eng, options.subproblem_solver);

  Eigen::VectorXd p(S);
  std::vector<VariableMeta> theta_vars(static_cast<std::size_t>(S));
  for (int s = 0; s < S; ++s) {
    p[s] = model.scenarios[static_cast<std::size_t>(s)].probability;
    const double Ls = theta_under_estimator(
        eng, f, model.scenarios[static_cast<std::size_t>(s)], dual_lp,
        options.theta_lower_bound);
    theta_vars[static_cast<std::size_t>(s)] = {VarType::Continuous, Ls, 1e20, "theta"};
  }
  Eigen::VectorXd master_obj(n1 + S);
  master_obj.head(n1) = f.c;
  master_obj.tail(S) = p;

  std::vector<int> int_idx, bin_idx;
  collect_integrality(f.vars, 0, int_idx, bin_idx);

  std::vector<CutRow> cuts;
  Eigen::VectorXd best_x;
  std::vector<Eigen::VectorXd> best_y;
  double UB = kInf, LB = -kInf;

  for (int it = 0; it < options.max_iterations; ++it) {
    out.iterations = it + 1;
    LPModel master = assemble_master(f, S, master_obj, theta_vars, cuts);
    api::Result mres = solve_generic(eng, master, int_idx, bin_idx,
                                     options.master_solver, remaining());
    if (!mres.stats.success || mres.x.size() != n1 + S) {
      out.status = "Lagrangian master solve failed: " + mres.stats.status;
      return out;
    }
    LB = mres.stats.objective;
    Eigen::VectorXd xk = mres.x.head(n1);
    for (int j = 0; j < n1; ++j) xk[j] = xk[j] >= 0.5 ? 1.0 : 0.0;

    std::vector<LagCut> lc(static_cast<std::size_t>(S));
    std::vector<OracleSolve> os(static_cast<std::size_t>(S));
    pool.map(S, [&](SolverEngine& e, int s) {
      const auto& r = model.scenarios[static_cast<std::size_t>(s)];
      lc[static_cast<std::size_t>(s)] =
          lagrangian_dual(e, r, xk, n1, options.lagrangian_inner_iterations,
                          options.lagrangian_dual_bound, options.subproblem_solver,
                          remaining());
      os[static_cast<std::size_t>(s)] =
          evaluate_recourse(e, r, xk, options.subproblem_solver, remaining());
    });

    double expectation = 0.0;
    std::vector<Eigen::VectorXd> yk(static_cast<std::size_t>(S));
    for (int s = 0; s < S; ++s) {
      const LagCut& l = lc[static_cast<std::size_t>(s)];
      if (!l.ok) {
        out.status = "Lagrangian dual solve failed";
        return out;
      }
      const OracleSolve& o = os[static_cast<std::size_t>(s)];
      if (!o.feasible) {
        out.status =
            "integer recourse infeasible at incumbent; Lagrangian mode assumes "
            "relatively complete recourse (design doc §3.6)";
        return out;
      }
      expectation += p[s] * o.q;
      yk[static_cast<std::size_t>(s)] = o.y;
      // (LAG):  theta_s >= lambda'x + v   =>   lambda'x - theta_s <= -v.
      CutRow cut;
      for (int j = 0; j < n1; ++j)
        if (l.lambda[j] != 0.0) cut.coeffs.emplace_back(j, l.lambda[j]);
      cut.coeffs.emplace_back(n1 + s, -1.0);
      cut.rhs = -l.v;
      cuts.push_back(std::move(cut));
      ++out.cuts_added;
    }

    const double cand = f.c.dot(xk) + expectation;
    if (cand < UB) {
      UB = cand;
      best_x = xk;
      best_y = yk;
    }
    out.lower_bound = LB;
    out.upper_bound = UB;
    out.relative_gap = relative_gap(UB, LB);
    if (options.verbose)
      std::fprintf(stderr, "[Lag] it=%d LB=%.10g UB=%.10g gap=%.3e cuts=%d\n",
                   it + 1, LB, UB, out.relative_gap, out.cuts_added);
    if (std::isfinite(UB) && LB > UB + 1e-6 * (1.0 + std::abs(UB))) {
      out.status = "Lagrangian lower bound exceeded upper bound";
      return out;
    }
    if (out.relative_gap <= options.gap_tolerance) {
      out.success = true;
      out.status = "Optimal";
      break;
    }
    if (options.time_limit_sec > 0.0 &&
        std::chrono::duration<double>(Clock::now() - t0).count() >=
            options.time_limit_sec) {
      out.status = "time limit";
      break;
    }
  }
  if (out.status.empty()) out.status = "iteration limit";
  if (best_x.size() == n1) {
    out.x = best_x;
    out.y = best_y;
    out.objective = UB;
  }
  return out;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Extensive form (deterministic equivalent) — stochastic
// ─────────────────────────────────────────────────────────────────────────────
DecompositionResult solve_extensive_form_stochastic(
    const TwoStageModel& model, const ExtensiveFormOptions& options) {
  DecompositionResult out;
  if (std::string err = validate_common(model); !err.empty()) {
    out.status = err;
    return out;
  }
  const auto& f = model.first;
  const int n1 = static_cast<int>(f.c.size());
  const int S = static_cast<int>(model.scenarios.size());

  std::vector<int> col_off(S);
  int ncols = n1;
  for (int s = 0; s < S; ++s) {
    col_off[static_cast<std::size_t>(s)] = ncols;
    ncols += static_cast<int>(model.scenarios[static_cast<std::size_t>(s)].W.cols());
  }

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(ncols);
  lp.c.head(n1) = f.c;
  lp.vars = f.vars;
  for (int s = 0; s < S; ++s) {
    const auto& r = model.scenarios[static_cast<std::size_t>(s)];
    lp.vars.insert(lp.vars.end(), r.vars.begin(), r.vars.end());
    lp.c.segment(col_off[static_cast<std::size_t>(s)], r.d.size()) = r.probability * r.d;
  }

  const int base_ineq = static_cast<int>(f.A.rows());
  std::vector<Triplet> ineq;
  add_block(ineq, f.A, 0, 0, 1.0);
  int ineq_rows = base_ineq;
  std::vector<double> bvals(f.b.data(), f.b.data() + f.b.size());
  for (int s = 0; s < S; ++s) {
    const auto& r = model.scenarios[static_cast<std::size_t>(s)];
    const int m = static_cast<int>(r.W.rows());
    add_block(ineq, r.T, ineq_rows, 0, -1.0);                              // -T_s x
    add_block(ineq, r.W, ineq_rows, col_off[static_cast<std::size_t>(s)], -1.0);  // -W_s y_s <= -h_s
    for (int i = 0; i < m; ++i) bvals.push_back(-r.h[i]);
    ineq_rows += m;
  }
  Eigen::SparseMatrix<double> A(ineq_rows, ncols);
  A.setFromTriplets(ineq.begin(), ineq.end());
  A.makeCompressed();
  lp.A = std::move(A);
  lp.b = Eigen::Map<Eigen::VectorXd>(bvals.data(), static_cast<Eigen::Index>(bvals.size()));

  const int base_eq = static_cast<int>(f.Aeq.rows());
  std::vector<Triplet> eq;
  add_block(eq, f.Aeq, 0, 0, 1.0);
  Eigen::SparseMatrix<double> Aeq(base_eq, ncols);
  Aeq.setFromTriplets(eq.begin(), eq.end());
  Aeq.makeCompressed();
  lp.Aeq = std::move(Aeq);
  lp.beq = f.beq;

  std::vector<int> int_idx, bin_idx;
  collect_integrality(f.vars, 0, int_idx, bin_idx);
  for (int s = 0; s < S; ++s)
    collect_integrality(model.scenarios[static_cast<std::size_t>(s)].vars,
                        col_off[static_cast<std::size_t>(s)], int_idx, bin_idx);

  SolverEngine eng;
  eng.register_default_adapters();
  api::Result res = solve_generic(eng, lp, int_idx, bin_idx, options.solver,
                                  options.time_limit_sec);
  out.status = res.stats.status;
  if (!res.stats.success || res.x.size() != ncols) return out;
  out.success = true;
  out.x = res.x.head(n1);
  out.objective = res.stats.objective;
  out.lower_bound = res.stats.objective;
  out.upper_bound = res.stats.objective;
  out.relative_gap = 0.0;
  out.y.resize(static_cast<std::size_t>(S));
  for (int s = 0; s < S; ++s)
    out.y[static_cast<std::size_t>(s)] =
        res.x.segment(col_off[static_cast<std::size_t>(s)],
                      model.scenarios[static_cast<std::size_t>(s)].W.cols());
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Extensive form (epigraph) — robust
// ─────────────────────────────────────────────────────────────────────────────
DecompositionResult solve_extensive_form_robust(
    const TwoStageModel& model, const ExtensiveFormOptions& options) {
  DecompositionResult out;
  if (std::string err = validate_common(model); !err.empty()) {
    out.status = err;
    return out;
  }
  const auto& f = model.first;
  const int n1 = static_cast<int>(f.c.size());
  const int S = static_cast<int>(model.scenarios.size());
  const int eta_col = n1;  // epigraph variable eta

  std::vector<int> col_off(S);
  int ncols = n1 + 1;
  for (int s = 0; s < S; ++s) {
    col_off[static_cast<std::size_t>(s)] = ncols;
    ncols += static_cast<int>(model.scenarios[static_cast<std::size_t>(s)].W.cols());
  }

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(ncols);
  lp.c.head(n1) = f.c;
  lp.c[eta_col] = 1.0;
  lp.vars = f.vars;
  lp.vars.push_back({VarType::Continuous, -1e20, 1e20, "eta"});
  for (int s = 0; s < S; ++s) {
    const auto& r = model.scenarios[static_cast<std::size_t>(s)];
    lp.vars.insert(lp.vars.end(), r.vars.begin(), r.vars.end());
  }

  const int base_ineq = static_cast<int>(f.A.rows());
  std::vector<Triplet> ineq;
  add_block(ineq, f.A, 0, 0, 1.0);
  int ineq_rows = base_ineq;
  std::vector<double> bvals(f.b.data(), f.b.data() + f.b.size());
  for (int s = 0; s < S; ++s) {
    const auto& r = model.scenarios[static_cast<std::size_t>(s)];
    const int m = static_cast<int>(r.W.rows());
    // Epigraph:  d_s' y_s - eta <= 0.
    for (int j = 0; j < static_cast<int>(r.d.size()); ++j)
      if (r.d[j] != 0.0)
        ineq.emplace_back(ineq_rows, col_off[static_cast<std::size_t>(s)] + j, r.d[j]);
    ineq.emplace_back(ineq_rows, eta_col, -1.0);
    bvals.push_back(0.0);
    ineq_rows += 1;
    // Recourse:  -T_s x - W_s y_s <= -h_s.
    add_block(ineq, r.T, ineq_rows, 0, -1.0);
    add_block(ineq, r.W, ineq_rows, col_off[static_cast<std::size_t>(s)], -1.0);
    for (int i = 0; i < m; ++i) bvals.push_back(-r.h[i]);
    ineq_rows += m;
  }
  Eigen::SparseMatrix<double> A(ineq_rows, ncols);
  A.setFromTriplets(ineq.begin(), ineq.end());
  A.makeCompressed();
  lp.A = std::move(A);
  lp.b = Eigen::Map<Eigen::VectorXd>(bvals.data(), static_cast<Eigen::Index>(bvals.size()));

  const int base_eq = static_cast<int>(f.Aeq.rows());
  std::vector<Triplet> eq;
  add_block(eq, f.Aeq, 0, 0, 1.0);
  Eigen::SparseMatrix<double> Aeq(base_eq, ncols);
  Aeq.setFromTriplets(eq.begin(), eq.end());
  Aeq.makeCompressed();
  lp.Aeq = std::move(Aeq);
  lp.beq = f.beq;

  std::vector<int> int_idx, bin_idx;
  collect_integrality(f.vars, 0, int_idx, bin_idx);
  for (int s = 0; s < S; ++s)
    collect_integrality(model.scenarios[static_cast<std::size_t>(s)].vars,
                        col_off[static_cast<std::size_t>(s)], int_idx, bin_idx);

  SolverEngine eng;
  eng.register_default_adapters();
  api::Result res = solve_generic(eng, lp, int_idx, bin_idx, options.solver,
                                  options.time_limit_sec);
  out.status = res.stats.status;
  if (!res.stats.success || res.x.size() != ncols) return out;
  out.success = true;
  out.x = res.x.head(n1);
  out.objective = res.stats.objective;
  out.lower_bound = res.stats.objective;
  out.upper_bound = res.stats.objective;
  out.relative_gap = 0.0;
  out.y.resize(static_cast<std::size_t>(S));
  for (int s = 0; s < S; ++s)
    out.y[static_cast<std::size_t>(s)] =
        res.x.segment(col_off[static_cast<std::size_t>(s)],
                      model.scenarios[static_cast<std::size_t>(s)].W.cols());
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// L-shaped / Benders — stochastic
// ─────────────────────────────────────────────────────────────────────────────
DecompositionResult solve_benders_stochastic(const TwoStageModel& model,
                                             const BendersOptions& options) {
  DecompositionResult out;
  if (std::string err = validate_common(model); !err.empty()) {
    out.status = err;
    return out;
  }
  const auto& f = model.first;
  const int n1 = static_cast<int>(f.c.size());
  const int S = static_cast<int>(model.scenarios.size());
  if (options.cut_mode == BendersCutMode::IntegerLShaped)
    return benders_integer_lshaped(model, options);
  if (options.cut_mode == BendersCutMode::Lagrangian)
    return benders_lagrangian(model, options);
  // Continuous (multi/single-cut) Benders optimality cuts require continuous
  // recourse (Assumption A1); integer recourse must use IntegerLShaped or CCG.
  for (int s = 0; s < S; ++s) {
    if (has_integrality(model.scenarios[static_cast<std::size_t>(s)].vars)) {
      out.status =
          "Benders MultiCut/SingleCut require continuous recourse; use "
          "IntegerLShaped, solve_ccg_robust, or the extensive form for integer "
          "recourse (design doc §3.3, §5)";
      return out;
    }
  }
  const bool multi = options.cut_mode == BendersCutMode::MultiCut;
  const int n_theta = multi ? S : 1;

  WorkerPool pool(resolve_workers(options.threads, S));
  SolverEngine& eng = pool.main();
  const std::string dual_lp = pick_dual_lp_solver(eng, options.subproblem_solver);
  const auto t0 = Clock::now();
  auto remaining = [&]() -> double {
    if (options.time_limit_sec <= 0.0) return 0.0;
    const double used =
        std::chrono::duration<double>(Clock::now() - t0).count();
    return std::max(1e-3, options.time_limit_sec - used);
  };

  // Epigraph columns and their valid lower bounds L_s.
  Eigen::VectorXd p(S);
  for (int s = 0; s < S; ++s) p[s] = model.scenarios[static_cast<std::size_t>(s)].probability;
  std::vector<VariableMeta> theta_vars(static_cast<std::size_t>(n_theta));
  Eigen::VectorXd theta_obj(n_theta);
  if (multi) {
    for (int s = 0; s < S; ++s) {
      const double Ls = theta_under_estimator(
          eng, f, model.scenarios[static_cast<std::size_t>(s)], dual_lp,
          options.theta_lower_bound);
      theta_vars[static_cast<std::size_t>(s)] = {VarType::Continuous, Ls, 1e20, "theta"};
      theta_obj[s] = p[s];
    }
  } else {
    double Lsum = 0.0;
    for (int s = 0; s < S; ++s)
      Lsum += p[s] * theta_under_estimator(
                         eng, f, model.scenarios[static_cast<std::size_t>(s)],
                         dual_lp, options.theta_lower_bound);
    theta_vars[0] = {VarType::Continuous, Lsum, 1e20, "theta"};
    theta_obj[0] = 1.0;
  }

  Eigen::VectorXd master_obj(n1 + n_theta);
  master_obj.head(n1) = f.c;
  master_obj.tail(n_theta) = theta_obj;

  std::vector<int> int_idx, bin_idx;
  collect_integrality(f.vars, 0, int_idx, bin_idx);

  // In-out stabilization (Ben-Ameur & Neto 2007) applies only to a continuous
  // first stage, where x_sep = alpha*x_center + (1-alpha)*x_out stays feasible
  // and yields a valid incumbent. alpha decays on master-bound stall so the
  // method reduces to Kelley (pure cutting plane) in the limit.
  const bool stab_eligible = int_idx.empty() && bin_idx.empty() &&
                             options.stabilization_alpha > 0.0 &&
                             options.stabilization_alpha < 1.0;
  double alpha = stab_eligible ? options.stabilization_alpha : 0.0;

  std::vector<CutRow> cuts;
  Eigen::VectorXd best_x, x_center;
  std::vector<Eigen::VectorXd> best_y;
  double UB = kInf;
  double LB = -kInf;
  double LB_prev = -kInf;

  for (int it = 0; it < options.max_iterations; ++it) {
    out.iterations = it + 1;
    LPModel master =
        assemble_master(f, n_theta, master_obj, theta_vars, cuts);
    api::Result mres = solve_generic(eng, master, int_idx, bin_idx,
                                     options.master_solver, remaining());
    if (!mres.stats.success || mres.x.size() != n1 + n_theta) {
      out.status = "Benders master solve failed: " + mres.stats.status;
      return out;
    }
    LB = mres.stats.objective;
    const Eigen::VectorXd x_out = mres.x.head(n1);
    if (x_center.size() != n1) x_center = x_out;
    if (it > 0 && LB <= LB_prev + 1e-9 * (1.0 + std::abs(LB))) alpha *= 0.5;
    if (alpha < 1e-4) alpha = 0.0;
    LB_prev = LB;
    const Eigen::VectorXd xk =
        alpha > 0.0 ? (alpha * x_center + (1.0 - alpha) * x_out).eval() : x_out;

    // Per-scenario recourse solves at the separation point. Parallel when
    // options.threads > 1; each worker owns an independent SolverEngine, and
    // the results land in disjoint slots, so the reduce phase stays serial.
    std::vector<RecourseSolve> rs(static_cast<std::size_t>(S));
    pool.map(S, [&](SolverEngine& e, int s) {
      rs[static_cast<std::size_t>(s)] = solve_recourse_for_cut(
          e, model.scenarios[static_cast<std::size_t>(s)], xk, dual_lp,
          remaining());
    });

    double recourse_expectation = 0.0;
    bool all_feasible = true;
    std::vector<Eigen::VectorXd> yk(static_cast<std::size_t>(S));
    std::vector<double> qk(static_cast<std::size_t>(S), 0.0);
    std::vector<Eigen::VectorXd> gk(static_cast<std::size_t>(S));
    for (int s = 0; s < S; ++s) {
      const RecourseSolve& r = rs[static_cast<std::size_t>(s)];
      if (!r.solved) {
        out.status = r.status.empty() ? "recourse solve failed" : r.status;
        return out;
      }
      gk[static_cast<std::size_t>(s)] = r.g;
      if (r.feasible) {
        qk[static_cast<std::size_t>(s)] = r.q;
        yk[static_cast<std::size_t>(s)] = r.y;
        recourse_expectation += p[s] * r.q;
      } else {
        all_feasible = false;
        // Feasibility cut (design doc §2.2):  g_f' x <= g_f' xk - w_s(xk).
        CutRow cut;
        double rhs = -r.w;
        for (int j = 0; j < n1; ++j) {
          const double gj = r.g[j];
          if (gj != 0.0) {
            cut.coeffs.emplace_back(j, gj);
            rhs += gj * xk[j];
          }
        }
        cut.rhs = rhs;
        cuts.push_back(std::move(cut));
        ++out.cuts_added;
      }
    }

    if (all_feasible) {
      const double cand = f.c.dot(xk) + recourse_expectation;
      if (cand < UB) {
        UB = cand;
        best_x = xk;
        best_y = yk;
        x_center = xk;  // move the in-out stability center to the incumbent
      }
      // Optimality cuts (design doc §2.1):
      //   multi:  theta_s >= q_s + g_s'(x - xk)
      //   single: theta   >= sum_s p_s [q_s + g_s'(x - xk)]
      if (multi) {
        for (int s = 0; s < S; ++s) {
          CutRow cut;
          double rhs = -qk[static_cast<std::size_t>(s)];
          for (int j = 0; j < n1; ++j) {
            const double gj = gk[static_cast<std::size_t>(s)][j];
            if (gj != 0.0) {
              cut.coeffs.emplace_back(j, gj);
              rhs += gj * xk[j];
            }
          }
          cut.coeffs.emplace_back(n1 + s, -1.0);  // - theta_s
          cut.rhs = rhs;
          cuts.push_back(std::move(cut));
          ++out.cuts_added;
        }
      } else {
        Eigen::VectorXd gagg = Eigen::VectorXd::Zero(n1);
        double qagg = 0.0;
        for (int s = 0; s < S; ++s) {
          gagg += p[s] * gk[static_cast<std::size_t>(s)];
          qagg += p[s] * qk[static_cast<std::size_t>(s)];
        }
        CutRow cut;
        double rhs = -qagg;
        for (int j = 0; j < n1; ++j) {
          if (gagg[j] != 0.0) {
            cut.coeffs.emplace_back(j, gagg[j]);
            rhs += gagg[j] * xk[j];
          }
        }
        cut.coeffs.emplace_back(n1, -1.0);  // - theta
        cut.rhs = rhs;
        cuts.push_back(std::move(cut));
        ++out.cuts_added;
      }
    }

    out.lower_bound = LB;
    out.upper_bound = UB;
    out.relative_gap = relative_gap(UB, LB);
    if (options.verbose)
      std::fprintf(stderr,
                   "[Benders] it=%d LB=%.10g UB=%.10g gap=%.3e cuts=%d\n",
                   it + 1, LB, UB, out.relative_gap, out.cuts_added);

    // Runtime validity guard: a valid cut family keeps LB <= UB.  A breach
    // signals a dual-sign/scaling defect (design doc §4 mismatch protocol).
    if (std::isfinite(UB) && LB > UB + 1e-6 * (1.0 + std::abs(UB))) {
      out.status = "Benders lower bound exceeded upper bound (invalid cut)";
      return out;
    }
    if (out.relative_gap <= options.gap_tolerance) {
      out.success = true;
      out.status = "Optimal";
      break;
    }
    if (options.time_limit_sec > 0.0 &&
        std::chrono::duration<double>(Clock::now() - t0).count() >=
            options.time_limit_sec) {
      out.status = "time limit";
      break;
    }
  }
  if (out.status.empty()) out.status = "iteration limit";
  if (best_x.size() == n1) {
    out.x = best_x;
    out.y = best_y;
    out.objective = UB;
  }
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Column-and-Constraint Generation — robust
// ─────────────────────────────────────────────────────────────────────────────
DecompositionResult solve_ccg_robust(const TwoStageModel& model,
                                     const CCGOptions& options) {
  DecompositionResult out;
  if (std::string err = validate_common(model); !err.empty()) {
    out.status = err;
    return out;
  }
  const auto& f = model.first;
  const int n1 = static_cast<int>(f.c.size());
  const int S = static_cast<int>(model.scenarios.size());
  const int eta_col = n1;

  WorkerPool pool(resolve_workers(options.threads, S));
  SolverEngine& eng = pool.main();
  const auto t0 = Clock::now();

  std::vector<int> active;                      // ordered active scenario set O
  std::vector<char> in_active(static_cast<std::size_t>(S), 0);
  Eigen::VectorXd best_x;
  double UB = kInf;
  double LB = -kInf;

  for (int it = 0; it < options.max_iterations; ++it) {
    out.iterations = it + 1;
    // Assemble master (M_O): variables [x | eta | y^s for s in O].
    const int K = static_cast<int>(active.size());
    std::vector<int> col_off(static_cast<std::size_t>(K));
    int ncols = n1 + 1;
    for (int a = 0; a < K; ++a) {
      col_off[static_cast<std::size_t>(a)] = ncols;
      ncols += static_cast<int>(
          model.scenarios[static_cast<std::size_t>(active[static_cast<std::size_t>(a)])].W.cols());
    }

    LPModel lp;
    lp.sense = Sense::Minimize;
    lp.c = Eigen::VectorXd::Zero(ncols);
    lp.c.head(n1) = f.c;
    lp.c[eta_col] = 1.0;
    lp.vars = f.vars;
    lp.vars.push_back({VarType::Continuous, -1e20, 1e20, "eta"});
    std::vector<int> int_idx, bin_idx;
    collect_integrality(f.vars, 0, int_idx, bin_idx);
    for (int a = 0; a < K; ++a) {
      const auto& r = model.scenarios[static_cast<std::size_t>(active[static_cast<std::size_t>(a)])];
      lp.vars.insert(lp.vars.end(), r.vars.begin(), r.vars.end());
      collect_integrality(r.vars, col_off[static_cast<std::size_t>(a)], int_idx, bin_idx);
    }

    const int base_ineq = static_cast<int>(f.A.rows());
    std::vector<Triplet> ineq;
    add_block(ineq, f.A, 0, 0, 1.0);
    int ineq_rows = base_ineq;
    std::vector<double> bvals(f.b.data(), f.b.data() + f.b.size());
    for (int a = 0; a < K; ++a) {
      const auto& r = model.scenarios[static_cast<std::size_t>(active[static_cast<std::size_t>(a)])];
      const int m = static_cast<int>(r.W.rows());
      for (int j = 0; j < static_cast<int>(r.d.size()); ++j)
        if (r.d[j] != 0.0)
          ineq.emplace_back(ineq_rows, col_off[static_cast<std::size_t>(a)] + j, r.d[j]);
      ineq.emplace_back(ineq_rows, eta_col, -1.0);  // d_s' y^s - eta <= 0
      bvals.push_back(0.0);
      ineq_rows += 1;
      add_block(ineq, r.T, ineq_rows, 0, -1.0);
      add_block(ineq, r.W, ineq_rows, col_off[static_cast<std::size_t>(a)], -1.0);
      for (int i = 0; i < m; ++i) bvals.push_back(-r.h[i]);
      ineq_rows += m;
    }
    Eigen::SparseMatrix<double> A(ineq_rows, ncols);
    A.setFromTriplets(ineq.begin(), ineq.end());
    A.makeCompressed();
    lp.A = std::move(A);
    lp.b = Eigen::Map<Eigen::VectorXd>(bvals.data(), static_cast<Eigen::Index>(bvals.size()));
    std::vector<Triplet> eq;
    add_block(eq, f.Aeq, 0, 0, 1.0);
    Eigen::SparseMatrix<double> Aeq(static_cast<int>(f.Aeq.rows()), ncols);
    Aeq.setFromTriplets(eq.begin(), eq.end());
    Aeq.makeCompressed();
    lp.Aeq = std::move(Aeq);
    lp.beq = f.beq;

    double rem = 0.0;
    if (options.time_limit_sec > 0.0)
      rem = std::max(1e-3, options.time_limit_sec -
                              std::chrono::duration<double>(Clock::now() - t0).count());
    api::Result mres =
        solve_generic(eng, lp, int_idx, bin_idx, options.master_solver, rem);
    if (!mres.stats.success || mres.x.size() != ncols) {
      out.status = "CCG master solve failed: " + mres.stats.status;
      return out;
    }
    const Eigen::VectorXd xk = mres.x.head(n1);
    // When O is empty the epigraph is unbounded below; LB stays -inf and only
    // the first oracle pass populates the active set.
    LB = (K == 0) ? -kInf : mres.stats.objective;

    // Oracle: worst-case scenario over the full finite uncertainty set.
    // Evaluations are independent and run in parallel when threads > 1.
    std::vector<OracleSolve> os(static_cast<std::size_t>(S));
    pool.map(S, [&](SolverEngine& e, int s) {
      os[static_cast<std::size_t>(s)] = evaluate_recourse(
          e, model.scenarios[static_cast<std::size_t>(s)], xk,
          options.recourse_solver, rem);
    });
    int worst = -1;
    double worst_q = -kInf;
    int infeasible_pick = -1;
    double max_q = -kInf;
    for (int s = 0; s < S; ++s) {
      if (!os[static_cast<std::size_t>(s)].feasible) {
        if (!in_active[static_cast<std::size_t>(s)] && infeasible_pick < 0)
          infeasible_pick = s;  // prioritize restoring robust feasibility
        continue;
      }
      const double q = os[static_cast<std::size_t>(s)].q;
      max_q = std::max(max_q, q);
      if (q > worst_q && !in_active[static_cast<std::size_t>(s)]) {
        worst_q = q;
        worst = s;
      }
    }

    if (infeasible_pick >= 0) {
      active.push_back(infeasible_pick);
      in_active[static_cast<std::size_t>(infeasible_pick)] = 1;
      out.scenarios_generated = static_cast<int>(active.size());
      continue;  // no finite UB this pass; add constraints and re-solve
    }

    const double cand = f.c.dot(xk) + max_q;
    if (cand < UB) {
      UB = cand;
      best_x = xk;
    }
    out.lower_bound = LB;
    out.upper_bound = UB;
    out.relative_gap = relative_gap(UB, LB);
    if (options.verbose)
      std::fprintf(stderr,
                   "[CCG] it=%d |O|=%d LB=%.10g UB=%.10g gap=%.3e worst=%d\n",
                   it + 1, K, LB, UB, out.relative_gap, worst);

    if (std::isfinite(LB) && out.relative_gap <= options.gap_tolerance) {
      out.success = true;
      out.status = "Optimal";
      break;
    }
    if (worst < 0) {
      // Every worst scenario is already active: the master already represents
      // the true maximum, so the incumbent is optimal.
      out.success = std::isfinite(UB);
      out.status = out.success ? "Optimal" : "no improving scenario";
      break;
    }
    active.push_back(worst);
    in_active[static_cast<std::size_t>(worst)] = 1;
    out.scenarios_generated = static_cast<int>(active.size());
    if (options.time_limit_sec > 0.0 &&
        std::chrono::duration<double>(Clock::now() - t0).count() >=
            options.time_limit_sec) {
      out.status = "time limit";
      break;
    }
  }
  if (out.status.empty()) out.status = "iteration limit";
  out.scenarios_generated = static_cast<int>(active.size());
  if (best_x.size() == n1) {
    out.x = best_x;
    out.objective = UB;
    // Recover recourse optimizers at x* for every scenario.
    out.y.resize(static_cast<std::size_t>(S));
    double rem = 0.0;
    for (int s = 0; s < S; ++s) {
      OracleSolve os = evaluate_recourse(
          eng, model.scenarios[static_cast<std::size_t>(s)], best_x,
          options.recourse_solver, rem);
      if (os.feasible) out.y[static_cast<std::size_t>(s)] = os.y;
    }
  }
  return out;
}

namespace {

/// Freeze a robust recourse at a concrete realization u:  h = h0 + P u.
Recourse freeze_recourse(const RobustRecourse& rr, const Eigen::VectorXd& u) {
  Recourse r;
  r.d = rr.d;
  r.T = rr.T;
  r.W = rr.W;
  r.h = rr.h0 + rr.P * u;
  r.vars = rr.vars;
  r.probability = 1.0;
  return r;
}

/// Assemble the robust master (M_O) over concrete recourse realizations.
///   variables [x | eta | y^a for a in active], min c'x + eta.
struct RobustMasterBuild {
  LPModel lp;
  std::vector<int> col_off;
  std::vector<int> int_idx, bin_idx;
  int ncols{0};
};

RobustMasterBuild build_robust_master(const FirstStage& f,
                                      const std::vector<Recourse>& active) {
  const int n1 = static_cast<int>(f.c.size());
  const int eta_col = n1;
  const int K = static_cast<int>(active.size());
  RobustMasterBuild mb;
  mb.col_off.resize(static_cast<std::size_t>(K));
  int ncols = n1 + 1;
  for (int a = 0; a < K; ++a) {
    mb.col_off[static_cast<std::size_t>(a)] = ncols;
    ncols += static_cast<int>(active[static_cast<std::size_t>(a)].W.cols());
  }
  mb.ncols = ncols;

  LPModel& lp = mb.lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(ncols);
  lp.c.head(n1) = f.c;
  lp.c[eta_col] = 1.0;
  lp.vars = f.vars;
  lp.vars.push_back({VarType::Continuous, -1e20, 1e20, "eta"});
  collect_integrality(f.vars, 0, mb.int_idx, mb.bin_idx);
  for (int a = 0; a < K; ++a) {
    const auto& r = active[static_cast<std::size_t>(a)];
    lp.vars.insert(lp.vars.end(), r.vars.begin(), r.vars.end());
    collect_integrality(r.vars, mb.col_off[static_cast<std::size_t>(a)],
                        mb.int_idx, mb.bin_idx);
  }

  const int base_ineq = static_cast<int>(f.A.rows());
  std::vector<Triplet> ineq;
  add_block(ineq, f.A, 0, 0, 1.0);
  int ineq_rows = base_ineq;
  std::vector<double> bvals(f.b.data(), f.b.data() + f.b.size());
  for (int a = 0; a < K; ++a) {
    const auto& r = active[static_cast<std::size_t>(a)];
    const int m = static_cast<int>(r.W.rows());
    for (int j = 0; j < static_cast<int>(r.d.size()); ++j)
      if (r.d[j] != 0.0)
        ineq.emplace_back(ineq_rows, mb.col_off[static_cast<std::size_t>(a)] + j, r.d[j]);
    ineq.emplace_back(ineq_rows, eta_col, -1.0);  // d'y^a - eta <= 0
    bvals.push_back(0.0);
    ineq_rows += 1;
    add_block(ineq, r.T, ineq_rows, 0, -1.0);
    add_block(ineq, r.W, ineq_rows, mb.col_off[static_cast<std::size_t>(a)], -1.0);
    for (int i = 0; i < m; ++i) bvals.push_back(-r.h[i]);
    ineq_rows += m;
  }
  Eigen::SparseMatrix<double> A(ineq_rows, ncols);
  A.setFromTriplets(ineq.begin(), ineq.end());
  A.makeCompressed();
  lp.A = std::move(A);
  lp.b = Eigen::Map<Eigen::VectorXd>(bvals.data(), static_cast<Eigen::Index>(bvals.size()));

  std::vector<Triplet> eq;
  add_block(eq, f.Aeq, 0, 0, 1.0);
  Eigen::SparseMatrix<double> Aeq(static_cast<int>(f.Aeq.rows()), ncols);
  Aeq.setFromTriplets(eq.begin(), eq.end());
  Aeq.makeCompressed();
  lp.Aeq = std::move(Aeq);
  lp.beq = f.beq;
  return mb;
}

struct PolyOracle {
  bool ok{false};
  double value{0.0};
  Eigen::VectorXd u;
  std::string status;
};

/// KKT max–min oracle (ORACLE), design doc §3.4:  max_{u in U} Q(x*,u) via the
/// big-M linearized single-level MILP.  Returns the worst value and realization.
PolyOracle poly_worst_case(const SolverEngine& eng, const RobustRecourse& rr,
                           const UncertaintySet& U, const Eigen::VectorXd& xstar,
                           double M, const std::string& solver,
                           double time_limit_sec) {
  PolyOracle out;
  const int m = static_cast<int>(rr.W.rows());
  const int n = static_cast<int>(rr.W.cols());
  const int nu = static_cast<int>(rr.P.cols());
  const int mu = static_cast<int>(U.G.rows());

  const int o_u = 0, o_y = nu, o_pi = nu + n, o_s = nu + n + m,
            o_rho = nu + n + 2 * m, o_b = nu + 2 * n + 2 * m,
            o_e = nu + 2 * n + 3 * m;
  const int ncols = nu + 3 * n + 3 * m;

  LPModel lp;
  lp.sense = Sense::Maximize;  // max d'y = max_u Q(x*,u)
  lp.c = Eigen::VectorXd::Zero(ncols);
  lp.c.segment(o_y, n) = rr.d;
  lp.vars.resize(static_cast<std::size_t>(ncols));
  for (int k = 0; k < nu; ++k)
    lp.vars[static_cast<std::size_t>(o_u + k)] = {VarType::Continuous, U.u_lb[k], U.u_ub[k], {}};
  for (int j = 0; j < n; ++j)
    lp.vars[static_cast<std::size_t>(o_y + j)] = rr.vars[static_cast<std::size_t>(j)];
  for (int i = 0; i < m; ++i)
    lp.vars[static_cast<std::size_t>(o_pi + i)] = {VarType::Continuous, 0.0, 1e20, {}};
  for (int i = 0; i < m; ++i)
    lp.vars[static_cast<std::size_t>(o_s + i)] = {VarType::Continuous, 0.0, 1e20, {}};
  for (int j = 0; j < n; ++j)
    lp.vars[static_cast<std::size_t>(o_rho + j)] = {VarType::Continuous, 0.0, 1e20, {}};
  for (int i = 0; i < m; ++i)
    lp.vars[static_cast<std::size_t>(o_b + i)] = {VarType::Binary, 0.0, 1.0, {}};
  for (int j = 0; j < n; ++j)
    lp.vars[static_cast<std::size_t>(o_e + j)] = {VarType::Binary, 0.0, 1.0, {}};

  // Equalities: E1 (m) primal defn, E2 (n) dual defn.
  std::vector<Triplet> eq;
  add_block(eq, rr.W, 0, o_y, 1.0);            // W y
  add_block(eq, rr.P, 0, o_u, -1.0);           // - P u
  for (int i = 0; i < m; ++i) eq.emplace_back(i, o_s + i, -1.0);  // - s
  Eigen::SparseMatrix<double> WT = rr.W.transpose();
  add_block(eq, WT, m, o_pi, 1.0);             // W' pi
  for (int j = 0; j < n; ++j) eq.emplace_back(m + j, o_rho + j, 1.0);  // + rho
  Eigen::SparseMatrix<double> Aeq(m + n, ncols);
  Aeq.setFromTriplets(eq.begin(), eq.end());
  Aeq.makeCompressed();
  lp.Aeq = std::move(Aeq);
  lp.beq.resize(m + n);
  lp.beq.head(m) = rr.h0 - rr.T * xstar;
  lp.beq.tail(n) = rr.d;

  // Inequalities: Ia1,Ia2 (2m), Ib1,Ib2 (2n), G u <= g (mu).
  std::vector<Triplet> in;
  std::vector<double> b;
  int row = 0;
  for (int i = 0; i < m; ++i) {  // pi_i - M b_i <= 0
    in.emplace_back(row, o_pi + i, 1.0);
    in.emplace_back(row, o_b + i, -M);
    b.push_back(0.0);
    ++row;
  }
  for (int i = 0; i < m; ++i) {  // s_i + M b_i <= M
    in.emplace_back(row, o_s + i, 1.0);
    in.emplace_back(row, o_b + i, M);
    b.push_back(M);
    ++row;
  }
  for (int j = 0; j < n; ++j) {  // y_j - M e_j <= 0
    in.emplace_back(row, o_y + j, 1.0);
    in.emplace_back(row, o_e + j, -M);
    b.push_back(0.0);
    ++row;
  }
  for (int j = 0; j < n; ++j) {  // rho_j + M e_j <= M
    in.emplace_back(row, o_rho + j, 1.0);
    in.emplace_back(row, o_e + j, M);
    b.push_back(M);
    ++row;
  }
  add_block(in, U.G, row, o_u, 1.0);  // G u <= g
  for (int i = 0; i < mu; ++i) b.push_back(U.g[i]);
  row += mu;
  Eigen::SparseMatrix<double> A(row, ncols);
  A.setFromTriplets(in.begin(), in.end());
  A.makeCompressed();
  lp.A = std::move(A);
  lp.b = Eigen::Map<Eigen::VectorXd>(b.data(), static_cast<Eigen::Index>(b.size()));

  std::vector<int> int_idx, bin_idx;
  for (int i = 0; i < m; ++i) bin_idx.push_back(o_b + i);
  for (int j = 0; j < n; ++j) bin_idx.push_back(o_e + j);

  api::Result res =
      solve_generic(eng, lp, int_idx, bin_idx, solver, time_limit_sec);
  if (!res.stats.success || res.x.size() != ncols) {
    out.status = "oracle MILP failed: " + res.stats.status;
    return out;
  }
  // Runtime big-M validity check (design doc §3.4 / §4).
  const double thr = 0.99 * M;
  auto seg_max = [&](int off, int len) {
    double mx = 0.0;
    for (int i = 0; i < len; ++i) mx = std::max(mx, std::abs(res.x[off + i]));
    return mx;
  };
  if (seg_max(o_pi, m) >= thr || seg_max(o_s, m) >= thr ||
      seg_max(o_y, n) >= thr || seg_max(o_rho, n) >= thr) {
    out.status = "oracle big-M too small; increase PolyhedralCCGOptions::big_m";
    return out;
  }
  out.ok = true;
  out.value = res.stats.objective;
  out.u = res.x.segment(o_u, nu);
  return out;
}

std::string validate_polyhedral(const PolyhedralRobustModel& model) {
  const auto& f = model.first;
  const auto& rr = model.recourse;
  const auto& U = model.uncertainty;
  const int n1 = static_cast<int>(f.c.size());
  const int m = static_cast<int>(rr.W.rows());
  const int n = static_cast<int>(rr.W.cols());
  const int nu = static_cast<int>(U.u_lb.size());
  if (n1 <= 0) return "first stage has no columns";
  if (static_cast<int>(f.vars.size()) != n1) return "first-stage vars size mismatch";
  if (m <= 0 || n <= 0) return "empty recourse";
  if (rr.T.rows() != m || rr.T.cols() != n1) return "technology matrix shape mismatch";
  if (static_cast<int>(rr.d.size()) != n) return "recourse cost size mismatch";
  if (static_cast<int>(rr.vars.size()) != n) return "recourse vars size mismatch";
  if (static_cast<int>(rr.h0.size()) != m) return "nominal rhs size mismatch";
  if (rr.P.rows() != m || rr.P.cols() != nu) return "uncertainty map shape mismatch";
  if (static_cast<int>(U.u_ub.size()) != nu) return "uncertainty bound size mismatch";
  if (U.G.cols() != nu || U.g.size() != U.G.rows())
    return "uncertainty budget shape mismatch";
  // The KKT max-min oracle (ORACLE) relies on strong LP duality of the inner
  // recourse; it is invalid for integer recourse (no MILP duality). Reject it
  // loudly rather than emit inexact cuts -- see design doc §3.5 for the exact
  // remedies (finite-scenario CCG, nested C&CG, Lagrangian/SDDiP cuts).
  if (has_integrality(rr.vars))
    return "polyhedral CCG requires continuous recourse; integer recourse needs "
           "finite-scenario solve_ccg_robust or nested C&CG (design doc §3.5)";
  return {};
}

}  // namespace

DecompositionResult solve_ccg_polyhedral_robust(
    const PolyhedralRobustModel& model, const PolyhedralCCGOptions& options) {
  DecompositionResult out;
  if (std::string err = validate_polyhedral(model); !err.empty()) {
    out.status = err;
    return out;
  }
  const auto& f = model.first;
  const int n1 = static_cast<int>(f.c.size());

  SolverEngine eng;
  eng.register_default_adapters();
  const auto t0 = Clock::now();
  auto remaining = [&]() -> double {
    if (options.time_limit_sec <= 0.0) return 0.0;
    return std::max(1e-3, options.time_limit_sec -
                             std::chrono::duration<double>(Clock::now() - t0).count());
  };

  std::vector<Recourse> active;   // frozen worst-case realizations
  Eigen::VectorXd best_x;
  double UB = kInf, LB = -kInf;

  for (int it = 0; it < options.max_iterations; ++it) {
    out.iterations = it + 1;
    RobustMasterBuild mb = build_robust_master(f, active);
    api::Result mres = solve_generic(eng, mb.lp, mb.int_idx, mb.bin_idx,
                                     options.master_solver, remaining());
    if (!mres.stats.success || mres.x.size() != mb.ncols) {
      out.status = "polyhedral CCG master solve failed: " + mres.stats.status;
      return out;
    }
    const Eigen::VectorXd xk = mres.x.head(n1);
    LB = active.empty() ? -kInf : mres.stats.objective;

    PolyOracle oracle = poly_worst_case(eng, model.recourse, model.uncertainty,
                                        xk, options.big_m, options.oracle_solver,
                                        remaining());
    if (!oracle.ok) {
      out.status = oracle.status;
      return out;
    }
    const double cand = f.c.dot(xk) + oracle.value;
    if (cand < UB) {
      UB = cand;
      best_x = xk;
    }
    out.lower_bound = LB;
    out.upper_bound = UB;
    out.relative_gap = relative_gap(UB, LB);
    if (options.verbose)
      std::fprintf(stderr,
                   "[PolyCCG] it=%d |O|=%zu LB=%.10g UB=%.10g gap=%.3e\n",
                   it + 1, active.size(), LB, UB, out.relative_gap);
    if (std::isfinite(LB) && out.relative_gap <= options.gap_tolerance) {
      out.success = true;
      out.status = "Optimal";
      break;
    }
    active.push_back(freeze_recourse(model.recourse, oracle.u));
    out.scenarios_generated = static_cast<int>(active.size());
    if (options.time_limit_sec > 0.0 &&
        std::chrono::duration<double>(Clock::now() - t0).count() >=
            options.time_limit_sec) {
      out.status = "time limit";
      break;
    }
  }
  if (out.status.empty()) out.status = "iteration limit";
  out.scenarios_generated = static_cast<int>(active.size());
  if (best_x.size() == n1) {
    out.x = best_x;
    out.objective = UB;
  }
  return out;
}

}  // namespace mipsolvers::engine::decomposition
