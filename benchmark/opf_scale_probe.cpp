// Native IPM versus Ipopt scalability probe on an OPF-structure nonconvex QP.
// Models the OPF KKT shape at scale: indefinite (nonconvex) Hessian on a grid
// graph + power-balance-like equalities + line-limit-type inequalities.
// Reports per-size: success, iterations, total ms, and (verbose) factor time.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/kernel/ipm/ipm_solver.hpp"
#include "mipsolvers/engine/solver/external/adapters.hpp"

using namespace mipsolvers::engine;
using T = Eigen::Triplet<double>;

// Grid-structured indefinite QP:
//   min  0.5 x'Qx + c'x   (Q indefinite → nonconvex)
//   s.t. Aeq x = beq      (power-balance-like, sparse)
//        Cx <= d          (line-limit-type)
//        lb <= x <= ub
int main(int argc, char** argv) {
  const int side = (argc > 1) ? atoi(argv[1]) : 40;   // side x side buses
  const int nb = side * side;
  const int n = 2 * nb;                                // v and theta per bus
  const std::string mode = (argc > 2) ? argv[2] : "both";
  bool verbose = false;
  bool least_square_init_duals = false;
  NewtonFormulation formulation = NewtonFormulation::Auto;
  int repetitions = 1;
  for (int arg = 3; arg < argc; ++arg) {
    const std::string value = argv[arg];
    if (value == "v" || value == "verbose") {
      verbose = true;
      continue;
    }
    if (value == "ls-duals") {
      least_square_init_duals = true;
      continue;
    }
    if (value == "no-ls-duals") {
      least_square_init_duals = false;
      continue;
    }
    if (value == "auto" || value == "condensed" || value == "augmented") {
      formulation = value == "condensed"
          ? NewtonFormulation::Condensed
          : (value == "augmented" ? NewtonFormulation::Augmented
                                    : NewtonFormulation::Auto);
      continue;
    }
    char* end = nullptr;
    const long parsed = std::strtol(value.c_str(), &end, 10);
    if (end == value.c_str() || *end != '\0' || parsed < 1 || parsed > 10000) {
      std::fprintf(
          stderr,
          "usage: %s [grid-side] [native|ipopt|both] [repetitions] [v] "
          "[ls-duals|no-ls-duals]\n",
          argv[0]);
      return 2;
    }
    repetitions = static_cast<int>(parsed);
  }
  if (mode != "native" && mode != "ipopt" && mode != "both") {
    std::fprintf(
        stderr,
        "usage: %s [grid-side] [native|ipopt|both] [repetitions] [v] "
        "[ls-duals|no-ls-duals]\n",
        argv[0]);
    return 2;
  }

  // Neighborhood: bus (r,c) connects to (r±1,c),(r,c±1).
  std::vector<std::pair<int,int>> edges;
  for (int r = 0; r < side; ++r)
    for (int c = 0; c < side; ++c) {
      const int i = r * side + c;
      if (r + 1 < side) edges.emplace_back(i, i + side);
      if (c + 1 < side) edges.emplace_back(i, i + 1);
    }
  const int ne = static_cast<int>(edges.size());

  std::vector<T> qt, ct, at, ct2;
  qt.reserve(static_cast<size_t>(n) * 8);
  at.reserve(static_cast<size_t>(n) * 10);
  ct2.reserve(static_cast<size_t>(ne) * 2);

  // Q: grid Laplacian-like with alternating sign blocks → indefinite.
  for (int i = 0; i < nb; ++i) {
    qt.emplace_back(i, i, (i % 7 == 0) ? -2.0 : 4.0);       // negative → nonconvex
    qt.emplace_back(nb + i, nb + i, (i % 5 == 0) ? -1.0 : 2.0);
  }
  for (auto [i, j] : edges) {
    qt.emplace_back(i, j, -1.0);
    qt.emplace_back(j, i, -1.0);
    qt.emplace_back(nb + i, nb + j, -0.5);
    qt.emplace_back(nb + j, nb + i, -0.5);
  }
  Eigen::SparseMatrix<double> Q(n, n);
  Q.setFromTriplets(qt.begin(), qt.end());
  Q.makeCompressed();

  Eigen::VectorXd c(n);
  for (int j = 0; j < n; ++j) c[j] = 0.1 * ((j * 37 % 11) - 5);

  // Equalities: for each bus k, two balance rows (P and Q analog).
  std::vector<T> aeqt;
  aeqt.reserve(static_cast<size_t>(nb) * 12);
  Eigen::VectorXd beq(2 * nb);
  for (int k = 0; k < nb; ++k) {
    // P balance: v_k * (sum of neighbor v) - theta_k = Pd
    aeqt.emplace_back(k, k, 1.0);
    aeqt.emplace_back(k, nb + k, -1.0);
    // Q balance: v_k + theta_k + sum neighbor theta = Qd
    aeqt.emplace_back(nb + k, k, 1.0);
    aeqt.emplace_back(nb + k, nb + k, 1.0);
  }
  // Neighbor couplings into balance rows.
  int er = 0;
  for (auto [i, j] : edges) {
    aeqt.emplace_back(i, j, 0.7);      // P_i couples v_j
    aeqt.emplace_back(j, i, 0.7);
    aeqt.emplace_back(nb + i, nb + j, 0.3);  // Q_i couples theta_j
    aeqt.emplace_back(nb + j, nb + i, 0.3);
    (void)er++;
  }
  Eigen::SparseMatrix<double> Aeq(2 * nb, n);
  Aeq.setFromTriplets(aeqt.begin(), aeqt.end());
  Aeq.makeCompressed();
  Eigen::VectorXd planted_x = Eigen::VectorXd::Zero(n);
  planted_x.head(nb).setOnes();
  beq = Aeq * planted_x;

  // Inequalities: line-limit (v_i - v_j) <= limit per edge.
  std::vector<T> ct3;
  ct3.reserve(static_cast<size_t>(ne) * 2);
  Eigen::VectorXd dvec(ne);
  for (size_t e = 0; e < edges.size(); ++e) {
    ct3.emplace_back(static_cast<int>(e), edges[e].first, 1.0);
    ct3.emplace_back(static_cast<int>(e), edges[e].second, -1.0);
    dvec[static_cast<Eigen::Index>(e)] = 0.5;
  }
  Eigen::SparseMatrix<double> C(ne, n);
  C.setFromTriplets(ct3.begin(), ct3.end());
  C.makeCompressed();

  NLPModel nlp;
  nlp.sense = Sense::Minimize;
  for (int j = 0; j < n; ++j) {
    const bool is_v = (j < nb);
    // Feasible by construction: v≈1, θ free to absorb the balance rows.
    nlp.vars.push_back({VarType::Continuous, is_v ? 0.9 : -6.0, is_v ? 1.1 : 6.0});
  }
  nlp.x0 = planted_x;

  const auto Qc = Q;
  const auto Cc = c;
  const auto Aeqc = Aeq;
  const auto beqc = beq;
  const auto Cm = C;
  const auto dv = dvec;

  nlp.f = [Qc, Cc](const Eigen::VectorXd& x) {
    return 0.5 * x.dot(Qc * x) + Cc.dot(x);
  };
  nlp.grad = [Qc, Cc](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
    g = Qc * x + Cc;
  };
  nlp.hess = [Qc](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& H) {
    H = Qc;
  };
  nlp.g = [Aeqc, beqc](const Eigen::VectorXd& x, Eigen::VectorXd& v) {
    v = Aeqc * x - beqc;
  };
  nlp.jac_g = [Aeqc](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& J) {
    J = Aeqc;
  };
  nlp.h = [Cm, dv](const Eigen::VectorXd& x, Eigen::VectorXd& v) {
    v = Cm * x - dv;
  };
  nlp.jac_h = [Cm](const Eigen::VectorXd&, Eigen::SparseMatrix<double>& J) {
    J = Cm;
  };

  IPMOptions opt;
  opt.verbose = verbose;
  opt.max_iter = 60;
  opt.tol_primal = 1e-6;
  opt.tol_dual = 1e-6;
  opt.tol_complementarity = 1e-6;
  opt.tol_accept = 0.0;
  opt.scale_problem = false;
  opt.allow_external_fallback = false;
  opt.least_square_init_duals = least_square_init_duals;
  opt.newton_formulation = formulation;

  nlp.solver_options.max_iterations = opt.max_iter;
  nlp.solver_options.tolerance = 1e-6;
  nlp.solver_options.acceptable_tolerance = 1e-6;
  nlp.solver_options.dual_infeasibility_tolerance = 1e-6;
  nlp.solver_options.constraint_violation_tolerance = 1e-6;
  nlp.solver_options.complementarity_tolerance = 1e-6;
  nlp.solver_options.acceptable_dual_infeasibility_tolerance = 1e-6;
  nlp.solver_options.acceptable_constraint_violation_tolerance = 1e-6;
  nlp.solver_options.acceptable_complementarity_tolerance = 1e-6;

  const auto report = [&](const char* requested, int run, int order_position,
                          const SolveResult& res, double elapsed_ms,
                          const IPMDetail* detail) {
    std::printf(
        "solver=%s actual=%s run=%d order_pos=%d ls_duals=%d buses=%d n=%d "
        "meq=%d miq=%d success=%d "
        "status=\"%s\" iters=%d obj=%.12g primal=%.3e dual=%.3e "
        "compl=%.3e total_ms=%.3f formulation=%s condensed_flops=%.0f "
        "augmented_flops=%.0f condensed_lnz=%.0f augmented_lnz=%.0f "
        "symbolic=%d factors=%d solves=%d\n",
        requested, res.stats.solver_name.c_str(), run, order_position,
        static_cast<int>(least_square_init_duals), nb, n, 2 * nb, ne,
        static_cast<int>(res.stats.success), res.stats.status.c_str(),
        res.stats.iterations, res.stats.objective, res.stats.primal_feas,
        res.stats.dual_feas, res.stats.complementarity, elapsed_ms,
        detail ? detail->newton_formulation.c_str() : "external",
        detail ? detail->condensed_symbolic_flops : 0.0,
        detail ? detail->augmented_symbolic_flops : 0.0,
        detail ? detail->condensed_symbolic_nonzeros : 0.0,
        detail ? detail->augmented_symbolic_nonzeros : 0.0,
        detail ? detail->symbolic_analyses : 0,
        detail ? detail->numeric_factorizations : 0,
        detail ? detail->linear_solves : 0);
  };

  bool all_requested_succeeded = true;
  const auto run_native = [&](int run, int order_position) {
    NativeIPMAdapter solver(opt);
    const auto t0 = std::chrono::steady_clock::now();
    const auto solved = solver.solve_nlp_detail(nlp);
    const SolveResult& res = solved.first;
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0).count();
    report("native", run, order_position, res, ms, &solved.second);
    all_requested_succeeded = all_requested_succeeded && res.stats.success;
  };
  const auto run_ipopt = [&](int run, int order_position) {
    IpoptAdapter solver;
    const auto t0 = std::chrono::steady_clock::now();
    const SolveResult res = solver.solve_nlp(nlp);
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0).count();
    report("ipopt", run, order_position, res, ms, nullptr);
    all_requested_succeeded = all_requested_succeeded && res.stats.success;
  };

  for (int run = 1; run <= repetitions; ++run) {
    if (mode == "native") {
      run_native(run, 1);
    } else if (mode == "ipopt") {
      run_ipopt(run, 1);
    } else if ((run % 2) == 1) {
      run_native(run, 1);
      run_ipopt(run, 2);
    } else {
      run_ipopt(run, 1);
      run_native(run, 2);
    }
  }
  return all_requested_succeeded ? 0 : 1;
}
