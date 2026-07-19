// Temporary scalability probe: OPF-structure nonconvex QP through the filter IPM.
// Models the OPF KKT shape at scale: indefinite (nonconvex) Hessian on a grid
// graph + power-balance-like equalities + line-limit-type inequalities.
// Reports per-size: success, iterations, total ms, and (verbose) factor time.
#include <chrono>
#include <cstdio>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/kernel/ipm/ipm_solver.hpp"

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
  const bool verbose = (argc > 2 && argv[2][0] == 'v');

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
    beq[k] = 0.05 * ((k * 13 % 7) - 3);
    // Q balance: v_k + theta_k + sum neighbor theta = Qd
    aeqt.emplace_back(nb + k, k, 1.0);
    aeqt.emplace_back(nb + k, nb + k, 1.0);
    beq[nb + k] = 0.02 * ((k * 17 % 5) - 2);
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
  nlp.x0 = Eigen::VectorXd::Zero(n);
  for (int j = 0; j < nb; ++j) nlp.x0[j] = 1.0;

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
  NativeIPMAdapter solver(opt);

  const auto t0 = std::chrono::steady_clock::now();
  const auto [res, detail] = solver.solve_nlp_detail(nlp);
  const double ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t0).count();
  printf("buses=%d n=%d meq=%d miq=%d | success=%d status=%s iters=%d obj=%.6g total=%.1fms\n",
         nb, n, 2 * nb, ne, (int)res.stats.success, res.stats.status.c_str(),
         res.stats.iterations, res.stats.objective, ms);
  return res.stats.success ? 0 : 1;
}
