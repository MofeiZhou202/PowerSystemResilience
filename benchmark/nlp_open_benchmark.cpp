// Native Filter-IPM versus Ipopt on the open HS071 NLP used by Ipopt's
// official C++ tutorial. Both solvers receive the same callbacks, derivatives,
// start point, bounds, and tolerances.
#include <chrono>
#include <cstdio>
#include <cstdlib>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/kernel/ipm/ipm_solver.hpp"
#include "mipsolvers/engine/solver/external/adapters.hpp"

using namespace mipsolvers::engine;

namespace {

NLPModel hs071() {
  NLPModel p;
  p.vars.assign(4, {VarType::Continuous, 1.0, 5.0});
  p.x0 = (Eigen::Vector4d() << 1.0, 5.0, 5.0, 1.0).finished();
  p.f = [](const Eigen::VectorXd& x) {
    return x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2];
  };
  p.grad = [](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
    g.resize(4);
    g << x[3] * (2.0 * x[0] + x[1] + x[2]), x[0] * x[3],
         x[0] * x[3] + 1.0, x[0] * (x[0] + x[1] + x[2]);
  };
  p.g = [](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
    g.resize(1);
    g[0] = x.squaredNorm() - 40.0;
  };
  p.jac_g = [](const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& j) {
    j.resize(1, 4);
    for (int col = 0; col < 4; ++col) j.insert(0, col) = 2.0 * x[col];
    j.makeCompressed();
  };
  p.h = [](const Eigen::VectorXd& x, Eigen::VectorXd& h) {
    h.resize(1);
    h[0] = 25.0 - x.prod();
  };
  p.jac_h = [](const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& j) {
    j.resize(1, 4);
    const double product = x.prod();
    for (int col = 0; col < 4; ++col) j.insert(0, col) = -product / x[col];
    j.makeCompressed();
  };
  p.lagrangian_hess = [](
      const Eigen::VectorXd& x, const Eigen::VectorXd& lambda,
      const Eigen::VectorXd* nu, Eigen::SparseMatrix<double>& h) {
    Eigen::Matrix4d dense = Eigen::Matrix4d::Zero();
    dense(0, 0) = 2.0 * x[3];
    dense(0, 1) = dense(1, 0) = x[3];
    dense(0, 2) = dense(2, 0) = x[3];
    dense(0, 3) = dense(3, 0) = 2.0 * x[0] + x[1] + x[2];
    dense(1, 3) = dense(3, 1) = x[0];
    dense(2, 3) = dense(3, 2) = x[0];
    if (lambda.size() == 1) dense.diagonal().array() += 2.0 * lambda[0];
    if (nu != nullptr && nu->size() == 1) {
      const double product = x.prod();
      for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 4; ++j) {
          const double value = -(*nu)[0] * product / (x[i] * x[j]);
          dense(i, j) += value;
          dense(j, i) += value;
        }
      }
    }
    h = dense.sparseView();
    h.makeCompressed();
  };
  return p;
}

bool report(const char* solver, int run, const SolveResult& r, double ms,
            const IPMDetail* detail) {
  const double reference = 17.014017289155;
  std::printf(
      "problem=HS071 source=Ipopt-tutorial solver=%s actual=%s run=%d "
      "success=%d accurate=%d iters=%d total_ms=%.3f obj=%.12g "
      "obj_error=%.3e primal=%.3e dual=%.3e compl=%.3e formulation=%s\n",
      solver, r.stats.solver_name.c_str(), run, static_cast<int>(r.stats.success),
      static_cast<int>(r.stats.success &&
                       std::abs(r.stats.objective - reference) <= 1e-6 &&
                       r.stats.primal_feas <= 1e-6),
      r.stats.iterations, ms, r.stats.objective,
      std::abs(r.stats.objective - reference), r.stats.primal_feas,
      r.stats.dual_feas, r.stats.complementarity,
      detail ? detail->newton_formulation.c_str() : "external");
  // Windows remediation R3: enforce the same public accuracy gate as the log.
  return r.stats.success && std::abs(r.stats.objective - reference) <= 1e-6 &&
         r.stats.primal_feas <= 1e-6;
}

}  // namespace

int main(int argc, char** argv) {
  const int repetitions = argc > 1 ? std::max(1, std::atoi(argv[1])) : 5;
  // R3c: KKT tolerances do not directly bound absolute objective error.
  const double tolerance = argc > 2 ? std::atof(argv[2]) : 1e-6;
  if (!(tolerance > 0.0) || !std::isfinite(tolerance)) return 2;
  std::printf("requested_kkt_tolerance=%.12g objective_acceptance=1e-6\n", tolerance);
  NLPModel model = hs071();
  model.solver_options.max_iterations = 200;
  model.solver_options.tolerance = tolerance;
  model.solver_options.acceptable_tolerance = tolerance;

  IPMOptions options;
  options.max_iter = 200;
  options.tol_primal = tolerance;
  options.tol_dual = tolerance;
  options.tol_complementarity = tolerance;
  options.tol_accept = 0.0;
  options.allow_external_fallback = false;

  bool ok = true;
  for (int run = 1; run <= repetitions; ++run) {
    if ((run & 1) != 0) {
      NativeIPMAdapter native(options);
      const auto t0 = std::chrono::steady_clock::now();
      const auto solved = native.solve_nlp_detail(model);
      const double ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - t0).count();
      ok = report("native", run, solved.first, ms, &solved.second) && ok;
    }
    IpoptAdapter ipopt;
    const auto t0 = std::chrono::steady_clock::now();
    const SolveResult solved = ipopt.solve_nlp(model);
    const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
    ok = report("ipopt", run, solved, ms, nullptr) && ok;
    if ((run & 1) == 0) {
      NativeIPMAdapter native(options);
      const auto tn = std::chrono::steady_clock::now();
      const auto native_solved = native.solve_nlp_detail(model);
      const double native_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - tn).count();
      ok = report("native", run, native_solved.first, native_ms,
                  &native_solved.second) && ok;
    }
  }
  return ok ? 0 : 1;
}
