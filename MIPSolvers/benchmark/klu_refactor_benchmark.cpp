#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp"

namespace eng = mipsolvers::engine;

namespace {

using Clock = std::chrono::steady_clock;

double elapsed_ms(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start)
      .count();
}

double median(std::vector<double> values) {
  const auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

int positive_int(const char* text, int fallback) {
  if (!text) return fallback;
  const int value = std::atoi(text);
  return value > 0 ? value : fallback;
}

Eigen::SparseMatrix<double> make_matrix(int n) {
  std::vector<Eigen::Triplet<double>> entries;
  entries.reserve(static_cast<std::size_t>(5 * n));
  // This fixed +/-1, +/-7 stencil is strictly diagonally dominant. It isolates
  // KLU's numeric lifecycle without numerical failure noise; see
  // docs/archive/klu_numeric_refactor_2026-08-20.md, sections 1 and 4.
  for (int col = 0; col < n; ++col) {
    entries.emplace_back(col, col, 6.0);
    if (col + 1 < n) {
      entries.emplace_back(col, col + 1, -1.0);
      entries.emplace_back(col + 1, col, -1.0);
    }
    if (col + 7 < n) {
      entries.emplace_back(col, col + 7, -0.25);
      entries.emplace_back(col + 7, col, -0.25);
    }
  }
  Eigen::SparseMatrix<double> matrix(n, n);
  matrix.setFromTriplets(entries.begin(), entries.end());
  matrix.makeCompressed();
  return matrix;
}

void update_values(Eigen::SparseMatrix<double>& matrix, int iteration) {
  // A bounded diagonal perturbation preserves both the CSC pattern and strict
  // diagonal dominance required by the validation model cited above.
  for (int i = 0; i < matrix.rows(); ++i) {
    matrix.coeffRef(i, i) =
        6.0 + 1e-3 * static_cast<double>((i + 3 * iteration) % 29);
  }
}

}  // namespace

int main(int argc, char** argv) {
#ifndef HACDCPF_HAVE_KLU
  std::fprintf(stderr, "KLU support is required for this benchmark\n");
  return 2;
#else
  int n = 4000;
  int repetitions = 80;
  int trials = 7;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--n" && i + 1 < argc) {
      n = positive_int(argv[++i], n);
    } else if (arg == "--repeat" && i + 1 < argc) {
      repetitions = positive_int(argv[++i], repetitions);
    } else if (arg == "--trials" && i + 1 < argc) {
      trials = positive_int(argv[++i], trials);
    } else if (arg == "--help") {
      std::puts(
          "klu_refactor_benchmark [--n N] [--repeat R] [--trials T]");
      return 0;
    }
  }
  if (n < 16) {
    std::fprintf(stderr, "--n must be at least 16\n");
    return 2;
  }

  Eigen::SparseMatrix<double> matrix = make_matrix(n);
  eng::EigenKluSolver full_solver;
  eng::EigenKluSolver refactor_solver;
  full_solver.analyze_pattern(matrix);
  refactor_solver.analyze_pattern(matrix);
  if (!full_solver.factorize(matrix) || !refactor_solver.factorize(matrix)) {
    std::fprintf(stderr, "initial factorization failed\n");
    return 1;
  }

  std::vector<double> full_trial_ms;
  std::vector<double> refactor_trial_ms;
  full_trial_ms.reserve(static_cast<std::size_t>(trials));
  refactor_trial_ms.reserve(static_cast<std::size_t>(trials));
  int iteration = 0;
  for (int trial = 0; trial < trials; ++trial) {
    double full_ms = 0.0;
    double refactor_ms = 0.0;
    for (int repeat = 0; repeat < repetitions; ++repeat, ++iteration) {
      update_values(matrix, iteration);
      // Alternate call order to distribute cache and frequency-state effects.
      if ((iteration & 1) == 0) {
        auto start = Clock::now();
        if (!full_solver.factorize(matrix)) return 1;
        full_ms += elapsed_ms(start);
        start = Clock::now();
        if (!refactor_solver.refactorize(matrix)) return 1;
        refactor_ms += elapsed_ms(start);
      } else {
        auto start = Clock::now();
        if (!refactor_solver.refactorize(matrix)) return 1;
        refactor_ms += elapsed_ms(start);
        start = Clock::now();
        if (!full_solver.factorize(matrix)) return 1;
        full_ms += elapsed_ms(start);
      }
    }
    full_trial_ms.push_back(full_ms);
    refactor_trial_ms.push_back(refactor_ms);
  }

  Eigen::VectorXd rhs(n);
  for (int i = 0; i < n; ++i) {
    rhs[i] = std::sin(0.01 * static_cast<double>(i)) + 0.25;
  }
  Eigen::VectorXd full_x;
  Eigen::VectorXd refactor_x;
  if (!full_solver.solve(rhs, full_x) ||
      !refactor_solver.solve(rhs, refactor_x)) {
    std::fprintf(stderr, "final solve failed\n");
    return 1;
  }

  const double rhs_scale = std::max(1.0, rhs.lpNorm<Eigen::Infinity>());
  const double full_residual =
      (matrix * full_x - rhs).lpNorm<Eigen::Infinity>() / rhs_scale;
  const double refactor_residual =
      (matrix * refactor_x - rhs).lpNorm<Eigen::Infinity>() / rhs_scale;
  const double solution_scale =
      std::max(1.0, full_x.lpNorm<Eigen::Infinity>());
  const double relative_difference =
      (full_x - refactor_x).lpNorm<Eigen::Infinity>() / solution_scale;
  const double full_ms = median(full_trial_ms);
  const double refactor_ms = median(refactor_trial_ms);
  const double speedup = refactor_ms > 0.0 ? full_ms / refactor_ms : 0.0;
  const double reduction = full_ms > 0.0 ? 1.0 - refactor_ms / full_ms : 0.0;

  std::printf(
      "KLU fixed-pattern refactor: n=%d nnz=%lld repetitions=%d trials=%d\n"
      "  full_factor_total=%.3f ms per_call=%.6f ms\n"
      "  refactor_total=%.3f ms per_call=%.6f ms\n"
      "  speedup=%.3fx reduction=%.2f%%\n"
      "  full_residual=%.3e refactor_residual=%.3e solution_delta=%.3e\n",
      n, static_cast<long long>(matrix.nonZeros()), repetitions, trials,
      full_ms, full_ms / repetitions, refactor_ms,
      refactor_ms / repetitions, speedup, 100.0 * reduction, full_residual,
      refactor_residual, relative_difference);

  return full_residual <= 1e-10 && refactor_residual <= 1e-10 &&
                 relative_difference <= 1e-10
             ? 0
             : 1;
#endif
}
