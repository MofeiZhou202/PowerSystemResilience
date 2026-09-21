#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/kernel/kkt/kkt_system.hpp"

namespace eng = mipsolvers::engine;

namespace {

using Clock = std::chrono::steady_clock;

double milliseconds(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start)
      .count();
}

double median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  const auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

int read_int(const char* text, int fallback) {
  if (!text) return fallback;
  const int value = std::atoi(text);
  return value > 0 ? value : fallback;
}

}  // namespace

int main(int argc, char** argv) {
  int n = 12000;
  int rows = 3000;
  int repeats = 20;
  int rhs_count = 8;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--n" && i + 1 < argc) n = read_int(argv[++i], n);
    else if (arg == "--rows" && i + 1 < argc)
      rows = read_int(argv[++i], rows);
    else if (arg == "--repeat" && i + 1 < argc)
      repeats = read_int(argv[++i], repeats);
    else if (arg == "--rhs" && i + 1 < argc)
      rhs_count = read_int(argv[++i], rhs_count);
    else if (arg == "--help") {
      std::puts("kkt_reuse_benchmark [--n N] [--rows M] [--repeat R] [--rhs K]");
      return 0;
    }
  }
  if (n < 3 * rows) {
    std::fprintf(stderr, "--n must be at least 3 * --rows\n");
    return 2;
  }

  std::vector<Eigen::Triplet<double>> w_entries;
  w_entries.reserve(static_cast<std::size_t>(5 * n));
  for (int i = 0; i < n; ++i) {
    w_entries.emplace_back(i, i, 6.0);
    if (i + 1 < n) {
      w_entries.emplace_back(i, i + 1, -1.0);
      w_entries.emplace_back(i + 1, i, -1.0);
    }
    if (i + 97 < n) {
      w_entries.emplace_back(i, i + 97, -0.25);
      w_entries.emplace_back(i + 97, i, -0.25);
    }
  }
  Eigen::SparseMatrix<double> w(n, n);
  w.setFromTriplets(w_entries.begin(), w_entries.end());
  w.makeCompressed();

  std::vector<Eigen::Triplet<double>> jg_entries;
  jg_entries.reserve(static_cast<std::size_t>(3 * rows));
  for (int row = 0; row < rows; ++row) {
    jg_entries.emplace_back(row, row, 1.0);
    jg_entries.emplace_back(row, row + rows, -0.5);
    jg_entries.emplace_back(row, row + 2 * rows, 0.25);
  }
  Eigen::SparseMatrix<double> jg(rows, n);
  jg.setFromTriplets(jg_entries.begin(), jg_entries.end());
  jg.makeCompressed();

  const int dim = n + rows;
  Eigen::VectorXd rhs(dim);
  for (int i = 0; i < dim; ++i) rhs[i] = std::sin(0.01 * i) + 0.25;

  eng::SparseKKTCache cache;
  if (!eng::factor_kkt_sparse(cache, w, jg, 1e-8)) {
    std::fprintf(stderr, "warm-up factorization failed\n");
    return 1;
  }
  Eigen::VectorXd dx, dy;
  if (!eng::solve_kkt_sparse(cache, rhs, dx, dy)) {
    std::fprintf(stderr, "warm-up solve failed\n");
    return 1;
  }

  std::vector<double> factor_ms;
  std::vector<double> solve_ms;
  factor_ms.reserve(static_cast<std::size_t>(repeats));
  solve_ms.reserve(static_cast<std::size_t>(repeats * rhs_count));
  for (int repeat = 0; repeat < repeats; ++repeat) {
    for (int i = 0; i < n; ++i) {
      w.coeffRef(i, i) = 6.0 + 1e-3 * ((i + repeat) % 17);
    }
    const auto factor_start = Clock::now();
    if (!eng::factor_kkt_sparse(cache, w, jg, 1e-8)) return 1;
    factor_ms.push_back(milliseconds(factor_start));
    for (int solve = 0; solve < rhs_count; ++solve) {
      rhs[solve % dim] += 1e-5;
      const auto solve_start = Clock::now();
      if (!eng::solve_kkt_sparse(cache, rhs, dx, dy)) return 1;
      solve_ms.push_back(milliseconds(solve_start));
    }
  }

  Eigen::VectorXd solution(dim);
  solution << dx, dy;
  const double relative_residual =
      (cache.kkt * solution - rhs).lpNorm<Eigen::Infinity>() /
      std::max(1.0, rhs.lpNorm<Eigen::Infinity>());

  Eigen::MatrixXd block_rhs(dim, rhs_count);
  for (int col = 0; col < rhs_count; ++col) {
    block_rhs.col(col) = rhs;
    block_rhs(col % dim, col) += 1e-4 * (col + 1);
  }
  Eigen::MatrixXd sequential_solution(dim, rhs_count), block_solution;
  auto solve_sequential = [&]() {
    for (int col = 0; col < rhs_count; ++col) {
      Eigen::VectorXd x;
      if (!cache.solver->solve(block_rhs.col(col), x)) return false;
      sequential_solution.col(col) = x;
    }
    return true;
  };
  if (!solve_sequential() ||
      !cache.solver->solve_many(block_rhs, block_solution)) {
    std::fprintf(stderr, "multi-RHS warm-up failed\n");
    return 1;
  }
  std::vector<double> sequential_ms, block_ms;
  sequential_ms.reserve(static_cast<std::size_t>(repeats));
  block_ms.reserve(static_cast<std::size_t>(repeats));
  for (int repeat = 0; repeat < repeats; ++repeat) {
    if ((repeat & 1) == 0) {
      const auto sequential_start = Clock::now();
      if (!solve_sequential()) return 1;
      sequential_ms.push_back(milliseconds(sequential_start));
      const auto block_start = Clock::now();
      if (!cache.solver->solve_many(block_rhs, block_solution)) return 1;
      block_ms.push_back(milliseconds(block_start));
    } else {
      const auto block_start = Clock::now();
      if (!cache.solver->solve_many(block_rhs, block_solution)) return 1;
      block_ms.push_back(milliseconds(block_start));
      const auto sequential_start = Clock::now();
      if (!solve_sequential()) return 1;
      sequential_ms.push_back(milliseconds(sequential_start));
    }
  }
  const double sequential_residual =
      (cache.kkt * sequential_solution - block_rhs).cwiseAbs().maxCoeff() /
      std::max(1.0, block_rhs.cwiseAbs().maxCoeff());
  const double block_residual =
      (cache.kkt * block_solution - block_rhs).cwiseAbs().maxCoeff() /
      std::max(1.0, block_rhs.cwiseAbs().maxCoeff());
  const double median_sequential_ms = median(sequential_ms);
  const double median_block_ms = median(block_ms);
  const std::size_t matrix_bytes =
      static_cast<std::size_t>(cache.kkt.nonZeros()) *
          (sizeof(double) + sizeof(int)) +
      static_cast<std::size_t>(cache.kkt.outerSize() + 1) * sizeof(int);
  std::printf(
      "KKT reuse: dim=%d nnz=%lld repeats=%d rhs/factor=%d backend=%s\n"
      "  symbolic=%d numeric=%d solves=%d resident_kkt=%.2f MiB\n"
      "  median_factor=%.3f ms median_solve=%.3f ms residual=%.3e\n"
      "  multi_rhs=%d sequential=%.3f ms block=%.3f ms speedup=%.2fx "
      "sequential_residual=%.3e block_residual=%.3e\n",
      dim, static_cast<long long>(cache.kkt.nonZeros()), repeats, rhs_count,
      cache.solver->backend_name(), cache.symbolic_analyses,
      cache.numeric_factorizations, cache.linear_solves,
      static_cast<double>(matrix_bytes) / (1024.0 * 1024.0),
      median(factor_ms), median(solve_ms), relative_residual, rhs_count,
      median_sequential_ms, median_block_ms,
      median_block_ms > 0.0 ? median_sequential_ms / median_block_ms : 0.0,
      sequential_residual, block_residual);
  return relative_residual <= 1e-9 && sequential_residual <= 1e-9 &&
                 block_residual <= 1e-9 && cache.symbolic_analyses == 1
             ? 0
             : 1;
}
