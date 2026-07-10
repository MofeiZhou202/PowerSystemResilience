#pragma once

/// sppt/benchmark.hpp
/// ==================
/// Pillar-4 self-contained scale/accuracy benchmark (docs/latex/sppt_theory.tex).
/// For each case it times the SPPT pipeline stages (projection, canonical
/// assembly, power-flow solve) and reports the MR3 commuting residual, giving a
/// reproducible scale-vs-accuracy profile of the SPPT-enabled platform without
/// requiring any external tool.

#include <string>
#include <utility>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::sppt {

struct BenchmarkRow {
  std::string case_name;
  int  n_ac_bus{0};
  int  n_ac_branch{0};
  bool converged{false};
  int  iterations{0};

  double project_ms{0.0};    ///< time for project_to_canonical_models (best of N)
  double assemble_ms{0.0};   ///< time for make_solver_data (best of N)
  double solve_ms{0.0};      ///< time for solve_power_flow (best of N)

  double commuting_residual{0.0};  ///< MR3 semantic-preservation residual (accuracy)
  std::string note;
};

struct Benchmark {
  std::vector<BenchmarkRow> rows;

  [[nodiscard]] std::string to_csv() const;
  [[nodiscard]] std::string to_latex() const;
};

/// Benchmark one already-loaded system; \p repeats controls best-of-N timing.
BenchmarkRow run_benchmark_case(const HybridPowerSystem& sys, std::string case_name,
                                int repeats = 3);

/// Benchmark a corpus given (display-name, file-path) pairs.
Benchmark run_benchmark(const std::vector<std::pair<std::string, std::string>>& cases,
                        int repeats = 3);

}  // namespace hacdcpf::sppt
