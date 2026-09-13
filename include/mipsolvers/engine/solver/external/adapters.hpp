#pragma once

#include <optional>
#include <string>

#include "mipsolvers/engine/solver/solver_adapter.hpp"

namespace mipsolvers::engine {

class HighsAdapter final : public SolverAdapter {
 public:
  explicit HighsAdapter(std::string executable = "");

  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  SolveResult solve_lp(const LPModel& prob) const override;
  SolveResult solve_lp(const LPModel& prob,
                       const SolveContext& context) const override;
  SolveResult solve_pricing_lp(const LPModel& prob, double time_limit_sec) const;
  SolveResult solve_milp(const MIPModel& prob) const override;
  SolveResult solve_milp(const MIPModel& prob,
                         const SolveContext& context) const override;

  bool available() const;
  const std::string& executable() const;

 private:
  std::string executable_;
};

class IpoptAdapter final : public SolverAdapter {
 public:
  explicit IpoptAdapter(std::string executable = "");

  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  SolveResult solve_nlp(const NLPModel& prob) const override;
  SolveResult solve_nlp(const NLPModel& prob,
                        const SolveContext& context) const override;

  bool available() const;
  const std::string& executable() const;

 private:
  SolveResult solve_nlp_impl(const NLPModel& prob,
                             double time_limit_sec) const;
  std::string executable_;
};

class ScipAdapter final : public SolverAdapter {
 public:
  explicit ScipAdapter(std::string executable = "");

  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  /// Solve a pure (linear) MILP.  SCIP natively reads the MPS export produced by
  /// write_lp_as_mps(), so the linear part is handed to SCIP directly without a
  /// symbolic-objective detour.  This makes SCIP a first-class MILP backend
  /// (e.g. unit commitment) alongside HiGHS and the native branch-and-cut.
  SolveResult solve_milp(const MIPModel& prob) const override;
  SolveResult solve_minlp(const MINLPModel& prob) const override;

  bool available() const;
  const std::string& executable() const;

 private:
  std::string executable_;
};

/// CPLEX adapter using the native Callable Library C API.
/// Defaults and validation gates: docs/cplex_callable_library.md,
/// "Cost model and fixed validation".
struct CplexOptions {
  double time_limit_sec{3600.0};
  double mip_gap{1e-9};
  int threads{0};
  int random_seed{0};
};

// Last solve_milp on the calling thread. Timing intervals bracket the CPLEX
// import, optimize and extraction calls; null means that phase was not reached.
struct CplexSolveInfo {
  std::optional<double> model_import_sec;
  std::optional<double> optimize_sec;
  std::optional<double> result_extract_sec;
  std::optional<double> best_bound;
  std::optional<long long> node_count;
  int status{0};
  bool has_solution{false};
  bool proven{false};
  bool optimal{false};
  bool timed_out{false};
};
CplexSolveInfo last_cplex_solve_info();

class CplexAdapter final : public SolverAdapter {
 public:
  CplexAdapter();
  explicit CplexAdapter(CplexOptions options);
  ~CplexAdapter() override;

  CplexAdapter(const CplexAdapter&) = delete;
  CplexAdapter& operator=(const CplexAdapter&) = delete;

  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  SolveResult solve_milp(const MIPModel& prob) const override;
  SolveResult solve_milp(const MIPModel& prob,
                         const SolveContext& context) const override;
  bool available() const;

 private:
  std::optional<CplexOptions> options_;
  std::string initialization_error_;
#ifdef HACDCPF_HAVE_CPLEX
  void* env_{nullptr};  // CPXENVptr (opaque to avoid a public header dependency)
#endif
};

/// Gurobi adapter using the native C API (requires libgurobi linked at build time).
struct GurobiOptions {
  double time_limit_sec{3600};
  double mip_gap{1e-9};
  int threads{0};
  int method{-1};
  int crossover{-1};
};

// Last solve_milp (including configured solve_lp) on the calling thread.
// Null means the phase was not reached. See docs/solvers.md, Gurobi timing.
struct GurobiSolveTiming {
  std::optional<double> model_import_sec;
  std::optional<double> optimize_sec;
  std::optional<double> result_extract_sec;
};
GurobiSolveTiming last_gurobi_solve_timing();

class GurobiAdapter final : public SolverAdapter {
 public:
  GurobiAdapter();
  explicit GurobiAdapter(GurobiOptions options);
  ~GurobiAdapter() override;

  GurobiAdapter(const GurobiAdapter&) = delete;
  GurobiAdapter& operator=(const GurobiAdapter&) = delete;

  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  SolveResult solve_lp(const LPModel& prob) const override;
  SolveResult solve_pricing_lp(const LPModel& prob);
  SolveResult solve_relaxation_lp(const LPModel& prob, double relative_tolerance);
  SolveResult solve_qp(const QPModel& prob) const override;
  SolveResult solve_milp(const MIPModel& prob) const override;

  bool available() const;

 private:
  std::optional<GurobiOptions> options_;
#ifdef HACDCPF_HAVE_GUROBI
  void* env_{nullptr};  // GRBenv* (opaque to avoid header dependency)
#endif
};

}  // namespace mipsolvers::engine
