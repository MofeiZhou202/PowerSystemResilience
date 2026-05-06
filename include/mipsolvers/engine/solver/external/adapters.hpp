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
  SolveResult solve_milp(const MIPModel& prob) const override;

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

  bool available() const;
  const std::string& executable() const;

 private:
  std::string executable_;
};

class ScipAdapter final : public SolverAdapter {
 public:
  explicit ScipAdapter(std::string executable = "");

  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  SolveResult solve_minlp(const MINLPModel& prob) const override;

  bool available() const;
  const std::string& executable() const;

 private:
  std::string executable_;
};

/// Gurobi adapter using the native C API (requires libgurobi linked at build time).
class GurobiAdapter final : public SolverAdapter {
 public:
  GurobiAdapter();
  ~GurobiAdapter() override;

  GurobiAdapter(const GurobiAdapter&) = delete;
  GurobiAdapter& operator=(const GurobiAdapter&) = delete;

  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  SolveResult solve_lp(const LPModel& prob) const override;
  SolveResult solve_qp(const QPModel& prob) const override;
  SolveResult solve_milp(const MIPModel& prob) const override;

  bool available() const;

 private:
  void* env_{nullptr};  // GRBenv* (opaque to avoid header dependency)
};

}  // namespace mipsolvers::engine
