#pragma once

/// Executable solver abstraction for OPFProblem.  This deliberately adapts the
/// production solve_ac_opf/solve_dc_opf entry points instead of creating a
/// second formulation path.

#include <stdexcept>
#include <variant>

#include "hacdcpf/optimal_power_flow/opf_problem.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"

namespace hacdcpf {
opf::ACOPFResult solve_ac_opf(const HybridPowerSystem& sys,
                              const opf::ACOPFOptions& opt);
opf::DCOPFResult solve_dc_opf(const HybridPowerSystem& sys,
                              const opf::DCOPFOptions& opt);
}  // namespace hacdcpf

namespace hacdcpf::opf {

using OPFSolveResult = std::variant<ACOPFResult, DCOPFResult>;

class IOPFSolver {
 public:
  virtual ~IOPFSolver() = default;
  virtual OPFSolveResult solve(const OPFProblem& problem) const = 0;
};

class DefaultOPFSolver final : public IOPFSolver {
 public:
  OPFSolveResult solve(const OPFProblem& problem) const override {
    if (problem.sys == nullptr)
      throw std::invalid_argument("OPFProblem::sys must not be null");
    if (problem.is_ac == problem.is_dc)
      throw std::invalid_argument(
          "OPFProblem must select exactly one of is_ac or is_dc");
    if (problem.is_ac)
      return ::hacdcpf::solve_ac_opf(*problem.sys, problem.ac_opts);
    return ::hacdcpf::solve_dc_opf(*problem.sys, problem.dc_opts);
  }
};

}  // namespace hacdcpf::opf
