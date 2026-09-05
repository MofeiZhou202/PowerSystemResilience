#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <string>

namespace hacdcpf::market::detail {
struct SouthernSolverStatus {
  std::string quality;
  bool limit_reached;
  bool optimality_proven;
};

inline SouthernSolverStatus southern_solver_status(const std::string& status, bool feasible, double gap) {
  // Adapter labels, not substring guesses: see southern_execution_contract.md.
  std::istringstream stream(status);
  std::string token; stream >> token;
  if (token == "StrictHiGHS" || token == "HiGHS") stream >> token;
  std::transform(token.begin(), token.end(), token.begin(), [](unsigned char c) { return std::tolower(c); });
  token.erase(std::remove(token.begin(), token.end(), '_'), token.end());
  const bool limit = token == "timelimit" || token == "iterationlimit" || token == "solutionlimit" ||
    token == "memorylimit" || token == "interrupt" || token == "highsinterrupt" || token == "nodelimit" || token == "worklimit";
  if (feasible) {
    const bool optimal = token == "optimal";
    return {optimal ? "optimal_within_tolerance" : limit ? "feasible_limit" : "feasible_unproven",
      limit, optimal && std::isfinite(gap) && gap <= 1e-9};
  }
  return {token == "infeasible" ? "proven_infeasible" : token == "unbounded" ? "unbounded" :
    token == "unboundedorinfeasible" ? "infeasible_or_unbounded" : limit ? "limit_without_verified_solution" : "solver_or_audit_failure",
    limit, false};
}
} // namespace hacdcpf::market::detail
