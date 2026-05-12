#pragma once

#include <vector>

#include "hacdcpf/model/results.hpp"
#include "hacdcpf/power_flow/solver_data.hpp"

namespace hacdcpf::powerflow {

std::vector<BranchFlow> compute_branch_flows(const SolverData& data,
                                              const std::vector<double>& vm,
                                              const std::vector<double>& va);

}  // namespace hacdcpf::powerflow
