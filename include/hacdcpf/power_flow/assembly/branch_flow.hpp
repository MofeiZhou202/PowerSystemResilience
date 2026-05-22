#pragma once

#include <vector>

#include "hacdcpf/power_flow/power_flow_result.hpp"
#include "hacdcpf/assembly/solver_data.hpp"

namespace hacdcpf::powerflow {

std::vector<BranchFlow> compute_branch_flows(const SolverData& data,
                                              const std::vector<double>& vm,
                                              const std::vector<double>& va);

}  // namespace hacdcpf::powerflow
