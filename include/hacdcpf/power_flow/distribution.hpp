#pragma once

// Re-export the distribution BFS power flow under the powerflow namespace.
// No additional .cpp needed — this is a header-only re-export.

#include "hacdcpf/power_flow/distribution_power_flow.hpp"

namespace hacdcpf::powerflow {

using hacdcpf::analysis::DPFOptions;
using hacdcpf::analysis::DPFResult;
using hacdcpf::analysis::solve_distribution_pf;

}  // namespace hacdcpf::powerflow
