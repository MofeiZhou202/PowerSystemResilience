// Bridge header: re-exports hacdcpf SolverData into mipsolvers::powerflow.
#pragma once

#include "hacdcpf/power_flow/solver_data.hpp"

namespace mipsolvers::powerflow {

using hacdcpf::powerflow::SolverData;

}  // namespace mipsolvers::powerflow
