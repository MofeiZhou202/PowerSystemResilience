#pragma once
// Backward compatibility — canonical location is now hacdcpf/power_flow/
#include "hacdcpf/power_flow/solver_data.hpp"
namespace hacdcpf::core { using hacdcpf::powerflow::SolverData; using hacdcpf::powerflow::make_solver_data; using hacdcpf::powerflow::aggregate_generation; using hacdcpf::powerflow::rebuild_matrices; }
