#pragma once
// Backward compatibility — canonical location is now hacdcpf/power_flow/
#include "hacdcpf/power_flow/residual_evaluator.hpp"
namespace hacdcpf::core { using hacdcpf::powerflow::ResidualBlocks; using hacdcpf::powerflow::evaluate_power_flow_residual; }
