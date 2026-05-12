#pragma once
// Backward compatibility — canonical location is now hacdcpf/power_flow/
#include "hacdcpf/power_flow/jacobian_builder.hpp"
namespace hacdcpf::core { using hacdcpf::powerflow::JacobianContext; using hacdcpf::powerflow::JacobianPattern; using hacdcpf::powerflow::build_jacobian_pattern; using hacdcpf::powerflow::evaluate_residual_and_jacobian; using hacdcpf::powerflow::evaluate_residual_only; }
