#pragma once
// Backward compatibility — canonical location is now hacdcpf/power_flow/
#include "hacdcpf/power_flow/admittance_builder.hpp"
namespace hacdcpf::core { using hacdcpf::powerflow::build_admittance_matrix; using hacdcpf::powerflow::build_dc_conductance; }
