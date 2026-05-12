#pragma once
// Backward compatibility — canonical location is now hacdcpf/power_flow/
#include "hacdcpf/power_flow/converter_model.hpp"
namespace hacdcpf::core { using hacdcpf::powerflow::converter_loss; using hacdcpf::powerflow::converter_loss_jacobian; using hacdcpf::powerflow::converter_ac_injection; using hacdcpf::powerflow::converter_dc_injection; }
