#pragma once
#include <string>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::io {

/// Parse a MATPOWER .m case file and return a HybridPowerSystem.
HybridPowerSystem parse_matpower(const std::string& filepath);

}  // namespace hacdcpf::io
