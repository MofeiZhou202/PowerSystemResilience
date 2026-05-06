#pragma once

#include <string>

#include "hacdcpf/model/system.hpp"

namespace hacdcpf::io {

HybridPowerSystem parse_matpower(const std::string& filepath);

}  // namespace hacdcpf::io
