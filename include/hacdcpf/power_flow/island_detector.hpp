#pragma once

#include <vector>

#include "hacdcpf/model/system.hpp"

namespace hacdcpf::powerflow {

std::vector<IslandInfo> detect_islands(const HybridPowerSystem& sys);

HybridPowerSystem extract_island_subsystem(const HybridPowerSystem& sys,
                                           const IslandInfo& island,
                                           int slack_bus_override = 0);

}  // namespace hacdcpf::powerflow