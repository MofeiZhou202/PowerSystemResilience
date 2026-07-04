#pragma once

#include "hacdcpf/integrated_energy/integrated_energy_options.hpp"
#include "hacdcpf/integrated_energy/integrated_energy_result.hpp"
#include "hacdcpf/integrated_energy/integrated_energy_system.hpp"

namespace hacdcpf::integrated_energy {

CampusIESResult solve_campus_ies(const CampusIESData& data,
                                 const CampusIESOptions& options = {});

}  // namespace hacdcpf::integrated_energy
