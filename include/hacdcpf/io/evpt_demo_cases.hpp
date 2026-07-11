#pragma once

// io/evpt_demo_cases.hpp
// ──────────────────────────────────────────────────────────────────────────
// Built-in EV power-traffic demo scenarios for the GUI and examples.
//
//   evpt_demo_small — 2-bus grid (cheap baseload + expensive peaker behind a
//     0.5 MW tie) coupled to a 4-node diamond road network with two route
//     alternatives and a V2G-capable station.  Runs interactively fast and
//     produces an LMP spread under Formulation C.
//
//   evpt_demo_grid — 5×5 one-way road grid (east/south links) coupled to the
//     IEEE 33-bus AC/DC feeder (case33bw_acdc).  Three charging stations map
//     onto feeder buses; a costlier local DG plus a tightened feeder-head
//     rating make EV charging visibly congest the feeder and split LMPs.

#include <string>
#include <vector>

#include "hacdcpf/ev_power_traffic/simulation.hpp"

namespace hacdcpf::io {

std::vector<std::string> evpt_demo_case_names();

/// Build a demo scenario by name; throws std::runtime_error for unknown names.
evpt::EVPowerTrafficProblem build_evpt_demo_case(const std::string& name);

evpt::EVPowerTrafficProblem build_evpt_demo_small();
evpt::EVPowerTrafficProblem build_evpt_demo_grid();

}  // namespace hacdcpf::io
