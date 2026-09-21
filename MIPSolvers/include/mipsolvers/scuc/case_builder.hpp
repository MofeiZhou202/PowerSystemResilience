/// case_builder.hpp — Synthetic SCUC test case generators
///
/// Provides pre-built SCUCInput objects for unit tests and benchmarking.
/// All cases include buses, branches, generators, loads, and profiles.

#pragma once

#include <string>

#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/scuc/scuc.hpp"

namespace mipsolvers::scuc {

/// 3-bus, 2-generator base case — tiny problem for fast unit tests.
/// T periods of length dt hours. No renewable or storage units.
SCUCInput build_3bus_case(int T = 3, double dt = 1.0);

/// 6-bus standard IEEE test case — 3 generators, 6 buses, 7 branches.
/// Optional renewable (wind) and storage.
SCUCInput build_6bus_case(int T = 24, double dt = 1.0,
                          bool with_wind    = false,
                          bool with_storage = false);

/// IEEE 39-bus New England test case — 10 generators, 39 buses, 46 branches.
/// Full 24-hour profile with load shape. Optional renewable.
SCUCInput build_ieee39_case(int T = 24, double dt = 1.0,
                            bool with_wind  = false,
                            bool with_solar = false);

/// IEEE 118-bus test case — 54 generators, 186 branches, 99 load buses.
/// Standard MATPOWER case118 topology; ~4242 MW peak aggregate load.
/// Optional wind (4 sites) and solar (3 sites).
SCUCInput build_ieee118_case(int T = 24, double dt = 1.0,
                             bool with_wind  = false,
                             bool with_solar = false);

/// Serialize SCUCInput to JSON string (for export / golden-file testing).
/// Pass indent >= 0 for pretty-printing, -1 for compact.
std::string scuc_input_to_json(const SCUCInput& inp, int indent = 2);

/// Build the raw SCUC MIP model from a SCUCInput without solving it.
/// Used by the benchmark runner to call solve_milp_bc() directly with
/// custom BCOptions (e.g., to inject warm starts or tune solver settings).
mipsolvers::engine::MIPModel build_scuc_mip(const SCUCInput& inp);

}  // namespace mipsolvers::scuc
