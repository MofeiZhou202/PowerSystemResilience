#pragma once

// This header previously re-exported DPFOptions, DPFResult, and
// solve_distribution_pf from the hacdcpf::analysis namespace.  Those types
// and that function no longer exist — they were removed when the distribution
// power flow API was redesigned.
//
// Use hacdcpf/power_flow/distribution_power_flow.hpp directly and refer to
//   hacdcpf::analysis::ThreePhaseNROptions
//   hacdcpf::analysis::ThreePhaseDPFResult
//   hacdcpf::analysis::solve_three_phase_nr
// instead.

#error "hacdcpf/power_flow/distribution.hpp: DPFOptions / DPFResult / " \
       "solve_distribution_pf no longer exist. " \
       "Include hacdcpf/power_flow/distribution_power_flow.hpp and use " \
       "ThreePhaseNROptions / ThreePhaseDPFResult / solve_three_phase_nr."
